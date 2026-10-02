//
// AppController - implementation: loading, the four division paths, joints,
// planner figures and the written report, plus the status text the UI shows.
//

#include "AppController.h"

#include "CurvedBsp.h"
#include "CutInD.h"
#include "DbgAnalysis.h"
#include "HarmonicFit.h"
#include "IritGuard.h"

#include "MaterialField.h"
#include "CadLoader.h"
#include "PlannerGraph.h"
#include "PlannerFigure.h"
#include "AssemblyOrder.h"
#include "DivisionReport.h"
#include "JointRotation.h"
#include <QDebug>
#include <QRandomGenerator>
#include <algorithm>
#include <cmath>
#include <QElapsedTimer>
#include <cstring>

#include <QFile>
#include <QHash>
#include <QFileInfo>
#include <QDir>
#include <QLocale>
#include <QtConcurrent/QtConcurrentRun>

// LAST, and it must stay last: IritSolid.h pulls in irit_sm.h, which #defines
// _mkdir, and above <QDir> that makes QDir::_mkdir a redeclaration (C2535).
// Qt headers first, IRIT headers after - same rule as CadLoader.cpp.
#include "IritSolid.h"

AppController::AppController(QObject *parent)
    : QObject(parent)
{
    connect(&m_fitWatcher, &QFutureWatcher<QVector<HarmonicFit::Result>>::finished,
            this, &AppController::finishFit);
    connect(&m_lockWatcher, &QFutureWatcher<QStringList>::finished,
            this, &AppController::finishInterlocking);
    connect(&m_splitWatcher, &QFutureWatcher<SplitRun>::finished,
            this, &AppController::finishSplit);
}

QStringList AppController::nameFilters() const
{
    return CadLoader::nameFilters();
}

QStringList AppController::primitiveKinds() const
{
    return Trivariate::primitiveKinds();
}

void AppController::loadFile(const QUrl &url)
{
    loadPath(url.isLocalFile() ? url.toLocalFile() : url.toString());
}

void AppController::loadPath(const QString &path)
{
    if (path.isEmpty()) {
        setError(QStringLiteral("No file selected."));
        return;
    }

    m_fileName   = QFileInfo(path).fileName();
    m_loadedPath = path;
    m_vrepCells.clear();
    m_vrepBlockOf.clear();
    m_vrepPieceOf.clear();
    m_vrepCuts.clear();
    m_vrepCutBlock.clear();
    m_vrepPieceCuts.clear();
    m_curvedCuts = false;
    m_wholeCutUsed = false;
    m_parts.clear();
    m_splitAlts.clear();
    m_splitNotes.clear();
    m_splitRank.clear();
    m_splitAlt = 0;
    m_blocks.clear();
    m_blockNames.clear();
    m_blockVol.clear();
    m_blockPart.clear();
    m_blockFits.clear();

    QElapsedTimer timer;
    timer.start();

    MeshData mesh;
    QString  error;
    if (!CadLoader::load(path, &mesh, &error)) {
        m_mesh = MeshData();
        m_sourceMesh = MeshData();
        m_pieces.clear();
        m_planFigures.clear();
        m_planFolderUrl.clear();
        m_reportUrl.clear();
        m_triv = Trivariate();
        emit meshChanged();
        emit piecesChanged();
        emit trivariateChanged();
        setError(error);
        return;
    }

    const qint64 ms = timer.elapsed();
    m_mesh       = mesh;
    m_sourceMesh = mesh;
    m_pieces.clear();
    m_planFigures.clear();
    m_planFolderUrl.clear();
    m_reportUrl.clear();
    m_triv = Trivariate();
    m_trivInfo.clear();
    m_divisionInfo.clear();

    const QLocale loc;
    QString d = QStringLiteral("%1 · %2 vertices · %3 triangles · %4 edges")
                    .arg(m_mesh.sourceKind,
                         loc.toString(m_mesh.vertexCount()),
                         loc.toString(m_mesh.triangleCount()),
                         loc.toString(m_mesh.edgeCount()));
    if (m_mesh.objectCount > 1)
        d += QStringLiteral(" · %1 objects").arg(m_mesh.objectCount);
    d += QStringLiteral(" · %1 ms").arg(ms);

    d += QStringLiteral("\nExtent  %1 × %2 × %3")
             .arg(m_mesh.bmax[0] - m_mesh.bmin[0], 0, 'g', 4)
             .arg(m_mesh.bmax[1] - m_mesh.bmin[1], 0, 'g', 4)
             .arg(m_mesh.bmax[2] - m_mesh.bmin[2], 0, 'g', 4);

    m_status   = m_fileName;
    m_detail   = d;
    m_hasError = false;
    m_trivReal = false;

    emit meshChanged();
    emit piecesChanged();
    emit trivariateChanged();
    emit statusChanged();

    // An IRIT file holding a trivariate (Elber's tvs_*.itd) is used AS the
    // trivariate, so Divide cuts its parameter domain D. Meshes (OBJ, STL)
    // have no trivariate and keep the old path.
    const QString ext = QFileInfo(path).suffix().toLower();
    if (ext == QLatin1String("itd") || ext == QLatin1String("ibd") || ext == QLatin1String("imd")) {
        QString terr;
        Trivariate tv = Trivariate::fromFile(path, &terr);
        if (tv.isValid()) {
            adoptTrivariate(std::move(tv), QStringLiteral("trivariate from %1").arg(m_fileName));
            m_trivReal = true;
            m_detail = m_trivInfo + QStringLiteral("\nRandom divides this trivariate's domain D (BSP in D).");
            emit statusChanged();
        }
    }
}

void AppController::adoptTrivariate(Trivariate tv, const QString &sourceDesc)
{
    if (!tv.isValid())
        return;

    m_triv = std::move(tv);
    m_pieces.clear();
    m_vrepCells.clear();
    m_vrepBlockOf.clear();
    m_vrepPieceOf.clear();
    m_vrepCuts.clear();
    m_vrepCutBlock.clear();
    m_vrepPieceCuts.clear();
    m_curvedCuts = false;
    m_wholeCutUsed = false;
    m_blocks.clear();
    m_blockNames.clear();
    m_blockVol.clear();
    m_blockPart.clear();
    m_blockFits.clear();
    m_planFigures.clear();
    m_planFolderUrl.clear();
    m_reportUrl.clear();

    double dom[6];
    int    ord[3];
    m_triv.domain(dom);
    m_triv.orders(ord);

    m_trivInfo = QStringLiteral(
                     "%1 · domain u[%2, %3] v[%4, %5] w[%6, %7] · orders %8/%9/%10")
                     .arg(sourceDesc)
                     .arg(dom[0], 0, 'g', 3).arg(dom[1], 0, 'g', 3)
                     .arg(dom[2], 0, 'g', 3).arg(dom[3], 0, 'g', 3)
                     .arg(dom[4], 0, 'g', 3).arg(dom[5], 0, 'g', 3)
                     .arg(ord[0]).arg(ord[1]).arg(ord[2]);
    m_divisionInfo.clear();

    MeshData whole;
    QString  err;
    if (m_triv.tessellate(&whole, fineNessFor(kModelFineNess), &err) && !whole.isEmpty()) {
        m_mesh = whole;
        emit meshChanged();
    }

    m_status   = m_triv.label();
    m_detail   = m_trivInfo;
    m_hasError = false;

    emit trivariateChanged();
    emit piecesChanged();
    emit statusChanged();
}

void AppController::useTrivariateFromFile()
{
    if (m_loadedPath.isEmpty()) {
        setError(QStringLiteral("Load a file first."));
        return;
    }
    QString err;
    Trivariate tv = Trivariate::fromFile(m_loadedPath, &err);
    if (!tv.isValid()) {
        setError(err);
        return;
    }
    adoptTrivariate(std::move(tv), QStringLiteral("from %1").arg(m_fileName));
    m_trivReal = true;
}

void AppController::usePrimitive(const QString &kind)
{
    QString err;
    Trivariate tv = Trivariate::primitive(kind, &err);
    if (!tv.isValid()) {
        setError(err);
        return;
    }
    adoptTrivariate(std::move(tv), QStringLiteral("primitive"));
    m_trivReal = false;
}

void AppController::useBoundingCage()
{
    if (m_mesh.isEmpty()) {
        setError(QStringLiteral("Load a model first - the cage is built around it."));
        return;
    }
    QString err;
    Trivariate tv = Trivariate::boundingCage(m_mesh, &err);
    if (!tv.isValid()) {
        setError(err);
        return;
    }
    adoptTrivariate(std::move(tv),
                    QStringLiteral("box over %1's extent - shape comes with "
                                   "the Increment 3 fit").arg(m_fileName));
    m_trivReal = false;
}

double AppController::fineNessFor(double base, double perControlPoint) const
{
    int len[3];
    m_triv.lengths(len);
    return std::max(base, perControlPoint * std::max({ len[0], len[1], len[2] }));
}

void AppController::fitTrivariate(int detail)
{
    if (m_fitting || m_splitting)
        return;
    if (m_sourceMesh.isEmpty() || m_sourceMesh.triangleCount() == 0) {
        setError(QStringLiteral("Load a closed triangle mesh (OBJ, STL) first."));
        return;
    }
    m_fitDetail = detail;
    // Split first - always (a branching model is never fitted as one tube).
    // The split runs on a worker; the fit starts when it is done.
    if (m_parts.isEmpty()) {
        m_fitAfterSplit = true;
        splitLimbs();
        return;
    }
    m_fitting = true;
    m_fitPath = m_loadedPath;
    emit fittingChanged();

    m_status   = QStringLiteral("Fitting a trivariate to %1 …").arg(m_fileName);
    m_detail   = QStringLiteral("Harmonic volumetric parameterization (Martin, Cohen & Kirby 2009): "
                                "surface harmonic, voxel harmonics, skeleton, w-paths, B-spline fit.");
    m_hasError = false;
    emit statusChanged();

    HarmonicFit::Options opt;
    if (detail > 0) {
        opt.nu = 48;
        opt.nv = 64;
        opt.nw = 6;
        opt.voxels = 128;
        opt.capPerimeter = 0.02;
    }
    // One trivariate per part, each starting (u = 0) on its largest cap, and
    // each made to enclose its part: pieces are trimmed by the part (Elber
    // section 5), so their outside is exactly the model.
    QVector<MeshData> meshes;
    QVector<HarmonicFit::Options> opts;
    // Enclosing fits + boolean trim (Elber section 5) are OFF: measured on
    // spot, the blocks never fully enclosed their parts (body: 101 vertices
    // out, 30 after local pushes, which then folded it) and 3-5 pieces failed
    // the boolean - the pieces held 85% of the volume, then less. Kept in the
    // code (Options::inflate, pushToEnclose) for the next attempt.
    opt.inflate = m_encloseTrim ? 0.03 : 0.0;
    m_fitParts = !m_parts.isEmpty();
    if (m_fitParts) {
        for (const HarmonicFit::Part &P : m_parts) {
            HarmonicFit::Options o = opt;
            const HarmonicFit::Cap *big = nullptr;
            for (const auto &c : P.caps) if (!big || c.area > big->area) big = &c;
            if (big) {
                o.capStart = true;
                for (int k = 0; k < 3; ++k) o.capCentre[k] = big->centre[k];
            }
            meshes.append(P.mesh);
            opts.append(o);
        }
        m_detail = QStringLiteral("Fitting %1 trivariates, one per part of the limb split …").arg(m_parts.size());
        emit statusChanged();
    } else {
        meshes.append(m_sourceMesh);
        opts.append(opt);
    }
    const bool enclose = m_fitParts;
    m_fitWatcher.setFuture(QtConcurrent::run([meshes, opts, enclose]() {
        QVector<HarmonicFit::Result> out;
        for (int i = 0; i < meshes.size(); ++i) {
            if (!enclose) { out.append(HarmonicFit::fit(meshes[i], opts[i])); continue; }
            // The enclosing trivariate of a part (for the trimmed V-rep): fitted
            // pole to pole to the part's envelope - the part thickened by 5% and
            // smoothed - then checked against the part itself; a thicker
            // envelope if any of its vertices is still outside (three tries).
            HarmonicFit::Options o = opts[i];
            o.capStart = false;
            o.closedEnds = true;
            o.inflate = 0.0;
            double off = 0.05;
            HarmonicFit::Result best;
            int bestOut = INT32_MAX;
            for (int t = 0; t < 3; ++t, off *= 1.6) {
                QString err;
                const MeshData env = HarmonicFit::envelope(meshes[i], off, 48, &err);
                if (env.triangleCount() == 0) { if (!best.ok) best.error = err; break; }
                HarmonicFit::Result r = HarmonicFit::fit(env, o);
                if (!r.ok) { if (!best.ok) best = r; continue; }
                double worst = 0.0;
                const int outN = HarmonicFit::outsideCount(r, meshes[i], &worst);
                r.notes << QStringLiteral("envelope %1%: %2 vertex(es) of the part outside the trivariate (worst %3%)")
                               .arg(100.0 * off, 0, 'f', 1).arg(outN).arg(100.0 * worst, 0, 'f', 2);
                if (outN < bestOut) { bestOut = outN; best = r; }
                if (outN == 0) break;
            }
            out.append(best);
        }
        return out;
    }));
}

void AppController::splitLimbs()
{
    if (m_splitting || m_fitting)
        return;
    if (m_sourceMesh.isEmpty() || m_sourceMesh.triangleCount() == 0) {
        setError(QStringLiteral("Load a closed triangle mesh (OBJ, STL) first."));
        m_fitAfterSplit = false;
        return;
    }
    m_splitting = true;
    emit fittingChanged();
    m_status = QStringLiteral("Splitting %1 …").arg(m_fileName);
    m_detail = QStringLiteral("Simulated annealing over the limb cuts, then a quick trial fit of every part of the "
                              "best few splits (folds, how much each envelope had to grow) - the healthiest wins.");
    m_hasError = false;
    emit statusChanged();
    const MeshData mesh = m_sourceMesh;
    m_splitWatcher.setFuture(QtConcurrent::run([mesh]() {
        SplitRun run;
        QElapsedTimer tm;
        tm.start();
        HarmonicFit::SplitOptions so;
        for (int pick = 0; pick < std::max(1, so.shortlist); ++pick) {
            so.pick = pick;
            QVector<HarmonicFit::Part> parts;
            QStringList notes;
            QString err;
            if (!HarmonicFit::splitLimbs(mesh, so, &parts, &notes, &err)) {
                if (pick == 0) run.error = err;
                break;
            }
            run.alts.append(parts);
            run.notes.append(notes);
            // A model with no limbs has nothing to choose between.
            if (parts.size() <= 1) { run.health.append(HarmonicFit::SplitHealth()); break; }
            run.health.append(HarmonicFit::splitHealth(parts));
        }
        run.ms = tm.elapsed();
        return run;
    }));
}

void AppController::finishSplit()
{
    m_splitting = false;
    emit fittingChanged();
    const SplitRun run = m_splitWatcher.result();
    if (run.alts.isEmpty()) {
        m_fitAfterSplit = false;
        setError(QStringLiteral("Limb split failed: %1").arg(run.error));
        return;
    }
    // Healthiest first (the search's order breaks ties).
    QVector<int> order(run.alts.size());
    for (int i = 0; i < order.size(); ++i) order[i] = i;
    std::stable_sort(order.begin(), order.end(), [&](int x, int y) { return run.health[x].score < run.health[y].score; });
    m_splitAlts.clear();
    m_splitNotes.clear();
    m_splitRank.clear();
    for (int r = 0; r < order.size(); ++r) {
        const int i = order[r];
        const HarmonicFit::SplitHealth &h = run.health[i];
        m_splitAlts.append(run.alts[i]);
        m_splitNotes.append(run.notes[i]);
        m_splitRank << (h.ok ? QStringLiteral("split %1 (search rank %2): %3 parts, trial fit: %4 folded part(s), %5 envelope "
                                              "thickening(s), %6 vertex(es) outside · score %7 · %8 s")
                                   .arg(r + 1).arg(i + 1).arg(run.alts[i].size()).arg(h.folded).arg(h.retries)
                                   .arg(h.outside).arg(h.score, 0, 'g', 4).arg(h.ms / 1000.0, 0, 'f', 1)
                             : QStringLiteral("split %1 (search rank %2): %3 part(s), no trial fit")
                                   .arg(r + 1).arg(i + 1).arg(run.alts[i].size()));
        for (const QString &l : h.lines) qInfo().noquote() << QStringLiteral("SPLIT  split %1 trial fit: ").arg(r + 1) + l;
    }
    for (const QString &l : m_splitRank) qInfo().noquote() << QStringLiteral("SPLIT  ") + l;
    m_splitAlt = 0;
    adoptSplit(m_splitAlts[0], m_splitNotes[0],
               QStringLiteral("%1 split(s) tried in %2 s, the healthiest kept:\n%3")
                   .arg(m_splitAlts.size()).arg(run.ms / 1000.0, 0, 'f', 1).arg(m_splitRank.join(QStringLiteral("\n"))));
    if (m_fitAfterSplit) {
        m_fitAfterSplit = false;
        fitTrivariate(m_fitDetail);
    }
}

void AppController::adoptSplit(const QVector<HarmonicFit::Part> &parts, const QStringList &notes, const QString &rank)
{
    for (const QString &n : notes)
        qInfo().noquote() << QStringLiteral("SPLIT  ") + n;
    m_parts = parts;
    m_blocks.clear();
    m_blockNames.clear();
    m_blockVol.clear();
    m_blockPart.clear();
    m_blockFits.clear();

    m_pieces.clear();
    m_vrepCells.clear();
    m_vrepBlockOf.clear();
    m_vrepPieceOf.clear();
    m_vrepCuts.clear();
    m_vrepCutBlock.clear();
    m_vrepPieceCuts.clear();
    m_curvedCuts = false;
    m_wholeCutUsed = false;
    m_planFigures.clear();
    m_planFolderUrl.clear();
    m_reportUrl.clear();
    m_jointNote.clear();
    for (int i = 0; i < parts.size(); ++i) {
        PuzzlePiece pc;
        pc.i = i;
        pc.mesh = parts[i].mesh;
        for (int a = 0; a < 3; ++a) {
            pc.centre[a] = 0.5f * (pc.mesh.bmin[a] + pc.mesh.bmax[a]);
            pc.size[a]   = pc.mesh.bmax[a] - pc.mesh.bmin[a];
        }
        m_pieces.append(std::move(pc));
    }

    QStringList lines;
    for (const HarmonicFit::Part &P : parts)
        lines << QStringLiteral("%1 %2%").arg(P.name).arg(100.0 * P.areaFrac, 0, 'f', 1);
    // The search's own lines (what it chose, and why) go on the page too.
    QStringList search;
    for (const QString &n : notes)
        if (n.startsWith(QStringLiteral("split search")) || n.startsWith(QStringLiteral("  cut ")))
            search << n.trimmed();
    m_divisionInfo = QStringLiteral("Morse split of %1: %2 part(s) (%3)\n%4%5"
                                    "Each cut is capped with the same surface on both sides, so the parts "
                                    "meet exactly. Fit trivariate now fits one trivariate per part.")
                         .arg(m_fileName).arg(parts.size()).arg(lines.join(QStringLiteral(", ")))
                         .arg(rank.isEmpty() ? QString() : rank + QStringLiteral("\n"))
                         .arg(search.isEmpty() ? QString() : search.join(QStringLiteral("\n")) + QStringLiteral("\n"));
    m_status   = QStringLiteral("%1 — %2 parts (Morse split)").arg(m_fileName).arg(parts.size());
    m_detail   = m_divisionInfo;
    m_hasError = false;
    emit piecesChanged();
    emit statusChanged();
}

void AppController::finishFit()
{
    m_fitting = false;
    emit fittingChanged();
    const QVector<HarmonicFit::Result> all = m_fitWatcher.result();
    if (m_fitPath != m_loadedPath || all.isEmpty())
        return;                                   // another model was loaded meanwhile
    if (m_fitParts) {
        finishPartsFit(all);
        return;
    }
    const HarmonicFit::Result r = all.front();
    for (const QString &n : r.notes)
        qInfo().noquote() << QStringLiteral("FIT  ") + n;
    if (!r.ok) {
        setError(QStringLiteral("Trivariate fit failed: %1").arg(r.error));
        return;
    }

    const QFileInfo fi(m_loadedPath);
    QString err;
    Trivariate tv = HarmonicFit::toTrivariate(r, QStringLiteral("fit of %1").arg(fi.completeBaseName()), &err);
    if (!tv.isValid()) {
        setError(err);
        return;
    }
    const HarmonicFit::Check ck = HarmonicFit::checkJacobian(tv);
    const HarmonicFit::Deviation dev = HarmonicFit::surfaceDeviation(tv, m_sourceMesh);

    const QString out = fi.absolutePath() + QStringLiteral("/tv_") + fi.completeBaseName() +
                        QStringLiteral(".itd");
    QString saveErr;
    const bool saved = tv.saveToFile(out, &saveErr);

    adoptTrivariate(std::move(tv), QStringLiteral("harmonic fit of %1").arg(m_fileName));
    m_trivReal = true;

    QString d = m_trivInfo;
    d += QStringLiteral("\nControl mesh %1×%2×%3 · fit error %4% of the model size · "
                        "det J > 0 at %5 of %6 samples (min/mean %7) · volume %8% of the mesh")
             .arg(r.nu).arg(r.nv).arg(r.nw)
             .arg(100.0 * r.fitError, 0, 'f', 2)
             .arg(ck.samples - ck.nonPositive).arg(ck.samples)
             .arg(ck.minRatio, 0, 'f', 3)
             .arg(r.meshVolume > 0 ? 100.0 * std::fabs(ck.volume) / r.meshVolume : 0.0, 0, 'f', 1);
    d += QStringLiteral("\nDistance model → trivariate (of the model size): mean %1% · 95% of vertices "
                        "within %2% · worst %3%")
             .arg(100.0 * dev.mean, 0, 'f', 2).arg(100.0 * dev.p95, 0, 'f', 2).arg(100.0 * dev.max, 0, 'f', 2);
    for (const QString &n : r.notes)
        if (n.contains(QStringLiteral("branch")) || n.contains(QStringLiteral("genus")) ||
            n.contains(QStringLiteral("w-paths")) || n.contains(QStringLiteral("caps")))
            d += QStringLiteral("\n") + n;
    d += saved ? QStringLiteral("\nSaved as %1 · Random now divides D (BSP in D).").arg(QFileInfo(out).fileName())
               : QStringLiteral("\nNot saved: %1").arg(saveErr);
    m_detail = d;
    emit statusChanged();
}

// One trivariate per part of the limb split: a multi-block V-rep. A part
// whose fit fails is reported and skipped; the rest still form the model.
void AppController::finishPartsFit(const QVector<HarmonicFit::Result> &fitted)
{
    const QFileInfo fi(m_loadedPath);
    // Limbs snapped onto the part they were cut from, so the blocks meet.
    // The blocks enclose their parts (fitted to envelopes); the pieces are
    // trimmed by the parts, so no snapping.
    QVector<HarmonicFit::Result> all = fitted;
    QStringList joins;
    m_blocksEnclose = true;
    for (const QString &n : joins) qInfo().noquote() << QStringLiteral("FIT  ") + n;
    std::vector<Trivariate> blocks;
    QStringList names, lines;
    QVector<double> vols;
    QVector<int> blockPart;
    QVector<HarmonicFit::Result> blockFits;
    int failed = 0, foldedBlocks = 0;
    for (int i = 0; i < all.size() && i < m_parts.size(); ++i) {
        const HarmonicFit::Result &r = all[i];
        const QString name = m_parts[i].name;
        for (const QString &n : r.notes)
            qInfo().noquote() << QStringLiteral("FIT  %1: ").arg(name) + n;
        if (!r.ok) {
            qWarning().noquote() << QStringLiteral("FIT  %1 skipped: %2").arg(name, r.error);
            lines << QStringLiteral("%1: fit failed (%2) - skipped").arg(name, r.error);
            ++failed;
            continue;
        }
        QString err;
        Trivariate tv = HarmonicFit::toTrivariate(r, name, &err);
        if (!tv.isValid()) {
            lines << QStringLiteral("%1: %2 - skipped").arg(name, err);
            ++failed;
            continue;
        }
        const HarmonicFit::Check ck = HarmonicFit::checkJacobian(tv);
        double worst = 0.0;
        const int outN = HarmonicFit::outsideCount(r, m_parts[i].mesh, &worst);
        // More than 1% of the samples folded: the block turns inside out in
        // places and its pieces are not valid solids there - say so.
        const bool folded = ck.samples > 0 && ck.nonPositive > ck.samples / 100;
        foldedBlocks += folded;
        lines << QStringLiteral("%1%5: folds (det J < 0) at %2 of %3 samples · encloses its part%4")
                     .arg(name).arg(ck.nonPositive).arg(ck.samples)
                     .arg(outN == 0 ? QString() : QStringLiteral(" except %1 vertex(es), worst %2% out").arg(outN).arg(100.0 * worst, 0, 'f', 2))
                     .arg(folded ? QStringLiteral(" - FOLDED, not usable as is") : QString());
        vols.append(std::fabs(ck.volume));
        names << name;
        blockPart.append(i);
        blockFits.append(r);
        blocks.push_back(std::move(tv));
    }
    // Backtracking: a part that failed or folded at full resolution sends the
    // fit to the next split on the shortlist.
    if ((failed > 0 || foldedBlocks > 0) && m_splitAlt + 1 < m_splitAlts.size()) {
        qWarning().noquote() << QStringLiteral("FIT  split %1: %2 part(s) failed, %3 folded - trying split %4")
                                    .arg(m_splitAlt + 1).arg(failed).arg(foldedBlocks).arg(m_splitAlt + 2);
        ++m_splitAlt;
        adoptSplit(m_splitAlts[m_splitAlt], m_splitNotes[m_splitAlt],
                   QStringLiteral("split %1 failed or folded at full resolution; now split %2:\n%3")
                       .arg(m_splitAlt).arg(m_splitAlt + 1).arg(m_splitRank.join(QStringLiteral("\n"))));
        fitTrivariate(m_fitDetail);
        return;
    }
    if (blocks.empty()) {
        setError(QStringLiteral("No part could be fitted: %1").arg(lines.join(QStringLiteral("; "))));
        return;
    }

    QVector<const Trivariate *> ptrs;
    for (const Trivariate &t : blocks) ptrs.append(&t);
    const QString out = fi.absolutePath() + QStringLiteral("/tv_") + fi.completeBaseName() + QStringLiteral("_parts.itd");
    QString saveErr;
    const bool saved = Trivariate::saveAll(ptrs, names, out, &saveErr);

    int biggest = 0;
    for (int b = 1; b < vols.size(); ++b) if (vols[b] > vols[biggest]) biggest = b;
    adoptTrivariate(blocks[biggest].clone(),
                    QStringLiteral("%1 trivariates fitted to the parts of %2").arg(blocks.size()).arg(m_fileName));
    m_blocks = std::move(blocks);
    m_blockNames = names;
    m_blockVol = vols;
    m_blockPart = blockPart;
    m_blockFits = blockFits;
    m_trivReal = true;

    // Show every block, not only the copy in m_triv.
    {
        MeshData merged;
        for (const Trivariate &t : m_blocks) {
            MeshData md;
            QString err;
            if (!t.tessellate(&md, fineNessFor(kModelFineNess), &err)) continue;
            const uint32_t base = uint32_t(merged.pos.size() / 3);
            merged.pos += md.pos;
            for (uint32_t x : md.tris) merged.tris.append(x + base);
            for (uint32_t x : md.edges) merged.edges.append(x + base);
        }
        if (!merged.isEmpty()) {
            merged.computeBounds();
            merged.computeNormals();
            m_mesh = merged;
            emit meshChanged();
        }
    }


    QString d = QStringLiteral("Split %4 of %5 · %1 part(s) fitted, %2 failed · each part has a trivariate that encloses it (fitted "
                               "to the part thickened and smoothed); pieces are cut in its D and trimmed by the "
                               "part - the exact shape of %3, as trimmed V-reps")
                    .arg(m_blocks.size()).arg(failed).arg(m_fileName)
                    .arg(m_splitAlt + 1).arg(std::max<qsizetype>(1, m_splitAlts.size()));
    d += QStringLiteral("\n") + lines.join(QStringLiteral("\n"));
    d += saved ? QStringLiteral("\nSaved as %1 · Random now divides every block in its own D.").arg(QFileInfo(out).fileName())
               : QStringLiteral("\nNot saved: %1").arg(saveErr);
    m_status = QStringLiteral("%1 — %2 trivariate block(s)").arg(m_fileName).arg(m_blocks.size());
    m_detail = d;
    m_hasError = false;
    emit statusChanged();
}

void AppController::showWholeModel()
{
    m_pieces.clear();
    m_vrepCells.clear();
    m_vrepBlockOf.clear();
    m_vrepPieceOf.clear();
    m_vrepCuts.clear();
    m_vrepCutBlock.clear();
    m_vrepPieceCuts.clear();
    m_curvedCuts = false;
    m_wholeCutUsed = false;
    m_planFigures.clear();
    m_planFolderUrl.clear();
    m_reportUrl.clear();
    m_divisionInfo.clear();
    emit piecesChanged();
    emit meshChanged();
    emit statusChanged();
}

void AppController::savePieces(const QUrl &url, bool separateFiles,
                               bool spread)
{
    if (m_pieces.isEmpty()) {
        setError(QStringLiteral("Divide the model before saving."));
        return;
    }

    QString path = url.isLocalFile() ? url.toLocalFile() : url.toString();
    if (path.isEmpty()) {
        setError(QStringLiteral("No file name given."));
        return;
    }

    if (!PieceExport::canWrite(path))
        path += QStringLiteral(".itd");

    PieceExport::Result r;
    QString err;
    if (!PieceExport::save(m_pieces, path, separateFiles, spread, &r, &err)) {
        setError(err);
        return;
    }

    for (const QString &line : r.problems)
        qDebug().noquote() << QStringLiteral("SAVE  ") + line;

    // BSP in D pieces are sub-trivariates of M: write them as such too, so the
    // result stays a V-rep. With enclosing blocks (after Fit trivariate) each
    // piece is a TRIMMED V-rep: its cell(s) of its part's trivariate, trimmed
    // by its triangles on the model or on a joint cap - the exact shape.
    QString vrepNote;
    if (!m_vrepCells.isEmpty() && m_triv.isValid() &&
        QFileInfo(path).suffix().compare(QStringLiteral("itd"), Qt::CaseInsensitive) == 0) {
        std::vector<Trivariate> subs;
        subs.reserve(m_vrepCells.size());
        QVector<int> subPiece;
        int failed = 0;
        for (int c = 0; c < m_vrepCells.size(); ++c) {
            const CellBox &cb = m_vrepCells[c];
            const Trivariate &src = (c < m_vrepBlockOf.size() && m_vrepBlockOf[c] < int(m_blocks.size()))
                                        ? m_blocks[size_t(m_vrepBlockOf[c])] : m_triv;
            Trivariate t = src.subRegion(cb.lo[0], cb.hi[0], cb.lo[1], cb.hi[1], cb.lo[2], cb.hi[2]);
            if (!t.isValid()) {
                qWarning().noquote() << QStringLiteral("SAVE  cell %1: sub-trivariate failed (%2) - skipped")
                                            .arg(c).arg(IritGuard::lastError());
                ++failed;
                continue;
            }
            subPiece.append(c < m_vrepPieceOf.size() ? m_vrepPieceOf[c] : c);
            subs.push_back(std::move(t));
        }
        // The trimming surface: the model's triangles plus every joint cap.
        MeshData surface = m_sourceMesh;
        if (m_blocksEnclose) {
            const MeshData caps = jointCapTriangles();
            for (int t = 0; t < caps.triangleCount(); ++t)
                for (int k = 0; k < 3; ++k) {
                    const uint32_t v = caps.tris[3 * t + k];
                    surface.tris.push_back(surface.addVertex(caps.pos[3 * v], caps.pos[3 * v + 1], caps.pos[3 * v + 2]));
                }
            surface.computeBounds();
        }
        const QFileInfo fi(path);
        const QString vpath = fi.absolutePath() + QStringLiteral("/") + fi.completeBaseName() + QStringLiteral("_vrep.itd");
        QString verr;
        bool ok = false;
        int trimTotal = 0;
        if (m_blocksEnclose && !m_vrepPieceOf.isEmpty()) {
            QVector<Trivariate::TrimmedPiece> tps(m_pieces.size());
            for (int pi = 0; pi < m_pieces.size(); ++pi) {
                tps[pi].name = QStringLiteral("piece_%1").arg(pi);
                const MeshData &m = m_pieces[pi].mesh;
                // Its cut faces are its exact cuts (below); the trimming
                // triangles are the model's skin and the joint caps only. A
                // whole-model cut runs past its own part into the extended
                // map, which is not exact: there every triangle trims too.
                const QVector<char> trim = m_wholeCutUsed ? QVector<char>(m.triangleCount(), 1)
                                                          : CutInD::trimTriangles(m, surface);
                for (int t = 0; t < m.triangleCount(); ++t) {
                    if (!trim[t]) continue;
                    for (int k = 0; k < 3; ++k)
                        for (int a = 0; a < 3; ++a) tps[pi].trim.append(m.pos[3 * m.tris[3 * t + k] + a]);
                    ++trimTotal;
                }
            }
            for (size_t k = 0; k < subs.size(); ++k)
                if (subPiece[int(k)] >= 0 && subPiece[int(k)] < tps.size()) tps[subPiece[int(k)]].cells.append(&subs[k]);
            // The cuts, exact: their surfaces in D, composed through their block.
            QVector<Trivariate::ExactCut> exact;
            for (int k = 0; k < m_vrepCuts.size(); ++k) {
                Trivariate::ExactCut ec;
                const int b = k < m_vrepCutBlock.size() ? m_vrepCutBlock[k] : -1;
                ec.block = (b >= 0 && b < int(m_blocks.size())) ? &m_blocks[size_t(b)] : nullptr;
                ec.name = QStringLiteral("%1 %2").arg(b >= 0 && b < m_blockNames.size() ? m_blockNames[b] : QStringLiteral("block"),
                                                      m_vrepCuts[k].kind);
                ec.surfaces = m_vrepCuts[k].surfaces;
                exact.append(ec);
            }
            for (int pi = 0; pi < tps.size() && pi < m_vrepPieceCuts.size(); ++pi) tps[pi].cuts = m_vrepPieceCuts[pi];
            QStringList cutNotes;
            QElapsedTimer ct;
            ct.start();
            ok = Trivariate::saveTrimmed(tps, exact, vpath, &verr, &cutNotes);
            for (const QString &n : cutNotes) qWarning().noquote() << QStringLiteral("SAVE  ") + n;
            if (ok)
                vrepNote = QStringLiteral(" · %1 trimmed V-rep piece(s) in %2 (%3 exact cut(s)%6, %4 trimming triangles, %7 ms)%5")
                               .arg(tps.size()).arg(QFileInfo(vpath).fileName()).arg(exact.size()).arg(trimTotal)
                               .arg(failed ? QStringLiteral(", %1 cell(s) failed").arg(failed) : QString())
                               .arg(cutNotes.isEmpty() ? QString() : QStringLiteral(", %1 not composed").arg(cutNotes.size()))
                               .arg(ct.elapsed());
        } else {
            QVector<const Trivariate *> ptrs;
            QStringList names;
            for (size_t k = 0; k < subs.size(); ++k) {
                ptrs.append(&subs[k]);
                names << QStringLiteral("piece_%1").arg(subPiece[int(k)]);
            }
            ok = !ptrs.isEmpty() && Trivariate::saveAll(ptrs, names, vpath, &verr);
            if (ok)
                vrepNote = QStringLiteral(" · %1 V-rep trivariate(s) in %2%3").arg(ptrs.size()).arg(QFileInfo(vpath).fileName())
                               .arg(failed ? QStringLiteral(" (%1 failed)").arg(failed) : QString());
        }
        if (!ok) vrepNote = QStringLiteral(" · V-rep pieces not written: %1").arg(verr);
    }

    m_status = QStringLiteral("Saved %1 piece(s) to %2")
                   .arg(r.written).arg(QFileInfo(path).fileName());
    m_detail = QStringLiteral("%1 · %2 piece(s) written%3%4")
                   .arg(r.format)
                   .arg(r.written)
                   .arg(r.skipped > 0
                            ? QStringLiteral(", %1 skipped").arg(r.skipped)
                            : QString())
                   .arg(QStringLiteral("%1%2")
                            .arg(r.separateFiles
                                     ? QStringLiteral(" · one file per piece")
                                     : QString())
                            .arg(r.spread
                                     ? QStringLiteral(" · spread out for slicing")
                                     : QString()));
    m_detail += vrepNote;
    m_hasError = false;
    emit statusChanged();
}

// The V-rep path: divides the bounding cage, then trims every piece back to the
// model (Elber section 5).
void AppController::runDivision(const DivisionSpec &spec)
{
    QElapsedTimer timer;
    timer.start();

    QVector<PuzzlePiece> pieces;
    QString warning;
    m_stats.subject = QStringLiteral("V-rep (bounding cage)");
    m_stats.cells   = spec.cellCount();
    QElapsedTimer stageTimer;
    stageTimer.start();
    if (!PuzzleDivider::divide(m_triv, spec, kPieceFineNess, &pieces, &warning)) {
        setError(warning);
        return;
    }

    m_stats.msExtract = stageTimer.elapsed();
    m_pieces = std::move(pieces);

    // Before the trim: the cut itself gets the dovetail profile, so the joint
    // ends up inside the model instead of stuck onto it.
    cutCellDovetails();

    trimPiecesToModel();

    planAndDrawFigures();

    double dom[6];
    m_triv.domain(dom);
    logDivision(QStringLiteral("V-rep cuts in parameter space · ") + spec.note,
                dom, spec.splits);
    logPieceSizes();

    applyJoints();
    m_stats.msTotal = timer.elapsed();
    describePieces(QStringLiteral("V-rep · ") + spec.note, spec.cellCount(), warning);
    m_divisionInfo += QStringLiteral(" · %1 ms").arg(m_stats.msTotal);

    if (!m_figureNote.isEmpty())
        m_divisionInfo += QStringLiteral("\n") + m_figureNote;

    m_status   = QStringLiteral("%1 — %2 pieces").arg(m_triv.label()).arg(m_pieces.size());
    m_detail   = m_divisionInfo;
    m_hasError = false;

    emit piecesChanged();
    emit statusChanged();
}

MeshData AppController::workingMesh() const
{
    return m_warp.active() ? MeshDivider::warp(m_mesh, m_warp) : m_mesh;
}

// The mesh path: clips the model itself against world-space cells, through the
// warp when curved cuts are asked for.
void AppController::runMeshDivision(const MeshDivisionSpec &spec, const MeshData &work)
{
    if (m_mesh.isEmpty()) {
        setError(QStringLiteral("Load a model first."));
        return;
    }

    QElapsedTimer timer;
    timer.start();

    const double detail = m_warp.active()
        ? double(m_mesh.diagonal()) / 32.0
        : 0.0;

    QVector<PuzzlePiece> pieces;
    QString warning;
    m_stats.subject = QStringLiteral("mesh (clipped)");
    m_stats.cells   = spec.cellCount();
    QElapsedTimer stageTimer;
    stageTimer.start();
    if (!MeshDivider::divide(work, spec, &pieces, &warning, detail)) {
        setError(warning);
        return;
    }

    MeshDivider::unwarp(&pieces, m_warp, m_mesh.bmin, m_mesh.bmax,
                        double(m_mesh.diagonal()));

    m_stats.msExtract = stageTimer.elapsed();
    m_pieces = std::move(pieces);
    planAndDrawFigures();

    const double dom[6] = { work.bmin[0], work.bmax[0],
                            work.bmin[1], work.bmax[1],
                            work.bmin[2], work.bmax[2] };
    logDivision(QStringLiteral("mesh cuts in world space · ") + spec.note,
                dom, spec.planes);
    logPieceSizes();

    applyJoints();
    m_stats.msTotal = timer.elapsed();
    describePieces(QStringLiteral("mesh · ") + spec.note +
                   QStringLiteral(" · ") + m_warp.describe(),
                   spec.cellCount(), warning);
    m_divisionInfo += QStringLiteral(" · %1 ms").arg(m_stats.msTotal);

    if (!m_figureNote.isEmpty())
        m_divisionInfo += QStringLiteral("\n") + m_figureNote;

    m_status   = QStringLiteral("%1 — %2 pieces").arg(m_fileName).arg(m_pieces.size());
    m_detail   = m_divisionInfo;
    m_hasError = false;

    emit piecesChanged();
    emit statusChanged();
}

// The trim, plus the guarantee that a piece is never handed back as its
// un-trimmed cage box.
//
// A dovetailed cut can leave IRIT unable to finish a later trim, and it does so
// INTERMITTENTLY: measured on cheburashka at 5 pieces, the same binary, model
// and seed cut the same 3 of 6 faces every run with no cut failure, yet one run
// in three still failed to trim a piece. Nothing at the cut stage separates the
// good runs from the bad, so the check lives here instead - if the trim failed
// and the cuts carried dovetails, restore the flat cells and trim again.
void AppController::trimPiecesToModel()
{
    if (m_sourceMesh.isEmpty())
        return;

    QElapsedTimer stageTimer;
    stageTimer.start();
    CageBoolean::Result br =
        CageBoolean::intersectAll(&m_pieces, m_sourceMesh, kPieceFineNess);

    if (br.failed > 0 && m_cellsDovetailed && !m_preJointPieces.isEmpty()) {
        qWarning().noquote()
            << QStringLiteral("TRIM failed on %1 piece(s) after dovetailed cuts"
                              " - restoring the flat cells and trimming again")
                   .arg(br.failed);

        // Kept only if the retry does better, so dovetails are never discarded
        // for nothing.
        const QVector<PuzzlePiece> dovetailed = m_pieces;

        m_pieces = m_preJointPieces;
        const CageBoolean::Result flat =
            CageBoolean::intersectAll(&m_pieces, m_sourceMesh, kPieceFineNess);

        if (flat.failed < br.failed) {
            m_cellJointNote =
                QStringLiteral("dovetails abandoned after the trim - every cut "
                               "succeeded, but IRIT then failed to trim %1 "
                               "piece(s) back to the model, which would have "
                               "left them as raw boxes; the pieces were "
                               "re-divided with flat cuts instead")
                    .arg(br.failed);
            qWarning().noquote() << m_cellJointNote;
            br = flat;
            m_cellsDovetailed = false;
        } else {
            // Flat cuts did no better, so the dovetails are not what broke the
            // trim. Put them back and let the report say the piece failed.
            m_pieces = dovetailed;
        }
    }

    m_stats.msTrim = stageTimer.elapsed();
    for (const QString &line : CageBoolean::describe(br))
        qDebug().noquote() << line;
    // An orphan is what a "floating fragment" in the viewport actually is: a
    // cell that clipped the model into disconnected lumps, where the spare
    // could not be welded into any neighbour. Measured identical with joints
    // off and on, so it is the division doing this, not the joints.
    m_booleanNote = QStringLiteral("%1 trimmed to the model, %2 empty "
                                   "cell(s) dropped, %3 failed%4")
                        .arg(br.intersected).arg(br.dropped).arg(br.failed)
                        .arg(br.orphans > 0
                                 ? QStringLiteral(", %1 detached lump(s) kept "
                                                  "as separate piece(s) - a "
                                                  "cell clipped material not "
                                                  "connected to the rest of it")
                                       .arg(br.orphans)
                                 : QString());
}

// The V-rep path for an explicit cell list - what the BSP produces - followed by
// the same section 5 trim.
// BSP of the real trivariate's parameter domain D (CurvedBsp): the cells are
// born in D, each piece is meshed in D and every vertex mapped through M, so M
// alone shapes the cuts. The bend of the splits is not a user setting: it stays
// 0 (flat cuts in D) until it is chosen automatically, e.g. by simulated
// annealing on the blocking analysis. The analysis itself lives in the
// "Analyse puzzle" window.
void AppController::runBspInD(int pieces)
{
    QElapsedTimer timer;
    timer.start();

    pieces = qBound(2, pieces, 64);            // one sub-trivariate per piece

    if (m_blocks.size() > 1 && !m_parts.isEmpty()) {
        // Several blocks, one per part of the limb split. Each part gets pieces
        // by its share of the model's volume; a part whose share rounds to
        // none is glued to the part it was cut from (whole parts: at their
        // shared cap; onto a cut part: to the piece holding its cap) - so the
        // count asked for is the count made, and joints are cuts only where
        // pieces really meet. Parts with pieces are cut in their block's D
        // (CutInD): the outside of every piece is the model's own surface.
        const int P = m_parts.size();
        QVector<int> partBlock(P, -1);
        for (int bi = 0; bi < m_blockPart.size(); ++bi)
            if (m_blockPart[bi] >= 0 && m_blockPart[bi] < P) partBlock[m_blockPart[bi]] = bi;
        if (kWholeCut) {
            // The whole model, cut in the parts' trivariates: every model
            // vertex is given its part (by position), each part its fit.
            auto key = [](float x, float y, float z) {
                QByteArray k(12, Qt::Uninitialized);
                memcpy(k.data(), &x, 4); memcpy(k.data() + 4, &y, 4); memcpy(k.data() + 8, &z, 4);
                return k;
            };
            QHash<QByteArray, int> where;
            for (int p = 0; p < P; ++p) {
                const MeshData &pm = m_parts[p].mesh;
                for (int v = 0; v < pm.vertexCount(); ++v) {
                    const QByteArray k = key(pm.pos[3 * v], pm.pos[3 * v + 1], pm.pos[3 * v + 2]);
                    if (!where.contains(k)) where.insert(k, p);
                }
            }
            QVector<int> vpart(m_sourceMesh.vertexCount(), -1);
            for (int i = 0; i < m_sourceMesh.vertexCount(); ++i)
                vpart[i] = where.value(key(m_sourceMesh.pos[3 * i], m_sourceMesh.pos[3 * i + 1], m_sourceMesh.pos[3 * i + 2]), -1);
            QVector<HarmonicFit::Result> fits(P);
            for (int p = 0; p < P; ++p) if (partBlock[p] >= 0 && partBlock[p] < m_blockFits.size()) fits[p] = m_blockFits[partBlock[p]];
            const CutInD::Result cr = CutInD::cutWhole(m_sourceMesh, vpart, fits, pieces, m_layoutSeed, kCutBend, kCutWaves);
            for (const QString &nt : cr.notes) qInfo().noquote() << QStringLiteral("CUTD  ") + nt;
            if (!cr.ok || cr.pieces.isEmpty()) {
                setError(QStringLiteral("Cutting the whole model failed: %1").arg(cr.error));
                return;
            }
            QVector<PuzzlePiece> all;
            QVector<CellBox> allCells;
            QVector<int> blockOf, pieceOf;
            QVector<int> cutBlock;
            QVector<QVector<QPair<int, int>>> pieceCuts;
            int openEdges = 0;
            double piecesVol = 0.0, modelVol = std::fabs(IritSolid::signedVolume(m_sourceMesh));
            for (const CutInD::Piece &cp : cr.pieces) {
                PuzzlePiece pc;
                pc.i = all.size();
                pc.mesh = cp.mesh;
                for (int ax = 0; ax < 3; ++ax) {
                    pc.centre[ax] = 0.5f * (pc.mesh.bmin[ax] + pc.mesh.bmax[ax]);
                    pc.size[ax]   = pc.mesh.bmax[ax] - pc.mesh.bmin[ax];
                }
                openEdges += CutInD::openEdges(pc.mesh);
                piecesVol += std::fabs(IritSolid::signedVolume(pc.mesh));
                for (const CutInD::PartCell &c : cp.cells) {
                    if (c.part < 0 || c.part >= P || partBlock[c.part] < 0) continue;
                    CellBox cb;
                    for (int ax = 0; ax < 3; ++ax) { cb.lo[ax] = c.lo[ax]; cb.hi[ax] = c.hi[ax]; }
                    allCells.append(cb);
                    blockOf.append(partBlock[c.part]);
                    pieceOf.append(all.size());
                }
                pieceCuts.append(cp.cuts);
                all.append(std::move(pc));
            }
            for (const CutInD::Cut &ct : cr.cuts) cutBlock.append(ct.part >= 0 && ct.part < P ? partBlock[ct.part] : -1);
            m_pieces = std::move(all);
            m_vrepCells = allCells;
            m_vrepBlockOf = blockOf;
            m_vrepPieceOf = pieceOf;
            m_vrepCuts = cr.cuts;
            m_vrepCutBlock = cutBlock;
            m_vrepPieceCuts = pieceCuts;
            m_curvedCuts = kCutBend > 0.0;
            m_wholeCutUsed = true;
            m_planFigures.clear();
            m_planFolderUrl.clear();
            m_reportUrl.clear();
            m_jointNote.clear();
            m_divisionInfo = QStringLiteral("Whole model cut in D (curved: bend %1): %2 of %3 pieces · pieces cross the limb joints · "
                                            "volume %4% of the model · %5 open edge(s) · seed %6 · %7 ms")
                                 .arg(kCutBend, 0, 'f', 2).arg(m_pieces.size()).arg(pieces)
                                 .arg(modelVol > 0 ? 100.0 * piecesVol / modelVol : 0.0, 0, 'f', 3)
                                 .arg(openEdges).arg(m_layoutSeed).arg(timer.elapsed());
            for (const QString &nt : cr.notes)
                if (nt.startsWith(QStringLiteral("a piece could not"))) m_divisionInfo += QStringLiteral("\n") + nt;
            m_divisionInfo += QStringLiteral("\nSave as .itd to also get the pieces as trimmed trivariates (*_vrep.itd).");
            m_status   = QStringLiteral("%1 — %2 pieces").arg(m_fileName).arg(m_pieces.size());
            m_detail   = m_divisionInfo;
            m_hasError = false;
            emit piecesChanged();
            emit statusChanged();
            return;
        }
        auto sameCap = [](const HarmonicFit::Cap &x, const HarmonicFit::Cap &y) {
            return std::fabs(x.centre[0] - y.centre[0]) < 1e-5 && std::fabs(x.centre[1] - y.centre[1]) < 1e-5 &&
                   std::fabs(x.centre[2] - y.centre[2]) < 1e-5;
        };
        // The part tree: the body (part 0) at the root, parents across shared caps.
        QVector<int> parent(P, -1), depth(P, -1);
        QVector<HarmonicFit::Cap> joint(P);
        {
            QVector<int> queue{ 0 };
            depth[0] = 0;
            for (int qi = 0; qi < queue.size(); ++qi) {
                const int p = queue[qi];
                for (int q = 0; q < P; ++q) {
                    if (depth[q] >= 0) continue;
                    for (const auto &cq : m_parts[q].caps) {
                        bool shared = false;
                        for (const auto &cp : m_parts[p].caps) if (sameCap(cp, cq)) { shared = true; break; }
                        if (!shared) continue;
                        parent[q] = p;
                        joint[q] = cq;
                        depth[q] = depth[p] + 1;
                        queue.append(q);
                        break;
                    }
                }
            }
        }
        // Pieces per part, by volume.
        QVector<double> vol(P, 0.0);
        double total = 0.0;
        for (int p = 0; p < P; ++p) { vol[p] = std::fabs(IritSolid::signedVolume(m_parts[p].mesh)); total += vol[p]; }
        QVector<int> n(P, 0);
        int roots = 0;
        for (int p = 0; p < P; ++p) if (parent[p] < 0) ++roots;
        int given = 0;
        for (int p = 0; p < P; ++p)
            if (parent[p] >= 0) { n[p] = int(std::floor(pieces * vol[p] / std::max(1e-30, total) + 0.5)); given += n[p]; }
        while (given > pieces - roots) {                           // too many: the smallest give theirs up
            int sm = -1;
            for (int p = 0; p < P; ++p) if (parent[p] >= 0 && n[p] > 0 && (sm < 0 || vol[p] < vol[sm])) sm = p;
            if (sm < 0) break;
            --n[sm];
            --given;
        }
        {
            int left = std::max(roots, pieces - given);
            QVector<int> rootIdx;
            for (int p = 0; p < P; ++p) if (parent[p] < 0) { n[p] = 1; --left; rootIdx.append(p); }
            std::sort(rootIdx.begin(), rootIdx.end(), [&](int x, int y) { return vol[x] > vol[y]; });
            if (!rootIdx.isEmpty()) n[rootIdx.front()] += std::max(0, left);
        }

        // Leaves first: a part with no pieces joins its parent - whole to whole
        // at the cap, or waits to be attached after its parent is cut.
        QVector<MeshData> whole(P);
        QVector<QVector<int>> cellsOf(P);          // the parts whose blocks a (whole) part now holds
        for (int p = 0; p < P; ++p) { whole[p] = m_parts[p].mesh; cellsOf[p] = { p }; }
        QVector<QVector<int>> pending(P);
        QVector<int> order(P);
        for (int p = 0; p < P; ++p) order[p] = p;
        std::sort(order.begin(), order.end(), [&](int x, int y) { return depth[x] > depth[y]; });
        QStringList warnings;
        int glued = 0;
        for (int p : order) {
            if (n[p] > 0 || parent[p] < 0) continue;
            const int q = parent[p];
            if (n[q] == 0) {
                QString why;
                MeshData u = CutInD::unionAtCap(whole[q], whole[p], joint[p].centre, &why);
                if (u.isEmpty()) {
                    warnings << QStringLiteral("%1 could not be glued to %2 (%3) - kept as its own piece")
                                    .arg(m_parts[p].name, m_parts[q].name, why);
                    n[p] = 1;
                    continue;
                }
                whole[q] = u;
                if (qEnvironmentVariableIsSet("GLUE_DEBUG"))
                    qInfo().noquote() << QStringLiteral("GLUE  %1 + %2 (whole): %3 open edge(s)")
                                             .arg(m_parts[q].name, m_parts[p].name).arg(CutInD::openEdgesWelded(u));
                cellsOf[q] += cellsOf[p];
                ++glued;
            } else {
                pending[q].append(p);
            }
        }

        // Parts with pieces: cut in D (or whole, for one piece), then the
        // waiting parts attached.
        QVector<PuzzlePiece> all;
        QVector<CellBox> allCells;
        QVector<int> blockOf, pieceOf;
        QVector<CutInD::Cut> allCuts;
        QVector<int> cutBlock;
        QVector<QVector<QPair<int, int>>> pieceCuts;
        int openEdges = 0;
        double piecesVol = 0.0;
        for (int p = 0; p < P; ++p) {
            if (n[p] <= 0) continue;
            QVector<MeshData> meshes;
            QVector<QVector<QPair<int, CellBox>>> cells;          // per piece: (part, its cell)
            QVector<QVector<QPair<int, int>>> cutRefs;            // per piece: (cut in allCuts, side)
            const int bi = partBlock[p];
            if (n[p] > 1 && bi >= 0 && bi < m_blockFits.size()) {
                const CutInD::Result cr = CutInD::cut(whole[p], m_blockFits[bi], n[p], m_layoutSeed + quint32(p),
                                                      0.4, kCutBend, kCutWaves);
                for (const QString &nt : cr.notes)
                    qInfo().noquote() << QStringLiteral("CUTD  %1: ").arg(m_parts[p].name) + nt;
                if (cr.ok) {
                    const int base = allCuts.size();
                    for (const CutInD::Cut &ct : cr.cuts) { allCuts.append(ct); cutBlock.append(bi); }
                    for (const CutInD::Piece &cp : cr.pieces) {
                        meshes.append(cp.mesh);
                        CellBox cb;
                        for (int ax = 0; ax < 3; ++ax) { cb.lo[ax] = cp.lo[ax]; cb.hi[ax] = cp.hi[ax]; }
                        cells.append({ qMakePair(p, cb) });
                        if (cp.vWrap > 0.0) {                     // its cell goes on past v's seam
                            CellBox w = cb;
                            w.lo[1] = 0.0;
                            w.hi[1] = cp.vWrap;
                            cells.last().append(qMakePair(p, w));
                        }
                        QVector<QPair<int, int>> refs;
                        for (const auto &r : cp.cuts) refs.append(qMakePair(base + r.first, r.second));
                        cutRefs.append(refs);
                    }
                } else {
                    warnings << QStringLiteral("%1: cut in D failed (%2) - one piece").arg(m_parts[p].name, cr.error);
                }
            }
            if (meshes.isEmpty()) {
                meshes.append(whole[p]);
                cutRefs.append(QVector<QPair<int, int>>());
                QVector<QPair<int, CellBox>> c;
                for (int q : cellsOf[p]) { CellBox cb; for (int ax = 0; ax < 3; ++ax) { cb.lo[ax] = 0; cb.hi[ax] = 1; } c.append(qMakePair(q, cb)); }
                cells.append(c);
            }
            for (int c : pending[p]) {
                QString why;
                const int t = CutInD::attachAtCap(&meshes, whole[c], joint[c].centre, &why);
                if (qEnvironmentVariableIsSet("GLUE_DEBUG")) {
                    QStringList oe;
                    for (const MeshData &mm : meshes) oe << QString::number(CutInD::openEdgesWelded(mm));
                    qInfo().noquote() << QStringLiteral("GLUE  %1 onto %2 (piece %3): open edges per piece %4 (whole part had %5)")
                                             .arg(m_parts[c].name, m_parts[p].name).arg(t).arg(oe.join(QLatin1Char(' ')))
                                             .arg(CutInD::openEdgesWelded(whole[c]));
                }
                if (t < 0) {
                    warnings << QStringLiteral("%1 could not be glued to %2 (%3) - kept as its own piece")
                                    .arg(m_parts[c].name, m_parts[p].name, why);
                    meshes.append(whole[c]);
                    cutRefs.append(QVector<QPair<int, int>>());
                    QVector<QPair<int, CellBox>> cc;
                    for (int q : cellsOf[c]) { CellBox cb; for (int ax = 0; ax < 3; ++ax) { cb.lo[ax] = 0; cb.hi[ax] = 1; } cc.append(qMakePair(q, cb)); }
                    cells.append(cc);
                    continue;
                }
                for (int q : cellsOf[c]) { CellBox cb; for (int ax = 0; ax < 3; ++ax) { cb.lo[ax] = 0; cb.hi[ax] = 1; } cells[t].append(qMakePair(q, cb)); }
                ++glued;
            }
            for (int k = 0; k < meshes.size(); ++k) {
                PuzzlePiece pc;
                pc.i = all.size();
                pc.mesh = meshes[k];
                for (int ax = 0; ax < 3; ++ax) {
                    pc.centre[ax] = 0.5f * (pc.mesh.bmin[ax] + pc.mesh.bmax[ax]);
                    pc.size[ax]   = pc.mesh.bmax[ax] - pc.mesh.bmin[ax];
                }
                openEdges += CutInD::openEdges(pc.mesh);
                piecesVol += std::fabs(IritSolid::signedVolume(pc.mesh));
                for (const auto &pr : cells[k]) {
                    if (partBlock[pr.first] < 0) continue;
                    allCells.append(pr.second);
                    blockOf.append(partBlock[pr.first]);
                    pieceOf.append(all.size());
                }
                pieceCuts.append(k < cutRefs.size() ? cutRefs[k] : QVector<QPair<int, int>>());
                all.append(std::move(pc));
            }
        }
        if (all.isEmpty()) {
            setError(QStringLiteral("No pieces. %1").arg(warnings.join(QStringLiteral("; "))));
            return;
        }
        m_pieces = std::move(all);
        m_vrepCells = allCells;
        m_vrepBlockOf = blockOf;
        m_vrepPieceOf = pieceOf;
        m_vrepCuts = allCuts;
        m_vrepCutBlock = cutBlock;
        m_vrepPieceCuts = pieceCuts;
        m_curvedCuts = kCutBend > 0.0;
        m_planFigures.clear();
        m_planFolderUrl.clear();
        m_reportUrl.clear();
        m_jointNote.clear();
        QStringList plan;
        for (int p = 0; p < P; ++p) if (n[p] > 0) plan << QStringLiteral("%1 %2").arg(m_parts[p].name).arg(n[p]);
        m_divisionInfo = QStringLiteral("Cut in D (curved: bend %9) : %1 of %8 pieces (%2) · %3 part(s) glued on · volume %4% of the model · "
                                        "%5 open edge(s) · seed %6 · %7 ms")
                             .arg(m_pieces.size()).arg(plan.join(QStringLiteral(", "))).arg(glued)
                             .arg(total > 0 ? 100.0 * piecesVol / total : 0.0, 0, 'f', 3)
                             .arg(openEdges).arg(m_layoutSeed).arg(timer.elapsed()).arg(pieces)
                             .arg(kCutBend, 0, 'f', 2);
        if (m_pieces.size() < pieces)
            warnings << QStringLiteral("%1 cut(s) could not be made cleanly (the part's map into D folds there) - "
                                       "fewer pieces than asked").arg(pieces - m_pieces.size());
        if (openEdges > 0)
            warnings << QStringLiteral("%1 open edge(s): some joints or cut faces are not watertight yet").arg(openEdges);
        if (!warnings.isEmpty()) m_divisionInfo += QStringLiteral("\n") + warnings.join(QStringLiteral("\n"));
        m_divisionInfo += QStringLiteral("\nSave as .itd to also get the pieces as trivariates (*_vrep.itd).");
        m_status   = QStringLiteral("%1 — %2 pieces").arg(m_fileName).arg(m_pieces.size());
        m_detail   = m_divisionInfo;
        m_hasError = false;
        emit piecesChanged();
        emit statusChanged();
        return;
    }

    double dom[6];
    m_triv.domain(dom);

    // Flat cuts in D (the bend is 0 until it is chosen automatically, e.g. by
    // simulated annealing on the blocking analysis). A flat-cut BSP leaf is a
    // box of D, so each piece is the EXACT sub-trivariate M restricted to it -
    // a V-rep piece, curved in R^3 only by M. Same tree as CurvedBsp::build and
    // the analyzer's "BSP in D" (same seed, same jitter), so piece ids match.
    const QVector<CellBox> cells = PuzzleDivider::buildBspCells(dom, pieces, 0.25, m_layoutSeed);
    QVector<PuzzlePiece> out;
    QString warning;
    if (!PuzzleDivider::divideCells(m_triv, cells, fineNessFor(kPieceFineNess, 1.0), &out, &warning)) {
        setError(warning.isEmpty() ? QStringLiteral("The BSP in D produced no pieces.") : warning);
        return;
    }

    m_pieces = std::move(out);
    m_vrepCells = cells;
    m_planFigures.clear();
    m_planFolderUrl.clear();
    m_reportUrl.clear();
    m_jointNote.clear();

    m_divisionInfo = QStringLiteral("BSP in D of %1 · %2 pieces · seed %3 · each piece a sub-trivariate "
                                    "(V-rep) of M · flat cuts in D, curved in R³ by M · %4 ms")
                         .arg(m_triv.label()).arg(m_pieces.size()).arg(m_layoutSeed)
                         .arg(timer.elapsed());
    if (!warning.isEmpty())
        m_divisionInfo += QStringLiteral(" · ") + warning;
    m_divisionInfo += QStringLiteral("\nSave as .itd to also get the pieces as trivariates (*_vrep.itd). "
                                     "Blocking analysis: \"Analyse puzzle…\" (BSP in D).");

    m_status   = QStringLiteral("%1 — %2 pieces (BSP in D)").arg(m_fileName).arg(m_pieces.size());
    m_detail   = m_divisionInfo;
    m_hasError = false;

    emit piecesChanged();
    emit statusChanged();
}

void AppController::runTrivCellDivision(const QVector<CellBox> &cells,
                                        const QString &note)
{
    QElapsedTimer timer;
    timer.start();

    QVector<PuzzlePiece> pieces;
    QString warning;
    m_stats.subject = QStringLiteral("V-rep (bounding cage)");
    m_stats.cells   = cells.size();
    QElapsedTimer stageTimer;
    stageTimer.start();
    if (!PuzzleDivider::divideCells(m_triv, cells, kPieceFineNess,
                                    &pieces, &warning)) {
        setError(warning);
        return;
    }

    m_stats.msExtract = stageTimer.elapsed();
    m_pieces = std::move(pieces);

    // Before the trim: the cut itself gets the dovetail profile, so the joint
    // ends up inside the model instead of stuck onto it.
    cutCellDovetails();

    trimPiecesToModel();

    // TEMPORARY - Stage 0: the identical cells down both paths, for comparison.
    if (!m_sourceMesh.isEmpty() && !m_worldCells.isEmpty()) {
        const double modelVol = std::fabs(IritSolid::signedVolume(m_sourceMesh));

        double vrepVol = 0.0;
        for (const PuzzlePiece &pc : m_pieces)
            vrepVol += std::fabs(IritSolid::signedVolume(pc.mesh));

        // buildBspCells works over wdom = {0..ext}, i.e. a MODEL-LOCAL frame
        // with the origin at bmin, while MeshDivider clips in true world
        // coordinates. Shift them or every cell misses the model.
        QVector<CellBox> worldTrue;
        worldTrue.reserve(m_worldCells.size());
        for (const CellBox &w : m_worldCells) {
            CellBox t;
            for (int a = 0; a < 3; ++a) {
                t.lo[a] = w.lo[a] + double(m_sourceMesh.bmin[a]);
                t.hi[a] = w.hi[a] + double(m_sourceMesh.bmin[a]);
            }
            worldTrue.append(t);
        }

        QVector<PuzzlePiece> clipped;
        QString clipWarn;
        QElapsedTimer clipTimer;
        clipTimer.start();
        const bool clipOk = MeshDivider::divideCells(m_sourceMesh, worldTrue,
                                                     &clipped, &clipWarn, 0.0);
        const qint64 clipMs = clipTimer.elapsed();

        double clipVol = 0.0;
        for (const PuzzlePiece &pc : clipped)
            clipVol += std::fabs(IritSolid::signedVolume(pc.mesh));

        qInfo().noquote()
            << QStringLiteral("COMPARE cells=%1 | VREP pieces=%2 vol=%3 ms=%4 "
                              "| CLIP ok=%5 pieces=%6 vol=%7 ms=%8 | %9")
                   .arg(m_worldCells.size())
                   .arg(m_pieces.size())
                   .arg(modelVol > 0.0 ? vrepVol / modelVol : 0.0, 0, 'f', 4)
                   .arg(m_stats.msTrim)
                   .arg(clipOk ? QStringLiteral("y") : QStringLiteral("n"))
                   .arg(clipped.size())
                   .arg(modelVol > 0.0 ? clipVol / modelVol : 0.0, 0, 'f', 4)
                   .arg(clipMs)
                   .arg(clipWarn.isEmpty() ? QStringLiteral("no warnings") : clipWarn);
    }

    planAndDrawFigures();

    logCells(QStringLiteral("V-rep BSP cells in parameter space · ") + note, cells);
    logPieceSizes();

    applyJoints();
    m_stats.msTotal = timer.elapsed();
    describePieces(QStringLiteral("V-rep · ") + note, cells.size(), warning);
    m_divisionInfo += QStringLiteral(" · %1 ms").arg(m_stats.msTotal);

    if (!m_figureNote.isEmpty())
        m_divisionInfo += QStringLiteral("\n") + m_figureNote;

    m_status   = QStringLiteral("%1 — %2 pieces").arg(m_triv.label()).arg(m_pieces.size());
    m_detail   = m_divisionInfo;
    m_hasError = false;

    emit piecesChanged();
    emit statusChanged();
}

// The mesh path for an explicit cell list, again from the BSP.
void AppController::runCellDivision(const QVector<CellBox> &cells,
                                    const MeshData &work, const QString &note)
{
    if (m_mesh.isEmpty()) {
        setError(QStringLiteral("Load a model first."));
        return;
    }

    QElapsedTimer timer;
    timer.start();

    const double detail = m_warp.active() ? double(m_mesh.diagonal()) / 32.0 : 0.0;

    QVector<PuzzlePiece> pieces;
    QString warning;
    m_stats.subject = QStringLiteral("mesh (clipped)");
    m_stats.cells   = cells.size();
    QElapsedTimer stageTimer;
    stageTimer.start();
    if (!MeshDivider::divideCells(work, cells, &pieces, &warning, detail)) {
        setError(warning);
        return;
    }

    MeshDivider::unwarp(&pieces, m_warp, m_mesh.bmin, m_mesh.bmax,
                        double(m_mesh.diagonal()));

    m_stats.msExtract = stageTimer.elapsed();
    m_pieces = std::move(pieces);
    planAndDrawFigures();
    logCells(note, cells);
    logPieceSizes();

    applyJoints();
    m_stats.msTotal = timer.elapsed();
    describePieces(note, cells.size(), warning);
    m_divisionInfo += QStringLiteral(" · %1 ms").arg(m_stats.msTotal);

    if (!m_figureNote.isEmpty())
        m_divisionInfo += QStringLiteral("\n") + m_figureNote;

    m_status   = QStringLiteral("%1 — %2 pieces").arg(m_fileName).arg(m_pieces.size());
    m_detail   = m_divisionInfo;
    m_hasError = false;

    emit piecesChanged();
    emit statusChanged();
}

// Builds the on-screen summary and writes the report.
void AppController::describePieces(const QString &note, int gridCells,
                                   const QString &warning)
{
    m_stats.warning = warning;
    writeDivisionReport();

    float biggest = 0.0f, smallest = 1e30f;
    float maxSide[3] = { 0, 0, 0 };
    for (const PuzzlePiece &p : m_pieces) {
        biggest  = qMax(biggest,  p.largestSide());
        smallest = qMin(smallest, p.largestSide());
        for (int a = 0; a < 3; ++a)
            maxSide[a] = qMax(maxSide[a], p.size[a]);
    }
    if (m_pieces.isEmpty())
        smallest = 0.0f;

    const int shared = PuzzleDivider::adjacencyOfBoxes(m_pieces, 1e-6).size();

    m_divisionInfo = QStringLiteral("%1 · %2 of %3 cells filled · %4 shared faces\n"
                                    "Largest piece bbox %5 × %6 × %7 "
                                    "(biggest side %8, smallest %9)")
                         .arg(note)
                         .arg(m_pieces.size())
                         .arg(gridCells)
                         .arg(shared)
                         .arg(maxSide[0], 0, 'g', 4)
                         .arg(maxSide[1], 0, 'g', 4)
                         .arg(maxSide[2], 0, 'g', 4)
                         .arg(biggest, 0, 'g', 4)
                         .arg(smallest, 0, 'g', 4);
    if (!m_jointNote.isEmpty())
        m_divisionInfo += QStringLiteral("\n") + m_jointNote;
    if (!warning.isEmpty())
        m_divisionInfo += QStringLiteral("\n") + warning;
}

void AppController::divideUniform(int nu, int nv, int nw)
{
    m_stats = DivisionStats();
    m_stats.mode = QStringLiteral("Uniform");
    m_stats.seed = m_layoutSeed;

    const int counts[3] = { qBound(1, nu, 64), qBound(1, nv, 64), qBound(1, nw, 64) };

    m_vrepCells.clear();
    m_vrepBlockOf.clear();
    m_vrepPieceOf.clear();
    m_vrepCuts.clear();
    m_vrepCutBlock.clear();
    m_vrepPieceCuts.clear();
    m_curvedCuts = false;
    m_wholeCutUsed = false;
    m_lastKind      = LastDivision::Uniform;
    m_lastCounts[0] = counts[0];
    m_lastCounts[1] = counts[1];
    m_lastCounts[2] = counts[2];

    m_warp.enabled = false;
    if (m_triv.isValid()) {
        runDivision(PuzzleDivider::uniform(m_triv, counts));
        return;
    }
    const MeshData work = workingMesh();
    runMeshDivision(MeshDivider::uniform(work, counts), work);
}

void AppController::divideBySize(double maxSizeMM, int maxPerAxis)
{
    m_stats = DivisionStats();
    m_stats.mode = QStringLiteral("Max piece size");
    m_stats.seed = m_layoutSeed;

    if (!(maxSizeMM > 0.0)) {
        setError(QStringLiteral("Max piece size has to be greater than zero."));
        return;
    }
    const double budget[3] = { maxSizeMM, maxSizeMM, maxSizeMM };
    const int    cap       = qBound(1, maxPerAxis, 64);

    m_vrepCells.clear();
    m_vrepBlockOf.clear();
    m_vrepPieceOf.clear();
    m_vrepCuts.clear();
    m_vrepCutBlock.clear();
    m_vrepPieceCuts.clear();
    m_curvedCuts = false;
    m_wholeCutUsed = false;
    m_lastKind    = LastDivision::BySize;
    m_lastMaxSize = maxSizeMM;
    m_lastMaxAxis = cap;

    m_warp.enabled = false;

    if (m_triv.isValid()) {
        runDivision(PuzzleDivider::toBuildVolume(m_triv, budget, cap));
        return;
    }
    const MeshData work = workingMesh();
    runMeshDivision(MeshDivider::toBuildVolume(work, budget, cap), work);
}

void AppController::newLayout()
{
    m_layoutSeed = QRandomGenerator::global()->bounded(1, 100000);
    if (!m_mesh.isEmpty() && !m_pieces.isEmpty())
        divideRandom(m_pieces.size());
}

void AppController::divideRandom(int pieces)
{
    const int target = qBound(1, pieces, 2048);

    m_vrepCells.clear();
    m_vrepBlockOf.clear();
    m_vrepPieceOf.clear();
    m_vrepCuts.clear();
    m_vrepCutBlock.clear();
    m_vrepPieceCuts.clear();
    m_curvedCuts = false;
    m_wholeCutUsed = false;
    m_lastKind   = LastDivision::Random;
    m_lastPieces = target;

    m_stats = DivisionStats();
    m_stats.mode      = QStringLiteral("Random");
    m_stats.requested = target;
    m_stats.seed      = m_layoutSeed;

    m_warp.enabled = false;

    if (m_triv.isValid() && m_trivReal) {
        runBspInD(target);
        return;
    }

    if (m_triv.isValid()) {
        double dom[6];
        m_triv.domain(dom);

        const MeshData &ref = m_sourceMesh.isEmpty() ? m_mesh : m_sourceMesh;
        double ext[3];
        for (int a = 0; a < 3; ++a)
            ext[a] = qMax(1e-9, double(ref.bmax[a]) - double(ref.bmin[a]));

        const double wdom[6] = { 0.0, ext[0], 0.0, ext[1], 0.0, ext[2] };

        QElapsedTimer stageTimer;
        stageTimer.start();
        const MaterialField field = MaterialField::build(ref);
        m_stats.msVoxelise = stageTimer.elapsed();
        m_stats.hasField = field.isValid();
        if (field.isValid()) {
            for (int a = 0; a < 3; ++a)
                m_stats.voxels[a] = field.dim(a);
            m_stats.voxelsFilled   = field.filledCount();
            m_stats.voxelSide      = field.side(0);
            m_stats.materialVolume = field.total();
        }

        if (!field.isValid())
            qDebug().noquote()
                << "DIVIDE  could not voxelise the model - falling back to "
                   "splitting the cage by volume, so empty cells are possible";

        stageTimer.restart();
        const QVector<CellBox> world =
            PuzzleDivider::buildBspCells(wdom, target, 0.35, m_layoutSeed, 0.0,
                                         field.isValid() ? &field : nullptr);
        m_stats.msSplit = stageTimer.elapsed();
        m_worldCells = world;          // TEMPORARY - Stage 0 comparison

        QVector<CellBox> cells;
        cells.reserve(world.size());
        for (const CellBox &w : world) {
            CellBox c;
            for (int a = 0; a < 3; ++a) {
                const double lo = dom[a * 2], span = dom[a * 2 + 1] - lo;
                c.lo[a] = lo + span * (w.lo[a] / ext[a]);
                c.hi[a] = lo + span * (w.hi[a] / ext[a]);
            }
            cells.append(c);
        }

        QString note = QStringLiteral("recursive split · %1 of %2 cells · seed %3")
                           .arg(cells.size()).arg(target).arg(m_layoutSeed);
        if (cells.size() < target)
            note += QStringLiteral(" · below target: the rest would have been "
                                   "under the minimum piece size");
        runTrivCellDivision(cells, note);
        return;
    }

    const MeshData work = workingMesh();
    const double dom[6] = { work.bmin[0], work.bmax[0],
                            work.bmin[1], work.bmax[1],
                            work.bmin[2], work.bmax[2] };

    QVector<CellBox>     cells;
    QVector<PuzzlePiece> pieces_;
    QString              warning;
    int absorbed = 0;

    if (!MeshDivider::divideBspAbsorbing(work, target, 0.28, m_layoutSeed,
                                         &pieces_, &cells, &absorbed, &warning)) {
        setError(warning);
        return;
    }

    double smallestCell = 1e300;
    for (const CellBox &c : cells)
        for (int a = 0; a < 3; ++a)
            smallestCell = qMin(smallestCell, c.extent(a));

    QString note = QStringLiteral("recursive split · %1 of %2 cells · seed %3")
                       .arg(cells.size()).arg(target).arg(m_layoutSeed);
    if (absorbed > 0)
        note += QStringLiteral(" · %1 crumb(s) absorbed into their neighbour")
                    .arg(absorbed);
    if (cells.size() < target)
        note += QStringLiteral(" · below target: the rest would have been under"
                               " the minimum piece size");
    note += QStringLiteral(" · smallest cell side %1").arg(smallestCell, 0, 'g', 3);

    runCellDivision(cells, work, note);
}

void AppController::logDivision(const QString &what, const double domain[6],
                                const QVector<double> cuts[3]) const
{
    static const char *kAxis = "uvw";

    qDebug().noquote() << QStringLiteral("---- %1 ----").arg(what);
    for (int a = 0; a < 3; ++a) {
        const double lo = domain[a * 2], hi = domain[a * 2 + 1];

        QStringList positions, gaps;
        double prev = lo, minGap = 1e300, maxGap = 0.0;
        for (double c : cuts[a]) {
            positions << QString::number(c, 'f', 4);
            const double g = c - prev;
            gaps << QString::number(g, 'f', 4);
            minGap = qMin(minGap, g);
            maxGap = qMax(maxGap, g);
            prev = c;
        }
        const double tail = hi - prev;
        gaps << QString::number(tail, 'f', 4);
        minGap = qMin(minGap, tail);
        maxGap = qMax(maxGap, tail);

        const double ratio = (minGap > 1e-12) ? maxGap / minGap : 0.0;

        qDebug().noquote()
            << QStringLiteral("  %1: domain [%2, %3]  %4 piece(s)")
                   .arg(QChar(kAxis[a]))
                   .arg(lo, 0, 'f', 4).arg(hi, 0, 'f', 4)
                   .arg(cuts[a].size() + 1);
        qDebug().noquote()
            << QStringLiteral("     cuts   : %1")
                   .arg(positions.isEmpty() ? QStringLiteral("(none)")
                                            : positions.join(QStringLiteral(", ")));
        qDebug().noquote()
            << QStringLiteral("     spacing: %1   max/min = %2%3")
                   .arg(gaps.join(QStringLiteral(", ")))
                   .arg(ratio, 0, 'f', 2)
                   .arg(ratio > 1.005 ? QStringLiteral("  <- non-uniform")
                                      : QStringLiteral("  <- even"));
    }
}

void AppController::logCells(const QString &what, const QVector<CellBox> &cells) const
{
    qDebug().noquote() << QStringLiteral("---- %1 ----").arg(what);

    double vmin = 1e300, vmax = 0.0;
    for (const CellBox &c : cells) {
        vmin = qMin(vmin, c.volume());
        vmax = qMax(vmax, c.volume());
    }
    qDebug().noquote()
        << QStringLiteral("  cell volumes: min %1  max %2  spread %3x")
               .arg(vmin, 0, 'g', 4).arg(vmax, 0, 'g', 4)
               .arg(vmin > 1e-12 ? vmax / vmin : 0.0, 0, 'f', 2);

    // The cells are in the trivariate's PARAMETER space while the model is in
    // world space, so both frames are printed together - a cell list that does
    // not span the domain is invisible from volumes alone.
    double dom[6];
    m_triv.domain(dom);
    qDebug().noquote()
        << QStringLiteral("  domain: u[%1, %2] v[%3, %4] w[%5, %6]")
               .arg(dom[0], 0, 'f', 4).arg(dom[1], 0, 'f', 4)
               .arg(dom[2], 0, 'f', 4).arg(dom[3], 0, 'f', 4)
               .arg(dom[4], 0, 'f', 4).arg(dom[5], 0, 'f', 4);

    const MeshData &ref = m_sourceMesh.isEmpty() ? m_mesh : m_sourceMesh;
    qDebug().noquote()
        << QStringLiteral("  model:  x[%1, %2] y[%3, %4] z[%5, %6]")
               .arg(double(ref.bmin[0]), 0, 'f', 4).arg(double(ref.bmax[0]), 0, 'f', 4)
               .arg(double(ref.bmin[1]), 0, 'f', 4).arg(double(ref.bmax[1]), 0, 'f', 4)
               .arg(double(ref.bmin[2]), 0, 'f', 4).arg(double(ref.bmax[2]), 0, 'f', 4);

    for (int c = 0; c < cells.size(); ++c) {
        qDebug().noquote()
            << QStringLiteral("    cell %1: u[%2, %3] v[%4, %5] w[%6, %7]")
                   .arg(c, 3)
                   .arg(cells[c].lo[0], 0, 'f', 4).arg(cells[c].hi[0], 0, 'f', 4)
                   .arg(cells[c].lo[1], 0, 'f', 4).arg(cells[c].hi[1], 0, 'f', 4)
                   .arg(cells[c].lo[2], 0, 'f', 4).arg(cells[c].hi[2], 0, 'f', 4);
    }

    CellBox hull = cells.isEmpty() ? CellBox() : cells[0];
    for (const CellBox &c : cells)
        for (int a = 0; a < 3; ++a) {
            hull.lo[a] = qMin(hull.lo[a], c.lo[a]);
            hull.hi[a] = qMax(hull.hi[a], c.hi[a]);
        }
    qDebug().noquote()
        << QStringLiteral("  cells cover: u[%1, %2] v[%3, %4] w[%5, %6]"
                          "  (must equal the domain above)")
               .arg(hull.lo[0], 0, 'f', 4).arg(hull.hi[0], 0, 'f', 4)
               .arg(hull.lo[1], 0, 'f', 4).arg(hull.hi[1], 0, 'f', 4)
               .arg(hull.lo[2], 0, 'f', 4).arg(hull.hi[2], 0, 'f', 4);

    double diag = 0.0;
    for (int a = 0; a < 3; ++a) {
        const double e = m_mesh.bmax[a] - m_mesh.bmin[a];
        diag += e * e;
    }
    const double eps = 1e-6 * qMax(1.0, std::sqrt(diag));

    const auto links = PuzzleDivider::adjacencyOfBoxes(m_pieces, eps);
    QVector<int> valence(m_pieces.size(), 0);
    for (const auto &l : links) { ++valence[l.a]; ++valence[l.b]; }

    int vlo = 1 << 30, vhi = 0;
    double vsum = 0.0;
    for (int v : valence) { vlo = qMin(vlo, v); vhi = qMax(vhi, v); vsum += v; }
    qDebug().noquote()
        << QStringLiteral("  shared faces: %1 · neighbours per piece: min %2  max %3  mean %4")
               .arg(links.size()).arg(m_pieces.isEmpty() ? 0 : vlo).arg(vhi)
               .arg(m_pieces.isEmpty() ? 0.0 : vsum / m_pieces.size(), 0, 'f', 2);
}

void AppController::logPieceSizes() const
{
    if (m_pieces.isEmpty())
        return;

    float smallest = 1e30f, largest = 0.0f;
    qDebug().noquote() << QStringLiteral("  piece world sizes (bbox extents):");
    for (const PuzzlePiece &p : m_pieces) {
        smallest = qMin(smallest, p.largestSide());
        largest  = qMax(largest,  p.largestSide());
        qDebug().noquote()
            << QStringLiteral("    [%1,%2,%3]  %4 x %5 x %6   longest side %7")
                   .arg(p.i).arg(p.j).arg(p.k)
                   .arg(p.size[0], 8, 'f', 3)
                   .arg(p.size[1], 8, 'f', 3)
                   .arg(p.size[2], 8, 'f', 3)
                   .arg(p.largestSide(), 8, 'f', 3);
    }
    qDebug().noquote()
        << QStringLiteral("  longest side across pieces: min %1  max %2  spread %3x")
               .arg(smallest, 0, 'f', 3).arg(largest, 0, 'f', 3)
               .arg(smallest > 1e-9f ? largest / smallest : 0.0f, 0, 'f', 2);
}

void AppController::setError(const QString &msg)
{
    m_status   = m_fileName.isEmpty() ? QStringLiteral("Nothing loaded") : m_fileName;
    m_detail   = msg;
    m_hasError = true;
    emit statusChanged();
}

// Joints are cut DURING a division, so this cannot just set a flag - with
// pieces already on screen nothing would change, which reads as "joints are
// broken". Repeat the last division; m_layoutSeed is untouched, so the same
// cells come back and only the joints differ.
void AppController::setAddJoints(bool on)
{
    if (m_addJoints == on)
        return;
    m_addJoints = on;
    emit jointsChanged();

    if (!m_reDividing && !m_pieces.isEmpty())
        repeatLastDivision();
}

// Re-runs whichever division produced the pieces currently on screen. A
// division never changes addJoints, so this cannot recurse; the guard says so
// explicitly rather than relying on that staying true.
void AppController::repeatLastDivision()
{
    if (m_reDividing)
        return;

    m_reDividing = true;
    switch (m_lastKind) {
    case LastDivision::Uniform:
        divideUniform(m_lastCounts[0], m_lastCounts[1], m_lastCounts[2]);
        break;
    case LastDivision::Random:
        divideRandom(m_lastPieces);
        break;
    case LastDivision::BySize:
        divideBySize(m_lastMaxSize, m_lastMaxAxis);
        break;
    case LastDivision::None:
        break;
    }
    m_reDividing = false;
}

// Runs the planner and writes the four figures beside the model.
void AppController::planAndDrawFigures()
{
    m_figureNote.clear();
    m_planFigures.clear();
    m_planFolderUrl.clear();
    m_reportUrl.clear();
    if (m_sourceMesh.isEmpty() || m_pieces.isEmpty())
        return;

    const Planner::Graph graph = Planner::build(m_pieces, 1e-6, 0.0);
    const Planner::TranslationalBlocking model;
    const Planner::Plan plan = Planner::extract(graph, model);

    for (const QString &line : plan.describe(8))
        qDebug().noquote() << line;

    QElapsedTimer figureTimer;
    figureTimer.start();
    const PlannerFigure::Result fig =
        PlannerFigure::write(m_pieces, graph, model, plan,
                             PlannerFigure::folderFor(m_loadedPath),
                             QFileInfo(m_loadedPath).completeBaseName());
    for (const QString &problem : fig.problems)
        qWarning().noquote() << "FIGURE" << problem;

    m_figureNote = fig.written.isEmpty()
        ? QStringLiteral("Planner figures not written")
        : QStringLiteral("Planner figures: %1")
              .arg(QDir::toNativeSeparators(fig.folder));

    ++m_figureVersion;
    for (const QString &path : fig.written)
        m_planFigures << QUrl::fromLocalFile(path).toString()
                         + QStringLiteral("?v=%1").arg(m_figureVersion);
    m_stats.msFigures = figureTimer.elapsed();

    if (!fig.written.isEmpty())
        m_planFolderUrl = QUrl::fromLocalFile(fig.folder).toString();

    m_stats.hasPlan       = true;
    m_stats.contacts      = graph.contactCount();
    m_stats.planComplete  = plan.complete;
    m_stats.caveat        = AssemblyOrder::caveat();
    m_stats.booleanNote   = m_booleanNote;
    m_stats.jointNote     = m_jointNote;
    m_stats.minNeighbours = 0;
    m_stats.maxNeighbours = 0;
    for (int i = 0; i < graph.incident.size(); ++i) {
        const int d = int(graph.incident[i].size());
        m_stats.minNeighbours = (i == 0) ? d : qMin(m_stats.minNeighbours, d);
        m_stats.maxNeighbours = qMax(m_stats.maxNeighbours, d);
    }
    m_stats.assemblyOrder.clear();
    for (const Planner::Step &st : plan.assembly)
        m_stats.assemblyOrder
            << QStringLiteral("%1 (%2)").arg(st.piece)
                   .arg(QString::fromLatin1(Planner::dirName(st.dir)));
    m_stats.stuck = plan.stuck;
    m_stats.figureFiles.clear();
    for (const QString &path : fig.written)
        m_stats.figureFiles << QFileInfo(path).fileName();

}

// Written after the joints are cut, so their stage time and note reach the page.
void AppController::writeDivisionReport()
{
    m_reportUrl.clear();
    if (!m_stats.hasPlan || m_planFolderUrl.isEmpty())
        return;

    m_stats.jointNote = m_jointNote;

    const QString folder = QUrl(m_planFolderUrl).toLocalFile();
    QString reportErr;
    const QString report =
        DivisionReport::write(folder,
                              QFileInfo(m_loadedPath).completeBaseName(),
                              m_sourceMesh.isEmpty() ? m_mesh : m_sourceMesh,
                              m_pieces, m_stats, &reportErr);
    if (report.isEmpty()) {
        qWarning().noquote() << "REPORT" << reportErr;
        return;
    }
    m_reportUrl = QUrl::fromLocalFile(report).toString();
    if (!m_figureNote.isEmpty())
        m_figureNote += QStringLiteral(" \u00b7 report.html");
}

// Cut the DIVISION itself with a dovetail profile, on the untrimmed cell boxes.
// The same tooth is unioned onto one cell and subtracted from its neighbour, so
// the shared face becomes a dovetail and the two cells still tile the same
// volume. Doing it before the trim is what makes the joint seamless (every
// outer surface ends up model surface), gap-free (one solid cuts both sides)
// and reliable (box operands).
void AppController::cutCellDovetails()
{
    m_cellJointNote.clear();
    m_preJointPieces.clear();
    m_cellsDovetailed = false;
    if (!m_addJoints || m_pieces.size() < 2)
        return;

    // Graph and directional blocking decide WHERE the profile goes. One modest
    // notch per tolerated face, not a row of teeth on every face: measured, a
    // row took cow from 1.8s to 103s and left a piece as a raw box, and
    // insetting the teeth failed the trim on every piece of both models.
    const Planner::Graph graph = Planner::build(m_pieces, 1e-6, 0.0);
    const Planner::TranslationalBlocking bare;
    const Planner::Plan plan = Planner::extract(graph, bare);

    QVector<int> slideAxis;
    Planner::JointSet chosen;
    if (plan.complete)
        chosen = Planner::chooseDovetailsAlongOrder(graph, plan, &slideAxis);

    // A contact that is a sliver of the largest one is where the model is thin,
    // and a notch there removes more than it joins.
    double maxArea = 0.0;
    for (const Planner::Contact &ct : graph.contacts)
        maxArea = qMax(maxArea, ct.area);
    const double minArea = 0.15 * maxArea;

    int cut = 0, failed = 0, skipped = 0;
    QString firstErr;

    // The cells kept whole for trimPiecesToModel(): a cut can look successful
    // here and still cost a trim later, intermittently.
    m_preJointPieces = m_pieces;

    for (int ci = 0; ci < graph.contactCount(); ++ci) {
        const Planner::Contact &c = graph.contacts[ci];

        if (ci >= chosen.size() || !chosen[ci] ||
            ci >= slideAxis.size() || slideAxis[ci] < 0) {
            ++skipped;
            continue;
        }
        if (c.area < minArea) {
            ++skipped;                 // too thin a junction to notch
            continue;
        }
        if (c.lowSide < 0 || c.highSide >= m_pieces.size())
            continue;

        MeshData &A = m_pieces[c.lowSide].mesh;
        MeshData &B = m_pieces[c.highSide].mesh;
        if (A.isEmpty() || B.isEmpty())
            continue;

        // At this stage a piece IS its cell box, so the shared face is a whole
        // rectangle.
        const int n = c.axis, u = (n + 1) % 3, v = (n + 2) % 3;

        const double plane = 0.5 * (double(A.bmax[n]) + double(B.bmin[n]));
        const double lo0 = qMax(double(A.bmin[u]), double(B.bmin[u])),
                     hi0 = qMin(double(A.bmax[u]), double(B.bmax[u])),
                     lo1 = qMax(double(A.bmin[v]), double(B.bmin[v])),
                     hi1 = qMin(double(A.bmax[v]), double(B.bmax[v]));
        if (hi0 - lo0 <= 0.0 || hi1 - lo1 <= 0.0)
            continue;

        JointPlacement j;
        j.axis  = n;
        j.size  = 1.0;
        j.at[n] = plane;
        j.at[u] = 0.5 * (lo0 + hi0);
        j.at[v] = 0.5 * (lo1 + hi1);
        j.lo[0] = lo0;  j.hi[0] = hi0;
        j.lo[1] = lo1;  j.hi[1] = hi1;

        // The direction the removal order says this piece slides, so the notch
        // runs along the cut rather than across the way out of it.
        j.slide = slideAxis[ci];
        j.room  = 0.0;
        j.wall  = qMin(double(A.bmax[n]) - double(A.bmin[n]),
                       double(B.bmax[n]) - double(B.bmin[n]));

        QString err;
        if (IritJoint::cutDovetail(&A, &B, j, m_joint, &err))
            ++cut;
        else {
            ++failed;
            if (firstErr.isEmpty())
                firstErr = err;
            qWarning().noquote() << "CELL DOVETAIL contact" << ci << "-" << err;
        }
    }

    // A failed cut costs only ITS OWN contact, not the whole division.
    //
    // This used to abandon every dovetail as soon as one cut failed, on the
    // theory that a refused boolean leaves IRIT unable to finish a later trim.
    // A/B measured on five models, one build: abandoning cost spot all 4 of its
    // good cuts and armadillo all 3, and bought nothing - keeping the partial
    // set trimmed with 0 failures and no cage box on every model. Where the
    // trim does break (cow, bimba) trimPiecesToModel() catches it and re-trims
    // flat, which is the guard that actually holds, so this one only threw
    // joints away.
    m_cellsDovetailed = (cut > 0);

    m_cellJointNote =
        QStringLiteral("dovetail cut into %1 of %2 shared faces%3%4")
            .arg(cut).arg(graph.contactCount())
            .arg(skipped > 0
                     ? QStringLiteral(", %1 left straight (the order needs them "
                                      "flat, or the junction is too thin to "
                                      "notch)").arg(skipped)
                     : QString())
            .arg(failed > 0 ? QStringLiteral(", %1 failed (%2)")
                                  .arg(failed).arg(firstErr)
                            : QString());
    qDebug().noquote() << m_cellJointNote;
}

// Joints follow the removal order: the order is found first, and only the faces
// it tolerates get a joint.
void AppController::applyJoints()
{
    m_jointNote.clear();
    if (!m_addJoints)
        return;

    // The dovetail belongs in the CUT, so this is where the old path ends -
    // cutCellDovetails() has already run on the V-rep paths.
    //
    // Do NOT try to cut one here. This runs after the trim, and by then the
    // pieces are organic: the shared face is no longer a rectangle, so a tooth
    // sized from the bounding boxes misses the material on anything concave.
    // The old "add a tail to a finished piece" step is gone for the same
    // reason - it pushed lumps through the surface and took cow 6s to 600s.
    m_jointNote = m_cellJointNote.isEmpty()
                      ? QStringLiteral("joints: none - the dovetail is cut into "
                                       "the division itself, which only happens "
                                       "before the model trim. Press Bounding "
                                       "cage first, then Divide.")
                      : m_cellJointNote;
    return;

    // Asked for joints and then saying nothing at all is how this looks broken
    // rather than inapplicable.
    if (m_pieces.size() < 2) {
        m_jointNote = QStringLiteral("joints: none - a joint sits between two "
                                     "pieces, and this division produced %1")
                          .arg(m_pieces.size());
        return;
    }

    QElapsedTimer jointTimer;
    jointTimer.start();

    const Planner::Graph graph = Planner::build(m_pieces, 1e-6, 0.0);
    const Planner::TranslationalBlocking bare;
    const Planner::Plan plan = Planner::extract(graph, bare);

    if (!plan.complete) {
        m_jointNote = QStringLiteral("joints: none - no removal order exists "
                                     "under the translational model, so there is "
                                     "nothing to place joints along");
        return;
    }

    // Dovetails, not pegs: a mated dovetail blocks withdrawal ACROSS the face,
    // so chooseDovetailsAlongOrder marks a contact only when the piece that
    // leaves first slides parallel to it, and never gives one piece two slide
    // axes. Either would weld the puzzle shut.
    QVector<int> slideAxis;
    const Planner::JointSet chosen =
        Planner::chooseDovetailsAlongOrder(graph, plan, &slideAxis);

    // Replayed under the DOVETAIL model: the peg model would prove nothing
    // here, since the two allow opposite directions.
    const Planner::DovetailBlocking jointed(chosen, slideAxis);
    QString   why;
    const int broken = Planner::replay(graph, jointed, plan, &why);

    // minPinThickness is an absolute millimetre figure (three extrusion
    // widths), which silently drops every face on a model that is not in
    // millimetres - spot is ~1 unit across and the gate wants 4.44. Cap it at a
    // fraction of the model: real-scale models are unaffected, unit-scale ones
    // get the joint plus a note that the pins will not print as they stand.
    JointParams joint = m_joint;
    double modelDiag = 0.0;
    {
        float lo[3] = {  1e30f,  1e30f,  1e30f };
        float hi[3] = { -1e30f, -1e30f, -1e30f };
        for (const PuzzlePiece &p : m_pieces)
            for (int a = 0; a < 3; ++a) {
                lo[a] = qMin(lo[a], p.mesh.bmin[a]);
                hi[a] = qMax(hi[a], p.mesh.bmax[a]);
            }
        for (int a = 0; a < 3; ++a) {
            const double e = double(hi[a]) - double(lo[a]);
            modelDiag += e * e;
        }
        modelDiag = std::sqrt(qMax(0.0, modelDiag));
    }

    const double relativeFloor = 0.012 * modelDiag;
    const bool   floorRelaxed  = modelDiag > 0.0 &&
                                 relativeFloor < joint.minPinThickness;
    if (floorRelaxed)
        joint.minPinThickness = relativeFloor;

    int skipped = 0;
    const QVector<QVector<JointPlacement> > places =
        IritJoint::planPlacementsFor(m_pieces, graph, chosen, joint, &skipped,
                                     &slideAxis);

    // What actually got cut, in model units: otherwise a dovetail too small to
    // see is indistinguishable from one that was never placed.
    {
        int shown = 0;
        for (int i = 0; i < places.size() && shown < 8; ++i) {
            for (const JointPlacement &jp : places[i]) {
                if (jp.slide < 0 || !jp.pin)
                    continue;
                const double f = qMin(jp.hi[0] - jp.lo[0], jp.hi[1] - jp.lo[1]);
                qDebug().noquote()
                    << QStringLiteral("DOVETAIL piece %1  normal %2 slide %3  "
                                      "face %4 x %5  depth %6  narrow %7  "
                                      "wide %8")
                           .arg(i).arg(jp.axis).arg(jp.slide)
                           .arg(jp.hi[0] - jp.lo[0], 0, 'f', 3)
                           .arg(jp.hi[1] - jp.lo[1], 0, 'f', 3)
                           .arg(joint.dtDepth  * f, 0, 'f', 3)
                           .arg(joint.dtNarrow * f, 0, 'f', 3)
                           .arg(joint.dtWide   * f, 0, 'f', 3);
                ++shown;
                break;
            }
        }
    }

    // Held in a local and appended at the end: m_jointNote is built further down
    // and would overwrite anything assigned to it here.
    QString rotationNote;

    // Can each piece turn far enough to seat a rotate-to-engage joint? Tested
    // on the trimmed geometry and BEFORE the booleans below run: once a pin is
    // unioned on, the mesh is no longer the piece as it seats. (The old test
    // turned the bounding box, whose corners sit at the half-diagonal, so it
    // reported a collision at any angle.)
    {
        QVector<int> seatOrder;
        seatOrder.reserve(plan.assembly.size());
        for (const Planner::Step &s : plan.assembly)
            seatOrder.append(s.piece);

        const QVector<JointRotation::Result> spin =
            JointRotation::testAll(m_pieces, seatOrder, places);

        int stuck = 0, reallyTested = 0, triangleTests = 0;
        double worst = 360.0;
        for (const JointRotation::Result &r : spin) {
            triangleTests += r.tested;
            if (r.note.startsWith(QStringLiteral("no joint")))
                continue;               // nothing was placed on this piece
            ++reallyTested;
            if (!r.clear) { ++stuck; worst = qMin(worst, r.maxAngleDeg); }
        }

        for (const QString &line : JointRotation::describe(spin, seatOrder))
            qDebug().noquote() << line;

        // "All clear" and "nothing to test" are not the same answer.
        if (reallyTested == 0)
            rotationNote = QStringLiteral("rotation: NOT TESTED - no joint was "
                                          "placed, so nothing was turned");
        else if (triangleTests == 0)
            rotationNote = QStringLiteral("rotation: NOT TESTED - %1 piece(s) had "
                                          "a joint but every neighbour was "
                                          "dismissed on its bounding box, so no "
                                          "geometry was compared")
                               .arg(reallyTested);
        else if (stuck == 0)
            rotationNote = QStringLiteral("rotation: all %1 jointed piece(s) turn "
                                          "the full 90 deg in place (%2 triangle "
                                          "tests)")
                               .arg(reallyTested).arg(triangleTests);
        else
            rotationNote = QStringLiteral("rotation: %1 of %2 jointed piece(s) "
                                          "cannot turn - worst stops at %3 deg; a "
                                          "bayonet there needs a seating region "
                                          "that is rotationally symmetric about "
                                          "the joint axis")
                               .arg(stuck).arg(reallyTested).arg(worst, 0, 'f', 1);
    }

    int done = 0, failed = 0, cuts = 0, noCut = 0, wrongWay = 0, detached = 0;
    QString firstErr;

    // Total material before any joint is cut, so the drift afterwards says
    // whether joints only MOVED material or invented some.
    double volBefore = 0.0;
    for (const PuzzlePiece &pc : m_pieces)
        volBefore += std::fabs(IritSolid::signedVolume(pc.mesh));

    for (int i = 0; i < m_pieces.size() && i < places.size(); ++i) {
        if (places[i].isEmpty())
            continue;

        // A pin ADDS material and a hole REMOVES it. Measuring that is the only
        // thing that catches an inverted boolean, which reports success while
        // computing the opposite operation. Per piece, not per boolean, so it
        // is only conclusive for a piece carrying one kind of feature.
        int pins = 0, holes = 0;
        for (const JointPlacement &jp : places[i]) {
            if (jp.pin)
                ++pins;
            else
                ++holes;
        }
        // Kept so a joint that breaks its piece can be undone. A tooth can
        // still reach past material that tapers away behind the face, and the
        // on-face solidity test cannot see that.
        const MeshData beforeMesh = m_pieces[i].mesh;
        const double vBefore = std::fabs(IritSolid::signedVolume(m_pieces[i].mesh));

        int     applied = 0, refused = 0;
        QString err;
        // No clipTo: intersecting a small tooth with a 5k-100k triangle organic
        // mesh fails inside Bool_lib and cost most of the joints. The tail is
        // kept inside the material by its LENGTH instead - see dovetailSolid.
        if (IritJoint::apply(&m_pieces[i].mesh, places[i], joint, &err,
                             &applied, &refused)) {
            const double vAfter = std::fabs(IritSolid::signedVolume(m_pieces[i].mesh));
            const double delta  = vAfter - vBefore;

            if ((holes == 0 && pins > 0 && delta < 0.0) ||
                (pins == 0 && holes > 0 && delta > 0.0)) {
                ++wrongWay;
                qWarning().noquote()
                    << "JOINT piece" << i << "- volume moved the WRONG WAY:"
                    << pins << "pin(s)," << holes << "hole(s), delta" << delta;
                
                // If the volume goes the wrong way, the Boolean operation completely
                // failed (e.g. on a non-watertight mesh) and likely deleted the main 
                // body of the piece, leaving only the tool as a floating artifact!
                m_pieces[i].mesh = beforeMesh;
                continue;
            }

            // A joint cut where there is no material to attach it to leaves the
            // piece in two parts - what a "peg floating in space" is from the
            // inside, and the one failure the volume check cannot see: the
            // volume still rises, it just rises somewhere detached.
            const int parts = CageBoolean::components(m_pieces[i].mesh).size();
            if (parts > 1) {
                // Put it back rather than ship a piece in two halves: a puzzle
                // missing a joint still assembles, a piece in pieces does not.
                ++detached;
                m_pieces[i].mesh = beforeMesh;
                qWarning().noquote()
                    << "JOINT piece" << i << "- left" << parts
                    << "disconnected parts; rolled back to the un-jointed piece";
                continue;
            }

            ++done;
            cuts += applied;
            noCut += refused;
        }
        else {
            ++failed;
            if (firstErr.isEmpty())
                firstErr = err;
            qWarning().noquote() << "JOINT piece" << i << "-" << err;
        }
    }

    // Material should only have MOVED between pieces, never appeared: if the
    // total shifts, a tail added something the model did not have.
    {
        double volAfter = 0.0;
        for (const PuzzlePiece &pc : m_pieces)
            volAfter += std::fabs(IritSolid::signedVolume(pc.mesh));

        const double drift = volBefore > 1e-12
                                 ? (volAfter - volBefore) / volBefore : 0.0;
        qDebug().noquote()
            << QStringLiteral("JOINT VOLUME before %1 after %2 drift %3%")
                   .arg(volBefore, 0, 'f', 4)
                   .arg(volAfter,  0, 'f', 4)
                   .arg(drift * 100.0, 0, 'f', 3);
    }

    for (const QString &line : plan.describe(6))
        qDebug().noquote() << line;

    double smallest = 1e30;
    for (const QVector<JointPlacement> &list : places)
        for (const JointPlacement &j : list)
            smallest = qMin(smallest, j.thinnest > 0.0
                                          ? j.thinnest
                                          : IritJoint::thinnestFeature(joint, j.size));

    m_jointNote = QStringLiteral("joints: %1 of %2 faces dovetailed "
                                 "(planner-chosen), %3 piece(s) cut, "
                                 "%4 boolean(s)")
                      .arg(Planner::countJoints(chosen))
                      .arg(graph.contactCount()).arg(done).arg(cuts);
    // Name the model actually replayed, not a hardcoded string - this said
    // "jointed translational DBG" long after the replay moved to dovetails.
    m_jointNote += (broken < 0)
        ? QStringLiteral("; order holds under %1 - real collision check pending")
              .arg(jointed.name())
        : QStringLiteral("; ORDER BROKEN - %1").arg(why);
    if (smallest < 1e29)
        m_jointNote += QStringLiteral(", thinnest feature %1 mm")
                           .arg(smallest, 0, 'f', 2);
    if (smallest < 1.2)
        m_jointNote += QStringLiteral(" - UNDER 3 extrusions, will not print");
    if (noCut > 0)
        m_jointNote += QStringLiteral(", %1 boolean(s) DECLINED - a missing hole "
                                      "leaves a pin with nowhere to go").arg(noCut);
    if (skipped > 0)
        m_jointNote += QStringLiteral(", %1 face(s) too small").arg(skipped);
    if (wrongWay > 0)
        m_jointNote += QStringLiteral(", %1 piece(s) moved volume the WRONG WAY "
                                      "- a pin that shrinks its piece, or a hole "
                                      "that grows it, means the boolean ran "
                                      "inverted").arg(wrongWay);
    if (detached > 0)
        m_jointNote += QStringLiteral(", %1 piece(s) ROLLED BACK - the joint "
                                      "broke the piece in two, so it was "
                                      "returned un-jointed and whole")
                           .arg(detached);
    if (failed > 0)
        m_jointNote += QStringLiteral(", %1 piece(s) FAILED (%2)")
                           .arg(failed).arg(firstErr);

    if (floorRelaxed)
        m_jointNote += QStringLiteral("\nscale: this model is %1 across, so the "
                                      "1.2 mm printability floor was lowered to "
                                      "%2 to place a joint at all - the pins are "
                                      "correct geometry but will not print until "
                                      "the model is scaled up")
                           .arg(modelDiag, 0, 'g', 3)
                           .arg(joint.minPinThickness, 0, 'g', 3);

    if (!rotationNote.isEmpty())
        m_jointNote += QStringLiteral("\n") + rotationNote;

    m_stats.msJoints = jointTimer.elapsed();
}

// The joint caps of the limb split: each cap is a fan round its centre in the
// parts on both sides (the same triangles), taken once per part.
MeshData AppController::jointCapTriangles() const
{
    MeshData caps;
    for (const HarmonicFit::Part &P : m_parts) {
        for (const HarmonicFit::Cap &cap : P.caps) {
            int cv = -1;
            for (int v = 0; v < P.mesh.vertexCount() && cv < 0; ++v)
                if (std::fabs(P.mesh.pos[3 * v] - cap.centre[0]) < 1e-5 && std::fabs(P.mesh.pos[3 * v + 1] - cap.centre[1]) < 1e-5 &&
                    std::fabs(P.mesh.pos[3 * v + 2] - cap.centre[2]) < 1e-5) cv = v;
            if (cv < 0) continue;
            for (int t = 0; t < P.mesh.triangleCount(); ++t) {
                const uint32_t *T = &P.mesh.tris[3 * t];
                if (T[0] != uint32_t(cv) && T[1] != uint32_t(cv) && T[2] != uint32_t(cv)) continue;
                for (int k = 0; k < 3; ++k) {
                    const uint32_t id = caps.addVertex(P.mesh.pos[3 * T[k]], P.mesh.pos[3 * T[k] + 1], P.mesh.pos[3 * T[k] + 2]);
                    caps.tris.push_back(id);
                }
            }
        }
    }
    caps.computeBounds();
    return caps;
}

void AppController::checkInterlocking()
{
    if (m_checking) return;
    if (!canCheckInterlocking()) {
        setError(QStringLiteral("Check interlocking needs pieces from Cut in D: Fit trivariate, then Divide."));
        return;
    }
    if (m_pieces.size() > 31) {
        setError(QStringLiteral("%1 pieces: the interlocking check takes up to 31.").arg(m_pieces.size()));
        return;
    }
    QVector<MeshData> meshes;
    for (const PuzzlePiece &pc : m_pieces) meshes.append(pc.mesh);
    const QVector<CutInD::Cut> cuts = m_vrepCuts;
    const QVector<int> cutBlock = m_vrepCutBlock;
    const QVector<HarmonicFit::Result> fits = m_blockFits;
    // Per-part cutting: the joint caps between parts. Whole-model cutting:
    // no caps, but a cut beyond its own part is only a mesh - every cut face
    // triangle adds its contact too.
    MeshData caps;
    if (!m_wholeCutUsed) caps = jointCapTriangles();
    else {
        for (const PuzzlePiece &pc : m_pieces) {
            const QVector<char> skin = CutInD::trimTriangles(pc.mesh, m_sourceMesh);
            for (int t = 0; t < pc.mesh.triangleCount(); ++t) {
                if (skin[t]) continue;
                for (int k = 0; k < 3; ++k) {
                    const uint32_t v = pc.mesh.tris[3 * t + k];
                    caps.tris.push_back(caps.addVertex(pc.mesh.pos[3 * v], pc.mesh.pos[3 * v + 1], pc.mesh.pos[3 * v + 2]));
                }
            }
        }
        caps.computeBounds();
    }
    double size = 0.0;
    for (int a = 0; a < 3; ++a) size = std::max(size, double(m_sourceMesh.bmax[a] - m_sourceMesh.bmin[a]));
    const QFileInfo mi(m_loadedPath);
    const QString reportPath = mi.absolutePath() + QStringLiteral("/") + mi.completeBaseName() + QStringLiteral("_interlocking.md");
    const QString model = m_fileName;
    const QString division = m_divisionInfo.section(QLatin1Char('\n'), 0, 0);

    m_checking = true;
    emit checkingChanged();
    m_detail = QStringLiteral("Checking interlocking of %1 pieces: sampling the exact cuts in D …").arg(meshes.size());
    emit statusChanged();

    m_lockWatcher.setFuture(QtConcurrent::run([meshes, cuts, cutBlock, fits, caps, size, reportPath, model, division]() {
        QElapsedTimer tm;
        tm.start();
        const int n = meshes.size();
        const CutInD::Locate loc = CutInD::locator(meshes);
        QVector<DbgAnalysis::DbgContactSample> all;
        int fromCuts = 0, fromCaps = 0;
        for (int k = 0; k < cuts.size(); ++k) {
            const int b = k < cutBlock.size() ? cutBlock[k] : -1;
            if (b < 0 || b >= fits.size()) continue;
            for (const CutInD::Contact &c : CutInD::cutContacts(cuts[k], fits[b], loc, 24)) {
                DbgAnalysis::DbgContactSample d;
                d.a = c.a; d.b = c.b;
                for (int q = 0; q < 3; ++q) d.n[q] = c.n[q];
                all.append(d);
                ++fromCuts;
            }
        }
        for (const CutInD::Contact &c : CutInD::capContacts(caps, loc, 1e-3 * size)) {
            DbgAnalysis::DbgContactSample d;
            d.a = c.a; d.b = c.b;
            for (int q = 0; q < 3; ++q) d.n[q] = c.n[q];
            all.append(d);
            ++fromCaps;
        }
        const qint64 sampleMs = tm.elapsed();
        DbgOptions opt;
        const DbgReport r = DbgAnalysis::runContacts(n, all, opt);
        if (!r.valid)
            return QStringList{ QStringLiteral("Interlocking check failed"), r.error, QString() };

        auto ids = [](const QVector<int> &v) {
            QStringList o;
            for (int p : v) o << QString::number(p);
            return o.join(QStringLiteral(", "));
        };
        auto dirText = [](const double d[3]) {
            return QStringLiteral("(%1, %2, %3)").arg(d[0], 0, 'f', 3).arg(d[1], 0, 'f', 3).arg(d[2], 0, 'f', 3);
        };
        // Pieces no contact was found on: nothing holds them, as far as this knows.
        QVector<int> touched(n, 0);
        for (const DbgPair &pr : r.pairs) { touched[pr.a] = 1; touched[pr.b] = 1; }
        QVector<int> loose;
        for (int i = 0; i < n; ++i) if (!touched[i]) loose.append(i);

        QString verdict;
        if (r.strictSingleKey)
            verdict = QStringLiteral("INTERLOCKING with a single key: only piece %1 can come out first").arg(r.keySequence.value(0));
        else if (r.mobileSubsets == 0 && r.subsetSearchFull)
            verdict = QStringLiteral("LOCKED SOLID: no piece and no group can slide out - it cannot be taken apart (or put together) by translations");
        else if (r.mobileSubsets == 0)
            verdict = QStringLiteral("no piece or small group can slide out (groups up to %1 tested - not a proof)").arg(r.subsetSizeTested);
        else if (r.mobilePieces > 1)
            verdict = QStringLiteral("NOT interlocking: %1 pieces can slide out on their own").arg(r.mobilePieces);
        else if (r.mobilePieces == 1)
            verdict = QStringLiteral("NOT interlocking: piece %1 slides out, but so do other groups").arg(r.keyPiece);
        else
            verdict = QStringLiteral("NOT interlocking: no single piece moves, but a group of %1 does").arg(r.smallestMobile);

        QStringList d;
        d << QStringLiteral("Interlocking: %1").arg(verdict);
        d << QStringLiteral("%1 pieces, %2 touching pairs, %3 contact samples (%4 on the exact cuts, n' ~ J^-T n; %5 on joint caps) · %6 s")
                 .arg(n).arg(r.pairs.size()).arg(all.size()).arg(fromCuts).arg(fromCaps)
                 .arg(tm.elapsed() / 1000.0, 0, 'f', 1);
        d << QStringLiteral("pairs blocked in every direction: %1 of %2 · pieces that slide out alone: %3%4")
                 .arg(r.blockedPairs).arg(r.pairs.size()).arg(r.mobilePieces)
                 .arg(r.singleKey ? QStringLiteral(" (piece %1)").arg(r.keyPiece) : QString());
        if (r.smallestMobile > 0)
            d << QStringLiteral("smallest group that slides out: {%1} along %2")
                     .arg(ids(r.smallestMobileSet)).arg(dirText(r.escapeDir));
        if (r.subsetSearchFull)
            d << QStringLiteral("key level %1%2 - %3").arg(r.keyLevel)
                     .arg(r.keySequence.isEmpty() ? QString() : QStringLiteral(" (keys %1)").arg(ids(r.keySequence)))
                     .arg(r.levelStop);
        if (!loose.isEmpty())
            d << QStringLiteral("no contact found on piece(s) %1 - they count as free").arg(ids(loose));
        d << QStringLiteral("Report: %1").arg(QFileInfo(reportPath).fileName());

        // The report.
        QStringList md;
        md << QStringLiteral("# Interlocking - %1").arg(model);
        md << QString() << QStringLiteral("Division: %1").arg(division);
        md << QString() << QStringLiteral("**%1**").arg(verdict);
        md << QString();
        md << QStringLiteral("## How it was checked");
        md << QStringLiteral("- Translational directional blocking graph (first-order: the first small step along a direction), "
                             "%1 sampled directions; clearance decided %2.")
                  .arg(opt.directions).arg(r.exactClearance ? QStringLiteral("exactly (Gilbert)") : QStringLiteral("by the sampled directions"));
        md << QStringLiteral("- Contacts on the cuts: each exact cut surface S(s,t) in D sampled 24 x 24; n_D = S_s x S_t; the pieces on "
                             "its two sides found by stepping 0.003 off it in D and locating both points in the model; normal in the "
                             "model n' = n_u (M_v x M_w) + n_v (M_w x M_u) + n_w (M_u x M_v) (= det J J^-T n_D), turned to point from a to b.");
        md << QStringLiteral("- Contacts on the joint caps between limbs (flat, not cuts in D): the cap triangles' normals.");
        md << QStringLiteral("- %1 contact samples (%2 on cuts, %3 on caps); sampling %4 ms, total %5 ms.")
                  .arg(all.size()).arg(fromCuts).arg(fromCaps).arg(sampleMs).arg(tm.elapsed());
        md << QStringLiteral("- Groups tested: %1.").arg(r.subsetSearchFull ? QStringLiteral("all") : QStringLiteral("up to size %1 (not a proof)").arg(r.subsetSizeTested));
        md << QString() << QStringLiteral("## Results");
        md << QStringLiteral("- touching pairs: %1; blocked in every direction: %2").arg(r.pairs.size()).arg(r.blockedPairs);
        md << QStringLiteral("- pieces that slide out alone: %1%2").arg(r.mobilePieces)
                  .arg(r.singleKey ? QStringLiteral(" (piece %1)").arg(r.keyPiece) : QString());
        md << QStringLiteral("- groups that can slide out: %1").arg(r.mobileSubsets);
        if (r.smallestMobile > 0)
            md << QStringLiteral("- smallest: {%1} along %2, clearance %3 deg")
                      .arg(ids(r.smallestMobileSet)).arg(dirText(r.escapeDir)).arg(r.escapeClearanceDeg, 0, 'f', 2);
        if (r.subsetSearchFull) {
            md << QStringLiteral("- key level: %1%2 - %3").arg(r.keyLevel)
                      .arg(r.keySequence.isEmpty() ? QString() : QStringLiteral(" (keys in order: %1)").arg(ids(r.keySequence)))
                      .arg(r.levelStop);
            QStringList steps;
            for (const DbgReport::Group &g : r.disassembly) steps << QStringLiteral("{%1} along %2").arg(ids(g.pieces)).arg(dirText(g.dir));
            md << QStringLiteral("- one way to take it apart: %1%2").arg(steps.isEmpty() ? QStringLiteral("none") : steps.join(QStringLiteral(" -> ")))
                      .arg(r.disassemblyComplete ? QString() : QStringLiteral(" - then STOPS: what is left is locked"));
        }
        if (!loose.isEmpty())
            md << QStringLiteral("- no contact found on piece(s) %1: nothing holds them in this analysis").arg(ids(loose));
        md << QString() << QStringLiteral("## Pieces alone");
        md << QStringLiteral("| piece | free directions (of %1) | clearance |").arg(opt.directions);
        md << QStringLiteral("|---|---|---|");
        for (int i = 0; i < n; ++i) {
            const double c = i < r.pieceClearanceDeg.size() ? r.pieceClearanceDeg[i] : -1.0;
            md << QStringLiteral("| %1 | %2 | %3 |").arg(i).arg(r.pieceFreeDirs.value(i))
                      .arg(c < -1.5 ? QStringLiteral("free") : c < 0 ? QStringLiteral("blocked") : QStringLiteral("%1 deg").arg(c, 0, 'f', 2));
        }
        md << QString() << QStringLiteral("## Touching pairs");
        md << QStringLiteral("| pair | normals | spread (deg) | free directions | blocked | clearance (deg) |");
        md << QStringLiteral("|---|---|---|---|---|---|");
        for (const DbgPair &pr : r.pairs)
            md << QStringLiteral("| %1-%2 | %3 | %4 | %5 | %6 | %7 |").arg(pr.a).arg(pr.b).arg(pr.normals)
                      .arg(pr.spreadDeg, 0, 'f', 1).arg(pr.freeDirs)
                      .arg(pr.blocked ? QStringLiteral("**yes**") : QStringLiteral("no")).arg(pr.clearanceDeg, 0, 'f', 2);
        if (!r.openings.isEmpty()) {
            md << QString() << QStringLiteral("## Every way it opens (smaller side, up to 64)");
            for (const DbgReport::Group &g : r.openings)
                md << QStringLiteral("- {%1} along %2").arg(ids(g.pieces)).arg(dirText(g.dir));
        }
        QFile f(reportPath);
        QString written;
        if (f.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
            f.write(md.join(QStringLiteral("\n")).toUtf8());
            written = reportPath;
        }
        return QStringList{ QStringLiteral("Interlocking: %1").arg(verdict), d.join(QStringLiteral("\n")), written };
    }));
}

void AppController::finishInterlocking()
{
    m_checking = false;
    emit checkingChanged();
    const QStringList out = m_lockWatcher.result();
    if (out.size() < 2) return;
    for (const QString &line : out[1].split(QLatin1Char('\n')))
        qInfo().noquote() << QStringLiteral("LOCK  ") + line;
    m_status = QStringLiteral("%1 — %2").arg(m_fileName, out[0]);
    m_detail = out[1];
    m_hasError = false;
    emit statusChanged();
}
