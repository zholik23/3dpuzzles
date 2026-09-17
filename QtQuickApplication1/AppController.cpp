//
// AppController - implementation: loading, the four division paths, joints,
// planner figures and the written report, plus the status text the UI shows.
//

#include "AppController.h"

#include "MaterialField.h"
#include "CadLoader.h"
#include "PlannerGraph.h"
#include "PlannerFigure.h"
#include "AssemblyOrder.h"
#include "DivisionReport.h"
#include <QDebug>
#include <QRandomGenerator>
#include <algorithm>
#include <cmath>
#include <QElapsedTimer>
#include <QFileInfo>
#include <QDir>
#include <QLocale>

AppController::AppController(QObject *parent)
    : QObject(parent)
{
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

    emit meshChanged();
    emit piecesChanged();
    emit trivariateChanged();
    emit statusChanged();
}

void AppController::adoptTrivariate(Trivariate tv, const QString &sourceDesc)
{
    if (!tv.isValid())
        return;

    m_triv = std::move(tv);
    m_pieces.clear();
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
    if (m_triv.tessellate(&whole, kModelFineNess, &err) && !whole.isEmpty()) {
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
}

void AppController::showWholeModel()
{
    m_pieces.clear();
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

    if (!m_sourceMesh.isEmpty()) {
        stageTimer.restart();
        const CageBoolean::Result br =
            CageBoolean::intersectAll(&m_pieces, m_sourceMesh, kPieceFineNess);
        m_stats.msTrim = stageTimer.elapsed();
        for (const QString &line : CageBoolean::describe(br))
            qDebug().noquote() << line;
        m_booleanNote = QStringLiteral("%1 trimmed to the model, %2 empty "
                                       "cell(s) dropped, %3 failed")
                            .arg(br.intersected).arg(br.dropped).arg(br.failed);
    }

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

// The V-rep path for an explicit cell list - what the BSP produces - followed by
// the same section 5 trim.
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

    if (!m_sourceMesh.isEmpty()) {
        stageTimer.restart();
        const CageBoolean::Result br =
            CageBoolean::intersectAll(&m_pieces, m_sourceMesh, kPieceFineNess);
        m_stats.msTrim = stageTimer.elapsed();
        for (const QString &line : CageBoolean::describe(br))
            qDebug().noquote() << line;
        m_booleanNote = QStringLiteral("%1 trimmed to the model, %2 empty "
                                       "cell(s) dropped, %3 failed")
                            .arg(br.intersected).arg(br.dropped).arg(br.failed);
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

    m_stats = DivisionStats();
    m_stats.mode      = QStringLiteral("Random");
    m_stats.requested = target;
    m_stats.seed      = m_layoutSeed;

    m_warp.enabled = false;

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
    // world space, so both frames are printed together: a cell list that does
    // not span the domain, or a domain that does not map onto the model, is
    // invisible from volumes alone.
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

void AppController::setAddJoints(bool on)
{
    if (m_addJoints == on)
        return;
    m_addJoints = on;
    emit jointsChanged();
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

// Joints follow the removal order: the order is found first, and only the faces
// it tolerates get a joint.
void AppController::applyJoints()
{
    m_jointNote.clear();
    if (!m_addJoints || m_pieces.size() < 2)
        return;

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

    const Planner::JointSet chosen = Planner::chooseAlongOrder(graph, plan);

    const Planner::JointedBlocking jointed(chosen);
    QString   why;
    const int broken = Planner::replay(graph, jointed, plan, &why);

    int skipped = 0;
    const QVector<QVector<JointPlacement> > places =
        IritJoint::planPlacementsFor(m_pieces, graph, chosen, m_joint, &skipped);

    int done = 0, failed = 0, cuts = 0, noCut = 0;
    QString firstErr;

    for (int i = 0; i < m_pieces.size() && i < places.size(); ++i) {
        if (places[i].isEmpty())
            continue;

        int     applied = 0, refused = 0;
        QString err;
        if (IritJoint::apply(&m_pieces[i].mesh, places[i], m_joint, &err,
                             &applied, &refused)) {
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

    for (const QString &line : plan.describe(6))
        qDebug().noquote() << line;

    double smallest = 1e30;
    for (const QVector<JointPlacement> &list : places)
        for (const JointPlacement &j : list)
            smallest = qMin(smallest, IritJoint::thinnestFeature(m_joint, j.size));

    m_jointNote = QStringLiteral("joints: %1 of %2 faces pegged (planner-chosen), "
                                 "%3 piece(s) cut, %4 boolean(s)")
                      .arg(Planner::countJoints(chosen))
                      .arg(graph.contactCount()).arg(done).arg(cuts);
    m_jointNote += (broken < 0)
        ? QStringLiteral("; order holds - valid under jointed translational DBG, "
                         "real collision check pending")
        : QStringLiteral("; ORDER BROKEN - %1").arg(why);
    if (smallest < 1e29)
        m_jointNote += QStringLiteral(", thinnest pin %1 mm").arg(smallest, 0, 'f', 2);
    if (smallest < 1.2)
        m_jointNote += QStringLiteral(" - UNDER 3 extrusions, will not print");
    if (noCut > 0)
        m_jointNote += QStringLiteral(", %1 boolean(s) DECLINED - a missing hole "
                                      "leaves a pin with nowhere to go").arg(noCut);
    if (skipped > 0)
        m_jointNote += QStringLiteral(", %1 face(s) too small").arg(skipped);
    if (failed > 0)
        m_jointNote += QStringLiteral(", %1 piece(s) FAILED (%2)")
                           .arg(failed).arg(firstErr);

    m_stats.msJoints = jointTimer.elapsed();
}
