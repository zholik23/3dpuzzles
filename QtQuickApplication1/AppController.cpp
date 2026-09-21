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
#include "JointRotation.h"
#include <QDebug>
#include <QRandomGenerator>
#include <algorithm>
#include <cmath>
#include <QElapsedTimer>
#include <QFileInfo>
#include <QDir>
#include <QLocale>

// LAST, and it must stay last. IritSolid.h pulls in inc_irit/irit_sm.h, which
// #defines _mkdir; put it above <QDir> and QDir::_mkdir turns into a
// redeclaration - "error C2535: member function already defined or declared".
// The same rule is written at the top of CadLoader.cpp and PieceExport.cpp:
// Qt headers first, IRIT headers after. Anything that includes an IRIT header
// belongs down here, not up with the project headers.
#include "IritSolid.h"

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

    // Before the trim: the cut itself gets the dovetail profile, so the joint
    // ends up inside the model instead of stuck onto it.
    cutCellDovetails();

    if (!m_sourceMesh.isEmpty()) {
        stageTimer.restart();
        const CageBoolean::Result br =
            CageBoolean::intersectAll(&m_pieces, m_sourceMesh, kPieceFineNess);
        m_stats.msTrim = stageTimer.elapsed();
        for (const QString &line : CageBoolean::describe(br))
            qDebug().noquote() << line;
        // Orphans are reported because they are what a "floating fragment" in
        // the viewport actually is: a cell whose intersection with the model
        // came out in disconnected lumps, and the spare lump could not be
        // welded into any neighbour without making that piece two solids, so it
        // was kept as a piece of its own. Measured with joints off and on - the
        // count is identical, so it is the division doing this, not the joints.
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

    // Before the trim: the cut itself gets the dovetail profile, so the joint
    // ends up inside the model instead of stuck onto it.
    cutCellDovetails();

    if (!m_sourceMesh.isEmpty()) {
        stageTimer.restart();
        const CageBoolean::Result br =
            CageBoolean::intersectAll(&m_pieces, m_sourceMesh, kPieceFineNess);
        m_stats.msTrim = stageTimer.elapsed();
        for (const QString &line : CageBoolean::describe(br))
            qDebug().noquote() << line;
        // Orphans are reported because they are what a "floating fragment" in
        // the viewport actually is: a cell whose intersection with the model
        // came out in disconnected lumps, and the spare lump could not be
        // welded into any neighbour without making that piece two solids, so it
        // was kept as a piece of its own. Measured with joints off and on - the
        // count is identical, so it is the division doing this, not the joints.
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

    m_lastKind   = LastDivision::Random;
    m_lastPieces = target;

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

// Joints are cut into the pieces DURING a division, so this cannot just set a
// flag: with pieces already on screen nothing would change and nothing would be
// said, which reads as "joints are broken" rather than "press Divide". Repeat
// the last division instead. m_layoutSeed is untouched, so the identical cells
// come back and only the joints differ.
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

// Cut the DIVISION itself with a dovetail profile.
//
// Runs on the untrimmed cell boxes, before the pieces are trimmed to the model.
// The same tooth is unioned onto one cell and subtracted from its neighbour, so
// the shared face stops being flat and becomes a dovetail - and because that is
// a re-cut and not an addition, the two cells still tile exactly the volume they
// tiled before.
//
// Everything good follows from doing it here rather than after the trim:
//   * after the trim every outer surface is model surface, so the joint cannot
//     be seen from outside - the shape stays seamless;
//   * the two sides are cut with the SAME solid, so there is no clearance gap;
//   * the operands are boxes, which is the case IRIT's booleans handle best.
void AppController::cutCellDovetails()
{
    m_cellJointNote.clear();
    if (!m_addJoints || m_pieces.size() < 2)
        return;

    // Graph and directional blocking decide WHERE the profile goes.
    //
    // MEASURED, and the reason this is not a row of teeth on every face: one
    // modest notch on the faces the removal order tolerates trims cleanly on
    // both test models (cow 1.8s, 0 failures). Cutting a row of larger teeth
    // into every contact instead took cow to 103s with a piece left as a raw
    // box, and insetting those teeth from the corners made it worse still -
    // every piece of both models failed to trim. IRIT's booleans do not
    // survive that many interacting cuts at this tessellation.
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

    // The dovetail belongs in the CUT, so this is where the old path ends.
    //
    // On the two V-rep paths cutCellDovetails() has already run, before the
    // trim, which is the right place: the joint ends up inside the model and
    // cannot show on the silhouette. The two mesh paths have no pre-trim stage,
    // so it runs here instead - still a re-cut between the two pieces, just
    // without the luxury of box operands.
    //
    // What is NOT done any more is the old "add a tail to a finished piece"
    // step. It is what pushed lumps through the model's surface, and once it
    // was emitting several teeth per face it also became pathologically slow -
    // cow.obj went from 6 seconds to over 600.
    // Do NOT try to cut one here. This runs after the trim, and by then the
    // pieces are organic: the shared face is no longer a rectangle, so a tooth
    // sized from the bounding boxes misses the material altogether on anything
    // concave ("Boolean: objects in a subtraction operation failed to
    // intersect" on armadillo). The dovetail is part of the CUT, and the cut
    // only exists before the trim - which is the V-rep path.
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

    // Dovetails, not pegs. chooseDovetailsAlongOrder marks a contact only when
    // the piece that leaves first slides PARALLEL to that face, and never hands
    // one piece two different slide axes - either of those would weld the
    // puzzle shut, because a mated dovetail blocks withdrawal across the face.
    QVector<int> slideAxis;
    const Planner::JointSet chosen =
        Planner::chooseDovetailsAlongOrder(graph, plan, &slideAxis);

    // Replayed under the DOVETAIL model, which is the real test that it still
    // comes apart once the joints are cut: replaying under the peg model would
    // prove nothing about dovetails, since the two allow opposite directions.
    const Planner::DovetailBlocking jointed(chosen, slideAxis);
    QString   why;
    const int broken = Planner::replay(graph, jointed, plan, &why);

    // The printability floor is an absolute millimetre figure: minPinThickness
    // guarantees three extrusion widths. It is meaningless on a model that is
    // not in millimetres, and worse than meaningless - it silently drops every
    // face. Spot is 0.94 x 1.69 x 1.72 units, and the gate needs a shared face
    // of 4.44 units, so nothing could ever be pegged.
    //
    // Cap the floor at a fraction of the model instead. On a real-scale model
    // the millimetre figure still wins and nothing changes; on a unit-scale one
    // the joint is cut and the note says the pins will not print as they stand.
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

    // What actually got cut, in model units. Without this there is no way to
    // tell a dovetail that is too small to see from one that was never placed
    // at all - both look like a flat face.
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

    // Can each piece actually turn far enough to seat a rotate-to-engage joint?
    // Tested on the trimmed geometry and BEFORE the booleans below run: once a
    // pin has been unioned on, the mesh is no longer the piece as it seats.
    //
    // The old test in AssemblyPlanner turned the piece's bounding box, whose
    // corners sit at the half-diagonal, so it reported a collision at any angle
    // and a bayonet could never pass it.
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

        // "All clear" and "nothing to test" are not the same answer, and
        // reporting the second as the first is how a rotation check quietly
        // stops meaning anything.
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
        // thing that catches an inverted boolean: IRIT decides inside from
        // outside by winding, and when it is wrong the call still reports
        // success while computing the opposite operation. Checked per piece
        // rather than per boolean, so it is only conclusive for a piece that
        // carries one kind of feature - which is the common case.
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
        // No clipTo. Clipping each tail against the model was tried and is
        // worse: intersecting a small tooth with a 5k-100k triangle organic
        // mesh fails inside Bool_lib ("failed to sort intersection list",
        // "empty polygon object") and cost most of the joints. The tail is kept
        // inside the material by its LENGTH instead - see dovetailSolid.
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
            // piece in two parts. That is precisely what a "peg floating in
            // space" is from the inside, and it is the one failure the volume
            // check cannot see: the volume still rises, it just rises somewhere
            // detached. The mid-face point can land in a void on a concave face.
            const int parts = CageBoolean::components(m_pieces[i].mesh).size();
            if (parts > 1) {
                // Put it back rather than ship a piece in two halves. The joint
                // is lost, the piece is whole - which is the right trade: a
                // puzzle missing a joint still assembles, a piece in pieces
                // does not.
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

    // Material should only have MOVED between pieces, never appeared. If the
    // total shifts, a tail added something the model did not have - which is
    // exactly the deformation the clip-to-mate step exists to prevent.
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
    // Name the model that was actually replayed, rather than a hardcoded
    // string: this said "jointed translational DBG" for a while after the
    // replay had already moved to the dovetail model.
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
