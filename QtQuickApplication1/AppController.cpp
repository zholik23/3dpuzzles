#include "AppController.h"

#include "MaterialField.h"
#include "CadLoader.h"

#include <QDebug>
#include <QRandomGenerator>
#include <algorithm>
#include <cmath>
#include <QElapsedTimer>
#include <QFileInfo>
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

// ------------------------------------------------------------------ loading --

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
    // Synchronous on the GUI thread: a large STL will visibly stall the window.
    // Left that way deliberately for now - moving it to a worker is a change
    // worth making on its own.
    if (!CadLoader::load(path, &mesh, &error)) {
        m_mesh = MeshData();
        m_sourceMesh = MeshData();
        m_pieces.clear();
        m_triv = Trivariate();
        emit meshChanged();
        emit piecesChanged();
        emit trivariateChanged();
        setError(error);
        return;
    }

    const qint64 ms = timer.elapsed();
    m_mesh       = mesh;
    m_sourceMesh = mesh;        // survives adopting a cage over the top
    m_pieces.clear();
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

    // Extent is worth showing: it is the number that decides whether a piece
    // fits the build volume once the division runs.
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

// ----------------------------------------------------- choosing a trivariate --

void AppController::adoptTrivariate(Trivariate tv, const QString &sourceDesc)
{
    if (!tv.isValid())
        return;

    m_triv = std::move(tv);
    m_pieces.clear();

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

    // Show the whole trivariate until a division is asked for.
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
    m_divisionInfo.clear();
    emit piecesChanged();
    // meshChanged makes the view drop back to the single-part model.
    emit meshChanged();
    emit statusChanged();
}

// ----------------------------------------------------------------- dividing --

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

    // A dialog that filters by type still lets a name through without one, and
    // the writer picks the format from the extension - so default rather than
    // refuse.
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

void AppController::runDivision(const DivisionSpec &spec)
{
    QElapsedTimer timer;
    timer.start();

    QVector<PuzzlePiece> pieces;
    QString warning;
    if (!PuzzleDivider::divide(m_triv, spec, kPieceFineNess, &pieces, &warning)) {
        setError(warning);
        return;
    }

    m_pieces = std::move(pieces);

    // Elber Section 5, Fig. 14c -> 14d -> 14e. The division above gives boxy
    // sub-trivariates of the CAGE; intersecting each with the original model is
    // what trims them back to the real surface while leaving the interior cut
    // faces alone. Without it the pieces keep the cage's shape, which is why a
    // cage-divided model comes out cuboid.
    if (!m_sourceMesh.isEmpty()) {
        const CageBoolean::Result br =
            CageBoolean::intersectAll(&m_pieces, m_sourceMesh, kPieceFineNess);
        for (const QString &line : CageBoolean::describe(br))
            qDebug().noquote() << line;
        // Dropped cells are not failures: a box cage covers more than the
        // model, so some cells legitimately hold no material.
        m_booleanNote = QStringLiteral("%1 trimmed to the model, %2 empty "
                                       "cell(s) dropped, %3 failed")
                            .arg(br.intersected).arg(br.dropped).arg(br.failed);
    }

    double dom[6];
    m_triv.domain(dom);
    logDivision(QStringLiteral("V-rep cuts in parameter space · ") + spec.note,
                dom, spec.splits);
    logPieceSizes();

    applyJoints();
    describePieces(QStringLiteral("V-rep · ") + spec.note, spec.cellCount(), warning);
    m_divisionInfo += QStringLiteral(" · %1 ms").arg(timer.elapsed());

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

void AppController::runMeshDivision(const MeshDivisionSpec &spec, const MeshData &work)
{
    if (m_mesh.isEmpty()) {
        setError(QStringLiteral("Load a model first."));
        return;
    }

    QElapsedTimer timer;
    timer.start();

    // Cut faces are created flat. If they are about to be un-warped they need
    // enough triangles to bend smoothly, otherwise un-warping just tilts one
    // big facet. Straight cuts skip subdivision entirely and cost nothing.
    const double detail = m_warp.active()
        ? double(m_mesh.diagonal()) / 32.0
        : 0.0;

    QVector<PuzzlePiece> pieces;
    QString warning;
    if (!MeshDivider::divide(work, spec, &pieces, &warning, detail)) {
        setError(warning);
        return;
    }

    // Back to the model's own space. The original surface lands exactly where
    // it started; only the cuts keep the curve.
    MeshDivider::unwarp(&pieces, m_warp, m_mesh.bmin, m_mesh.bmax,
                        double(m_mesh.diagonal()));

    m_pieces = std::move(pieces);

    // The mesh path cuts in world space, so the "domain" is the working mesh's
    // bounding box and the cuts are millimetres, not parameters.
    const double dom[6] = { work.bmin[0], work.bmax[0],
                            work.bmin[1], work.bmax[1],
                            work.bmin[2], work.bmax[2] };
    logDivision(QStringLiteral("mesh cuts in world space · ") + spec.note,
                dom, spec.planes);
    logPieceSizes();

    applyJoints();
    describePieces(QStringLiteral("mesh · ") + spec.note +
                   QStringLiteral(" · ") + m_warp.describe(),
                   spec.cellCount(), warning);
    m_divisionInfo += QStringLiteral(" · %1 ms").arg(timer.elapsed());

    m_status   = QStringLiteral("%1 — %2 pieces").arg(m_fileName).arg(m_pieces.size());
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
    if (!PuzzleDivider::divideCells(m_triv, cells, kPieceFineNess,
                                    &pieces, &warning)) {
        setError(warning);
        return;
    }

    m_pieces = std::move(pieces);

    // Elber Section 5: the cells above are sub-trivariates of the CAGE, so each
    // still has the cage's outer shape until it is intersected with the model.
    if (!m_sourceMesh.isEmpty()) {
        const CageBoolean::Result br =
            CageBoolean::intersectAll(&m_pieces, m_sourceMesh, kPieceFineNess);
        for (const QString &line : CageBoolean::describe(br))
            qDebug().noquote() << line;
        m_booleanNote = QStringLiteral("%1 trimmed to the model, %2 empty "
                                       "cell(s) dropped, %3 failed")
                            .arg(br.intersected).arg(br.dropped).arg(br.failed);
    }

    logCells(QStringLiteral("V-rep BSP cells in parameter space · ") + note, cells);
    logPieceSizes();

    applyJoints();
    describePieces(QStringLiteral("V-rep · ") + note, cells.size(), warning);
    m_divisionInfo += QStringLiteral(" · %1 ms").arg(timer.elapsed());

    m_status   = QStringLiteral("%1 — %2 pieces").arg(m_triv.label()).arg(m_pieces.size());
    m_detail   = m_divisionInfo;
    m_hasError = false;

    emit piecesChanged();
    emit statusChanged();
}

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
    if (!MeshDivider::divideCells(work, cells, &pieces, &warning, detail)) {
        setError(warning);
        return;
    }

    MeshDivider::unwarp(&pieces, m_warp, m_mesh.bmin, m_mesh.bmax,
                        double(m_mesh.diagonal()));

    m_pieces = std::move(pieces);
    logCells(note, cells);
    logPieceSizes();

    applyJoints();
    describePieces(note, cells.size(), warning);
    m_divisionInfo += QStringLiteral(" · %1 ms").arg(timer.elapsed());

    m_status   = QStringLiteral("%1 — %2 pieces").arg(m_fileName).arg(m_pieces.size());
    m_detail   = m_divisionInfo;
    m_hasError = false;

    emit piecesChanged();
    emit statusChanged();
}

void AppController::describePieces(const QString &note, int gridCells,
                                   const QString &warning)
{
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

    // The adjacency graph is what the joint planner will run its spanning tree
    // over, so its size is worth reporting even before joints exist.
    const int shared = PuzzleDivider::adjacency(m_pieces, DivisionSpec()).size();

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
    if (!(maxSizeMM > 0.0)) {
        setError(QStringLiteral("Max piece size has to be greater than zero."));
        return;
    }
    const double budget[3] = { maxSizeMM, maxSizeMM, maxSizeMM };
    const int    cap       = qBound(1, maxPerAxis, 64);

    // Straight cuts: bending them would move the pieces off the sizes that were
    // just measured, and the point of this mode is that the sizes are right.
    m_warp.enabled = false;

    if (m_triv.isValid()) {
        // Walks the trivariate's own arc length, then measures each cell's real
        // world bbox and tightens until the pieces genuinely fit.
        runDivision(PuzzleDivider::toBuildVolume(m_triv, budget, cap));
        return;
    }
    // On a mesh this caps the piece size but comes out evenly spaced - in world
    // space equal slabs already satisfy a size limit. The unequal-cut mode for
    // a mesh is Random; this one earns its name only on a trivariate.
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

    // Straight planar cuts. The warp is left off: it deforms the outer shape of
    // a coarse mesh, and the randomness that matters is in WHERE the cuts fall.
    m_warp.enabled = false;

    if (m_triv.isValid()) {
        // Recursive split, not a grid. Rounding the target up to a cube
        // (ceil(cbrt(n))^3) is what used to turn a request for 2 pieces into 8,
        // and it also forced every interior piece to have exactly six
        // neighbours - the regular arrangement the BSP exists to avoid.
        double dom[6];
        m_triv.domain(dom);

        // Split in WORLD proportions, then map the cells back to parameter
        // space. The cage maps its unit domain onto a bounding box that is
        // rarely cubic, so "cut the longest axis" and the minimum-size floor
        // are only meaningful once the domain is scaled to real extents -
        // otherwise a parameter-cubic cell comes out as a long world slab.
        const MeshData &ref = m_sourceMesh.isEmpty() ? m_mesh : m_sourceMesh;
        double ext[3];
        for (int a = 0; a < 3; ++a)
            ext[a] = qMax(1e-9, double(ref.bmax[a]) - double(ref.bmin[a]));

        const double wdom[6] = { 0.0, ext[0], 0.0, ext[1], 0.0, ext[2] };

        // Voxelise the model so the split can follow the material instead of
        // the cage. Local coordinates: the field's origin is the model's
        // minimum corner, which is exactly what wdom above is measured from.
        const MaterialField field = MaterialField::build(ref);
        if (!field.isValid())
            qDebug().noquote()
                << "DIVIDE  could not voxelise the model - falling back to "
                   "splitting the cage by volume, so empty cells are possible";

        const QVector<CellBox> world =
            PuzzleDivider::buildBspCells(wdom, target, 0.35, m_layoutSeed, 0.0,
                                         field.isValid() ? &field : nullptr);

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

    // minSide 0 = automatic: the splitter derives a floor from the model size
    // and the target count. That bounds the CELL, which is not the same thing
    // as bounding the PIECE - on an organic model a full-size cell can still
    // clip down to a crumb at a claw tip or the end of a tail. So the cells are
    // clipped, the result measured, and any crumb absorbed by collapsing its
    // parent in the tree: a cell and its sibling merge back into the parent box
    // exactly, so the division stays a partition of boxes throughout.
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


// ------------------------------------------------------------------ logging --

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

        // A ratio of 1.00 means the cuts are evenly spaced; anything above that
        // is the division actually being non-uniform.
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

// A recursive split has no per-axis cut lists to print, so what matters is the
// spread of the cells themselves and how many neighbours each piece ends up
// with - the two things a global-plane grid cannot vary.
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

    // Neighbour counts. On a plane grid every interior piece has exactly six;
    // anything else here is the proof the split is genuinely irregular.
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
    // If the size-based division did its job these two are close together.
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

// ------------------------------------------------------------------- joints --

void AppController::setAddJoints(bool on)
{
    if (m_addJoints == on)
        return;
    m_addJoints = on;
    emit jointsChanged();
}

void AppController::applyJoints()
{
    m_jointNote.clear();
    if (!m_addJoints || m_pieces.size() < 2)
        return;

    // The planner decides WHERE joints may go, not the geometry. A pin on every
    // shared face pegs most pieces on two axes at once and the puzzle then does
    // not come apart at all - so the removal order is found first, and only the
    // faces that order can tolerate get a joint.
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

    // Choose, then verify. The order must still hold with the pegs fitted.
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
}
