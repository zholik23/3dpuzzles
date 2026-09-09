#include "AssemblyDivider.h"

#include "MeshDivider.h"

#include <cmath>

const char *Contact::name(int dir)
{
    static const char *kNames[6] = { "-X", "+X", "-Y", "+Y", "-Z", "+Z" };
    return (dir >= 0 && dir < 6) ? kNames[dir] : "??";
}

int DividedSolid::contactCount() const
{
    int n = 0;
    for (const QVector<Contact> &c : contacts)
        n += c.size();
    return n / 2;                     // every face is recorded from both sides
}

// ------------------------------------------------------------------ Stage A --

bool AssemblyDivider::splitToTarget(const MeshData &solid, int targetPieces,
                                    quint32 seed, DividedSolid *out, QString *error)
{
    *out = DividedSolid();
    out->requestedPieces = targetPieces;

    if (solid.triangleCount() == 0) {
        if (error) *error = QStringLiteral("The solid has no geometry to split.");
        return false;
    }
    if (targetPieces < 1) {
        if (error) *error = QStringLiteral("Target piece count must be at least 1.");
        return false;
    }

    // --- split ------------------------------------------------------------
    const double domain[6] = { solid.bmin[0], solid.bmax[0],
                               solid.bmin[1], solid.bmax[1],
                               solid.bmin[2], solid.bmax[2] };

    Q_UNUSED(domain);

    // --- split and clip, absorbing crumbs ---------------------------------
    // Goes through divideBspAbsorbing rather than buildBspCells + divideCells so
    // Stage A sees exactly the division the app produces. Splitting them would
    // mean the contact graph was built on pieces nobody else has.
    QString clipReport;
    if (!MeshDivider::divideBspAbsorbing(solid, targetPieces, 0.28, seed,
                                         &out->pieces, &out->cells,
                                         &out->absorbed, &clipReport)) {
        if (error) *error = clipReport;
        return false;
    }
    out->clipReport = clipReport;
    out->emptyCells = int(out->cells.size()) - out->pieceCount();

    // --- contacts ---------------------------------------------------------
    const double eps = 1e-6 * qMax(1.0, double(solid.diagonal()));
    const auto links = PuzzleDivider::adjacencyOfBoxes(out->pieces, eps);

    out->contacts.resize(out->pieceCount());

    for (const PuzzleDivider::Neighbours &l : links) {
        if (l.a < 0 || l.b < 0 ||
            l.a >= out->pieceCount() || l.b >= out->pieceCount())
            continue;

        const PuzzlePiece &A = out->pieces[l.a];
        const PuzzlePiece &B = out->pieces[l.b];
        const int ax = l.axis;

        // Which side of A does B sit on? adjacencyOfBoxes only says the two meet
        // across this axis; the sign is what turns that into a blocked
        // direction, so resolve it here.
        const bool bIsAbove = B.p0[ax] >= A.p1[ax] - eps;

        // Shared face area, from the overlap on the other two axes.
        const int u = (ax + 1) % 3, v = (ax + 2) % 3;
        const double du = qMin(A.p1[u], B.p1[u]) - qMax(A.p0[u], B.p0[u]);
        const double dv = qMin(A.p1[v], B.p1[v]) - qMax(A.p0[v], B.p0[v]);
        const double area = qMax(0.0, du) * qMax(0.0, dv);

        const int dirFromA = ax * 2 + (bIsAbove ? 1 : 0);

        out->contacts[l.a].append({ l.b, dirFromA, area });
        out->contacts[l.b].append({ l.a, Contact::opposite(dirFromA), area });
    }

    // Record the original extent against the union of the piece boxes. Dividing
    // must not change the outer shape, and this is the cheapest check that it
    // has not: the two have to agree.
    for (int a = 0; a < 3; ++a) {
        out->solidMin[a] = solid.bmin[a];
        out->solidMax[a] = solid.bmax[a];
        out->unionMin[a] =  1e300;
        out->unionMax[a] = -1e300;
    }
    for (const PuzzlePiece &p : out->pieces)
        for (int a = 0; a < 3; ++a) {
            out->unionMin[a] = qMin(out->unionMin[a], double(p.mesh.bmin[a]));
            out->unionMax[a] = qMax(out->unionMax[a], double(p.mesh.bmax[a]));
        }

    // Pieces reported open by the clipper are still pieces - they just are not
    // watertight yet. Counted so Stage B's result can be read with that in mind.
    out->openPieces = clipReport.contains(QStringLiteral("open shell"))
                          ? out->pieceCount() : 0;

    return true;
}

QStringList AssemblyDivider::describeStageA(const DividedSolid &d, int maxPiecesShown)
{
    QStringList out;

    out << QStringLiteral("STAGE A  target %1 -> %2 cells, %3 pieces"
                          " (%4 empty), %5 shared faces")
               .arg(d.requestedPieces)
               .arg(d.cells.size())
               .arg(d.pieceCount())
               .arg(d.emptyCells)
               .arg(d.contactCount());

    if (d.absorbed > 0)
        out << QStringLiteral("         %1 crumb(s) absorbed - a cell that clipped"
                              " to almost no material was merged back with its"
                              " BSP sibling").arg(d.absorbed);

    if (d.pieceCount() != d.requestedPieces)
        out << QStringLiteral("         count is a TARGET - came out at %1, not %2"
                              " (cells outside the model hold no material)")
                   .arg(d.pieceCount()).arg(d.requestedPieces);

    const int shown = qMin(maxPiecesShown, d.pieceCount());
    for (int i = 0; i < shown; ++i) {
        const PuzzlePiece &p = d.pieces[i];
        // Cell AND material, because they are not the same thing and the
        // difference is where crumbs come from: a full-size cell out at a
        // spike tip can hold almost nothing.
        const double cellVol = (p.p1[0] - p.p0[0]) * (p.p1[1] - p.p0[1])
                             * (p.p1[2] - p.p0[2]);
        const double matVol  = double(p.size[0]) * double(p.size[1])
                             * double(p.size[2]);

        out << QStringLiteral("  piece %1  cell [%2 %3 %4]..[%5 %6 %7] vol %8"
                              "   material %9 x %10 x %11 vol %12   %13 neighbour(s)")
                   .arg(i, 3)
                   .arg(p.p0[0], 0, 'g', 4).arg(p.p0[1], 0, 'g', 4).arg(p.p0[2], 0, 'g', 4)
                   .arg(p.p1[0], 0, 'g', 4).arg(p.p1[1], 0, 'g', 4).arg(p.p1[2], 0, 'g', 4)
                   .arg(cellVol, 0, 'g', 4)
                   .arg(p.size[0], 0, 'g', 4).arg(p.size[1], 0, 'g', 4)
                   .arg(p.size[2], 0, 'g', 4)
                   .arg(matVol, 0, 'g', 4)
                   .arg(d.contacts[i].size());

        for (const Contact &c : d.contacts[i])
            out << QStringLiteral("        %1 -> piece %2   face area %3")
                       .arg(Contact::name(c.dir))
                       .arg(c.neighbour, 3)
                       .arg(c.area, 0, 'f', 2);
    }
    if (shown < d.pieceCount())
        out << QStringLiteral("  ... %1 more piece(s) not listed")
                   .arg(d.pieceCount() - shown);

    double worst = 0.0;
    for (int a = 0; a < 3; ++a) {
        worst = qMax(worst, std::fabs(d.unionMin[a] - d.solidMin[a]));
        worst = qMax(worst, std::fabs(d.unionMax[a] - d.solidMax[a]));
    }
    out << QStringLiteral("  outer extent  solid [%1 %2 %3]..[%4 %5 %6]")
               .arg(d.solidMin[0], 0, 'f', 3).arg(d.solidMin[1], 0, 'f', 3)
               .arg(d.solidMin[2], 0, 'f', 3)
               .arg(d.solidMax[0], 0, 'f', 3).arg(d.solidMax[1], 0, 'f', 3)
               .arg(d.solidMax[2], 0, 'f', 3);
    out << QStringLiteral("                pieces [%1 %2 %3]..[%4 %5 %6]   worst"
                          " deviation %7 %8")
               .arg(d.unionMin[0], 0, 'f', 3).arg(d.unionMin[1], 0, 'f', 3)
               .arg(d.unionMin[2], 0, 'f', 3)
               .arg(d.unionMax[0], 0, 'f', 3).arg(d.unionMax[1], 0, 'f', 3)
               .arg(d.unionMax[2], 0, 'f', 3)
               .arg(worst, 0, 'g', 3)
               .arg(worst < 1e-4 ? QStringLiteral("<- outer shape unchanged")
                                 : QStringLiteral("<- OUTER SHAPE ALTERED"));

    if (!d.clipReport.isEmpty())
        out << QStringLiteral("  clipper: %1").arg(d.clipReport);

    // Nothing has been tested yet - say so, so a Stage A log is never mistaken
    // for a result.
    out << QStringLiteral("         Stage A only: contact graph built, NOT yet"
                          " tested for assemblability.");
    return out;
}
