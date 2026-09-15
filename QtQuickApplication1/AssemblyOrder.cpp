#include "AssemblyOrder.h"

#include "PlannerGraph.h"
#include "PlannerBlocking.h"
#include "PlannerOrder.h"

#include <cmath>

namespace {

// Does `piece` actually have material at the shared face?
//
// The contact was found between two axis-aligned CELLS. After the Boolean the
// piece may have been pulled back from that face entirely, in which case the
// cells meet but the pieces do not and the contact is a phantom - it will
// register as blocking when nothing is physically in the way.
//
// Answered by looking for a vertex of the piece's own mesh that lies on the
// plane, inside the overlap rectangle. A vertex is enough: if the surface
// reaches the plane at all it has vertices there, because the plane is one of
// the cutting planes and the Boolean lays an edge loop along it.
bool materialAtFace(const PuzzlePiece &p, int axis, double plane,
                    const double lo2[2], const double hi2[2], double tol)
{
    const int b0 = (axis + 1) % 3, b1 = (axis + 2) % 3;
    for (int v = 0; v + 2 < p.mesh.pos.size(); v += 3) {
        const double n = double(p.mesh.pos[v + axis]);
        if (std::fabs(n - plane) > tol)
            continue;
        const double u = double(p.mesh.pos[v + b0]);
        const double w = double(p.mesh.pos[v + b1]);
        if (u >= lo2[0] - tol && u <= hi2[0] + tol &&
            w >= lo2[1] - tol && w <= hi2[1] + tol)
            return true;
    }
    return false;
}

QString dirList(Planner::DirMask m, bool wantBlocked)
{
    QStringList out;
    for (int d = 0; d < Planner::DirectionCount; ++d)
        if (Planner::isBlocked(m, d) == wantBlocked)
            out << QString::fromLatin1(Planner::dirName(d));
    return out.isEmpty() ? QStringLiteral("none") : out.join(QLatin1Char(' '));
}

} // namespace

QString AssemblyOrder::caveat()
{
    return QStringLiteral("assemblable under translational blocking; "
                          "rotational/swept check pending");
}

AssemblyOrder::Result AssemblyOrder::run(const QVector<PuzzlePiece> &pieces,
                                         double tol)
{
    const Planner::TranslationalBlocking model;
    return run(pieces, model, tol);
}

AssemblyOrder::Result AssemblyOrder::run(const QVector<PuzzlePiece> &pieces,
                                         const Planner::BlockingModel &model,
                                         double tol)
{
    Result r;

    // ---------------------------------------------------------------- step 1
    const Planner::Graph g = Planner::build(pieces, tol, 0.0);
    r.contacts = g.contactCount();

    r.minNeighbours = g.pieceCount() ? g.pieceCount() : 0;
    r.maxNeighbours = 0;
    for (int i = 0; i < g.pieceCount(); ++i) {
        const int n = g.incident[i].size();
        r.minNeighbours = qMin(r.minNeighbours, n);
        r.maxNeighbours = qMax(r.maxNeighbours, n);
    }

    r.adjacencyLog << QStringLiteral("STEP 1  adjacency: %1 pieces, %2 shared faces")
                          .arg(g.pieceCount()).arg(g.contactCount());
    for (int i = 0; i < g.pieceCount(); ++i) {
        QStringList nb;
        for (int c : g.incident[i])
            nb << QStringLiteral("%1 on %2")
                      .arg(g.otherSide(c, i))
                      .arg(QString::fromLatin1(Planner::dirName(g.directionFrom(c, i))));
        r.adjacencyLog << QStringLiteral("   piece %1: %2")
                              .arg(i, 2)
                              .arg(nb.isEmpty() ? QStringLiteral("no neighbours")
                                                : nb.join(QStringLiteral(", ")));
    }

    // How many of those contacts are between cells that touch while the pieces
    // inside them do not. Must be zero on an axis-aligned model.
    //
    // The contact planes are in the units of p0/p1 - the trivariate's PARAMETER
    // domain on the cage path - while the meshes are in world coordinates, so
    // the two cannot be compared directly. The map between them is recovered
    // from the pieces themselves: the cells tile the cage, and the trimmed
    // pieces together span the model, so the extremes of each give the two
    // boxes the map runs between. On the mesh path, where p0/p1 are already
    // world, this comes out as the identity.
    double pLo[3], pHi[3], wLo[3], wHi[3];
    bool haveBox = false;
    for (const PuzzlePiece &q : pieces) {
        if (q.mesh.triangleCount() == 0)
            continue;
        for (int a = 0; a < 3; ++a) {
            const double p0 = q.p0[a], p1 = q.p1[a];
            const double w0 = double(q.mesh.bmin[a]), w1 = double(q.mesh.bmax[a]);
            if (!haveBox) {
                pLo[a] = p0; pHi[a] = p1; wLo[a] = w0; wHi[a] = w1;
            } else {
                pLo[a] = qMin(pLo[a], p0); pHi[a] = qMax(pHi[a], p1);
                wLo[a] = qMin(wLo[a], w0); wHi[a] = qMax(wHi[a], w1);
            }
        }
        haveBox = true;
    }

    if (haveBox) {
        for (int c = 0; c < g.contactCount(); ++c) {
            const Planner::Contact &k = g.contacts[c];
            const PuzzlePiece &A = pieces[k.lowSide], &B = pieces[k.highSide];
            const int b0 = (k.axis + 1) % 3, b1 = (k.axis + 2) % 3;

            const auto toWorld = [&](int axis, double v) {
                const double span = pHi[axis] - pLo[axis];
                if (!(std::fabs(span) > 1e-12))
                    return v;
                return wLo[axis] + (v - pLo[axis]) * (wHi[axis] - wLo[axis]) / span;
            };
            const auto scaleOf = [&](int axis) {
                const double span = pHi[axis] - pLo[axis];
                return (std::fabs(span) > 1e-12)
                           ? std::fabs((wHi[axis] - wLo[axis]) / span) : 1.0;
            };

            const double plane = toWorld(k.axis, k.plane);
            const double lo2[2] = { toWorld(b0, qMax(A.p0[b0], B.p0[b0])),
                                    toWorld(b1, qMax(A.p0[b1], B.p0[b1])) };
            const double hi2[2] = { toWorld(b0, qMin(A.p1[b0], B.p1[b0])),
                                    toWorld(b1, qMin(A.p1[b1], B.p1[b1])) };

            // A world-space tolerance: a fraction of the thinner piece, so it
            // scales with the model instead of assuming its units.
            const double faceTol =
                qMax(tol * scaleOf(k.axis), 1e-3 * k.depth * scaleOf(k.axis));

            if (!materialAtFace(A, k.axis, plane, lo2, hi2, faceTol) ||
                !materialAtFace(B, k.axis, plane, lo2, hi2, faceTol))
                ++r.phantomContacts;
        }
    }
    if (r.phantomContacts > 0)
        r.adjacencyLog << QStringLiteral(
            "   NOTE %1 of %2 contacts are between cells that meet where the "
            "trimmed pieces do not - counted as blocking, so the answer errs "
            "towards stuck")
                .arg(r.phantomContacts).arg(g.contactCount());

    // ---------------------------------------------------------------- step 2
    r.blockingLog << QStringLiteral("STEP 2  blocking under: %1").arg(model.name());
    r.blockingLog << QStringLiteral("        %1").arg(model.caveat());

    QVector<bool> all(g.pieceCount(), true);
    for (int i = 0; i < g.pieceCount(); ++i) {
        const Planner::DirMask m = model.blocked(g, i, all);
        QStringList by;
        for (int d = 0; d < Planner::DirectionCount; ++d) {
            if (!Planner::isBlocked(m, d))
                continue;
            QStringList who;
            for (int p : model.blockers(g, i, d, all))
                who << QString::number(p);
            by << QStringLiteral("%1 by %2")
                      .arg(QString::fromLatin1(Planner::dirName(d)),
                           who.join(QLatin1Char('/')));
        }
        r.blockingLog << QStringLiteral("   piece %1: free %2 | blocked %3")
                             .arg(i, 2)
                             .arg(dirList(m, false), dirList(m, true));
        if (!by.isEmpty())
            r.blockingLog << QStringLiteral("             %1").arg(by.join(QStringLiteral(", ")));
    }

    // ---------------------------------------------------------------- step 3
    const Planner::Plan plan = Planner::extract(g, model);
    r.assemblable = plan.complete;
    for (const Planner::Step &s : plan.removal)
        r.removal.append({ s.piece, s.dir });
    for (const Planner::Step &s : plan.assembly)
        r.assembly.append({ s.piece, s.dir });
    r.stuck = plan.stuck;

    if (plan.complete) {
        QStringList seq;
        for (const Step &s : r.assembly)
            seq << QString::number(s.piece);
        r.resultLog << QStringLiteral("STEP 3  ASSEMBLY ORDER: piece %1")
                           .arg(seq.join(QStringLiteral(", ")));
        for (int i = 0; i < r.assembly.size(); ++i)
            r.resultLog << QStringLiteral("   %1. piece %2, inserted along %3")
                               .arg(i + 1, 2).arg(r.assembly[i].piece, 2)
                               .arg(QString::fromLatin1(Planner::dirName(r.assembly[i].dir)));
        r.resultLog << QStringLiteral("        %1").arg(caveat());
    } else {
        QStringList ids;
        for (int p : r.stuck)
            ids << QString::number(p);
        r.resultLog << QStringLiteral("STEP 3  DEADLOCK: pieces {%1} are mutually blocked")
                           .arg(ids.join(QStringLiteral(", ")));
        r.resultLog << QStringLiteral(
            "        %1 of %2 pieces came off before it stuck")
                .arg(r.removal.size()).arg(g.pieceCount());
        r.resultLog << QStringLiteral(
            "        the search is greedy and never backtracks, so this shows "
            "no order was FOUND - not that none exists");
    }
    return r;
}

QStringList AssemblyOrder::Result::describe() const
{
    QStringList out;
    out << adjacencyLog << QString() << blockingLog << QString() << resultLog;
    return out;
}
