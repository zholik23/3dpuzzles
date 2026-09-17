//
// AssemblyPlanner - implementation: the spanning tree of spiral joints, dovetails
// on the remaining edges, and the box tests for approach and rotation sweep.
//

#include "AssemblyPlanner.h"

#include <QQueue>
#include <algorithm>
#include <cmath>

int AssemblyPlan::spiralCount() const
{
    int n = 0;
    for (const PlannedJoint &j : joints)
        if (j.kind == PlannedJoint::Spiral)
            ++n;
    return n;
}

int AssemblyPlan::dovetailCount() const
{
    return int(joints.size()) - spiralCount();
}

namespace {

struct Box {
    double lo[3], hi[3];

    bool overlaps(const Box &o, double eps) const
    {
        for (int a = 0; a < 3; ++a)
            if (hi[a] - o.lo[a] <= eps || o.hi[a] - lo[a] <= eps)
                return false;
        return true;
    }
};

Box boxOf(const PuzzlePiece &p)
{
    Box b;
    for (int a = 0; a < 3; ++a) { b.lo[a] = p.p0[a]; b.hi[a] = p.p1[a]; }
    return b;
}

// The corridor a piece travels down on its way in: its own box, extended from
// its final position out to the edge of the world.
Box approachCorridor(const Box &b, int axis, bool fromPositiveSide, double reach)
{
    Box c = b;
    if (fromPositiveSide) c.hi[axis] += reach;
    else                  c.lo[axis] -= reach;
    return c;
}

// The volume a piece sweeps while turning about an axis. Sampled as the union of
// the box at several angles, so too few samples miss the bulge between them and
// would pass an assembly that actually jams.
Box rotationSweep(const Box &b, int axis, double cu, double cv,
                  double deg, int samples)
{
    const int u = (axis + 1) % 3;
    const int v = (axis + 2) % 3;

    Box s;
    s.lo[axis] = b.lo[axis];
    s.hi[axis] = b.hi[axis];
    s.lo[u] = s.lo[v] =  1e300;
    s.hi[u] = s.hi[v] = -1e300;

    const double rad = deg * 3.14159265358979323846 / 180.0;
    samples = qMax(2, samples);

    for (int i = 0; i < samples; ++i) {
        const double t = rad * double(i) / double(samples - 1);
        const double c = std::cos(t), sn = std::sin(t);

        for (int k = 0; k < 4; ++k) {
            const double pu = ((k & 1) ? b.hi[u] : b.lo[u]) - cu;
            const double pv = ((k & 2) ? b.hi[v] : b.lo[v]) - cv;
            const double ru = cu + pu * c - pv * sn;
            const double rv = cv + pu * sn + pv * c;
            s.lo[u] = qMin(s.lo[u], ru); s.hi[u] = qMax(s.hi[u], ru);
            s.lo[v] = qMin(s.lo[v], rv); s.hi[v] = qMax(s.hi[v], rv);
        }
    }
    return s;
}

}

// Spanning tree of spiral joints rooted at the best-connected piece, dovetails
// on the loop-closing edges, then the locking and collision checks.
AssemblyPlan AssemblyPlanner::plan(const QVector<PuzzlePiece> &pieces,
                                   const QVector<PuzzleDivider::Neighbours> &links,
                                   const Params &params)
{
    AssemblyPlan out;
    const int n = pieces.size();
    if (n == 0) {
        out.problems << QStringLiteral("No pieces to plan.");
        return out;
    }

    struct Edge { int other, axis, id; };
    QVector<QVector<Edge>> adj(n);
    for (int e = 0; e < links.size(); ++e) {
        const auto &l = links[e];
        adj[l.a].append({ l.b, l.axis, e });
        adj[l.b].append({ l.a, l.axis, e });
    }

    int root = 0;
    for (int i = 1; i < n; ++i)
        if (adj[i].size() > adj[root].size())
            root = i;

    out.parentOf.assign(n, -1);
    QVector<bool> seen(n, false);
    QVector<bool> isTreeEdge(links.size(), false);

    QQueue<int> queue;
    queue.enqueue(root);
    seen[root] = true;
    out.order.append(root);

    while (!queue.isEmpty()) {
        const int cur = queue.dequeue();
        for (const Edge &e : adj[cur]) {
            if (seen[e.other])
                continue;
            seen[e.other]      = true;
            out.parentOf[e.other] = cur;
            isTreeEdge[e.id]   = true;
            out.order.append(e.other);
            queue.enqueue(e.other);
        }
    }

    for (int i = 0; i < n; ++i)
        if (!seen[i])
            out.problems << QStringLiteral(
                "Piece %1 touches nothing - it cannot be joined to the assembly.").arg(i);

    QVector<int> spiralAxisOf(n, -1);
    for (int e = 0; e < links.size(); ++e) {
        const auto &l = links[e];
        PlannedJoint j;
        j.axis = l.axis;
        if (isTreeEdge[e]) {
            j.kind   = PlannedJoint::Spiral;
            j.parent = (out.parentOf[l.b] == l.a) ? l.a : l.b;
            j.child  = (j.parent == l.a) ? l.b : l.a;
            spiralAxisOf[j.child] = l.axis;
        }
        else {
            j.kind   = PlannedJoint::Dovetail;
            j.parent = l.a;
            j.child  = l.b;
        }
        out.joints.append(j);
    }

    QVector<bool> locked(n, false);
    for (const PlannedJoint &j : out.joints) {
        if (j.kind != PlannedJoint::Dovetail)
            continue;
        for (int side = 0; side < 2; ++side) {
            const int piece = side ? j.child : j.parent;
            if (spiralAxisOf[piece] >= 0 && j.axis != spiralAxisOf[piece])
                locked[piece] = true;
        }
    }
    for (int i = 0; i < n; ++i)
        if (i != root && spiralAxisOf[i] >= 0 && !locked[i])
            out.problems << QStringLiteral(
                "Piece %1 has no dovetail across its spiral axis - nothing stops "
                "it unwinding.").arg(i);

    double reach = 0.0;
    for (int a = 0; a < 3; ++a) {
        double lo = 1e300, hi = -1e300;
        for (const PuzzlePiece &p : pieces) { lo = qMin(lo, p.p0[a]); hi = qMax(hi, p.p1[a]); }
        reach = qMax(reach, (hi - lo) * 2.0);
    }

    QVector<int> placed;
    placed.reserve(n);
    placed.append(out.order.isEmpty() ? 0 : out.order.first());

    for (int k = 1; k < out.order.size(); ++k) {
        const int c      = out.order[k];
        const int parent = out.parentOf[c];
        if (parent < 0)
            continue;

        const Box cb = boxOf(pieces[c]);
        const Box pb = boxOf(pieces[parent]);
        const int axis = spiralAxisOf[c];
        if (axis < 0) {
            placed.append(c);
            continue;
        }

        const bool fromPositive = cb.lo[axis] >= pb.hi[axis] - params.clearance;

        const int u = (axis + 1) % 3, v = (axis + 2) % 3;
        const double cu = 0.5 * (qMax(cb.lo[u], pb.lo[u]) + qMin(cb.hi[u], pb.hi[u]));
        const double cv = 0.5 * (qMax(cb.lo[v], pb.lo[v]) + qMin(cb.hi[v], pb.hi[v]));

        const Box corridor = approachCorridor(cb, axis, fromPositive, reach);
        const Box sweep    = rotationSweep(cb, axis, cu, cv,
                                           params.seatingRotationDeg,
                                           params.rotationSamples);

        for (int q : placed) {
            if (q == parent)
                continue;
            const Box qb = boxOf(pieces[q]);
            if (corridor.overlaps(qb, params.clearance))
                out.problems << QStringLiteral(
                    "Piece %1 cannot reach its place: piece %2 is in the approach "
                    "path along %3.").arg(c).arg(q).arg(QChar("xyz"[axis]));
            else if (sweep.overlaps(qb, params.clearance))
                out.problems << QStringLiteral(
                    "Piece %1 cannot turn to seat: piece %2 is inside the %3 deg "
                    "rotation sweep about %4.")
                    .arg(c).arg(q)
                    .arg(params.seatingRotationDeg, 0, 'g', 3)
                    .arg(QChar("xyz"[axis]));
        }
        placed.append(c);
    }

    return out;
}

QStringList AssemblyPlanner::describe(const AssemblyPlan &plan,
                                      const QVector<PuzzlePiece> &pieces)
{
    QStringList out;
    out << QStringLiteral("pieces %1 · spirals %2 · dovetails %3")
               .arg(pieces.size()).arg(plan.spiralCount()).arg(plan.dovetailCount());

    if (plan.valid()) {
        out << QStringLiteral("plan is VALID - every piece reachable, turnable "
                              "and locked");
    }
    else {
        out << QStringLiteral("plan is INVALID - %1 problem(s):")
                   .arg(plan.problems.size());
        for (int i = 0; i < qMin(8, int(plan.problems.size())); ++i)
            out << QStringLiteral("   %1").arg(plan.problems[i]);
        if (plan.problems.size() > 8)
            out << QStringLiteral("   ... and %1 more")
                       .arg(plan.problems.size() - 8);
    }
    return out;
}
