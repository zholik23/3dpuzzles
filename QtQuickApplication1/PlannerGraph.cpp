#include "PlannerGraph.h"

#include <cmath>

namespace Planner {

const char *dirName(int d)
{
    static const char *kNames[DirectionCount] = { "+X", "-X", "+Y", "-Y", "+Z", "-Z" };
    return (d >= 0 && d < DirectionCount) ? kNames[d] : "??";
}

int Graph::otherSide(int contactIndex, int piece) const
{
    if (contactIndex < 0 || contactIndex >= contacts.size())
        return -1;
    const Contact &c = contacts[contactIndex];
    if (piece == c.lowSide)  return c.highSide;
    if (piece == c.highSide) return c.lowSide;
    return -1;
}

int Graph::directionFrom(int contactIndex, int piece) const
{
    if (contactIndex < 0 || contactIndex >= contacts.size())
        return -1;
    const Contact &c = contacts[contactIndex];
    // The low-side piece has its neighbour in the +axis direction, and the
    // high-side piece has it in -axis.
    if (piece == c.lowSide)  return c.axis * 2 + 0;
    if (piece == c.highSide) return c.axis * 2 + 1;
    return -1;
}

Graph build(const QVector<PuzzlePiece> &pieces, double eps, double minArea)
{
    Graph g;
    g.nodes.reserve(pieces.size());
    for (int i = 0; i < pieces.size(); ++i) {
        Node n;
        n.id = i;
        for (int a = 0; a < 3; ++a) {
            n.lo[a] = pieces[i].p0[a];
            n.hi[a] = pieces[i].p1[a];
        }
        g.nodes.append(n);
    }
    g.incident.resize(pieces.size());

    // The pairing is the divider's. Re-deriving it here would be a second
    // opinion on the same question, and the two could drift apart.
    const QVector<PuzzleDivider::Neighbours> links =
        PuzzleDivider::adjacencyOfBoxes(pieces, eps);

    for (const PuzzleDivider::Neighbours &l : links) {
        if (l.a < 0 || l.b < 0 || l.a >= pieces.size() || l.b >= pieces.size())
            continue;
        const int axis = l.axis;
        if (axis < 0 || axis > 2)
            continue;

        // Which one is on the low side of the shared plane. Comparing the far
        // faces rather than the near ones: two boxes that meet on a plane have
        // one's hi equal to the other's lo, and it is that pairing that names
        // the sides.
        int lo = l.a, hi = l.b;
        if (std::fabs(pieces[hi].p1[axis] - pieces[lo].p0[axis]) <= eps)
            qSwap(lo, hi);

        const PuzzlePiece &A = pieces[lo], &B = pieces[hi];

        // The contact rectangle, on the two axes that are not the normal.
        double extent[2] = { 0.0, 0.0 };
        int    k = 0;
        for (int b = 0; b < 3; ++b) {
            if (b == axis)
                continue;
            const double l0 = qMax(A.p0[b], B.p0[b]);
            const double l1 = qMin(A.p1[b], B.p1[b]);
            extent[k++] = qMax(0.0, l1 - l0);
        }
        const double area = extent[0] * extent[1];
        if (area <= minArea)
            continue;               // a sliver, not a face worth planning on

        Contact c;
        c.lowSide  = lo;
        c.highSide = hi;
        c.axis     = axis;
        c.plane    = 0.5 * (A.p1[axis] + B.p0[axis]);
        c.area     = area;
        c.ext[0]   = extent[0];
        c.ext[1]   = extent[1];
        c.depth    = qMin(A.p1[axis] - A.p0[axis], B.p1[axis] - B.p0[axis]);

        const int idx = g.contacts.size();
        g.contacts.append(c);
        g.incident[lo].append(idx);
        g.incident[hi].append(idx);
    }
    return g;
}

QStringList Graph::describe(int maxPieces) const
{
    QStringList out;
    out << QStringLiteral("STAGE 1  adjacency graph: %1 pieces, %2 face contacts")
               .arg(pieceCount()).arg(contactCount());

    int valMin = 1 << 30, valMax = 0;
    for (int i = 0; i < incident.size(); ++i) {
        valMin = qMin(valMin, int(incident[i].size()));
        valMax = qMax(valMax, int(incident[i].size()));
    }
    if (!incident.isEmpty())
        out << QStringLiteral("         neighbours per piece: min %1, max %2")
                   .arg(valMin).arg(valMax);

    const int n = (maxPieces < 0) ? pieceCount() : qMin(maxPieces, pieceCount());
    for (int i = 0; i < n; ++i) {
        QStringList bits;
        for (int ci : incident[i]) {
            const int d = directionFrom(ci, i);
            bits << QStringLiteral("%1 %2 (area %3)")
                        .arg(QString::fromLatin1(dirName(d)))
                        .arg(otherSide(ci, i))
                        .arg(contacts[ci].area, 0, 'g', 3);
        }
        out << QStringLiteral("  piece %1: %2")
                   .arg(i, 3)
                   .arg(bits.isEmpty() ? QStringLiteral("(no neighbours)")
                                       : bits.join(QStringLiteral(", ")));
    }
    if (n < pieceCount())
        out << QStringLiteral("  ... and %1 more").arg(pieceCount() - n);
    return out;
}

} // namespace Planner
