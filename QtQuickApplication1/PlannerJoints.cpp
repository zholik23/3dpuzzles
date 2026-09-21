//
// PlannerJoints - implementation: the jointed blocking model, and the choice of
// which contacts may carry a joint without breaking the removal order.
//

#include "PlannerJoints.h"

namespace Planner {

JointedBlocking::JointedBlocking(const JointSet &joints)
    : m_joints(joints)
{
}

QString JointedBlocking::name() const
{
    return QStringLiteral("jointed translational DBG (placeholder)");
}

QString JointedBlocking::caveat() const
{
    return QStringLiteral("pegs constrain to their own axis; straight-line "
                          "withdrawal only, no rotation, no swept volume - "
                          "real collision check pending");
}

// Starts from all six directions allowed and takes them away. A mated peg
// permits only straight back out along its own axis.
DirMask JointedBlocking::blocked(const Graph &g, int piece,
                                 const QVector<bool> &present) const
{
    DirMask m = 0;
    if (piece < 0 || piece >= g.incident.size())
        return m;

    DirMask allowed = 0x3Fu;

    for (int ci : g.incident[piece]) {
        const int other = g.otherSide(ci, piece);
        if (other < 0 || other >= present.size() || !present[other])
            continue;

        const int toward = g.directionFrom(ci, piece);
        m = withBlocked(m, toward);

        if (ci < m_joints.size() && m_joints[ci]) {
            allowed &= (1u << dirOpposite(toward));
        }
    }

    return (m | (~allowed)) & 0x3Fu;
}

QVector<int> JointedBlocking::blockers(const Graph &g, int piece, int dir,
                                       const QVector<bool> &present) const
{
    QVector<int> out;
    if (piece < 0 || piece >= g.incident.size())
        return out;

    for (int ci : g.incident[piece]) {
        const int other = g.otherSide(ci, piece);
        if (other < 0 || other >= present.size() || !present[other])
            continue;

        const int toward = g.directionFrom(ci, piece);
        const bool jointed = (ci < m_joints.size() && m_joints[ci]);

        if (toward == dir || (jointed && dir != dirOpposite(toward)))
            if (!out.contains(other))
                out.append(other);
    }
    return out;
}

JointSet allContacts(const Graph &g)
{
    return JointSet(g.contactCount(), true);
}

// Picks which contacts may carry a joint without breaking the removal order: the
// piece that leaves first is the one that has to pull a peg.
JointSet chooseAlongOrder(const Graph &g, const Plan &plan)
{
    JointSet joints(g.contactCount(), false);
    if (!plan.complete)
        return joints;

    QVector<int> step(g.pieceCount(), -1);
    QVector<int> exitDir(g.pieceCount(), -1);
    for (int i = 0; i < plan.removal.size(); ++i) {
        const Step &s = plan.removal[i];
        if (s.piece >= 0 && s.piece < step.size()) {
            step[s.piece]    = i;
            exitDir[s.piece] = s.dir;
        }
    }

    for (int ci = 0; ci < g.contactCount(); ++ci) {
        const Contact &c = g.contacts[ci];
        if (step[c.lowSide] < 0 || step[c.highSide] < 0)
            continue;

        const int first = (step[c.lowSide] < step[c.highSide]) ? c.lowSide
                                                               : c.highSide;
        const int away  = dirOpposite(g.directionFrom(ci, first));
        if (exitDir[first] == away)
            joints[ci] = true;
    }
    return joints;
}

int countJoints(const JointSet &joints)
{
    int n = 0;
    for (bool b : joints)
        if (b)
            ++n;
    return n;
}

QStringList describeJoints(const Graph &g, const JointSet &joints, int maxPieces)
{
    QStringList out;
    out << QStringLiteral("STAGE 4b joints on %1 of %2 contacts")
               .arg(countJoints(joints)).arg(g.contactCount());

    const int n = (maxPieces < 0) ? g.pieceCount()
                                  : qMin(maxPieces, g.pieceCount());
    for (int i = 0; i < n; ++i) {
        QStringList bits;
        for (int ci : g.incident[i]) {
            if (ci >= joints.size() || !joints[ci])
                continue;
            bits << QStringLiteral("%1 with %2")
                        .arg(QString::fromLatin1(dirName(g.directionFrom(ci, i))))
                        .arg(g.otherSide(ci, i));
        }
        out << QStringLiteral("  piece %1: %2")
                   .arg(i, 3)
                   .arg(bits.isEmpty() ? QStringLiteral("(no joints)")
                                       : bits.join(QStringLiteral(", ")));
    }
    if (n < g.pieceCount())
        out << QStringLiteral("  ... and %1 more").arg(g.pieceCount() - n);
    return out;
}

QStringList explainOverConstrained(const Graph &g, const JointSet &joints,
                                   int maxPieces)
{
    QStringList out;
    const int cap = (maxPieces < 0) ? (1 << 30) : maxPieces;
    int shown = 0, total = 0;

    for (int i = 0; i < g.pieceCount(); ++i) {
        QVector<int> axes;
        QStringList  detail;
        for (int ci : g.incident[i]) {
            if (ci >= joints.size() || !joints[ci])
                continue;
            const int d = g.directionFrom(ci, i);
            if (!axes.contains(dirAxis(d)))
                axes.append(dirAxis(d));
            detail << QStringLiteral("%1 with %2")
                          .arg(QString::fromLatin1(dirName(d)))
                          .arg(g.otherSide(ci, i));
        }
        if (axes.size() < 2)
            continue;

        ++total;
        if (shown < cap) {
            out << QStringLiteral("  piece %1 has pegs on %2 axes: %3")
                       .arg(i, 3).arg(axes.size())
                       .arg(detail.join(QStringLiteral(", ")));
            ++shown;
        }
    }

    if (total == 0)
        out << QStringLiteral("  no piece carries pegs on more than one axis");
    else
        out.prepend(QStringLiteral("  %1 piece(s) carry pegs on more than one "
                                   "axis and cannot translate out at all:")
                        .arg(total));
    if (total > shown)
        out << QStringLiteral("  ... and %1 more").arg(total - shown);
    return out;
}

// ----------------------------------------------------------- dovetails ----

DovetailBlocking::DovetailBlocking(const JointSet &joints,
                                   const QVector<int> &slideAxis)
    : m_joints(joints), m_slide(slideAxis)
{
}

QString DovetailBlocking::name() const
{
    return QStringLiteral("dovetailed translational DBG");
}

QString DovetailBlocking::caveat() const
{
    return QStringLiteral("a mated dovetail slides along its own axis only and "
                          "blocks withdrawal across the face; straight-line "
                          "motion, no rotation, no swept volume - real "
                          "collision check pending");
}

// Start from all six allowed and take them away. A mated dovetail leaves only
// the two directions along its slide axis; two dovetails on different axes
// leave none, which is why the chooser below refuses to create that case.
DirMask DovetailBlocking::blocked(const Graph &g, int piece,
                                  const QVector<bool> &present) const
{
    DirMask m = 0;
    if (piece < 0 || piece >= g.incident.size())
        return m;

    DirMask allowed = 0x3Fu;

    for (int ci : g.incident[piece]) {
        const int other = g.otherSide(ci, piece);
        if (other < 0 || other >= present.size() || !present[other])
            continue;

        m = withBlocked(m, g.directionFrom(ci, piece));

        if (ci < m_joints.size() && m_joints[ci] &&
            ci < m_slide.size() && m_slide[ci] >= 0) {
            const int a = m_slide[ci];
            allowed &= (1u << (a * 2)) | (1u << (a * 2 + 1));
        }
    }

    return (m | (~allowed)) & 0x3Fu;
}

QVector<int> DovetailBlocking::blockers(const Graph &g, int piece, int dir,
                                        const QVector<bool> &present) const
{
    QVector<int> out;
    if (piece < 0 || piece >= g.incident.size())
        return out;

    for (int ci : g.incident[piece]) {
        const int other = g.otherSide(ci, piece);
        if (other < 0 || other >= present.size() || !present[other])
            continue;

        const int  toward = g.directionFrom(ci, piece);
        const bool dove   = (ci < m_joints.size() && m_joints[ci] &&
                             ci < m_slide.size() && m_slide[ci] >= 0);

        if (toward == dir || (dove && dirAxis(dir) != m_slide[ci]))
            if (!out.contains(other))
                out.append(other);
    }
    return out;
}

// A dovetail may sit on a contact only when the piece that leaves first slides
// out PARALLEL to that face. Put one on the face a piece pulls away from and it
// can never leave - the exact opposite of where a peg belongs.
//
// One piece also cannot carry dovetails on two different slide axes: the second
// would pin it solid. The first axis a piece is given wins, and later contacts
// that disagree are left plain. That is what keeps "a dovetail everywhere"
// from welding the puzzle shut.
JointSet chooseDovetailsAlongOrder(const Graph &g, const Plan &plan,
                                   QVector<int> *slideAxis)
{
    JointSet     joints(g.contactCount(), false);
    QVector<int> slide(g.contactCount(), -1);

    if (!plan.complete) {
        if (slideAxis != nullptr)
            *slideAxis = slide;
        return joints;
    }

    QVector<int> step(g.pieceCount(), -1);
    QVector<int> exitDir(g.pieceCount(), -1);
    for (int i = 0; i < plan.removal.size(); ++i) {
        const Step &s = plan.removal[i];
        if (s.piece >= 0 && s.piece < step.size()) {
            step[s.piece]    = i;
            exitDir[s.piece] = s.dir;
        }
    }

    QVector<int> pieceSlide(g.pieceCount(), -1);

    for (int ci = 0; ci < g.contactCount(); ++ci) {
        const Contact &c = g.contacts[ci];
        if (c.lowSide < 0 || c.highSide < 0)
            continue;
        if (step[c.lowSide] < 0 || step[c.highSide] < 0)
            continue;

        const int first = (step[c.lowSide] < step[c.highSide]) ? c.lowSide
                                                               : c.highSide;
        const int e = exitDir[first];
        if (e < 0)
            continue;

        const int a = dirAxis(e);
        if (a == c.axis)
            continue;                  // would block the withdrawal itself

        if (pieceSlide[first] >= 0 && pieceSlide[first] != a)
            continue;                  // that piece is already committed

        joints[ci]        = true;
        slide[ci]         = a;
        pieceSlide[first] = a;
    }

    if (slideAxis != nullptr)
        *slideAxis = slide;
    return joints;
}

}
