#pragma once
//
// Planner stage 4 - joints that follow the plan instead of fighting it.
//
// A face contact only stops a piece leaving through that face; a mated peg
// leaves it exactly one direction, and pegs on two axes leave none. So a pin on
// every shared face is a welded block, not a puzzle - the order is decided
// first, and the joints placed to suit it.
//
// Still translational: no rotation, no swept volume.
//
#include "PlannerOrder.h"

namespace Planner {

typedef QVector<bool> JointSet;

class JointedBlocking : public BlockingModel {
public:
    explicit JointedBlocking(const JointSet &joints);

    QString name()   const override;
    QString caveat() const override;
    DirMask blocked(const Graph &g, int piece,
                    const QVector<bool> &present) const override;
    QVector<int> blockers(const Graph &g, int piece, int dir,
                          const QVector<bool> &present) const override;

private:
    JointSet m_joints;
};

JointSet allContacts(const Graph &g);

JointSet chooseAlongOrder(const Graph &g, const Plan &plan);

// A dovetail is the opposite of a peg: a peg allows only straight back out
// along the face normal, a mated dovetail forbids exactly that and allows only
// sliding in the contact plane. So a dovetail belongs on the faces a piece
// slides PAST, never on the face it pulls away from.
class DovetailBlocking : public BlockingModel {
public:
    DovetailBlocking(const JointSet &joints, const QVector<int> &slideAxis);

    QString name()   const override;
    QString caveat() const override;
    DirMask blocked(const Graph &g, int piece,
                    const QVector<bool> &present) const override;
    QVector<int> blockers(const Graph &g, int piece, int dir,
                          const QVector<bool> &present) const override;

private:
    JointSet     m_joints;
    QVector<int> m_slide;        // per contact: slide axis, or -1
};

// Picks the contacts that may carry a dovetail without breaking the order, and
// the slide axis for each. slideAxis is filled per contact (-1 where none).
JointSet chooseDovetailsAlongOrder(const Graph &g, const Plan &plan,
                                   QVector<int> *slideAxis);

int  countJoints(const JointSet &joints);
QStringList describeJoints(const Graph &g, const JointSet &joints,
                           int maxPieces = 8);

QStringList explainOverConstrained(const Graph &g, const JointSet &joints,
                                   int maxPieces = 8);

}
