#pragma once
//
// Planner stage 4 - joints that follow the plan instead of fighting it.
//
// A face contact only stops a piece leaving THROUGH that face. A JOINT on that
// face is far stronger: a peg in a hole slides along the peg and nowhere else,
// so while the two are mated the piece has exactly one direction available. Put
// pegs on two different axes of the same piece and it cannot be moved at all.
//
// That is why "a pin on every shared face" is not a puzzle - it is a welded
// block. The order has to be decided first, and the joints placed to suit it.
//
// Still translational: no rotation, no swept volume. The real collision check
// replaces the model, not this reasoning.
//
#include "PlannerOrder.h"

namespace Planner {

// Which contacts carry a joint, indexed by contact index in the Graph.
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

// A joint on every contact - what the divider currently produces.
JointSet allContacts(const Graph &g);

// Joints chosen so they cannot break `plan`: a contact carries one only when
// the piece that comes off FIRST leaves straight along that contact's normal,
// away from its mate.
//
// That single rule is enough. A piece has one removal direction, so every joint
// it is still mated to when its turn comes lies on the same axis and the same
// side - pulling the piece out IS pulling those pegs out. Joints on its other
// faces belong to neighbours that have already gone, and a joint whose mate is
// absent constrains nothing.
JointSet chooseAlongOrder(const Graph &g, const Plan &plan);

int  countJoints(const JointSet &joints);
QStringList describeJoints(const Graph &g, const JointSet &joints,
                           int maxPieces = 8);

// Why a jointed puzzle is stuck, in terms a person can act on: which pieces
// carry mated joints on more than one axis.
QStringList explainOverConstrained(const Graph &g, const JointSet &joints,
                                   int maxPieces = 8);

} // namespace Planner
