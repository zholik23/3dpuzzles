#pragma once
//
// Planner stage 4 - joints that follow the plan instead of fighting it.
//
// A face contact only stops a piece leaving through that face. A joint is far
// stronger: while mated, a peg leaves the piece exactly one direction, and pegs
// on two axes of the same piece leave none. So a pin on every shared face is not
// a puzzle but a welded block - the order is decided first, and the joints placed
// to suit it.
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

int  countJoints(const JointSet &joints);
QStringList describeJoints(const Graph &g, const JointSet &joints,
                           int maxPieces = 8);

QStringList explainOverConstrained(const Graph &g, const JointSet &joints,
                                   int maxPieces = 8);

}
