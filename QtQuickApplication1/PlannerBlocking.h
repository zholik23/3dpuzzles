#pragma once
//
// Planner stage 2 - the Directional Blocking Graph.
//
// PLACEHOLDER MODEL. The blocking test here is purely TRANSLATIONAL: a piece is
// blocked in a direction if a neighbour sits on that face. For axis-aligned
// boxes that is exact for straight-line withdrawal to infinity, and it is the
// standard first model (Wilson & Latombe 1994; Halperin, Latombe & Wilson's
// NDBG). It is NOT the collision test this project needs: it knows nothing
// about rotation, about swept volume, or about pieces that are not in contact
// but are still in the way. Replacing it is the point of the interface below.
//
// The seam is deliberately wider than "does neighbour B block A". It takes the
// whole present-piece set, because a swept-volume test has to be free to
// consult pieces that never touch the piece being moved - a pairwise interface
// would have to be thrown away when that lands.
//
#include "PlannerGraph.h"

namespace Planner {

// A set of directions, one bit per Direction.
typedef unsigned DirMask;

inline bool    isBlocked(DirMask m, int d) { return ((m >> d) & 1u) != 0u; }
inline DirMask withBlocked(DirMask m, int d) { return m | (1u << d); }
inline bool    allBlocked(DirMask m) { return (m & 0x3Fu) == 0x3Fu; }

QString maskToString(DirMask m);

class BlockingModel {
public:
    virtual ~BlockingModel();

    virtual QString name() const = 0;
    // One line stating what this model does not cover. Printed with every plan
    // so a result is never read as stronger than the model behind it.
    virtual QString caveat() const = 0;

    // Directions `piece` cannot move in, given that only `present` pieces are
    // still in the assembly.
    virtual DirMask blocked(const Graph &g, int piece,
                            const QVector<bool> &present) const = 0;

    // For the log: which present pieces are responsible for blocking `dir`.
    virtual QVector<int> blockers(const Graph &g, int piece, int dir,
                                  const QVector<bool> &present) const = 0;
};

// ---------------------------------------------------------------------------
// The placeholder. Straight-line withdrawal, blocked only by face contact.
class TranslationalBlocking : public BlockingModel {
public:
    QString name()   const override;
    QString caveat() const override;
    DirMask blocked(const Graph &g, int piece,
                    const QVector<bool> &present) const override;
    QVector<int> blockers(const Graph &g, int piece, int dir,
                          const QVector<bool> &present) const override;
};

// ---------------------------------------------------------------------------
// TEST ONLY. Blocks every direction of any piece that still has a neighbour, so
// the NON-ASSEMBLABLE path in stage 3 can actually be exercised. The
// translational model above cannot produce that outcome on axis-aligned boxes -
// see the note in PlannerOrder.cpp - so without this the failure branch would
// never be run.
class AlwaysBlocking : public BlockingModel {
public:
    QString name()   const override;
    QString caveat() const override;
    DirMask blocked(const Graph &g, int piece,
                    const QVector<bool> &present) const override;
    QVector<int> blockers(const Graph &g, int piece, int dir,
                          const QVector<bool> &present) const override;
};

// Stage 2 log: for the first `maxPieces` pieces, which directions are blocked
// and by whom, with every piece still present.
QStringList describeBlocking(const Graph &g, const BlockingModel &model,
                             int maxPieces = 8);

} // namespace Planner
