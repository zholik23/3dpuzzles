#pragma once
//
// Planner stage 2 - the Directional Blocking Graph.
//
// Placeholder model. A piece is blocked in a direction if a neighbour sits on
// that face, which is exact for straight-line withdrawal of axis-aligned boxes
// (Wilson & Latombe 1994) and blind to rotation, to swept volume, and to pieces
// that are in the way without touching. Replacing it is the point of the
// interface below.
//
// The seam takes the whole present-piece set rather than a pair, because a
// swept-volume test must be free to consult pieces that never touch the one being
// moved.
//
#include "PlannerGraph.h"

namespace Planner {

typedef unsigned DirMask;

inline bool    isBlocked(DirMask m, int d) { return ((m >> d) & 1u) != 0u; }
inline DirMask withBlocked(DirMask m, int d) { return m | (1u << d); }
inline bool    allBlocked(DirMask m) { return (m & 0x3Fu) == 0x3Fu; }

QString maskToString(DirMask m);

class BlockingModel {
public:
    virtual ~BlockingModel();

    virtual QString name() const = 0;
    virtual QString caveat() const = 0;

    virtual DirMask blocked(const Graph &g, int piece,
                            const QVector<bool> &present) const = 0;

    virtual QVector<int> blockers(const Graph &g, int piece, int dir,
                                  const QVector<bool> &present) const = 0;
};

class TranslationalBlocking : public BlockingModel {
public:
    QString name()   const override;
    QString caveat() const override;
    DirMask blocked(const Graph &g, int piece,
                    const QVector<bool> &present) const override;
    QVector<int> blockers(const Graph &g, int piece, int dir,
                          const QVector<bool> &present) const override;
};

class AlwaysBlocking : public BlockingModel {
public:
    QString name()   const override;
    QString caveat() const override;
    DirMask blocked(const Graph &g, int piece,
                    const QVector<bool> &present) const override;
    QVector<int> blockers(const Graph &g, int piece, int dir,
                          const QVector<bool> &present) const override;
};

QStringList describeBlocking(const Graph &g, const BlockingModel &model,
                             int maxPieces = 8);

}
