#pragma once
//
// Planner stage 1 - the piece adjacency graph.
//
// Consumes whatever the divider produced. It does NOT re-derive the pairing:
// PuzzleDivider::adjacencyOfBoxes already finds boxes that meet on a plane with
// a real overlap there, and already rejects edge-only and corner-only touches.
// This adds what the later stages need and it does not: which side of the
// shared plane each piece is on, where that plane is, and how big the contact
// actually is.
//
// Nothing here touches IRIT. The planner works on the pieces' boxes, so it has
// no kernel dependency at all - which is worth keeping, because it means the
// planner can be tested without a geometry kernel in the loop.
//
#include "PuzzleDivider.h"

#include <QList>
#include <QString>
#include <QStringList>
#include <QVector>

namespace Planner {

// The six axis directions, in one fixed order the whole planner shares.
enum Direction { PosX = 0, NegX, PosY, NegY, PosZ, NegZ, DirectionCount };

inline int dirAxis(int d)     { return d >> 1; }             // 0 = x, 1 = y, 2 = z
inline int dirSign(int d)     { return (d & 1) ? -1 : +1; }
inline int dirOpposite(int d) { return d ^ 1; }

const char *dirName(int d);

struct Node {
    int    id    = -1;
    double lo[3] = { 0, 0, 0 };
    double hi[3] = { 0, 0, 0 };
};

// One real face contact.
//
// `lowSide` is the piece on the smaller side of the shared plane. So from
// lowSide the neighbour lies along +axis, and from highSide along -axis. That
// orientation is the whole content of the edge as far as stage 2 is concerned.
struct Contact {
    int    lowSide  = -1;
    int    highSide = -1;
    int    axis     = 0;      // 0/1/2 - normal of the shared face
    double plane    = 0.0;    // where the shared face sits along `axis`
    double area     = 0.0;    // overlap area on that plane
    double ext[2]   = { 0, 0 };  // its two side lengths, on the non-normal axes
    double depth    = 0.0;    // thinnest of the two pieces along `axis`
};

class Graph {
public:
    QVector<Node>          nodes;
    QVector<Contact>       contacts;
    QVector<QVector<int> > incident;    // per node: indices into `contacts`

    int pieceCount()   const { return nodes.size(); }
    int contactCount() const { return contacts.size(); }

    // Which way the neighbour lies, seen from `piece`.
    int directionFrom(int contactIndex, int piece) const;
    // The piece on the other end of the contact.
    int otherSide(int contactIndex, int piece) const;

    // Stage 1 log: every piece, its neighbours, and the direction each is in.
    QStringList describe(int maxPieces = -1) const;
};

// `minArea` drops contacts whose overlap is real but negligible - a sliver left
// by a BSP cut that would otherwise be treated as a full face. Zero keeps
// everything adjacencyOfBoxes returned.
Graph build(const QVector<PuzzlePiece> &pieces, double eps = 1e-6,
            double minArea = 0.0);

} // namespace Planner
