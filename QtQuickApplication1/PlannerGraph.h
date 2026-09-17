#pragma once
//
// Planner stage 1 - the piece adjacency graph.
//
// It does not re-derive the pairing: PuzzleDivider::adjacencyOfBoxes already
// finds boxes that meet on a plane with real overlap and rejects edge-only and
// corner-only touches. This adds what the later stages need - which side of the
// shared plane each piece is on, where that plane lies, and how large the contact
// is.
//
// Nothing here touches IRIT, so the planner can be tested without a geometry
// kernel in the loop.
//
#include "PuzzleDivider.h"

#include <QList>
#include <QString>
#include <QStringList>
#include <QVector>

namespace Planner {

enum Direction { PosX = 0, NegX, PosY, NegY, PosZ, NegZ, DirectionCount };

inline int dirAxis(int d)     { return d >> 1; }
inline int dirSign(int d)     { return (d & 1) ? -1 : +1; }
inline int dirOpposite(int d) { return d ^ 1; }

const char *dirName(int d);

struct Node {
    int    id    = -1;
    double lo[3] = { 0, 0, 0 };
    double hi[3] = { 0, 0, 0 };
};

struct Contact {
    int    lowSide  = -1;
    int    highSide = -1;
    int    axis     = 0;
    double plane    = 0.0;
    double area     = 0.0;
    double ext[2]   = { 0, 0 };
    double depth    = 0.0;
};

class Graph {
public:
    QVector<Node>          nodes;
    QVector<Contact>       contacts;
    QVector<QVector<int> > incident;

    int pieceCount()   const { return nodes.size(); }
    int contactCount() const { return contacts.size(); }

    int directionFrom(int contactIndex, int piece) const;
    int otherSide(int contactIndex, int piece) const;

    QStringList describe(int maxPieces = -1) const;
};

Graph build(const QVector<PuzzlePiece> &pieces, double eps = 1e-6,
            double minArea = 0.0);

}
