#pragma once
//
// AssemblyDivider - split a solid into ~N pieces that are verified to come
// apart, rather than merely divided.
//
// Three stages, deliberately separable so each can be tested on its own:
//
//   Stage A  split to the target count and work out who touches whom
//   Stage B  decide whether the result actually comes apart  (not built yet)
//   Stage C  repair the cuts when it does not                (not built yet)
//
// This module owns none of the geometry. The recursive split is
// PuzzleDivider::buildBspCells, the clipping is MeshDivider::divideCells and the
// face detection is PuzzleDivider::adjacencyOfBoxes - all reused unchanged.
// What this adds is the orchestration and, for Stage A, the one thing the
// existing adjacency does not give: the SIGNED direction of each contact.
//
// adjacencyOfBoxes reports that two pieces meet across the X axis. Stage B needs
// to know which of them is on the other's +X side, because that is exactly what
// makes +X a blocked direction. So Stage A resolves the sign and passes it on.
//
#include "MeshData.h"
#include "PuzzleDivider.h"

#include <QString>
#include <QStringList>
#include <QVector>

// One face contact, seen from the piece that owns it.
struct Contact {
    int    neighbour = -1;
    int    dir       = 0;     // 0..5 = -X +X -Y +Y -Z +Z, pointing at the neighbour
    double area      = 0.0;   // shared face area, so slivers can be told from
                              // real contact later

    static int  axisOf(int dir)       { return dir >> 1; }
    static bool isPositive(int dir)   { return (dir & 1) != 0; }
    static int  opposite(int dir)     { return dir ^ 1; }
    static const char *name(int dir);
};

struct DividedSolid {
    QVector<CellBox>          cells;     // every cell the split produced
    QVector<PuzzlePiece>      pieces;    // the ones that actually hold material
    QVector<QVector<Contact>> contacts;  // per piece, indexed alongside `pieces`

    int requestedPieces = 0;
    int emptyCells      = 0;   // cells the model never reaches
    int openPieces      = 0;   // pieces that came out as non-watertight shells
    int absorbed        = 0;   // crumbs merged back into a neighbour

    QString clipReport;        // whatever MeshDivider had to say

    double solidMin[3] = { 0, 0, 0 };   // the original solid's extent, and
    double solidMax[3] = { 0, 0, 0 };   // the union of the piece boxes, so the
    double unionMin[3] = { 0, 0, 0 };   // two can be compared - they must match
    double unionMax[3] = { 0, 0, 0 };   // or the outer shape has been altered

    int pieceCount()   const { return int(pieces.size()); }
    int contactCount() const;  // shared faces, counted once each
};

class AssemblyDivider {
public:
    // ---- STAGE A ---------------------------------------------------------
    // Splits `solid` into about `targetPieces` and resolves the contact graph.
    //
    // The count is a TARGET. Cells that fall outside the model hold no material
    // and are dropped, so the piece count can come out below the target; the
    // result records how many that was.
    //
    // Selection of which cell to split is volume-weighted rather than strictly
    // largest-first - that is buildBspCells' existing behaviour and it is
    // deliberate, since always taking the largest gives one canonical division
    // with no seed to vary, and Stage C's repair needs alternatives to try.
    static bool splitToTarget(const MeshData &solid, int targetPieces,
                              quint32 seed, DividedSolid *out, QString *error);

    static QStringList describeStageA(const DividedSolid &d, int maxPiecesShown = 16);
};
