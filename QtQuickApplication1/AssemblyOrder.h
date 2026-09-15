#pragma once
//
// AssemblyOrder - can these pieces be put together, and in what order?
//
// Three steps, in this order:
//   1. ADJACENCY   which pieces share a real face, and on which axis
//   2. BLOCKING    per piece, which of the six axis directions are free
//   3. PEELING     repeatedly take off a piece that has a free direction
//
// Reaching the end means ASSEMBLABLE and the removal order reversed is the
// assembly sequence. Getting stuck with pieces left means a cyclic deadlock,
// and the pieces still present are named.
//
// WHAT THIS DOES NOT ESTABLISH
// ---------------------------------------------------------------------------
// The blocking test is STRAIGHT-LINE ONLY, along the six axis directions. It
// knows nothing about rotation, nothing about swept volume, and nothing about a
// piece that is in the way without touching. A result here is "assemblable
// under translational blocking; rotational/swept check pending" and must never
// be written down as collision-free. Every string this module produces carries
// that qualifier, deliberately.
//
// A SECOND APPROXIMATION, WORTH KNOWING ABOUT
// ---------------------------------------------------------------------------
// Blocking is computed on the pieces' CELL BOXES - the axis-aligned regions the
// divider cut - not on the trimmed geometry. For an axis-aligned model (a cube
// cut by planes) the two agree exactly. For a curved model they do not: two
// cells can meet on a plane where the trimmed pieces never touch, because the
// Boolean pulled both back from that face. Such a contact is counted as
// blocking when physically it is not, so the answer errs towards "stuck" rather
// than towards a false "assemblable".
//
// Rather than leave that as a disclaimer, `phantomContacts` counts them: for
// every contact it checks whether both pieces actually have material at the
// shared face. Verify on the cube first, where that count must be zero.
//
// This module owns the question and the reporting. The three algorithms live in
// Planner (PlannerGraph / PlannerBlocking / PlannerOrder) and are called, not
// re-implemented - a second copy would drift from the one the joint code uses.
// It touches neither the divider nor any joint code.
//
#include "PuzzleDivider.h"

#include <QString>
#include <QStringList>
#include <QVector>

namespace Planner { class BlockingModel; }

class AssemblyOrder {
public:
    struct Step {
        int piece = -1;
        int dir   = -1;        // 0..5 = +X -X +Y -Y +Z -Z
    };

    struct Result {
        bool          assemblable = false;
        QVector<Step> removal;       // first piece off first
        QVector<Step> assembly;      // the reverse, directions flipped
        QVector<int>  stuck;         // present when it gave up; empty if solved

        int contacts        = 0;
        int phantomContacts = 0;     // cells meet, material does not
        int minNeighbours   = 0;
        int maxNeighbours   = 0;

        QStringList adjacencyLog;
        QStringList blockingLog;
        QStringList resultLog;

        QStringList describe() const;   // all three, in order
    };

    // `tol` is the gap below which two cell faces count as touching. It is in
    // the units of PuzzlePiece::p0/p1, which is the trivariate's PARAMETER
    // domain on the cage path and world units on the mesh path - so a value
    // that suits one path may not suit the other.
    static Result run(const QVector<PuzzlePiece> &pieces, double tol = 1e-4);

    // The same, with the blocking rule swapped out. This is the seam: when a
    // swept-volume or rotational test exists, it arrives here and nothing else
    // in this file changes.
    static Result run(const QVector<PuzzlePiece> &pieces,
                      const Planner::BlockingModel &model, double tol = 1e-4);

    // Printed with every verdict so a result cannot be quoted without it.
    static QString caveat();
};
