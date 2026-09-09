#pragma once
//
// MeshDivider - divides a loaded mesh into pieces by clipping it against a
// non-uniform grid of world-space cells.
//
// Why this exists alongside PuzzleDivider:
//
//   PuzzleDivider is Elber's method. It divides the parameter domain of a
//   trivariate, so every piece is a real solid sub-trivariate. That is what
//   the joints and the assembly planner will eventually need - but it requires
//   a V-rep, and an STL/OBJ has no interior.
//
//   MeshDivider works on the mesh directly. Each piece keeps the model's true
//   shape - a torus stays a torus, a lattice stays a lattice - which the
//   bounding-cage placeholder could never do, because a box over the extent
//   throws the shape away.
//
// The trade-off is honest and worth knowing: clipping a surface mesh gives
// surface pieces. Where a cut passes through solid material this leaves an
// open hole, which is capped when the cut outline is a single loop. A cut that
// produces nested outlines (through a hollow wall, or a lattice strut with an
// interior void) is left open rather than capped wrongly, and reported.
//
#include "CutWarp.h"
#include "MeshData.h"
#include "PuzzleDivider.h"      // for PuzzlePiece

#include <QString>
#include <QVector>

struct MeshDivisionSpec {
    QVector<double> planes[3];      // interior cut positions, world coordinates
    QString         note;
    bool            capped = false; // hit the per-axis cell limit

    int cells(int axis) const { return planes[axis].size() + 1; }
    int cellCount()     const { return cells(0) * cells(1) * cells(2); }
};

class MeshDivider {
public:
    // Equal slabs along each axis.
    static MeshDivisionSpec uniform(const MeshData &mesh, const int counts[3]);

    // Equal slabs, then each cut nudged by up to `jitter` of a slab width -
    // Elber's difficulty randomisation, so no two pieces are interchangeable.
    static MeshDivisionSpec jittered(const MeshData &mesh, const int counts[3],
                                     double jitter, quint32 seed);

    // Cuts placed so no cell exceeds `budget` on any axis, spaced EVENLY.
    //
    // Evenly is not a shortcut - it is the whole answer. In world space a size
    // limit is satisfied by n = ceil(extent / limit) equal slabs, so there is
    // nothing for the spacing to be non-uniform ABOUT. Unequal cuts on a mesh
    // have to come from somewhere else: either imposed (jittered), or derived
    // from a quantity that genuinely varies along the model, which surface area
    // is not - a coarse box has all its area at a handful of face centres.
    static MeshDivisionSpec toBuildVolume(const MeshData &mesh,
                                          const double budget[3],
                                          int maxCellsPerAxis);

    // Cuts placed so each slab holds roughly the same triangle area. On a model
    // whose detail is unevenly distributed - a lattice, a part with one dense
    // end - this is the division that actually balances the pieces, and it is
    // the case uniform slabs handle worst.
    static MeshDivisionSpec balanced(const MeshData &mesh, const int counts[3]);

    // `cutDetail` is the longest edge a cut face is allowed to keep. Cut faces
    // are subdivided below it so that un-warping bends them into a smooth
    // curve rather than one flat facet. Pass 0 to skip subdivision entirely,
    // which is what flat cuts want.
    static bool divide(const MeshData &mesh, const MeshDivisionSpec &spec,
                       QVector<PuzzlePiece> *pieces, QString *report,
                       double cutDetail = 0.0);

    // Clips the model into an arbitrary list of boxes rather than a grid. This
    // is what lets a recursive (BSP) split be used: the cells no longer line up
    // in rows and columns, so there are no per-axis plane lists to describe
    // them, but each one is still a box and clips exactly the same way.
    static bool divideCells(const MeshData &mesh, const QVector<CellBox> &cells,
                            QVector<PuzzlePiece> *pieces, QString *report,
                            double cutDetail = 0.0);

    // Recursive split + clip, with crumbs absorbed.
    //
    // The splitter's minimum bounds the CELL, which is not the same as bounding
    // the PIECE: on an organic model a full-size cell can still clip down to a
    // crumb at a claw tip or the end of a tail. So this clips, measures what
    // came out, and collapses the parent of anything far smaller than the
    // median piece - a cell and its BSP sibling merge back into the parent box
    // exactly, so the division stays a partition of boxes throughout.
    static bool divideBspAbsorbing(const MeshData &mesh, int targetPieces,
                                   double jitter, quint32 seed,
                                   QVector<PuzzlePiece> *pieces,
                                   QVector<CellBox> *cells,
                                   int *absorbed, QString *report);

    // Curved cuts, done by warping the model, cutting it flat, and un-warping
    // the pieces. See CutWarp for why the map is always invertible.
    static MeshData warp(const MeshData &mesh, const CutWarp &w);
    static void     unwarp(QVector<PuzzlePiece> *pieces, const CutWarp &w,
                           const float bmin[3], const float bmax[3],
                           double diagonal);
};
