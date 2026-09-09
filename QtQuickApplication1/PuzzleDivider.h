#pragma once
//
// PuzzleDivider - cuts a trivariate's parameter domain into cuboid cells and
// extracts each cell as its own sub-trivariate. One cell = one puzzle piece.
//
// This is Elber's construction minus the joints: divide the domain of M into a
// grid, region-extract each sub-domain, and let the composition through M give
// freeform pieces that reassemble into the original solid.
//
// The division is non-uniform by construction: a DivisionSpec is just a sorted
// list of interior split parameters per axis, so cells need not be equal.
// Uniform is the special case where those splits are evenly spaced.
//
#include "MeshData.h"
#include "Trivariate.h"

#include <QRandomGenerator>
#include <QString>
#include <QVector>

struct PuzzlePiece {
    int      i = 0, j = 0, k = 0;      // cell index within the grid
    double   p0[3] = { 0, 0, 0 };      // parameter box corner (u, v, w)
    double   p1[3] = { 0, 0, 0 };      // opposite parameter corner
    MeshData mesh;                     // tessellated boundary of the piece
    float    centre[3] = { 0, 0, 0 };  // physical bbox centre, used to explode
    float    size[3]   = { 0, 0, 0 };  // physical bbox extent, vs build volume

    float largestSide() const;
};

// Interior split parameters per axis, in domain coordinates, strictly between
// the domain bounds and sorted ascending. Empty means "do not cut this axis".
struct DivisionSpec {
    QVector<double> splits[3];
    QString         note;              // how it was built, for the UI
    bool            capped = false;    // hit the per-axis cell limit, so the
                                       // size budget could not be honoured

    int cells(int axis) const { return splits[axis].size() + 1; }
    int cellCount()     const { return cells(0) * cells(1) * cells(2); }
};

// One leaf of a recursive split: an axis-aligned box, in parameter space for a
// trivariate or world space for a mesh. Still a box, so region extraction and
// box clipping both work on it unchanged.
class MaterialField;

struct CellBox {
    double lo[3] = { 0, 0, 0 };
    double hi[3] = { 0, 0, 0 };

    double extent(int axis) const { return hi[axis] - lo[axis]; }
    double volume() const { return extent(0) * extent(1) * extent(2); }
};

class PuzzleDivider {
public:
    // Recursive (BSP) split into `targetPieces` boxes.
    //
    // This is the division that global cut planes cannot express. Planes run
    // the whole way through the model, so piece sizes are locked to rows and
    // columns and every interior piece has exactly six neighbours. Splitting
    // one cell at a time frees the sizes from each other and makes the
    // adjacency irregular - a big piece can border several small ones - which
    // is what a puzzle actually looks like.
    //
    // Cells are chosen for splitting in proportion to their volume and cut on
    // their longest axis, near the middle, so the pieces stay blocky instead of
    // degenerating into slivers.
    // `minSide` is the shortest a cell is allowed to get on any axis. A cell is
    // only cut where BOTH halves stay above it, and a cell with no axis long
    // enough to cut that way is left alone - so the split can finish below the
    // target rather than manufacture slivers nobody can print or handle.
    //
    // Pass 0 for automatic: 0.45 of the side of an average piece, which scales
    // with both the model and the requested count.
    // `material`, when given, makes the split MODEL-AWARE: cells are chosen and
    // cut by how much of the model they contain rather than by how big their box
    // is. That is what stops the splitter putting a cell in thin air.
    //
    // The guarantee it buys is worth stating exactly. The root cell holds all
    // the material, and a cut is only accepted when BOTH halves keep some, so by
    // induction every leaf holds material. No empty cells means none get dropped
    // later, so the piece count comes out as asked. Without it, a cage that is
    // 90% air - an armadillo's is - reliably loses a piece or two.
    static QVector<CellBox> buildBspCells(const double domain[6], int targetPieces,
                                          double splitJitter, quint32 seed,
                                          double minSide = 0.0,
                                          const MaterialField *material = nullptr);

    // The same split, kept as a TREE rather than flattened to leaves.
    //
    // Worth keeping because of what it makes possible afterwards: a cell and
    // its sibling always merge back into their parent box EXACTLY, so a piece
    // that turns out to hold almost no material can be absorbed by collapsing
    // its parent. Two arbitrary adjacent cells have no such box, which is why
    // the tree has to survive the split rather than being discarded.
    struct BspNode {
        CellBox box;
        int     child[2] = { -1, -1 };   // both -1 = leaf
        int     parent   = -1;
        bool    isLeaf() const { return child[0] < 0; }
    };

    static QVector<BspNode> buildBspTree(const double domain[6], int targetPieces,
                                         double splitJitter, quint32 seed,
                                         double minSide = 0.0,
                                         const MaterialField *material = nullptr);

    // Leaf node indices, in traversal order. `boxes` receives the matching
    // cells, so leaf i of the division is tree node leafIndex[i].
    static QVector<int> leavesOf(const QVector<BspNode> &tree,
                                 QVector<CellBox> *boxes);

    // Turns `node` back into a leaf, discarding its subtree. Its box already
    // covers exactly what its descendants covered, so the division stays a
    // partition - it just has one fewer, larger piece there.
    static void collapse(QVector<BspNode> &tree, int node);

    // Splits one leaf in place, same rules as the initial build: longest axis
    // first, near the middle, both halves clearing `minSide`. Returns false if
    // the cell is too small to cut on any axis.
    //
    // This is what balances the division. Absorbing crumbs merges cells and can
    // leave a piece far bigger than the rest; splitting the oversized ones back
    // down is the other half of the same job.
    static bool splitLeaf(QVector<BspNode> &tree, int node,
                          double splitJitter, double minSide,
                          QRandomGenerator *rng);

    // Face-sharing pairs among arbitrary boxes: they meet on a plane and their
    // footprints overlap there with real area. Needed because BSP cells have no
    // i/j/k to step through.
    struct Neighbours { int a, b, axis; };
    static QVector<Neighbours> adjacencyOfBoxes(const QVector<PuzzlePiece> &pieces,
                                                double eps);

    // Evenly spaced splits: counts[a] cells along axis a.
    static DivisionSpec uniform(const Trivariate &tv, const int counts[3]);

    // Evenly spaced, then each interior split nudged by up to `jitter` of a
    // cell width. This is Elber's difficulty randomisation - pieces stop being
    // interchangeable once they are all different sizes.
    static DivisionSpec jittered(const Trivariate &tv, const int counts[3],
                                 double jitter, quint32 seed);

    // Splits placed so that no cell exceeds `budget` millimetres on any axis.
    // Because M is generally not isometric, equal parameter steps do NOT give
    // equal physical sizes - so this walks the trivariate and places each split
    // where the accumulated physical extent reaches the budget. That is where
    // the non-uniform division earns its keep: sizing pieces to a build volume.
    static DivisionSpec toBuildVolume(const Trivariate &tv, const double budget[3],
                                      int maxCellsPerAxis);

    // The first half of toBuildVolume: splits placed at equal physical arc
    // length along each parameter axis. Exposed because it is the honest,
    // cheap answer when the caller does not need the world-bbox guarantee.
    static DivisionSpec splitsFromArcLength(const Trivariate &tv,
                                            const double budget[3],
                                            int maxCellsPerAxis);

    // Extracts every cell. `fineNess` follows IRIT's tessellation convention.
    // Divide into an arbitrary set of parameter-space boxes, rather than the
    // grid a DivisionSpec describes.
    //
    // A DivisionSpec is three lists of cut planes, so it can only ever produce
    // nu x nv x nw cells - asking it for 2 pieces gets you 2x2x2 = 8. The BSP
    // cells from buildBspCells are not a grid at all, and this is what puts
    // them through the same region extraction, so a requested piece count is
    // honoured exactly and the adjacency comes out irregular.
    static bool divideCells(const Trivariate &tv, const QVector<CellBox> &cells,
                            double fineNess, QVector<PuzzlePiece> *pieces,
                            QString *error);

    static bool divide(const Trivariate &tv, const DivisionSpec &spec,
                       double fineNess, QVector<PuzzlePiece> *pieces,
                       QString *error);

    // Face-sharing pairs, in grid order. The joint planner will run its
    // spanning tree over exactly this graph: one helical joint per tree edge,
    // a dovetail on every remaining (loop-closing) edge.
    static QVector<Neighbours> adjacency(const QVector<PuzzlePiece> &pieces,
                                         const DivisionSpec &spec);
};
