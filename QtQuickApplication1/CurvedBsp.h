#pragma once
//
// CurvedBsp - a BSP division of the trivariate's PARAMETER DOMAIN D whose splits
// are curved surfaces, not planes.
//
// Why: a flat split in D becomes curved in R^3 only as much as M bends it, and
// on Elber's sphere and duck M leaves two of the three cut directions flat, so
// pairs of pieces slide out. Here each split is designed to curve:
//
//     g(p) = (p_a - at) - A * sin(2 pi k s_b + phi_b) * sin(2 pi k s_c + phi_c)
//
// for split axis a, the other two axes b and c (s_b, s_c their position across
// the node's box, 0..1). g = 0 is the cut; g >= 0 is the high side (child 1).
// A pull along d separates the two sides only if d . n >= 0 over the whole
// surface, with n' ~ J^-T grad g - the same test the grid analysis uses.
//
// The tree comes from PuzzleDivider::buildBspTree (the app's own BSP). What is
// added is the bend: amplitude A is a fraction `bend` of the room between the
// split and the nearest (already curved) faces of its cell, so a split never
// crosses an earlier one and every piece stays one connected region.
//
// Pieces live in D; to see them in R^3 their boundary is meshed in D and every
// vertex mapped through M (Trivariate::evaluate) - the trivariate B-spline is
// what gives the pieces their shape, not a box cage.
//
#include "MeshData.h"
#include "PuzzleDivider.h"
#include "Trivariate.h"

#include <QVector>

class CurvedBsp {
public:
    struct Split {
        int    axis = 0;
        double at = 0.0;            // split position along axis
        double amp = 0.0;           // A, in parameter units along axis
        int    waves = 1;           // k
        double phase[2] = { 0, 0 }; // phi_b, phi_c
        double lo[3] = { 0, 0, 0 }; // the node's box: the wave spans it
        double hi[3] = { 0, 0, 0 };
    };

    // bend in [0, 0.9]: 0 = flat splits (a plain BSP in D), 0.9 = as curved as
    // the cell allows. waves >= 1. Same seed = same division.
    static CurvedBsp build(const Trivariate &tv, int pieces, double bend,
                           int waves, quint32 seed);

    bool isValid() const { return !m_tree.isEmpty(); }
    int  pieceCount() const { return m_leafNode.size(); }
    void domain(double d[6]) const;

    // Which piece contains p (parameter space). `margin`, if given, gets the
    // smallest |g| met on the way down - how far p is from any cut.
    int classify(const double p[3], double *margin = nullptr) const;

    // Inside test as a field: > 0 inside piece `piece`, < 0 outside, 0 on its
    // boundary. Used to mesh the piece.
    double pieceField(int piece, const double p[3]) const;

    // The cut surfaces, for sampling interfaces.
    int splitCount() const { return m_internal.size(); }
    int splitNode(int i) const { return m_internal[i]; }
    const Split &split(int node) const { return m_split[node]; }
    int  child(int node, int side) const { return m_tree[node].child[side]; }
    bool isInSubtree(int node, int root) const;
    int  pieceOfNode(int node) const { return m_pieceOf.value(node, -1); }
    int  pieceNode(int piece) const { return m_leafNode.value(piece, -1); }
    double g(int node, const double p[3]) const;
    void   gradG(int node, const double p[3], double out[3]) const;
    double pointOnSplit(int node, double pb, double pc) const;   // p_a on g = 0
    const double *faceBulge(int node) const { return m_bulge[node].b; }

    // The boundary of one piece, meshed in D on an n^3 grid and mapped through
    // M. Returns false if the piece is empty at this resolution.
    bool meshPiece(const Trivariate &tv, int piece, int n, MeshData *out) const;

private:
    struct Bulge { double b[6] = { 0, 0, 0, 0, 0, 0 }; };   // lo/hi per axis

    QVector<PuzzleDivider::BspNode> m_tree;
    QVector<Split> m_split;          // per node (internal nodes only meaningful)
    QVector<Bulge> m_bulge;          // per node: how far its faces bend past the box
    QVector<int>   m_internal;       // internal node indices
    QVector<int>   m_leafNode;       // piece id -> node
    QVector<int>   m_pieceOf;        // node -> piece id, -1 for internal
    double         m_dom[6] = { 0, 0, 0, 0, 0, 0 };
};
