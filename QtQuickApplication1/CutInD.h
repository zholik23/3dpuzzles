#pragma once
//
// CutInD - cuts a part's own mesh in the parameter domain D of its fitted
// trivariate M, so the pieces keep the model's exact outer surface while every
// cut is the image of a plane in D.
//
// Each vertex of the part is mapped into D by inverting M (Newton on the
// control net); a vertex outside the block gets extended coordinates along the
// surface normal, so the block does not have to enclose its part. D is used in
// its solid-cylinder picture Q = (u, r cos 2pi v, r sin 2pi v), r = 1 - w: the
// skeleton is the line r = 0 and v has no seam there. In Q every cut is one
// field:
//   - the core, r = rc: a tube round the skeleton (pieces never meet on the
//     axis, where M collapses);
//   - planes through the axis, r sin(2pi v - theta): the seam cut and cuts
//     round the shell;
//   - u = c: cuts along the part.
// A cut splits a closed mesh along the field's level line (crossing points
// shared per edge) and closes both sides with ONE cap, triangulated in the
// cut's own 2D coordinates, its inner points placed on M(plane) - so
// neighbouring pieces meet exactly and every piece is closed.
//
// Plain C++ (no IRIT); the block is HarmonicFit's control net.
//
#include "HarmonicFit.h"
#include "MeshData.h"
#include "Trivariate.h"

#include <QString>
#include <QStringList>
#include <QVector>

#include <functional>

class CutInD {
public:
    // A cut, exact: its surface in D (a B-spline surface - the bulge is a
    // B-spline, the same function the mesh was cut by), and M composed with it
    // is the cut in the model. The seam is two cuts (its two half-planes).
    struct Cut {
        QString                      kind;       // "core", "seam", "u", "angle", "plane"
        QVector<Trivariate::DSurface> surfaces;
        int                          part = -1;  // cutWhole: the part whose trivariate it lives in
    };
    // cutWhole: a piece's cell in one part's trivariate (a piece may span parts).
    struct PartCell {
        int    part = -1;
        double lo[3] = { 0, 0, 0 }, hi[3] = { 1, 1, 1 };
    };
    struct Piece {
        MeshData mesh;
        double   lo[3] = { 0, 0, 0 };   // its cell in D: u, v, w
        double   hi[3] = { 1, 1, 1 };
        double   vWrap = 0.0;           // > 0: its cell also takes v in [0, vWrap) (past the closed v's seam)
        bool     core = false;
        // The cuts bounding it: (index into Result::cuts, its side: +1 = the
        // side dS/ds x dS/dt of the cut's D surface points to, -1 the other).
        QVector<QPair<int, int>> cuts;
        QVector<PartCell>        cells;   // cutWhole only
    };
    struct Result {
        bool          ok = false;
        QString       error;
        QStringList   notes;
        QVector<Piece> pieces;
        QVector<Cut>  cuts;
        double        partVolume = 0.0, piecesVolume = 0.0;
        int           openEdges = 0;         // boundary edges left in the pieces (0 = all closed)
    };

    // coreRadius: the core tube, as a share of the way from the skeleton (0)
    // to the surface (1); 0 = no core.
    // bend > 0: curved cuts in D. A cut along the part (u = c) or round it
    // (through the axis) gets a bulge - a sine wave (`waves` per side) under a
    // bell that is zero at the edges of the piece being cut, so it never
    // crosses the piece's other cuts - of height `bend` x the room between
    // the cut and the piece's nearest faces (those faces' own bulges counted).
    // The bulge is a quadratic B-spline (the sine x bell at its Greville
    // points), so every cut is exactly a B-spline surface in D.
    // The core tube and the first cut through the axis stay flat.
    static Result cut(const MeshData &part, const HarmonicFit::Result &block, int pieces,
                      quint32 seed, double coreRadius = 0.4, double bend = 0.0, int waves = 1);

    // The whole model cut at once, not part by part: the limb split only
    // supplies coordinates. Each piece is cut in the trivariate of the part
    // most of it belongs to (its "home"); the cut is applied to the whole
    // piece, so where the piece reaches into a neighbouring part the cut runs
    // on through it in the home trivariate's extended map (along its side
    // normals, past its ends) - pieces cross the joints, and the split's flat
    // caps never become piece faces. Exact (M o S) inside the home part,
    // smooth but not exact beyond it.
    // Candidate cuts per piece - across its home part (u = c) or round its
    // axis - are ranked by how evenly they halve the piece and how little
    // they run through other parts; a cut leaving a side in two is refused.
    // vertexPart: per model vertex, its part (-1 unknown); fits: per part.
    static Result cutWhole(const MeshData &model, const QVector<int> &vertexPart,
                           const QVector<HarmonicFit::Result> &fits, int pieces, quint32 seed,
                           double bend = 0.0, int waves = 1);

    // Gluing parts of a limb split back together (both exact: the parts meet
    // on the SAME cap triangles, a fan round the cap's centre).
    //
    // Two whole parts: the shared fan is removed from both and the rest joined.
    static MeshData unionAtCap(const MeshData &a, const MeshData &b, const double capCentre[3],
                               QString *why = nullptr);
    // A whole part onto the pieces its parent was cut into: it joins the piece
    // holding most of the cap; where other pieces also touch the cap, their cap
    // triangles (turned round) become the new piece's boundary with them, and
    // the part's rim triangles are split where the parent's cuts cross the rim,
    // so the result stays closed. Returns the index of the piece it joined, or
    // -1 (with `why`) if it could not.
    static int attachAtCap(QVector<MeshData> *parentPieces, const MeshData &child,
                           const double capCentre[3], QString *why = nullptr);

    // Edges not shared by exactly two triangles (0 = closed).
    static int openEdges(const MeshData &m);
    static int openEdgesWelded(const MeshData &m);   // after fusing equal positions

    // --- contacts, for the blocking analysis ----------------------------------
    //
    // A contact sample between two pieces: n (not unit) points out of piece a,
    // into piece b, in the model.
    struct Contact {
        int    a = -1, b = -1;
        double n[3] = { 0, 0, 0 };
    };
    // Which piece holds a point of the model: three axis rays against each
    // closed piece mesh, majority vote; -1 = none (outside the model).
    using Locate = std::function<int(const double *)>;
    static Locate locator(const QVector<MeshData> &pieces);
    // A cut, sampled on its exact surface in D (samples x samples per surface).
    // At each sample the pieces on its two sides are found by stepping off it
    // along its normal in D and locating both points in the model; where two
    // different pieces meet (inside the model), the contact normal is
    //     n' ~ J^-T n_D = n_u (M_v x M_w) + n_v (M_w x M_u) + n_w (M_u x M_v)
    // (cofactors of M's Jacobian, no inverse), turned to point from a to b.
    static QVector<Contact> cutContacts(const Cut &cut, const HarmonicFit::Result &block,
                                        const Locate &locate, int samples = 24);
    // The same across the flat joint caps between parts of the limb split
    // (their triangles in `caps`): the normal is the triangle's. `eps` is how
    // far each side is probed.
    static QVector<Contact> capContacts(const MeshData &caps, const Locate &locate, double eps);

    // A piece's trimming triangles: those on `surface` (the model's triangles
    // and the joint caps), as opposed to its cut faces, which are faces of its
    // sub-trivariate. 1 per trimming triangle of `piece`.
    static QVector<char> trimTriangles(const MeshData &piece, const MeshData &surface);
};
