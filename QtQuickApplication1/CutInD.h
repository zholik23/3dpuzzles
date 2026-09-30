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

#include <QString>
#include <QStringList>
#include <QVector>

class CutInD {
public:
    struct Piece {
        MeshData mesh;
        double   lo[3] = { 0, 0, 0 };   // its cell in D: u, v, w
        double   hi[3] = { 1, 1, 1 };
        bool     core = false;
    };
    struct Result {
        bool          ok = false;
        QString       error;
        QStringList   notes;
        QVector<Piece> pieces;
        double        partVolume = 0.0, piecesVolume = 0.0;
        int           openEdges = 0;         // boundary edges left in the pieces (0 = all closed)
    };

    // coreRadius: the core tube, as a share of the way from the skeleton (0)
    // to the surface (1); 0 = no core.
    static Result cut(const MeshData &part, const HarmonicFit::Result &block, int pieces,
                      quint32 seed, double coreRadius = 0.4);

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
};
