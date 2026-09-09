#pragma once
//
// CageBoolean - the second half of Elber's Section 5 (Fig. 14c -> 14d -> 14e).
//
// Dividing a bounding cage gives boxy sub-trivariates: the interior cut faces
// are right, the outer surface is the cage's, not the model's. The step that
// fixes that is a per-piece BOOLEAN INTERSECTION with the original model, which
// in Elber's words "will capture the geometry of the model on the outside while
// preserving the interior topology of the puzzle elements on the inside".
//
// Why this is not the same as clipping. MeshDivider::divideCells already
// computes model ∩ box geometrically, and for a BOX cage piece the two agree.
// The moment the cage stops being a box - which is the whole point of fitting a
// trivariate to the model in Increment 3 - clipping against axis-aligned planes
// cannot express the piece any more, and only a real boolean can.
//
// The intersection is the fragile step, so it is per-piece and failure is
// local: a piece whose boolean fails or comes back empty is reported and
// skipped, never allowed to take the run down with it.
//
#include "MeshData.h"
#include "PuzzleDivider.h"

#include <QString>
#include <QStringList>

class CageBoolean {
public:
    struct Result {
        int  intersected = 0;   // pieces that came back with real geometry
        int  dropped     = 0;   // cage cell held no model material - see below
        int  failed      = 0;   // boolean errored, or returned nothing usable
        int  skipped     = 0;   // piece was empty before we started
        int  noiseDropped= 0;   // debris too small to be a piece - see below
        double discardedVolume = 0.0;  // how much material that debris carried
        int  lumpsMerged = 0;   // detached lumps unioned back into a neighbour
        int  lumpsWelded = 0;   // ditto, but joined at the shared cut face
        int  orphans     = 0;   // lumps that touched nothing; kept standalone
        int  multiPart   = 0;   // FINAL pieces still made of >1 solid (must be 0)
        bool modelClosed = true;
        QStringList problems;   // one line per failure, naming the piece
        QStringList notes;      // non-failures worth reporting, e.g. drops
    };

    // A box cage always covers more than the model, so some cells come out
    // holding no material at all - four of a cow's 27 cells, for instance. An
    // empty intersection there is the CORRECT answer, and such pieces are
    // dropped rather than counted as failures. Keeping them (the earlier
    // behaviour) left boxes floating beside the model and inflated the piece
    // total to 186% of the model volume.
    //
    // Empty is told from broken by the guard, not by guesswork: IritGuard
    // replaces IRIT's own fatal handler, so every real boolean failure comes
    // back as an error rather than as an empty object. An empty result from a
    // call that reported success therefore means the cell and the model really
    // are disjoint.
    //
    // CONNECTEDNESS. A cage cell is a convex box; the model is not convex. One
    // cell can therefore catch several lumps of material with air between them
    // - a slice of thigh and a slice of tail - and the intersection returns all
    // of them as one "piece". Such a piece is not a body: it cannot be printed
    // as one part and there is no material path for a joint to act through. On
    // the armadillo this is common, and it gets WORSE with fewer pieces,
    // because bigger cells span more of the model (measured: 3 of 6 pieces at 6
    // pieces, carrying 8 detached lumps between them).
    //
    // This is inherent to the cage method rather than to any one splitter, and
    // it is why the paper demonstrates on a head - a blob, where a box cell
    // almost never catches two disjoint parts.
    //
    // Every intersection is therefore split into connected components and:
    //   * components of negligible volume are discarded as boolean noise
    //     (measured on the armadillo: real lumps run 72..1100 while the
    //     numerical debris sits at 1e-7..1, three orders of magnitude apart);
    //   * the largest component stays as the piece;
    //   * every other real lump is given back to the neighbouring piece it is
    //     actually attached to, since the material continues across the cut
    //     plane that severed it.
    // A lump that turns out to touch nothing is kept as its own piece and
    // counted in `orphans` rather than being quietly attached to something it
    // does not adjoin. `multiPart` re-checks the finished pieces and must be 0.

    // Replaces each piece's mesh with (piece ∩ model), and REMOVES pieces whose
    // cell held no material. Pieces that fail keep their original boxy
    // geometry, so the result is never silently emptied - the caller can see
    // from `failed` how much of the output is still cage.
    //
    // `model` should be watertight. IRIT's booleans on an open shell produce
    // garbage rather than an error, so an open model is reported in the result
    // and the intersection is attempted anyway.
    static Result intersectAll(QVector<PuzzlePiece> *pieces,
                               const MeshData &model,
                               double fineNess = 20.0);

    enum class Outcome {
        Ok,        // real geometry came back
        EmptyCell, // succeeded, but cell and model are disjoint
        Failed     // the boolean errored
    };

    // One piece, for testing the step in isolation.
    static Outcome intersect(const MeshData &piece, const MeshData &model,
                             MeshData *out, double fineNess, QString *error);

    static QStringList describe(const Result &r);

    // Boolean union of two solids, used to reattach a detached lump.
    static bool unite(const MeshData &a, const MeshData &b, MeshData *out,
                      double fineNess, QString *error);

    // Connected components of a mesh, largest first. Exposed because "is this
    // piece one solid?" is a question worth asking of any division, not only
    // of this one.
    static QVector<MeshData> components(const MeshData &m);
};
