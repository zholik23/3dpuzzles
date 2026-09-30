#pragma once
//
// HarmonicFit - a trivariate B-spline fitted to a closed triangle mesh, after
// Martin, Cohen & Kirby 2009, "Volumetric parameterization and trivariate
// B-spline fitting using harmonic functions" (CAGD 26, 648-664).
//
// This is what lets an OBJ/STL model be divided in its parameter domain D like
// Elber's tvs_*.itd: the result is a real V-rep M: D -> R^3, not a box cage.
//
// The paper's steps, and what is done here for each:
//   1. surface harmonic u (0 at a min pole, 1 at a max pole)  - cotangent
//      Laplacian, CG. The poles are the two ends of the principal axis
//      (the paper lets the user pick them).
//   2. v around each u level                                    - the level loop
//      is extracted, oriented by the surface normal, started next to the
//      previous level's start and sampled by arclength (the paper uses a second
//      harmonic between two critical paths; arclength keeps the same seam idea).
//   3. volume harmonic uH with u on the boundary               - finite
//      differences on a voxel grid (the paper uses FEM on a tet mesh).
//   4. skeleton: traced from the deepest point along +-grad uH, kept centred
//      by a small step up the distance field across the level set.
//   5. w harmonic, 0 on the boundary and 1 on the skeleton.
//   6. w-paths from each boundary point X(u_i, v_j) to the skeleton, along
//      grad w projected orthogonal to grad uH (so they stay in the u_i level),
//      resampled into the hex grid p_ijk; then smoothed.
//   7. iterative B-spline fit, lambda = 0.5, degrees 3/3/1, open in u and w,
//      periodic in v (eq. 2-4 of the paper).
//
// Same limits as the paper: genus 0 and no bifurcations. A model with limbs
// (armadillo) keeps the main tube only - the fit reports how many u levels had
// extra loops and how much of the model's volume the trivariate covers. The
// small caps around the two poles are left out (the u range stops short of
// 0 and 1), as in the paper.
//
// fit() is plain C++ (no IRIT), so it can run on a worker thread;
// toTrivariate() builds the IRIT trivariate from its control mesh.
//
#include "MeshData.h"
#include "Trivariate.h"

#include <QString>
#include <QStringList>
#include <QVector>

class HarmonicFit {
public:
    struct Options {
        // 24 x 24: a 16 x 16 net was measured to fold (det J < 0) at a few outer
        // cells on spot where a loop dips into a leg; 24 x 24 did not.
        int    nu = 24;          // control points along u (the long axis)
        int    nv = 24;          // around (periodic)
        int    nw = 5;           // from the surface (w = 0) to the skeleton (w = 1)
        int    voxels = 72;      // grid cells along the longest side
        double lambda = 0.5;     // relaxation step of the fit (paper: 0.5)
        int    maxIter = 40;
        double tol = 1e-3;       // stop when max |p - S| < tol * model size
        int    smooth = 3;       // w-path smoothing passes over the inner layers
        // The u range stops where the level loop is shorter than this fraction
        // of the longest loop: the caps beyond are not covered (as in the paper).
        double capPerimeter = 0.12;
        // A part cut off by splitLimbs: u = 0 is its cap (the fan whose centre
        // is capCentre), so the block's u = 0 face lies on the cap it shares
        // with its neighbour. The far pole is the vertex farthest from it.
        bool   capStart = false;
        double capCentre[3] = { 0, 0, 0 };
        // > 0: the block is made to ENCLOSE its part, for Elber's section 5
        // trim - the outer row of the hex grid (on the surface) moves out along
        // the surface normal by inflate x the part's size, the cap row out past
        // the cap and the tip past the tip, before the spline fit.
        double inflate = 0.0;
        double normalStart = 0.5;  // w-paths start along grad w, blended into the level set over this fraction
    };

    struct Result {
        bool        ok = false;
        QString     error;
        QStringList notes;                 // what happened, for the status line
        int         nu = 0, nv = 0, nw = 0;
        QVector<double> ctrl;              // 3 * (i + nu * (j + nv * k))
        QVector<double> grid;              // the hex grid p_ijk, same layout
        double      fitError = 0.0;        // max |p - S| / model size
        int         iterations = 0;
        int         fallbackPaths = 0;     // w-paths replaced by a straight line
        int         branchLevels = 0;      // u levels with more than one loop
        int         genus = -1;
        double      meshVolume = 0.0;
        double      size = 1.0;            // bounding-box diagonal
    };

    static Result fit(const MeshData &mesh, const Options &opt);

    // The fitted control mesh as an IRIT trivariate, domain [0,1]^3:
    // orders 4/4/2, v closed (built float, converted to open end conditions).
    static Trivariate toTrivariate(const Result &r, const QString &label,
                                   QString *error);

    // det J on a grid of sample points: how many are <= 0 (the w = 1 face is the
    // skeleton, where det J vanishes by construction, so it is not sampled), and
    // the trivariate's volume against the mesh's.
    struct Check {
        int    samples = 0;
        int    nonPositive = 0;   // folds: det J < 0 (a collapsed tip, det J = 0, is not counted)
        double minRatio = 0.0;   // min det / mean det
        double volume = 0.0;
        QVector<double> bad;     // (u, v, w) of the first non-positive samples
    };
    static Check checkJacobian(const Trivariate &tv, int n = 28);

    // How closely the trivariate's boundary follows the mesh: the distance
    // from every mesh vertex to the nearest point of the boundary faces
    // (w = 0 side, u = 0 and u = 1 ends), as a fraction of the model size.
    // This is the real accuracy; Result::fitError only compares the spline to
    // its own hex grid.
    struct Deviation {
        double mean = 0.0, p95 = 0.0, max = 0.0;   // fractions of the model size
        double over1 = 0.0;                        // fraction of vertices > 1% away
    };
    static Deviation surfaceDeviation(const Trivariate &tv, const MeshData &mesh, int n = 200);

    // How many of the mesh's vertices lie outside the fitted block (its
    // boundary: the w = 0 side and the u ends); `worst` gets the largest
    // distance outside, of the part's size. 0 = the block encloses the mesh.
    static int outsideCount(const Result &r, const MeshData &mesh, double *worst = nullptr);

    // After an enclosing fit: the few spots where vertices of the part still
    // stick out are pushed out locally, in rounds - each outer control point
    // by the largest need among the boundary samples it shapes (facing its
    // way), the u = 0 end row along its normal. The best round is kept.
    // Returns the vertices still outside.
    static int pushToEnclose(Result *r, const MeshData &mesh, int rounds = 8, double gap = 0.005);

    // Makes the trivariate ENCLOSE the model, for Elber's section 5 trim
    // (pieces cut in D are intersected with the mesh, so anything outside M
    // would be lost). The fit is kept as it is; material is added around it:
    // a new outer layer in w, offset along the fit's surface normals by what
    // each spot needs (the deepest model vertex outside it, plus `margin` of
    // the model size), and a new row past each u end over the caps. r becomes
    // (nu + 2) x nv x (nw + 1). Returns false if model vertices are still
    // outside; `notes` says how many and how far.
    static bool enclose(Result *r, const MeshData &mesh, QStringList *notes,
                        double margin = 0.01, int rounds = 1);

    // Limb split (the paper's future work, sec. 10: decompose a branching model
    // into parts, one B-spline each). Limb tips are the local maxima of the
    // geodesic distance from a root on the body; from each tip, geodesic rings
    // grow up the limb, and the limb is cut on the last calm ring before the
    // ring length jumps (where the limb widens into the body - its neck).
    // Every cut is capped with the SAME surface on both sides, so the parts are
    // closed and meet exactly - the first puzzle interfaces.
    struct Cap {
        double centre[3] = { 0, 0, 0 };   // the fan's centre vertex
        double area = 0.0;
    };
    struct Part {
        MeshData mesh;          // closed: the part's surface plus its caps
        QString  name;          // "body", "limb 1", ...
        int      parent = -1;   // part it was cut from (-1 for the body)
        double   level = 0.0;   // ring distance from the tip at its cut
        double   areaFrac = 0.0;
        QVector<Cap> caps;      // its cut faces
    };
    struct SplitOptions {
        double minArea = 0.0015;       // a limb must have at least this share of the area (ears, horns: 0.2-0.8%)
        double maxArea = 0.35;         // and at most this share
        double minPersistence = 0.05;  // and be at least this long (tip to cut, of the model size)
        double maxLength = 0.45;       // rings are grown this far from a tip at most
        double neckJump = 1.6;         // a joint: the first ring this much longer than the limb's narrowest
        double neckCalm = 0.15;        // calm ring (grew less than this over the last 5% of the size)
        double bulbDrop = 0.12;        // a bulb (head): the ring is this much narrower than the widest one
        double tipRadius = 0.08;       // tips closer than this (geodesic, of the size) are one tip
    };
    static bool splitLimbs(const MeshData &mesh, const SplitOptions &opt,
                           QVector<Part> *parts, QStringList *notes, QString *error);


    // Makes the blocks of a split meet. A limb's u = 0 face lies on its flat
    // cap, but the part it was cut from (the body) is a tube that only
    // approximates that cap on its side - measured on spot: the body's side is
    // 0.35-1.5% (worst 3.8%) of the model size off the leg caps, a visible lip.
    // Each such limb's cap row of control points is projected onto the body's
    // surface, and the next three rows follow with 3/4, 1/2, 1/4 of the move,
    // so both blocks share one surface at the joint. A cap that is the u = 0
    // face of both blocks (the neck: head and body) needs nothing.
    // results[i] belongs to parts[i]; failed fits are skipped.
    static void snapParts(QVector<Result> *results, const QVector<Part> &parts, QStringList *notes);
};
