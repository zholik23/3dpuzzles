#pragma once
//
// DbgAnalysis - translational directional blocking graph for a trivariate
// puzzle, and the disassembly questions that follow from it.
//
// The division happens in the PARAMETER domain of M. A flat plane there becomes
// a curved surface in R^3, and a curved interface can refuse every direction at
// once, which a planar one never can. That is the whole point of cutting in D.
//
// The test, for one interface S between pieces A and B, with n the outward
// normal of A:
//
//     A can translate along d without pushing into B   <=>   d . n <= 0
//
// applied at every sampled point of S. A flat interface has one n, so exactly
// half the sphere of directions works. A curved one has many, and if they spread
// far enough their half-spaces intersect in nothing: the pair is locked.
//
// Normals come from the map, never from a tessellation. For an interface of
// constant u the surface tangents are M_v and M_w, so n is parallel to
// M_v x M_w - the same thing as J^-T applied to the domain normal, without
// having to invert J.
//
// THE DBG PROPER
//
// Blocking between single pieces is not enough to call an assembly
// interlocking: a GROUP of pieces can slide out together while no member of it
// could move alone. So every proper non-empty subset is tested, not just the
// singletons. A subset S moves along d when every interface that crosses the
// boundary of S permits it; interfaces buried inside S do not constrain it,
// because those two pieces move together.
//
// That distinction is what separates a pile of pairwise facts from a blocking
// graph, and it is the difference between "no single piece comes out" and
// "provably cannot be taken apart".
//
#include "Trivariate.h"

#include <QString>
#include <QVector>

struct DbgOptions {
    int    cells[3]     = { 2, 2, 2 };  // division of the parameter domain
    int    faceSamples  = 11;           // per axis on each interface (11x11)
    int    directions   = 4000;         // candidate pull directions
    int    jacobianGrid = 12;           // samples per axis for min det J
    double touch        = 1e-9;         // slack on d . n <= 0

    // Subsets up to this size are tested. All proper subsets are enumerated
    // when the piece count makes that affordable; beyond that the search is
    // capped, and the report says so rather than implying a full proof.
    int    maxSubset    = 4;
    int    fullSubsetsUpTo = 18;        // pieces (2^18 groups, a few seconds)

    // Up to this many pieces, blocked-or-free is decided by the exact clearance
    // (Gilbert); beyond it, by the sampled directions.
    int    exactUpTo    = 12;
};

struct DbgPair {
    int    a = -1, b = -1;   // piece indices; a is on the low side
    int    axis = 0;         // interface normal axis in the domain
    int    normals = 0;      // how many normals were sampled
    int    freeDirs = 0;     // sampled directions that separate a from b (informative)
    double spreadDeg = 0.0;  // widest angle between a normal and their mean
    bool   blocked = false;  // no direction separates them - decided by the clearance
    double clearanceDeg = 0.0;  // how far the pull may tilt and still separate them
};

struct DbgReport {
    bool    valid = false;
    QString error;

    int     pieces = 0;
    double  minDetJ = 0.0;            // over the domain, boundary included
    double  minDetAt[3] = { 0, 0, 0 };
    double  minDetInterior = 0.0;     // ignoring the domain boundary
    bool    jacobianOk = false;

    QVector<DbgPair> pairs;
    QVector<int>     pieceFreeDirs;   // per piece, sampled directions that free it
    // Per piece: how far its pull may tilt and still slide it out alone; -1 =
    // blocked. This, not the sampled count, decides mobility: a free set thinner
    // than the direction sampling is still free (sphere R = 0.05, piece 7:
    // 0 of 4000 sampled directions, yet it slides out).
    QVector<double>  pieceClearanceDeg;   // -2 = free, clearance not computed
    bool             exactClearance = false;   // false: decided by sampled directions

    int  blockedPairs = 0;
    int  mobilePieces = 0;
    bool singleKey = false;
    int  keyPiece = -1;

    double medianFreeDirs = 0.0;

    // World-space extent of each piece, obtained by sampling M over that
    // piece's cell. Lets the analysis be checked against the actual deformed
    // tiles the IRIT script writes out - if these boxes match, the analyser is
    // looking at the same puzzle.
    QVector<double> pieceBox;   // 6 per piece: xlo xhi ylo yhi zlo zhi

    // --- the DBG's answer ---------------------------------------------------
    int          mobileSubsets = 0;      // proper non-empty subsets that can move
    int          smallestMobile = 0;     // size of the smallest; 0 = none found
    QVector<int> smallestMobileSet;      // which pieces

    // A direction that actually frees that group, averaged over all the
    // directions which do. This is what turns "a pair escapes" into something
    // demonstrable: translate those pieces along it and the puzzle opens.
    double       escapeDir[3] = { 0, 0, 0 };
    int          escapeCount = 0;        // how many sampled directions free the group
    double       escapeClearanceDeg = 0.0;
    bool         interlocked = false;    // no proper subset can move
    bool         subsetSearchFull = false;  // false = capped, so not a proof
    int          subsetSizeTested = 0;

    // --- every way the assembled puzzle can open ----------------------------
    // A group and the rest of the puzzle always come apart together (the rest
    // moves along -d), so each is listed once, by its smaller side.
    struct Group {
        QVector<int> pieces;
        int          dirs = 0;           // sampled directions that free it (informative)
        double       clearanceDeg = 0.0; // how far the pull may tilt; decides "free"
        double       dir[3] = { 0, 0, 0 };  // the direction with the most clearance
    };
    QVector<Group> openings;             // capped at 64, smallest first

    // --- single key, then level k -------------------------------------------
    // Song et al. 2012, recursive interlocking: an assembly is interlocking when
    // exactly ONE piece (the key) can move and every other piece and every
    // group is held. Remove the key; if what is left is again interlocking with
    // its own key, that is level 2, and so on. keyLevel counts how many states
    // in a row have a unique key. Chen et al. 2022 count MOVES before the first
    // piece comes free, which needs finite motion search rather than a DBG, so
    // that number is not computed here.
    bool         strictSingleKey = false;  // the whole puzzle: one key, nothing else moves
    int          keyLevel = 0;
    QVector<int> keySequence;              // the keys, in removal order
    bool         fullyRecursive = false;   // every state down to 2 pieces had a unique key
    QString      levelStop;                // why the recursion stopped

    // --- one way to take it all apart -----------------------------------------
    // Greedy: repeatedly remove the smallest group that can move. Stops early
    // when what is left is interlocked or held by a blocked pair.
    QVector<Group> disassembly;
    bool           disassemblyComplete = false;
};

// The shape criteria of the experiment, per piece.
struct DbgPieceQuality {
    double volume   = 0.0;     // integral of det J over the piece's cell
    double minDetJ  = 0.0;     // over the piece's cell, boundary included
    double thickness = -1.0;   // shortest inward ray through the piece; -1 = not measured
    double thickAt[3] = { 0, 0, 0 };   // where, in R^3
    int    rays = 0;           // rays that contributed
};

struct DbgQuality {
    QVector<DbgPieceQuality> pieces;
    double modelSize  = 0.0;   // largest side of the model's box
    double volMin = 0.0, volMax = 0.0, volMean = 0.0;
    double volCv  = 0.0;       // standard deviation / mean
};

// Independent check of a DBG verdict by actually moving pieces.
//
// The DBG is a first-order test built on interface normals: it says the FIRST
// infinitesimal step along d is free. It cannot catch a sign error that mirrors
// every direction (the flat control passes either way - half the sphere is half
// the sphere), and it does not prove a group can travel all the way out.
//
// This test uses no normals. It samples points inside the moving pieces,
// translates them along d in small steps, and at each step inverts M (Newton,
// continued from the previous step) to find which cell of the domain each moved
// point now lies in. A point landing inside a piece that is NOT moving is a
// collision.
struct DbgSweep {
    int    samples  = 0;    // points tracked
    int    collided = 0;    // points that entered a non-moving piece
    int    lost     = 0;    // left the model, or Newton failed to converge
    double firstHit = -1.0; // travel distance at the first collision; -1 = none
};

namespace DbgAnalysis {

DbgReport run(const Trivariate &tv, const DbgOptions &opt);

// Move `group` (analyser piece ids) along `dir` up to distance `travel`.
DbgSweep sweep(const Trivariate &tv, const DbgOptions &opt,
               const QVector<int> &group, const double dir[3],
               double travel, int steps = 40, int perAxis = 8);

// Volume, fold check and minimum thickness of every piece.
//
// Thickness at a surface point is the length of the ray shot straight inward
// (along the negative normal) until it leaves the piece. The smallest over all
// sampled points is the piece's minimum thickness. Points are taken at least
// 10% of a face away from its edges (a sharp edge is always thin at the edge
// itself), and points where the normal is undefined - a pole, or a face that
// collapses to a point - are skipped.
//
// Connectivity needs no test: a piece is the image of one box under a
// continuous map, so it is always one piece. What CAN go wrong is a fold (det J
// <= 0), where the piece passes through itself; minDetJ reports that.
DbgQuality quality(const Trivariate &tv, const DbgOptions &opt,
                   int raysPerFaceAxis = 5);

QString formatReport(const QString &label, const DbgReport &r,
                     const DbgOptions &opt);

QString tableHeader();
QString tableRow(const QString &label, const DbgReport &r);

}
