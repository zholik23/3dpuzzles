#pragma once
//
// IritJoint - puts joints on divided pieces the way Elber's puz_vol.irt does:
// build one joint solid, then BOOLEAN it onto the piece - union for a pin,
// subtract a slightly larger copy for the matching hole.
//
// The joint is now HIS pin, verbatim from PuzTile() in
// C:\irit\irit\scripts\puz_vol.irt: seven circles of radius 0.2 lofted along
// the axis at the scales 1.0 / 0.75 / 0.775 / 0.8 / 0.8 / 0.6 / 0.0. That
// profile is a friction pin with a barb - it swells from 0.75 to 0.8 and then
// necks to 0.6 before the tip, so it snaps past the mouth of the hole and is
// held by the neck.
//
// Two of those seven sections are load-bearing for the BOOLEAN, not the shape,
// and they are the two the spiral experiment in scripts/spiral_test.irt left
// out:
//
//   * `Crc * tz( -0.02 )` puts the first section BELOW the face, so the tool
//     really crosses the piece boundary. A tool flush at z = 0 only touches,
//     and IRIT reports "objects in a subtraction operation failed to intersect"
//     and hands the first operand back unchanged.
//   * `Crc * sc( 0.0 )` closes the top. An open tube is not a solid and cannot
//     be a boolean operand at all.
//
// The pin and the hole come from the SAME loft at the same place: only the
// operation and Elber's clearance scale differ, so the two always match.
//
// Interface is plain MeshData in / MeshData out - no IRIT types leak out, so
// only this .cpp needs the IRIT headers.
//
#include "MeshData.h"
#include "PlannerJoints.h"
#include "PuzzleDivider.h"

#include <QString>
#include <QVector>

struct JointParams {
    // Elber's numbers, in joint-local units. `pinRadius` is his
    // `Crc = pcircle( vector( 0, 0, 0 ), 0.2 )`; the section scales that ride
    // on it are fixed, because they are what the pin IS.
    double pinRadius   = 0.20;
    double height      = 0.60;   // his sections span z = 0 .. 0.6

    // How far below the mating face the loft starts, as a fraction of `height`.
    // Elber's `tz( -0.02 )` is 3% of his 0.6 pin - fine at his unit scale, but
    // once the joint is scaled to a real face that becomes ~0.1 mm of overlap,
    // which is at or under the boolean's own tolerance. The tool then only
    // grazes the face instead of crossing it and the result comes back short.
    // Sinking it properly costs nothing: below the face is the pin's own piece,
    // already solid, and it is outside the piece being bored.
    double baseSink    = 0.25;

    // Smallest pin the printer can make, in model units. Faces that cannot
    // carry at least this get NO joint rather than an unprintable one - the
    // planner is happy with unpegged faces, so this is a free constraint.
    double minPinThickness = 1.2;   // three 0.4 mm extrusions

    // Female clearance, his SclJointXY / sz( 1.05 ).
    double clearanceXY = 1.06;
    double clearanceZ  = 1.05;

    // How much of the shared face the joint may take up, and the limits it is
    // clamped to in model units.
    double faceFraction = 0.45;
    double minSize      = 0.0;   // 0 = no floor
    double maxSize      = 0.0;   // 0 = no ceiling

    double fineNess     = 12.0;  // IRIT tessellation of the lofted surface
};

// One joint on one piece. `at` is the point on the shared face, `axis` is the
// face normal (0/1/2) and the joint always points along +axis: the pin grows
// out of the low piece, the hole is bored into the high piece from the same
// side, so the two are the same solid in the same place.
struct JointPlacement {
    double at[3]  = { 0, 0, 0 };
    double size   = 1.0;         // model units across the joint
    int    axis   = 0;
    bool   pin    = true;        // true = union a pin, false = subtract a hole
};

namespace IritJoint {

// Works out where every joint goes. For each shared face: a PIN on the piece
// on the low side of the axis and the matching HOLE on the high side, both at
// the centre of the overlap rectangle and both pointing along +axis - which is
// Elber's XMinHole / XMaxPin rule, and needs no assembly order to decide.
//
// The size follows the face: a joint is scaled to `faceFraction` of the
// smaller side of the overlap, then clamped so the pin cannot be longer than
// the piece it has to reach into. Returns one list per piece, indexed as
// `pieces`. `skipped` counts faces too small to carry a joint.
QVector<QVector<JointPlacement> > planPlacements(
        const QVector<PuzzlePiece> &pieces,
        const QVector<PuzzleDivider::Neighbours> &links,
        const JointParams &p,
        int *skipped = nullptr);

// Placements for exactly the contacts the PLANNER chose. This is the one to
// use: a joint on every shared face pegs most pieces on two different axes at
// once, and then nothing can be pulled out - the planner decides which faces
// may carry one without breaking the removal order it found.
QVector<QVector<JointPlacement> > planPlacementsFor(
        const QVector<PuzzlePiece> &pieces,
        const Planner::Graph &g,
        const Planner::JointSet &keep,
        const JointParams &p,
        int *skipped = nullptr);

// Size for a joint on one face, or 0 if the face cannot carry one: scaled to
// `faceFraction` of the narrower side, then held short of the depth available.
double sizeForFace(const JointParams &p, double faceMin, double depth);

// Applies every placement to one piece. On failure `mesh` is left untouched,
// `error` is set, and false is returned; `applied` reports how many booleans
// actually took (IRIT can decline one and still return a usable object).
// `declined` counts booleans IRIT refused to perform because the two objects
// did not intersect. That is NOT cosmetic: a declined hole leaves a pin with
// nowhere to go, and the puzzle will not close up. It must be reported.
bool apply(MeshData                      *mesh,
           const QVector<JointPlacement> &places,
           const JointParams             &p,
           QString                       *error,
           int                           *applied,
           int                           *declined = nullptr);

// The joint on its own, for looking at it. Same loft the booleans use.
bool preview(MeshData *out, const JointParams &p, QString *error);

// Thinnest solid dimension of a pin built at `size` - the diameter at the
// neck. Below about three extrusion widths the pin is not printable, which is
// easy to walk into on small pieces.
double thinnestFeature(const JointParams &p, double size);

} // namespace IritJoint
