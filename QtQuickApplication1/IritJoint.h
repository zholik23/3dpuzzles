#pragma once
//
// IritJoint - puts joints on divided pieces the way Elber's puz_vol.irt does:
// build one joint solid, then boolean it onto the piece - union for a pin,
// subtract a slightly larger copy for the hole. Both come from the same loft, so
// the two always match.
//
// The profile is his PuzTile() friction pin. Two of its sections exist for the
// boolean rather than the shape: one starts below the face so the tool really
// crosses the piece boundary, and a zero-scale section closes the top, because an
// open tube is not a solid and cannot be a boolean operand.
//
// MeshData in, MeshData out - no IRIT types leak out.
//
#include "MeshData.h"
#include "PlannerJoints.h"
#include "PuzzleDivider.h"

#include <QString>
#include <QVector>

struct JointParams {
    double pinRadius   = 0.20;
    double height      = 0.60;

    double baseSink    = 0.25;

    double minPinThickness = 1.2;

    double clearanceXY = 1.06;
    double clearanceZ  = 1.05;

    double faceFraction = 0.45;
    double minSize      = 0.0;
    double maxSize      = 0.0;

    double fineNess     = 12.0;

    // --- dovetail ---------------------------------------------------------
    // A trapezoid in the (normal, cross) plane, extruded along the slide axis.
    // Narrow where it crosses the contact plane, wider at its full depth, so
    // the socket cannot lift off along the normal - that undercut IS the lock.
    // Fractions of the shared face's smaller side, so the joint scales with the
    // face it sits on and not with the model.
    // Proportions taken from IrtMdlrPuzDef1stTrapezePts in Elber's Puzzles
    // plugin (the "Linear Puzzle Slice"), which is IRIT's dovetail. Per tooth,
    // in units of its pitch, his profile is:
    //
    //     base  X 0.2 -> 0.5  (width 0.30)  at Y = 0
    //     top   X 0.0 -> 0.7  (width 0.70)  at Y = 0.40
    //
    // The top is wider than the base and overhangs on BOTH sides - that
    // undercut is the lock. Earlier values here (0.22 deep, 0.52 wide) were
    // several times shallower and read as a flat face on the model.
    // Sized so the notch reads as a step across the cut without taking a deep
    // bite out of the neighbour. At 0.40 deep on a 60 mm face the tooth
    // swallowed 24 mm of the piece opposite, and wherever the model was thin
    // there the trim left it as a flat flap instead of a joint.
    // Sized so the notch reads as a step across the cut without taking a deep
    // bite out of the neighbour. Measured: at 0.40 deep the tooth swallowed a
    // quarter of the piece opposite and the trim left flat flaps; a row of
    // larger teeth on every face broke the trim outright.
    double dtDepth      = 0.18;   // how far the tail reaches into the socket
    double dtNarrow     = 0.15;   // width at the contact plane
    double dtWide       = 0.35;   // width at full depth; must exceed dtNarrow
    double dtRunFrac    = 0.95;   // fraction of the face length the tail runs

    // How many teeth to try to put along one cut. Elber's linear slice repeats
    // its trapeze NumPieces times across the cut; a single tooth reads as a
    // block stuck on the model rather than a joint. Reduced automatically when
    // the face is too small to carry that many without going under the
    // printability floor.
    int    dtTeeth      = 3;
};

struct JointPlacement {
    double at[3]  = { 0, 0, 0 };
    double size   = 1.0;
    int    axis   = 0;       // contact normal
    bool   pin    = true;    // true = carries the tail, false = carries the socket

    // Dovetail only. slide >= 0 means "this is a dovetail, sliding along that
    // axis"; -1 keeps the old friction pin. The tail runs the full length of
    // the shared face along the slide axis so the other piece can slide on from
    // the edge, which is why the face rectangle is carried here: lo/hi are the
    // shared face's span in the two in-plane axes, (axis+1)%3 then (axis+2)%3.
    int    slide  = -1;
    double lo[2]  = { 0, 0 };
    double hi[2]  = { 0, 0 };

    // The narrowest solid dimension of THIS joint, in model units. Carried on
    // the placement because only the planner knows which kind of joint it is:
    // a pin's thinnest feature and a dovetail's are different formulas, and
    // reporting one for the other is how "thinnest pin 0.20 mm" ends up
    // describing a dovetail.
    double thinnest = 0.0;

    // How much solid material the joint actually has to live in, measured on
    // the face itself: room is the half-width of a square that is solid on BOTH
    // pieces, wall is the thinner of the two pieces across the contact. A tooth
    // larger than these does not join anything - it slices a lump off.
    double room = 0.0;
    double wall = 0.0;

    // One tooth's share of the cut. When set, the trapezoid is sized from THIS
    // rather than from the whole face, and the tooth still runs the full length
    // of the cut - which is what makes a row of teeth read as a dovetail joint
    // instead of one tab sitting on a flat plane.
    double pitch = 0.0;

    // The piece on the other side of this contact. The tail is clipped to it
    // before being unioned on, so the joint only ever MOVES material that the
    // neighbour already had - never invents any. That is what keeps the outer
    // shape of the model exactly as it was.
    int    mate = -1;
};

namespace IritJoint {

QVector<QVector<JointPlacement> > planPlacements(
        const QVector<PuzzlePiece> &pieces,
        const QVector<PuzzleDivider::Neighbours> &links,
        const JointParams &p,
        int *skipped = nullptr);

// slideAxis, when given, is per contact: >= 0 makes that joint a DOVETAIL
// sliding along that axis, -1 leaves it the old friction pin.
QVector<QVector<JointPlacement> > planPlacementsFor(
        const QVector<PuzzlePiece> &pieces,
        const Planner::Graph &g,
        const Planner::JointSet &keep,
        const JointParams &p,
        int *skipped = nullptr,
        const QVector<int> *slideAxis = nullptr);

// Cut ONE dovetail tooth between two pieces, as a re-cut rather than an
// addition: the very same solid is unioned onto the tail and subtracted from
// the socket, so the two stay exact complements.
//
//     tail   = tail   OR  T
//     socket = socket SUB T
//
// Union preserved, so nothing is added and nothing is lost; and because the
// same T is used on both sides there is NO clearance gap between them. Meant to
// run on the untrimmed cell boxes, before the pieces are trimmed to the model -
// then every outer surface of the result is model surface and the joint is
// invisible from outside.
bool cutDovetail(MeshData             *tail,
                 MeshData             *socket,
                 const JointPlacement &j,
                 const JointParams    &p,
                 QString              *error);

double sizeForFace(const JointParams &p, double faceMin, double depth,
                   bool dovetail = false);

// The narrowest solid dimension of a dovetail on a face of this size.
double dovetailThinnest(const JointParams &p, double faceMin);

// clipTo, when given, is the SOURCE MODEL. Each tail is intersected with it
// before being unioned on, so a tooth can never add material outside the
// model's own surface - which is what was deforming the silhouette - and a
// tooth over empty space simply vanishes instead of welding on a lump.
//
// It must be the model and not the neighbouring piece: the tail's root
// deliberately reaches back into its own piece, and clipping to the neighbour
// shears that root off, leaving an operand exactly coplanar with the piece's
// cut face. That is the one case IRIT's booleans reliably get wrong.
bool apply(MeshData                      *mesh,
           const QVector<JointPlacement> &places,
           const JointParams             &p,
           QString                       *error,
           int                           *applied,
           int                           *declined = nullptr,
           const MeshData                *clipTo = nullptr);

bool preview(MeshData *out, const JointParams &p, QString *error);

double thinnestFeature(const JointParams &p, double size);

}
