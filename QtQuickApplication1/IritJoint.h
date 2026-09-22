#pragma once
//
// IritJoint - joints on divided pieces, as in Elber's puz_vol.irt: build one
// joint solid, then boolean it onto the piece. MeshData in, MeshData out.
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

    // Dovetail: a trapezoid in the (normal, cross) plane extruded along the
    // slide axis - narrow at the contact plane, wider at full depth, so the
    // socket cannot lift off along the normal. Fractions of the shared face's
    // smaller side. Proportions follow IRIT's own linear slice
    // (IrtMdlrPuzDef1stTrapezePts). Deeper than ~0.2 eats the neighbouring
    // piece and the trim leaves flat flaps where the model is thin.
    double dtDepth      = 0.18;   // how far the tail reaches into the socket
    double dtNarrow     = 0.15;   // width at the contact plane
    double dtWide       = 0.35;   // width at full depth; must exceed dtNarrow
    double dtRunFrac    = 0.95;   // fraction of the face length the tail runs
    int    dtTeeth      = 3;      // teeth per cut; reduced if the face is small
};

struct JointPlacement {
    double at[3]  = { 0, 0, 0 };
    double size   = 1.0;
    int    axis   = 0;       // contact normal
    bool   pin    = true;    // true = carries the tail, false = the socket

    // slide >= 0 makes this a dovetail sliding along that axis; -1 = friction
    // pin. lo/hi span the shared face in (axis+1)%3 then (axis+2)%3.
    int    slide  = -1;
    double lo[2]  = { 0, 0 };
    double hi[2]  = { 0, 0 };

    double thinnest = 0.0;   // narrowest solid dimension, model units
    double room     = 0.0;   // half-width of a square solid on BOTH pieces
    double wall     = 0.0;   // thinner of the two pieces across the contact
    double pitch    = 0.0;   // one tooth's share of the cut; 0 = size from face
    int    mate     = -1;    // piece on the other side of the contact
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

// Cut one dovetail tooth between two pieces as a re-cut, not an addition:
// tail = tail OR T, socket = socket SUB T with the same T, so the two stay
// exact complements with no clearance gap. Must run on the untrimmed cell
// boxes - after the trim the shared face is no longer a rectangle.
bool cutDovetail(MeshData             *tail,
                 MeshData             *socket,
                 const JointPlacement &j,
                 const JointParams    &p,
                 QString              *error);

double sizeForFace(const JointParams &p, double faceMin, double depth,
                   bool dovetail = false);

// The narrowest solid dimension of a dovetail on a face of this size.
double dovetailThinnest(const JointParams &p, double faceMin);

// clipTo, when given, must be the SOURCE MODEL, not the neighbouring piece:
// the tail's root reaches back into its own piece, and clipping to the
// neighbour shears it off, leaving an operand coplanar with the cut face -
// the one case IRIT's booleans reliably get wrong.
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
