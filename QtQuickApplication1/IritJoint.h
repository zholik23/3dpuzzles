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
};

struct JointPlacement {
    double at[3]  = { 0, 0, 0 };
    double size   = 1.0;
    int    axis   = 0;
    bool   pin    = true;
};

namespace IritJoint {

QVector<QVector<JointPlacement> > planPlacements(
        const QVector<PuzzlePiece> &pieces,
        const QVector<PuzzleDivider::Neighbours> &links,
        const JointParams &p,
        int *skipped = nullptr);

QVector<QVector<JointPlacement> > planPlacementsFor(
        const QVector<PuzzlePiece> &pieces,
        const Planner::Graph &g,
        const Planner::JointSet &keep,
        const JointParams &p,
        int *skipped = nullptr);

double sizeForFace(const JointParams &p, double faceMin, double depth);

bool apply(MeshData                      *mesh,
           const QVector<JointPlacement> &places,
           const JointParams             &p,
           QString                       *error,
           int                           *applied,
           int                           *declined = nullptr);

bool preview(MeshData *out, const JointParams &p, QString *error);

double thinnestFeature(const JointParams &p, double size);

}
