#pragma once
//
// JointRotation - can a piece actually turn far enough to seat a rotate-to-engage
// joint (a bayonet), given the pieces already in place?
//
// This replaces a test that could never say yes: the old one turned the corners
// of the piece's BOUNDING BOX, which sit at the half-diagonal, so any nonzero
// rotation swung them into a neighbour (measured on a 24-piece cube, 1 degree
// gave 42 collisions and 90 degrees gave 45). Deciding this needs the REAL
// trimmed geometry, so rotate the piece's own mesh and look for actual overlap
// with the pieces already placed.
//
// Contact at rest is expected - neighbours share a face - so the moving piece is
// pulled in by `clearance` first. Overlap means interpenetration, not touching.
//
#include "IritJoint.h"
#include "MeshData.h"
#include "PuzzleDivider.h"

#include <QString>
#include <QStringList>
#include <QVector>

namespace JointRotation {

struct Params {
    // The seating rotation a bayonet needs. 90 is the usual quarter-turn.
    double angleDeg = 90.0;

    // Angles tested between 0 and angleDeg. Too few and the sweep steps over
    // the orientation that actually jams.
    int samples = 24;

    // How far the moving piece is pulled in from its neighbours before testing,
    // as a fraction of the model diagonal. Face contact at rest is not a
    // collision; interpenetration past this is.
    double clearanceFrac = 5e-4;
};

struct Result {
    bool   clear        = false;  // the whole rotation is reachable
    double maxAngleDeg  = 0.0;    // largest angle reached with no overlap
    int    blockedBy    = -1;     // piece that stopped it, -1 if none
    double blockedAtDeg = 0.0;
    int    tested       = 0;      // triangle tests run, summed over all angles
    int    rejected     = 0;      // bounding-box rejections, summed over all angles
    QString note;
};

// Rotates piece `moving` about the joint axis through `place.at`, and reports how
// far it gets. `present[i]` says whether piece i is already in place; the moving
// piece itself is skipped.
Result test(const QVector<PuzzlePiece> &pieces,
            int                         moving,
            const JointPlacement       &place,
            const QVector<bool>        &present,
            const Params               &params = Params());

// Every piece that carries a joint, tested in the order given. `order[k]` is the
// piece seated at step k, so each one is tested against the pieces before it.
//
// Only the FIRST placement on a piece is tested. A rotate-to-engage joint gives
// a piece one seating rotation, which is what the spanning tree produces - a
// piece needing two of them could not seat at all. Pins on other faces still
// need the translational check, which is stage 2's job, not this.
QVector<Result> testAll(const QVector<PuzzlePiece>                 &pieces,
                        const QVector<int>                         &order,
                        const QVector<QVector<JointPlacement> >    &places,
                        const Params                               &params = Params());

QStringList describe(const QVector<Result> &results, const QVector<int> &order);

// The piece's mesh turned `deg` about `axis` through `centre`. Exposed so a
// preview can draw the swept orientation the test rejected.
MeshData rotated(const MeshData &m, int axis, const double centre[3], double deg);

}
