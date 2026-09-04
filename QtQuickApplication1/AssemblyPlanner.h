#pragma once
//
// AssemblyPlanner - decides which joint goes on which face, and in what order
// the pieces go together.
//
// This is the "automatic (dis)assemblability verification" Elber lists as open
// future work. The structure follows the self-locking argument:
//
//   * The rotate-to-engage joints (the spirals) form a SPANNING TREE over the
//     piece-adjacency graph. Each piece is therefore seated onto exactly one
//     parent, with exactly one seating rotation - which is all a piece can
//     physically perform.
//   * Every remaining edge closes a loop and gets a DOVETAIL instead. The
//     dovetail is what blocks the rotation the spiral would need in order to
//     back out, so the assembly cannot shake itself apart.
//
// Two things then have to be checked rather than assumed:
//   1. Each piece can actually execute its seating motion - approach along the
//      joint axis, then rotate about it - without hitting a piece already in
//      place.
//   2. Each spiral has at least one dovetail oriented to block its unwind.
//
// Nothing here depends on the spiral's cross-section, only on the fact that
// seating it needs a rotation about a known axis. So the geometry can be
// decided separately.
//
#include "PuzzleDivider.h"

#include <QString>
#include <QStringList>
#include <QVector>

struct PlannedJoint {
    enum Kind { Spiral, Dovetail };

    int  parent = -1;      // piece the child seats onto (tree edges only)
    int  child  = -1;
    int  axis   = 0;       // 0/1/2 - normal of the shared face, and for a
                           // spiral, the axis it rotates about
    Kind kind   = Spiral;
};

struct AssemblyPlan {
    QVector<PlannedJoint> joints;
    QVector<int>          order;      // piece indices, in placement order
    QVector<int>          parentOf;   // -1 for the root
    QStringList           problems;   // empty means the plan is valid

    int  spiralCount()   const;
    int  dovetailCount() const;
    bool valid() const { return problems.isEmpty(); }
};

class AssemblyPlanner {
public:
    struct Params {
        // How far a piece must turn to seat. This is the single number that
        // decides whether an assembly is possible: a multi-turn screw sweeps
        // most of a cylinder and collides with everything, a quarter-turn
        // bayonet sweeps very little. Defaults to a quarter turn.
        double seatingRotationDeg = 90.0;

        // Gap allowed between pieces before two boxes count as touching.
        double clearance = 1e-4;

        // How many angles the rotation sweep is sampled at. More is tighter but
        // slower; the sweep is a union of sampled poses, so too few samples
        // UNDER-estimates the swept volume and can pass a plan that would jam.
        int rotationSamples = 12;
    };

    static AssemblyPlan plan(const QVector<PuzzlePiece> &pieces,
                             const QVector<PuzzleDivider::Neighbours> &links,
                             const Params &params);

    // Human-readable summary, for the debug log.
    static QStringList describe(const AssemblyPlan &plan,
                                const QVector<PuzzlePiece> &pieces);
};
