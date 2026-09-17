#pragma once
//
// AssemblyPlanner - which joint goes on which face, and in what order the pieces
// go together.
//
// The spiral (rotate-to-engage) joints form a spanning tree over the adjacency
// graph, so each piece seats onto exactly one parent with one seating rotation.
// Every remaining edge closes a loop and gets a dovetail, which blocks the
// rotation a spiral would need to back out.
//
#include "PuzzleDivider.h"

#include <QString>
#include <QStringList>
#include <QVector>

struct PlannedJoint {
    enum Kind { Spiral, Dovetail };

    int  parent = -1;
    int  child  = -1;
    int  axis   = 0;
    Kind kind   = Spiral;
};

struct AssemblyPlan {
    QVector<PlannedJoint> joints;
    QVector<int>          order;
    QVector<int>          parentOf;
    QStringList           problems;

    int  spiralCount()   const;
    int  dovetailCount() const;
    bool valid() const { return problems.isEmpty(); }
};

class AssemblyPlanner {
public:
    struct Params {
        double seatingRotationDeg = 90.0;

        double clearance = 1e-4;

        int rotationSamples = 12;
    };

    static AssemblyPlan plan(const QVector<PuzzlePiece> &pieces,
                             const QVector<PuzzleDivider::Neighbours> &links,
                             const Params &params);

    static QStringList describe(const AssemblyPlan &plan,
                                const QVector<PuzzlePiece> &pieces);
};
