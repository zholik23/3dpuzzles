#pragma once
//
// AssemblyOrder - can these pieces be put together, and in what order?
//
// Adjacency, then blocking, then peeling. The three algorithms live in Planner
// (PlannerGraph / PlannerBlocking / PlannerOrder) and are called, not copied;
// this owns the question and the reporting.
//
// The blocking test is translational only, so every result it produces reads
// "assemblable under translational blocking; rotational/swept check pending" and
// must never be written down as collision-free.
//
#include "PuzzleDivider.h"

#include <QString>
#include <QStringList>
#include <QVector>

namespace Planner { class BlockingModel; }

class AssemblyOrder {
public:
    struct Step {
        int piece = -1;
        int dir   = -1;
    };

    struct Result {
        bool          assemblable = false;
        QVector<Step> removal;
        QVector<Step> assembly;
        QVector<int>  stuck;

        int contacts        = 0;
        int phantomContacts = 0;
        int minNeighbours   = 0;
        int maxNeighbours   = 0;

        QStringList adjacencyLog;
        QStringList blockingLog;
        QStringList resultLog;

        QStringList describe() const;
    };

    static Result run(const QVector<PuzzlePiece> &pieces, double tol = 1e-4);

    static Result run(const QVector<PuzzlePiece> &pieces,
                      const Planner::BlockingModel &model, double tol = 1e-4);

    static QString caveat();
};
