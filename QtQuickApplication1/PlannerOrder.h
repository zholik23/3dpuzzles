#pragma once
//
// Planner stage 3 - assembly order extraction.
//
// Disassembly is the easier direction to search, so the order is found by
// taking pieces OFF and then reversed. A piece can come off when the blocking
// model leaves it at least one free direction against the pieces still present;
// the direction it left along is, reversed, the direction it has to be inserted
// along during assembly.
//
// What a result here does and does not mean is spelled out on Plan::describe()
// and repeated in the log. Short version: valid under whichever BlockingModel
// was used, and nothing more.
//
#include "PlannerBlocking.h"

namespace Planner {

struct Step {
    int piece = -1;
    int dir   = -1;      // for `removal`: the way it came off
                         // for `assembly`: the way it goes in (the opposite)
};

struct Plan {
    QVector<Step> removal;      // disassembly, first piece off first
    QVector<Step> assembly;     // reverse of `removal`, directions flipped
    QVector<int>  stuck;        // pieces still present when it gave up
    bool          complete = false;
    QString       modelName;
    QString       modelCaveat;
    QStringList   problems;

    QStringList describe(int maxSteps = -1) const;
};

// Greedy monotone removal, no backtracking. See the note at the top of the .cpp
// about what a failure here does and does not prove.
Plan extract(const Graph &g, const BlockingModel &model);

// Checks an order that was found under one model against another - the honest
// way to test a change (adding joints, swapping in the real collision check)
// without letting the search quietly pick a different order to suit itself.
// Returns the index of the first step that does not hold, or -1 if all do.
int replay(const Graph &g, const BlockingModel &model, const Plan &plan,
           QString *why = nullptr);

} // namespace Planner
