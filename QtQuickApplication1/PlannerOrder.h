#pragma once
//
// Planner stage 3 - assembly order extraction.
//
// Disassembly is the easier direction to search, so the order is found by taking
// pieces off and then reversed. A piece can come off when the blocking model
// leaves it a free direction against the pieces still present, and the direction
// it left along is, reversed, the one it must be inserted along.
//
// A result is valid under whichever BlockingModel was used, and nothing more.
//
#include "PlannerBlocking.h"

namespace Planner {

struct Step {
    int piece = -1;
    int dir   = -1;
};

struct Plan {
    QVector<Step> removal;
    QVector<Step> assembly;
    QVector<int>  stuck;
    bool          complete = false;
    QString       modelName;
    QString       modelCaveat;
    QStringList   problems;

    QStringList describe(int maxSteps = -1) const;
};

Plan extract(const Graph &g, const BlockingModel &model);

int replay(const Graph &g, const BlockingModel &model, const Plan &plan,
           QString *why = nullptr);

}
