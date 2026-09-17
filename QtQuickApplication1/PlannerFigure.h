#pragma once
//
// PlannerFigure - pictures of what the assembly planner decided: the pieces, the
// adjacency graph, the blocking, and the removal order.
//
// Written for slides rather than debugging, from the real result of the run that
// produced them. The graph, blocking model and plan are passed in rather than
// recomputed, so a figure cannot disagree with the log line beside it.
//
// Every figure that states a result carries AssemblyOrder::caveat(), because a
// picture gets quoted far more readily than a log line.
//
#include "PlannerOrder.h"
#include "PuzzleDivider.h"

#include <QString>
#include <QStringList>
#include <QVector>

namespace PlannerFigure {

struct Result {
    QString     folder;
    QStringList written;
    QStringList problems;
};

Result write(const QVector<PuzzlePiece> &pieces,
             const Planner::Graph &graph,
             const Planner::BlockingModel &model,
             const Planner::Plan &plan,
             const QString &folder,
             const QString &title);

QString folderFor(const QString &modelPath);

}
