#pragma once
//
// PlannerFigure - pictures of what the assembly planner decided.
//
// Written for slides rather than for debugging: each image explains one stage
// of the planner to someone who has not read the code, using the real pieces
// and the real result of the run that produced them. Nothing is illustrative.
//
//   1_pieces.png           the pieces, pulled apart and numbered
//   2_adjacency_graph.png  stage 1 - a line for every shared face, coloured by axis
//   3_blocking.png         stage 2 - for every piece, which straight moves are
//                          blocked, and by which neighbour
//   4_removal_order.png    stage 3 - pieces taken off one at a time; played
//                          backwards, the assembly order
//
// The graph, blocking model and plan are passed IN rather than recomputed, so a
// figure can never disagree with the log line printed beside it.
//
// Every figure that states a result carries AssemblyOrder::caveat(). The model
// behind these pictures is translational only, and a picture gets quoted far
// more readily than a log line.
//
#include "PlannerOrder.h"
#include "PuzzleDivider.h"

#include <QString>
#include <QStringList>
#include <QVector>

namespace PlannerFigure {

struct Result {
    QString     folder;       // where the images actually went
    QStringList written;      // full paths, in presentation order
    QStringList problems;     // empty when everything was written
};

// Writes the four figures into `folder`, creating it if needed. If it cannot be
// created - a model opened from a read-only location - the images go to
// Pictures/PuzzleDivider/<title>_planner instead, and `problems` says so.
// `title` names the model in the first figure's heading.
Result write(const QVector<PuzzlePiece> &pieces,
             const Planner::Graph &graph,
             const Planner::BlockingModel &model,
             const Planner::Plan &plan,
             const QString &folder,
             const QString &title);

// <model folder>/<model name>_planner - beside the model, so each model's
// figures are found next to it. Empty if `modelPath` is.
QString folderFor(const QString &modelPath);

} // namespace PlannerFigure
