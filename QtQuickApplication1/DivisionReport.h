#pragma once
//
// DivisionReport - a written analysis of one division, saved as report.html
// beside the planner pictures: the model, the stage timings, every piece, and
// what the planner concluded.
//
// Anything the app measured is reported as measured; anything it did not is left
// out rather than estimated. Volumes are computed here from the triangles, so
// this file needs no IRIT headers.
//
#include "MeshData.h"
#include "PuzzleDivider.h"

#include <QString>
#include <QStringList>
#include <QVector>

struct DivisionStats {
    QString mode;
    QString subject;
    int     requested   = 0;
    int     cells       = 0;
    quint32 seed        = 0;

    bool    hasField    = false;
    int     voxels[3]   = { 0, 0, 0 };
    int     voxelsFilled = 0;
    double  voxelSide   = 0.0;
    double  materialVolume = 0.0;

    qint64  msVoxelise  = -1;
    qint64  msSplit     = -1;
    qint64  msExtract   = -1;
    qint64  msTrim      = -1;
    qint64  msJoints    = -1;
    qint64  msFigures   = -1;
    qint64  msTotal     = -1;

    QString booleanNote;
    QString jointNote;
    QString warning;

    bool    hasPlan     = false;
    int     contacts    = 0;
    int     minNeighbours = 0;
    int     maxNeighbours = 0;
    bool    planComplete  = false;
    QStringList assemblyOrder;
    QVector<int> stuck;
    QString caveat;

    QStringList figureFiles;
};

namespace DivisionReport {

QString write(const QString &folder,
              const QString &title,
              const MeshData &model,
              const QVector<PuzzlePiece> &pieces,
              const DivisionStats &stats,
              QString *error);

}
