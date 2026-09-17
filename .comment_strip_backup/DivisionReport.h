#pragma once
//
// DivisionReport - a written analysis of one division, saved beside the model.
//
// The app already computes everything a reader would want to know about a
// division and then throws most of it away: the voxel grid it measured with,
// how long each stage took, what every piece came out as, what the planner
// concluded. This collects that into one self-contained `report.html` in the
// same folder as the planner pictures, so a division can be quoted, compared
// with another run, or put in front of someone a week later.
//
// It is written on EVERY division, beside `1_pieces.png` and the rest, and it
// links those pictures, so the folder is the whole record of one run.
//
// Numbers only. Anything the app measured is reported as measured, anything it
// did not is left out rather than estimated - a report that guesses is worse
// than no report. Volumes are computed here, from the triangles, so this file
// needs no IRIT headers.
//
#include "MeshData.h"
#include "PuzzleDivider.h"

#include <QString>
#include <QStringList>
#include <QVector>

struct DivisionStats {
    // --- what was asked for ------------------------------------------------
    QString mode;                  // "Random", "Uniform", "Max piece size"
    QString subject;               // "V-rep (bounding cage)" or "mesh"
    int     requested   = 0;       // pieces asked for, 0 when not a count mode
    int     cells       = 0;       // cells the split produced
    quint32 seed        = 0;

    // --- the voxel field, when one was built -------------------------------
    bool    hasField    = false;
    int     voxels[3]   = { 0, 0, 0 };
    int     voxelsFilled = 0;
    double  voxelSide   = 0.0;
    double  materialVolume = 0.0;  // what the field measured the model to be

    // --- timings, milliseconds; -1 means "this stage did not run" ----------
    qint64  msVoxelise  = -1;
    qint64  msSplit     = -1;
    qint64  msExtract   = -1;      // region extraction / clipping
    qint64  msTrim      = -1;      // Elber section 5 boolean
    qint64  msJoints    = -1;
    qint64  msFigures   = -1;
    qint64  msTotal     = -1;

    // --- what the trim did -------------------------------------------------
    QString booleanNote;
    QString jointNote;
    QString warning;

    // --- the planner -------------------------------------------------------
    bool    hasPlan     = false;
    int     contacts    = 0;
    int     minNeighbours = 0;
    int     maxNeighbours = 0;
    bool    planComplete  = false;
    QStringList assemblyOrder;     // "9 (-X)", "8 (-X)", ...
    QVector<int> stuck;
    QString caveat;

    QStringList figureFiles;       // file names, relative to the report
};

namespace DivisionReport {

// Writes `report.html` into `folder`. Returns the full path, or an empty
// string with `error` set. `model` is the solid that was divided, used for the
// model facts and as the denominator of the volume check.
QString write(const QString &folder,
              const QString &title,
              const MeshData &model,
              const QVector<PuzzlePiece> &pieces,
              const DivisionStats &stats,
              QString *error);

} // namespace DivisionReport
