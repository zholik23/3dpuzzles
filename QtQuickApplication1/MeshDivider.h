#pragma once
//
// MeshDivider - divides a loaded mesh by clipping it against a non-uniform grid
// of world-space cells.
//
// It exists alongside PuzzleDivider because Elber's method needs a V-rep, and an
// STL or OBJ has no interior. Working on the mesh keeps the model's true shape,
// which a bounding-cage placeholder cannot.
//
// The trade-off is honest: clipping a surface mesh gives surface pieces. A cut
// through solid material leaves an open hole, capped when its outline is a single
// loop. Nested outlines - a hollow wall, a lattice strut - are left open and
// reported rather than capped wrongly.
//
#include "CutWarp.h"
#include "MeshData.h"
#include "PuzzleDivider.h"

#include <QString>
#include <QVector>

struct MeshDivisionSpec {
    QVector<double> planes[3];
    QString         note;
    bool            capped = false;

    int cells(int axis) const { return planes[axis].size() + 1; }
    int cellCount()     const { return cells(0) * cells(1) * cells(2); }
};

class MeshDivider {
public:
    static MeshDivisionSpec uniform(const MeshData &mesh, const int counts[3]);

    static MeshDivisionSpec jittered(const MeshData &mesh, const int counts[3],
                                     double jitter, quint32 seed);

    static MeshDivisionSpec toBuildVolume(const MeshData &mesh,
                                          const double budget[3],
                                          int maxCellsPerAxis);

    static MeshDivisionSpec balanced(const MeshData &mesh, const int counts[3]);

    static bool divide(const MeshData &mesh, const MeshDivisionSpec &spec,
                       QVector<PuzzlePiece> *pieces, QString *report,
                       double cutDetail = 0.0);

    static bool divideCells(const MeshData &mesh, const QVector<CellBox> &cells,
                            QVector<PuzzlePiece> *pieces, QString *report,
                            double cutDetail = 0.0);

    static bool divideBspAbsorbing(const MeshData &mesh, int targetPieces,
                                   double jitter, quint32 seed,
                                   QVector<PuzzlePiece> *pieces,
                                   QVector<CellBox> *cells,
                                   int *absorbed, QString *report);

    static MeshData warp(const MeshData &mesh, const CutWarp &w);
    static void     unwarp(QVector<PuzzlePiece> *pieces, const CutWarp &w,
                           const float bmin[3], const float bmax[3],
                           double diagonal);
};
