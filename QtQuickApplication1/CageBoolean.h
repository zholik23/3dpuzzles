#pragma once
//
// CageBoolean - Elber section 5's per-piece boolean intersection with the model,
// which gives a cage piece the model's outer surface while preserving the
// puzzle's interior faces.
//
// Clipping is not a substitute. It agrees only while the cage is a box, and the
// point of fitting a trivariate to the model is that it stops being one. The
// intersection is the fragile step, so failure is local: a piece whose boolean
// fails or comes back empty is reported and skipped, never allowed to take the
// run down with it.
//
#include "MeshData.h"
#include "PuzzleDivider.h"

#include <QString>
#include <QStringList>

class CageBoolean {
public:
    struct Result {
        int  intersected = 0;
        int  dropped     = 0;
        int  failed      = 0;
        int  skipped     = 0;
        int  noiseDropped= 0;
        double discardedVolume = 0.0;
        int  lumpsMerged = 0;
        int  lumpsWelded = 0;
        int  orphans     = 0;
        int  multiPart   = 0;
        bool modelClosed = true;
        QStringList problems;
        QStringList notes;
    };

    static Result intersectAll(QVector<PuzzlePiece> *pieces,
                               const MeshData &model,
                               double fineNess = 20.0);

    enum class Outcome {
        Ok,
        EmptyCell,
        Failed
    };

    static Outcome intersect(const MeshData &piece, const MeshData &model,
                             MeshData *out, double fineNess, QString *error);

    static QStringList describe(const Result &r);

    static bool unite(const MeshData &a, const MeshData &b, MeshData *out,
                      double fineNess, QString *error);

    static QVector<MeshData> components(const MeshData &m);
};
