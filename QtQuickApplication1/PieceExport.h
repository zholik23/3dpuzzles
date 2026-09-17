#pragma once
//
// PieceExport - writes the divided model out to a file.
//
// Every piece is written as its own named object rather than as one merged soup
// of triangles: a puzzle is a set of separate solids, and a consumer that cannot
// tell them apart cannot print them or reason about assembly.
//
// The format follows the extension - .itd round-trips back into this app, .obj is
// widely readable, .stl is what a slicer wants.
//
#include "MeshData.h"
#include "PuzzleDivider.h"

#include <QString>
#include <QStringList>
#include <QVector>

class PieceExport {
public:
    struct Result {
        int         written = 0;
        int         skipped = 0;
        QString     format;
        bool        separateFiles = false;
        bool        spread        = false;
        QStringList problems;
    };

    static bool save(const QVector<PuzzlePiece> &pieces, const QString &path,
                     bool separateFiles, bool spread,
                     Result *result, QString *error);

    static QStringList nameFilters();

    static bool canWrite(const QString &path);
};
