#pragma once
//
// CadLoader - one entry point for every CAD file the app accepts.
//
// Everything is routed through IRIT's own parsers, so a native .itd and a dumb
// triangle soup end up as the same MeshData. No IRIT headers leak out of here:
// callers see only MeshData.
//
#include "MeshData.h"
#include <QString>
#include <QStringList>

class CadLoader {
public:
    enum Format {
        Fmt_Unknown = 0,
        Fmt_STL,
        Fmt_OBJ,
        Fmt_IRIT,
        Fmt_IGES,
        Fmt_STEP
    };

    static bool load(const QString &path, MeshData *out, QString *error);

    static Format     detectFormat(const QString &path);
    static QString    formatName(Format f);
    static QStringList nameFilters();
};
