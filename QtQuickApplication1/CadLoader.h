#pragma once
//
// CadLoader - one entry point for every CAD file the app accepts.
//
// Everything is routed through IRIT's own parsers so that a native .itd
// (with freeform surfaces or trivariates) and a dumb triangle soup end up
// in exactly the same MeshData. Freeform geometry is tessellated on load
// purely for display; the original trivariates are what Increment 3/4 will
// actually divide.
//
// No IRIT headers leak out of here - callers only see MeshData.
//
#include "MeshData.h"
#include <QString>
#include <QStringList>

class CadLoader {
public:
    enum Format {
        Fmt_Unknown = 0,
        Fmt_STL,        // .stl            - binary or ASCII, auto-detected
        Fmt_OBJ,        // .obj            - Wavefront
        Fmt_IRIT,       // .itd .ibd .imd  - native IRIT, incl. freeforms
        Fmt_IGES,       // .igs .iges
        Fmt_STEP        // .stp .step      - recognised, but IRIT has no reader
    };

    // Loads `path` into `out`. Returns false and fills `error` on failure.
    static bool load(const QString &path, MeshData *out, QString *error);

    static Format     detectFormat(const QString &path);
    static QString    formatName(Format f);
    static QStringList nameFilters();      // for the QML FileDialog
};
