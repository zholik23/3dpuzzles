#pragma once
//
// PieceExport - writing the divided model out to a file.
//
// The pieces only exist inside the app until they can be handed to a slicer or
// back to IRIT, so this is the step that makes a division usable rather than
// merely viewable.
//
// Every piece is written as its OWN named object rather than as one merged
// soup of triangles. That distinction is the whole point of the file: a puzzle
// is a set of separate solids, and a consumer that cannot tell them apart
// cannot print them, arrange them on a plate, or reason about assembly.
//
// The format is chosen by the file's extension:
//   .itd   IRIT's native format - a list object holding one polygon object per
//          piece. The only format here that round-trips back into this app.
//   .obj   Wavefront, one group per piece. Widely readable.
//   .stl   One solid per piece. What a slicer wants for printing.
//
#include "MeshData.h"
#include "PuzzleDivider.h"

#include <QString>
#include <QStringList>
#include <QVector>

class PieceExport {
public:
    struct Result {
        int         written = 0;   // pieces that made it into the file
        int         skipped = 0;   // empty or unusable pieces
        QString     format;        // what was actually written
        bool        separateFiles = false;
        bool        spread        = false;
        QStringList problems;
    };

    // Writes `pieces` to `path`, picking the format from the extension.
    //
    // `separateFiles` writes one file per piece beside `path`, named
    // <stem>_piece_000.<ext>, instead of a single file holding all of them.
    //
    // This matters for every format, not only STL. Pieces are written in MODEL
    // coordinates, so reassembled they fill exactly the original model's space:
    // a combined file looks like the undivided model in any viewer that merges
    // groups on import, however correctly the pieces are named inside it.
    // `spread` lays the pieces out side by side on the XY plane instead of
    // leaving them where they belong in the model.
    //
    // Needed because slicers merge on import. A puzzle written in model
    // coordinates has its pieces touching face to face - that is what a puzzle
    // IS - and Bambu Studio, PrusaSlicer and Orca all read touching shells as
    // one object, however the file names or groups them. Naming cannot fix
    // that; only distance can. Each piece is moved into its own cell of a grid
    // with a clear gap, and dropped onto z = 0 ready to print.
    //
    // It destroys the assembled positions, so it is for printing only - never
    // for a file you intend to reassemble or measure.
    static bool save(const QVector<PuzzlePiece> &pieces, const QString &path,
                     bool separateFiles, bool spread,
                     Result *result, QString *error);

    // Extensions this can write, as Qt name filters, for the save dialog.
    static QStringList nameFilters();

    // True when `path`'s extension is one of them.
    static bool canWrite(const QString &path);
};
