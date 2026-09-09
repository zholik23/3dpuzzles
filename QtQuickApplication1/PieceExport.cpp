#include "PieceExport.h"

// Qt before IRIT, deliberately. The IRIT headers define bare names that collide
// with Qt's - QDir fails to compile ("_mkdir already defined") when it is the
// other way round.
#include <QDataStream>
#include <QFile>
#include <QFileInfo>

#include <cmath>

#include "IritGuard.h"
#include "IritSolid.h"

extern "C" {
#include "inc_irit/misc_lib.h"     // IritMiscMatGenUnitMat
}

namespace {

enum class Kind { Itd, Obj, Stl, Unknown };

Kind kindOf(const QString &path)
{
    const QString ext = QFileInfo(path).suffix().toLower();
    if (ext == QStringLiteral("itd"))                              return Kind::Itd;
    if (ext == QStringLiteral("obj"))                              return Kind::Obj;
    if (ext == QStringLiteral("stl"))                              return Kind::Stl;
    return Kind::Unknown;
}

// The write runs inside IritGuard, so the context holds only POD - a longjmp
// out of an IRIT fatal error cannot unwind C++ destructors.
struct SaveCtx {
    IritPrsrObjectStruct *list;
    const char           *path;
    int                   kind;        // Kind, as an int
    int                   ok;
};

void doSave(void *v)
{
    SaveCtx *c = static_cast<SaveCtx *>(v);
    c -> ok = 0;

    switch (Kind(c -> kind)) {
    case Kind::Itd:
        // Indent 0: the file is for machines and for round-tripping, and an
        // indented dump of a few hundred thousand vertices is enormous.
        IritPrsrPutObjectToFile3(c -> path, c -> list, 0);
        c -> ok = 1;                   // this one reports failure by fatal error
        break;

    case Kind::Obj:
        // UniqueVertices merges the shared vertices a boolean leaves behind;
        // without it a piece exports with duplicates on every cut face.
        c -> ok = IritPrsrOBJSaveFile(c -> list, c -> path,
                                      FALSE,    // no warning pop-ups
                                      TRUE,     // triangulate convex polys
                                      TRUE);    // unique vertices
        break;

    default:
        break;
    }
}


// STL is written here rather than through IritPrsrSTLSaveFile, which has two
// behaviours that make it unusable for this:
//
//   * a binary STL cannot hold more than one named solid, so it silently
//     switches to one-file-per-object even when asked for a single file;
//   * it derives those file names by truncating the path at the FIRST '.' it
//     finds - strchr where strrchr was meant. Any dot earlier in the path (a
//     directory such as ".claude", or "v1.2") sends the output somewhere else
//     entirely. Saving to ...\tmp\export\pieces.stl wrote 1.stl..6.stl into
//     C:\Users\Admin instead.
//
// The format itself is 84 bytes of header plus 50 per triangle, so writing it
// directly costs less than working around either problem.
bool writeStlBinary(const QVector<const MeshData *> &meshes, const QString &path,
                    QString *error)
{
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly)) {
        if (error) *error = QStringLiteral("Cannot write %1: %2")
                                .arg(path, f.errorString());
        return false;
    }

    QDataStream out(&f);
    out.setByteOrder(QDataStream::LittleEndian);
    out.setFloatingPointPrecision(QDataStream::SinglePrecision);

    char header[80] = { 0 };
    qsnprintf(header, sizeof header, "Puzzle Divider - %d piece(s)",
              int(meshes.size()));
    out.writeRawData(header, 80);

    quint32 total = 0;
    for (const MeshData *m : meshes)
        total += quint32(m->triangleCount());
    out << total;

    for (const MeshData *m : meshes) {
        for (int t = 0; t + 2 < m->tris.size(); t += 3) {
            const float *A = &m->pos[m->tris[t + 0] * 3];
            const float *B = &m->pos[m->tris[t + 1] * 3];
            const float *C = &m->pos[m->tris[t + 2] * 3];

            // Facet normal from the winding, normalised. A zero-area triangle
            // gets a zero normal, which is what slicers expect to ignore.
            const double ux = B[0] - A[0], uy = B[1] - A[1], uz = B[2] - A[2];
            const double vx = C[0] - A[0], vy = C[1] - A[1], vz = C[2] - A[2];
            double nx = uy * vz - uz * vy;
            double ny = uz * vx - ux * vz;
            double nz = ux * vy - uy * vx;
            const double len = std::sqrt(nx * nx + ny * ny + nz * nz);
            if (len > 0.0) { nx /= len; ny /= len; nz /= len; }

            out << float(nx) << float(ny) << float(nz);
            for (const float *V : { A, B, C })
                out << V[0] << V[1] << V[2];
            out << quint16(0);
        }
    }

    f.close();
    if (f.error() != QFileDevice::NoError) {
        if (error) *error = QStringLiteral("Failed while writing %1: %2")
                                .arg(path, f.errorString());
        return false;
    }
    return true;
}


// Writes one already-built IRIT object (a list of pieces, or a single piece) to
// `path` in the given format, under the guard.
bool writeIritList(IritPrsrObjectStruct *list, const QString &path, Kind kind,
                   QString *error)
{
    const QByteArray pathBytes = path.toLocal8Bit();

    SaveCtx c;
    c.list = list;
    c.path = pathBytes.constData();
    c.kind = int(kind);
    c.ok   = 0;

    const bool guarded = IritGuard::run(&c, doSave);
    if (!guarded || !c.ok) {
        if (error)
            *error = guarded
                ? QStringLiteral("IRIT could not write %1.").arg(path)
                : QStringLiteral("IRIT failed while writing %1 - %2")
                      .arg(path, IritGuard::lastError());
        return false;
    }
    return true;
}

// The per-piece file name for `index`, derived from the name the user chose.
// Built with QFileInfo rather than by hand so a dot elsewhere in the path is
// harmless - exactly what IRIT's own STL writer gets wrong.
QString pieceFileName(const QString &path, int index, const QString &ext)
{
    const QFileInfo fi(path);
    return QStringLiteral("%1/%2_piece_%3.%4")
               .arg(fi.path(), fi.completeBaseName())
               .arg(index, 3, 10, QLatin1Char('0'))
               .arg(ext);
}


// Moves each piece into its own cell of a grid on the XY plane, sitting on
// z = 0. Cells are sized by the largest piece and separated by a real gap, so
// no two pieces can touch however oddly shaped they are - which is the only
// thing a slicer will accept as "these are separate objects".
void spreadApart(QVector<MeshData> *pieces)
{
    const int n = pieces->size();
    if (n == 0)
        return;

    double cellW = 0.0, cellD = 0.0;
    for (const MeshData &m : *pieces) {
        cellW = qMax(cellW, double(m.bmax[0]) - double(m.bmin[0]));
        cellD = qMax(cellD, double(m.bmax[1]) - double(m.bmin[1]));
    }
    // A tenth of the larger cell side: visibly apart at any scale, without
    // spreading the plate so wide that nothing fits on a bed.
    const double gap = 0.10 * qMax(cellW, cellD);
    const int cols = qMax(1, int(std::ceil(std::sqrt(double(n)))));

    for (int i = 0; i < n; ++i) {
        MeshData &m = (*pieces)[i];
        const int col = i % cols, row = i / cols;

        // Align each piece's minimum corner to its cell origin: every piece
        // then lies inside a cellW x cellD footprint, and consecutive cells are
        // a full gap apart.
        const double dx = col * (cellW + gap) - double(m.bmin[0]);
        const double dy = row * (cellD + gap) - double(m.bmin[1]);
        const double dz = -double(m.bmin[2]);          // rest it on the bed

        for (int v = 0; v + 2 < m.pos.size(); v += 3) {
            m.pos[v + 0] += float(dx);
            m.pos[v + 1] += float(dy);
            m.pos[v + 2] += float(dz);
        }
        m.edges.clear();
        m.finalize();
    }
}

} // namespace

QStringList PieceExport::nameFilters()
{
    return {
        QStringLiteral("IRIT native (*.itd)"),
        QStringLiteral("Wavefront OBJ (*.obj)"),
        QStringLiteral("STL for printing (*.stl)"),
    };
}

bool PieceExport::canWrite(const QString &path)
{
    return kindOf(path) != Kind::Unknown;
}

bool PieceExport::save(const QVector<PuzzlePiece> &pieces, const QString &path,
                       bool separateFiles, bool spread,
                       Result *result, QString *error)
{
    Result r;

    const Kind kind = kindOf(path);
    if (kind == Kind::Unknown) {
        if (error)
            *error = QStringLiteral("Cannot write \"%1\" - use .itd, .obj or .stl.")
                         .arg(QFileInfo(path).fileName());
        return false;
    }
    r.format = QFileInfo(path).suffix().toUpper();

    if (pieces.isEmpty()) {
        if (error) *error = QStringLiteral("There are no pieces to save.");
        return false;
    }

    // Orient every piece up front. Consumers of all three formats - slicers
    // included - read outward normals as "solid", so an inside-out piece would
    // print as its own negative.
    QVector<MeshData> oriented;
    oriented.reserve(pieces.size());
    for (int i = 0; i < pieces.size(); ++i) {
        if (pieces[i].mesh.triangleCount() == 0) {
            ++r.skipped;
            r.problems << QStringLiteral("piece %1: no geometry").arg(i);
            continue;
        }
        MeshData o = pieces[i].mesh;
        IritSolid::orientConsistently(&o);
        oriented.append(o);
    }
    if (oriented.isEmpty()) {
        if (error) *error = QStringLiteral("No piece had any geometry.");
        return false;
    }

    // Spread before anything is written, so every format and both file layouts
    // see the same geometry.
    if (spread) {
        spreadApart(&oriented);
        r.spread = true;
    }

    const QString ext = QFileInfo(path).suffix().toLower();

    // ---- one file per piece ---------------------------------------------
    //
    // Worth having for every format, not only STL. The pieces are written in
    // MODEL coordinates - reassembled they occupy exactly the original model's
    // space - so a single file looks like the undivided model in any viewer
    // that merges groups on import. Separate files cannot be misread that way.
    if (separateFiles) {
        for (int i = 0; i < oriented.size(); ++i) {
            const QString one = pieceFileName(path, i, ext);

            if (kind == Kind::Stl) {
                QVector<const MeshData *> single { &oriented[i] };
                if (!writeStlBinary(single, one, error))
                    return false;
            } else {
                IritPrsrObjectStruct *obj =
                    IritSolid::fromMesh(oriented[i], IritSolid::Winding::Outward);
                if (obj == NULL) {
                    ++r.skipped;
                    r.problems << QStringLiteral("piece %1: no usable polygons").arg(i);
                    continue;
                }
                IritFree(obj -> ObjName);
                obj -> ObjName = IritMiscStrdup(
                    qPrintable(QStringLiteral("piece_%1")
                                   .arg(i, 3, 10, QLatin1Char('0'))));

                IritPrsrObjectStruct *one_list = IritPrsrGenLISTObject(obj);
                IritPrsrListObjectInsert(one_list, 1, NULL);

                const bool ok = writeIritList(one_list, one, kind, error);
                IritPrsrFreeObject(one_list);
                if (!ok)
                    return false;
            }
            ++r.written;
        }
        r.separateFiles = true;
        if (result) *result = r;
        return true;
    }

    // ---- everything in one file -----------------------------------------
    if (kind == Kind::Stl) {
        QVector<const MeshData *> all;
        for (const MeshData &m : oriented)
            all.append(&m);
        if (!writeStlBinary(all, path, error))
            return false;
        r.written = oriented.size();
        if (result) *result = r;
        return true;
    }

    // One IRIT object per piece, gathered into a list, so the pieces stay
    // distinguishable inside the file - an OBJ group, an ITD object.
    IritPrsrObjectStruct *list = IritPrsrGenLISTObject(NULL);
    int at = 0;
    for (int i = 0; i < oriented.size(); ++i) {
        IritPrsrObjectStruct *obj =
            IritSolid::fromMesh(oriented[i], IritSolid::Winding::Outward);
        if (obj == NULL) {
            ++r.skipped;
            r.problems << QStringLiteral("piece %1: no usable polygons").arg(i);
            continue;
        }
        IritFree(obj -> ObjName);
        obj -> ObjName = IritMiscStrdup(
            qPrintable(QStringLiteral("piece_%1").arg(i, 3, 10, QLatin1Char('0'))));
        IritPrsrListObjectInsert(list, at++, obj);
        ++r.written;
    }
    IritPrsrListObjectInsert(list, at, NULL);

    if (r.written == 0) {
        IritPrsrFreeObject(list);
        if (error) *error = QStringLiteral("None of the %1 piece(s) could be "
                                           "converted for writing.")
                                .arg(pieces.size());
        return false;
    }

    if (!writeIritList(list, path, kind, error)) {
        IritPrsrFreeObject(list);
        return false;
    }
    IritPrsrFreeObject(list);

    if (result) *result = r;
    return true;
}
