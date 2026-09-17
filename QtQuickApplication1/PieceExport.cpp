//
// PieceExport - implementation: the .itd, .obj and .stl writers and the
// spread-apart layout used for slicing.
//
// Qt headers must come before the IRIT ones. The IRIT C headers define bare names
// that collide with Qt's, and QDir fails to compile with "_mkdir already defined"
// if the order is reversed.
//

#include "PieceExport.h"

#include <QDataStream>
#include <QFile>
#include <QFileInfo>

#include <cmath>

#include "IritGuard.h"
#include "IritSolid.h"

extern "C" {
#include "inc_irit/misc_lib.h"
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

struct SaveCtx {
    IritPrsrObjectStruct *list;
    const char           *path;
    int                   kind;
    int                   ok;
};

// The write, inside IritGuard; POD only.
void doSave(void *v)
{
    SaveCtx *c = static_cast<SaveCtx *>(v);
    c -> ok = 0;

    switch (Kind(c -> kind)) {
    case Kind::Itd:
        IritPrsrPutObjectToFile3(c -> path, c -> list, 0);
        c -> ok = 1;
        break;

    case Kind::Obj:
        c -> ok = IritPrsrOBJSaveFile(c -> list, c -> path,
                                      FALSE,
                                      TRUE,
                                      TRUE);
        break;

    default:
        break;
    }
}

// STL written directly rather than through IritPrsrSTLSaveFile, which cannot
// hold more than one named solid in a binary file and truncates output paths at
// the first '.' rather than the last.
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

// Writes one already-built IRIT object - a list of pieces, or a single piece -
// in the given format, under the guard.
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

QString pieceFileName(const QString &path, int index, const QString &ext)
{
    const QFileInfo fi(path);
    return QStringLiteral("%1/%2_piece_%3.%4")
               .arg(fi.path(), fi.completeBaseName())
               .arg(index, 3, 10, QLatin1Char('0'))
               .arg(ext);
}

// Moves the pieces apart so a slicer sees separate solids on the plate.
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
    const double gap = 0.10 * qMax(cellW, cellD);
    const int cols = qMax(1, int(std::ceil(std::sqrt(double(n)))));

    for (int i = 0; i < n; ++i) {
        MeshData &m = (*pieces)[i];
        const int col = i % cols, row = i / cols;

        const double dx = col * (cellW + gap) - double(m.bmin[0]);
        const double dy = row * (cellD + gap) - double(m.bmin[1]);
        const double dz = -double(m.bmin[2]);

        for (int v = 0; v + 2 < m.pos.size(); v += 3) {
            m.pos[v + 0] += float(dx);
            m.pos[v + 1] += float(dy);
            m.pos[v + 2] += float(dz);
        }
        m.edges.clear();
        m.finalize();
    }
}

}

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

    if (spread) {
        spreadApart(&oriented);
        r.spread = true;
    }

    const QString ext = QFileInfo(path).suffix().toLower();

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
