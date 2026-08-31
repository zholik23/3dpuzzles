#include "CadLoader.h"
#include "IritGuard.h"
#include "IritMesh.h"          // brings in the IRIT C headers

#include <QByteArray>
#include <QFile>
#include <QFileInfo>
#include <QtGlobal>

// ---------------------------------------------------------------- formats --

CadLoader::Format CadLoader::detectFormat(const QString &path)
{
    QString s = QFileInfo(path).suffix().toLower();
    // IRIT accepts .Z/.gz wrapped data files; look one suffix further back.
    if (s == QLatin1String("z") || s == QLatin1String("gz"))
        s = QFileInfo(QFileInfo(path).completeBaseName()).suffix().toLower();

    if (s == QLatin1String("stl"))                                return Fmt_STL;
    if (s == QLatin1String("obj"))                                return Fmt_OBJ;
    if (s == QLatin1String("itd") || s == QLatin1String("ibd") ||
        s == QLatin1String("imd") || s == QLatin1String("idat"))  return Fmt_IRIT;
    if (s == QLatin1String("igs") || s == QLatin1String("iges"))  return Fmt_IGES;
    if (s == QLatin1String("stp") || s == QLatin1String("step") ||
        s == QLatin1String("p21"))                                return Fmt_STEP;
    return Fmt_Unknown;
}

QString CadLoader::formatName(Format f)
{
    switch (f) {
        case Fmt_STL:  return QStringLiteral("STL");
        case Fmt_OBJ:  return QStringLiteral("OBJ");
        case Fmt_IRIT: return QStringLiteral("IRIT");
        case Fmt_IGES: return QStringLiteral("IGES");
        case Fmt_STEP: return QStringLiteral("STEP");
        default:       return QStringLiteral("?");
    }
}

QStringList CadLoader::nameFilters()
{
    return {
        QStringLiteral("All supported (*.stl *.obj *.itd *.ibd *.imd *.igs *.iges)"),
        QStringLiteral("Meshes (*.stl *.obj)"),
        QStringLiteral("IRIT native (*.itd *.ibd *.imd)"),
        QStringLiteral("IGES (*.igs *.iges)"),
        QStringLiteral("All files (*)")
    };
}

// Binary STL files very often still begin with the word "solid", so the header
// text is not a reliable discriminator. Size is: a valid binary STL is exactly
// 84 + 50*N bytes, with N stored as a uint32 at offset 80.
static bool stlLooksBinary(const QString &path)
{
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly))
        return false;
    if (f.size() < 84)
        return false;
    if (!f.seek(80))
        return false;
    quint32 nTri = 0;
    if (f.read(reinterpret_cast<char *>(&nTri), 4) != 4)
        return false;                                    // little-endian host
    return f.size() == qint64(84) + qint64(nTri) * 50;
}

// ------------------------------------------------------- guarded IRIT work --
//
// Everything that can trip an IRIT fatal error runs inside IritGuard::run,
// which longjmps out on failure. These callbacks therefore touch only POD.

struct LoadCtx {
    const char           *path;
    int                   format;
    int                   binaryStl;
    IritPrsrObjectStruct *result;
};

static void doLoad(void *v)
{
    LoadCtx *c = static_cast<LoadCtx *>(v);
    c -> result = NULL;

    switch (c -> format) {
        case CadLoader::Fmt_STL: {
            IritPrsrSTLLoadDfltFileParamsStruct P = IritPrsrSTLLoadDfltParams;
            P.BinarySTL  = c -> binaryStl;
            P.EndianSwap = FALSE;
            P.NormalFlip = FALSE;
            c -> result = IritPrsrSTLLoadFile(c -> path, &P);
            break;
        }
        case CadLoader::Fmt_OBJ: {
            IritPrsrOBJLoadDfltFileParamsStruct P = IritPrsrOBJLoadDfltParams;
            P.WarningMsgs = FALSE;
            c -> result = IritPrsrOBJLoadFile(c -> path, &P);
            break;
        }
        case CadLoader::Fmt_IGES: {
            IritPrsrIgesLoadDfltFileParamsStruct P = IritPrsrIgesLoadDfltParams;
            c -> result = IritPrsrIgesLoadFile(c -> path, &P);
            break;
        }
        case CadLoader::Fmt_IRIT:
        default:
            c -> result = IritPrsrGetObjects2(c -> path);
            break;
    }
}

// ------------------------------------------------------------------- load --

bool CadLoader::load(const QString &path, MeshData *out, QString *error)
{
    const auto fail = [error](const QString &msg) {
        if (error != nullptr)
            *error = msg;
        return false;
    };

    *out = MeshData();

    const QFileInfo fi(path);
    if (!fi.exists() || !fi.isFile())
        return fail(QStringLiteral("File not found: %1").arg(path));
    if (fi.size() == 0)
        return fail(QStringLiteral("File is empty: %1").arg(fi.fileName()));

    const Format fmt = detectFormat(path);
    if (fmt == Fmt_STEP)
        return fail(QStringLiteral(
            "STEP (.stp/.step) has no reader in IRIT. Convert it to STL or OBJ "
            "first - FreeCAD, or File > Export from your CAD tool."));

    out -> sourceKind = formatName(fmt == Fmt_Unknown ? Fmt_IRIT : fmt);

    // Phase 1: parse the file.
    const QByteArray path8 = QFile::encodeName(path);
    LoadCtx lc;
    lc.path      = path8.constData();
    lc.format    = fmt;
    lc.binaryStl = (fmt == Fmt_STL) ? (stlLooksBinary(path) ? 1 : 0) : 0;
    lc.result    = NULL;

    if (!IritGuard::run(&lc, doLoad))
        return fail(QStringLiteral("Could not read %1 - %2")
                        .arg(fi.fileName(), IritGuard::lastError()));

    if (lc.result == NULL)
        return fail(fmt == Fmt_Unknown
            ? QStringLiteral("Unrecognised file type '.%1'. Supported: STL, OBJ, "
                             "IRIT (.itd/.ibd/.imd), IGES.").arg(fi.suffix())
            : QStringLiteral("The %1 parser returned no objects for %2.")
                  .arg(formatName(fmt), fi.fileName()));

    // What was actually in the file, recorded before tessellate() consumes it.
    const QStringList contents = IritMesh::inventory(lc.result);

    // Phases 2 and 3: tessellate freeforms, triangulate, harvest. Consumes
    // lc.result either way.
    QString tessErr;
    if (!IritMesh::tessellate(lc.result, out, 20.0, &tessErr))
        return fail(QStringLiteral("Tessellating %1 failed - %2")
                        .arg(fi.fileName(), tessErr));

    if (out -> isEmpty())
        return fail(QStringLiteral("%1 parsed cleanly but held no displayable "
                                   "geometry. It contains: %2.")
                        .arg(fi.fileName(),
                             contents.isEmpty() ? QStringLiteral("nothing")
                                                : contents.join(QStringLiteral(", "))));

    return true;
}
