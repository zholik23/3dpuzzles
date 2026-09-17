//
// CadLoader - implementation: format detection and loading through IRIT's
// parsers, under IritGuard so a malformed file returns false instead of killing
// the process.
//
// Qt headers must come before IritGuard.h and IritMesh.h. Those pull in the
// IRIT C headers, which define bare names that collide with Qt's: with the
// order reversed, QDir fails to compile with "_mkdir already defined".
//

#include "CadLoader.h"

#include <QByteArray>
#include <QDebug>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QSet>
#include <QTemporaryFile>
#include <QtGlobal>

#include "IritGuard.h"
#include "IritMesh.h"

// Format from the extension, looking one suffix further back: IRIT accepts .Z
// and .gz wrapped data files.
CadLoader::Format CadLoader::detectFormat(const QString &path)
{
    QString s = QFileInfo(path).suffix().toLower();
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

// Binary STL by size, not by header: binary files very often still begin with
// the word "solid". A valid binary STL is exactly 84 + 50*N bytes.
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
        return false;
    return f.size() == qint64(84) + qint64(nTri) * 50;
}

struct LoadCtx {
    const char           *path;
    int                   format;
    int                   binaryStl;
    IritPrsrObjectStruct *result;
};

// Parse, tessellate and harvest, inside IritGuard; POD only.
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

// The statements IRIT's OBJ reader knows, from IPO2IParseStatements[] in
// prsr_lib/obj_irit.c.
static bool objKeywordKnown(const QByteArray &kw)
{
    static const QSet<QByteArray> known = {
        "v", "vt", "vn", "vp", "cstype", "deg", "bmat", "step", "p", "l", "f",
        "curv", "curv2", "surf", "con", "g", "group", "s", "mg", "o", "bevel",
        "c_interp", "d_interp", "lod", "usemtl", "mtllib", "res", "usemap",
        "maplib", "shadow_obj", "trace_obj", "ctech", "stech", "cmd", "call",
        "bzp", "bsp", "cdc", "cdp"
    };
    return known.contains(kw);
}

static bool objIsSpace(char c)
{
    return c == ' ' || c == '\t' || c == '\r' || c == '\n';
}

static QByteArray objKeywordOf(const QByteArray &trimmed)
{
    if (trimmed.isEmpty() || trimmed.startsWith('#'))
        return QByteArray();
    int i = 0;
    while (i < trimmed.size() && !objIsSpace(trimmed[i]))
        ++i;
    return trimmed.left(i).toLower();
}

// The distinct statements in `path` that IRIT would refuse, in file order.
static QList<QByteArray> objUnknownKeywords(const QString &path)
{
    QList<QByteArray> unknown;
    QSet<QByteArray>  seen;

    QFile f(path);
    if (!f.open(QIODevice::ReadOnly))
        return unknown;

    while (!f.atEnd()) {
        const QByteArray kw = objKeywordOf(f.readLine().trimmed());
        if (kw.isEmpty() || objKeywordKnown(kw) || seen.contains(kw))
            continue;
        seen.insert(kw);
        unknown.append(kw);
    }
    return unknown;
}

// Copies `src` to `dst` without the lines whose keyword is in `drop`. A dropped
// line ending in a backslash continues onto the next, which goes too.
static bool writeObjWithout(const QString &src, QFile *dst,
                            const QSet<QByteArray> &drop)
{
    QFile in(src);
    if (!in.open(QIODevice::ReadOnly))
        return false;

    bool droppingContinuation = false;
    while (!in.atEnd()) {
        const QByteArray line    = in.readLine();
        const QByteArray trimmed = line.trimmed();

        if (droppingContinuation) {
            droppingContinuation = trimmed.endsWith('\\');
            continue;
        }
        const QByteArray kw = objKeywordOf(trimmed);
        if (!kw.isEmpty() && drop.contains(kw)) {
            droppingContinuation = trimmed.endsWith('\\');
            continue;
        }
        if (dst -> write(line) != line.size())
            return false;
    }
    return true;
}

// IRIT treats an unrecognised OBJ statement as FATAL rather than skipping it:
// the keyword falls through IPO2IParseStatements[] and hits
// IPO2IErrorAndExit("Can't recognize the statement ..."), which longjmps out and
// abandons the whole file. That is why a model a slicer opens can fail here -
// the ABC dataset writes a non-standard "vc" (vertex colour) line on every one
// of its models, and the parse dies on it.
//
// Returns the path to hand IRIT: the original when the file is clean, otherwise
// a filtered copy. The copy is written beside the source so a relative mtllib
// still resolves, falling back to the temp directory if that folder is
// read-only. `tmp` owns the copy and deletes it when it goes out of scope.
static QString sanitiseObj(const QString &path, QTemporaryFile *tmp,
                           QStringList *dropped)
{
    const QList<QByteArray> unknown = objUnknownKeywords(path);
    if (unknown.isEmpty())
        return path;

    QSet<QByteArray> drop;
    for (const QByteArray &kw : unknown) {
        drop.insert(kw);
        dropped -> append(QString::fromLatin1(kw));
    }

    tmp -> setFileTemplate(QFileInfo(path).absolutePath() +
                           QStringLiteral("/irit_XXXXXX.obj"));
    if (!tmp -> open()) {
        tmp -> setFileTemplate(
            QDir::temp().filePath(QStringLiteral("irit_XXXXXX.obj")));
        if (!tmp -> open()) {
            dropped -> clear();
            return path;          // cannot filter - let IRIT report what it sees
        }
    }

    const bool ok = writeObjWithout(path, tmp, drop);
    tmp -> close();
    if (!ok) {
        dropped -> clear();
        return path;
    }
    return tmp -> fileName();
}

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

    // Kept alive until the parse is done: it owns the filtered copy, if one was
    // needed, and deletes it on the way out.
    QTemporaryFile objTmp;
    QString parsePath = path;
    if (fmt == Fmt_OBJ) {
        QStringList dropped;
        parsePath = sanitiseObj(path, &objTmp, &dropped);
        if (!dropped.isEmpty())
            qWarning().noquote()
                << QStringLiteral("LOAD  %1: statements IRIT cannot parse were "
                                  "left out: %2")
                       .arg(fi.fileName(), dropped.join(QStringLiteral(", ")));
    }

    const QByteArray path8 = QFile::encodeName(parsePath);
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

    const QStringList contents = IritMesh::inventory(lc.result);

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
