#include "AppController.h"
#include "CadLoader.h"
#include "MeshDivider.h"
#include "PuzzleDivider.h"
#include "Trivariate.h"
#include "IritGuard.h"
#include "MeshView.h"

#include <QDebug>
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QQmlError>
#include <qqml.h>

#include <QFileInfo>
#include <QImage>
#include <QPainter>
#include <cstdio>

// Headless check of the loader alone:  QtQuickApplication1.exe --probe FILE...
// No window, no QML - so a loader regression cannot be confused with a UI one.
// Exit code is the number of files that failed.
static int probe(const QStringList &files)
{
    int failed = 0;
    for (const QString &f : files) {
        MeshData mesh;
        QString  error;
        if (CadLoader::load(f, &mesh, &error)) {
            std::printf("OK    %-28s  %-5s  v=%-7d t=%-7d e=%-7d obj=%-3d ff=%d\n"
                        "                                    bbox [%g %g %g] .. [%g %g %g]\n",
                        qPrintable(QFileInfo(f).fileName()),
                        qPrintable(mesh.sourceKind),
                        mesh.vertexCount(), mesh.triangleCount(), mesh.edgeCount(),
                        mesh.objectCount, mesh.freeformCount,
                        mesh.bmin[0], mesh.bmin[1], mesh.bmin[2],
                        mesh.bmax[0], mesh.bmax[1], mesh.bmax[2]);
        }
        else {
            std::printf("FAIL  %-28s  %s\n",
                        qPrintable(QFileInfo(f).fileName()), qPrintable(error));
            ++failed;
        }
        std::fflush(stdout);
    }
    return failed;
}

// Headless render:  QtQuickApplication1.exe --render MODEL OUT.png
// Paints a MeshView straight onto a QImage - no window, no QML - so the
// rasteriser can be checked on its own, and diffed between builds.
static int render(const QString &modelPath, const QString &outPath)
{
    MeshData mesh;
    QString  error;
    if (!CadLoader::load(modelPath, &mesh, &error)) {
        std::printf("FAIL  %s\n", qPrintable(error));
        return 1;
    }

    MeshView view;
    view.setWidth(900);
    view.setHeight(700);
    view.setMesh(mesh);

    QImage img(900, 700, QImage::Format_RGB32);
    {
        QPainter p(&img);
        view.paint(&p);
    }
    if (!img.save(outPath)) {
        std::printf("FAIL  could not write %s\n", qPrintable(outPath));
        return 1;
    }
    std::printf("OK    %s -> %s  (%d tris)\n",
                qPrintable(QFileInfo(modelPath).fileName()),
                qPrintable(outPath), mesh.triangleCount());
    return 0;
}

// Headless division:
//   --divide SOURCE MODE ARGS... [--png OUT.png]
// where SOURCE is a primitive name (Sphere/Torus/Cylinder/Cone/Box), a .itd
// holding a trivariate, or "cage:MODEL" for a bounding cage over a mesh; and
// MODE is  uniform NU NV NW | jitter NU NV NW PCT SEED | fit BX BY BZ MAX.
static int divideCli(const QStringList &a)
{
    if (a.size() < 2) {
        std::printf("usage: --divide SOURCE MODE ARGS... [--png OUT.png]\n");
        return 2;
    }

    const QString source = a.at(0);
    QString err;
    Trivariate tv;

    if (Trivariate::primitiveKinds().contains(source, Qt::CaseInsensitive)) {
        for (const QString &k : Trivariate::primitiveKinds())
            if (k.compare(source, Qt::CaseInsensitive) == 0)
                tv = Trivariate::primitive(k, &err);
    }
    else if (source.startsWith(QStringLiteral("cage:"))) {
        MeshData mesh;
        const QString path = source.mid(5);
        if (!CadLoader::load(path, &mesh, &err)) {
            std::printf("FAIL  %s\n", qPrintable(err));
            return 1;
        }
        tv = Trivariate::boundingCage(mesh, &err);
    }
    else {
        tv = Trivariate::fromFile(source, &err);
    }

    if (!tv.isValid()) {
        std::printf("FAIL  %s\n", qPrintable(err));
        return 1;
    }

    double dom[6];
    int    ord[3];
    tv.domain(dom);
    tv.orders(ord);
    std::printf("TV    %-34s domain u[%g %g] v[%g %g] w[%g %g] orders %d/%d/%d\n",
                qPrintable(tv.label()),
                dom[0], dom[1], dom[2], dom[3], dom[4], dom[5],
                ord[0], ord[1], ord[2]);

    const QString mode = a.size() > 1 ? a.at(1) : QStringLiteral("uniform");
    const auto num = [&a](int i, double dflt) {
        return (a.size() > i) ? a.at(i).toDouble() : dflt;
    };

    DivisionSpec spec;
    if (mode == QStringLiteral("jitter")) {
        const int c[3] = { int(num(2, 2)), int(num(3, 2)), int(num(4, 2)) };
        spec = PuzzleDivider::jittered(tv, c, num(5, 25) / 100.0, quint32(num(6, 1)));
    }
    else if (mode == QStringLiteral("fit")) {
        const double b[3] = { num(2, 1), num(3, 1), num(4, 1) };
        spec = PuzzleDivider::toBuildVolume(tv, b, int(num(5, 16)));
    }
    else {
        const int c[3] = { int(num(2, 2)), int(num(3, 2)), int(num(4, 2)) };
        spec = PuzzleDivider::uniform(tv, c);
    }
    std::printf("SPEC  %s\n", qPrintable(spec.note));
    for (int ax = 0; ax < 3; ++ax) {
        std::printf("      %c splits:", "uvw"[ax]);
        for (double s : spec.splits[ax])
            std::printf(" %.4f", s);
        std::printf("%s\n", spec.splits[ax].isEmpty() ? " (none)" : "");
    }

    QVector<PuzzlePiece> pieces;
    QString warn;
    if (!PuzzleDivider::divide(tv, spec, 12.0, &pieces, &warn)) {
        std::printf("FAIL  %s\n", qPrintable(warn));
        return 1;
    }

    float maxSide = 0.0f, minSide = 1e30f;
    int   tris = 0;
    for (const PuzzlePiece &p : pieces) {
        maxSide = qMax(maxSide, p.largestSide());
        minSide = qMin(minSide, p.largestSide());
        tris   += p.mesh.triangleCount();
    }
    const int shared = PuzzleDivider::adjacency(pieces, spec).size();

    std::printf("OK    %d pieces, %d shared faces, %d triangles; "
                "piece side max %.4f min %.4f\n",
                int(pieces.size()), shared, tris, maxSide, minSide);
    if (!warn.isEmpty())
        std::printf("WARN  %s\n", qPrintable(warn));

    const int png = a.indexOf(QStringLiteral("--png"));
    if (png >= 0 && a.size() > png + 1) {
        MeshView view;
        view.setWidth(900);
        view.setHeight(700);
        view.setPieces(pieces);
        view.setExplode(0.55);

        QImage img(900, 700, QImage::Format_RGB32);
        {
            QPainter p(&img);
            view.paint(&p);
        }
        if (img.save(a.at(png + 1)))
            std::printf("PNG   %s\n", qPrintable(a.at(png + 1)));
        else
            std::printf("FAIL  could not write %s\n", qPrintable(a.at(png + 1)));
    }
    std::fflush(stdout);
    return 0;
}

// Headless mesh division:
//   --meshdivide MODEL MODE ARGS... [--png OUT.png]
// MODE is  uniform NX NY NZ | jitter NX NY NZ PCT SEED
//        | fit BX BY BZ MAX | balanced NX NY NZ
// Unlike --divide this clips the model itself, so the pieces keep its shape.
static int meshDivideCli(const QStringList &a)
{
    if (a.size() < 2) {
        std::printf("usage: --meshdivide MODEL MODE ARGS... [--png OUT.png]\n");
        return 2;
    }

    MeshData mesh;
    QString err;
    if (!CadLoader::load(a.at(0), &mesh, &err)) {
        std::printf("FAIL  %s\n", qPrintable(err));
        return 1;
    }
    std::printf("MESH  %-28s v=%d t=%d  extent %g x %g x %g\n",
                qPrintable(QFileInfo(a.at(0)).fileName()),
                mesh.vertexCount(), mesh.triangleCount(),
                mesh.bmax[0] - mesh.bmin[0],
                mesh.bmax[1] - mesh.bmin[1],
                mesh.bmax[2] - mesh.bmin[2]);

    const QString mode = a.at(1);
    const auto num = [&a](int i, double dflt) {
        return (a.size() > i) ? a.at(i).toDouble() : dflt;
    };

    // Recursive split: no per-axis plane lists, so it takes its own path.
    if (mode == QStringLiteral("bsp")) {
        const double dom[6] = { mesh.bmin[0], mesh.bmax[0],
                                mesh.bmin[1], mesh.bmax[1],
                                mesh.bmin[2], mesh.bmax[2] };
        const QVector<CellBox> cells =
            PuzzleDivider::buildBspCells(dom, int(num(2, 24)), 0.28,
                                         quint32(num(3, 7)));
        QVector<PuzzlePiece> bp;
        QString bw;
        if (!MeshDivider::divideCells(mesh, cells, &bp, &bw)) {
            std::printf("FAIL  %s\n", qPrintable(bw));
            return 1;
        }
        double vlo = 1e300, vhi = 0.0;
        for (const CellBox &c : cells) { vlo = qMin(vlo, c.volume()); vhi = qMax(vhi, c.volume()); }

        const auto links = PuzzleDivider::adjacencyOfBoxes(bp, 1e-6);
        QVector<int> val(bp.size(), 0);
        for (const auto &l : links) { ++val[l.a]; ++val[l.b]; }
        int nlo = 1 << 30, nhi = 0;
        for (int v : val) { nlo = qMin(nlo, v); nhi = qMax(nhi, v); }

        float slo = 1e30f, shi = 0.0f;
        for (const PuzzlePiece &p : bp) { slo = qMin(slo, p.largestSide()); shi = qMax(shi, p.largestSide()); }

        std::printf("BSP   %d cells -> %d pieces\n", int(cells.size()), int(bp.size()));
        std::printf("      cell volume  min %.4g  max %.4g  spread %.2fx\n",
                    vlo, vhi, vlo > 0 ? vhi / vlo : 0.0);
        std::printf("      piece side   min %.4g  max %.4g  spread %.2fx\n",
                    slo, shi, slo > 0 ? shi / slo : 0.0);
        std::printf("      neighbours per piece: min %d  max %d  (%d shared faces)\n",
                    bp.isEmpty() ? 0 : nlo, nhi, int(links.size()));
        if (!bw.isEmpty())
            std::printf("NOTE  %s\n", qPrintable(bw));

        const int png2 = a.indexOf(QStringLiteral("--png"));
        if (png2 >= 0 && a.size() > png2 + 1) {
            MeshView view;
            view.setWidth(900); view.setHeight(700);
            view.setPieces(bp);
            view.setExplode(0.5);
            QImage img(900, 700, QImage::Format_RGB32);
            { QPainter pr(&img); view.paint(&pr); }
            std::printf(img.save(a.at(png2 + 1)) ? "PNG   %s\n" : "FAIL  %s\n",
                        qPrintable(a.at(png2 + 1)));
        }
        std::fflush(stdout);
        return 0;
    }

    MeshDivisionSpec spec;
    if (mode == QStringLiteral("jitter")) {
        const int c[3] = { int(num(2, 2)), int(num(3, 2)), int(num(4, 2)) };
        spec = MeshDivider::jittered(mesh, c, num(5, 25) / 100.0, quint32(num(6, 1)));
    }
    else if (mode == QStringLiteral("fit")) {
        const double b[3] = { num(2, 1), num(3, 1), num(4, 1) };
        spec = MeshDivider::toBuildVolume(mesh, b, int(num(5, 16)));
    }
    else if (mode == QStringLiteral("balanced")) {
        const int c[3] = { int(num(2, 2)), int(num(3, 2)), int(num(4, 2)) };
        spec = MeshDivider::balanced(mesh, c);
    }
    else {
        const int c[3] = { int(num(2, 2)), int(num(3, 2)), int(num(4, 2)) };
        spec = MeshDivider::uniform(mesh, c);
    }

    std::printf("SPEC  %s\n", qPrintable(spec.note));
    for (int ax = 0; ax < 3; ++ax) {
        std::printf("      %c cuts:", "xyz"[ax]);
        for (double s : spec.planes[ax])
            std::printf(" %.4f", s);
        std::printf("%s\n", spec.planes[ax].isEmpty() ? " (none)" : "");
    }

    QVector<PuzzlePiece> pieces;
    QString warn;
    if (!MeshDivider::divide(mesh, spec, &pieces, &warn)) {
        std::printf("FAIL  %s\n", qPrintable(warn));
        return 1;
    }

    // Volume is the honest check on a division: the pieces must add up to the
    // model. A missing or wrongly wound cap shows here as a shortfall.
    const auto volumeOf = [](const MeshData &m) {
        double v = 0.0;
        for (int t = 0; t + 2 < m.tris.size(); t += 3) {
            const float *a = &m.pos[m.tris[t + 0] * 3];
            const float *b = &m.pos[m.tris[t + 1] * 3];
            const float *c = &m.pos[m.tris[t + 2] * 3];
            v += double(a[0]) * (double(b[1]) * double(c[2]) - double(b[2]) * double(c[1]))
               - double(a[1]) * (double(b[0]) * double(c[2]) - double(b[2]) * double(c[0]))
               + double(a[2]) * (double(b[0]) * double(c[1]) - double(b[1]) * double(c[0]));
        }
        return qAbs(v) / 6.0;
    };

    int    tris = 0;
    float  maxSide = 0.0f;
    double pieceVol = 0.0;
    for (const PuzzlePiece &p : pieces) {
        tris     += p.mesh.triangleCount();
        maxSide   = qMax(maxSide, p.largestSide());
        pieceVol += volumeOf(p.mesh);
    }
    const double modelVol = volumeOf(mesh);
    // Only meaningful once every piece is closed: the divergence integral over
    // an open shell is not a volume, so a run with open pieces can still land
    // on 100% by accident.
    std::printf("VOL   model %.6g  pieces %.6g  (%.3f%%)%s\n",
                modelVol, pieceVol,
                modelVol > 0.0 ? 100.0 * pieceVol / modelVol : 0.0,
                warn.contains(QStringLiteral("open shells"))
                    ? "  [not conclusive - some pieces are open]" : "");
    std::printf("OK    %d pieces of %d cells, %d triangles, largest side %.4f\n",
                int(pieces.size()), spec.cellCount(), tris, maxSide);
    if (!warn.isEmpty())
        std::printf("NOTE  %s\n", qPrintable(warn));

    const int png = a.indexOf(QStringLiteral("--png"));
    if (png >= 0 && a.size() > png + 1) {
        MeshView view;
        view.setWidth(900);
        view.setHeight(700);
        view.setPieces(pieces);
        view.setExplode(0.5);
        QImage img(900, 700, QImage::Format_RGB32);
        { QPainter p(&img); view.paint(&p); }
        std::printf(img.save(a.at(png + 1)) ? "PNG   %s\n" : "FAIL  %s\n",
                    qPrintable(a.at(png + 1)));
    }
    std::fflush(stdout);
    return 0;
}

int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);
    app.setApplicationName(QStringLiteral("Puzzle Divider"));

    // Before any other IRIT call: stop a malformed file from taking the
    // process down through IRIT's default exit()-on-error handlers.
    IritGuard::installHandlers();

    {
        QStringList args = QGuiApplication::arguments();
        const int p = args.indexOf(QStringLiteral("--probe"));
        if (p >= 0)
            return probe(args.mid(p + 1));

        const int md = args.indexOf(QStringLiteral("--meshdivide"));
        if (md >= 0)
            return meshDivideCli(args.mid(md + 1));

        const int dv = args.indexOf(QStringLiteral("--divide"));
        if (dv >= 0)
            return divideCli(args.mid(dv + 1));

        const int r = args.indexOf(QStringLiteral("--render"));
        if (r >= 0) {
            if (args.size() < r + 3) {
                std::printf("usage: --render MODEL OUT.png\n");
                return 2;
            }
            return render(args.at(r + 1), args.at(r + 2));
        }
    }

    qmlRegisterType<MeshView>("PuzzleDivider", 1, 0, "MeshView");
    qmlRegisterUncreatableType<AppController>(
        "PuzzleDivider", 1, 0, "AppController",
        QStringLiteral("AppController is provided as the 'app' context property."));

    QQmlApplicationEngine engine;

    // Without this, a QML error means the window silently never appears and
    // the process exits with -1 - the failure mode is invisible under the
    // Windows subsystem.
    QObject::connect(&engine, &QQmlApplicationEngine::warnings,
                     [](const QList<QQmlError> &warnings) {
                         for (const QQmlError &w : warnings)
                             qCritical().noquote() << w.toString();
                     });

    AppController controller;
    engine.rootContext()->setContextProperty(QStringLiteral("app"), &controller);

    engine.load(QUrl(QStringLiteral("qrc:/qt/qml/qtquickapplication1/main.qml")));
    if (engine.rootObjects().isEmpty()) {
        qCritical() << "Failed to load main.qml - see the errors above.";
        return -1;
    }

    // Allow "app.exe model.stl" so a file can be loaded without the dialog.
    const QStringList args = QGuiApplication::arguments();
    if (args.size() > 1)
        controller.loadPath(args.at(1));

    return app.exec();
}
