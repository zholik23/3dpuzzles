#include "AppController.h"
#include "PlannerFigure.h"
#include "AssemblyDivider/AssemblyDivider.h"
#include "CadLoader.h"
#include <functional>

#include "AssemblyOrder.h"
#include "CageBoolean.h"
#include <QElapsedTimer>

#include "MaterialField.h"
#include "PieceExport.h"
#include "IritSolid.h"
#include "IritJoint.h"
#include "PlannerJoints.h"
#include "MeshDivider.h"
#include "PuzzleDivider.h"
#include "Trivariate.h"
#include "IritGuard.h"
#include "MeshView.h"

#include <QDebug>
#include <QHash>
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
    
    
        const QString outPath = "C:\Users\Admin\Documents\IRIT_to_Gcode-main\docs";

        // Example: If you want to save sub-regions based on your PuzzlePiece parameter boxes (p0 and p1):
        // You can loop through pieces, extract their subRegion, and save them.
        // Or if you just want to save the master trivariate:
        QString err1;
        if (tv.saveToFile(outPath, &err1)) {
            std::printf("ITD   %s\n", qPrintable(outPath));
        }
        else {
            std::printf("FAIL  %s\n", qPrintable(err1));
        }
    
    std::fflush(stdout);
    return 0;
}

// Volume by the divergence theorem, plus the count of edges used by exactly one
// triangle. The volume is only meaningful for a closed mesh - which is exactly
// why the open-edge count is reported next to it.
struct MeshStats { double volume; int openEdges; int tris; };

static MeshStats meshStats(const MeshData &m)
{
    MeshStats st = { 0.0, 0, m.triangleCount() };

    for (int t = 0; t + 2 < m.tris.size(); t += 3) {
        const float *a = &m.pos[m.tris[t + 0] * 3];
        const float *b = &m.pos[m.tris[t + 1] * 3];
        const float *c = &m.pos[m.tris[t + 2] * 3];
        st.volume += double(a[0]) * (double(b[1]) * double(c[2]) - double(b[2]) * double(c[1]))
                   - double(a[1]) * (double(b[0]) * double(c[2]) - double(b[2]) * double(c[0]))
                   + double(a[2]) * (double(b[0]) * double(c[1]) - double(b[1]) * double(c[0]));
    }
    st.volume = st.volume / 6.0;   // SIGNED: negative means inward-wound

    // Weld by position first: the boolean emits its own vertices, so index
    // identity says nothing about whether two triangles share an edge.
    QHash<QString, int> weld;
    QVector<int>        id(m.vertexCount(), -1);
    for (int v = 0; v < m.vertexCount(); ++v) {
        const QString key = QStringLiteral("%1_%2_%3")
            .arg(double(m.pos[v * 3 + 0]), 0, 'f', 4)
            .arg(double(m.pos[v * 3 + 1]), 0, 'f', 4)
            .arg(double(m.pos[v * 3 + 2]), 0, 'f', 4);
        auto it = weld.find(key);
        if (it == weld.end()) { weld.insert(key, v); id[v] = v; }
        else                  { id[v] = it.value(); }
    }

    QHash<quint64, int> use;
    for (int t = 0; t + 2 < m.tris.size(); t += 3)
        for (int e = 0; e < 3; ++e) {
            const int u = id[m.tris[t + e]], v = id[m.tris[t + (e + 1) % 3]];
            if (u == v) continue;
            const quint64 key = (quint64(qMin(u, v)) << 32) | quint32(qMax(u, v));
            use[key] += 1;
        }
    for (auto it = use.constBegin(); it != use.constEnd(); ++it)
        if (it.value() == 1)
            ++st.openEdges;

    return st;
}

// Cuts Elber-style pin/hole pairs into every shared face, in place.
//   --joints [--pinmin MM] [--sink FRACTION]
// Reports per-piece failures rather than hiding them: a boolean that declines
// almost always means the piece is not closed, which is worth knowing.
static void jointsCli(const QStringList &a, QVector<PuzzlePiece> *pieces)
{
    const int at = a.indexOf(QStringLiteral("--joints"));
    if (at < 0 || pieces == NULL || pieces->size() < 2)
        return;

    JointParams jp;
    const int pinAt = a.indexOf(QStringLiteral("--pinmin"));
    if (pinAt >= 0 && a.size() > pinAt + 1)
        jp.minPinThickness = a.at(pinAt + 1).toDouble();
    const int sinkAt = a.indexOf(QStringLiteral("--sink"));
    if (sinkAt >= 0 && a.size() > sinkAt + 1)
        jp.baseSink = a.at(sinkAt + 1).toDouble();
    if (a.contains(QStringLiteral("--noclear"))) {
        // Pin and hole identical. The volume the assembly then loses is the
        // boolean's own error, with the designed clearance taken out of the
        // picture - not a printable setting, a measurement aid.
        jp.clearanceXY = 1.0;
        jp.clearanceZ  = 1.0;
    }

    // Same route the app takes: the planner picks the faces, the geometry
    // follows. Pegging every face would leave the puzzle welded shut.
    const Planner::Graph graph = Planner::build(*pieces, 1e-6, 0.0);
    const Planner::TranslationalBlocking bare;
    const Planner::Plan order = Planner::extract(graph, bare);
    if (!order.complete) {
        std::printf("JOINT no removal order under the translational model - "
                    "nothing to place joints along\n");
        return;
    }
    const Planner::JointSet chosen = Planner::chooseAlongOrder(graph, order);

    int skipped = 0;
    const QVector<QVector<JointPlacement> > plan =
        IritJoint::planPlacementsFor(*pieces, graph, chosen, jp, &skipped);

    {   // Orientation probe. IRIT's booleans decide inside from the polygon
        // winding, so a tool wound inward turns a union into a bite.
        MeshData tool;
        QString  terr;
        if (IritJoint::preview(&tool, jp, &terr)) {
            const MeshStats ts = meshStats(tool);
            std::printf("JOINT tool: signed volume %.5g, %d tris, %d open edges\n",
                        ts.volume, ts.tris, ts.openEdges);
        }
        else {
            std::printf("JOINT tool probe failed: %s\n", qPrintable(terr));
        }
        if (!pieces->isEmpty()) {
            const MeshStats ps = meshStats((*pieces)[0].mesh);
            std::printf("JOINT piece 0: signed volume %.5g, %d tris, %d open edges\n",
                        ps.volume, ps.tris, ps.openEdges);
        }
    }

    double volBefore = 0.0, volAfter = 0.0;
    int    openBefore = 0, openAfter = 0, damaged = 0;
    for (const PuzzlePiece &q : *pieces) {
        const MeshStats st = meshStats(q.mesh);
        volBefore  += st.volume;
        openBefore += st.openEdges;
    }

    int done = 0, failed = 0, cuts = 0, noCut = 0;
    for (int i = 0; i < pieces->size() && i < plan.size(); ++i) {
        if (plan[i].isEmpty())
            continue;
        const MeshStats was = meshStats((*pieces)[i].mesh);
        int     applied = 0, refused = 0;
        QString err;
        if (IritJoint::apply(&(*pieces)[i].mesh, plan[i], jp, &err, &applied,
                             &refused)) {
            ++done;
            cuts += applied;
            noCut += refused;
            const MeshStats now = meshStats((*pieces)[i].mesh);
            // A pin adds volume and a hole removes some, but neither should be
            // anywhere near the size of the piece. A big swing means the
            // boolean ate the piece rather than modifying it.
            if (was.volume > 1e-9 &&
                qAbs(now.volume - was.volume) > 0.25 * was.volume) {
                ++damaged;
                std::printf("JOINT piece %d volume %.4g -> %.4g (%.0f%%), "
                            "open edges %d -> %d\n",
                            i, was.volume, now.volume,
                            100.0 * now.volume / was.volume,
                            was.openEdges, now.openEdges);
            }
        }
        else {
            ++failed;
            std::printf("JOINT FAIL piece %d: %s\n", i, qPrintable(err));
        }
    }

    for (const PuzzlePiece &q : *pieces) {
        const MeshStats st = meshStats(q.mesh);
        volAfter  += st.volume;
        openAfter += st.openEdges;
    }
    double smallest = 1e30;
    for (const QVector<JointPlacement> &list : plan)
        for (const JointPlacement &j : list)
            smallest = qMin(smallest, IritJoint::thinnestFeature(jp, j.size));

    const Planner::JointedBlocking jointed(chosen);
    QString   why;
    const int broken = Planner::replay(graph, jointed, order, &why);

    std::printf("JOINT %d of %d faces pegged (planner-chosen), %d pieces cut, "
                "%d booleans, %d faces too small, %d pieces failed\n",
                Planner::countJoints(chosen), graph.contactCount(),
                done, cuts, skipped, failed);
    if (noCut > 0)
        std::printf("JOINT WARNING %d boolean(s) DECLINED - the tool missed the "
                    "piece. A missing hole leaves a pin with nowhere to go and "
                    "the puzzle will not close.\n", noCut);
    std::printf("JOINT volume %.5g -> %.5g (%.2f%%), open edges %d -> %d, "
                "%d piece(s) changed volume by more than 25%%\n",
                volBefore, volAfter,
                volBefore > 1e-9 ? 100.0 * volAfter / volBefore : 0.0,
                openBefore, openAfter, damaged);
    std::printf("JOINT %s\n", broken < 0
        ? "order holds with the pegs fitted - valid under jointed translational "
          "DBG; real collision check pending"
        : qPrintable(QStringLiteral("ORDER BROKEN - %1").arg(why)));
    if (smallest < 1e29)
        std::printf("JOINT thinnest pin %.2f mm%s\n", smallest,
                    smallest < 1.2 ? "  - UNDER 3 extrusions, will not print" : "");
    std::fflush(stdout);
}

// Assembly planner, stages 1-3, on whatever the divider just produced:
//   --plan [--stress] [--full]
// --stress swaps in the deliberately over-blocking test model, so the
// non-assemblable branch can be seen to work. --full prints every piece rather
// than the first few.
static void plannerCli(const QStringList &a, const QVector<PuzzlePiece> &pieces)
{
    if (!a.contains(QStringLiteral("--plan")))
        return;
    const int lim = a.contains(QStringLiteral("--full")) ? -1 : 8;

    const Planner::Graph g = Planner::build(pieces, 1e-6, 0.0);
    for (const QString &line : g.describe(lim))
        std::printf("%s\n", qPrintable(line));

    Planner::TranslationalBlocking translational;
    Planner::AlwaysBlocking        stress;
    const Planner::BlockingModel &model =
        a.contains(QStringLiteral("--stress"))
            ? static_cast<const Planner::BlockingModel &>(stress)
            : static_cast<const Planner::BlockingModel &>(translational);

    for (const QString &line : Planner::describeBlocking(g, model, lim))
        std::printf("%s\n", qPrintable(line));

    const Planner::Plan plan = Planner::extract(g, model);
    for (const QString &line : plan.describe(lim))
        std::printf("%s\n", qPrintable(line));

    if (!plan.complete) {
        std::fflush(stdout);
        return;
    }

    // Stage 4a: what the divider currently does - a joint on every shared face.
    const Planner::JointSet every = Planner::allContacts(g);
    const Planner::JointedBlocking jointedAll(every);
    const Planner::Plan planAll = Planner::extract(g, jointedAll);
    std::printf("STAGE 4a joint on every contact (%d joints)\n",
                Planner::countJoints(every));
    if (planAll.complete) {
        std::printf("         still disassemblable - %d pieces ordered\n",
                    int(planAll.assembly.size()));
    }
    else {
        std::printf("         NON-ASSEMBLABLE: %d piece(s) stuck\n",
                    int(planAll.stuck.size()));
        for (const QString &line : Planner::explainOverConstrained(g, every, lim < 0 ? -1 : lim))
            std::printf("%s\n", qPrintable(line));
    }

    // Stage 4b: joints placed to suit the order found in stage 3.
    const Planner::JointSet chosen = Planner::chooseAlongOrder(g, plan);
    for (const QString &line : Planner::describeJoints(g, chosen, lim))
        std::printf("%s\n", qPrintable(line));

    const Planner::JointedBlocking jointedChosen(chosen);
    QString   why;
    const int bad = Planner::replay(g, jointedChosen, plan, &why);
    if (bad < 0)
        std::printf("         the stage 3 order still holds with these joints "
                    "fitted - valid under jointed translational DBG; real "
                    "collision check pending\n");
    else
        std::printf("         ORDER BROKEN by the chosen joints - %s\n",
                    qPrintable(why));

    int uncovered = 0;
    for (int i = 0; i < g.contactCount(); ++i)
        if (!chosen[i])
            ++uncovered;
    std::printf("         %d of %d contacts carry no joint - those faces are "
                "held by their neighbours, not pegged\n",
                uncovered, g.contactCount());

    std::fflush(stdout);
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
        QVector<CellBox> cells;
        QVector<PuzzlePiece> bp;
        QString bw;
        int absorbed = 0;
        if (!MeshDivider::divideBspAbsorbing(mesh, int(num(2, 24)), 0.28,
                                             quint32(num(3, 7)),
                                             &bp, &cells, &absorbed, &bw)) {
            std::printf("FAIL  %s\n", qPrintable(bw));
            return 1;
        }
        if (absorbed > 0)
            std::printf("ABSORB %d crumb(s) merged back into their neighbour\n",
                        absorbed);
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

        // Planning is Planner (stages 1-3) via --plan, below. The older
        // AssemblyPlanner is no longer called from here: it assigns joints on a
        // spanning tree and checks a spiral/dovetail lock, neither of which is
        // the current design, and its verdict contradicted the new planner's on
        // the same pieces.
        if (!bw.isEmpty())
            std::printf("NOTE  %s\n", qPrintable(bw));

        jointsCli(a, &bp);
        plannerCli(a, bp);

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

    jointsCli(a, &pieces);
    plannerCli(a, pieces);

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

// Stage A of the assemblable divider:
//   --assemble MODEL [N] [SEED]
// Splits to a target count and prints the contact graph. Nothing is tested for
// assemblability yet - that is Stage B.
static int assembleCli(const QStringList &a)
{
    if (a.isEmpty()) {
        std::printf("usage: --assemble MODEL [N] [SEED]\n");
        return 2;
    }

    MeshData solid;
    QString err;
    if (!CadLoader::load(a.at(0), &solid, &err)) {
        std::printf("FAIL  %s\n", qPrintable(err));
        return 1;
    }

    const int     target = (a.size() > 1) ? a.at(1).toInt()  : 9;
    const quint32 seed   = (a.size() > 2) ? quint32(a.at(2).toUInt()) : 7u;

    std::printf("MODEL %s  v=%d t=%d  extent %g x %g x %g\n",
                qPrintable(QFileInfo(a.at(0)).fileName()),
                solid.vertexCount(), solid.triangleCount(),
                solid.bmax[0] - solid.bmin[0],
                solid.bmax[1] - solid.bmin[1],
                solid.bmax[2] - solid.bmin[2]);

    DividedSolid divided;
    if (!AssemblyDivider::splitToTarget(solid, target, seed, &divided, &err)) {
        std::printf("FAIL  %s\n", qPrintable(err));
        return 1;
    }

    for (const QString &line : AssemblyDivider::describeStageA(divided))
        std::printf("%s\n", qPrintable(line));

    std::fflush(stdout);
    return 0;
}

// Elber Section 5 end to end:  --cage MODEL [N] [SEED] [--png BEFORE.png AFTER.png] [--figures DIR]
// Wraps the model in a bounding-cage trivariate, divides the cage, then
// intersects each boxy cage piece with the original model.
// --field MODEL [RES ...] : how the voxel measurement converges with
// resolution, and what it costs. No booleans, so it runs in seconds.
static int fieldCli(const QStringList &a)
{
    if (a.isEmpty()) {
        std::printf("usage: --field MODEL [RES ...]\n");
        return 2;
    }
    MeshData model;
    QString err;
    if (!CadLoader::load(a.at(0), &model, &err)) {
        std::printf("FAIL  %s\n", qPrintable(err));
        return 1;
    }

    // The reference the voxels are trying to match.
    const double exact = qAbs(IritSolid::signedVolume(model));
    double ext[3];
    for (int k = 0; k < 3; ++k)
        ext[k] = double(model.bmax[k]) - double(model.bmin[k]);
    std::printf("MODEL %s  bbox %.4g x %.4g x %.4g  exact volume %.6g\n",
                qPrintable(QFileInfo(a.at(0)).fileName()),
                ext[0], ext[1], ext[2], exact);
    std::printf("%-6s %-22s %12s %14s %10s %8s\n",
                "res", "grid", "cubes", "measured vol", "error", "ms");

    QVector<int> list;
    for (int i = 1; i < a.size(); ++i) {
        bool ok = false;
        const int v = a.at(i).toInt(&ok);
        if (ok) list.append(v);
    }
    if (list.isEmpty())
        list = { 8, 12, 16, 24, 32, 48, 64, 96, 128, 160, 192 };

    for (int res : list) {
        QElapsedTimer t;
        t.start();
        const MaterialField f = MaterialField::build(model, res);
        const qint64 ms = t.elapsed();
        if (!f.isValid())
            continue;
        const double vol = f.total();
        // Does the grid still see the model as ONE body? A cube coarser than a
        // limb makes that limb vanish, and the model falls apart in the grid
        // even though the mesh is perfectly connected. Aggregate volume can
        // stay accurate while this is already broken, so it is checked apart.
        const double lo[3] = { 0.0, 0.0, 0.0 };
        const double hi[3] = { ext[0], ext[1], ext[2] };
        const bool whole = f.isConnected(lo, hi);

        // Probe the exact call the splitter makes: halve the box on each axis
        // and ask whether each half is one lump. On a convex model every answer
        // must be "yes", so any "no" here is a bug in the test, not a fact
        // about the model.
        QString halves;
        for (int ax = 0; ax < 3; ++ax) {
            double m1[3] = { ext[0], ext[1], ext[2] };
            double l2[3] = { 0, 0, 0 };
            m1[ax] = l2[ax] = 0.5 * ext[ax];
            int b1 = 0, t1 = 0, b2 = 0, t2 = 0;
            const int L1 = f.lumpStats(lo, m1, &b1, &t1);
            const int L2 = f.lumpStats(l2, hi, &b2, &t2);
            halves += QStringLiteral(" %1:%2lumps(%3/%4)|%5lumps(%6/%7)")
                          .arg(QChar('X' + ax))
                          .arg(L1).arg(b1).arg(t1)
                          .arg(L2).arg(b2).arg(t2);
        }

        std::printf("%-6d %-22s %12d %14.6g %9.2f%% %9.3f %8lld  %s\n",
                    res,
                    qPrintable(QStringLiteral("%1 x %2 x %3")
                        .arg(f.dim(0)).arg(f.dim(1)).arg(f.dim(2))),
                    f.cellCount(), vol,
                    100.0 * (vol - exact) / qMax(1e-9, exact),
                    f.side(0), (long long)ms,
                    whole ? "one body" : "FALLS APART");
        std::printf("        halves  lumps(biggest/total):%s\n", qPrintable(halves));
        std::fflush(stdout);
    }
    return 0;
}

// --shot MODEL N SEED WHAT OUT.png : one render, camera pinned to the cage box
// so every WHAT registers pixel-for-pixel with every other.
//   WHAT = model | cells | pieces | cell:K | piece:K
static int shotCli(const QStringList &a)
{
    if (a.size() < 5) {
        std::printf("usage: --shot MODEL N SEED WHAT OUT.png\n");
        return 2;
    }
    MeshData model;
    QString err;
    if (!CadLoader::load(a.at(0), &model, &err)) {
        std::printf("FAIL  %s\n", qPrintable(err));
        return 1;
    }
    const int n    = a.at(1).toInt();
    const int seed = a.at(2).toInt();
    const QString what = a.at(3);
    const QString out  = a.at(4);

    Trivariate cage = Trivariate::boundingCage(model, &err);
    if (!cage.isValid()) { std::printf("FAIL  %s\n", qPrintable(err)); return 1; }

    double dom[6];
    cage.domain(dom);
    double ext[3];
    for (int k = 0; k < 3; ++k)
        ext[k] = qMax(1e-9, double(model.bmax[k]) - double(model.bmin[k]));
    const double wdom[6] = { 0.0, ext[0], 0.0, ext[1], 0.0, ext[2] };

    const MaterialField field = MaterialField::build(model);
    const QVector<CellBox> world =
        PuzzleDivider::buildBspCells(wdom, qMax(1, n), 0.35, quint32(seed), 0.0,
                                     field.isValid() ? &field : nullptr);
    QVector<CellBox> cells;
    for (const CellBox &w : world) {
        CellBox c;
        for (int k = 0; k < 3; ++k) {
            const double lo = dom[k * 2], span = dom[k * 2 + 1] - lo;
            c.lo[k] = lo + span * (w.lo[k] / ext[k]);
            c.hi[k] = lo + span * (w.hi[k] / ext[k]);
        }
        cells.append(c);
    }

    QVector<PuzzlePiece> pieces;
    QString warn;
    if (!PuzzleDivider::divideCells(cage, cells, 12.0, &pieces, &warn)) {
        std::printf("FAIL  %s\n", qPrintable(warn));
        return 1;
    }

    const bool wantTrimmed = what.startsWith(QStringLiteral("piece"));
    if (wantTrimmed)
        CageBoolean::intersectAll(&pieces, model, 12.0);

    QVector<PuzzlePiece> show;
    if (what == QStringLiteral("model")) {
        PuzzlePiece p;
        p.mesh = model;
        show.append(p);
    } else if (what.contains(QLatin1Char(':'))) {
        const int k = what.section(QLatin1Char(':'), 1).toInt();
        if (k < 0 || k >= pieces.size()) {
            std::printf("FAIL  index %d of %d\n", k, int(pieces.size()));
            return 1;
        }
        show.append(pieces.at(k));
    } else {
        show = pieces;
    }

    MeshView v;
    v.setWidth(1100);
    v.setHeight(1100);
    v.setPieces(show);
    v.setExplode(0.0);
    v.setYaw(28);
    v.setPitch(-14);
    // Shaded only. The wireframe overlay is high-frequency detail that survives
    // any fade, so a ghosted model drawn with edges reads as speckle and fights
    // the piece laid over it.
    v.setShaded(true);
    v.setShowEdges(false);
    // Pinned to the CAGE, so a single cell and the whole model land in the same
    // place at the same scale and can be composited.
    const float bmin[3] = { model.bmin[0], model.bmin[1], model.bmin[2] };
    const float bmax[3] = { model.bmax[0], model.bmax[1], model.bmax[2] };
    v.setFixedBounds(bmin, bmax);

    QImage img(1100, 1100, QImage::Format_ARGB32);
    img.fill(Qt::white);
    { QPainter pr(&img); v.paint(&pr); }
    std::printf(img.save(out) ? "SHOT  %s (%s)\n" : "FAIL  %s (%s)\n",
                qPrintable(out), qPrintable(what));
    std::fflush(stdout);
    return 0;
}

// --order MODEL [N] [SEED] : divide, then ask whether the pieces can be
// assembled. Prints the three steps and the verdict.
static int orderCli(const QStringList &a)
{
    if (a.isEmpty()) {
        std::printf("usage: --order MODEL [N] [SEED]\n");
        return 2;
    }
    MeshData model;
    QString err;
    if (!CadLoader::load(a.at(0), &model, &err)) {
        std::printf("FAIL  %s\n", qPrintable(err));
        return 1;
    }
    const int n    = (a.size() > 1) ? a.at(1).toInt() : 6;
    const int seed = (a.size() > 2) ? a.at(2).toInt() : 7;

    Trivariate cage = Trivariate::boundingCage(model, &err);
    if (!cage.isValid()) { std::printf("FAIL  %s\n", qPrintable(err)); return 1; }

    double dom[6];
    cage.domain(dom);
    double ext[3];
    for (int k = 0; k < 3; ++k)
        ext[k] = qMax(1e-9, double(model.bmax[k]) - double(model.bmin[k]));
    const double wdom[6] = { 0.0, ext[0], 0.0, ext[1], 0.0, ext[2] };

    const MaterialField field = MaterialField::build(model);
    const QVector<CellBox> world =
        PuzzleDivider::buildBspCells(wdom, qMax(1, n), 0.35, quint32(seed), 0.0,
                                     field.isValid() ? &field : nullptr);
    QVector<CellBox> cells;
    for (const CellBox &w : world) {
        CellBox c;
        for (int k = 0; k < 3; ++k) {
            const double lo = dom[k * 2], span = dom[k * 2 + 1] - lo;
            c.lo[k] = lo + span * (w.lo[k] / ext[k]);
            c.hi[k] = lo + span * (w.hi[k] / ext[k]);
        }
        cells.append(c);
    }

    QVector<PuzzlePiece> pieces;
    QString warn;
    if (!PuzzleDivider::divideCells(cage, cells, 12.0, &pieces, &warn)) {
        std::printf("FAIL  %s\n", qPrintable(warn));
        return 1;
    }
    const CageBoolean::Result br = CageBoolean::intersectAll(&pieces, model, 12.0);
    std::printf("DIVIDE  %s \u00b7 asked %d \u00b7 %d piece(s), %d failed\n",
                qPrintable(QFileInfo(a.at(0)).fileName()), n,
                int(pieces.size()), br.failed);
    std::printf("\n");

    const AssemblyOrder::Result r = AssemblyOrder::run(pieces, 1e-4);
    for (const QString &line : r.describe())
        std::printf("%s\n", qPrintable(line));

    std::fflush(stdout);
    return r.assemblable ? 0 : 3;
}

static int cageCli(const QStringList &a)
{
    if (a.isEmpty()) {
        std::printf("usage: --cage MODEL [N] [SEED] [--png BEFORE.png AFTER.png] [--figures DIR]\n");
        return 2;
    }

    MeshData model;
    QString err;
    if (!CadLoader::load(a.at(0), &model, &err)) {
        std::printf("FAIL  %s\n", qPrintable(err));
        return 1;
    }
    const int n    = (a.size() > 1) ? a.at(1).toInt() : 8;
    const int seed = (a.size() > 2) ? a.at(2).toInt() : 7;

    Trivariate cage = Trivariate::boundingCage(model, &err);
    if (!cage.isValid()) {
        std::printf("FAIL  %s\n", qPrintable(err));
        return 1;
    }

    // Recursive split in world proportions, mapped back to the parameter
    // domain - the same path the GUI takes, so the CLI measures what the app
    // actually does. A grid spec cannot honour an arbitrary piece count.
    double dom[6];
    cage.domain(dom);

    double ext[3];
    for (int a = 0; a < 3; ++a)
        ext[a] = qMax(1e-9, double(model.bmax[a]) - double(model.bmin[a]));
    const double wdom[6] = { 0.0, ext[0], 0.0, ext[1], 0.0, ext[2] };

    // Resolution is overridable so the choice of 96 can be justified by
    // measurement rather than asserted.
    const int res = qEnvironmentVariableIsSet("MATFIELD_RES")
                        ? qEnvironmentVariableIntValue("MATFIELD_RES") : 96;
    QElapsedTimer fieldTimer;
    fieldTimer.start();
    const MaterialField field = MaterialField::build(model, res);
    const qint64 fieldMs = fieldTimer.elapsed();
    if (field.isValid())
        std::printf("GRID  res %d -> %d x %d x %d = %d voxels (%d inside, %.1f%%)"
                    " · side %.4g · built in %lld ms\n",
                    res, field.dim(0), field.dim(1), field.dim(2),
                    field.cellCount(), field.filledCount(),
                    100.0 * field.filledCount() / qMax(1, field.cellCount()),
                    field.side(0), (long long)fieldMs);

    const QVector<CellBox> world =
        PuzzleDivider::buildBspCells(wdom, qMax(1, n), 0.35, quint32(seed), 0.0,
                                     field.isValid() ? &field : nullptr);

    QVector<CellBox> cells;
    cells.reserve(world.size());
    for (const CellBox &w : world) {
        CellBox c;
        for (int a = 0; a < 3; ++a) {
            const double lo = dom[a * 2], span = dom[a * 2 + 1] - lo;
            c.lo[a] = lo + span * (w.lo[a] / ext[a]);
            c.hi[a] = lo + span * (w.hi[a] / ext[a]);
        }
        cells.append(c);
    }

    QVector<PuzzlePiece> pieces;
    QString warn;
    if (!PuzzleDivider::divideCells(cage, cells, 12.0, &pieces, &warn)) {
        std::printf("FAIL  %s\n", qPrintable(warn));
        return 1;
    }
    std::printf("CAGE  asked %d -> %d BSP cell(s) -> %d boxy cage piece(s)\n",
                n, int(cells.size()), int(pieces.size()));

    const int png = a.indexOf(QStringLiteral("--png"));
    const auto shoot = [&](const QVector<PuzzlePiece> &ps, const QString &f) {
        MeshView v;
        v.setWidth(900); v.setHeight(700);
        v.setPieces(ps);
        v.setExplode(0.45);
        QImage img(900, 700, QImage::Format_RGB32);
        { QPainter pr(&img); v.paint(&pr); }
        std::printf(img.save(f) ? "PNG   %s\n" : "FAIL  %s\n", qPrintable(f));
    };
    if (png >= 0 && a.size() > png + 1)
        shoot(pieces, a.at(png + 1));

    // Volume is the decisive check on an intersection: the trimmed pieces must
    // add up to the MODEL, not to the cage. Eyeballing a render cannot tell a
    // real intersection from a union that happens to look plausible.
    const auto vol = [](const MeshData &m) {
        double v = 0.0;
        for (int t = 0; t + 2 < m.tris.size(); t += 3) {
            const float *A = &m.pos[m.tris[t + 0] * 3];
            const float *B = &m.pos[m.tris[t + 1] * 3];
            const float *C = &m.pos[m.tris[t + 2] * 3];
            v += double(A[0]) * (double(B[1]) * double(C[2]) - double(B[2]) * double(C[1]))
               - double(A[1]) * (double(B[0]) * double(C[2]) - double(B[2]) * double(C[0]))
               + double(A[2]) * (double(B[0]) * double(C[1]) - double(B[1]) * double(C[0]));
        }
        return qAbs(v) / 6.0;
    };
    // The cage total has to be measured on repaired geometry: straight out of
    // the tessellator a cage box has mixed winding, and the divergence integral
    // then reads anything at all (a whole 40^3 cage measured as 0).
    double cageVol = 0.0;
    for (int i = 0; i < pieces.size(); ++i) {
        MeshData c = pieces[i].mesh;
        IritSolid::orientConsistently(&c);
        cageVol += IritSolid::signedVolume(c);
        // How much of the model actually falls in this cell. A cell with no
        // model vertices in it has nothing to intersect, so an empty result
        // there is the CORRECT answer, not a boolean failure.
        int inside = 0;
        for (int v = 0; v + 2 < model.pos.size(); v += 3)
            if (model.pos[v + 0] >= c.bmin[0] && model.pos[v + 0] <= c.bmax[0] &&
                model.pos[v + 1] >= c.bmin[1] && model.pos[v + 1] <= c.bmax[1] &&
                model.pos[v + 2] >= c.bmin[2] && model.pos[v + 2] <= c.bmax[2])
                ++inside;
        std::printf("  cage[%d] tris %5d  vol %9.5g  model verts in cell %d\n",
                    i, c.triangleCount(), IritSolid::signedVolume(c), inside);
    }

    const CageBoolean::Result r = CageBoolean::intersectAll(&pieces, model, 12.0);

    double trimVol = 0.0;
    for (const PuzzlePiece &p : pieces) trimVol += vol(p.mesh);

    // How many SEPARATE solids each piece is made of. A cage cell is convex but
    // the model is not, so one cell can catch two unconnected lumps - a bit of
    // a leg and a bit of a tail - and they come back as a single "piece" that
    // could never be printed or assembled as one body.
    const auto components = [](const MeshData &src, QVector<double> *vols) {
        MeshData m = src;
        IritSolid::weldClose(&m);            // boolean output is not welded

        const int nv = int(m.pos.size() / 3);
        QVector<int> parent(nv);
        for (int i = 0; i < nv; ++i) parent[i] = i;
        std::function<int(int)> find = [&](int x) {
            while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
            return x;
        };
        const auto unite = [&](int a, int b) {
            a = find(a); b = find(b);
            if (a != b) parent[a] = b;
        };
        for (int t = 0; t + 2 < m.tris.size(); t += 3) {
            unite(m.tris[t + 0], m.tris[t + 1]);
            unite(m.tris[t + 1], m.tris[t + 2]);
        }

        // Volume per component, by the divergence theorem on its own triangles.
        QHash<int, double> vol;
        for (int t = 0; t + 2 < m.tris.size(); t += 3) {
            const float *A = &m.pos[m.tris[t + 0] * 3];
            const float *B = &m.pos[m.tris[t + 1] * 3];
            const float *C = &m.pos[m.tris[t + 2] * 3];
            const double d =
                  double(A[0]) * (double(B[1]) * double(C[2]) - double(B[2]) * double(C[1]))
                - double(A[1]) * (double(B[0]) * double(C[2]) - double(B[2]) * double(C[0]))
                + double(A[2]) * (double(B[0]) * double(C[1]) - double(B[1]) * double(C[0]));
            vol[find(m.tris[t + 0])] += d / 6.0;
        }
        vols->clear();
        for (auto it = vol.constBegin(); it != vol.constEnd(); ++it)
            vols->append(qAbs(it.value()));
        std::sort(vols->begin(), vols->end(), std::greater<double>());
        return vols->size();
    };

    int splitPieces = 0, strayLumps = 0;
    for (int i = 0; i < pieces.size(); ++i) {
        const MeshData &m = pieces[i].mesh;
        QVector<double> cv;
        const int nc = components(m, &cv);
        if (nc > 1) { ++splitPieces; strayLumps += nc - 1; }

        QString extra;
        for (int c = 0; c < cv.size() && c < 6; ++c)
            extra += QStringLiteral(" %1").arg(cv[c], 0, 'g', 3);

        std::printf("  trim[%d] tris %5d  vol %9.5g  closed %s  PARTS %d %s%s\n",
                    i, m.triangleCount(), IritSolid::signedVolume(m),
                    IritSolid::isClosed(m) ? "yes" : "NO", nc,
                    qPrintable(extra), nc > 1 ? "  <- DISCONNECTED" : "");
    }
    std::printf("PARTS %d of %d piece(s) are more than one solid; %d stray lump(s)\n",
                splitPieces, int(pieces.size()), strayLumps);

    const double modelVol = vol(model);
    // Two independent measures of the model's own volume. The divergence
    // integral counts every shell it is given, so a mesh with interior
    // geometry reads high; the voxel fill uses parity down each column and
    // reports what is actually solid. They disagreeing is a fact about the
    // MODEL, not about the division.
    std::printf("MODEL divergence %.5g   voxel-fill %.5g (%.0f%% of divergence)\n",
                vol(model), field.total(),
                100.0 * field.total() / qMax(1e-9, vol(model)));
    std::printf("VOL   model %.5g   cage pieces %.5g (%.0f%%)   trimmed %.5g (%.0f%%)\n",
                modelVol, cageVol, 100.0 * cageVol / qMax(1e-9, modelVol),
                trimVol, 100.0 * trimVol / qMax(1e-9, modelVol));
    for (const QString &line : CageBoolean::describe(r))
        std::printf("%s\n", qPrintable(line));

    if (png >= 0 && a.size() > png + 2)
        shoot(pieces, a.at(png + 2));

    // --save PATH [PATH...] : write the divided model out, format by extension.
    const int sv = a.indexOf(QStringLiteral("--save"));
    if (sv >= 0)
        for (int k = sv + 1; k < a.size() && !a.at(k).startsWith(QStringLiteral("--")); ++k) {
            PieceExport::Result pr;
            QString perr;
            if (PieceExport::save(pieces, a.at(k),
                                  a.contains(QStringLiteral("--split")),
                                  a.contains(QStringLiteral("--spread")),
                                  &pr, &perr))
                std::printf("SAVE  %s : %d piece(s) written, %d skipped\n",
                            qPrintable(a.at(k)), pr.written, pr.skipped);
            else
                std::printf("SAVE  FAIL %s : %s\n",
                            qPrintable(a.at(k)), qPrintable(perr));
        }

    // Stages 1-3 on the trimmed pieces. Silent unless --plan is passed.
    //
    // Read the verdict with care: these pieces come from cutting a box with
    // flat planes, and a box partition can always be peeled along an axis, so
    // "assemblable" here is correct and uninformative. It starts carrying
    // information once joints constrain the motion.
    plannerCli(a, pieces);

    // --figures DIR : the planner's three stages as PNGs for slides - the same
    // pictures the GUI writes beside the model after a cage division.
    const int fg = a.indexOf(QStringLiteral("--figures"));
    if (fg >= 0 && a.size() > fg + 1) {
        const Planner::Graph                 figGraph = Planner::build(pieces, 1e-6, 0.0);
        const Planner::TranslationalBlocking figModel;
        const Planner::Plan                  figPlan  = Planner::extract(figGraph, figModel);
        const PlannerFigure::Result fr =
            PlannerFigure::write(pieces, figGraph, figModel, figPlan, a.at(fg + 1),
                                 QFileInfo(a.at(0)).completeBaseName());
        for (const QString &f : fr.written)
            std::printf("FIG   %s\n", qPrintable(f));
        for (const QString &problem : fr.problems)
            std::printf("FIG   FAIL %s\n", qPrintable(problem));
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

        const int od = args.indexOf(QStringLiteral("--order"));
        if (od >= 0)
            return orderCli(args.mid(od + 1));

        const int sh = args.indexOf(QStringLiteral("--shot"));
        if (sh >= 0)
            return shotCli(args.mid(sh + 1));

        const int fl = args.indexOf(QStringLiteral("--field"));
        if (fl >= 0)
            return fieldCli(args.mid(fl + 1));

        const int cg = args.indexOf(QStringLiteral("--cage"));
        if (cg >= 0)
            return cageCli(args.mid(cg + 1));

        const int asm_ = args.indexOf(QStringLiteral("--assemble"));
        if (asm_ >= 0)
            return assembleCli(args.mid(asm_ + 1));

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
