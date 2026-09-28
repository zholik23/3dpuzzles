//
// main - the GUI entry point: install the IRIT error handlers, register the
// QML types, load main.qml, and run.
//
#include "AppController.h"
#include "IritGuard.h"
#include "DbgAnalysis.h"
#include "MeshView.h"
#include "PuzzleAnalyzer.h"

#include <QDebug>
#include <QFileInfo>
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QQmlError>
#include <QStringList>
#include <QUrl>
#include <qqml.h>

#include <algorithm>
#include <cmath>

int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);
    app.setApplicationName(QStringLiteral("Puzzle Divider"));

    // Before any other IRIT call: stop a malformed file from taking the
    // process down through IRIT's default exit()-on-error handlers.
    IritGuard::installHandlers();

    // TEMPORARY - diagnostic only.
    {
        const QStringList a = QGuiApplication::arguments();
        if (a.size() > 2 && a.at(2) == QStringLiteral("--selftest")) {
            const int n = (a.size() > 3) ? a.at(3).toInt() : 4;
            AppController c;
            c.loadPath(a.at(1));
            c.useBoundingCage();
            c.divideRandom(n);
            qInfo().noquote() << "PIECES  :" << c.pieceCount();
            return 0;
        }
    }

    // TEMPORARY - experiment driver:
    //   app --dbg <trivariate.itd|box> [nx ny nz]
    {
        const QStringList a = QGuiApplication::arguments();
        if (a.size() > 2 && a.at(1) == QStringLiteral("--dbg")) {
            const QString src = a.at(2);

            // A primitive name (Box, Sphere, Cylinder, Cone, Torus) or a file.
            QString err, kind;
            for (const QString &k : Trivariate::primitiveKinds())
                if (k.compare(src, Qt::CaseInsensitive) == 0)
                    kind = k;

            Trivariate tv = kind.isEmpty() ? Trivariate::fromFile(src, &err)
                                           : Trivariate::primitive(kind, &err);
            if (!tv.isValid()) {
                qCritical().noquote() << "DBG  could not load" << src << ":" << err;
                return 2;
            }

            DbgOptions opt;
            if (a.size() > 5) {
                opt.cells[0] = a.at(3).toInt();
                opt.cells[1] = a.at(4).toInt();
                opt.cells[2] = a.at(5).toInt();
            }
            if (a.size() > 6)
                opt.directions = std::max(8, a.at(6).toInt());   // optional: finer direction set

            const QString label = QFileInfo(src).completeBaseName();
            const DbgReport r = DbgAnalysis::run(tv, opt);

            // Machine-readable row first, so a script can assemble the table.
            qInfo().noquote() << "ROW|" + DbgAnalysis::tableRow(label, r);
            qInfo().noquote() << "DETAIL-BEGIN";
            qInfo().noquote() << DbgAnalysis::formatReport(label, r, opt);
            {
                QStringList keys;
                for (int k : r.keySequence) keys << QString::number(k);
                qInfo().noquote() << QStringLiteral("LEVEL  strict single key %1 | k = %2 | keys %3 | %4")
                                         .arg(r.strictSingleKey ? "yes" : "no").arg(r.keyLevel)
                                         .arg(keys.join(QStringLiteral(","))).arg(r.levelStop);
                QStringList st;
                for (const DbgReport::Group &g : r.disassembly) {
                    QStringList m;
                    for (int p : g.pieces) m << QString::number(p);
                    st << QStringLiteral("{%1}").arg(m.join(QStringLiteral(",")));
                }
                qInfo().noquote() << (r.subsetSearchFull
                    ? QStringLiteral("TAKEAPART  %1%2").arg(st.join(QStringLiteral(" -> ")))
                          .arg(r.disassemblyComplete ? "" : "  (stuck)")
                    : QStringLiteral("TAKEAPART  not computed - %1 pieces, more than %2")
                          .arg(r.pieces).arg(opt.fullSubsetsUpTo));
                const DbgQuality q = DbgAnalysis::quality(tv, opt);
                for (int i = 0; i < q.pieces.size(); ++i)
                    qInfo().noquote() << QStringLiteral("PIECE %1  vol %2  thick %3 (%4 rays)  minDetJ %5  at (%6, %7, %8)")
                                             .arg(i).arg(q.pieces[i].volume, 0, 'g', 4)
                                             .arg(q.pieces[i].thickness, 0, 'g', 4).arg(q.pieces[i].rays)
                                             .arg(q.pieces[i].minDetJ, 0, 'g', 3)
                                             .arg(q.pieces[i].thickAt[0], 0, 'f', 3)
                                             .arg(q.pieces[i].thickAt[1], 0, 'f', 3)
                                             .arg(q.pieces[i].thickAt[2], 0, 'f', 3);
                qInfo().noquote() << QStringLiteral("SIZES  model %1  vol min %2 max %3 cv %4")
                                         .arg(q.modelSize, 0, 'g', 4).arg(q.volMin, 0, 'g', 4)
                                         .arg(q.volMax, 0, 'g', 4).arg(q.volCv, 0, 'f', 3);
            }
            qInfo().noquote() << "DETAIL-END";
            return r.valid ? 0 : 1;
        }

        // TEMPORARY - independent check of a DBG verdict:
        //   app --sweep <trivariate.itd|box> [nx ny nz]
        // Runs the DBG, then physically slides its escaping group along the
        // reported direction (and the opposite one, and each member alone) and
        // looks for interpenetration. Uses no normals at all.
        if (a.size() > 2 && a.at(1) == QStringLiteral("--sweep")) {
            const QString src = a.at(2);
            QString err, kind;
            for (const QString &k : Trivariate::primitiveKinds())
                if (k.compare(src, Qt::CaseInsensitive) == 0)
                    kind = k;
            Trivariate tv = kind.isEmpty() ? Trivariate::fromFile(src, &err)
                                           : Trivariate::primitive(kind, &err);
            if (!tv.isValid()) {
                qCritical().noquote() << "SWEEP  could not load" << src << ":" << err;
                return 2;
            }
            DbgOptions opt;
            if (a.size() > 5) {
                opt.cells[0] = a.at(3).toInt();
                opt.cells[1] = a.at(4).toInt();
                opt.cells[2] = a.at(5).toInt();
            }
            const DbgReport r = DbgAnalysis::run(tv, opt);

            // Optional: slide a chosen group along every direction in a cone,
            // to find out whether a group the DBG calls blocked has a free
            // direction thinner than its sampling.
            //   --sweep <file> nx ny nz --scan <ids> cx cy cz <halfAngleDeg> <count>
            if (a.size() > 12 && a.at(6) == QStringLiteral("--scan")) {
                QVector<int> grp;
                for (const QString &t : a.at(7).split(QLatin1Char(',')))
                    grp.append(t.toInt());
                double c[3] = { a.at(8).toDouble(), a.at(9).toDouble(), a.at(10).toDouble() };
                const double cl = std::sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]);
                for (double &x : c) x /= cl;
                const double half = a.at(11).toDouble() * 3.14159265358979323846 / 180.0;
                const int count = std::max(1, a.at(12).toInt());

                double ext = 0.0;
                for (int k = 0; k < 3; ++k) {
                    double lo = 1e300, hi = -1e300;
                    for (int i = 0; i + 5 < r.pieceBox.size(); i += 6) {
                        lo = std::min(lo, r.pieceBox[i + 2 * k]);
                        hi = std::max(hi, r.pieceBox[i + 2 * k + 1]);
                    }
                    ext = std::max(ext, hi - lo);
                }
                const double travel = 0.4 * ext;

                // An orthonormal frame around the cone axis.
                double e1[3] = { -c[1], c[0], 0.0 };
                if (std::fabs(c[2]) > 0.9) { e1[0] = 0.0; e1[1] = -c[2]; e1[2] = c[1]; }
                const double l1 = std::sqrt(e1[0] * e1[0] + e1[1] * e1[1] + e1[2] * e1[2]);
                for (double &x : e1) x /= l1;
                const double e2[3] = { c[1] * e1[2] - c[2] * e1[1],
                                       c[2] * e1[0] - c[0] * e1[2],
                                       c[0] * e1[1] - c[1] * e1[0] };

                int best = -1;
                double bestDir[3] = { 0, 0, 0 };
                int clear = 0;
                for (int n = 0; n < count; ++n) {
                    // Fibonacci points on the spherical cap.
                    const double cz = 1.0 - (1.0 - std::cos(half)) * (n + 0.5) / count;
                    const double rr = std::sqrt(std::max(0.0, 1.0 - cz * cz));
                    const double ph = 2.399963229728653 * n;
                    double d[3];
                    for (int k = 0; k < 3; ++k)
                        d[k] = cz * c[k] + rr * (std::cos(ph) * e1[k] + std::sin(ph) * e2[k]);
                    const DbgSweep s = DbgAnalysis::sweep(tv, opt, grp, d, travel);
                    if (s.collided == 0) ++clear;
                    if (best < 0 || s.collided < best) {
                        best = s.collided;
                        for (int k = 0; k < 3; ++k) bestDir[k] = d[k];
                    }
                }
                qInfo().noquote() << QStringLiteral("SCAN  group {%1}: %2 directions within %3 deg of (%4, %5, %6), "
                                                    "travel %7 -> %8 clear; fewest collisions %9 along (%10, %11, %12)")
                                         .arg(a.at(7)).arg(count).arg(a.at(11))
                                         .arg(c[0], 0, 'f', 3).arg(c[1], 0, 'f', 3).arg(c[2], 0, 'f', 3)
                                         .arg(travel, 0, 'f', 2).arg(clear).arg(best)
                                         .arg(bestDir[0], 0, 'f', 3).arg(bestDir[1], 0, 'f', 3).arg(bestDir[2], 0, 'f', 3);
                return 0;
            }

            if (!r.valid || r.smallestMobileSet.isEmpty()) {
                qInfo().noquote() << "SWEEP  no escaping group to test";
                return 1;
            }

            // Travel 40% of the model's size - well past any contact.
            double lo[3] = { 1e300, 1e300, 1e300 }, hi[3] = { -1e300, -1e300, -1e300 };
            for (int i = 0; i + 5 < r.pieceBox.size(); i += 6)
                for (int k = 0; k < 3; ++k) {
                    lo[k] = std::min(lo[k], r.pieceBox[i + 2 * k]);
                    hi[k] = std::max(hi[k], r.pieceBox[i + 2 * k + 1]);
                }
            double ext = 0.0;
            for (int k = 0; k < 3; ++k) ext = std::max(ext, hi[k] - lo[k]);
            const double travel = 0.4 * ext;

            QStringList ids;
            for (int p : r.smallestMobileSet) ids << QString::number(p);
            qInfo().noquote() << QStringLiteral("SWEEP  DBG says group {%1} escapes along "
                                                "(%2, %3, %4); travel %5")
                                     .arg(ids.join(QStringLiteral(",")))
                                     .arg(r.escapeDir[0], 0, 'f', 4)
                                     .arg(r.escapeDir[1], 0, 'f', 4)
                                     .arg(r.escapeDir[2], 0, 'f', 4)
                                     .arg(travel, 0, 'f', 3);

            const auto report = [&](const QString &what, const QVector<int> &g,
                                    const double dir[3], const QString &expect) {
                const DbgSweep s = DbgAnalysis::sweep(tv, opt, g, dir, travel);
                const QString got = s.collided > 0 ? QStringLiteral("COLLIDES")
                                                   : QStringLiteral("clear");
                qInfo().noquote() << QStringLiteral("SWEEP  %1 | expect %2 | got %3 | "
                                                    "samples %4, collided %5, left model %6, "
                                                    "first hit at %7")
                                         .arg(what, -34).arg(expect, -8).arg(got, -8)
                                         .arg(s.samples).arg(s.collided).arg(s.lost)
                                         .arg(s.firstHit < 0 ? QStringLiteral("-")
                                                             : QString::number(s.firstHit, 'f', 4));
            };

            const double plus[3]  = {  r.escapeDir[0],  r.escapeDir[1],  r.escapeDir[2] };
            const double minus[3] = { -r.escapeDir[0], -r.escapeDir[1], -r.escapeDir[2] };
            report(QStringLiteral("group {%1} along +d").arg(ids.join(QStringLiteral(","))),
                   r.smallestMobileSet, plus, QStringLiteral("clear"));
            report(QStringLiteral("group {%1} along -d").arg(ids.join(QStringLiteral(","))),
                   r.smallestMobileSet, minus, QStringLiteral("COLLIDES"));
            for (int p : r.smallestMobileSet) {
                const bool alone = (r.pieceFreeDirs.value(p) > 0);
                report(QStringLiteral("piece %1 alone along +d").arg(p), QVector<int> { p }, plus,
                       alone ? QStringLiteral("either") : QStringLiteral("COLLIDES"));
            }
            return 0;
        }
    }

    qmlRegisterType<MeshView>("PuzzleDivider", 1, 0, "MeshView");
    qmlRegisterType<PuzzleAnalyzer>("PuzzleDivider", 1, 0, "PuzzleAnalyzer");
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
