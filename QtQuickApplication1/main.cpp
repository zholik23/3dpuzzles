//
// main - the GUI entry point: install the IRIT error handlers, register the
// QML types, load main.qml, and run.
//
#include "AppController.h"
#include "IritGuard.h"
#include "CurvedBsp.h"
#include "DbgAnalysis.h"
#include "CadLoader.h"
#include "HarmonicFit.h"
#include "MeshView.h"
#include "PuzzleAnalyzer.h"

#include <QDebug>
#include <QEventLoop>
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

// Last: IritSolid.h pulls in irit_sm.h, which #defines _mkdir (see AppController.cpp).
#include "IritSolid.h"

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

        // TEMPORARY - Morse split of a mesh into tube-like parts:
        //   app --split <model.obj>
        if (a.size() > 2 && a.at(1) == QStringLiteral("--split")) {
            MeshData mesh;
            QString err;
            if (!CadLoader::load(a.at(2), &mesh, &err)) {
                qCritical().noquote() << "SPLIT  could not load" << a.at(2) << ":" << err;
                return 2;
            }
            QVector<HarmonicFit::Part> parts;
            QStringList notes;
            HarmonicFit::SplitOptions so;
            if (a.size() > 3) so.neckJump = a.at(3).toDouble();
            if (a.size() > 4) so.neckCalm = a.at(4).toDouble();
            if (!HarmonicFit::splitLimbs(mesh, so, &parts, &notes, &err)) {
                qCritical().noquote() << "SPLIT  failed:" << err;
                return 1;
            }
            for (const QString &n : notes) qInfo().noquote() << "SPLIT  " + n;
            double sum = 0.0;
            for (const auto &P : parts) {
                const double v = IritSolid::signedVolume(P.mesh);
                sum += v;
                qInfo().noquote() << QStringLiteral("SPLIT  %1 volume %2  box x %3..%4 y %5..%6 z %7..%8").arg(P.name).arg(v, 0, 'g', 5)
                                         .arg(P.mesh.bmin[0], 0, 'f', 2).arg(P.mesh.bmax[0], 0, 'f', 2)
                                         .arg(P.mesh.bmin[1], 0, 'f', 2).arg(P.mesh.bmax[1], 0, 'f', 2)
                                         .arg(P.mesh.bmin[2], 0, 'f', 2).arg(P.mesh.bmax[2], 0, 'f', 2);
            }
            qInfo().noquote() << QStringLiteral("SPLIT  parts sum %1, model %2")
                                     .arg(sum, 0, 'g', 6).arg(IritSolid::signedVolume(mesh), 0, 'g', 6);
            return 0;
        }

        // TEMPORARY - the app's own path: load, split, fit parts, divide, save:
        //   app --partsdivide <model.obj> pieces <out.itd>
        if (a.size() > 4 && a.at(1) == QStringLiteral("--partsdivide")) {
            AppController c;
            c.loadPath(a.at(2));
            if (a.size() > 5 && a.at(5) == QStringLiteral("trim")) c.setEncloseTrim(true);
            c.splitLimbs();
            qInfo().noquote() << "APP  " + c.status();
            QEventLoop loop;
            QObject::connect(&c, &AppController::fittingChanged, &loop, [&]() { if (!c.fitting()) loop.quit(); });
            c.fitTrivariate(0);
            if (c.fitting()) loop.exec();
            qInfo().noquote() << "APP  " + c.status();
            qInfo().noquote() << "APP  " + c.detail();
            c.divideRandom(a.at(3).toInt());
            qInfo().noquote() << "APP  " + c.status();
            qInfo().noquote() << "APP  " + c.detail();
            {
                double sum = 0.0;
                for (const PuzzlePiece &pc : c.pieces()) sum += std::fabs(IritSolid::signedVolume(pc.mesh));   // tessellated pieces face inward
                MeshData src;
                QString e2;
                CadLoader::load(a.at(2), &src, &e2);
                qInfo().noquote() << QStringLiteral("APP  pieces' volume %1 vs model %2 (%3%)")
                                         .arg(sum, 0, 'g', 6).arg(IritSolid::signedVolume(src), 0, 'g', 6)
                                         .arg(100.0 * sum / IritSolid::signedVolume(src), 0, 'f', 2);
            }
            c.savePieces(QUrl::fromLocalFile(a.at(4)), false, false);
            qInfo().noquote() << "APP  " + c.status() + " | " + c.detail();
            return 0;
        }

        // TEMPORARY - split, then one trivariate per part:
        //   app --fitparts <model.obj>
        if (a.size() > 2 && a.at(1) == QStringLiteral("--fitparts")) {
            MeshData mesh;
            QString err;
            if (!CadLoader::load(a.at(2), &mesh, &err)) {
                qCritical().noquote() << "PARTS  could not load" << a.at(2) << ":" << err;
                return 2;
            }
            QVector<HarmonicFit::Part> parts;
            QStringList notes;
            if (!HarmonicFit::splitLimbs(mesh, HarmonicFit::SplitOptions(), &parts, &notes, &err)) {
                qCritical().noquote() << "PARTS  split failed:" << err;
                return 1;
            }
            for (const QString &n : notes) qInfo().noquote() << "PARTS  " + n;
            QVector<HarmonicFit::Result> fits;
            for (const HarmonicFit::Part &P : parts) {
                HarmonicFit::Options o;
                const HarmonicFit::Cap *big = nullptr;
                for (const auto &c : P.caps) if (!big || c.area > big->area) big = &c;
                if (big) {
                    o.capStart = true;
                    for (int k = 0; k < 3; ++k) o.capCentre[k] = big->centre[k];
                }
                fits.append(HarmonicFit::fit(P.mesh, o));
            }
            const bool snap = !(a.size() > 3 && a.at(3) == QStringLiteral("nosnap"));
            if (snap) {
                QStringList joins;
                HarmonicFit::snapParts(&fits, parts, &joins);
                for (const QString &n : joins) qInfo().noquote() << "PARTS  " + n;
            }
            // Enclosure per part (for the section 5 trim): how many part vertices
            // stay outside, and whether the enclosing block folds.
            for (int pi = 0; pi < parts.size(); ++pi) {
                if (!fits[pi].ok) continue;
                HarmonicFit::Result enc = fits[pi];
                QStringList en;
                const bool all = HarmonicFit::enclose(&enc, parts[pi].mesh, &en);
                Trivariate te = HarmonicFit::toTrivariate(enc, parts[pi].name, &err);
                const HarmonicFit::Check ce = HarmonicFit::checkJacobian(te);
                qInfo().noquote() << QStringLiteral("PARTS  enclose %1: %2 | all inside %3 | folds %4 of %5")
                                         .arg(parts[pi].name, en.join(QStringLiteral("; "))).arg(all ? "yes" : "no")
                                         .arg(ce.nonPositive).arg(ce.samples);
            }
            std::vector<Trivariate> tvs(parts.size());
            for (int pi = 0; pi < parts.size(); ++pi)
                if (fits[pi].ok) tvs[size_t(pi)] = HarmonicFit::toTrivariate(fits[pi], parts[pi].name, &err);
            for (int pi = 0; pi < parts.size(); ++pi) {
                const HarmonicFit::Part &P = parts[pi];
                const HarmonicFit::Cap *big = nullptr;
                for (const auto &c : P.caps) if (!big || c.area > big->area) big = &c;
                const HarmonicFit::Result &r = fits[pi];
                if (!r.ok) {
                    qInfo().noquote() << QStringLiteral("PARTS  %1: fit FAILED - %2").arg(P.name, r.error);
                    continue;
                }
                const Trivariate &tv = tvs[size_t(pi)];
                const HarmonicFit::Check ck = HarmonicFit::checkJacobian(tv);
                const HarmonicFit::Deviation dv = HarmonicFit::surfaceDeviation(tv, P.mesh);
                qInfo().noquote() << QStringLiteral("PARTS  %1: %2 cap(s), det J <= 0 at %3 of %4, volume %5% of the part, "
                                                    "distance mean %6% max %7%")
                                         .arg(P.name).arg(P.caps.size()).arg(ck.nonPositive).arg(ck.samples)
                                         .arg(100.0 * std::fabs(ck.volume) / std::max(1e-30, r.meshVolume), 0, 'f', 1)
                                         .arg(100 * dv.mean, 0, 'f', 2).arg(100 * dv.max, 0, 'f', 2);
                // How far this block's surface is from each of its caps (the
                // interfaces): cap fan triangles sampled, distance to the block.
                for (const auto &cap : P.caps) {
                    int cv = -1;
                    for (int v = 0; v < P.mesh.vertexCount(); ++v)
                        if (std::fabs(P.mesh.pos[3 * v] - cap.centre[0]) < 1e-5 && std::fabs(P.mesh.pos[3 * v + 1] - cap.centre[1]) < 1e-5 &&
                            std::fabs(P.mesh.pos[3 * v + 2] - cap.centre[2]) < 1e-5) { cv = v; break; }
                    if (cv < 0) continue;
                    MeshData pts;
                    for (int t = 0; t < P.mesh.triangleCount(); ++t) {
                        const uint32_t *T = &P.mesh.tris[3 * t];
                        if (T[0] != uint32_t(cv) && T[1] != uint32_t(cv) && T[2] != uint32_t(cv)) continue;
                        for (double a1 = 0.1; a1 < 1.0; a1 += 0.2)
                            for (double a2 = 0.1; a1 + a2 < 1.0; a2 += 0.2) {
                                double q[3];
                                for (int k = 0; k < 3; ++k)
                                    q[k] = a1 * P.mesh.pos[3 * T[0] + k] + a2 * P.mesh.pos[3 * T[1] + k] + (1 - a1 - a2) * P.mesh.pos[3 * T[2] + k];
                                pts.addVertex(q[0], q[1], q[2]);
                            }
                    }
                    const HarmonicFit::Deviation cd = HarmonicFit::surfaceDeviation(tv, pts);
                    qInfo().noquote() << QStringLiteral("PARTS    cap (area %1): block surface from the cap mean %2% max %3%%4")
                                             .arg(cap.area, 0, 'g', 3).arg(100 * cd.mean, 0, 'f', 2).arg(100 * cd.max, 0, 'f', 2)
                                             .arg(big == &cap ? QStringLiteral("  [u = 0 face]") : QString());
                }
                // The joint: this block's u = 0 face sampled, distance to the
                // block on the other side of the cap.
                if (big) {
                    for (int q = 0; q < parts.size(); ++q) {
                        if (q == pi || !tvs[size_t(q)].isValid()) continue;
                        bool shares = false;
                        for (const auto &c2 : parts[q].caps)
                            if (std::fabs(c2.centre[0] - big->centre[0]) < 1e-5 && std::fabs(c2.centre[1] - big->centre[1]) < 1e-5 &&
                                std::fabs(c2.centre[2] - big->centre[2]) < 1e-5) shares = true;
                        if (!shares) continue;
                        MeshData face;
                        for (int j = 0; j < 40; ++j)
                            for (int k = 0; k < 10; ++k) {
                                double q3[3];
                                tv.evaluate(0.0, j / 40.0, k / 10.0, q3);
                                face.addVertex(q3[0], q3[1], q3[2]);
                            }
                        const HarmonicFit::Deviation g = HarmonicFit::surfaceDeviation(tvs[size_t(q)], face);
                        qInfo().noquote() << QStringLiteral("PARTS    joint to %1: gap mean %2% max %3%")
                                                 .arg(parts[q].name).arg(100 * g.mean, 0, 'f', 2).arg(100 * g.max, 0, 'f', 2);
                    }
                }
                QStringList bad;
                for (int b = 0; b + 2 < ck.bad.size() && b < 30; b += 3)
                    bad << QStringLiteral("(%1 %2 %3)").arg(ck.bad[b], 0, 'f', 2).arg(ck.bad[b + 1], 0, 'f', 2).arg(ck.bad[b + 2], 0, 'f', 2);
                if (!bad.isEmpty()) qInfo().noquote() << "PARTS    folds at u v w: " + bad.join(' ');
            }
            return 0;
        }

        // TEMPORARY - harmonic trivariate fit of a mesh:
        //   app --fit <model.obj> [nu nv nw voxels]   writes tv_<model>.itd beside it
        if (a.size() > 2 && a.at(1) == QStringLiteral("--fit")) {
            MeshData mesh;
            QString err;
            if (!CadLoader::load(a.at(2), &mesh, &err)) {
                qCritical().noquote() << "FIT  could not load" << a.at(2) << ":" << err;
                return 2;
            }
            HarmonicFit::Options o;
            if (a.size() > 6) {
                o.nu = a.at(3).toInt(); o.nv = a.at(4).toInt();
                o.nw = a.at(5).toInt(); o.voxels = a.at(6).toInt();
                if (a.size() > 7) o.smooth = a.at(7).toInt();
                if (a.size() > 8) o.normalStart = a.at(8).toDouble();
                if (a.size() > 9) o.capPerimeter = a.at(9).toDouble();
            }
            const HarmonicFit::Result r = HarmonicFit::fit(mesh, o);
            for (const QString &n : r.notes) qInfo().noquote() << "FIT  " + n;
            if (!r.ok) {
                qCritical().noquote() << "FIT  failed:" << r.error;
                return 1;
            }
            Trivariate tv = HarmonicFit::toTrivariate(r, QStringLiteral("fit"), &err);
            if (!tv.isValid()) {
                qCritical().noquote() << "FIT  build failed:" << err;
                return 1;
            }
            const HarmonicFit::Check ck = HarmonicFit::checkJacobian(tv);
            qInfo().noquote() << QStringLiteral("FIT  det J <= 0 at %1 of %2 samples, min/mean %3, "
                                                "volume %4 vs mesh %5 (%6%)")
                                     .arg(ck.nonPositive).arg(ck.samples).arg(ck.minRatio, 0, 'f', 3)
                                     .arg(ck.volume, 0, 'g', 5).arg(r.meshVolume, 0, 'g', 5)
                                     .arg(100.0 * std::fabs(ck.volume) / std::max(1e-30, r.meshVolume), 0, 'f', 1);
            for (int b = 0; b + 2 < ck.bad.size(); b += 3)
                qInfo().noquote() << QStringLiteral("FIT  det J <= 0 at u %1 v %2 w %3").arg(ck.bad[b], 0, 'f', 3).arg(ck.bad[b + 1], 0, 'f', 3).arg(ck.bad[b + 2], 0, 'f', 3);
            const HarmonicFit::Deviation dv = HarmonicFit::surfaceDeviation(tv, mesh);
            qInfo().noquote() << QStringLiteral("FIT  mesh to trivariate: mean %1%  p95 %2%  max %3%  "
                                                "vertices > 1%: %4%")
                                     .arg(100 * dv.mean, 0, 'f', 2).arg(100 * dv.p95, 0, 'f', 2)
                                     .arg(100 * dv.max, 0, 'f', 2).arg(100 * dv.over1, 0, 'f', 1);
            const QFileInfo fi(a.at(2));
            const QString out = fi.absolutePath() + "/tv_" + fi.completeBaseName() + ".itd";
            qInfo().noquote() << (tv.saveToFile(out, &err) ? "FIT  saved " + out : "FIT  not saved: " + err);

            HarmonicFit::Result enc = r;
            QStringList en;
            const bool all = HarmonicFit::enclose(&enc, mesh, &en);
            for (const QString &n : en) qInfo().noquote() << "FIT  " + n;
            Trivariate tve = HarmonicFit::toTrivariate(enc, QStringLiteral("enclosing"), &err);
            const HarmonicFit::Check cke = HarmonicFit::checkJacobian(tve);
            qInfo().noquote() << QStringLiteral("FIT  enclosing: all inside %1, det J <= 0 at %2 of %3, volume %4% of the mesh")
                                     .arg(all ? "yes" : "no").arg(cke.nonPositive).arg(cke.samples)
                                     .arg(100.0 * std::fabs(cke.volume) / std::max(1e-30, r.meshVolume), 0, 'f', 1);
            const QString oute = fi.absolutePath() + "/tvenc_" + fi.completeBaseName() + ".itd";
            qInfo().noquote() << (tve.saveToFile(oute, &err) ? "FIT  saved " + oute : "FIT  not saved: " + err);
            return 0;
        }

        // TEMPORARY - curved BSP in D:
        //   app --bsp <trivariate.itd> pieces bend waves seed [meshN]
        if (a.size() > 6 && a.at(1) == QStringLiteral("--bsp")) {
            QString err;
            Trivariate tv = Trivariate::fromFile(a.at(2), &err);
            if (!tv.isValid()) {
                qCritical().noquote() << "BSP  could not load" << a.at(2) << ":" << err;
                return 2;
            }
            const CurvedBsp bsp = CurvedBsp::build(tv, a.at(3).toInt(), a.at(4).toDouble(),
                                                   a.at(5).toInt(), quint32(a.at(6).toUInt()));
            qInfo().noquote() << QStringLiteral("AFFINE  M deviates %1% of model size from a trilinear box")
                                     .arg(100.0 * DbgAnalysis::affineDeviation(tv), 0, 'f', 2);
            for (int i = 0; i < bsp.splitCount(); ++i) {
                const CurvedBsp::Split &sp = bsp.split(bsp.splitNode(i));
                qInfo().noquote() << QStringLiteral("SPLIT  node %1 axis %2 at %3 amp %4")
                                         .arg(bsp.splitNode(i)).arg(QStringLiteral("uvw").mid(sp.axis, 1))
                                         .arg(sp.at, 0, 'f', 3).arg(sp.amp, 0, 'f', 4);
            }
            DbgOptions opt;
            const DbgReport r = DbgAnalysis::runBsp(tv, bsp, opt);
            const QString label = QFileInfo(a.at(2)).completeBaseName() + QStringLiteral("_bsp");
            qInfo().noquote() << "ROW|" + DbgAnalysis::tableRow(label, r);
            qInfo().noquote() << DbgAnalysis::formatReport(label, r, opt);
            QStringList keys;
            for (int k : r.keySequence) keys << QString::number(k);
            qInfo().noquote() << QStringLiteral("LEVEL  strict single key %1 | k = %2 | keys %3 | %4")
                                     .arg(r.strictSingleKey ? "yes" : "no").arg(r.keyLevel)
                                     .arg(keys.join(QStringLiteral(","))).arg(r.levelStop);
            const DbgQuality q = DbgAnalysis::qualityBsp(tv, bsp);
            for (int i = 0; i < q.pieces.size(); ++i)
                qInfo().noquote() << QStringLiteral("PIECE %1  vol %2  minDetJ %3")
                                         .arg(i).arg(q.pieces[i].volume, 0, 'g', 4)
                                         .arg(q.pieces[i].minDetJ, 0, 'g', 3);
            const int meshN = a.size() > 7 ? a.at(7).toInt() : 0;
            for (int i = 0; meshN > 0 && i < bsp.pieceCount(); ++i) {
                MeshData md;
                const bool ok = bsp.meshPiece(tv, i, meshN, &md);
                qInfo().noquote() << QStringLiteral("MESH %1  %2  %3 tris  box [%4,%5] [%6,%7] [%8,%9]")
                                         .arg(i).arg(ok ? "ok" : "EMPTY").arg(md.triangleCount())
                                         .arg(md.bmin[0], 0, 'f', 2).arg(md.bmax[0], 0, 'f', 2)
                                         .arg(md.bmin[1], 0, 'f', 2).arg(md.bmax[1], 0, 'f', 2)
                                         .arg(md.bmin[2], 0, 'f', 2).arg(md.bmax[2], 0, 'f', 2);
            }
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
