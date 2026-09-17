//
// CageBoolean - implementation: the per-piece intersection with the model, plus
// component labelling, welding, and the reporting of pieces that failed.
//

#include "CageBoolean.h"

#include "IritGuard.h"
#include "IritMesh.h"
#include "IritSolid.h"

#include <QHash>

#include <algorithm>
#include <cmath>

namespace {

struct AndCtx {
    IritPrsrObjectStruct *a;
    IritPrsrObjectStruct *b;
    IritPrsrObjectStruct *result;
    int                   unite;
};

// The boolean itself, inside IritGuard. The context holds only POD, because a
// longjmp cannot unwind C++ destructors.
void doAnd(void *v)
{
    AndCtx *c = static_cast<AndCtx *>(v);

    IritSolid::closeLists(c -> a);
    IritSolid::closeLists(c -> b);

    c -> result = c -> unite ? IritBooleanOR(c -> a, c -> b)
                             : IritBooleanAND(c -> a, c -> b);
}

// A piece face sitting on the model's bounding box is coplanar with the model's
// own outer face, and coplanar faces are what IRIT's booleans handle worst - a
// whole piece can come back empty. Nudging only those outer vertices outward
// cures it: the sliver added lies outside the solid, so model AND piece is
// unchanged. Interior cut faces are left exactly where they are, or neighbouring
// pieces would overlap instead of meeting.
//
// Same remedy as PuzBspGrowOuterFaces in ext_lib/PuzBspCore.c, which took a cube
// split into 8 from 7 pieces at 88.3% of the model volume to 8 at 100%.
void growOuterFaces(MeshData *piece, const MeshData &model)
{
    double ext = 0.0;
    for (int a = 0; a < 3; ++a)
        ext = qMax(ext, double(model.bmax[a]) - double(model.bmin[a]));
    if (ext <= 0.0)
        return;

    const float eps = float(ext * 1e-4);        // far below any printable size

    for (int v = 0; v + 2 < piece -> pos.size(); v += 3)
        for (int a = 0; a < 3; ++a) {
            float &c = piece -> pos[v + a];
            if (c <= model.bmin[a] + eps)
                c -= eps;
            else if (c >= model.bmax[a] - eps)
                c += eps;
        }
}

struct Labels {
    MeshData     mesh;
    QVector<int> triLabel;
    int          count = 0;
};

// Labels every triangle with its connected component. Welds first: boolean
// output has coincident-but-distinct vertices, and without a weld every triangle
// looks like its own island.
Labels labelComponents(const MeshData &src)
{
    Labels L;
    L.mesh = src;
    IritSolid::weldClose(&L.mesh);

    const int nv = int(L.mesh.pos.size() / 3);
    const int nt = L.mesh.triangleCount();
    L.triLabel.assign(nt, -1);
    if (nt == 0)
        return L;

    QVector<int> parent(nv);
    for (int i = 0; i < nv; ++i)
        parent[i] = i;

    const auto find = [&parent](int x) {
        while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
        return x;
    };
    const auto unite = [&](int a, int b) {
        a = find(a); b = find(b);
        if (a != b) parent[a] = b;
    };
    for (int t = 0; t < nt; ++t) {
        unite(L.mesh.tris[t * 3 + 0], L.mesh.tris[t * 3 + 1]);
        unite(L.mesh.tris[t * 3 + 1], L.mesh.tris[t * 3 + 2]);
    }

    QHash<int, int> rootToIdx;
    for (int t = 0; t < nt; ++t) {
        const int r = find(L.mesh.tris[t * 3]);
        auto it = rootToIdx.find(r);
        if (it == rootToIdx.end())
            it = rootToIdx.insert(r, rootToIdx.size());
        L.triLabel[t] = it.value();
    }
    L.count = rootToIdx.size();
    return L;
}

double meshVolume(const MeshData &m)
{
    return qAbs(IritSolid::signedVolume(m));
}

// Smallest gap between two axis-aligned boxes; 0 when they touch or overlap.
double boxGap(const MeshData &a, const MeshData &b)
{
    double d2 = 0.0;
    for (int k = 0; k < 3; ++k) {
        const double gap = qMax(double(a.bmin[k]) - double(b.bmax[k]),
                                double(b.bmin[k]) - double(a.bmax[k]));
        if (gap > 0.0) d2 += gap * gap;
    }
    return std::sqrt(d2);
}

// Joins two solids by concatenating and welding along their shared face. The
// fallback when a union will not run: it leaves an internal wall, but the result
// is one component and the volume stays right.
MeshData weldTogether(const MeshData &a, const MeshData &b)
{
    MeshData m = a;
    const uint32_t base = uint32_t(m.pos.size() / 3);
    m.pos.append(b.pos);
    for (uint32_t idx : b.tris)
        m.tris.append(base + idx);
    m.edges.clear();
    m.finalize();
    IritSolid::weldClose(&m);
    return m;
}

}

CageBoolean::Outcome CageBoolean::intersect(const MeshData &piece,
                                            const MeshData &model,
                                            MeshData *out, double fineNess,
                                            QString *error)
{
    *out = MeshData();

    if (piece.triangleCount() == 0 || model.triangleCount() == 0) {
        if (error) *error = QStringLiteral("empty operand");
        return Outcome::Failed;
    }

    MeshData pieceFixed = piece;
    IritSolid::orientConsistently(&pieceFixed);
    growOuterFaces(&pieceFixed, model);

    IritPrsrObjectStruct *pieceObj = IritSolid::fromMesh(pieceFixed, IritSolid::Winding::Inward);
    if (pieceObj == NULL) {
        if (error) *error = QStringLiteral("piece has no usable polygons");
        return Outcome::Failed;
    }

    MeshData modelFixed = model;
    IritSolid::orientConsistently(&modelFixed);
    IritPrsrObjectStruct *modelObj = IritSolid::fromMesh(modelFixed, IritSolid::Winding::Inward);
    if (modelObj == NULL) {
        IritPrsrFreeObject(pieceObj);
        if (error) *error = QStringLiteral("model has no usable polygons");
        return Outcome::Failed;
    }

    AndCtx ctx;
    ctx.a = pieceObj;
    ctx.b = modelObj;
    ctx.result = NULL;
    ctx.unite  = 0;

    // Coplanar handling on for the call, as GuIritDllPuzzles does. Restored
    // afterwards, which is reached even when the guard longjmps out of a fatal
    // error, so the flag is never left set for the rest of the run.
    const int oldCoplanar = IritBoolSetHandleCoplanarPoly(TRUE);
    const bool ok = IritGuard::run(&ctx, doAnd);
    IritBoolSetHandleCoplanarPoly(oldCoplanar);

    if (ok && ctx.result != NULL && ctx.result -> U.Pl == NULL) {
        return Outcome::EmptyCell;
    }

    if (!ok || ctx.result == NULL) {
        if (error)
            *error = ok ? QStringLiteral("intersection returned nothing")
                        : (IritGuard::lastError().isEmpty()
                               ? QStringLiteral("IRIT fatal error")
                               : IritGuard::lastError());
        return Outcome::Failed;
    }

    QString tessErr;
    if (!IritMesh::tessellate(ctx.result, out, fineNess, &tessErr)) {
        if (error) *error = tessErr;
        return Outcome::Failed;
    }
    IritSolid::orientConsistently(out);

    if (out->isEmpty())
        return Outcome::EmptyCell;

    return Outcome::Ok;
}

QVector<MeshData> CageBoolean::components(const MeshData &m)
{
    const Labels L = labelComponents(m);

    QVector<MeshData> out(L.count);
    QVector<QHash<uint32_t, uint32_t>> remap(L.count);

    for (int t = 0; t < L.triLabel.size(); ++t) {
        const int c = L.triLabel[t];
        if (c < 0) continue;
        MeshData &dst = out[c];
        for (int k = 0; k < 3; ++k) {
            const uint32_t v = L.mesh.tris[t * 3 + k];
            auto it = remap[c].find(v);
            if (it == remap[c].end()) {
                const uint32_t n = uint32_t(dst.pos.size() / 3);
                dst.pos.append(L.mesh.pos[v * 3 + 0]);
                dst.pos.append(L.mesh.pos[v * 3 + 1]);
                dst.pos.append(L.mesh.pos[v * 3 + 2]);
                it = remap[c].insert(v, n);
            }
            dst.tris.append(it.value());
        }
    }
    for (MeshData &d : out) {
        d.finalize();
        IritSolid::orientConsistently(&d);
    }

    std::sort(out.begin(), out.end(),
              [](const MeshData &a, const MeshData &b) {
                  return meshVolume(a) > meshVolume(b);
              });
    return out;
}

// Boolean union, used to reattach a detached lump to its neighbour.
bool CageBoolean::unite(const MeshData &a, const MeshData &b, MeshData *out,
                        double fineNess, QString *error)
{
    *out = MeshData();
    if (a.triangleCount() == 0 || b.triangleCount() == 0) {
        if (error) *error = QStringLiteral("empty operand");
        return false;
    }

    MeshData af = a, bf = b;
    IritSolid::orientConsistently(&af);
    IritSolid::orientConsistently(&bf);

    IritPrsrObjectStruct *ao = IritSolid::fromMesh(af, IritSolid::Winding::Inward);
    if (ao == NULL) { if (error) *error = QStringLiteral("no polygons"); return false; }
    IritPrsrObjectStruct *bo = IritSolid::fromMesh(bf, IritSolid::Winding::Inward);
    if (bo == NULL) {
        IritPrsrFreeObject(ao);
        if (error) *error = QStringLiteral("no polygons");
        return false;
    }

    AndCtx ctx;
    ctx.a = ao; ctx.b = bo; ctx.result = NULL; ctx.unite = 1;

    if (!IritGuard::run(&ctx, doAnd) || ctx.result == NULL) {
        if (error) *error = IritGuard::lastError().isEmpty()
                                ? QStringLiteral("union returned nothing")
                                : IritGuard::lastError();
        return false;
    }

    QString tessErr;
    if (!IritMesh::tessellate(ctx.result, out, fineNess, &tessErr)) {
        if (error) *error = tessErr;
        return false;
    }
    IritSolid::orientConsistently(out);
    return !out->isEmpty();
}

// Section 5 for every piece. Failure is local: a piece whose boolean fails or
// comes back empty is reported and skipped.
CageBoolean::Result CageBoolean::intersectAll(QVector<PuzzlePiece> *pieces,
                                              const MeshData &model,
                                              double fineNess)
{
    Result r;
    r.modelClosed = IritSolid::isClosed(model);
    if (!r.modelClosed)
        r.problems << QStringLiteral(
            "the model is not watertight - IRIT's booleans give garbage rather "
            "than an error on an open shell, so treat these results with care");

    QVector<PuzzlePiece> kept;
    kept.reserve(pieces->size());

    const double modelVol = qAbs(IritSolid::signedVolume(model));
    const double noiseFloor =
        0.005 * modelVol / qMax(1, int(pieces->size()));

    QVector<MeshData> lumps;

    for (int i = 0; i < pieces->size(); ++i) {
        PuzzlePiece p = (*pieces)[i];
        if (p.mesh.triangleCount() == 0) {
            ++r.skipped;
            continue;
        }

        bool cellHasModel = false;
        for (int v = 0; v + 2 < model.pos.size() && !cellHasModel; v += 3)
            cellHasModel =
                model.pos[v + 0] >= p.mesh.bmin[0] && model.pos[v + 0] <= p.mesh.bmax[0] &&
                model.pos[v + 1] >= p.mesh.bmin[1] && model.pos[v + 1] <= p.mesh.bmax[1] &&
                model.pos[v + 2] >= p.mesh.bmin[2] && model.pos[v + 2] <= p.mesh.bmax[2];

        MeshData trimmed;
        QString  err;
        const Outcome o = intersect(p.mesh, model, &trimmed, fineNess, &err);

        if (o == Outcome::EmptyCell && !cellHasModel) {
            ++r.dropped;
            continue;
        }
        if (o != Outcome::Ok) {
            ++r.failed;
            r.problems << QStringLiteral("piece %1: %2").arg(i).arg(
                o == Outcome::EmptyCell
                    ? QStringLiteral("empty intersection, but the cell does "
                                     "contain model geometry")
                    : err);
            kept.append(p);
            continue;
        }

        QVector<MeshData> parts = components(trimmed);
        if (parts.isEmpty()) {
            ++r.dropped;
            continue;
        }

        for (int c = 1; c < parts.size(); ++c) {
            const double v = meshVolume(parts[c]);
            if (v < noiseFloor) {
                ++r.noiseDropped;
                r.discardedVolume += v;
            } else {
                lumps.append(parts[c]);
            }
        }

        p.mesh = parts[0];
        for (int a = 0; a < 3; ++a) {
            p.centre[a] = 0.5f * (p.mesh.bmin[a] + p.mesh.bmax[a]);
            p.size[a]   = p.mesh.bmax[a] - p.mesh.bmin[a];
        }
        kept.append(p);
        ++r.intersected;
    }

    for (const MeshData &lump : lumps) {
        int    best     = -1;
        double bestGap  = 1e300;
        for (int i = 0; i < kept.size(); ++i) {
            const double g = boxGap(lump, kept[i].mesh);
            if (g < bestGap) { bestGap = g; best = i; }
        }

        bool joined = false;
        if (best >= 0) {
            MeshData merged = weldTogether(kept[best].mesh, lump);
            if (components(merged).size() == 1)
                ++r.lumpsWelded;
            else
                merged = MeshData();

            if (!merged.isEmpty()) {
                const double want = meshVolume(kept[best].mesh) + meshVolume(lump);
                const double got  = meshVolume(merged);
                if (qAbs(got - want) > 1e-6 * qMax(1.0, want))
                    r.problems << QStringLiteral("lump reattached to piece %1 "
                                                 "changed volume: expected %2, "
                                                 "got %3")
                                      .arg(best).arg(want, 0, 'g', 6)
                                      .arg(got, 0, 'g', 6);
                kept[best].mesh = merged;
                for (int a = 0; a < 3; ++a) {
                    kept[best].centre[a] =
                        0.5f * (merged.bmin[a] + merged.bmax[a]);
                    kept[best].size[a] = merged.bmax[a] - merged.bmin[a];
                }
                joined = true;
            }
        }

        if (!joined) {
            PuzzlePiece stray;
            stray.mesh = lump;
            for (int a = 0; a < 3; ++a) {
                stray.centre[a] = 0.5f * (lump.bmin[a] + lump.bmax[a]);
                stray.size[a]   = lump.bmax[a] - lump.bmin[a];
            }
            kept.append(stray);
            ++r.orphans;
        }
    }

    for (const PuzzlePiece &p : kept)
        if (components(p.mesh).size() != 1)
            ++r.multiPart;

    if (r.discardedVolume > 1e-4 * modelVol)
        r.problems << QStringLiteral("discarded debris totals %1 - %2% of the "
                                     "model, which is more than rounding")
                          .arg(r.discardedVolume, 0, 'g', 4)
                          .arg(100.0 * r.discardedVolume / modelVol, 0, 'f', 3);

    if (r.dropped > 0)
        r.notes << QStringLiteral(
            "%1 cage cell(s) held no model material and were dropped - normal "
            "for a box cage, which always covers more than the model")
                       .arg(r.dropped);

    *pieces = kept;
    return r;
}

QStringList CageBoolean::describe(const Result &r)
{
    QStringList out;
    out << QStringLiteral("BOOLEAN  %1 piece(s) intersected cleanly, %2 dropped "
                          "(empty cell), %3 failed, %4 skipped")
               .arg(r.intersected).arg(r.dropped).arg(r.failed).arg(r.skipped);
    for (const QString &n : r.notes)
        out << QStringLiteral("         %1").arg(n);
    if (r.noiseDropped > 0 || r.lumpsMerged > 0 || r.lumpsWelded > 0 ||
        r.orphans > 0)
        out << QStringLiteral("         connectedness: %1 sliver(s) discarded as "
                              "noise, %2 lump(s) unioned + %3 welded back into a "
                              "neighbour, %4 kept standalone")
                   .arg(r.noiseDropped).arg(r.lumpsMerged)
                   .arg(r.lumpsWelded).arg(r.orphans);
    out << (r.multiPart == 0
                ? QStringLiteral("         every piece is a single connected solid")
                : QStringLiteral("         WARNING: %1 piece(s) are still more "
                                 "than one solid and cannot be assembled")
                      .arg(r.multiPart));
    if (!r.modelClosed)
        out << QStringLiteral("         model is NOT watertight");
    for (int i = 0; i < qMin(10, int(r.problems.size())); ++i)
        out << QStringLiteral("         %1").arg(r.problems[i]);
    if (r.problems.size() > 10)
        out << QStringLiteral("         ... and %1 more")
                   .arg(r.problems.size() - 10);
    if (r.failed > 0)
        out << QStringLiteral("         failed pieces kept their un-trimmed cage "
                              "shape - they are still boxes");
    return out;
}
