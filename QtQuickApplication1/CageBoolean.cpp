#include "CageBoolean.h"

#include "IritGuard.h"
#include "IritMesh.h"
#include "IritSolid.h"

#include <QHash>

#include <algorithm>
#include <cmath>

namespace {

// The boolean runs inside IritGuard, which longjmps out on an IRIT fatal error,
// so the context holds only POD and no C++ object is live across the call.
struct AndCtx {
    IritPrsrObjectStruct *a;         // the cage piece
    IritPrsrObjectStruct *b;         // the model
    IritPrsrObjectStruct *result;
    int                   unite;     // 0 = intersect, 1 = union
};

void doAnd(void *v)
{
    AndCtx *c = static_cast<AndCtx *>(v);

    // Both operands defensively re-closed: an object out of the tessellator or
    // out of a previous boolean can have open vertex lists, and IRIT rejects
    // those outright rather than coping.
    IritSolid::closeLists(c -> a);
    IritSolid::closeLists(c -> b);

    c -> result = c -> unite ? IritBooleanOR(c -> a, c -> b)
                             : IritBooleanAND(c -> a, c -> b);
}


// Labels every triangle with the connected component it belongs to. Welds
// first: boolean output arrives with coincident-but-distinct vertices, and
// without a weld every triangle looks like its own island.
struct Labels {
    MeshData     mesh;      // the welded mesh the labels refer to
    QVector<int> triLabel;
    int          count = 0;
};

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

    // Iterative find with path halving - a recursive one blows the stack on a
    // 20k-triangle piece.
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


// Joins two solids by concatenating them and welding the coincident vertices
// along the cut face they share.
//
// The fallback for when a union will not run. It leaves the shared face in
// place as an internal wall, which a clean union would have dissolved, but it
// cannot fail and it gets both properties that matter here: the two lumps end
// up ONE connected component, and the volume stays right - the wall is
// traversed once in each direction, so its contribution to the divergence
// integral cancels regardless of how either side happens to be triangulated.
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

} // namespace

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

    // The cage pieces come from trivariate boundary tessellation, which does
    // not guarantee a consistently wound solid. Fixing that here rather than
    // inside fromMesh keeps the conversion itself a pure translation.
    MeshData pieceFixed = piece;
    IritSolid::orientConsistently(&pieceFixed);

    IritPrsrObjectStruct *pieceObj = IritSolid::fromMesh(pieceFixed, IritSolid::Winding::Inward);
    if (pieceObj == NULL) {
        if (error) *error = QStringLiteral("piece has no usable polygons");
        return Outcome::Failed;
    }

    // A fresh copy of the model per piece. IRIT's booleans consume and modify
    // their operands, so sharing one object across every piece would corrupt it
    // after the first intersection.
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

    const bool ok = IritGuard::run(&ctx, doAnd);

    if (ok && ctx.result != NULL && ctx.result -> U.Pl == NULL) {
        // Succeeded with nothing in it: the cell holds no material.
        return Outcome::EmptyCell;
    }

    if (!ok || ctx.result == NULL) {
        // The operands are IRIT's to free once the boolean has taken them, and
        // after a longjmp their state is unknown, so they are deliberately left
        // alone here. Leaking one failed piece beats a double free.
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
    // The result comes back wound the way it was fed in - inward, which is
    // inside-out for everything downstream. Put it back on the app's
    // convention so volumes, normals and shading stay meaningful.
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
        // Each component must stand on its own as an outward-wound solid: it is
        // about to be measured and possibly merged independently of the mesh it
        // came out of.
        IritSolid::orientConsistently(&d);
    }

    std::sort(out.begin(), out.end(),
              [](const MeshData &a, const MeshData &b) {
                  return meshVolume(a) > meshVolume(b);
              });
    return out;
}


// Boolean union, used to reattach a detached lump to its neighbour. Same
// winding convention and the same guarded call as the intersection.
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

    // Anything under this is debris rather than a piece, and is discarded.
    //
    // Measured against an AVERAGE PIECE, not against the model, because that is
    // what decides whether a lump is worth keeping: half a percent of a piece is
    // far below anything that can be printed or handled. On the armadillo at 6
    // pieces the floor lands near 200 - which discards a 29-unit speck that was
    // otherwise being promoted to a seventh "piece", while keeping the genuine
    // detached lumps that run 500..1100.
    //
    // The cost is explicit: the discarded volume is reported, so a run that
    // throws away more than a rounding error says so rather than quietly
    // returning less than the model.
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

        // Does this cell hold any of the model at all? Used only to sanity
        // check a drop: an empty result for a cell that demonstrably contains
        // model vertices is a failure, not an empty cell, and is reported as
        // one rather than quietly discarding material.
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
            ++r.dropped;                      // correct: nothing there to keep
            continue;
        }
        if (o != Outcome::Ok) {
            ++r.failed;
            r.problems << QStringLiteral("piece %1: %2").arg(i).arg(
                o == Outcome::EmptyCell
                    ? QStringLiteral("empty intersection, but the cell does "
                                     "contain model geometry")
                    : err);
            kept.append(p);                   // keeps its boxy geometry
            continue;
        }

        // One cell can hold several disjoint lumps of material. Keep the
        // largest as the piece, bin the numerical debris, and hold the rest
        // aside to be given back to whichever neighbour they belong to.
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

    // Give every detached lump back to the piece it is actually attached to.
    // The lump was severed by a cut plane, so the material continues into the
    // neighbour on the other side of it: that neighbour is the one whose box
    // the lump touches. A union rather than a concatenation, so the shared cut
    // face is dissolved and the result is one solid rather than two boxes
    // glued together.
    for (const MeshData &lump : lumps) {
        int    best     = -1;
        double bestGap  = 1e300;
        for (int i = 0; i < kept.size(); ++i) {
            const double g = boxGap(lump, kept[i].mesh);
            if (g < bestGap) { bestGap = g; best = i; }
        }

        bool joined = false;
        if (best >= 0) {
            // Welded, not unioned. IritBooleanOR is the right operation here in
            // principle, but the operands share a cut face exactly, and that is
            // the configuration IRIT's booleans handle worst: across every run
            // measured it declined 100% of the time - and it does so by way of
            // an assert inside bool1low.c, which in a GUI build puts up a modal
            // dialog the guard cannot suppress (IRIT's static libs carry their
            // own CRT, so _CrtSetReportMode here does not reach it). Calling it
            // therefore bought nothing and cost a crash.
            //
            // The weld leaves the shared face as an internal wall where a union
            // would have dissolved it. Both properties that matter survive: the
            // result is ONE connected component, and the volume is right - the
            // wall is traversed once in each direction, so it cancels.
            MeshData merged = weldTogether(kept[best].mesh, lump);
            if (components(merged).size() == 1)
                ++r.lumpsWelded;
            else
                merged = MeshData();              // did not actually join

            if (!merged.isEmpty()) {
                // Reattaching must conserve volume exactly. Checked rather than
                // assumed: an inside-out lump subtracts instead of adding, and
                // the only symptom is a total that looks merely plausible.
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
            // Touched nothing, or the union would not have produced a single
            // solid. Standing it up as its own piece is honest; silently
            // gluing it to a neighbour it does not adjoin would not be.
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

    // Postcondition. Every piece must be exactly one solid; anything else is a
    // piece nobody can print or assemble, and it gets said out loud.
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
