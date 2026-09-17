//
// PuzzleDivider - implementation: the spec builders, sub-trivariate region
// extraction, the material-aware BSP, and box adjacency.
//

#include "PuzzleDivider.h"

#include "MaterialField.h"
#include "IritGuard.h"
#include "IritMesh.h"

#include <QHash>
#include <QRandomGenerator>
#include <algorithm>
#include <cstdio>
#include <cmath>
#include <cstring>

extern "C" {
#include "inc_irit/triv_lib.h"
}

float PuzzlePiece::largestSide() const
{
    return qMax(size[0], qMax(size[1], size[2]));
}

namespace {

// Domain bounds for one axis.
void axisDomain(const Trivariate &tv, int axis, double *lo, double *hi)
{
    double d[6];
    tv.domain(d);
    *lo = d[axis * 2];
    *hi = d[axis * 2 + 1];
}

// Evaluates M with `axis` set to t and the other two axes held at the given
// fractions of their domains.
bool evalOnAxis(const Trivariate &tv, int axis, double t,
                double fracB, double fracC, double p[3])
{
    double d[6], uvw[3];
    tv.domain(d);

    const int b = (axis + 1) % 3;
    const int c = (axis + 2) % 3;
    uvw[axis] = t;
    uvw[b]    = d[b * 2] + fracB * (d[b * 2 + 1] - d[b * 2]);
    uvw[c]    = d[c * 2] + fracC * (d[c * 2 + 1] - d[c * 2]);

    return tv.evaluate(uvw[0], uvw[1], uvw[2], p);
}

}

DivisionSpec PuzzleDivider::uniform(const Trivariate &tv, const int counts[3])
{
    DivisionSpec spec;
    for (int a = 0; a < 3; ++a) {
        double lo, hi;
        axisDomain(tv, a, &lo, &hi);
        const int n = qMax(1, counts[a]);
        for (int s = 1; s < n; ++s)
            spec.splits[a].append(lo + (hi - lo) * double(s) / double(n));
    }
    spec.note = QStringLiteral("uniform %1 x %2 x %3")
                    .arg(spec.cells(0)).arg(spec.cells(1)).arg(spec.cells(2));
    return spec;
}

DivisionSpec PuzzleDivider::jittered(const Trivariate &tv, const int counts[3],
                                     double jitter, quint32 seed)
{
    DivisionSpec spec = uniform(tv, counts);
    jitter = qBound(0.0, jitter, 0.45);

    QRandomGenerator rng(seed);
    for (int a = 0; a < 3; ++a) {
        double lo, hi;
        axisDomain(tv, a, &lo, &hi);
        const int n = spec.cells(a);
        if (n < 2)
            continue;
        const double cell = (hi - lo) / double(n);
        for (double &s : spec.splits[a])
            s += (rng.generateDouble() * 2.0 - 1.0) * jitter * cell;
        std::sort(spec.splits[a].begin(), spec.splits[a].end());
    }

    spec.note = QStringLiteral("jittered %1 x %2 x %3 (%4%, seed %5)")
                    .arg(spec.cells(0)).arg(spec.cells(1)).arg(spec.cells(2))
                    .arg(int(jitter * 100)).arg(seed);
    return spec;
}

namespace {

// Ordered cell boundaries per axis: domain start, interior splits, end.
void cellBounds(const Trivariate &tv, const DivisionSpec &spec,
                QVector<double> bounds[3])
{
    double dom[6];
    tv.domain(dom);
    for (int a = 0; a < 3; ++a) {
        bounds[a].clear();
        bounds[a].append(dom[a * 2]);
        for (double s : spec.splits[a])
            if (s > dom[a * 2] && s < dom[a * 2 + 1])
                bounds[a].append(s);
        bounds[a].append(dom[a * 2 + 1]);
        std::sort(bounds[a].begin(), bounds[a].end());
    }
}

// Largest physical span any single cell reaches along each parameter axis,
// measured along the cell's own edges rather than its world bounding box.
bool worstCellSpan(const Trivariate &tv, const DivisionSpec &spec, double worst[3])
{
    worst[0] = worst[1] = worst[2] = 0.0;

    QVector<double> b[3];
    cellBounds(tv, spec, b);
    bool measured = false;

    for (int i = 0; i + 1 < b[0].size(); ++i)
        for (int j = 0; j + 1 < b[1].size(); ++j)
            for (int k = 0; k + 1 < b[2].size(); ++k) {
                const double lo[3] = { b[0][i],     b[1][j],     b[2][k]     };
                const double hi[3] = { b[0][i + 1], b[1][j + 1], b[2][k + 1] };

                for (int a = 0; a < 3; ++a) {
                    const int c1 = (a + 1) % 3, c2 = (a + 2) % 3;

                    for (int e = 0; e < 4; ++e) {
                        double p0[3], p1[3];
                        p0[c1] = p1[c1] = (e & 1) ? hi[c1] : lo[c1];
                        p0[c2] = p1[c2] = (e & 2) ? hi[c2] : lo[c2];
                        p0[a] = lo[a];
                        p1[a] = hi[a];

                        double q0[3], q1[3];
                        if (!tv.evaluate(p0[0], p0[1], p0[2], q0) ||
                            !tv.evaluate(p1[0], p1[1], p1[2], q1))
                            return false;

                        const double dx = q1[0] - q0[0];
                        const double dy = q1[1] - q0[1];
                        const double dz = q1[2] - q0[2];
                        worst[a] = qMax(worst[a],
                                        std::sqrt(dx * dx + dy * dy + dz * dz));
                        measured = true;
                    }
                }
            }
    return measured;
}

}

DivisionSpec PuzzleDivider::toBuildVolume(const Trivariate &tv,
                                          const double budget[3],
                                          int maxCellsPerAxis)
{
    DivisionSpec spec;
    double scale[3] = { 1.0, 1.0, 1.0 };
    double worst[3] = { 0.0, 0.0, 0.0 };

    const double kTol = 1.02;

    for (int pass = 0; pass < 5; ++pass) {
        const double scaled[3] = { budget[0] * scale[0],
                                   budget[1] * scale[1],
                                   budget[2] * scale[2] };
        spec = splitsFromArcLength(tv, scaled, maxCellsPerAxis);

        if (!worstCellSpan(tv, spec, worst))
            break;

        bool   allFit = true;
        double over[3];
        for (int a = 0; a < 3; ++a) {
            over[a] = (budget[a] > 0.0) ? worst[a] / budget[a] : 0.0;
            if (over[a] > kTol)
                allFit = false;
        }
        if (allFit || spec.capped)
            break;

        for (int a = 0; a < 3; ++a)
            if (over[a] > kTol)
                scale[a] /= over[a] * 1.02;
    }

    double worstOver = 0.0;
    for (int a = 0; a < 3; ++a)
        if (budget[a] > 0.0)
            worstOver = qMax(worstOver, worst[a] / budget[a]);

    spec.note = QStringLiteral("fit to %1 x %2 x %3 -> %4 x %5 x %6 cells "
                               "(largest piece %7 x %8 x %9)")
                    .arg(budget[0], 0, 'g', 3).arg(budget[1], 0, 'g', 3)
                    .arg(budget[2], 0, 'g', 3)
                    .arg(spec.cells(0)).arg(spec.cells(1)).arg(spec.cells(2))
                    .arg(worst[0], 0, 'g', 3).arg(worst[1], 0, 'g', 3)
                    .arg(worst[2], 0, 'g', 3);
    if (worstOver > kTol)
        spec.note += QStringLiteral(" - OVER BUDGET by %1x, raise the cell "
                                    "limit above %2")
                         .arg(worstOver, 0, 'g', 3).arg(maxCellsPerAxis);
    return spec;
}

// Places cuts by arc length, measures the cells, then tightens each axis
// separately where they came out too big.
DivisionSpec PuzzleDivider::splitsFromArcLength(const Trivariate &tv,
                                                const double budget[3],
                                                int maxCellsPerAxis)
{
    DivisionSpec spec;
    maxCellsPerAxis = qBound(1, maxCellsPerAxis, 64);

    static const double kFrac[3] = { 0.05, 0.5, 0.95 };
    const int kSteps = 240;

    for (int a = 0; a < 3; ++a) {
        double lo, hi;
        axisDomain(tv, a, &lo, &hi);
        if (!(hi > lo) || budget[a] <= 0.0)
            continue;

        QVector<double> t(kSteps + 1), len(kSteps + 1);
        double prev[3][3][3];
        double accrued = 0.0;
        int    last    = 0;
        bool   ok      = true;

        for (int s = 0; s <= kSteps; ++s) {
            const double tt = lo + (hi - lo) * double(s) / double(kSteps);

            double here[3][3][3];
            double step = 0.0;

            for (int ib = 0; ib < 3 && ok; ++ib)
                for (int ic = 0; ic < 3 && ok; ++ic) {
                    if (!evalOnAxis(tv, a, tt, kFrac[ib], kFrac[ic], here[ib][ic]))
                        ok = false;
                    else if (s > 0) {
                        const double dx = here[ib][ic][0] - prev[ib][ic][0];
                        const double dy = here[ib][ic][1] - prev[ib][ic][1];
                        const double dz = here[ib][ic][2] - prev[ib][ic][2];
                        step = qMax(step, std::sqrt(dx * dx + dy * dy + dz * dz));
                    }
                }

            if (!ok)
                break;

            std::memcpy(prev, here, sizeof(prev));
            accrued += step;
            t[s]   = tt;
            len[s] = accrued;
            last   = s;
        }

        const double total = (last > 0) ? len[last] : 0.0;
        if (!(total > 0.0))
            continue;

        int n = int(std::ceil(total / budget[a]));
        n = qMax(n, 1);
        if (n > maxCellsPerAxis) {
            n = maxCellsPerAxis;
            spec.capped = true;
        }

        int idx = 0;
        for (int c = 1; c < n; ++c) {
            const double target = total * double(c) / double(n);
            while (idx < last && len[idx + 1] < target)
                ++idx;
            const double l0 = len[idx], l1 = len[qMin(idx + 1, last)];
            const double f  = (l1 > l0) ? (target - l0) / (l1 - l0) : 0.0;
            spec.splits[a].append(t[idx] + f * (t[qMin(idx + 1, last)] - t[idx]));
        }
        std::sort(spec.splits[a].begin(), spec.splits[a].end());
    }

    return spec;
}

namespace {

struct RegionCtx {
    const TrivTVStruct *src;
    double              p0[3], p1[3];
    TrivTVStruct       *result;
};

// Region extraction, inside IritGuard: it can raise an IRIT fatal error, so it
// touches only POD.
void doRegion(void *v)
{
    RegionCtx *c = static_cast<RegionCtx *>(v);
    static const TrivTVDirType kDir[3] =
        { TRIV_CONST_U_DIR, TRIV_CONST_V_DIR, TRIV_CONST_W_DIR };

    c -> result = NULL;

    const TrivTVStruct *cur = c -> src;
    TrivTVStruct *owned = NULL;

    for (int a = 0; a < 3; ++a) {
        TrivTVStruct *next = IritTrivTVRegionFromTV(cur, c -> p0[a], c -> p1[a],
                                                    kDir[a]);
        if (owned != NULL)
            IritTrivTVFree(owned);
        if (next == NULL)
            return;
        owned = next;
        cur   = next;
    }
    c -> result = owned;
}

}

// Extracts one sub-trivariate per cell box.
bool PuzzleDivider::divideCells(const Trivariate &tv,
                                const QVector<CellBox> &cells,
                                double fineNess, QVector<PuzzlePiece> *pieces,
                                QString *error)
{
    pieces->clear();

    if (!tv.isValid()) {
        if (error) *error = QStringLiteral("No trivariate to divide.");
        return false;
    }
    if (cells.isEmpty()) {
        if (error) *error = QStringLiteral("No cells to extract.");
        return false;
    }

    const TrivTVStruct *src = static_cast<const TrivTVStruct *>(tv.raw());
    int failedCells = 0;

    for (int c = 0; c < cells.size(); ++c) {
        const CellBox &cb = cells[c];

        RegionCtx rc;
        rc.src = src;
        for (int a = 0; a < 3; ++a) {
            rc.p0[a] = cb.lo[a];
            rc.p1[a] = cb.hi[a];
        }
        rc.result = NULL;

        if (rc.p1[0] <= rc.p0[0] || rc.p1[1] <= rc.p0[1] ||
            rc.p1[2] <= rc.p0[2]) {
            ++failedCells;
            continue;
        }

        if (!IritGuard::run(&rc, doRegion) || rc.result == NULL) {
            ++failedCells;
            continue;
        }

        PuzzlePiece piece;
        piece.i = c; piece.j = 0; piece.k = 0;
        for (int a = 0; a < 3; ++a) {
            piece.p0[a] = rc.p0[a];
            piece.p1[a] = rc.p1[a];
        }

        Trivariate cell = Trivariate::adopt(rc.result,
                              QStringLiteral("cell %1").arg(c));
        QString cellErr;
        if (!cell.tessellate(&piece.mesh, fineNess, &cellErr) ||
            piece.mesh.isEmpty()) {
            ++failedCells;
            continue;
        }

        for (int a = 0; a < 3; ++a) {
            piece.centre[a] = 0.5f * (piece.mesh.bmin[a] + piece.mesh.bmax[a]);
            piece.size[a]   = piece.mesh.bmax[a] - piece.mesh.bmin[a];
        }
        pieces->append(std::move(piece));
    }

    if (pieces->isEmpty()) {
        if (error)
            *error = QStringLiteral("All %1 cells failed to extract.")
                         .arg(cells.size());
        return false;
    }
    if (failedCells > 0 && error != nullptr)
        *error = QStringLiteral("%1 of %2 cells could not be extracted.")
                     .arg(failedCells).arg(cells.size());
    return true;
}

bool PuzzleDivider::divide(const Trivariate &tv, const DivisionSpec &spec,
                           double fineNess, QVector<PuzzlePiece> *pieces,
                           QString *error)
{
    pieces->clear();

    if (!tv.isValid()) {
        if (error) *error = QStringLiteral("No trivariate to divide.");
        return false;
    }

    double dom[6];
    tv.domain(dom);

    QVector<double> bounds[3];
    for (int a = 0; a < 3; ++a) {
        bounds[a].append(dom[a * 2]);
        for (double s : spec.splits[a])
            if (s > dom[a * 2] && s < dom[a * 2 + 1])
                bounds[a].append(s);
        bounds[a].append(dom[a * 2 + 1]);
        std::sort(bounds[a].begin(), bounds[a].end());
    }

    const int nu = bounds[0].size() - 1;
    const int nv = bounds[1].size() - 1;
    const int nw = bounds[2].size() - 1;
    if (nu < 1 || nv < 1 || nw < 1) {
        if (error) *error = QStringLiteral("The split parameters left no cells.");
        return false;
    }

    const TrivTVStruct *src = static_cast<const TrivTVStruct *>(tv.raw());
    int failedCells = 0;

    for (int i = 0; i < nu; ++i)
        for (int j = 0; j < nv; ++j)
            for (int k = 0; k < nw; ++k) {
                RegionCtx rc;
                rc.src   = src;
                rc.p0[0] = bounds[0][i];     rc.p1[0] = bounds[0][i + 1];
                rc.p0[1] = bounds[1][j];     rc.p1[1] = bounds[1][j + 1];
                rc.p0[2] = bounds[2][k];     rc.p1[2] = bounds[2][k + 1];
                rc.result = NULL;

                if (rc.p1[0] <= rc.p0[0] || rc.p1[1] <= rc.p0[1] ||
                    rc.p1[2] <= rc.p0[2]) {
                    ++failedCells;
                    continue;
                }

                if (!IritGuard::run(&rc, doRegion) || rc.result == NULL) {
                    ++failedCells;
                    continue;
                }

                PuzzlePiece piece;
                piece.i = i; piece.j = j; piece.k = k;
                for (int a = 0; a < 3; ++a) {
                    piece.p0[a] = rc.p0[a];
                    piece.p1[a] = rc.p1[a];
                }

                Trivariate cell = Trivariate::adopt(rc.result,
                                     QStringLiteral("cell %1,%2,%3").arg(i).arg(j).arg(k));
                QString cellErr;
                if (!cell.tessellate(&piece.mesh, fineNess, &cellErr) ||
                    piece.mesh.isEmpty()) {
                    ++failedCells;
                    continue;
                }

                for (int a = 0; a < 3; ++a) {
                    piece.centre[a] = 0.5f * (piece.mesh.bmin[a] + piece.mesh.bmax[a]);
                    piece.size[a]   = piece.mesh.bmax[a] - piece.mesh.bmin[a];
                }
                pieces->append(std::move(piece));
            }

    if (pieces->isEmpty()) {
        if (error)
            *error = QStringLiteral("All %1 cells failed to extract. The trivariate "
                                    "may be too coarse to subdivide this finely.")
                         .arg(nu * nv * nw);
        return false;
    }
    if (failedCells > 0 && error != nullptr)
        *error = QStringLiteral("%1 of %2 cells could not be extracted.")
                     .arg(failedCells).arg(nu * nv * nw);
    const QString outPath = "C:\Users\Admin\Documents\IRIT_to_Gcode-main\docs";

    QString err1;
    if (tv.saveToFile(outPath, &err1)) {
        std::printf("ITD   %s\n", qPrintable(outPath));
    }
    else {
        std::printf("FAIL  %s\n", qPrintable(err1));
    }
    return true;
}

// Neighbours by stepping grid coordinates. Only valid for a grid division - BSP
// cells have no (i,j,k), so use adjacencyOfBoxes for those.
QVector<PuzzleDivider::Neighbours>
PuzzleDivider::adjacency(const QVector<PuzzlePiece> &pieces, const DivisionSpec &spec)
{
    QVector<Neighbours> out;

    QHash<quint64, int> byCell;
    byCell.reserve(pieces.size());
    const auto key = [](int i, int j, int k) {
        return (quint64(quint16(i)) << 32) | (quint64(quint16(j)) << 16) | quint16(k);
    };
    for (int n = 0; n < pieces.size(); ++n)
        byCell.insert(key(pieces[n].i, pieces[n].j, pieces[n].k), n);

    for (int n = 0; n < pieces.size(); ++n) {
        const PuzzlePiece &p = pieces[n];
        const int step[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
        for (int a = 0; a < 3; ++a) {
            const auto it = byCell.constFind(key(p.i + step[a][0],
                                                 p.j + step[a][1],
                                                 p.k + step[a][2]));
            if (it != byCell.constEnd())
                out.append({ n, *it, a });
        }
    }
    Q_UNUSED(spec);
    return out;
}

namespace {

struct CutWeights {
    double balance = 1.0;
    double thin    = 0.0;
    double discon  = 0.0;
};

struct CutCost {
    double imbalance     = 0.0;
    double thinness      = 0.0;
    double disconnection = 0.0;

    double total(const CutWeights &w) const
    {
        return w.balance * imbalance
             + w.thin    * thinness
             + w.discon  * disconnection;
    }
};

struct Candidate {
    double f;
    double cost;
    double imbalance;
};

bool splitLogging()
{
    static const bool on = qEnvironmentVariableIsSet("BSP_LOG") &&
                           qEnvironmentVariableIntValue("BSP_LOG") != 0;
    return on;
}

}

QVector<PuzzleDivider::BspNode>
PuzzleDivider::buildBspTree(const double domain[6], int targetPieces,
                            double splitJitter, quint32 seed, double minSide,
                            const MaterialField *material)
{
    QVector<BspNode> tree;

    BspNode root;
    for (int a = 0; a < 3; ++a) {
        root.box.lo[a] = domain[a * 2];
        root.box.hi[a] = domain[a * 2 + 1];
    }
    tree.append(root);

    targetPieces = qBound(1, targetPieces, 4096);
    if (targetPieces == 1 || !(tree[0].box.volume() > 0.0))
        return tree;

    if (minSide <= 0.0)
        minSide = 0.45 * std::cbrt(tree[0].box.volume() / double(targetPieces));

    QRandomGenerator rng(seed);
    splitJitter = qBound(0.0, splitJitter, 0.40);

    const bool useMat = material != nullptr && material->isValid() &&
                        material->total() > 0.0;

    const int kCutCandidates = 24;
    const CutWeights kCutWeights;

    QVector<int>  leaves;
    QVector<bool> exhausted;
    leaves.append(0);
    exhausted.append(false);

    while (leaves.size() < targetPieces) {
        double total = 0.0;
        QVector<double> weight(leaves.size());
        for (int i = 0; i < leaves.size(); ++i) {
            const CellBox &lb = tree[leaves[i]].box;
            weight[i] = exhausted[i]
                ? 0.0
                : qMax(0.0, useMat ? material->volumeIn(lb.lo, lb.hi)
                                   : lb.volume());
            total    += weight[i];
        }
        if (!(total > 0.0))
            break;

        int slot = -1;
        double best = 0.0;
        for (int i = 0; i < leaves.size(); ++i)
            if (weight[i] > best) { best = weight[i]; slot = i; }
        if (slot < 0)
            break;

        const int  nodeIdx = leaves[slot];
        CellBox    cell    = tree[nodeIdx].box;

        int order[3] = { 0, 1, 2 };
        for (int i = 0; i < 3; ++i)
            for (int j = i + 1; j < 3; ++j)
                if (cell.extent(order[j]) > cell.extent(order[i]))
                    std::swap(order[i], order[j]);

        bool didSplit = false;
        for (int pass = 0; pass < 2 && !didSplit; ++pass)
        for (int t = 0; t < 3 && !didSplit; ++t) {
            const int    axis   = order[t];
            const double extent = cell.extent(axis);

            if (extent < 2.0 * minSide)
                continue;

            const double edge = minSide / extent;
            const double lo   = useMat ? edge       : qMax(0.5 - splitJitter, edge);
            const double hi   = useMat ? 1.0 - edge : qMin(0.5 + splitJitter, 1.0 - edge);
            if (hi <= lo)
                continue;

            double f = lo + rng.generateDouble() * (hi - lo);

            if (useMat) {
                const double cellMat = material->volumeIn(cell.lo, cell.hi);

                const double degenerate = 4.0 * material->voxelVolume();
                if (cellMat < 2.0 * degenerate)
                    continue;

                QVector<Candidate> cands;
                cands.reserve(kCutCandidates);
                for (int c = 0; c < kCutCandidates; ++c) {
                    const double cf = lo + (hi - lo) * (c + 0.5) / kCutCandidates;
                    double cut[3] = { cell.hi[0], cell.hi[1], cell.hi[2] };
                    cut[axis] = cell.lo[axis] + extent * cf;

                    const double left  = material->volumeIn(cell.lo, cut);
                    const double right = cellMat - left;
                    if (left <= degenerate || right <= degenerate)
                        continue;

                    CutCost cost;
                    cost.imbalance = std::fabs(left - right) / cellMat;
                    cands.append({ cf, cost.total(kCutWeights), cost.imbalance });
                }
                if (cands.isEmpty()) {
                    if (splitLogging())
                        std::printf("SPLIT cell %d · axis %c · no candidate "
                                    "leaves material on both sides\n",
                                    nodeIdx, "XYZ"[axis]);
                    continue;
                }

                std::sort(cands.begin(), cands.end(),
                          [](const Candidate &a, const Candidate &b) {
                              return a.cost < b.cost;
                          });

                if (splitLogging()) {
                    std::printf("SPLIT cell %d \u00b7 axis %c \u00b7 %d candidates\n",
                                nodeIdx, "XYZ"[axis], int(cands.size()));
                    for (int c = 0; c < cands.size(); ++c)
                        std::printf("        f=%.3f imb=%.3f%s",
                                    cands[c].f, cands[c].imbalance,
                                    (c % 5 == 4 || c + 1 == cands.size()) ? "\n" : "");
                }

                int picked = -1;
                for (int c = 0; c < cands.size() && picked < 0; ++c) {
                    if (pass != 0) { picked = c; break; }
                    double cut[3] = { cell.hi[0], cell.hi[1], cell.hi[2] };
                    cut[axis] = cell.lo[axis] + extent * cands[c].f;
                    double rlo[3] = { cell.lo[0], cell.lo[1], cell.lo[2] };
                    rlo[axis] = cut[axis];
                    if (material->isConnected(cell.lo, cut) &&
                        material->isConnected(rlo, cell.hi))
                        picked = c;
                }
                if (picked < 0) {
                    if (splitLogging())
                        std::printf("    -> none of %d kept both halves connected; "
                                    "trying another axis\n", int(cands.size()));
                    continue;
                }

                f = cands[picked].f;

                if (splitLogging())
                    std::printf("    -> chose f=%.3f  imbalance %.3f  "
                                "(rank %d of %d)\n",
                                cands[picked].f, cands[picked].imbalance,
                                picked + 1, int(cands.size()));
            }

            const double at = cell.lo[axis] + extent * f;

            BspNode left, right;
            left.box  = cell;   left.box.hi[axis]  = at;   left.parent  = nodeIdx;
            right.box = cell;   right.box.lo[axis] = at;   right.parent = nodeIdx;

            const int li = tree.size();
            tree.append(left);
            tree.append(right);
            tree[nodeIdx].child[0] = li;
            tree[nodeIdx].child[1] = li + 1;

            leaves[slot] = li;
            exhausted[slot] = false;
            leaves.append(li + 1);
            exhausted.append(false);
            didSplit = true;
        }

        if (!didSplit)
            exhausted[slot] = true;
    }

    return tree;
}

QVector<int> PuzzleDivider::leavesOf(const QVector<BspNode> &tree,
                                     QVector<CellBox> *boxes)
{
    QVector<int> out;
    if (boxes)
        boxes->clear();
    if (tree.isEmpty())
        return out;

    QVector<int> stack;
    stack.append(0);
    while (!stack.isEmpty()) {
        const int n = stack.takeLast();
        if (n < 0 || n >= tree.size())
            continue;
        if (tree[n].isLeaf()) {
            out.append(n);
            if (boxes)
                boxes->append(tree[n].box);
        }
        else {
            stack.append(tree[n].child[1]);
            stack.append(tree[n].child[0]);
        }
    }
    return out;
}

bool PuzzleDivider::splitLeaf(QVector<BspNode> &tree, int node,
                              double splitJitter, double minSide,
                              QRandomGenerator *rng)
{
    if (node < 0 || node >= tree.size() || !tree[node].isLeaf() || rng == nullptr)
        return false;

    CellBox cell = tree[node].box;
    splitJitter = qBound(0.0, splitJitter, 0.40);

    int order[3] = { 0, 1, 2 };
    for (int i = 0; i < 3; ++i)
        for (int j = i + 1; j < 3; ++j)
            if (cell.extent(order[j]) > cell.extent(order[i]))
                std::swap(order[i], order[j]);

    for (int t = 0; t < 3; ++t) {
        const int    axis   = order[t];
        const double extent = cell.extent(axis);
        if (extent < 2.0 * minSide)
            continue;

        const double edge = minSide / extent;
        const double lo   = qMax(0.5 - splitJitter, edge);
        const double hi   = qMin(0.5 + splitJitter, 1.0 - edge);
        if (hi <= lo)
            continue;

        const double f  = lo + rng->generateDouble() * (hi - lo);
        const double at = cell.lo[axis] + extent * f;

        BspNode left, right;
        left.box  = cell;   left.box.hi[axis]  = at;   left.parent  = node;
        right.box = cell;   right.box.lo[axis] = at;   right.parent = node;

        const int li = tree.size();
        tree.append(left);
        tree.append(right);
        tree[node].child[0] = li;
        tree[node].child[1] = li + 1;
        return true;
    }
    return false;
}

void PuzzleDivider::collapse(QVector<BspNode> &tree, int node)
{
    if (node < 0 || node >= tree.size())
        return;
    tree[node].child[0] = -1;
    tree[node].child[1] = -1;
}

QVector<CellBox> PuzzleDivider::buildBspCells(const double domain[6],
                                              int targetPieces,
                                              double splitJitter, quint32 seed,
                                              double minSide,
                                              const MaterialField *material)
{
    const QVector<BspNode> tree =
        buildBspTree(domain, targetPieces, splitJitter, seed, minSide, material);
    QVector<CellBox> boxes;
    leavesOf(tree, &boxes);
    return boxes;
}

// Neighbours by geometry: boxes meeting on a plane with real overlap. Works for
// any division, BSP cells included.
QVector<PuzzleDivider::Neighbours>
PuzzleDivider::adjacencyOfBoxes(const QVector<PuzzlePiece> &pieces, double eps)
{
    QVector<Neighbours> out;

    for (int i = 0; i < pieces.size(); ++i)
        for (int j = i + 1; j < pieces.size(); ++j) {
            const PuzzlePiece &A = pieces[i], &B = pieces[j];

            for (int a = 0; a < 3; ++a) {
                const bool meets = std::fabs(A.p1[a] - B.p0[a]) <= eps ||
                                   std::fabs(B.p1[a] - A.p0[a]) <= eps;
                if (!meets)
                    continue;

                bool overlaps = true;
                for (int b = 0; b < 3 && overlaps; ++b) {
                    if (b == a)
                        continue;
                    const double lo = qMax(A.p0[b], B.p0[b]);
                    const double hi = qMin(A.p1[b], B.p1[b]);
                    if (hi - lo <= eps)
                        overlaps = false;
                }
                if (overlaps) {
                    out.append({ i, j, a });
                    break;
                }
            }
        }
    return out;
}
