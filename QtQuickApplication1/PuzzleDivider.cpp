#include "PuzzleDivider.h"
#include "IritGuard.h"
#include "IritMesh.h"          // brings in the IRIT C headers

#include <QHash>
#include <QRandomGenerator>
#include <algorithm>
#include <cmath>
#include <cstring>

extern "C" {
#include "inc_irit/triv_lib.h"
}

float PuzzlePiece::largestSide() const
{
    return qMax(size[0], qMax(size[1], size[2]));
}

// ------------------------------------------------------------ split makers --

namespace {

// Domain bounds for one axis.
void axisDomain(const Trivariate &tv, int axis, double *lo, double *hi)
{
    double d[6];
    tv.domain(d);
    *lo = d[axis * 2];
    *hi = d[axis * 2 + 1];
}

// Evaluates M at a parameter point with `axis` set to t and the other two axes
// held at the given fractions of their domains.
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

} // namespace

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
    jitter = qBound(0.0, jitter, 0.45);         // beyond 0.45 splits can cross

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

// Ordered cell boundaries per axis: domain start, interior splits, domain end.
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

// Largest physical span any single cell reaches along each PARAMETER axis.
//
// Measuring the cell's world-axis bounding box instead was wrong: for anything
// but a box, world X picks up contributions from u, v and w at once, so an
// overshoot could never be attributed to the axis that caused it. Here each
// cell is measured along its own u, v and w edges, which is the quantity the
// cut spacing on that axis actually controls.
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

                    // The cell's four edges parallel to axis `a`. Straight-line
                    // distance, not arc: what has to fit on a print bed is the
                    // piece's actual extent, and a curved edge measures shorter
                    // corner to corner than along its length.
                    for (int e = 0; e < 4; ++e) {
                        double p0[3], p1[3];
                        p0[c1] = p1[c1] = (e & 1) ? hi[c1] : lo[c1];
                        p0[c2] = p1[c2] = (e & 2) ? hi[c2] : lo[c2];
                        p0[a] = lo[a];
                        p1[a] = hi[a];

                        double q0[3], q1[3];
                        if (!tv.evaluate(p0[0], p0[1], p0[2], q0) ||
                            !tv.evaluate(p1[0], p1[1], p1[2], q1))
                            return false;        // scalar trivariate

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

} // namespace

DivisionSpec PuzzleDivider::toBuildVolume(const Trivariate &tv,
                                          const double budget[3],
                                          int maxCellsPerAxis)
{
    // Cuts are placed by arc length, the resulting cells are measured, and the
    // budget is tightened where they came out too big.
    //
    // Each axis is tightened SEPARATELY. A single shared factor meant that one
    // overshooting axis dragged the other two down with it, subdividing axes
    // that were already comfortably inside the limit - which is exactly what
    // makes a "roughly equal size" division come out with a 2x spread and far
    // more pieces than asked for.
    DivisionSpec spec;
    double scale[3] = { 1.0, 1.0, 1.0 };
    double worst[3] = { 0.0, 0.0, 0.0 };

    // Landing 0.02% over a limit is a hit, not a miss. Without a tolerance the
    // loop runs every pass, shrinks the budget each time, and still reports
    // failure.
    const double kTol = 1.02;

    for (int pass = 0; pass < 5; ++pass) {
        const double scaled[3] = { budget[0] * scale[0],
                                   budget[1] * scale[1],
                                   budget[2] * scale[2] };
        spec = splitsFromArcLength(tv, scaled, maxCellsPerAxis);

        if (!worstCellSpan(tv, spec, worst))
            break;                          // nothing measurable; keep the guess

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
                scale[a] /= over[a] * 1.02;   // only the axis that overshot
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

DivisionSpec PuzzleDivider::splitsFromArcLength(const Trivariate &tv,
                                                const double budget[3],
                                                int maxCellsPerAxis)
{
    DivisionSpec spec;
    maxCellsPerAxis = qBound(1, maxCellsPerAxis, 64);

    // Nine sample lines per axis: the centre plus corner-ish positions of the
    // other two parameters. A cell's physical size varies across the domain
    // when M is non-isometric, so measuring only the centre line would
    // undershoot near the boundary.
    static const double kFrac[3] = { 0.05, 0.5, 0.95 };
    const int kSteps = 240;

    for (int a = 0; a < 3; ++a) {
        double lo, hi;
        axisDomain(tv, a, &lo, &hi);
        if (!(hi > lo) || budget[a] <= 0.0)
            continue;

        // Pass 1: cumulative physical length along the axis, sampled densely.
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
                break;                   // scalar trivariate: nothing to measure

            std::memcpy(prev, here, sizeof(prev));
            accrued += step;
            t[s]   = tt;
            len[s] = accrued;
            last   = s;
        }

        const double total = (last > 0) ? len[last] : 0.0;
        if (!(total > 0.0))
            continue;

        // Pass 2: how many cells the budget asks for, then splits placed at
        // equal *physical* increments. Walking the whole axis first and
        // dividing after is what keeps the last cell from swallowing the rest
        // of the domain when the cap bites.
        int n = int(std::ceil(total / budget[a]));
        n = qMax(n, 1);
        if (n > maxCellsPerAxis) {
            n = maxCellsPerAxis;
            spec.capped = true;          // budget cannot be met within the cap
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

// ------------------------------------------------------------------ divide --

namespace {

// Region extraction can raise an IRIT fatal error, so it runs inside the guard
// and touches only POD.
struct RegionCtx {
    const TrivTVStruct *src;
    double              p0[3], p1[3];
    TrivTVStruct       *result;
};

void doRegion(void *v)
{
    RegionCtx *c = static_cast<RegionCtx *>(v);
    static const TrivTVDirType kDir[3] =
        { TRIV_CONST_U_DIR, TRIV_CONST_V_DIR, TRIV_CONST_W_DIR };

    c -> result = NULL;

    // Cut one axis at a time; each call narrows the box a little further.
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

} // namespace

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

    // Full ordered boundary list per axis: domain start, interior splits, end.
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

                // A zero-width cell would make region extraction meaningless.
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

                // Tessellate through the same path a whole file takes, so a
                // piece and the original model are rendered identically.
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
    return true;
}

QVector<PuzzleDivider::Neighbours>
PuzzleDivider::adjacency(const QVector<PuzzlePiece> &pieces, const DivisionSpec &spec)
{
    QVector<Neighbours> out;

    // Index cells by (i,j,k) so a missing cell simply has no edges.
    QHash<quint64, int> byCell;
    byCell.reserve(pieces.size());
    const auto key = [](int i, int j, int k) {
        return (quint64(quint16(i)) << 32) | (quint64(quint16(j)) << 16) | quint16(k);
    };
    for (int n = 0; n < pieces.size(); ++n)
        byCell.insert(key(pieces[n].i, pieces[n].j, pieces[n].k), n);

    for (int n = 0; n < pieces.size(); ++n) {
        const PuzzlePiece &p = pieces[n];
        // Only step forward on each axis, so every pair is reported once.
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

// ------------------------------------------------------------- BSP cells --

QVector<CellBox> PuzzleDivider::buildBspCells(const double domain[6],
                                              int targetPieces,
                                              double splitJitter, quint32 seed)
{
    QVector<CellBox> cells;

    CellBox root;
    for (int a = 0; a < 3; ++a) {
        root.lo[a] = domain[a * 2];
        root.hi[a] = domain[a * 2 + 1];
    }
    cells.append(root);

    targetPieces = qBound(1, targetPieces, 4096);
    if (targetPieces == 1 || !(root.volume() > 0.0))
        return cells;

    QRandomGenerator rng(seed);
    splitJitter = qBound(0.0, splitJitter, 0.40);

    while (cells.size() < targetPieces) {
        // Pick a cell in proportion to its volume. Splitting uniformly at
        // random would keep re-splitting whatever is already smallest and end
        // up with a cloud of slivers next to one big block; weighting by volume
        // keeps the sizes varied but comparable.
        double total = 0.0;
        QVector<double> weight(cells.size());
        for (int i = 0; i < cells.size(); ++i) {
            weight[i] = qMax(0.0, cells[i].volume());
            total    += weight[i];
        }
        if (!(total > 0.0))
            break;

        double r = rng.generateDouble() * total;
        int pick = cells.size() - 1;
        for (int i = 0; i < cells.size(); ++i) {
            r -= weight[i];
            if (r <= 0.0) { pick = i; break; }
        }

        // Always cut the longest axis. Cutting a random axis lets a cell get
        // repeatedly sliced the same way and turn into a wafer, which is both
        // ugly and unprintable.
        CellBox cell = cells[pick];
        int    axis  = 0;
        double best  = -1.0;
        for (int a = 0; a < 3; ++a)
            if (cell.extent(a) > best) { best = cell.extent(a); axis = a; }
        if (!(best > 0.0))
            break;

        // Near the middle, nudged. Splitting close to an end is what produces
        // the slivers, so the jitter is bounded well inside the cell.
        const double f  = 0.5 + (rng.generateDouble() * 2.0 - 1.0) * splitJitter;
        const double at = cell.lo[axis] + best * f;

        CellBox right = cell;
        right.lo[axis] = at;
        cell.hi[axis]  = at;

        cells[pick] = cell;              // assign before append: appending can
        cells.append(right);             // reallocate and invalidate references
    }
    return cells;
}

QVector<PuzzleDivider::Neighbours>
PuzzleDivider::adjacencyOfBoxes(const QVector<PuzzlePiece> &pieces, double eps)
{
    QVector<Neighbours> out;

    // BSP cells have no grid indices to step through, so neighbours are found
    // geometrically: two boxes meet on a plane and their footprints there
    // overlap with real area. Touching only along an edge or at a corner is not
    // a shared face and must not become a joint.
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
                    break;               // one shared face is enough
                }
            }
        }
    return out;
}
