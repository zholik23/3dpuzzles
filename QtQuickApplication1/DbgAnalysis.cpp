//
// DbgAnalysis - implementation.
//

#include "DbgAnalysis.h"

#include "CurvedBsp.h"

#include <QStringList>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <unordered_map>
#include <vector>

namespace {

struct V3 { double x = 0, y = 0, z = 0; };

inline double dot(const V3 &a, const V3 &b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3 cross(const V3 &a, const V3 &b)
{
    return { a.y * b.z - a.z * b.y,
             a.z * b.x - a.x * b.z,
             a.x * b.y - a.y * b.x };
}
inline double len(const V3 &a) { return std::sqrt(dot(a, a)); }
inline V3 unit(const V3 &a)
{
    const double l = len(a);
    return (l > 1e-300) ? V3 { a.x / l, a.y / l, a.z / l } : V3 { 0, 0, 0 };
}

inline int popcount64(uint64_t v)
{
    int n = 0;
    while (v) { v &= v - 1; ++n; }
    return n;
}

V3 evalAt(const Trivariate &tv, const double p[3])
{
    double r[3] = { 0, 0, 0 };
    tv.evaluate(p[0], p[1], p[2], r);
    return { r[0], r[1], r[2] };
}

// Central difference along one domain axis, stepping inward at the edges so the
// sample never leaves the domain.
V3 tangent(const Trivariate &tv, const double p[3], int axis,
           const double dom[6], double h)
{
    const double lo = dom[axis * 2], hi = dom[axis * 2 + 1];
    double a[3] = { p[0], p[1], p[2] }, b[3] = { p[0], p[1], p[2] };

    a[axis] = std::max(lo, p[axis] - h);
    b[axis] = std::min(hi, p[axis] + h);
    const double span = b[axis] - a[axis];
    if (!(span > 0.0))
        return { 0, 0, 0 };

    const V3 pa = evalAt(tv, a), pb = evalAt(tv, b);
    return { (pb.x - pa.x) / span, (pb.y - pa.y) / span, (pb.z - pa.z) / span };
}

// Directions spread evenly over the sphere (Fibonacci), so no axis is favoured.
// An axis-aligned direction set would call a curved interface blocked simply by
// missing the one direction that works.
//
// Every direction comes with its exact opposite. A group sliding along d is the
// same event as the rest of the puzzle sliding along -d, so the two must get
// the same answer; without the pairing a thin set of free directions could be
// hit for one side and missed for the other, and groups stopped coming in pairs.
std::vector<V3> sphereDirections(int n)
{
    const int half = std::max(1, n / 2);
    std::vector<V3> d;
    d.reserve(half * 2);
    const double golden = 2.399963229728653;
    for (int i = 0; i < half; ++i) {
        const double z = 1.0 - (i + 0.5) / half;          // upper hemisphere
        const double r = std::sqrt(std::max(0.0, 1.0 - z * z));
        const double phi = golden * i;
        d.push_back({ r * std::cos(phi), r * std::sin(phi), z });
    }
    for (int i = 0; i < half; ++i)
        d.push_back({ -d[i].x, -d[i].y, -d[i].z });
    return d;
}

// How far a group can tilt its pull and still slide out: the clearance.
//
// A group moves along d when d . p <= 0 for every constraint normal p on its
// boundary. Such a d exists exactly when the origin lies outside the convex
// hull of those normals, and then
//
//     max over |d| <= 1 of  min_p ( -d . p )  =  distance(origin, hull)
//
// which is the sine of the widest angle d can move off the best direction while
// still clearing every normal. Gilbert's algorithm walks towards the hull point
// nearest the origin; at every step x . v / |x| (v the support point) is a
// certified LOWER bound on that distance and |x| an upper bound. So "free" is
// decided with a proof, not by hoping a sampled direction lands in a thin set.
struct Clearance {
    double sinc = 0.0;   // lower bound on sin(clearance); <= kBlocked means blocked
    V3     dir;          // the direction with the most clearance (when free)
};

// Below this the free set is narrower than 0.001 deg: physically meaningless,
// and where a knife-edge contact (exactly 90 deg of spread) ends up.
constexpr double kBlocked = 1.7453e-5;

Clearance clearanceOf(const std::vector<const V3 *> &pts, const std::vector<int> &counts,
                      const std::vector<double> &signs)
{
    Clearance c;
    // Support point: argmin over all constraint points of x . p.
    const auto support = [&](const V3 &x, V3 *best) {
        double bd = 1e300;
        for (size_t f = 0; f < pts.size(); ++f)
            for (int i = 0; i < counts[f]; ++i) {
                const V3 p { signs[f] * pts[f][i].x, signs[f] * pts[f][i].y, signs[f] * pts[f][i].z };
                const double d = dot(x, p);
                if (d < bd) { bd = d; *best = p; }
            }
        return bd;
    };
    if (pts.empty())
        return c;
    V3 x { signs[0] * pts[0][0].x, signs[0] * pts[0][0].y, signs[0] * pts[0][0].z };
    double bestLb = -1.0;
    V3 bestDir;
    for (int it = 0; it < 20000; ++it) {
        const double xx = dot(x, x);
        const double xl = std::sqrt(xx);
        if (xl < kBlocked)
            break;                                  // the origin is (practically) in the hull
        V3 v;
        const double xv = support(x, &v);
        const double lb = xv / xl;
        if (lb > bestLb) { bestLb = lb; bestDir = V3 { -x.x / xl, -x.y / xl, -x.z / xl }; }
        if (xl - lb < 1e-7)
            break;                                  // converged: |x| is the distance
        const V3 dv { v.x - x.x, v.y - x.y, v.z - x.z };
        const double den = dot(dv, dv);
        if (!(den > 1e-300))
            break;
        const double t = std::clamp(-dot(x, dv) / den, 0.0, 1.0);
        x = V3 { x.x + t * dv.x, x.y + t * dv.y, x.z + t * dv.z };
    }
    c.sinc = bestLb;
    c.dir = bestDir;
    return c;
}

inline double clearanceDeg(double sinc)
{
    return std::asin(std::clamp(sinc, -1.0, 1.0)) * 180.0 / 3.14159265358979323846;
}

// The Jacobian of M over the whole domain, boundary included.
void measureJacobian(const Trivariate &tv, const DbgOptions &opt, DbgReport &rep)
{
    double dom[6];
    tv.domain(dom);
    double h[3];
    for (int a = 0; a < 3; ++a)
        h[a] = (dom[a * 2 + 1] - dom[a * 2]) * 1e-5;

    // --- Jacobian of M over the domain -------------------------------------
    //
    // det J <= 0 means the map folds - the rubber turns inside out - and the
    // partition is meaningless there.
    //
    // Sampled INCLUDING the boundary, because a surface of revolution is
    // degenerate at its poles and those sit exactly on v = 0 and v = 1; a
    // centre-only grid reports a healthy Jacobian for a singular map. The
    // strictly-interior minimum is kept separately, since a polar degeneracy is
    // intrinsic to revolution and is not the same fault as folding.
    bool firstDet = true, firstInt = true;
    const int g = std::max(2, opt.jacobianGrid);
    for (int i = 0; i <= g; ++i)
        for (int j = 0; j <= g; ++j)
            for (int k = 0; k <= g; ++k) {
                double p[3];
                p[0] = dom[0] + (dom[1] - dom[0]) * double(i) / g;
                p[1] = dom[2] + (dom[3] - dom[2]) * double(j) / g;
                p[2] = dom[4] + (dom[5] - dom[4]) * double(k) / g;

                const double det = dot(tangent(tv, p, 0, dom, h[0]),
                                       cross(tangent(tv, p, 1, dom, h[1]),
                                             tangent(tv, p, 2, dom, h[2])));

                if (firstDet || det < rep.minDetJ) {
                    rep.minDetJ = det;
                    rep.minDetAt[0] = p[0];
                    rep.minDetAt[1] = p[1];
                    rep.minDetAt[2] = p[2];
                    firstDet = false;
                }
                const bool interior = (i > 0 && i < g) && (j > 0 && j < g) && (k > 0 && k < g);
                if (interior && (firstInt || det < rep.minDetInterior)) {
                    rep.minDetInterior = det;
                    firstInt = false;
                }
            }
    rep.jacobianOk = rep.minDetJ > 0.0;
}

// One shared face between two pieces, before analysis: which pieces, and
// the normals sampled on it (unit, outward from the low-side piece a).
struct RawInterface {
    DbgPair          pair;
    std::vector<V3>  norms;
};

// Everything after the interfaces: per-pair verdicts, then every group,
// the openings, single key and level k, and one take-apart. It does not
// care how the pieces were cut - a grid or a curved BSP feeds it alike.
void finishBlocking(DbgReport &rep, const DbgOptions &opt, std::vector<RawInterface> &in)
{
    // --- interfaces ---------------------------------------------------------
    //
    // Only interior cuts are shared between two pieces; the outer boundary of
    // the domain maps to the model's own surface.
    const std::vector<V3> dirs = sphereDirections(std::max(8, opt.directions));
    const int nDirs = int(dirs.size());
    const int nWords = (nDirs + 63) / 64;
    const int fs = std::max(2, opt.faceSamples);

    // Per interface, one bit per direction: may the low- (high-) side piece
    // translate that way without pushing into its neighbour? Storing it as a
    // bitset is what makes the subset search below cheap - a group's mobility
    // becomes a bitwise AND rather than a loop over normals.
    std::vector<uint64_t> okLow, okHigh;
    // The same, with an allowance of `slack`: a direction is kept if it clears
    // every normal to within slack. Every direction lies within `spacing` of a
    // sampled one, so if even the relaxed sets leave nothing, no direction at
    // all clears the normals: the group is certainly blocked, and the exact
    // clearance computation can be skipped.
    std::vector<uint64_t> okLowR, okHighR;
    const double spacing = 2.0 * std::sqrt(4.0 * 3.14159265358979323846 / nDirs);
    const double slack = 2.0 * std::sin(0.5 * spacing) + 1e-9;
    std::vector<int> ifA, ifB;
    std::vector<std::vector<V3>> ifNorms;   // outward from the low-side piece

    for (RawInterface &ri : in) {
        DbgPair pr = ri.pair;
        std::vector<V3> &norms = ri.norms;
        if (norms.empty())
            continue;
        pr.normals = int(norms.size());

        V3 mean { 0, 0, 0 };
        for (const V3 &nv : norms) { mean.x += nv.x; mean.y += nv.y; mean.z += nv.z; }
        mean = unit(mean);
        double worst = 0.0;
        for (const V3 &nv : norms)
            worst = std::max(worst,
                             std::acos(std::clamp(dot(nv, mean), -1.0, 1.0)));
        pr.spreadDeg = worst * 180.0 / 3.14159265358979323846;

        const size_t base = okLow.size();
        okLow.resize(base + nWords, 0);
        okHigh.resize(base + nWords, 0);
        okLowR.resize(base + nWords, 0);
        okHighR.resize(base + nWords, 0);

        int lowCount = 0, highCount = 0;
        for (int d = 0; d < nDirs; ++d) {
            bool lo = true, hi = true, loR = true, hiR = true;
            for (const V3 &nv : norms) {
                const double s = dot(dirs[d], nv);
                if (s >  opt.touch) lo = false;   // low side moves: d.n <= 0
                if (s < -opt.touch) hi = false;   // high side: (-d).n <= 0
                if (s >  slack) loR = false;
                if (s < -slack) hiR = false;
                if (!loR && !hiR) break;
            }
            const uint64_t bit = uint64_t(1) << (d % 64);
            if (lo)  { okLow [base + d / 64] |= bit; ++lowCount; }
            if (hi)  { okHigh[base + d / 64] |= bit; ++highCount; }
            if (loR) okLowR [base + d / 64] |= bit;
            if (hiR) okHighR[base + d / 64] |= bit;
        }

        pr.freeDirs = lowCount;
        (void)highCount;

        // Blocked or not is decided by the clearance, not the
        // count: a free set thinner than the direction sampling
        // still counts as free.
        {
            const std::vector<const V3 *> pts { norms.data() };
            const std::vector<int> cnt { int(norms.size()) };
            const std::vector<double> sg { 1.0 };
            const Clearance cl = clearanceOf(pts, cnt, sg);
            pr.blocked = !(cl.sinc > kBlocked);
            pr.clearanceDeg = pr.blocked ? 0.0 : clearanceDeg(cl.sinc);
        }

        ifA.push_back(pr.a);
        ifB.push_back(pr.b);
        {
            std::vector<V3> uniq;
            for (const V3 &nv : norms) {
                bool dup = false;
                for (const V3 &q : uniq)
                    if (std::fabs(q.x - nv.x) + std::fabs(q.y - nv.y) + std::fabs(q.z - nv.z) < 1e-9) { dup = true; break; }
                if (!dup) uniq.push_back(nv);
            }
            ifNorms.push_back(std::move(uniq));
        }
        rep.pairs.append(pr);
    }

    const int nIf = int(ifA.size());
    for (const DbgPair &p : rep.pairs)
        if (p.blocked) ++rep.blockedPairs;

    // --- mobility of a set of pieces ----------------------------------------
    //
    // A set moves along d only when every interface CROSSING ITS BOUNDARY
    // permits it. Interfaces buried inside the set impose nothing, because both
    // of their pieces travel together - which is exactly why a group can escape
    // where no single member could.
    //
    // `active` is the set of pieces still in the puzzle: once a piece has been
    // taken out, its interfaces stop constraining anything. That is what lets
    // the same test answer the recursive questions below.
    std::vector<uint64_t> acc(nWords);
    const uint32_t allMask = (rep.pieces >= 32) ? 0xffffffffu
                                                : ((uint32_t(1) << rep.pieces) - 1);
    const auto mobileIn = [&](uint32_t mask, uint32_t active) {
        for (int w = 0; w < nWords; ++w) acc[w] = ~uint64_t(0);
        if (nDirs % 64)
            acc[nWords - 1] = (uint64_t(1) << (nDirs % 64)) - 1;

        for (int i = 0; i < nIf; ++i) {
            if (!((active >> ifA[i]) & 1u) || !((active >> ifB[i]) & 1u))
                continue;                       // one side already removed
            const bool inA = (mask >> ifA[i]) & 1u;
            const bool inB = (mask >> ifB[i]) & 1u;
            if (inA == inB)
                continue;                       // internal, or wholly outside
            const std::vector<uint64_t> &src = inA ? okLow : okHigh;
            const size_t base = size_t(i) * nWords;
            uint64_t any = 0;
            for (int w = 0; w < nWords; ++w) { acc[w] &= src[base + w]; any |= acc[w]; }
            if (!any) return 0;
        }
        int c = 0;
        for (int w = 0; w < nWords; ++w) c += popcount64(acc[w]);
        return c;
    };
    const auto mobileDirs = [&](uint32_t mask) { return mobileIn(mask, allMask); };

    // The decision itself: the clearance of a group, given the pieces still
    // present. Cached, because the level and take-apart searches revisit the
    // same (group, pieces present) combinations.
    std::unordered_map<uint64_t, Clearance> clrCache;
    const auto clearIn = [&](uint32_t mask, uint32_t active) -> Clearance {
        const uint64_t key = (uint64_t(active) << 32) | mask;
        const auto hit = clrCache.find(key);
        if (hit != clrCache.end())
            return hit->second;

        // Past opt.exactUpTo pieces the searches visit hundreds of thousands of
        // groups, too many for the exact computation: fall back to the sampled
        // directions, and say so in the report.
        if (!rep.exactClearance) {
            Clearance c;
            const int cnt = mobileIn(mask, active);
            if (cnt > 0) {
                V3 sum { 0, 0, 0 };
                for (int d = 0; d < nDirs; ++d)
                    if ((acc[d / 64] >> (d % 64)) & 1u) { sum.x += dirs[d].x; sum.y += dirs[d].y; sum.z += dirs[d].z; }
                c.dir = unit(sum);
                c.sinc = 1.0;                   // free, clearance unknown
            } else {
                c.sinc = -1.0;
            }
            if (clrCache.size() < 200000) clrCache.emplace(key, c);
            return c;
        }

        // Certified shortcut: nothing survives even with the allowance.
        {
            for (int w = 0; w < nWords; ++w) acc[w] = ~uint64_t(0);
            if (nDirs % 64)
                acc[nWords - 1] = (uint64_t(1) << (nDirs % 64)) - 1;
            bool any = true;
            for (int i = 0; i < nIf && any; ++i) {
                if (!((active >> ifA[i]) & 1u) || !((active >> ifB[i]) & 1u))
                    continue;
                const bool inA = (mask >> ifA[i]) & 1u;
                const bool inB = (mask >> ifB[i]) & 1u;
                if (inA == inB)
                    continue;
                const std::vector<uint64_t> &src = inA ? okLowR : okHighR;
                uint64_t o = 0;
                for (int w = 0; w < nWords; ++w) { acc[w] &= src[size_t(i) * nWords + w]; o |= acc[w]; }
                any = (o != 0);
            }
            if (!any) {
                Clearance c;
                c.sinc = -1.0;
                clrCache.emplace(key, c);
                return c;
            }
        }
        std::vector<const V3 *> pts;
        std::vector<int> cnt;
        std::vector<double> sg;
        for (int i = 0; i < nIf; ++i) {
            if (!((active >> ifA[i]) & 1u) || !((active >> ifB[i]) & 1u))
                continue;
            const bool inA = (mask >> ifA[i]) & 1u;
            const bool inB = (mask >> ifB[i]) & 1u;
            if (inA == inB)
                continue;
            pts.push_back(ifNorms[i].data());
            cnt.push_back(int(ifNorms[i].size()));
            sg.push_back(inA ? 1.0 : -1.0);     // the high side sees the normal reversed
        }
        Clearance c;
        if (pts.empty()) {
            c.sinc = 1.0;                       // nothing touches it
        } else {
            c = clearanceOf(pts, cnt, sg);
        }
        clrCache.emplace(key, c);
        return c;
    };
    const auto movesIn = [&](uint32_t mask, uint32_t active) { return clearIn(mask, active).sinc > kBlocked; };
    const auto groupFrom = [&](uint32_t mask, uint32_t active) {
        DbgReport::Group g;
        for (int p = 0; p < rep.pieces && p < 32; ++p)
            if ((mask >> p) & 1u) g.pieces.append(p);
        const Clearance c = clearIn(mask, active);
        g.clearanceDeg = rep.exactClearance ? clearanceDeg(c.sinc) : -2.0;   // -2 = not computed
        g.dir[0] = c.dir.x; g.dir[1] = c.dir.y; g.dir[2] = c.dir.z;
        g.dirs = mobileIn(mask, active);        // how many sampled directions agree
        return g;
    };

    const auto membersOf = [&](uint32_t mask) {
        QVector<int> v;
        for (int p = 0; p < rep.pieces && p < 32; ++p)
            if ((mask >> p) & 1u) v.append(p);
        return v;
    };

    rep.exactClearance = rep.pieces <= opt.exactUpTo;
    rep.pieceFreeDirs.resize(rep.pieces);
    rep.pieceClearanceDeg.resize(rep.pieces);
    for (int p = 0; p < rep.pieces; ++p) {
        rep.pieceFreeDirs[p] = mobileDirs(uint32_t(1) << p);
        const Clearance c = clearIn(uint32_t(1) << p, allMask);
        const bool moves = c.sinc > kBlocked;
        rep.pieceClearanceDeg[p] = moves ? (rep.exactClearance ? clearanceDeg(c.sinc) : -2.0) : -1.0;
        if (moves) { ++rep.mobilePieces; rep.keyPiece = p; }
    }
    rep.singleKey = (rep.mobilePieces == 1);
    if (!rep.singleKey)
        rep.keyPiece = -1;

    // --- the DBG question: can ANY proper subset come out? ------------------
    const bool full = rep.pieces <= opt.fullSubsetsUpTo && rep.pieces <= 24;
    rep.subsetSearchFull = full;
    rep.subsetSizeTested = full ? rep.pieces - 1 : std::max(1, opt.maxSubset);

    if (full) {
        const uint32_t all = (rep.pieces >= 32) ? 0xffffffffu
                                                : ((uint32_t(1) << rep.pieces) - 1);
        for (uint32_t mask = 1; mask < all; ++mask) {
            if (!movesIn(mask, allMask))
                continue;
            ++rep.mobileSubsets;
            const int sz = popcount64(mask);
            if (rep.smallestMobile == 0 || sz < rep.smallestMobile) {
                rep.smallestMobile = sz;
                rep.smallestMobileSet.clear();
                for (int p = 0; p < rep.pieces; ++p)
                    if ((mask >> p) & 1u) rep.smallestMobileSet.append(p);
            }
        }
    }
    else {
        // Too many pieces to enumerate everything: test small groups only, and
        // say so, because absence of a mobile subset is then not a proof.
        std::vector<int> idx(rep.subsetSizeTested);
        std::function<void(int, int)> rec = [&](int start, int depth) {
            if (depth > 0) {
                uint32_t mask = 0;
                for (int t = 0; t < depth; ++t) mask |= uint32_t(1) << idx[t];
                if (movesIn(mask, allMask)) {
                    ++rep.mobileSubsets;
                    if (rep.smallestMobile == 0 || depth < rep.smallestMobile) {
                        rep.smallestMobile = depth;
                        rep.smallestMobileSet.clear();
                        for (int t = 0; t < depth; ++t)
                            rep.smallestMobileSet.append(idx[t]);
                    }
                }
            }
            if (depth == rep.subsetSizeTested)
                return;
            for (int p = start; p < rep.pieces; ++p) {
                idx[depth] = p;
                rec(p + 1, depth + 1);
            }
        };
        rec(0, 0);
    }

    // The escape direction for the winning group: the direction with the most
    // clearance, i.e. the centre of its free cone.
    if (rep.smallestMobile > 0 && !rep.smallestMobileSet.isEmpty()) {
        uint32_t mask = 0;
        for (int p : rep.smallestMobileSet)
            mask |= uint32_t(1) << p;
        const DbgReport::Group g = groupFrom(mask, allMask);
        rep.escapeDir[0] = g.dir[0]; rep.escapeDir[1] = g.dir[1]; rep.escapeDir[2] = g.dir[2];
        rep.escapeCount = g.dirs;
        rep.escapeClearanceDeg = g.clearanceDeg;
    }

    rep.interlocked = (rep.mobileSubsets == 0);

    if (full) {
        // --- every opening of the assembled puzzle -------------------------
        for (uint32_t mask = 1; mask < allMask; ++mask) {
            const uint32_t rest = allMask & ~mask;
            const int a = popcount64(mask), b = popcount64(rest);
            if (a > b || (a == b && mask > rest))
                continue;                       // listed by its other side
            // Either side may be the one that moves; they are the same split.
            if (!movesIn(mask, allMask) && !movesIn(rest, allMask)) continue;
            DbgReport::Group g = groupFrom(mask, allMask);
            if (!movesIn(mask, allMask)) {
                const DbgReport::Group o = groupFrom(rest, allMask);
                g.clearanceDeg = o.clearanceDeg;
                g.dirs = o.dirs;
                for (int k = 0; k < 3; ++k) g.dir[k] = -o.dir[k];
            }
            rep.openings.append(g);
        }
        std::sort(rep.openings.begin(), rep.openings.end(),
                  [](const DbgReport::Group &x, const DbgReport::Group &y) {
                      if (x.pieces.size() != y.pieces.size())
                          return x.pieces.size() < y.pieces.size();
                      return x.clearanceDeg > y.clearanceDeg;
                  });
        if (rep.openings.size() > 64)
            rep.openings.resize(64);

        // --- single key, then level k (Song 2012 recursive interlocking) ---
        // In a state with pieces `active`, the moving groups come in pairs (a
        // group and the rest). A unique key means exactly one such pair, with
        // a single piece on one side.
        uint32_t active = allMask;
        while (popcount64(active) > 2) {
            int pairs = 0;
            uint32_t key = 0;
            for (uint32_t s = (active - 1) & active; s; s = (s - 1) & active) {
                const uint32_t o = active & ~s;
                if (s > o) continue;            // each split once
                if (!movesIn(s, active) && !movesIn(o, active)) continue;
                ++pairs;
                if (popcount64(s) == 1) key = s;
                if (popcount64(o) == 1) key = o;
            }
            if (pairs == 1 && key) {
                if (rep.keyLevel == 0) rep.strictSingleKey = true;
                ++rep.keyLevel;
                rep.keySequence.append(membersOf(key).value(0));
                active &= ~key;
                continue;
            }
            const int left = popcount64(active);
            rep.levelStop = (pairs == 0)
                ? QStringLiteral("%1 pieces left and none of them can move - locked for good").arg(left)
                : (key == 0
                    ? QStringLiteral("%1 pieces left: no single piece can move, only groups").arg(left)
                    : QStringLiteral("%1 pieces left: %2 different ways to open, not one key")
                          .arg(left).arg(pairs));
            break;
        }
        if (popcount64(active) <= 2 && rep.levelStop.isEmpty()) {
            rep.fullyRecursive = true;
            rep.levelStop = QStringLiteral("every state had a unique key, down to the last two pieces");
        }

        // --- one full disassembly, smallest group first --------------------
        active = allMask;
        while (popcount64(active) > 1) {
            uint32_t best = 0;
            double bestClr = -1.0;
            for (uint32_t s = (active - 1) & active; s; s = (s - 1) & active) {
                const Clearance c = clearIn(s, active);
                if (!(c.sinc > kBlocked)) continue;
                const int ps = popcount64(s), pb = popcount64(best);
                if (!best || ps < pb || (ps == pb && c.sinc > bestClr)) { best = s; bestClr = c.sinc; }
            }
            if (!best) break;                   // what is left is locked
            rep.disassembly.append(groupFrom(best, active));
            active &= ~best;
        }
        rep.disassemblyComplete = (popcount64(active) <= 1);
    }

    QVector<int> fd;
    fd.reserve(rep.pairs.size());
    for (const DbgPair &p : rep.pairs)
        fd.append(p.freeDirs);
    std::sort(fd.begin(), fd.end());
    if (!fd.isEmpty())
        rep.medianFreeDirs = fd[fd.size() / 2];

}

}  // namespace

DbgReport DbgAnalysis::run(const Trivariate &tv, const DbgOptions &opt)
{
    DbgReport rep;
    if (!tv.isValid()) {
        rep.error = QStringLiteral("trivariate is not valid");
        return rep;
    }

    double dom[6];
    tv.domain(dom);
    for (int a = 0; a < 3; ++a)
        if (!(dom[a * 2 + 1] > dom[a * 2])) {
            rep.error = QStringLiteral("degenerate domain on axis %1").arg(a);
            return rep;
        }

    const int n[3] = { std::max(1, opt.cells[0]),
                       std::max(1, opt.cells[1]),
                       std::max(1, opt.cells[2]) };
    rep.pieces = n[0] * n[1] * n[2];

    double h[3];
    for (int a = 0; a < 3; ++a)
        h[a] = (dom[a * 2 + 1] - dom[a * 2]) * 1e-5;

    const auto cut = [&](int axis, int i) {
        const double lo = dom[axis * 2], hi = dom[axis * 2 + 1];
        return lo + (hi - lo) * double(i) / double(n[axis]);
    };
    const auto pieceAt = [&](int i, int j, int k) {
        return (i * n[1] + j) * n[2] + k;
    };

    measureJacobian(tv, opt, rep);

    // --- interfaces ---------------------------------------------------------
    //
    // Only interior cuts are shared between two pieces; the outer boundary of
    // the domain maps to the model's own surface.
    const int fs = std::max(2, opt.faceSamples);
    std::vector<RawInterface> interfaces;

    for (int axis = 0; axis < 3; ++axis) {
        const int u = (axis + 1) % 3, v = (axis + 2) % 3;

        for (int c = 1; c < n[axis]; ++c) {
            const double plane = cut(axis, c);

            for (int iu = 0; iu < n[u]; ++iu)
                for (int iv = 0; iv < n[v]; ++iv) {
                    int loIdx[3], hiIdx[3];
                    loIdx[axis] = c - 1;  hiIdx[axis] = c;
                    loIdx[u] = hiIdx[u] = iu;
                    loIdx[v] = hiIdx[v] = iv;

                    DbgPair pr;
                    pr.axis = axis;
                    pr.a = pieceAt(loIdx[0], loIdx[1], loIdx[2]);
                    pr.b = pieceAt(hiIdx[0], hiIdx[1], hiIdx[2]);

                    const double u0 = cut(u, iu), u1 = cut(u, iu + 1);
                    const double v0 = cut(v, iv), v1 = cut(v, iv + 1);

                    // Raw cross products first: their length is the local area
                    // element, needed to tell a real normal from noise.
                    std::vector<V3> raw;
                    raw.reserve(fs * fs);
                    for (int su = 0; su < fs; ++su)
                        for (int sv = 0; sv < fs; ++sv) {
                            double p[3];
                            p[axis] = plane;
                            p[u] = u0 + (u1 - u0) * su / double(fs - 1);
                            p[v] = v0 + (v1 - v0) * sv / double(fs - 1);

                            // n parallel to M_u x M_v for the two in-plane
                            // axes. With det J > 0 that points along increasing
                            // `axis`, i.e. outward from the low-side piece, and
                            // equals J^-T applied to the domain normal without
                            // inverting J.
                            raw.push_back(cross(tangent(tv, p, u, dom, h[u]),
                                                tangent(tv, p, v, dom, h[v])));
                        }

                    // Where the map degenerates - the poles of a surface of
                    // revolution, where one tangent vanishes - the cross product
                    // is round-off, and normalising it yields a direction that is
                    // pure noise. Those samples cover no area, so they carry no
                    // contact; drop them rather than let them block directions.
                    double maxArea = 0.0;
                    for (const V3 &c : raw) maxArea = std::max(maxArea, len(c));
                    std::vector<V3> norms;
                    norms.reserve(raw.size());
                    for (const V3 &c : raw)
                        if (len(c) > 1e-4 * maxArea)
                            norms.push_back(unit(c));

                    if (norms.empty())
                        continue;
                    RawInterface ri;
                    ri.pair = pr;
                    ri.norms = std::move(norms);
                    interfaces.push_back(std::move(ri));
                }
        }
    }

    finishBlocking(rep, opt, interfaces);

    // Each piece's extent in R^3, sampled over its own cell of the domain.
    // This is what lets the analysis be checked against the deformed tiles the
    // IRIT script writes: the analyser never reads those files, so matching
    // boxes are the evidence that it is dividing the same puzzle.
    rep.pieceBox.resize(rep.pieces * 6);
    for (int i = 0; i < n[0]; ++i)
        for (int j = 0; j < n[1]; ++j)
            for (int k = 0; k < n[2]; ++k) {
                const int id = pieceAt(i, j, k);
                double lo[3] = {  1e300,  1e300,  1e300 };
                double hi[3] = { -1e300, -1e300, -1e300 };
                const int S = 7;
                for (int a = 0; a <= S; ++a)
                    for (int b = 0; b <= S; ++b)
                        for (int c = 0; c <= S; ++c) {
                            double p[3];
                            p[0] = cut(0, i) + (cut(0, i + 1) - cut(0, i)) * a / S;
                            p[1] = cut(1, j) + (cut(1, j + 1) - cut(1, j)) * b / S;
                            p[2] = cut(2, k) + (cut(2, k + 1) - cut(2, k)) * c / S;
                            const V3 q = evalAt(tv, p);
                            const double e[3] = { q.x, q.y, q.z };
                            for (int d = 0; d < 3; ++d) {
                                lo[d] = std::min(lo[d], e[d]);
                                hi[d] = std::max(hi[d], e[d]);
                            }
                        }
                for (int d = 0; d < 3; ++d) {
                    rep.pieceBox[id * 6 + d * 2]     = lo[d];
                    rep.pieceBox[id * 6 + d * 2 + 1] = hi[d];
                }
            }

    rep.valid = true;
    return rep;
}

QString DbgAnalysis::tableHeader()
{
    return QStringLiteral(
        "| case | det J interior | pairs | blocked | mobile pieces | mobile subsets | smallest | single key |\n"
        "|---|---|---|---|---|---|---|---|");
}

QString DbgAnalysis::tableRow(const QString &label, const DbgReport &r)
{
    if (!r.valid)
        return QStringLiteral("| %1 | - | - | - | - | - | - | error: %2 |")
                   .arg(label, r.error);

    return QStringLiteral("| %1 | %2 | %3 | %4 | %5 | %6 | %7 | %8 |")
               .arg(label)
               .arg(r.minDetInterior, 0, 'g', 4)
               .arg(r.pairs.size())
               .arg(r.blockedPairs)
               .arg(r.mobilePieces)
               .arg(r.mobileSubsets)
               .arg(r.smallestMobile == 0 ? QStringLiteral("none")
                                          : QString::number(r.smallestMobile))
               .arg(r.singleKey ? QStringLiteral("**yes** (piece %1)").arg(r.keyPiece)
                                : QStringLiteral("no"));
}

QString DbgAnalysis::formatReport(const QString &label, const DbgReport &r,
                                  const DbgOptions &opt)
{
    QStringList out;
    out << QStringLiteral("### %1").arg(label);
    if (!r.valid) {
        out << QStringLiteral("**failed:** %1").arg(r.error);
        return out.join(QStringLiteral("\n"));
    }

    out << QString();
    out << QStringLiteral("- pieces: %1, interfaces: %2").arg(r.pieces).arg(r.pairs.size());
    out << QStringLiteral("- min det J over the domain: **%1** at (u,v,w)=(%2, %3, %4) - %5")
               .arg(r.minDetJ, 0, 'g', 4)
               .arg(r.minDetAt[0], 0, 'f', 3).arg(r.minDetAt[1], 0, 'f', 3)
               .arg(r.minDetAt[2], 0, 'f', 3)
               .arg(r.jacobianOk ? QStringLiteral("positive")
                                 : QStringLiteral("**NOT POSITIVE**"));
    out << QStringLiteral("- min det J strictly inside the domain: **%1**")
               .arg(r.minDetInterior, 0, 'g', 4);
    out << QStringLiteral("- directions tested: %1, samples per interface: %2")
               .arg(opt.directions).arg(opt.faceSamples * opt.faceSamples);
    out << QStringLiteral("- blocked pairs: %1 of %2").arg(r.blockedPairs).arg(r.pairs.size());
    out << QStringLiteral("- mobile pieces: %1 of %2%3")
               .arg(r.mobilePieces).arg(r.pieces)
               .arg(r.singleKey ? QStringLiteral("  **<- single key: piece %1**").arg(r.keyPiece)
                                : QString());

    out << QString();
    out << QStringLiteral("**DBG disassembly**");
    out << QStringLiteral("- proper subsets that can translate out: **%1**").arg(r.mobileSubsets);
    if (r.smallestMobile > 0) {
        QStringList ids;
        for (int p : r.smallestMobileSet) ids << QString::number(p);
        out << QStringLiteral("- smallest mobile group: size %1 - pieces {%2}")
                   .arg(r.smallestMobile).arg(ids.join(QStringLiteral(", ")));
    }
    if (r.smallestMobile > 0) {
        QStringList ids;
        for (int p : r.smallestMobileSet) ids << QString::number(p);
        out << QStringLiteral("- escape direction for that group: **(%1, %2, %3)**, "
                              "clearance %4 deg (%5 of the sampled directions; -2 = not computed)")
                   .arg(r.escapeDir[0], 0, 'f', 4)
                   .arg(r.escapeDir[1], 0, 'f', 4)
                   .arg(r.escapeDir[2], 0, 'f', 4)
                   .arg(r.escapeClearanceDeg, 0, 'f', 2)
                   .arg(r.escapeCount);
        out << QStringLiteral("- SHOWLINE %1 | %2 %3 %4")
                   .arg(ids.join(QStringLiteral(",")))
                   .arg(r.escapeDir[0], 0, 'f', 6)
                   .arg(r.escapeDir[1], 0, 'f', 6)
                   .arg(r.escapeDir[2], 0, 'f', 6);
    }

    out << QStringLiteral("- **%1**")
               .arg(r.interlocked
                        ? (r.subsetSearchFull
                               ? QStringLiteral("INTERLOCKED - no proper subset of any size can be "
                                                "translated out (all subsets tested)")
                               : QStringLiteral("no mobile group found, but only subsets up to size "
                                                "%1 were tested - this is NOT a proof")
                                     .arg(r.subsetSizeTested))
                        : QStringLiteral("not interlocked - a group can be removed"));

    out << QString();
    out << QStringLiteral("| pair | axis | normals | spread (deg) | free dirs | blocked | clearance (deg) |");
    out << QStringLiteral("|---|---|---|---|---|---|---|");
    for (const DbgPair &p : r.pairs)
        out << QStringLiteral("| %1-%2 | %3 | %4 | %5 | %6 | %7 | %8 |")
                   .arg(p.a).arg(p.b)
                   .arg(QStringLiteral("uvw").mid(p.axis, 1))
                   .arg(p.normals)
                   .arg(p.spreadDeg, 0, 'f', 1)
                   .arg(p.freeDirs)
                   .arg(p.blocked ? QStringLiteral("**yes**") : QStringLiteral("no"))
                   .arg(p.clearanceDeg, 0, 'f', 3);

    out << QString();
    QStringList per;
    for (int i = 0; i < r.pieceFreeDirs.size(); ++i)
        per << QStringLiteral("p%1=%2").arg(i).arg(r.pieceFreeDirs[i]);
    out << QStringLiteral("Directions that free each piece: `%1`")
               .arg(per.join(QStringLiteral("  ")));
    QStringList clr;
    for (int i = 0; i < r.pieceClearanceDeg.size(); ++i)
        clr << (r.pieceClearanceDeg[i] < -1.5 ? QStringLiteral("p%1=free").arg(i)
                : r.pieceClearanceDeg[i] < 0.0
                    ? QStringLiteral("p%1=blocked").arg(i)
                    : QStringLiteral("p%1=%2").arg(i).arg(r.pieceClearanceDeg[i], 0, 'f', 3));
    out << QStringLiteral("Clearance of each piece alone, degrees: `%1`").arg(clr.join(QStringLiteral("  ")));
    if (!r.exactClearance)
        out << QStringLiteral("NOTE: more than %1 pieces - groups were decided by the sampled directions, "
                              "not the exact clearance; a free set thinner than the sampling can be missed.")
                   .arg(opt.exactUpTo);

    if (r.pieceBox.size() == r.pieces * 6) {
        out << QString();
        out << QStringLiteral("Piece extents in R3 - compare against the deformed "
                              "tiles the script saves:");
        out << QStringLiteral("| piece | x | y | z |");
        out << QStringLiteral("|---|---|---|---|");
        for (int i = 0; i < r.pieces; ++i)
            out << QStringLiteral("| %1 | [%2, %3] | [%4, %5] | [%6, %7] |")
                       .arg(i)
                       .arg(r.pieceBox[i * 6 + 0], 0, 'f', 2).arg(r.pieceBox[i * 6 + 1], 0, 'f', 2)
                       .arg(r.pieceBox[i * 6 + 2], 0, 'f', 2).arg(r.pieceBox[i * 6 + 3], 0, 'f', 2)
                       .arg(r.pieceBox[i * 6 + 4], 0, 'f', 2).arg(r.pieceBox[i * 6 + 5], 0, 'f', 2);
    }

    return out.join(QStringLiteral("\n"));
}

// ---------------------------------------------------------------------------
// Sweep check: move the group for real and look for interpenetration.
// ---------------------------------------------------------------------------
namespace {

// Solve [a b c] x = r by Cramer's rule.
bool solve3(const V3 &a, const V3 &b, const V3 &c, const V3 &r, double x[3])
{
    const double det = dot(a, cross(b, c));
    if (!(std::fabs(det) > 1e-300))
        return false;
    x[0] = dot(r, cross(b, c)) / det;
    x[1] = dot(a, cross(r, c)) / det;
    x[2] = dot(a, cross(b, r)) / det;
    return true;
}

// Find p with M(p) = y by Newton, starting from p (usually the previous
// solution along a path) and clamped to the domain. Returns the residual: a
// large one means y has no preimage in the domain - it lies outside the model.
double invertNear(const Trivariate &tv, const double dom[6], const double h[3],
                  const V3 &y, double p[3], double tolOk)
{
    double res = 1e300;
    for (int it = 0; it < 30; ++it) {
        const V3 m = evalAt(tv, p);
        const V3 f { m.x - y.x, m.y - y.y, m.z - y.z };
        res = len(f);
        if (res < tolOk) break;
        double dp[3];
        if (!solve3(tangent(tv, p, 0, dom, h[0]),
                    tangent(tv, p, 1, dom, h[1]),
                    tangent(tv, p, 2, dom, h[2]),
                    V3 { -f.x, -f.y, -f.z }, dp))
            break;
        for (int k = 0; k < 3; ++k) {
            p[k] += dp[k];
            p[k] = std::max(dom[k * 2], std::min(dom[k * 2 + 1], p[k]));
        }
    }
    return res;
}

// The size of the mapped domain, from its eight corners - the scale every
// tolerance is taken against.
double modelScale(const Trivariate &tv, const double dom[6])
{
    double lo3[3] = { 1e300, 1e300, 1e300 }, hi3[3] = { -1e300, -1e300, -1e300 };
    for (int c = 0; c < 8; ++c) {
        double p[3] = { dom[(c & 1) ? 1 : 0], dom[(c & 2) ? 3 : 2], dom[(c & 4) ? 5 : 4] };
        const V3 q = evalAt(tv, p);
        const double e[3] = { q.x, q.y, q.z };
        for (int k = 0; k < 3; ++k) { lo3[k] = std::min(lo3[k], e[k]); hi3[k] = std::max(hi3[k], e[k]); }
    }
    double scale = 0.0;
    for (int k = 0; k < 3; ++k) scale = std::max(scale, hi3[k] - lo3[k]);
    return (scale > 0.0) ? scale : 1.0;
}

}  // namespace

namespace {

// Slide the given start points (parameter space) along d and report where they
// land. `classify` says which piece a parameter point is in, and whether it is
// clearly inside it rather than on a cut. No normals are involved.
DbgSweep slidePoints(const Trivariate &tv, const std::vector<std::array<double, 3>> &starts,
                     const std::vector<char> &moving,
                     const std::function<int(const double *, bool *)> &classify,
                     const double dirIn[3], double travel, int steps)
{
    DbgSweep s;
    double dom[6];
    tv.domain(dom);
    double h[3];
    for (int a = 0; a < 3; ++a)
        h[a] = (dom[a * 2 + 1] - dom[a * 2]) * 1e-5;
    const V3 d = unit(V3 { dirIn[0], dirIn[1], dirIn[2] });
    const double tolOk = 1e-6 * modelScale(tv, dom);   // residual that counts as solved

    for (const auto &st0 : starts) {
        double p[3] = { st0[0], st0[1], st0[2] };
        const V3 x0 = evalAt(tv, p);
        ++s.samples;

        for (int st = 1; st <= steps; ++st) {
            const double t = travel * double(st) / double(steps);
            const V3 y { x0.x + t * d.x, x0.y + t * d.y, x0.z + t * d.z };

            // Newton, continued from the previous step's solution.
            const double res = invertNear(tv, dom, h, y, p, tolOk);
            if (res >= tolOk * 100.0) {
                // No preimage inside the domain: the point has left the model
                // (or Newton failed). Stop tracking.
                ++s.lost;
                break;
            }
            bool clear = false;
            const int cell = classify(p, &clear);
            if (cell >= 0 && cell < int(moving.size()) && !moving[cell] && clear) {
                ++s.collided;
                if (s.firstHit < 0.0 || t < s.firstHit) s.firstHit = t;
                break;
            }
        }
    }
    return s;
}

}  // namespace

DbgSweep DbgAnalysis::sweep(const Trivariate &tv, const DbgOptions &opt,
                            const QVector<int> &group, const double dirIn[3],
                            double travel, int steps, int perAxis)
{
    if (!tv.isValid() || group.isEmpty())
        return DbgSweep();

    double dom[6];
    tv.domain(dom);
    const int n[3] = { std::max(1, opt.cells[0]),
                       std::max(1, opt.cells[1]),
                       std::max(1, opt.cells[2]) };
    const int pieces = n[0] * n[1] * n[2];

    std::vector<char> moving(pieces, 0);
    for (int g : group)
        if (g >= 0 && g < pieces) moving[g] = 1;

    // Which cell a parameter point lies in, and whether it is clearly inside it
    // rather than sitting on a cut (where the answer is ambiguous).
    const double margin = 1e-3;
    const auto cellOf = [&](const double *p, bool *clearlyInside) {
        int idx[3];
        *clearlyInside = true;
        for (int a = 0; a < 3; ++a) {
            const double lo = dom[a * 2], hi = dom[a * 2 + 1];
            const double t = (p[a] - lo) / (hi - lo);
            if (t < 0.0 || t > 1.0) return -1;
            const double f = t * n[a];
            int i = int(std::floor(f));
            if (i >= n[a]) i = n[a] - 1;
            if (i < 0) i = 0;
            const double fr = f - i;
            if (fr < margin * n[a] || fr > 1.0 - margin * n[a]) *clearlyInside = false;
            idx[a] = i;
        }
        return (idx[0] * n[1] + idx[1]) * n[2] + idx[2];
    };

    // Samples reach close to the cell's faces - that is where a blocked move
    // shows up first.
    std::vector<std::array<double, 3>> starts;
    const int per = std::max(2, perAxis);
    for (int g : group) {
        if (g < 0 || g >= pieces) continue;
        const int ci[3] = { g / (n[1] * n[2]), (g / n[2]) % n[1], g % n[2] };
        for (int a = 0; a < per; ++a)
            for (int b = 0; b < per; ++b)
                for (int c = 0; c < per; ++c) {
                    const int ab[3] = { a, b, c };
                    std::array<double, 3> p;
                    for (int k = 0; k < 3; ++k) {
                        const double lo = dom[k * 2], hi = dom[k * 2 + 1];
                        const double c0 = lo + (hi - lo) * double(ci[k]) / n[k];
                        const double c1 = lo + (hi - lo) * double(ci[k] + 1) / n[k];
                        const double f = 0.01 + 0.98 * double(ab[k]) / double(per - 1);
                        p[k] = c0 + (c1 - c0) * f;
                    }
                    starts.push_back(p);
                }
    }
    return slidePoints(tv, starts, moving, cellOf, dirIn, travel, steps);
}

// ---------------------------------------------------------------------------
// Shape criteria: volume, folds, minimum thickness.
// ---------------------------------------------------------------------------
DbgQuality DbgAnalysis::quality(const Trivariate &tv, const DbgOptions &opt,
                                int raysPerFaceAxis)
{
    DbgQuality q;
    if (!tv.isValid())
        return q;

    double dom[6];
    tv.domain(dom);
    const int n[3] = { std::max(1, opt.cells[0]),
                       std::max(1, opt.cells[1]),
                       std::max(1, opt.cells[2]) };
    const int pieces = n[0] * n[1] * n[2];

    double h[3];
    for (int a = 0; a < 3; ++a)
        h[a] = (dom[a * 2 + 1] - dom[a * 2]) * 1e-5;

    q.modelSize = modelScale(tv, dom);
    const double tolOk = 1e-6 * q.modelSize;
    q.pieces.resize(pieces);

    const auto cut = [&](int axis, int i) {
        const double lo = dom[axis * 2], hi = dom[axis * 2 + 1];
        return lo + (hi - lo) * double(i) / double(n[axis]);
    };
    const auto detAt = [&](const double p[3]) {
        return dot(tangent(tv, p, 0, dom, h[0]),
                   cross(tangent(tv, p, 1, dom, h[1]), tangent(tv, p, 2, dom, h[2])));
    };

    const int R = std::max(2, raysPerFaceAxis);

    // A face can collapse - to the centre of a ball, the axis of a revolution,
    // the tip of the duck - and there the normal is round-off or nearly so, and
    // a ray shot along it grazes the neighbouring cut and exits at once.
    // Comparing faces only with themselves lets that through, so points are
    // also held against the area element a face of ordinary size would have
    // (measured: the duck's collapsed tip face sits at 2e-4 of it, healthy
    // faces at 5e-2 and up).
    double refArea[3];
    for (int a = 0; a < 3; ++a) {
        const int u = (a + 1) % 3, v = (a + 2) % 3;
        refArea[a] = q.modelSize * q.modelSize /
                     ((dom[u * 2 + 1] - dom[u * 2]) * (dom[v * 2 + 1] - dom[v * 2]));
    }

    for (int i = 0; i < n[0]; ++i)
        for (int j = 0; j < n[1]; ++j)
            for (int k = 0; k < n[2]; ++k) {
                const int id = (i * n[1] + j) * n[2] + k;
                DbgPieceQuality &pq = q.pieces[id];
                const int ci[3] = { i, j, k };
                double c0[3], c1[3];
                for (int a = 0; a < 3; ++a) { c0[a] = cut(a, ci[a]); c1[a] = cut(a, ci[a] + 1); }

                // Volume by the midpoint rule on det J; the fold check on a
                // grid that includes the cell's faces, where folds start.
                const int G = 6;
                double vol = 0.0;
                const double cellVol = (c1[0] - c0[0]) * (c1[1] - c0[1]) * (c1[2] - c0[2]);
                for (int a = 0; a < G; ++a)
                    for (int b = 0; b < G; ++b)
                        for (int c = 0; c < G; ++c) {
                            const double p[3] = { c0[0] + (c1[0] - c0[0]) * (a + 0.5) / G,
                                                  c0[1] + (c1[1] - c0[1]) * (b + 0.5) / G,
                                                  c0[2] + (c1[2] - c0[2]) * (c + 0.5) / G };
                            vol += detAt(p);
                        }
                pq.volume = std::fabs(vol) * cellVol / double(G * G * G);

                bool first = true;
                for (int a = 0; a <= G; ++a)
                    for (int b = 0; b <= G; ++b)
                        for (int c = 0; c <= G; ++c) {
                            const double p[3] = { c0[0] + (c1[0] - c0[0]) * a / G,
                                                  c0[1] + (c1[1] - c0[1]) * b / G,
                                                  c0[2] + (c1[2] - c0[2]) * c / G };
                            const double d = detAt(p);
                            if (first || d < pq.minDetJ) { pq.minDetJ = d; first = false; }
                        }

                // The piece's diagonal sets the ray step.
                double lo[3] = { 1e300, 1e300, 1e300 }, hi[3] = { -1e300, -1e300, -1e300 };
                for (int a = 0; a <= 2; ++a)
                    for (int b = 0; b <= 2; ++b)
                        for (int c = 0; c <= 2; ++c) {
                            const double p[3] = { c0[0] + (c1[0] - c0[0]) * a / 2,
                                                  c0[1] + (c1[1] - c0[1]) * b / 2,
                                                  c0[2] + (c1[2] - c0[2]) * c / 2 };
                            const V3 x = evalAt(tv, p);
                            const double e[3] = { x.x, x.y, x.z };
                            for (int d = 0; d < 3; ++d) {
                                lo[d] = std::min(lo[d], e[d]);
                                hi[d] = std::max(hi[d], e[d]);
                            }
                        }
                const double diag = std::sqrt((hi[0] - lo[0]) * (hi[0] - lo[0]) +
                                              (hi[1] - lo[1]) * (hi[1] - lo[1]) +
                                              (hi[2] - lo[2]) * (hi[2] - lo[2]));
                const int    steps = 30;
                const double step  = std::max(diag, 1e-9 * q.modelSize) * 1.5 / steps;

                const double eps[3] = { (c1[0] - c0[0]) * 1e-6, (c1[1] - c0[1]) * 1e-6,
                                        (c1[2] - c0[2]) * 1e-6 };
                const auto inside = [&](const double p[3]) {
                    for (int a = 0; a < 3; ++a)
                        if (p[a] < c0[a] - eps[a] || p[a] > c1[a] + eps[a]) return false;
                    return true;
                };

                // Rays from all six faces, started at least 10% of the face
                // away from its edges. Where two faces meet at a sharp edge the
                // material always tapers to nothing - the rim of a hemisphere -
                // and rays started right there would report the edge, not a wall.
                for (int axis = 0; axis < 3; ++axis) {
                    const int u = (axis + 1) % 3, v = (axis + 2) % 3;
                    for (int side = 0; side < 2; ++side) {
                        struct Start { double p[3]; V3 raw; };
                        std::vector<Start> starts;
                        double maxArea = 0.0;
                        for (int a = 0; a < R; ++a)
                            for (int b = 0; b < R; ++b) {
                                Start st;
                                st.p[axis] = side ? c1[axis] : c0[axis];
                                st.p[u] = c0[u] + (c1[u] - c0[u]) * (0.1 + 0.8 * a / double(R - 1));
                                st.p[v] = c0[v] + (c1[v] - c0[v]) * (0.1 + 0.8 * b / double(R - 1));
                                st.raw = cross(tangent(tv, st.p, u, dom, h[u]),
                                               tangent(tv, st.p, v, dom, h[v]));
                                maxArea = std::max(maxArea, len(st.raw));
                                starts.push_back(st);
                            }

                        for (const Start &st : starts) {
                            if (!(len(st.raw) > 1e-4 * maxArea) ||
                                !(len(st.raw) > 1e-3 * refArea[axis]))
                                continue;               // degenerate: no normal here
                            // M_u x M_v points toward increasing `axis` (det J > 0),
                            // so it is inward on the low face, outward on the high.
                            V3 in = unit(st.raw);
                            if (side) in = V3 { -in.x, -in.y, -in.z };

                            const V3 x0 = evalAt(tv, st.p);
                            double p[3]   = { st.p[0], st.p[1], st.p[2] };
                            double pIn[3] = { p[0], p[1], p[2] };
                            double tIn = 0.0, tOut = -1.0;
                            for (int s = 1; s <= steps; ++s) {
                                const double t = step * s;
                                const V3 y { x0.x + t * in.x, x0.y + t * in.y, x0.z + t * in.z };
                                const double res = invertNear(tv, dom, h, y, p, tolOk);
                                if (res >= tolOk * 100.0 || !inside(p)) { tOut = t; break; }
                                tIn = t;
                                pIn[0] = p[0]; pIn[1] = p[1]; pIn[2] = p[2];
                            }
                            if (tOut < 0.0)
                                continue;               // never left: not a usable ray

                            // Refine the exit between the last inside step and
                            // the first outside one.
                            for (int it = 0; it < 12; ++it) {
                                const double t = 0.5 * (tIn + tOut);
                                const V3 y { x0.x + t * in.x, x0.y + t * in.y, x0.z + t * in.z };
                                double pt[3] = { pIn[0], pIn[1], pIn[2] };
                                const double res = invertNear(tv, dom, h, y, pt, tolOk);
                                if (res < tolOk * 100.0 && inside(pt)) {
                                    tIn = t;
                                    pIn[0] = pt[0]; pIn[1] = pt[1]; pIn[2] = pt[2];
                                } else {
                                    tOut = t;
                                }
                            }
                            const double thick = 0.5 * (tIn + tOut);
                            ++pq.rays;
                            if (pq.thickness < 0.0 || thick < pq.thickness) {
                                pq.thickness = thick;
                                pq.thickAt[0] = x0.x; pq.thickAt[1] = x0.y; pq.thickAt[2] = x0.z;
                            }
                        }
                    }
                }
            }

    // Size uniformity.
    if (!q.pieces.isEmpty()) {
        double sum = 0.0;
        q.volMin = q.volMax = q.pieces[0].volume;
        for (const DbgPieceQuality &pq : q.pieces) {
            sum += pq.volume;
            q.volMin = std::min(q.volMin, pq.volume);
            q.volMax = std::max(q.volMax, pq.volume);
        }
        q.volMean = sum / q.pieces.size();
        double var = 0.0;
        for (const DbgPieceQuality &pq : q.pieces)
            var += (pq.volume - q.volMean) * (pq.volume - q.volMean);
        var /= q.pieces.size();
        q.volCv = (q.volMean > 0.0) ? std::sqrt(var) / q.volMean : 0.0;
    }
    return q;
}

// ---------------------------------------------------------------------------
// Curved BSP in D
// ---------------------------------------------------------------------------
DbgReport DbgAnalysis::runBsp(const Trivariate &tv, const CurvedBsp &bsp, const DbgOptions &opt)
{
    DbgReport rep;
    if (!tv.isValid() || !bsp.isValid()) {
        rep.error = QStringLiteral("no trivariate, or no division");
        return rep;
    }
    double dom[6];
    tv.domain(dom);
    double h[3];
    for (int a = 0; a < 3; ++a)
        h[a] = (dom[a * 2 + 1] - dom[a * 2]) * 1e-5;
    rep.pieces = bsp.pieceCount();

    measureJacobian(tv, opt, rep);

    // --- interfaces: sample every curved split -----------------------------
    //
    // A split is the surface p_a = at + A f(p_b, p_c) over its cell. Each sample
    // is nudged a hair to either side; the two pieces found there are the pair
    // this part of the split separates (one split can border many pairs). The
    // normal in R^3 is J^-T grad g, written with cofactors so J is never
    // inverted:  n' ~ g_u (M_v x M_w) + g_v (M_w x M_u) + g_w (M_u x M_v).
    std::map<std::pair<int, int>, std::vector<V3>> raw;
    std::map<std::pair<int, int>, int> axisOf;
    const int S = std::max(5, 3 * opt.faceSamples);
    for (int si = 0; si < bsp.splitCount(); ++si) {
        const int node = bsp.splitNode(si);
        const CurvedBsp::Split &sp = bsp.split(node);
        const int a = sp.axis, bb = (a + 1) % 3, cc = (a + 2) % 3;
        const double *bulge = bsp.faceBulge(node);
        // The cell may bulge past its box where earlier splits bent outward.
        const double b0 = std::max(dom[2 * bb], sp.lo[bb] - bulge[2 * bb]);
        const double b1 = std::min(dom[2 * bb + 1], sp.hi[bb] + bulge[2 * bb + 1]);
        const double c0 = std::max(dom[2 * cc], sp.lo[cc] - bulge[2 * cc]);
        const double c1 = std::min(dom[2 * cc + 1], sp.hi[cc] + bulge[2 * cc + 1]);
        const double eps = 1e-5 * (dom[2 * a + 1] - dom[2 * a]);

        for (int ib = 0; ib < S; ++ib)
            for (int ic = 0; ic < S; ++ic) {
                double p[3];
                p[bb] = b0 + (b1 - b0) * (ib + 0.5) / S;
                p[cc] = c0 + (c1 - c0) * (ic + 0.5) / S;
                p[a] = bsp.pointOnSplit(node, p[bb], p[cc]);
                if (p[a] <= dom[2 * a] || p[a] >= dom[2 * a + 1])
                    continue;
                double gr[3];
                bsp.gradG(node, p, gr);
                const double gl = std::sqrt(gr[0] * gr[0] + gr[1] * gr[1] + gr[2] * gr[2]);
                double lo[3], hi[3];
                for (int k = 0; k < 3; ++k) {
                    lo[k] = p[k] - eps * gr[k] / gl;
                    hi[k] = p[k] + eps * gr[k] / gl;
                }
                const int pl = bsp.classify(lo), ph = bsp.classify(hi);
                if (pl < 0 || ph < 0 || pl == ph)
                    continue;
                // Both sides must be this split's own children, or the sample
                // lies outside the cell the split belongs to.
                const int nl = bsp.child(node, 0), nh = bsp.child(node, 1);
                if (!bsp.isInSubtree(bsp.pieceNode(pl), nl) || !bsp.isInSubtree(bsp.pieceNode(ph), nh))
                    continue;

                const V3 Mu = tangent(tv, p, 0, dom, h[0]);
                const V3 Mv = tangent(tv, p, 1, dom, h[1]);
                const V3 Mw = tangent(tv, p, 2, dom, h[2]);
                const V3 cu = cross(Mv, Mw), cv = cross(Mw, Mu), cw = cross(Mu, Mv);
                const V3 nn { gr[0] * cu.x + gr[1] * cv.x + gr[2] * cw.x,
                              gr[0] * cu.y + gr[1] * cv.y + gr[2] * cw.y,
                              gr[0] * cu.z + gr[1] * cv.z + gr[2] * cw.z };
                raw[{ pl, ph }].push_back(nn);
                axisOf[{ pl, ph }] = a;
            }
    }

    std::vector<RawInterface> interfaces;
    for (auto &kv : raw) {
        double maxArea = 0.0;
        for (const V3 &c : kv.second) maxArea = std::max(maxArea, len(c));
        RawInterface ri;
        ri.pair.a = kv.first.first;
        ri.pair.b = kv.first.second;
        ri.pair.axis = axisOf[kv.first];
        for (const V3 &c : kv.second)
            if (len(c) > 1e-4 * maxArea)
                ri.norms.push_back(unit(c));
        if (!ri.norms.empty())
            interfaces.push_back(std::move(ri));
    }

    finishBlocking(rep, opt, interfaces);

    // Piece extents in R^3, from a grid of D classified into pieces.
    rep.pieceBox.assign(rep.pieces * 6, 0.0);
    for (int i = 0; i < rep.pieces; ++i)
        for (int k = 0; k < 3; ++k) {
            rep.pieceBox[i * 6 + 2 * k] = 1e300;
            rep.pieceBox[i * 6 + 2 * k + 1] = -1e300;
        }
    const int G = 24;
    for (int i = 0; i < G; ++i)
        for (int j = 0; j < G; ++j)
            for (int k = 0; k < G; ++k) {
                const double p[3] = { dom[0] + (dom[1] - dom[0]) * (i + 0.5) / G,
                                      dom[2] + (dom[3] - dom[2]) * (j + 0.5) / G,
                                      dom[4] + (dom[5] - dom[4]) * (k + 0.5) / G };
                const int pc = bsp.classify(p);
                if (pc < 0) continue;
                const V3 q = evalAt(tv, p);
                const double e[3] = { q.x, q.y, q.z };
                for (int d = 0; d < 3; ++d) {
                    rep.pieceBox[pc * 6 + 2 * d] = std::min(rep.pieceBox[pc * 6 + 2 * d], e[d]);
                    rep.pieceBox[pc * 6 + 2 * d + 1] = std::max(rep.pieceBox[pc * 6 + 2 * d + 1], e[d]);
                }
            }

    rep.valid = true;
    return rep;
}

DbgReport DbgAnalysis::runContacts(int pieces, const QVector<DbgContactSample> &contacts, const DbgOptions &opt)
{
    DbgReport rep;
    if (pieces < 2 || pieces > 31) {
        rep.error = QStringLiteral("%1 pieces: the analysis takes 2 to 31").arg(pieces);
        return rep;
    }
    rep.pieces = pieces;
    // One interface per pair, a the lower index; normals turned to point out of a.
    std::map<std::pair<int, int>, std::vector<V3>> raw;
    for (const DbgContactSample &c : contacts) {
        if (c.a < 0 || c.b < 0 || c.a >= pieces || c.b >= pieces || c.a == c.b) continue;
        V3 n { c.n[0], c.n[1], c.n[2] };
        if (!(len(n) > 0)) continue;
        n = unit(n);
        if (c.a < c.b) raw[{ c.a, c.b }].push_back(n);
        else raw[{ c.b, c.a }].push_back(V3{ -n.x, -n.y, -n.z });
    }
    std::vector<RawInterface> interfaces;
    for (auto &kv : raw) {
        RawInterface ri;
        ri.pair.a = kv.first.first;
        ri.pair.b = kv.first.second;
        ri.norms = std::move(kv.second);
        interfaces.push_back(std::move(ri));
    }
    finishBlocking(rep, opt, interfaces);
    rep.valid = true;
    return rep;
}

DbgQuality DbgAnalysis::qualityBsp(const Trivariate &tv, const CurvedBsp &bsp)
{
    DbgQuality q;
    if (!tv.isValid() || !bsp.isValid())
        return q;
    double dom[6];
    tv.domain(dom);
    double h[3];
    for (int a = 0; a < 3; ++a)
        h[a] = (dom[a * 2 + 1] - dom[a * 2]) * 1e-5;
    q.modelSize = modelScale(tv, dom);
    q.pieces.resize(bsp.pieceCount());
    std::vector<char> seen(bsp.pieceCount(), 0);

    // Volume = integral of det J over the piece's region of D (midpoint rule on
    // a grid classified into pieces); the fold check on the same samples.
    const int G = 28;
    const double cellVol = (dom[1] - dom[0]) * (dom[3] - dom[2]) * (dom[5] - dom[4]) / (G * G * G);
    for (int i = 0; i < G; ++i)
        for (int j = 0; j < G; ++j)
            for (int k = 0; k < G; ++k) {
                const double p[3] = { dom[0] + (dom[1] - dom[0]) * (i + 0.5) / G,
                                      dom[2] + (dom[3] - dom[2]) * (j + 0.5) / G,
                                      dom[4] + (dom[5] - dom[4]) * (k + 0.5) / G };
                const int pc = bsp.classify(p);
                if (pc < 0) continue;
                const double det = dot(tangent(tv, p, 0, dom, h[0]),
                                       cross(tangent(tv, p, 1, dom, h[1]), tangent(tv, p, 2, dom, h[2])));
                DbgPieceQuality &pq = q.pieces[pc];
                pq.volume += std::fabs(det) * cellVol;
                if (!seen[pc] || det < pq.minDetJ) { pq.minDetJ = det; seen[pc] = 1; }
            }
    // Thickness is not measured for curved pieces yet (the ray test walks the
    // cell's box faces); -1 = not measured.
    if (!q.pieces.isEmpty()) {
        double sum = 0.0;
        q.volMin = q.volMax = q.pieces[0].volume;
        for (const DbgPieceQuality &pq : q.pieces) {
            sum += pq.volume;
            q.volMin = std::min(q.volMin, pq.volume);
            q.volMax = std::max(q.volMax, pq.volume);
        }
        q.volMean = sum / q.pieces.size();
        double var = 0.0;
        for (const DbgPieceQuality &pq : q.pieces)
            var += (pq.volume - q.volMean) * (pq.volume - q.volMean);
        q.volCv = q.volMean > 0.0 ? std::sqrt(var / q.pieces.size()) / q.volMean : 0.0;
    }
    return q;
}

DbgSweep DbgAnalysis::sweepBsp(const Trivariate &tv, const CurvedBsp &bsp,
                               const QVector<int> &group, const double dir[3],
                               double travel, int steps, int perAxis)
{
    if (!tv.isValid() || !bsp.isValid() || group.isEmpty())
        return DbgSweep();
    double dom[6];
    tv.domain(dom);
    std::vector<char> moving(bsp.pieceCount(), 0);
    for (int g : group)
        if (g >= 0 && g < bsp.pieceCount()) moving[g] = 1;

    // Start points: a grid over D, kept where it falls inside a moving piece.
    std::vector<std::array<double, 3>> starts;
    const int G = std::max(8, 2 * perAxis);
    for (int i = 0; i < G; ++i)
        for (int j = 0; j < G; ++j)
            for (int k = 0; k < G; ++k) {
                const std::array<double, 3> p { dom[0] + (dom[1] - dom[0]) * (i + 0.5) / G,
                                                dom[2] + (dom[3] - dom[2]) * (j + 0.5) / G,
                                                dom[4] + (dom[5] - dom[4]) * (k + 0.5) / G };
                const int pc = bsp.classify(p.data());
                if (pc >= 0 && moving[pc]) starts.push_back(p);
            }
    const double clearMargin = 1e-3 * std::min({ dom[1] - dom[0], dom[3] - dom[2], dom[5] - dom[4] });
    const auto cls = [&](const double *p, bool *clearly) {
        double m = 0.0;
        const int pc = bsp.classify(p, &m);
        *clearly = m > clearMargin;
        return pc;
    };
    return slidePoints(tv, starts, moving, cls, dir, travel, steps);
}

// How far M is from the trilinear blend of its 8 corners, as a fraction of the
// model's size. A box cage (the old bounding-cage trivariate) gives 0: every
// cut in D then stays flat in R^3 and cannot interlock by curvature.
double DbgAnalysis::affineDeviation(const Trivariate &tv)
{
    if (!tv.isValid())
        return 0.0;
    double dom[6];
    tv.domain(dom);
    V3 c[8];
    for (int k = 0; k < 8; ++k) {
        const double p[3] = { dom[(k & 1) ? 1 : 0], dom[(k & 2) ? 3 : 2], dom[(k & 4) ? 5 : 4] };
        c[k] = evalAt(tv, p);
    }
    const double size = modelScale(tv, dom);
    double worst = 0.0;
    const int G = 8;
    for (int i = 0; i <= G; ++i)
        for (int j = 0; j <= G; ++j)
            for (int k = 0; k <= G; ++k) {
                const double t[3] = { double(i) / G, double(j) / G, double(k) / G };
                const double p[3] = { dom[0] + (dom[1] - dom[0]) * t[0],
                                      dom[2] + (dom[3] - dom[2]) * t[1],
                                      dom[4] + (dom[5] - dom[4]) * t[2] };
                V3 lin { 0, 0, 0 };
                for (int q = 0; q < 8; ++q) {
                    const double w = ((q & 1) ? t[0] : 1 - t[0]) * ((q & 2) ? t[1] : 1 - t[1]) *
                                     ((q & 4) ? t[2] : 1 - t[2]);
                    lin.x += w * c[q].x; lin.y += w * c[q].y; lin.z += w * c[q].z;
                }
                const V3 m = evalAt(tv, p);
                worst = std::max(worst, len(V3 { m.x - lin.x, m.y - lin.y, m.z - lin.z }));
            }
    return worst / size;
}
