//
// CutInD - implementation. See the header.
//
#include "CutInD.h"

#include <algorithm>
#include <cstdio>
#include <string>
#include <array>
#include <cmath>
#include <functional>
#include <map>
#include <random>
#include <unordered_map>
#include <vector>

namespace {

using V3 = std::array<double, 3>;
constexpr double kTwoPi = 6.28318530717958647692;

inline V3 add(const V3 &a, const V3 &b) { return { a[0] + b[0], a[1] + b[1], a[2] + b[2] }; }
inline V3 sub(const V3 &a, const V3 &b) { return { a[0] - b[0], a[1] - b[1], a[2] - b[2] }; }
inline V3 mul(const V3 &a, double s)    { return { a[0] * s, a[1] * s, a[2] * s }; }
inline double dot(const V3 &a, const V3 &b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
inline V3 cross(const V3 &a, const V3 &b)
{
    return { a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0] };
}
inline double len(const V3 &a) { return std::sqrt(dot(a, a)); }
inline long long edgeKey(int a, int b)
{
    if (a > b) std::swap(a, b);
    return (static_cast<long long>(a) << 32) | static_cast<unsigned int>(b);
}

// A position's exact key: the bits of its three floats. Printing to 7 digits
// (as first done) fused different floats - a float needs 9 - and left edges
// shared by 4 and 152 triangles after gluing.
inline std::string posKey(double x, double y, double z)
{
    const float f[3] = { float(x), float(y), float(z) };
    return std::string(reinterpret_cast<const char *>(f), sizeof f);
}

// ------------------------------------------------------------------ the block
std::vector<double> basisAt(const std::vector<double> &U, int order, int n, double t)
{
    std::vector<double> N(U.size() - 1, 0.0);
    int span = order - 1;
    if (t >= U[n]) span = n - 1;
    else
        for (int m = order - 1; m < n; ++m)
            if (t >= U[m] && t < U[m + 1]) { span = m; break; }
    N[span] = 1.0;
    for (int d = 1; d < order; ++d)
        for (int m = 0; m + d < int(U.size()) - 1; ++m) {
            double v = 0.0;
            const double d0 = U[m + d] - U[m], d1 = U[m + d + 1] - U[m + 1];
            if (d0 > 0) v += (t - U[m]) / d0 * N[m];
            if (d1 > 0) v += (U[m + d + 1] - t) / d1 * N[m + 1];
            N[m] = v;
        }
    N.resize(n);
    return N;
}

std::vector<double> openKnots(int n, int order)
{
    std::vector<double> U(n + order);
    const int inner = n - order + 1;
    for (int m = 0; m < n + order; ++m) U[m] = std::clamp(double(m - order + 1) / inner, 0.0, 1.0);
    return U;
}

// HarmonicFit's control net: open cubic in u, closed cubic in v, linear in w,
// domain [0,1]^3 - plus E, M extended past the faces along their normals.
struct Block {
    int nu = 0, nv = 0, nw = 0;
    std::vector<V3> C;
    std::vector<double> U, V, W;

    explicit Block(const HarmonicFit::Result &r) : nu(r.nu), nv(r.nv), nw(r.nw), C(size_t(r.nu) * r.nv * r.nw)
    {
        for (size_t m = 0; m < C.size(); ++m) C[m] = { r.ctrl[int(3 * m)], r.ctrl[int(3 * m) + 1], r.ctrl[int(3 * m) + 2] };
        U = openKnots(nu, 4);
        W = openKnots(nw, 2);
        V.resize(nv + 7);
        for (int m = 0; m < nv + 7; ++m) V[m] = double(m - 3) / nv;
    }
    static double wrap(double v) { v -= std::floor(v); return v >= 1.0 ? 0.0 : v; }
    V3 eval(double u, double v, double w) const
    {
        u = std::clamp(u, 0.0, 1.0);
        w = std::clamp(w, 0.0, 1.0);
        v = wrap(v);
        const std::vector<double> bu = basisAt(U, 4, nu, u), bv = basisAt(V, 4, nv + 3, v), bw = basisAt(W, 2, nw, w);
        V3 p = { 0, 0, 0 };
        for (int k = 0; k < nw; ++k) {
            if (bw[k] == 0.0) continue;
            for (int jj = 0; jj < nv + 3; ++jj) {
                if (bv[jj] == 0.0) continue;
                for (int i = 0; i < nu; ++i) {
                    if (bu[i] == 0.0) continue;
                    p = add(p, mul(C[size_t(i) + size_t(nu) * (size_t(jj % nv) + size_t(nv) * k)], bu[i] * bv[jj] * bw[k]));
                }
            }
        }
        return p;
    }
    V3 d(double u, double v, double w, int a) const
    {
        const double h = 1e-4;
        double lo[3] = { u, v, w }, hi[3] = { u, v, w };
        if (a == 1) { lo[1] -= h; hi[1] += h; }
        else {
            lo[a] = std::max(0.0, lo[a] - h);
            hi[a] = std::min(1.0, hi[a] + h);
        }
        const double span = hi[a] - lo[a];
        return mul(sub(eval(hi[0], hi[1], hi[2]), eval(lo[0], lo[1], lo[2])), 1.0 / span);
    }
    // Outward normal of the side face w = 0 (w grows inward).
    V3 sideNormal(double u, double v) const
    {
        V3 n = cross(d(u, v, 0.0, 0), d(u, v, 0.0, 1));
        if (dot(n, d(u, v, 0.0, 2)) > 0) n = mul(n, -1.0);
        const double l = len(n);
        return l > 0 ? mul(n, 1.0 / l) : V3{ 0, 0, 0 };
    }
    // M extended: past w = 0 along the side normal, past u = 0 / 1 along -/+ M_u,
    // each scaled by the derivative there - the inverse below uses the same rule.
    V3 E(double u, double v, double w) const
    {
        const double uc = std::clamp(u, 0.0, 1.0), wc = std::clamp(w, 0.0, 1.0);
        V3 p = eval(uc, v, wc);
        if (w < 0.0) p = add(p, mul(sideNormal(uc, v), -w * len(d(uc, v, 0.0, 2))));
        if (u < 0.0) {
            const V3 mu = d(0.0, v, wc, 0);
            p = add(p, mul(mu, u));                                   // back along M_u before 0
        } else if (u > 1.0) {
            // Past the tip, where M collapses to one point: straight on from
            // just before it, so different (v, w) stay different points (the
            // tip's own M_u sent whole planes of D onto one line: duplicate
            // vertices and slivers in the cut faces).
            const double h = 0.02;
            const V3 p0 = eval(1.0 - h, v, wc);
            p = add(p, mul(sub(p, p0), (u - 1.0) / h));
        }
        return p;
    }
};

// Q (the solid-cylinder picture of D) <-> (u, v, w).
inline V3 toQ(double u, double v, double w)
{
    const double r = std::max(0.0, 1.0 - w);
    return { u, r * std::cos(kTwoPi * v), r * std::sin(kTwoPi * v) };
}
inline void fromQ(const V3 &q, double *u, double *v, double *w)
{
    const double r = std::sqrt(q[1] * q[1] + q[2] * q[2]);
    *u = q[0];
    *w = 1.0 - r;
    double th = std::atan2(q[2], q[1]);
    if (th < 0) th += kTwoPi;
    *v = th / kTwoPi;
}

// ------------------------------------------------------------------ the part in D
// Every vertex of the part: its D coordinates by Newton on M from the nearest
// of a grid of samples; a vertex M does not reach (outside the block) gets the
// nearest side or end face point and an extended coordinate past it.
std::vector<V3> mapToQ(const Block &B, const MeshData &part, double size, int *outside, int *unconverged)
{
    struct S { V3 p; double u, v, w; };
    std::vector<S> smp;
    const int Gu = 24, Gv = 32, Gw = 10;
    for (int i = 0; i <= Gu; ++i)
        for (int j = 0; j < Gv; ++j)
            for (int k = 0; k <= Gw; ++k) {
                const double u = double(i) / Gu, v = double(j) / Gv, w = double(k) / Gw;
                smp.push_back({ B.eval(u, v, w), u, v, w });
            }
    V3 lo = { 1e300, 1e300, 1e300 }, hi = { -1e300, -1e300, -1e300 };
    for (const auto &s : smp) for (int a = 0; a < 3; ++a) { lo[a] = std::min(lo[a], s.p[a]); hi[a] = std::max(hi[a], s.p[a]); }
    for (int i = 0; i < part.vertexCount(); ++i)
        for (int a = 0; a < 3; ++a) { lo[a] = std::min(lo[a], double(part.pos[3 * i + a])); hi[a] = std::max(hi[a], double(part.pos[3 * i + a])); }
    double cell = 0.0;
    for (int a = 0; a < 3; ++a) cell = std::max(cell, (hi[a] - lo[a]) / 32.0);
    cell = std::max(cell, 1e-12);
    int nb[3];
    for (int a = 0; a < 3; ++a) nb[a] = int((hi[a] - lo[a]) / cell) + 1;
    auto cid = [&](const V3 &p, int a) { return std::clamp(int((p[a] - lo[a]) / cell), 0, nb[a] - 1); };
    std::vector<std::vector<int>> bucket(size_t(nb[0]) * nb[1] * nb[2]);
    for (int m = 0; m < int(smp.size()); ++m)
        bucket[size_t(cid(smp[m].p, 0)) + size_t(nb[0]) * (cid(smp[m].p, 1) + size_t(nb[1]) * cid(smp[m].p, 2))].push_back(m);
    auto nearest = [&](const V3 &x) {
        const int c[3] = { cid(x, 0), cid(x, 1), cid(x, 2) };
        double best = 1e300;
        int bi = 0;
        for (int rr = 0; rr < std::max({ nb[0], nb[1], nb[2] }); ++rr) {
            for (int dk = -rr; dk <= rr; ++dk)
                for (int dj = -rr; dj <= rr; ++dj)
                    for (int di = -rr; di <= rr; ++di) {
                        if (std::max({ std::abs(di), std::abs(dj), std::abs(dk) }) != rr) continue;
                        const int q[3] = { c[0] + di, c[1] + dj, c[2] + dk };
                        if (q[0] < 0 || q[1] < 0 || q[2] < 0 || q[0] >= nb[0] || q[1] >= nb[1] || q[2] >= nb[2]) continue;
                        for (int m : bucket[size_t(q[0]) + size_t(nb[0]) * (q[1] + size_t(nb[1]) * q[2])]) {
                            const double dd = len(sub(smp[m].p, x));
                            if (dd < best) { best = dd; bi = m; }
                        }
                    }
            if (best < 1e299 && (rr - 1) * cell > best) break;
        }
        return bi;
    };
    auto solve3 = [](const V3 &a, const V3 &b, const V3 &c, const V3 &r, double out[3]) {
        const double det = dot(a, cross(b, c));
        if (std::fabs(det) < 1e-30) return false;
        out[0] = dot(r, cross(b, c)) / det;
        out[1] = dot(a, cross(r, c)) / det;
        out[2] = dot(a, cross(b, r)) / det;
        return true;
    };

    const double tol = 1e-5 * size;
    const V3 tip = B.eval(1.0, 0.0, 0.0);
    std::vector<V3> Q(part.vertexCount());
    *outside = 0;
    *unconverged = 0;
    for (int i = 0; i < part.vertexCount(); ++i) {
        const V3 x = { part.pos[3 * i], part.pos[3 * i + 1], part.pos[3 * i + 2] };
        if (len(sub(x, tip)) < 1e-6 * size) { Q[i] = { 1.0, 0.0, 0.0 }; continue; }   // the collapsed tip
        const S &s0 = smp[nearest(x)];
        double u = s0.u, v = s0.v, w = s0.w;
        double res = 1e300;
        for (int it = 0; it < 25; ++it) {
            const V3 r = sub(x, B.eval(u, v, w));
            res = len(r);
            if (res < tol) break;
            double dp[3];
            if (!solve3(B.d(u, v, w, 0), B.d(u, v, w, 1), B.d(u, v, w, 2), r, dp)) break;
            for (int a = 0; a < 3; ++a) dp[a] = std::clamp(dp[a], -0.1, 0.1);
            u = std::clamp(u + dp[0], 0.0, 1.0);
            v = Block::wrap(v + dp[1]);
            w = std::clamp(w + dp[2], 0.0, 1.0);
        }
        if (res >= tol * 10) {
            // Not reached by M: past the side (w = 0) or an end (u = 0 / 1).
            const V3 q = B.eval(u, v, w);
            const V3 off = sub(x, q);
            bool placed = false;
            if (w <= 0.02) {
                // Gauss-Newton on the side face for the nearest point.
                for (int it = 0; it < 8; ++it) {
                    const V3 r = sub(x, B.eval(u, v, 0.0));
                    const V3 a0 = B.d(u, v, 0.0, 0), a1 = B.d(u, v, 0.0, 1);
                    const double A = dot(a0, a0), Bb = dot(a0, a1), Cc = dot(a1, a1);
                    const double det = A * Cc - Bb * Bb;
                    if (std::fabs(det) < 1e-30) break;
                    const double b0 = dot(a0, r), b1 = dot(a1, r);
                    u = std::clamp(u + std::clamp((Cc * b0 - Bb * b1) / det, -0.1, 0.1), 0.0, 1.0);
                    v = Block::wrap(v + std::clamp((A * b1 - Bb * b0) / det, -0.1, 0.1));
                }
                const V3 qs = B.eval(u, v, 0.0);
                const V3 n = B.sideNormal(u, v);
                const double dd = dot(sub(x, qs), n);
                if (dd > 0) {
                    w = -dd / std::max(1e-30, len(B.d(u, v, 0.0, 2)));
                    placed = true;
                    ++*outside;
                } else {
                    w = 0.0;
                }
            }
            if (!placed && (u <= 1e-3 || u >= 1.0 - 1e-3)) {
                const V3 mu = B.d(u, v, w, 0);
                const double s = dot(mu, mu);
                if (s > 0) {
                    const double t = dot(off, mu) / s;                 // along M_u, past the end
                    if ((u <= 1e-3 && t < 0) || (u >= 1.0 - 1e-3 && t > 0)) { u += t; placed = true; ++*outside; }
                }
            }
            if (!placed) ++*unconverged;
        }
        // Bounded: past the collapsed tip M_u is tiny, and dividing by it gave
        // coordinates that made the cut outlines explode (150,000-vertex pieces).
        u = std::clamp(u, -0.5, 1.5);
        w = std::max(w, -1.5);
        Q[i] = toQ(u, v, w);
    }
    return Q;
}

// ------------------------------------------------------------------ meshes cut in Q
struct PMesh {
    std::vector<V3> x, q;                       // model position, Q position
    std::vector<std::array<int, 3>> t;
    double u0 = 0, u1 = 1, th0 = 0, th1 = kTwoPi, r0 = 0, r1 = 1e9;
    bool core = false;
    double volume() const
    {
        double v = 0.0;
        for (const auto &tr : t) v += dot(x[tr[0]], cross(x[tr[1]], x[tr[2]]));
        return std::fabs(v) / 6.0;
    }
};

// Ear clipping of a simple polygon (2D, may be non-convex); returns index
// triples into pts in the polygon's own orientation.
std::vector<std::array<int, 3>> earClip(const std::vector<std::array<double, 2>> &pts, const std::vector<int> &poly,
                                        bool *stuck = nullptr)
{
    std::vector<std::array<int, 3>> out;
    std::vector<int> P = poly;
    double area = 0.0;
    for (size_t i = 0; i < P.size(); ++i) {
        const auto &a = pts[P[i]], &b = pts[P[(i + 1) % P.size()]];
        area += a[0] * b[1] - b[0] * a[1];
    }
    const bool flip = area < 0;
    if (flip) std::reverse(P.begin(), P.end());
    auto crs = [&](int a, int b, int c) {
        return (pts[b][0] - pts[a][0]) * (pts[c][1] - pts[a][1]) - (pts[b][1] - pts[a][1]) * (pts[c][0] - pts[a][0]);
    };
    auto inside = [&](int p, int a, int b, int c) {
        const double d1 = crs(a, b, p), d2 = crs(b, c, p), d3 = crs(c, a, p);
        return d1 > 1e-14 && d2 > 1e-14 && d3 > 1e-14;
    };
    size_t guard = 0;
    while (P.size() > 3 && guard++ < 100000) {
        const size_t n = P.size();
        int ear = -1;
        double bestC = -1e300;
        int fallback = -1;
        for (size_t i = 0; i < n; ++i) {
            const int a = P[(i + n - 1) % n], b = P[i], c = P[(i + 1) % n];
            const double cr = crs(a, b, c);
            if (cr > bestC) { bestC = cr; fallback = int(i); }
            if (cr <= 1e-14) continue;
            bool empty = true;
            for (size_t j = 0; j < n && empty; ++j) {
                const int p = P[j];
                if (p == a || p == b || p == c) continue;
                if (pts[p] == pts[a] || pts[p] == pts[b] || pts[p] == pts[c]) continue;   // bridge duplicates
                if (inside(p, a, b, c)) empty = false;
            }
            if (empty) { ear = int(i); break; }
        }
        if (ear < 0) {                                  // no clean ear: the outline folds over itself
            if (stuck) *stuck = true;
            ear = fallback;
        }
        const int a = P[(ear + n - 1) % n], b = P[ear], c = P[(ear + 1) % n];
        out.push_back({ a, b, c });
        P.erase(P.begin() + ear);
    }
    if (P.size() == 3) out.push_back({ P[0], P[1], P[2] });
    if (flip) for (auto &tr : out) std::swap(tr[1], tr[2]);
    return out;
}

// One cut: split `in` by field f (lo: f < 0) and close both sides with the same
// cap, triangulated in 2D coordinates to2D(q) and refined with points
// from2D(a, b) -> q placed on the cut surface. Returns false if a side is empty
// or the cut outline could not be closed.
bool cutMesh(const PMesh &in, const Block &B, const std::function<double(const V3 &)> &f,
             const std::function<std::array<double, 2>(const V3 &)> &to2D,
             const std::function<V3(double, double)> &from2D, double refine,
             PMesh *lo, PMesh *hi, QString *why)
{
    const int n = int(in.x.size());
    std::vector<double> fv(n);
    for (int i = 0; i < n; ++i) { fv[i] = f(in.q[i]); if (fv[i] == 0.0) fv[i] = 1e-12; }

    // Global vertex list: the input's, then crossings, then cap points.
    std::vector<V3> gx = in.x, gq = in.q;
    std::unordered_map<long long, int> crossMap;
    auto crossing = [&](int a, int b) {
        const long long k = edgeKey(a, b);
        auto it = crossMap.find(k);
        if (it != crossMap.end()) return it->second;
        const double t = fv[a] / (fv[a] - fv[b]);
        // A crossing on (or next to) a vertex is that vertex: a new point on
        // top of it fused with it later and left edges shared by 4-152 triangles.
        if (t < 1e-7) { crossMap.emplace(k, a); return a; }
        if (t > 1.0 - 1e-7) { crossMap.emplace(k, b); return b; }
        gx.push_back(add(in.x[a], mul(sub(in.x[b], in.x[a]), t)));
        gq.push_back(add(in.q[a], mul(sub(in.q[b], in.q[a]), t)));
        const int id = int(gx.size()) - 1;
        crossMap.emplace(k, id);
        return id;
    };
    std::vector<std::array<int, 3>> tLo, tHi;
    for (const auto &tr : in.t) {
        const bool s0 = fv[tr[0]] < 0, s1 = fv[tr[1]] < 0, s2 = fv[tr[2]] < 0;
        if (s0 == s1 && s1 == s2) { (s0 ? tLo : tHi).push_back(tr); continue; }
        const int odd = s0 == s1 ? 2 : (s0 == s2 ? 1 : 0);
        const int o = tr[odd], a = tr[(odd + 1) % 3], b = tr[(odd + 2) % 3];
        const int pa = crossing(o, a), pb = crossing(o, b);
        auto &oSide = fv[o] < 0 ? tLo : tHi;
        auto &rSide = fv[o] < 0 ? tHi : tLo;
        auto put = [](std::vector<std::array<int, 3>> &side, int x, int y, int z) {
            if (x != y && y != z && x != z) side.push_back({ x, y, z });   // a crossing on a vertex: no sliver
        };
        put(oSide, o, pa, pb);
        put(rSide, pa, a, b);
        put(rSide, pa, b, pb);
    }
    if (tLo.empty() || tHi.empty()) { if (why) *why = QStringLiteral("the cut misses the piece"); return false; }

    // The outline: boundary edges of the low side, walked.
    std::unordered_map<long long, int> ecount;
    for (const auto &tr : tLo) for (int k = 0; k < 3; ++k) ++ecount[edgeKey(tr[k], tr[(k + 1) % 3])];
    std::multimap<int, int> next;
    for (const auto &tr : tLo)
        for (int k = 0; k < 3; ++k)
            if (ecount[edgeKey(tr[k], tr[(k + 1) % 3])] == 1) next.emplace(tr[k], tr[(k + 1) % 3]);
    std::vector<std::vector<int>> loops;
    while (!next.empty()) {
        std::vector<int> loop;
        auto it = next.begin();
        const int start = it->first;
        int cur = start;
        size_t guard = 0;
        while (guard++ < 1000000) {
            auto jt = next.find(cur);
            if (jt == next.end()) break;
            const int nx = jt->second;
            next.erase(jt);
            loop.push_back(cur);
            cur = nx;
            if (cur == start) break;
        }
        if (cur != start || loop.size() < 3) { if (why) *why = QStringLiteral("the cut outline is not a closed loop"); return false; }
        loops.push_back(loop);
    }

    // The cap polygon runs against the low side's boundary.
    std::vector<std::array<double, 2>> pts(gq.size());
    for (size_t i = 0; i < gq.size(); ++i) pts[i] = to2D(gq[i]);
    for (auto &l : loops) std::reverse(l.begin(), l.end());
    auto area2 = [&](const std::vector<int> &l) {
        double a = 0.0;
        for (size_t i = 0; i < l.size(); ++i) {
            const auto &p = pts[l[i]], &q = pts[l[(i + 1) % l.size()]];
            a += p[0] * q[1] - q[0] * p[1];
        }
        return 0.5 * a;
    };
    int big = 0;
    for (size_t i = 1; i < loops.size(); ++i) if (std::fabs(area2(loops[i])) > std::fabs(area2(loops[big]))) big = int(i);
    const bool outerSign = area2(loops[big]) > 0;
    std::vector<std::vector<int>> outers, holes;
    for (auto &l : loops) ((area2(l) > 0) == outerSign ? outers : holes).push_back(l);
    auto pointIn = [&](const std::array<double, 2> &p, const std::vector<int> &l) {
        bool in = false;
        for (size_t i = 0, j = l.size() - 1; i < l.size(); j = i++) {
            const auto &a = pts[l[i]], &b = pts[l[j]];
            if ((a[1] > p[1]) != (b[1] > p[1]) && p[0] < (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]) + a[0]) in = !in;
        }
        return in;
    };
    // Bridge every hole into the outer loop that contains it (rightmost hole
    // point, ray to +x, the hit edge's endpoint further along x).
    // A "hole" that lies in no outline is its own region (a mapping into D
    // that is not clean can flip a small outline): triangulated on its own,
    // so every edge of the cut still gets its cap and the pieces stay closed.
    std::vector<std::vector<int>> loose;
    for (auto &h : holes) {
        int oi = -1;
        for (size_t o = 0; o < outers.size(); ++o) if (pointIn(pts[h[0]], outers[o])) { oi = int(o); break; }
        if (oi < 0) { loose.push_back(h); continue; }
        auto &O = outers[oi];
        size_t hm = 0;
        for (size_t i = 1; i < h.size(); ++i) if (pts[h[i]][0] > pts[h[hm]][0]) hm = i;
        const auto P = pts[h[hm]];
        double bestX = 1e300;
        int bridge = -1;
        for (size_t i = 0; i < O.size(); ++i) {
            const auto &a = pts[O[i]], &b = pts[O[(i + 1) % O.size()]];
            if ((a[1] > P[1]) == (b[1] > P[1])) continue;
            const double xi = a[0] + (P[1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1]);
            if (xi < P[0] || xi >= bestX) continue;
            bestX = xi;
            bridge = int(a[0] > b[0] ? i : (i + 1) % O.size());
        }
        if (bridge < 0) { loose.push_back(h); continue; }
        std::vector<int> merged(O.begin(), O.begin() + bridge + 1);
        for (size_t k = 0; k <= h.size(); ++k) merged.push_back(h[(hm + k) % h.size()]);
        merged.push_back(O[bridge]);
        merged.insert(merged.end(), O.begin() + bridge + 1, O.end());
        O.swap(merged);
    }
    for (auto &l : loose) outers.push_back(l);
    std::vector<std::array<int, 3>> cap;
    bool stuck = false;
    for (const auto &O : outers) {
        const auto tris = earClip(pts, O, &stuck);
        cap.insert(cap.end(), tris.begin(), tris.end());
    }
    // A folded outline (the part's map into D is not one-to-one there) gives
    // overlapping cap triangles - a double-covered face, duplicate vertices.
    // Refuse the cut; the caller tries another position.
    if (stuck) { if (why) *why = QStringLiteral("the cut outline folds over itself in D"); return false; }
    // Refine: big cap triangles get their centroid on the cut surface (the
    // outline's edges are never split, so the sides stay watertight).
    for (int pass = 0; pass < 2; ++pass) {
        std::vector<std::array<int, 3>> out;
        for (const auto &tr : cap) {
            double e = 0.0;
            for (int k = 0; k < 3; ++k) {
                const auto &a = pts[tr[k]], &b = pts[tr[(k + 1) % 3]];
                e = std::max(e, std::hypot(a[0] - b[0], a[1] - b[1]));
            }
            if (e <= refine) { out.push_back(tr); continue; }
            const double ca = (pts[tr[0]][0] + pts[tr[1]][0] + pts[tr[2]][0]) / 3.0;
            const double cb = (pts[tr[0]][1] + pts[tr[1]][1] + pts[tr[2]][1]) / 3.0;
            const V3 q = from2D(ca, cb);
            double u, v, w;
            fromQ(q, &u, &v, &w);
            gx.push_back(B.E(u, v, w));
            gq.push_back(q);
            pts.push_back({ ca, cb });
            const int c = int(gx.size()) - 1;
            out.push_back({ tr[0], tr[1], c });
            out.push_back({ tr[1], tr[2], c });
            out.push_back({ tr[2], tr[0], c });
        }
        cap.swap(out);
    }

    // Assemble both sides: the low one gets the cap, the high one its reverse.
    auto build = [&](const std::vector<std::array<int, 3>> &tris, bool reverseCap, PMesh *m) {
        std::unordered_map<int, int> id;
        auto get = [&](int g) {
            auto it = id.find(g);
            if (it != id.end()) return it->second;
            m->x.push_back(gx[g]);
            m->q.push_back(gq[g]);
            const int k = int(m->x.size()) - 1;
            id.emplace(g, k);
            return k;
        };
        for (const auto &tr : tris) m->t.push_back({ get(tr[0]), get(tr[1]), get(tr[2]) });
        for (const auto &tr : cap)
            m->t.push_back(reverseCap ? std::array<int, 3>{ get(tr[0]), get(tr[2]), get(tr[1]) }
                                      : std::array<int, 3>{ get(tr[0]), get(tr[1]), get(tr[2]) });
    };
    *lo = PMesh();
    *hi = PMesh();
    lo->u0 = hi->u0 = in.u0; lo->u1 = hi->u1 = in.u1;
    lo->th0 = hi->th0 = in.th0; lo->th1 = hi->th1 = in.th1;
    lo->r0 = hi->r0 = in.r0; lo->r1 = hi->r1 = in.r1;
    lo->core = hi->core = in.core;
    build(tLo, false, lo);
    build(tHi, true, hi);
    return true;
}

// The core's cap is a tube, r = rc, between the outline on the part's cap end
// and the one near its tip: rings of points on M(u, v, 1 - rc) in between,
// zipped by angle.
bool cutCore(const PMesh &in, const Block &B, double rc, PMesh *core, PMesh *shell, QString *why)
{
    // Split with a flat placeholder cap, then replace the cap by the tube.
    const int n = int(in.x.size());
    std::vector<double> fv(n);
    for (int i = 0; i < n; ++i) {
        fv[i] = std::sqrt(in.q[i][1] * in.q[i][1] + in.q[i][2] * in.q[i][2]) - rc;
        if (fv[i] == 0.0) fv[i] = 1e-12;
    }
    std::vector<V3> gx = in.x, gq = in.q;
    std::unordered_map<long long, int> crossMap;
    auto crossing = [&](int a, int b) {
        const long long k = edgeKey(a, b);
        auto it = crossMap.find(k);
        if (it != crossMap.end()) return it->second;
        const double t = fv[a] / (fv[a] - fv[b]);
        if (t < 1e-7) { crossMap.emplace(k, a); return a; }
        if (t > 1.0 - 1e-7) { crossMap.emplace(k, b); return b; }
        gx.push_back(add(in.x[a], mul(sub(in.x[b], in.x[a]), t)));
        V3 q = add(in.q[a], mul(sub(in.q[b], in.q[a]), t));
        const double r = std::sqrt(q[1] * q[1] + q[2] * q[2]);
        if (r > 0) { q[1] *= rc / r; q[2] *= rc / r; }      // on the tube exactly
        gq.push_back(q);
        const int id = int(gx.size()) - 1;
        crossMap.emplace(k, id);
        return id;
    };
    std::vector<std::array<int, 3>> tIn, tOut;
    for (const auto &tr : in.t) {
        const bool s0 = fv[tr[0]] < 0, s1 = fv[tr[1]] < 0, s2 = fv[tr[2]] < 0;
        if (s0 == s1 && s1 == s2) { (s0 ? tIn : tOut).push_back(tr); continue; }
        const int odd = s0 == s1 ? 2 : (s0 == s2 ? 1 : 0);
        const int o = tr[odd], a = tr[(odd + 1) % 3], b = tr[(odd + 2) % 3];
        const int pa = crossing(o, a), pb = crossing(o, b);
        auto &oSide = fv[o] < 0 ? tIn : tOut;
        auto &rSide = fv[o] < 0 ? tOut : tIn;
        auto put = [](std::vector<std::array<int, 3>> &side, int x, int y, int z) {
            if (x != y && y != z && x != z) side.push_back({ x, y, z });   // a crossing on a vertex: no sliver
        };
        put(oSide, o, pa, pb);
        put(rSide, pa, a, b);
        put(rSide, pa, b, pb);
    }
    if (tIn.empty() || tOut.empty()) { if (why) *why = QStringLiteral("the core misses the part"); return false; }

    std::unordered_map<long long, int> ecount;
    for (const auto &tr : tIn) for (int k = 0; k < 3; ++k) ++ecount[edgeKey(tr[k], tr[(k + 1) % 3])];
    std::multimap<int, int> next;
    for (const auto &tr : tIn)
        for (int k = 0; k < 3; ++k)
            if (ecount[edgeKey(tr[k], tr[(k + 1) % 3])] == 1) next.emplace(tr[k], tr[(k + 1) % 3]);
    std::vector<std::vector<int>> loops;
    while (!next.empty()) {
        std::vector<int> loop;
        const int start = next.begin()->first;
        int cur = start;
        size_t guard = 0;
        while (guard++ < 1000000) {
            auto jt = next.find(cur);
            if (jt == next.end()) break;
            const int nx = jt->second;
            next.erase(jt);
            loop.push_back(cur);
            cur = nx;
            if (cur == start) break;
        }
        if (cur != start || loop.size() < 3) { if (why) *why = QStringLiteral("the core outline is not closed"); return false; }
        loops.push_back(loop);
    }
    if (loops.size() != 2) {
        if (why) *why = QStringLiteral("the core meets the part in %1 outline(s), not 2").arg(loops.size());
        return false;
    }
    auto meanU = [&](const std::vector<int> &l) { double s = 0; for (int i : l) s += gq[i][0]; return s / l.size(); };
    if (meanU(loops[0]) > meanU(loops[1])) std::swap(loops[0], loops[1]);
    double uA = -1e300, uB = 1e300;
    for (int i : loops[0]) uA = std::max(uA, gq[i][0]);
    for (int i : loops[1]) uB = std::min(uB, gq[i][0]);
    if (!(uB > uA)) { if (why) *why = QStringLiteral("the core's two outlines overlap in u"); return false; }

    // Rings in between, 48 points each, and the two outlines, ordered by angle.
    auto angle = [&](int g) { double a = std::atan2(gq[g][2], gq[g][1]); return a < 0 ? a + kTwoPi : a; };
    auto byAngle = [&](std::vector<int> l) {
        std::sort(l.begin(), l.end(), [&](int x, int y) { return angle(x) < angle(y); });
        return l;
    };
    std::vector<std::vector<int>> rings;
    rings.push_back(byAngle(loops[0]));
    const int K = std::max(2, int(std::ceil((uB - uA) * 24)));
    const int NA = 48;
    for (int k = 1; k < K; ++k) {
        const double u = uA + (uB - uA) * k / K;
        std::vector<int> ring;
        for (int j = 0; j < NA; ++j) {
            const double v = (j + 0.5) / NA;
            gq.push_back(toQ(u, v, 1.0 - rc));
            gx.push_back(B.E(u, v, 1.0 - rc));
            ring.push_back(int(gx.size()) - 1);
        }
        rings.push_back(ring);
    }
    rings.push_back(byAngle(loops[1]));
    // Zip neighbouring rings: walk both by angle, always advancing the one
    // whose next point comes first.
    std::vector<std::array<int, 3>> tube;
    for (size_t k = 0; k + 1 < rings.size(); ++k) {
        const auto &A = rings[k], &Bv = rings[k + 1];
        size_t i = 0, j = 0;
        const size_t na = A.size(), nb = Bv.size();
        while (i < na || j < nb) {
            const double ai = i < na ? angle(A[(i + 1) % na]) + (i + 1 >= na ? kTwoPi : 0.0) : 1e300;
            const double bj = j < nb ? angle(Bv[(j + 1) % nb]) + (j + 1 >= nb ? kTwoPi : 0.0) : 1e300;
            if (ai <= bj) { tube.push_back({ A[i % na], A[(i + 1) % na], Bv[j % nb] }); ++i; }
            else          { tube.push_back({ A[i % na], Bv[(j + 1) % nb], Bv[j % nb] }); ++j; }
        }
    }
    // Orientation: the core's cap must face outward (r growing).
    {
        double s = 0.0;
        for (const auto &tr : tube) {
            const V3 nq = cross(sub(gq[tr[1]], gq[tr[0]]), sub(gq[tr[2]], gq[tr[0]]));
            const V3 c = mul(add(add(gq[tr[0]], gq[tr[1]]), gq[tr[2]]), 1.0 / 3.0);
            s += nq[1] * c[1] + nq[2] * c[2];
        }
        if (s < 0) for (auto &tr : tube) std::swap(tr[1], tr[2]);
    }
    auto build = [&](const std::vector<std::array<int, 3>> &tris, bool rev, PMesh *m) {
        std::unordered_map<int, int> id;
        auto get = [&](int g) {
            auto it = id.find(g);
            if (it != id.end()) return it->second;
            m->x.push_back(gx[g]);
            m->q.push_back(gq[g]);
            const int kk = int(m->x.size()) - 1;
            id.emplace(g, kk);
            return kk;
        };
        for (const auto &tr : tris) m->t.push_back({ get(tr[0]), get(tr[1]), get(tr[2]) });
        for (const auto &tr : tube)
            m->t.push_back(rev ? std::array<int, 3>{ get(tr[0]), get(tr[2]), get(tr[1]) }
                               : std::array<int, 3>{ get(tr[0]), get(tr[1]), get(tr[2]) });
    };
    *core = PMesh();
    *shell = PMesh();
    core->core = true;
    core->r0 = 0; core->r1 = rc;
    shell->r0 = rc; shell->r1 = 1e9;
    build(tIn, false, core);
    build(tOut, true, shell);
    return true;
}

int openEdgesOf(const PMesh &m)
{
    std::unordered_map<long long, int> e;
    for (const auto &tr : m.t) for (int k = 0; k < 3; ++k) ++e[edgeKey(tr[k], tr[(k + 1) % 3])];
    int open = 0;
    for (const auto &kv : e) if (kv.second != 2) ++open;
    return open;
}

}  // namespace

CutInD::Result CutInD::cut(const MeshData &part, const HarmonicFit::Result &block, int pieces, quint32 seed,
                           double coreRadius)
{
    Result R;
    if (!block.ok || part.triangleCount() == 0) { R.error = QStringLiteral("no block or no part"); return R; }
    const Block B(block);
    double size = 0.0;
    for (int a = 0; a < 3; ++a) size += double(part.bmax[a] - part.bmin[a]) * double(part.bmax[a] - part.bmin[a]);
    size = std::sqrt(size);

    // The part: welded (positions), each vertex mapped into Q.
    PMesh root;
    {
        std::unordered_map<std::string, int> weld;
        std::vector<int> remap(part.vertexCount());
        MeshData uniq;
        for (int i = 0; i < part.vertexCount(); ++i) {
            const std::string key = posKey(part.pos[3 * i], part.pos[3 * i + 1], part.pos[3 * i + 2]);
            auto it = weld.find(key);
            if (it == weld.end()) {
                remap[i] = uniq.addVertex(part.pos[3 * i], part.pos[3 * i + 1], part.pos[3 * i + 2]);
                weld.emplace(key, remap[i]);
            } else remap[i] = it->second;
        }
        int outside = 0, unconv = 0;
        const std::vector<V3> Q = mapToQ(B, uniq, size, &outside, &unconv);
        for (int i = 0; i < uniq.vertexCount(); ++i) {
            root.x.push_back({ uniq.pos[3 * i], uniq.pos[3 * i + 1], uniq.pos[3 * i + 2] });
            root.q.push_back(Q[i]);
        }
        for (int t = 0; t < part.triangleCount(); ++t) {
            const int a = remap[part.tris[3 * t]], b = remap[part.tris[3 * t + 1]], c = remap[part.tris[3 * t + 2]];
            if (a != b && b != c && a != c) root.t.push_back({ a, b, c });
        }
        R.notes << QStringLiteral("%1 vertices mapped into D: %2 outside the block (extended), %3 not converged")
                       .arg(uniq.vertexCount()).arg(outside).arg(unconv);
    }
    R.partVolume = root.volume();

    // Scales for choosing the cut direction: length along u, girth round v.
    double Lu = 0.0, girth = 0.0;
    for (int k = 0; k < 20; ++k) Lu += len(sub(B.E((k + 1) / 20.0, 0.0, 1.0), B.E(k / 20.0, 0.0, 1.0)));
    for (int k = 0; k < 10; ++k) girth += len(sub(B.E((k + 0.5) / 10, 0.0, 0.0), B.E((k + 0.5) / 10, 0.0, 1.0)));
    girth = kTwoPi * girth / 10.0;

    if (pieces <= 1) {                              // one piece: the part itself
        Piece p;
        p.mesh = part;
        R.pieces.append(p);
        R.piecesVolume = R.partVolume;
        R.ok = true;
        return R;
    }
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> jit(-0.15, 0.15);
    std::vector<PMesh> done;
    std::vector<PMesh> todo;
    QString why;
    const double refine = 0.12;

    // 1. The core round the skeleton.
    PMesh shellAll;
    bool haveCore = false;
    if (coreRadius > 0.0 && pieces >= 3) {
        PMesh core, shell;
        if (cutCore(root, B, coreRadius, &core, &shell, &why)) {
            haveCore = true;
            todo.push_back(std::move(core));
            shellAll = std::move(shell);
        } else {
            R.notes << QStringLiteral("no core (%1)").arg(why);
        }
    }
    if (!haveCore) shellAll = root;

    // 2. The seam: a plane through the axis splits the ring in two halves.
    std::vector<PMesh> shells;
    {
        PMesh a, b;
        const bool ok = cutMesh(shellAll, B, [](const V3 &q) { return q[2]; },
                                [](const V3 &q) { return std::array<double, 2>{ q[0], q[1] }; },
                                [](double x, double y) { return V3{ x, y, 0.0 }; }, refine, &a, &b, &why);
        if (!ok) { R.error = QStringLiteral("the first cut through the axis failed: %1").arg(why); return R; }
        a.th0 = 0.5 * kTwoPi; a.th1 = kTwoPi;          // q.z < 0: v in (1/2, 1)
        b.th0 = 0.0; b.th1 = 0.5 * kTwoPi;
        shells.push_back(std::move(a));
        shells.push_back(std::move(b));
    }
    for (auto &s : shells) todo.push_back(std::move(s));

    // 3. Split the biggest piece until there are enough: along u, or round v
    //    (shell only), whichever is longer in the model.
    auto pieceCount = [&]() { return int(todo.size() + done.size()); };
    int failures = 0;
    while (pieceCount() < pieces && !todo.empty() && failures < 20) {
        size_t bi = 0;
        for (size_t i = 1; i < todo.size(); ++i) if (todo[i].volume() > todo[bi].volume()) bi = i;
        PMesh cur = std::move(todo[bi]);
        todo.erase(todo.begin() + bi);
        const double lu = (cur.u1 - cur.u0) * Lu;
        const double lv = cur.core ? 0.0 : (cur.th1 - cur.th0) / kTwoPi * girth * 0.75;
        PMesh a, b;
        bool ok = false;
        const bool byU = lu >= lv;
        for (int attempt = 0; attempt < 6 && !ok; ++attempt) {
            // Preferred direction first; a refused cut (outline folding in D)
            // is retried at other positions, alternating the direction.
            const bool u = cur.core ? true : ((attempt % 2 == 0) ? byU : !byU);
            if (u) {
                // The part's actual u range in this piece.
                double umin = 1e300, umax = -1e300;
                for (const V3 &q : cur.q) { umin = std::min(umin, q[0]); umax = std::max(umax, q[0]); }
                umin = std::max(umin, cur.u0); umax = std::min(umax, cur.u1);
                const double c = umin + (0.5 + jit(rng)) * (umax - umin);
                ok = cutMesh(cur, B, [c](const V3 &q) { return q[0] - c; },
                             [](const V3 &q) { return std::array<double, 2>{ q[1], q[2] }; },
                             [c](double x, double y) { return V3{ c, x, y }; }, refine, &a, &b, &why);
                if (ok) { a.u1 = c; b.u0 = c; }
            } else {
                const double th = cur.th0 + (0.5 + jit(rng)) * (cur.th1 - cur.th0);
                const double ct = std::cos(th), st = std::sin(th);
                ok = cutMesh(cur, B, [ct, st](const V3 &q) { return q[2] * ct - q[1] * st; },
                             [ct, st](const V3 &q) { return std::array<double, 2>{ q[0], q[1] * ct + q[2] * st }; },
                             [ct, st](double x, double y) { return V3{ x, y * ct, y * st }; }, refine, &a, &b, &why);
                if (ok) { a.th1 = th; b.th0 = th; }
            }
        }
        if (!ok) {
            ++failures;
            R.notes << QStringLiteral("a cut failed (%1) - that piece is left whole").arg(why);
            done.push_back(std::move(cur));
            continue;
        }
        todo.push_back(std::move(a));
        todo.push_back(std::move(b));
    }
    for (auto &m : todo) done.push_back(std::move(m));

    // Out: meshes and their cells in D.
    for (const PMesh &m : done) {
        Piece p;
        for (const V3 &x : m.x) p.mesh.addVertex(x[0], x[1], x[2]);
        for (const auto &tr : m.t) {
            p.mesh.tris.push_back(uint32_t(tr[0]));
            p.mesh.tris.push_back(uint32_t(tr[1]));
            p.mesh.tris.push_back(uint32_t(tr[2]));
        }
        p.mesh.computeBounds();
        p.mesh.computeNormals();
        p.lo[0] = std::clamp(m.u0, 0.0, 1.0); p.hi[0] = std::clamp(m.u1, 0.0, 1.0);
        p.lo[1] = m.core ? 0.0 : m.th0 / kTwoPi; p.hi[1] = m.core ? 1.0 : m.th1 / kTwoPi;
        p.lo[2] = m.core ? 1.0 - coreRadius : 0.0;
        p.hi[2] = m.core ? 1.0 : (haveCore ? 1.0 - coreRadius : 1.0);
        p.core = m.core;
        R.piecesVolume += m.volume();
        R.openEdges += openEdgesOf(m);
        R.pieces.append(std::move(p));
    }
    R.notes << QStringLiteral("%1 piece(s)%2; their volume %3% of the part's; %4 open edge(s)")
                   .arg(R.pieces.size()).arg(haveCore ? QStringLiteral(" (1 core)") : QString())
                   .arg(R.partVolume > 0 ? 100.0 * R.piecesVolume / R.partVolume : 0.0, 0, 'f', 3)
                   .arg(R.openEdges);
    R.ok = true;
    return R;
}

// ================================================================== gluing
namespace {

struct Soup {
    std::vector<V3> p;
    std::vector<std::array<int, 3>> t;
};

double meshSize(const MeshData &m)
{
    double d = 0.0;
    for (int a = 0; a < 3; ++a) d += double(m.bmax[a] - m.bmin[a]) * double(m.bmax[a] - m.bmin[a]);
    return std::sqrt(d);
}

// Welds a soup by exact (float) position into a MeshData.
MeshData weld(const Soup &s)
{
    MeshData out;
    std::unordered_map<std::string, uint32_t> id;
    std::vector<uint32_t> map(s.p.size());
    for (size_t i = 0; i < s.p.size(); ++i) {
        const std::string key = posKey(s.p[i][0], s.p[i][1], s.p[i][2]);
        auto it = id.find(key);
        if (it == id.end()) {
            map[i] = out.addVertex(s.p[i][0], s.p[i][1], s.p[i][2]);
            id.emplace(key, map[i]);
        } else map[i] = it->second;
    }
    for (const auto &tr : s.t) {
        const uint32_t a = map[tr[0]], b = map[tr[1]], c = map[tr[2]];
        if (a == b || b == c || a == c) continue;
        out.tris.push_back(a); out.tris.push_back(b); out.tris.push_back(c);
    }
    out.computeBounds();
    out.computeNormals();
    return out;
}

V3 posOf(const MeshData &m, int i) { return { m.pos[3 * i], m.pos[3 * i + 1], m.pos[3 * i + 2] }; }

int nearestVertex(const MeshData &m, const V3 &c, double *dist)
{
    int best = -1;
    double bd = 1e300;
    for (int i = 0; i < m.vertexCount(); ++i) {
        const double d = len(sub(posOf(m, i), c));
        if (d < bd) { bd = d; best = i; }
    }
    if (dist) *dist = bd;
    return best;
}

double pointTriDist(const V3 &p, const V3 &a, const V3 &b, const V3 &c)
{
    // Closest point (Ericson 5.1.5).
    const V3 ab = sub(b, a), ac = sub(c, a), ap = sub(p, a);
    const double d1 = dot(ab, ap), d2 = dot(ac, ap);
    if (d1 <= 0 && d2 <= 0) return len(ap);
    const V3 bp = sub(p, b);
    const double d3 = dot(ab, bp), d4 = dot(ac, bp);
    if (d3 >= 0 && d4 <= d3) return len(bp);
    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) return len(sub(p, add(a, mul(ab, d1 / (d1 - d3)))));
    const V3 cp = sub(p, c);
    const double d5 = dot(ab, cp), d6 = dot(ac, cp);
    if (d6 >= 0 && d5 <= d6) return len(cp);
    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) return len(sub(p, add(a, mul(ac, d2 / (d2 - d6)))));
    const double va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0)
        return len(sub(p, add(b, mul(sub(c, b), (d4 - d3) / ((d4 - d3) + (d5 - d6))))));
    const double den = 1.0 / (va + vb + vc);
    return len(sub(p, add(a, add(mul(ab, vb * den), mul(ac, vc * den)))));
}

}  // namespace

int CutInD::openEdgesWelded(const MeshData &m)
{
    Soup sp;
    for (int i = 0; i < m.vertexCount(); ++i) sp.p.push_back(posOf(m, i));
    for (int t = 0; t < m.triangleCount(); ++t) sp.t.push_back({ int(m.tris[3 * t]), int(m.tris[3 * t + 1]), int(m.tris[3 * t + 2]) });
    return openEdges(weld(sp));
}

int CutInD::openEdges(const MeshData &m)
{
    std::unordered_map<long long, int> e;
    for (int t = 0; t < m.triangleCount(); ++t)
        for (int k = 0; k < 3; ++k) ++e[edgeKey(int(m.tris[3 * t + k]), int(m.tris[3 * t + (k + 1) % 3]))];
    int open = 0;
    for (const auto &kv : e) if (kv.second != 2) ++open;
    return open;
}

MeshData CutInD::unionAtCap(const MeshData &a, const MeshData &b, const double capCentre[3], QString *why)
{
    const V3 c = { capCentre[0], capCentre[1], capCentre[2] };
    const double tol = 1e-5 * std::max(meshSize(a), meshSize(b));
    double da = 0, db = 0;
    const int ca = nearestVertex(a, c, &da), cb = nearestVertex(b, c, &db);
    if (ca < 0 || cb < 0 || da > tol || db > tol) {
        if (why) *why = QStringLiteral("the shared cap was not found on both parts");
        return MeshData();
    }
    Soup s;
    auto addMesh = [&](const MeshData &m, int centre) {
        const int base = int(s.p.size());
        for (int i = 0; i < m.vertexCount(); ++i) s.p.push_back(posOf(m, i));
        for (int t = 0; t < m.triangleCount(); ++t) {
            const int x = int(m.tris[3 * t]), y = int(m.tris[3 * t + 1]), z = int(m.tris[3 * t + 2]);
            if (x == centre || y == centre || z == centre) continue;      // the shared fan
            s.t.push_back({ base + x, base + y, base + z });
        }
    };
    addMesh(a, ca);
    addMesh(b, cb);
    return weld(s);
}

int CutInD::attachAtCap(QVector<MeshData> *pieces, const MeshData &child, const double capCentre[3], QString *why)
{
    const V3 c = { capCentre[0], capCentre[1], capCentre[2] };
    const double size = meshSize(child);
    const double tol = 1e-5 * std::max(1e-12, size);
    double dc = 0;
    const int cv = nearestVertex(child, c, &dc);
    if (cv < 0 || dc > 10 * tol) { if (why) *why = QStringLiteral("the cap is not on the part"); return -1; }

    // The child's cap: its fan round the centre, and the rim edges.
    std::vector<std::array<V3, 3>> fan;
    std::vector<char> isFan(child.triangleCount(), 0);
    std::vector<std::pair<int, int>> rim;                    // rim edges as in the fan triangles (x -> y)
    for (int t = 0; t < child.triangleCount(); ++t) {
        const int v[3] = { int(child.tris[3 * t]), int(child.tris[3 * t + 1]), int(child.tris[3 * t + 2]) };
        for (int k = 0; k < 3; ++k)
            if (v[k] == cv) {
                isFan[t] = 1;
                fan.push_back({ posOf(child, v[0]), posOf(child, v[1]), posOf(child, v[2]) });
                rim.push_back({ v[(k + 1) % 3], v[(k + 2) % 3] });
            }
    }
    if (fan.empty()) { if (why) *why = QStringLiteral("the part has no fan at the cap"); return -1; }
    auto onFan = [&](const V3 &p) {
        for (const auto &f : fan) if (pointTriDist(p, f[0], f[1], f[2]) < 20 * tol) return true;
        return false;
    };

    // The parent pieces' triangles lying on that cap.
    const int np = pieces->size();
    std::vector<std::vector<int>> capTris(np);
    std::vector<double> capArea(np, 0.0);
    for (int j = 0; j < np; ++j) {
        const MeshData &m = (*pieces)[j];
        for (int t = 0; t < m.triangleCount(); ++t) {
            const V3 a = posOf(m, int(m.tris[3 * t])), b = posOf(m, int(m.tris[3 * t + 1])), d = posOf(m, int(m.tris[3 * t + 2]));
            if (!onFan(a) || !onFan(b) || !onFan(d) || !onFan(mul(add(add(a, b), d), 1.0 / 3.0))) continue;
            capTris[j].push_back(t);
            capArea[j] += 0.5 * len(cross(sub(b, a), sub(d, a)));
        }
    }
    int target = 0;
    for (int j = 1; j < np; ++j) if (capArea[j] > capArea[target]) target = j;
    if (capArea[target] <= 0.0) { if (why) *why = QStringLiteral("no piece of the parent touches the cap"); return -1; }

    // Points the parent's cuts put on the rim edges: the child's rim triangles
    // are split there, or the joint would have cracks (T-junctions).
    std::vector<V3> rimPts;
    for (int j = 0; j < np; ++j) {
        const MeshData &m = (*pieces)[j];
        for (int t : capTris[j])
            for (int k = 0; k < 3; ++k) rimPts.push_back(posOf(m, int(m.tris[3 * t + k])));
    }

    Soup s;
    // The target piece without its cap part.
    {
        const MeshData &m = (*pieces)[target];
        std::vector<char> drop(m.triangleCount(), 0);
        for (int t : capTris[target]) drop[t] = 1;
        const int base = int(s.p.size());
        for (int i = 0; i < m.vertexCount(); ++i) s.p.push_back(posOf(m, i));
        for (int t = 0; t < m.triangleCount(); ++t)
            if (!drop[t]) s.t.push_back({ base + int(m.tris[3 * t]), base + int(m.tris[3 * t + 1]), base + int(m.tris[3 * t + 2]) });
    }
    // The other pieces' cap parts, turned round: the new piece's faces toward them.
    for (int j = 0; j < np; ++j) {
        if (j == target) continue;
        const MeshData &m = (*pieces)[j];
        for (int t : capTris[j]) {
            const int base = int(s.p.size());
            for (int k = 0; k < 3; ++k) s.p.push_back(posOf(m, int(m.tris[3 * t + k])));
            s.t.push_back({ base, base + 2, base + 1 });
        }
    }
    // The child without its fan; a triangle on a rim edge is split at the
    // points the parent's cuts put on that edge.
    {
        const int base = int(s.p.size());
        for (int i = 0; i < child.vertexCount(); ++i) s.p.push_back(posOf(child, i));
        std::unordered_map<long long, int> rimEdge;
        for (const auto &e : rim) rimEdge[edgeKey(e.first, e.second)] = 1;
        for (int t = 0; t < child.triangleCount(); ++t) {
            if (isFan[t]) continue;
            const int v[3] = { int(child.tris[3 * t]), int(child.tris[3 * t + 1]), int(child.tris[3 * t + 2]) };
            int e = -1;
            for (int k = 0; k < 3 && e < 0; ++k) if (rimEdge.count(edgeKey(v[k], v[(k + 1) % 3]))) e = k;
            if (e < 0) { s.t.push_back({ base + v[0], base + v[1], base + v[2] }); continue; }
            const int x = v[e], y = v[(e + 1) % 3], o = v[(e + 2) % 3];
            const V3 px = posOf(child, x), py = posOf(child, y);
            const V3 dxy = sub(py, px);
            const double L2 = dot(dxy, dxy);
            std::vector<std::pair<double, V3>> on;
            for (const V3 &q : rimPts) {
                const double tt = L2 > 0 ? dot(sub(q, px), dxy) / L2 : 0.0;
                if (tt <= 1e-6 || tt >= 1.0 - 1e-6) continue;
                if (len(sub(q, add(px, mul(dxy, tt)))) > 20 * tol) continue;
                bool dup = false;
                for (const auto &w : on) if (len(sub(w.second, q)) < tol) { dup = true; break; }
                if (!dup) on.push_back({ tt, q });
            }
            if (on.empty()) { s.t.push_back({ base + v[0], base + v[1], base + v[2] }); continue; }
            std::sort(on.begin(), on.end(), [](const auto &l, const auto &r) { return l.first < r.first; });
            int prev = base + x;
            for (const auto &w : on) {
                s.p.push_back(w.second);
                const int id = int(s.p.size()) - 1;
                s.t.push_back({ prev, id, base + o });
                prev = id;
            }
            s.t.push_back({ prev, base + y, base + o });
        }
    }
    (*pieces)[target] = weld(s);
    return target;
}
