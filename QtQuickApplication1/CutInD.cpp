//
// CutInD - implementation. See the header.
//
#include "CutInD.h"

#include <QMap>

#include <algorithm>
#include <cstdio>
#include <string>
#include <array>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
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

// A cut's bulge: a quadratic tensor-product B-spline over [s0,s1] x [t0,t1]
// that is zero on its border. The mesh is cut by this function and the cut's
// surface in D is built from the same coefficients, so the two agree exactly.
struct Bulge {
    double s0 = 0, s1 = 1, t0 = 0, t1 = 1;
    int n = 0;                         // 0: flat
    std::vector<double> K, c;          // knots on [0,1] (n + 3); coefficients c[i + n * j]
    double operator()(double s, double t) const
    {
        if (n == 0) return 0.0;
        const double a = std::clamp((s - s0) / (s1 - s0), 0.0, 1.0), b = std::clamp((t - t0) / (t1 - t0), 0.0, 1.0);
        const std::vector<double> Na = basisAt(K, 3, n, a), Nb = basisAt(K, 3, n, b);
        double v = 0.0;
        for (int j = 0; j < n; ++j) {
            if (Nb[j] == 0.0) continue;
            for (int i = 0; i < n; ++i) v += c[size_t(i) + size_t(n) * j] * Na[i] * Nb[j];
        }
        return v;
    }
    double greville(int i) const { return 0.5 * (K[i + 1] + K[i + 2]); }
};

// Height A x bell(s) x bell(t) x sin(waves) x sin(waves), bell = sin(pi x), at
// the Greville points; the border coefficients are 0, so is the bulge there.
Bulge makeBulge(double A, double s0, double s1, double t0, double t1, int waves, double ph1, double ph2)
{
    Bulge b;
    b.s0 = s0; b.s1 = s1; b.t0 = t0; b.t1 = t1;
    if (!(A > 0.0) || !(s1 > s0) || !(t1 > t0)) return b;
    const int w = std::max(1, waves);
    b.n = 5 + 4 * w;
    b.K = openKnots(b.n, 3);
    b.c.assign(size_t(b.n) * b.n, 0.0);
    const double pi = kTwoPi / 2.0, k2 = kTwoPi * w;
    for (int j = 1; j + 1 < b.n; ++j)
        for (int i = 1; i + 1 < b.n; ++i) {
            const double gi = b.greville(i), gj = b.greville(j);
            b.c[size_t(i) + size_t(b.n) * j] =
                A * std::sin(pi * gi) * std::sin(pi * gj) * std::sin(k2 * gi + ph1) * std::sin(k2 * gj + ph2);
        }
    return b;
}

// The cut's surface in D: toD(s, t, height) is affine, so its control points
// are toD at the bulge's Greville points and coefficients (exact). Flat: the
// bilinear patch over the corners.
Trivariate::DSurface graphSurface(const Bulge &b, const std::function<V3(double, double, double)> &toD)
{
    Trivariate::DSurface d;
    const double S[2][2] = { { b.s0, b.s1 }, { b.t0, b.t1 } };
    if (b.n == 0) {
        for (int a = 0; a < 2; ++a) {
            d.order[a] = 2;
            d.n[a] = 2;
            d.knots[a] = { S[a][0], S[a][0], S[a][1], S[a][1] };
        }
        for (int j = 0; j < 2; ++j)
            for (int i = 0; i < 2; ++i) {
                const V3 p = toD(S[0][i], S[1][j], 0.0);
                d.ctrl << p[0] << p[1] << p[2];
            }
        return d;
    }
    for (int a = 0; a < 2; ++a) {
        d.order[a] = 3;
        d.n[a] = b.n;
        for (double k : b.K) d.knots[a] << S[a][0] + k * (S[a][1] - S[a][0]);
    }
    for (int j = 0; j < b.n; ++j)
        for (int i = 0; i < b.n; ++i) {
            const double s = b.s0 + b.greville(i) * (b.s1 - b.s0), t = b.t0 + b.greville(j) * (b.t1 - b.t0);
            const V3 p = toD(s, t, b.c[size_t(i) + size_t(b.n) * j]);
            d.ctrl << p[0] << p[1] << p[2];
        }
    return d;
}

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
    double bu[2] = { 0, 0 }, bt[2] = { 0, 0 };  // how far its low / high u and angle faces bulge in
    bool core = false;
    std::vector<std::pair<int, int>> cuts;       // (cut, side) - see CutInD::Piece
    std::vector<int> lab;                        // cutWhole: per vertex, its part
    int home = -1;                               // cutWhole: the part it is being cut in
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

// Outline loops from directed boundary edges. Where an outline touches
// itself at a vertex (a figure-eight), the walk can take either branch; when
// it comes back to a vertex already on its path, that stretch is closed off as
// its own loop - so any outline whose edges balance in and out at every vertex
// splits into simple loops. (Walking to the start only broke on the body.)
bool extractLoops(std::multimap<int, int> next, std::vector<std::vector<int>> *loops)
{
    loops->clear();
    while (!next.empty()) {
        std::vector<int> path{ next.begin()->first };
        std::unordered_map<int, size_t> at{ { path[0], 0 } };
        size_t guard = 0;
        while (guard++ < 10000000) {
            auto jt = next.find(path.back());
            if (jt == next.end()) return false;              // in and out do not balance here
            const int nx = jt->second;
            next.erase(jt);
            auto seen = at.find(nx);
            if (seen != at.end()) {
                std::vector<int> loop(path.begin() + seen->second, path.end());
                for (size_t k = seen->second + 1; k < path.size(); ++k) at.erase(path[k]);
                path.resize(seen->second + 1);
                if (loop.size() >= 3) loops->push_back(loop);
                if (path.size() == 1 && next.find(path[0]) == next.end()) break;
                continue;
            }
            at.emplace(nx, path.size());
            path.push_back(nx);
        }
    }
    return true;
}

int openEdgesOf(const struct PMesh &m);

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
    std::vector<int> glab = in.lab;               // (cutWhole) their parts
    const bool labs = !in.lab.empty();
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
        if (labs) glab.push_back(in.lab[a]);
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
    if (!extractLoops(next, &loops) || loops.empty()) {
        if (why) {
            std::unordered_map<int, int> deg;
            for (const auto &kv : next) { ++deg[kv.first]; --deg[kv.second]; }
            int bad = 0;
            QStringList ex;
            for (const auto &kv : deg)
                if (kv.second != 0) {
                    ++bad;
                    if (ex.size() < 4)
                        ex << QStringLiteral("v%1 (%2, out-in %3, f %4)").arg(kv.first)
                                  .arg(kv.first < n ? QStringLiteral("mesh vertex") : QStringLiteral("crossing"))
                                  .arg(kv.second).arg(kv.first < n ? fv[kv.first] : 0.0, 0, 'g', 3);
                }
            std::unordered_map<long long, int> ec3;
            int nonManifold = 0;
            for (const auto &tr : tLo) for (int k = 0; k < 3; ++k) ++ec3[edgeKey(tr[k], tr[(k + 1) % 3])];
            for (const auto &kv : ec3) if (kv.second > 2) ++nonManifold;
            *why = QStringLiteral("the cut outline is not a closed loop (%1 unbalanced vertices: %2; %3 edges used 3+ times)")
                       .arg(bad).arg(ex.join(QStringLiteral(", "))).arg(nonManifold);
        }
        return false;
    }

    // The cap polygon runs against the low side's boundary.
    std::vector<std::array<double, 2>> pts(gq.size());
    for (size_t i = 0; i < gq.size(); ++i) pts[i] = to2D(gq[i]);
    for (auto &l : loops) std::reverse(l.begin(), l.end());
    auto triangulate = [&](std::vector<std::vector<int>> loops, std::vector<std::array<int, 3>> *capOut) {
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
        bool stuck = false;
        for (const auto &O : outers) {
            const auto tris = earClip(pts, O, &stuck);
            capOut->insert(capOut->end(), tris.begin(), tris.end());
        }
        // A folded outline (the part's map into D is not one-to-one there) gives
        // overlapping cap triangles - a double-covered face, duplicate vertices.
        // Refuse the cut; the caller tries another position.
        // A folded outline gives overlapping cap triangles - a double-covered
        // face, duplicate vertices.
        return !stuck;
    };
    std::vector<std::array<int, 3>> cap;
    bool planeChart = false;
    if (!triangulate(loops, &cap)) {
        // The outline folds in the cut's own chart: the map into D is not
        // one-to-one there (a whole-model cut running on through another
        // part). Its best plane in the model instead, without interior points
        // on the cut surface.
        std::vector<int> ids;
        for (const auto &l : loops) ids.insert(ids.end(), l.begin(), l.end());
        V3 c = { 0, 0, 0 };
        for (int id : ids) c = add(c, gx[id]);
        c = mul(c, 1.0 / std::max<size_t>(1, ids.size()));
        double C[3][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };
        for (int id : ids) { const V3 d = sub(gx[id], c); for (int x = 0; x < 3; ++x) for (int y = 0; y < 3; ++y) C[x][y] += d[x] * d[y]; }
        // The normal: the direction of least spread (power iteration on the inverse is overkill: cross of the two largest).
        V3 e1 = { 1, 0, 0 };
        for (int it = 0; it < 50; ++it) {
            V3 nv = { C[0][0] * e1[0] + C[0][1] * e1[1] + C[0][2] * e1[2], C[1][0] * e1[0] + C[1][1] * e1[1] + C[1][2] * e1[2],
                      C[2][0] * e1[0] + C[2][1] * e1[1] + C[2][2] * e1[2] };
            const double l = len(nv);
            if (!(l > 0)) break;
            e1 = mul(nv, 1.0 / l);
        }
        const double l1 = dot(e1, V3{ C[0][0] * e1[0] + C[0][1] * e1[1] + C[0][2] * e1[2], C[1][0] * e1[0] + C[1][1] * e1[1] + C[1][2] * e1[2],
                                      C[2][0] * e1[0] + C[2][1] * e1[1] + C[2][2] * e1[2] });
        double D2[3][3];
        for (int x = 0; x < 3; ++x) for (int y = 0; y < 3; ++y) D2[x][y] = C[x][y] - l1 * e1[x] * e1[y];
        V3 e2 = std::fabs(e1[0]) < 0.9 ? V3{ 1, 0, 0 } : V3{ 0, 1, 0 };
        for (int it = 0; it < 50; ++it) {
            V3 nv = { D2[0][0] * e2[0] + D2[0][1] * e2[1] + D2[0][2] * e2[2], D2[1][0] * e2[0] + D2[1][1] * e2[1] + D2[1][2] * e2[2],
                      D2[2][0] * e2[0] + D2[2][1] * e2[1] + D2[2][2] * e2[2] };
            nv = sub(nv, mul(e1, dot(nv, e1)));
            const double l = len(nv);
            if (!(l > 0)) break;
            e2 = mul(nv, 1.0 / l);
        }
        std::vector<std::array<double, 2>> pp(gx.size());
        for (size_t i = 0; i < gx.size(); ++i) { const V3 d = sub(gx[i], c); pp[i] = { dot(d, e1), dot(d, e2) }; }
        // Keep the cap's orientation: the plane chart may mirror the cut's.
        auto signedArea = [](const std::vector<std::array<double, 2>> &P, const std::vector<int> &l) {
            double a = 0.0;
            for (size_t i = 0; i < l.size(); ++i) { const auto &p = P[l[i]], &q = P[l[(i + 1) % l.size()]]; a += p[0] * q[1] - q[0] * p[1]; }
            return a;
        };
        size_t big = 0;
        for (size_t i = 1; i < loops.size(); ++i) if (std::fabs(signedArea(pts, loops[i])) > std::fabs(signedArea(pts, loops[big]))) big = i;
        if ((signedArea(pts, loops[big]) > 0) != (signedArea(pp, loops[big]) > 0))
            for (auto &q : pp) q[1] = -q[1];
        cap.clear();
        pts.swap(pp);
        if (!triangulate(loops, &cap)) { if (why) *why = QStringLiteral("the cut outline folds over itself in D"); return false; }
        planeChart = true;
    }
    // Refine: big cap triangles get their centroid on the cut surface (the
    // outline's edges are never split, so the sides stay watertight).
    for (int pass = 0; pass < (planeChart ? 0 : 2); ++pass) {
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
            if (labs) glab.push_back(in.home);
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
            if (labs) m->lab.push_back(glab[g]);
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
    lo->cuts = hi->cuts = in.cuts;
    for (int k = 0; k < 2; ++k) {
        lo->bu[k] = hi->bu[k] = in.bu[k];
        lo->bt[k] = hi->bt[k] = in.bt[k];
    }
    build(tLo, false, lo);
    build(tHi, true, hi);
    // Both sides must be closed; a cut that is not is refused (a broken piece
    // must never be passed on to the next cut).
    if (openEdgesOf(*lo) || openEdgesOf(*hi)) { if (why) *why = QStringLiteral("the cut left a side open"); return false; }
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
    if (!extractLoops(next, &loops)) { if (why) *why = QStringLiteral("the core outline is not closed"); return false; }
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
    // An outline keeps its own order (sorting its points by angle broke it
    // wherever the angle does not grow steadily round it - the tube then left
    // gaps along the outline): it is only turned to run the way the angle
    // grows and started at its smallest angle.
    auto byAngle = [&](std::vector<int> l) {
        double turn = 0.0;
        for (size_t i = 0; i < l.size(); ++i) {
            double d = angle(l[(i + 1) % l.size()]) - angle(l[i]);
            if (d > kTwoPi / 2) d -= kTwoPi;
            if (d < -kTwoPi / 2) d += kTwoPi;
            turn += d;
        }
        if (turn < 0) std::reverse(l.begin(), l.end());
        size_t s0 = 0;
        for (size_t i = 1; i < l.size(); ++i) if (angle(l[i]) < angle(l[s0])) s0 = i;
        std::rotate(l.begin(), l.begin() + s0, l.end());
        return l;
    };
    // Position along a ring, 0..1: by length round the outline (not by angle,
    // which need not grow steadily).
    auto params = [&](const std::vector<int> &l) {
        std::vector<double> t(l.size() + 1, 0.0);
        for (size_t i = 1; i <= l.size(); ++i) {
            const V3 &a = gq[l[i - 1]], &b = gq[l[i % l.size()]];
            t[i] = t[i - 1] + std::hypot(b[1] - a[1], b[2] - a[2]);
        }
        for (double &x : t) x /= std::max(1e-30, t.back());
        return t;
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
    // Zip neighbouring rings: walk both by position round them, always
    // advancing the one whose next point comes first.
    std::vector<std::array<int, 3>> tube;
    for (size_t k = 0; k + 1 < rings.size(); ++k) {
        const auto &A = rings[k], &Bv = rings[k + 1];
        const std::vector<double> ta = params(A), tb = params(Bv);
        size_t i = 0, j = 0;
        const size_t na = A.size(), nb = Bv.size();
        while (i < na || j < nb) {
            const double ai = i < na ? ta[i + 1] : 1e300;
            const double bj = j < nb ? tb[j + 1] : 1e300;
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
    if (openEdgesOf(*core) || openEdgesOf(*shell)) { if (why) *why = QStringLiteral("the core cut left a side open"); return false; }
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
                           double coreRadius, double bend, int waves)
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
            // w = 1 - rc over all of (u, v); dS/ds x dS/dt = +w, towards the axis.
            Cut ct;
            ct.kind = QStringLiteral("core");
            Trivariate::DSurface d = graphSurface(makeBulge(0.0, 0.0, 1.0, 0.0, 1.0, 1, 0.0, 0.0),
                                                  [coreRadius](double s, double t, double) { return V3{ s, t, 1.0 - coreRadius }; });
            d.iso = 2;
            ct.surfaces << d;
            R.cuts << ct;
            core.cuts.push_back({ int(R.cuts.size()) - 1, +1 });
            shell.cuts.push_back({ int(R.cuts.size()) - 1, -1 });
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
        // The plane is turned by a small angle (and retried at others): at
        // exactly v = 0 vertices sat on it and the outline broke.
        PMesh a, b;
        bool ok = false;
        double th = 0.0;
        for (int attempt = 0; attempt < 6 && !ok; ++attempt) {
            th = 0.0123 + 0.071 * attempt;
            const double ct = std::cos(th), st = std::sin(th);
            ok = cutMesh(shellAll, B, [ct, st](const V3 &q) { return q[2] * ct - q[1] * st; },
                         [ct, st](const V3 &q) { return std::array<double, 2>{ q[0], q[1] * ct + q[2] * st }; },
                         [ct, st](double x, double y) { return V3{ x, y * ct, y * st }; }, refine, &a, &b, &why);
        }
        if (!ok) { R.error = QStringLiteral("the first cut through the axis failed: %1").arg(why); return R; }
        a.th0 = th + 0.5 * kTwoPi; a.th1 = th + kTwoPi;   // behind the plane: v in (th/2pi + 1/2, th/2pi + 1)
        b.th0 = th; b.th1 = th + 0.5 * kTwoPi;
        // Its two half-planes, v = th/2pi and v = th/2pi + 1/2, over u and the
        // shell's w; dS/ds x dS/dt = -v. a is below the first (round the seam
        // of v) and above the second, b the other way round.
        const double wTop = haveCore ? 1.0 - coreRadius : 1.0;
        for (int h = 0; h < 2; ++h) {
            const double vh = th / kTwoPi + 0.5 * h;
            Cut ct;
            ct.kind = h == 0 ? QStringLiteral("seam") : QStringLiteral("seam (far half)");
            Trivariate::DSurface d = graphSurface(makeBulge(0.0, 0.0, 1.0, 0.0, wTop, 1, 0.0, 0.0),
                                                  [vh](double s, double t, double) { return V3{ s, vh, t }; });
            d.iso = 1;
            ct.surfaces << d;
            R.cuts << ct;
            a.cuts.push_back({ int(R.cuts.size()) - 1, h == 0 ? +1 : -1 });
            b.cuts.push_back({ int(R.cuts.size()) - 1, h == 0 ? -1 : +1 });
        }
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
            // Where the piece reaches, round the axis and out from it (at most
            // the block's face, w = 0: the cut's surface stays inside D).
            double rLo = cur.core ? 0.0 : cur.r0, rHi = cur.core ? cur.r1 : 0.0;
            if (!cur.core) for (const V3 &q : cur.q) rHi = std::max(rHi, std::hypot(q[1], q[2]));
            rHi = std::min(rHi, 1.0);
            if (!(rHi > rLo + 1e-9)) rHi = std::min(1.0, rLo + 0.5);
            const double th0 = cur.th0, th1 = cur.th1, u0 = cur.u0, u1 = cur.u1;
            const double ph1 = kTwoPi * (0.5 + jit(rng)), ph2 = kTwoPi * (0.5 - jit(rng));
            auto angleIn = [th0](double y, double z) {
                double d = std::fmod(std::atan2(z, y) - th0, kTwoPi);
                if (d < 0) d += kTwoPi;
                return th0 + d;
            };
            if (u) {
                // The part's actual u range in this piece.
                double umin = 1e300, umax = -1e300;
                for (const V3 &q : cur.q) { umin = std::min(umin, q[0]); umax = std::max(umax, q[0]); }
                umin = std::max(umin, cur.u0); umax = std::min(umax, cur.u1);
                const double c = umin + (0.5 + jit(rng)) * (umax - umin);
                // The bulge, over (angle, radius) across the piece.
                const double A = bend * std::max(0.0, std::min(c - u0 - cur.bu[0], u1 - c - cur.bu[1]));
                const Bulge bulge = makeBulge(A, th0, th1, rLo, rHi, waves, ph1, ph2);
                ok = cutMesh(cur, B,
                             [&](const V3 &q) { return q[0] - c - bulge(angleIn(q[1], q[2]), std::hypot(q[1], q[2])); },
                             [](const V3 &q) { return std::array<double, 2>{ q[1], q[2] }; },
                             [&](double x, double y) { return V3{ c + bulge(angleIn(x, y), std::hypot(x, y)), x, y }; },
                             refine, &a, &b, &why);
                if (ok) {
                    a.u1 = c; a.bu[1] = A; b.u0 = c; b.bu[0] = A;
                    // In D: u = c + bulge(2pi v, 1 - w); dS/ds x dS/dt = -u, so the
                    // low side (a) is +1.
                    Cut ct;
                    ct.kind = QStringLiteral("u");
                    Trivariate::DSurface d = graphSurface(bulge, [c](double s, double t, double h) {
                        return V3{ c + h, s / kTwoPi, 1.0 - t };
                    });
                    if (bulge.n == 0) d.iso = 0;
                    if (th0 >= kTwoPi) for (int m = 1; m < d.ctrl.size(); m += 3) d.ctrl[m] -= 1.0;
                    else if (th1 > kTwoPi) d.vWrapAt = kTwoPi;
                    ct.surfaces << d;
                    R.cuts << ct;
                    a.cuts.push_back({ int(R.cuts.size()) - 1, +1 });
                    b.cuts.push_back({ int(R.cuts.size()) - 1, -1 });
                }
            } else {
                const double th = cur.th0 + (0.5 + jit(rng)) * (cur.th1 - cur.th0);
                // The cut's angle bends over (u, radius) across the piece - never
                // across v's seam (2 pi), where its surface in D would break.
                double Bn = bend * std::max(0.0, std::min(th - th0 - cur.bt[0], th1 - th - cur.bt[1]));
                if (th0 < kTwoPi && th1 > kTwoPi) Bn = std::min(Bn, 0.9 * std::fabs(th - kTwoPi));
                const Bulge bulge = makeBulge(Bn, u0, u1, rLo, rHi, waves, ph1, ph2);
                auto thc = [&](double uu, double r) { return th + bulge(uu, r); };
                ok = cutMesh(cur, B,
                             [&](const V3 &q) {
                                 const double t = thc(q[0], std::hypot(q[1], q[2]));
                                 return q[2] * std::cos(t) - q[1] * std::sin(t);
                             },
                             [](const V3 &q) { return std::array<double, 2>{ q[0], std::hypot(q[1], q[2]) }; },
                             [&](double x, double r) { const double t = thc(x, r); return V3{ x, r * std::cos(t), r * std::sin(t) }; },
                             refine, &a, &b, &why);
                if (ok) {
                    a.th1 = th; a.bt[1] = Bn; b.th0 = th; b.bt[0] = Bn;
                    // In D: v = (th + bulge(u, 1 - w)) / 2pi; dS/ds x dS/dt = +v, so
                    // the low side (a) is -1.
                    const double shift = th >= kTwoPi ? 1.0 : 0.0;
                    Cut ct;
                    ct.kind = QStringLiteral("angle");
                    Trivariate::DSurface d = graphSurface(bulge, [th, shift](double s, double t, double h) {
                        return V3{ s, (th + h) / kTwoPi - shift, 1.0 - t };
                    });
                    if (bulge.n == 0) d.iso = 1;
                    ct.surfaces << d;
                    R.cuts << ct;
                    a.cuts.push_back({ int(R.cuts.size()) - 1, -1 });
                    b.cuts.push_back({ int(R.cuts.size()) - 1, +1 });
                }
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
        // Its cell in D, widened by its faces' bulges so the cell holds it.
        p.lo[0] = std::clamp(m.u0 - m.bu[0], 0.0, 1.0); p.hi[0] = std::clamp(m.u1 + m.bu[1], 0.0, 1.0);
        if (m.core) {
            p.lo[1] = 0.0;
            p.hi[1] = 1.0;
        } else {
            // v closes: a cell running past 1 goes on from 0 (vWrap).
            double v0 = std::max(0.0, (m.th0 - m.bt[0]) / kTwoPi), v1 = (m.th1 + m.bt[1]) / kTwoPi;
            if (v0 >= 1.0) { v0 -= 1.0; v1 -= 1.0; }
            p.lo[1] = v0;
            p.hi[1] = std::min(v1, 1.0);
            if (v1 > 1.0) p.vWrap = std::min(v1 - 1.0, v0);
        }
        for (const auto &cr : m.cuts) p.cuts.append(qMakePair(cr.first, cr.second));
        p.lo[2] = m.core ? 1.0 - coreRadius : 0.0;
        p.hi[2] = m.core ? 1.0 : (haveCore ? 1.0 - coreRadius : 1.0);
        p.core = m.core;
        R.piecesVolume += m.volume();
        R.openEdges += openEdgesOf(m);
        R.pieces.append(std::move(p));
    }
    R.notes << QStringLiteral("%1 piece(s)%2%5; their volume %3% of the part's; %4 open edge(s)")
                   .arg(R.pieces.size()).arg(haveCore ? QStringLiteral(" (1 core)") : QString())
                   .arg(R.partVolume > 0 ? 100.0 * R.piecesVolume / R.partVolume : 0.0, 0, 'f', 3)
                   .arg(R.openEdges)
                   .arg(bend > 0 ? QStringLiteral(", curved cuts (bend %1, %2 wave(s))").arg(bend, 0, 'f', 2).arg(std::max(1, waves))
                                 : QString());
    R.ok = true;
    return R;
}

// ================================================================== the whole model
namespace {

// Connected pieces of a mesh (through shared vertices).
int componentsOf(const PMesh &m)
{
    std::vector<int> par(m.x.size());
    for (size_t i = 0; i < par.size(); ++i) par[i] = int(i);
    auto find = [&](int x) { while (par[x] != x) x = par[x] = par[par[x]]; return x; };
    std::vector<char> used(m.x.size(), 0);
    for (const auto &t : m.t)
        for (int k = 0; k < 3; ++k) {
            used[t[k]] = 1;
            const int a = find(t[k]), b = find(t[(k + 1) % 3]);
            if (a != b) par[a] = b;
        }
    int c = 0;
    for (size_t i = 0; i < par.size(); ++i) if (used[i] && find(int(i)) == int(i)) ++c;
    return c;
}

// Angles round the axis that a set of points covers: [a0, a1] (a1 may pass
// 2 pi) - the complement of the widest empty gap; full = no gap wider than 60 deg.
void angularExtent(std::vector<double> ang, double *a0, double *a1, bool *full)
{
    std::sort(ang.begin(), ang.end());
    *full = true; *a0 = 0.0; *a1 = kTwoPi;
    if (ang.size() < 2) return;
    double gap = -1.0;
    size_t at = 0;
    for (size_t i = 0; i < ang.size(); ++i) {
        const double nx = i + 1 < ang.size() ? ang[i + 1] : ang[0] + kTwoPi;
        if (nx - ang[i] > gap) { gap = nx - ang[i]; at = i; }
    }
    if (gap < kTwoPi / 6.0) return;
    *full = false;
    *a0 = at + 1 < ang.size() ? ang[at + 1] : ang[0];
    *a1 = ang[at];
    if (*a1 < *a0) *a1 += kTwoPi;
}

}  // namespace

CutInD::Result CutInD::cutWhole(const MeshData &model, const QVector<int> &vertexPart,
                                const QVector<HarmonicFit::Result> &fits, int pieces, quint32 seed,
                                double bend, int waves)
{
    Result R;
    const int P = fits.size();
    std::vector<std::unique_ptr<Block>> blocks(size_t(std::max(0, P)));
    int nb = 0;
    for (int p = 0; p < P; ++p)
        if (fits[p].ok) { blocks[size_t(p)] = std::make_unique<Block>(fits[p]); ++nb; }
    if (nb == 0 || model.triangleCount() == 0) { R.error = QStringLiteral("no trivariate or no model"); return R; }
    double size = 0.0;
    for (int a = 0; a < 3; ++a) size += double(model.bmax[a] - model.bmin[a]) * double(model.bmax[a] - model.bmin[a]);
    size = std::sqrt(size);

    // The model, welded; every vertex knows its part.
    PMesh root;
    {
        std::unordered_map<std::string, int> weldMap;
        std::vector<int> remap(model.vertexCount());
        for (int i = 0; i < model.vertexCount(); ++i) {
            const std::string key = posKey(model.pos[3 * i], model.pos[3 * i + 1], model.pos[3 * i + 2]);
            auto it = weldMap.find(key);
            if (it == weldMap.end()) {
                remap[i] = int(root.x.size());
                weldMap.emplace(key, remap[i]);
                root.x.push_back({ model.pos[3 * i], model.pos[3 * i + 1], model.pos[3 * i + 2] });
                const int l = i < vertexPart.size() ? vertexPart[i] : -1;
                root.lab.push_back(l >= 0 && l < P && blocks[size_t(l)] ? l : -1);
            } else remap[i] = it->second;
        }
        for (int t = 0; t < model.triangleCount(); ++t) {
            const int a = remap[model.tris[3 * t]], b = remap[model.tris[3 * t + 1]], c = remap[model.tris[3 * t + 2]];
            if (a != b && b != c && a != c) root.t.push_back({ a, b, c });
        }
        // A vertex without a part (or whose part has no trivariate) takes a
        // neighbour's.
        for (int pass = 0; pass < 50; ++pass) {
            bool changed = false;
            for (const auto &t : root.t)
                for (int k = 0; k < 3; ++k)
                    if (root.lab[t[k]] < 0)
                        for (int j = 1; j < 3; ++j)
                            if (root.lab[t[(k + j) % 3]] >= 0) { root.lab[t[k]] = root.lab[t[(k + j) % 3]]; changed = true; break; }
            if (!changed) break;
        }
        root.q.assign(root.x.size(), V3{ 0, 0, 0 });
    }
    R.partVolume = root.volume();

    auto homeOf = [&](const PMesh &m) {
        std::vector<double> a(size_t(P), 0.0);
        for (const auto &t : m.t) {
            const double ar = len(cross(sub(m.x[t[1]], m.x[t[0]]), sub(m.x[t[2]], m.x[t[0]])));
            for (int k = 0; k < 3; ++k) { const int l = m.lab[t[k]]; if (l >= 0) a[size_t(l)] += ar; }
        }
        std::vector<int> ls;
        for (int l = 0; l < P; ++l) if (blocks[size_t(l)] && a[size_t(l)] > 0) ls.push_back(l);
        std::sort(ls.begin(), ls.end(), [&](int x, int y) { return a[size_t(x)] > a[size_t(y)]; });
        return ls;                                   // the parts it reaches, most first
    };
    auto mapInto = [&](const std::vector<V3> &xs, int b) {
        MeshData md;
        for (const V3 &x : xs) md.addVertex(float(x[0]), float(x[1]), float(x[2]));
        int o = 0, u = 0;
        return mapToQ(*blocks[size_t(b)], md, size, &o, &u);
    };

    // The model's volume per surface area: the thickness scale for (b).
    double refVA = 0.0;
    {
        double ar = 0.0;
        for (const auto &t : root.t) ar += 0.5 * len(cross(sub(root.x[t[1]], root.x[t[0]]), sub(root.x[t[2]], root.x[t[0]])));
        refVA = root.volume() / std::max(1e-30, ar);
    }
    int limbCuts = 0;
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> jit(-0.15, 0.15);
    std::vector<PMesh> todo{ root }, done;
    int failures = 0;
    const double refine = 0.12;
    QMap<int, int> cutsPerPart;
    while (int(todo.size() + done.size()) < pieces && !todo.empty() && failures < 30) {
        size_t bi = 0;
        for (size_t i = 1; i < todo.size(); ++i) if (todo[i].volume() > todo[bi].volume()) bi = i;
        PMesh cur = std::move(todo[bi]);
        todo.erase(todo.begin() + long(bi));
        // Candidates in every part the piece reaches (a)... the part it mostly
        // is (across it, round its axis), and every limb it holds: a clean
        // cross-section of that limb in the limb's own trivariate. A body cut
        // running on down a leg through the extended map wrapped the leg in a
        // thin shell; the leg's own u = c cuts straight across it.
        const std::vector<int> homes = homeOf(cur);
        const int n = int(cur.x.size());
        std::vector<double> share(size_t(P), 0.0);
        double areaAll = 0.0;
        for (const auto &t : cur.t) {
            const double ar = len(cross(sub(cur.x[t[1]], cur.x[t[0]]), sub(cur.x[t[2]], cur.x[t[0]])));
            areaAll += ar;
            for (int k = 0; k < 3; ++k) { const int l = cur.lab[t[k]]; if (l >= 0) share[size_t(l)] += ar / 3.0; }
        }
        struct Ctx {
            int part = -1;
            std::vector<V3> q;
            double umin = 0, umax = 0, rLo = 0, rHi = 1, a0 = 0, a1 = kTwoPi;
            bool full = true, main = false;
        };
        std::vector<Ctx> ctx;
        for (size_t h = 0; h < homes.size() && ctx.size() < 5; ++h) {
            if (h > 0 && share[size_t(homes[h])] < 0.02 * areaAll) continue;   // a sliver of a part
            Ctx c;
            c.part = homes[h];
            c.main = ctx.empty();
            // Its own vertices mapped into its D; every other vertex takes the
            // coordinates of the nearest of them (over the mesh) - where it is
            // attached. Far from a part its extended map means nothing (a leg
            // cut also sliced the body); this way the rest of the piece stays
            // on one side and the cut stays inside the part, where it is exact.
            {
                std::vector<V3> own;
                std::vector<int> ownIdx(static_cast<size_t>(n), -1);
                for (int i = 0; i < n; ++i) if (cur.lab[i] == c.part) { ownIdx[size_t(i)] = int(own.size()); own.push_back(cur.x[i]); }
                if (own.size() < 4) continue;
                const std::vector<V3> Qown = mapInto(own, c.part);
                std::vector<std::vector<int>> adj(static_cast<size_t>(n));
                for (const auto &t : cur.t)
                    for (int k = 0; k < 3; ++k) { adj[size_t(t[k])].push_back(t[(k + 1) % 3]); adj[size_t(t[(k + 1) % 3])].push_back(t[k]); }
                // Each connected region of other parts (a whole leg, the head
                // with its horns) is ONE point: the mean of where it attaches.
                // It goes to one side of any cut whole - never split along
                // its length by a cut that touches its joint.
                c.q.assign(static_cast<size_t>(n), V3{ 0, 0, 0 });
                for (int i = 0; i < n; ++i) if (ownIdx[size_t(i)] >= 0) c.q[size_t(i)] = Qown[size_t(ownIdx[size_t(i)])];
                std::vector<int> comp(static_cast<size_t>(n), -1);
                for (int s0 = 0; s0 < n; ++s0) {
                    if (ownIdx[size_t(s0)] >= 0 || comp[size_t(s0)] >= 0) continue;
                    std::vector<int> members{ s0 };
                    comp[size_t(s0)] = s0;
                    V3 anchor = { 0, 0, 0 };
                    int na = 0;
                    for (size_t h2 = 0; h2 < members.size(); ++h2)
                        for (int nb : adj[size_t(members[h2])]) {
                            if (ownIdx[size_t(nb)] >= 0) { anchor = add(anchor, Qown[size_t(ownIdx[size_t(nb)])]); ++na; continue; }
                            if (comp[size_t(nb)] < 0) { comp[size_t(nb)] = s0; members.push_back(nb); }
                        }
                    if (na > 0) anchor = mul(anchor, 1.0 / na);
                    for (int m2 : members) c.q[size_t(m2)] = anchor;
                }
            }
            double umin = 1e300, umax = -1e300, rmin = 1e300, rmax = -1e300;
            std::vector<double> ang;
            for (int i = 0; i < n; ++i) {
                if (cur.lab[i] != c.part) continue;
                const V3 &q = c.q[size_t(i)];
                const double r = std::hypot(q[1], q[2]);
                umin = std::min(umin, q[0]); umax = std::max(umax, q[0]);
                rmin = std::min(rmin, r); rmax = std::max(rmax, r);
                if (r > 0.05) { double aa = std::atan2(q[2], q[1]); if (aa < 0) aa += kTwoPi; ang.push_back(aa); }
            }
            if (!(umax > umin)) continue;
            c.umin = std::max(umin, 0.0); c.umax = std::min(umax, 1.0);
            c.rLo = std::max(0.0, rmin); c.rHi = std::min(1.0, std::max(rmax, c.rLo + 0.05));
            angularExtent(ang, &c.a0, &c.a1, &c.full);
            ctx.push_back(std::move(c));
        }

        // kind 0 = across the part (u = c); 1 = a plane through its axis (the
        // piece goes all round it); 2 = round the axis inside the piece's sector.
        struct Cand { int ci; int kind; double c; double score; };
        std::vector<Cand> cand;
        for (int ci = 0; ci < int(ctx.size()); ++ci) {
            const Ctx &c = ctx[size_t(ci)];
            if (c.main) {
                for (int i = 1; i <= 7; ++i) cand.push_back({ ci, 0, c.umin + (c.umax - c.umin) * i / 8.0, 0.0 });
                if (c.full) for (int i = 0; i < 6; ++i) cand.push_back({ ci, 1, 0.0123 + i * kTwoPi / 12.0, 0.0 });
                else for (int i = 1; i <= 5; ++i) cand.push_back({ ci, 2, c.a0 + (c.a1 - c.a0) * i / 6.0, 0.0 });
            } else {
                for (int i = 1; i <= 4; ++i) cand.push_back({ ci, 0, c.umin + (c.umax - c.umin) * i / 5.0, 0.0 });
            }
        }
        auto inSectorOf = [](const Ctx &c, double y, double z) {
            double d = std::fmod(std::atan2(z, y) - c.a0, kTwoPi);
            if (d < 0) d += kTwoPi;
            return c.a0 + d;
        };
        auto flatField = [&](const Cand &cd, const V3 &q) {
            const Ctx &c = ctx[size_t(cd.ci)];
            if (cd.kind == 0) return q[0] - cd.c;
            if (cd.kind == 1) return q[2] * std::cos(cd.c) - q[1] * std::sin(cd.c);
            return inSectorOf(c, q[1], q[2]) - cd.c;
        };
        for (Cand &cd : cand) {
            const Ctx &c = ctx[size_t(cd.ci)];
            std::vector<double> f(static_cast<size_t>(n), 0.0);
            for (int i = 0; i < n; ++i) f[size_t(i)] = flatField(cd, c.q[size_t(i)]);
            // Volumes by side, each closed by a flat fill through O (the middle
            // of the crossings); surface areas by side.
            V3 O = { 0, 0, 0 };
            int nO = 0, nCross = 0, foreign = 0;
            for (const auto &t : cur.t)
                for (int k = 0; k < 3; ++k) {
                    const int x = t[k], y = t[(k + 1) % 3];
                    if (x > y || (f[size_t(x)] < 0) == (f[size_t(y)] < 0)) continue;
                    const double tt = f[size_t(x)] / (f[size_t(x)] - f[size_t(y)]);
                    O = add(O, add(cur.x[x], mul(sub(cur.x[y], cur.x[x]), tt)));
                    ++nO;
                    ++nCross;
                    if (cur.lab[x] != c.part || cur.lab[y] != c.part) ++foreign;
                }
            if (nO == 0) { cd.score = 1e9; continue; }
            O = mul(O, 1.0 / nO);
            double lo = 0.0, hi = 0.0, alo = 0.0, ahi = 0.0;
            for (const auto &t : cur.t) {
                const int neg = (f[size_t(t[0])] < 0) + (f[size_t(t[1])] < 0) + (f[size_t(t[2])] < 0);
                const double v6 = dot(sub(cur.x[t[0]], O), cross(sub(cur.x[t[1]], O), sub(cur.x[t[2]], O)));
                const double ar = 0.5 * len(cross(sub(cur.x[t[1]], cur.x[t[0]]), sub(cur.x[t[2]], cur.x[t[0]])));
                if (neg >= 2) { lo += v6; alo += ar; } else { hi += v6; ahi += ar; }
            }
            lo = std::fabs(lo) / 6.0; hi = std::fabs(hi) / 6.0;
            const double vsum = lo + hi;
            if (!(vsum > 0) || std::min(lo, hi) < 0.08 * vsum) { cd.score = 1e9; continue; }
            // (b) Thickness: a side's volume per surface area against the
            // model's. A shell round a leg is a few percent of it, a leg piece
            // a third or more.
            const double thin = std::min(lo / std::max(1e-30, alo), hi / std::max(1e-30, ahi)) / refVA;
            const double thinPen = 6.0 * std::max(0.0, 0.25 - thin) / 0.25;
            cd.score = std::fabs(lo - hi) / vsum + 4.0 * (nCross ? double(foreign) / nCross : 0.0) + thinPen
                       + (c.main ? 0.0 : 0.15) + 0.05 * jit(rng);
        }
        std::sort(cand.begin(), cand.end(), [](const Cand &x, const Cand &y) { return x.score < y.score; });

        PMesh a, b;
        bool ok = false;
        QString why;
        int tried = 0;
        for (const Cand &cd : cand) {
            if (cd.score >= 1e8 || tried >= 12) break;
            ++tried;
            const Ctx &cx = ctx[size_t(cd.ci)];
            const int hb = cx.part;
            cur.q = cx.q;
            cur.home = hb;
            const Block &B = *blocks[size_t(hb)];
            const double umin = cx.umin, umax = cx.umax, rLo = cx.rLo, rHi = cx.rHi, a0 = cx.a0, a1 = cx.a1;
            const bool full = cx.full;
            auto inSector = [&](double y, double z) { return inSectorOf(cx, y, z); };
            const double ph1 = kTwoPi * (0.5 + jit(rng)), ph2 = kTwoPi * (0.5 - jit(rng));
            Cut ct;
            ct.part = hb;
            int sideA = 0, sideB = 0;
            if (cd.kind == 0) {
                const double c = std::clamp(cd.c, 0.0, 1.0);
                const double s0 = full ? 0.0 : a0, s1 = full ? kTwoPi : a1;
                const Bulge bulge = makeBulge(bend * 0.5 * std::min(c - umin, umax - c), s0, s1, rLo, rHi, waves, ph1, ph2);
                auto angOf = [&](double y, double z) {
                    if (!full) return inSector(y, z);
                    double t = std::atan2(z, y);
                    return t < 0 ? t + kTwoPi : t;
                };
                ok = cutMesh(cur, B,
                             [&](const V3 &q) { return q[0] - c - bulge(angOf(q[1], q[2]), std::hypot(q[1], q[2])); },
                             [](const V3 &q) { return std::array<double, 2>{ q[1], q[2] }; },
                             [&](double x, double y) { return V3{ c + bulge(angOf(x, y), std::hypot(x, y)), x, y }; },
                             refine, &a, &b, &why);
                if (ok) {
                    ct.kind = cx.main ? QStringLiteral("u") : QStringLiteral("limb cross-section");
                    Trivariate::DSurface d = graphSurface(bulge, [c](double s, double t, double h) {
                        return V3{ c + h, s / kTwoPi, 1.0 - t };
                    });
                    if (bulge.n == 0) d.iso = 0;
                    if (s0 >= kTwoPi) for (int m = 1; m < d.ctrl.size(); m += 3) d.ctrl[m] -= 1.0;
                    else if (s1 > kTwoPi) d.vWrapAt = kTwoPi;
                    ct.surfaces << d;
                    sideA = +1; sideB = -1;
                }
            } else if (cd.kind == 1) {
                const double th = cd.c, ctt = std::cos(th), stt = std::sin(th);
                ok = cutMesh(cur, B, [ctt, stt](const V3 &q) { return q[2] * ctt - q[1] * stt; },
                             [ctt, stt](const V3 &q) { return std::array<double, 2>{ q[0], q[1] * ctt + q[2] * stt }; },
                             [ctt, stt](double x, double y) { return V3{ x, y * ctt, y * stt }; }, refine, &a, &b, &why);
                if (ok) ct.kind = QStringLiteral("plane");
            } else {
                const double th = cd.c;
                double Bn = bend * 0.5 * std::min(th - a0, a1 - th);
                if (th - Bn < kTwoPi && th + Bn > kTwoPi) Bn = std::min(Bn, 0.9 * std::fabs(th - kTwoPi));
                const Bulge bulge = makeBulge(Bn, umin, umax, rLo, rHi, waves, ph1, ph2);
                auto thc = [&](double uu, double r) { return th + bulge(uu, r); };
                ok = cutMesh(cur, B, [&](const V3 &q) { return inSector(q[1], q[2]) - thc(q[0], std::hypot(q[1], q[2])); },
                             [](const V3 &q) { return std::array<double, 2>{ q[0], std::hypot(q[1], q[2]) }; },
                             [&](double x, double r) { const double t = thc(x, r); return V3{ x, r * std::cos(t), r * std::sin(t) }; },
                             refine, &a, &b, &why);
                if (ok) {
                    ct.kind = QStringLiteral("angle");
                    const double shift = th >= kTwoPi ? 1.0 : 0.0;
                    Trivariate::DSurface d = graphSurface(bulge, [th, shift](double s, double t, double h) {
                        return V3{ s, (th + h) / kTwoPi - shift, 1.0 - t };
                    });
                    if (bulge.n == 0) d.iso = 1;
                    ct.surfaces << d;
                    sideA = -1; sideB = +1;
                }
            }
            // Both sides one piece each, and neither a thin shell (checked on
            // the real pieces, curved cut and all).
            if (ok && (componentsOf(a) != 1 || componentsOf(b) != 1)) { ok = false; why = QStringLiteral("a side came apart"); }
            if (ok) {
                auto areaOf = [](const PMesh &m) {
                    double s2 = 0.0;
                    for (const auto &t : m.t) s2 += 0.5 * len(cross(sub(m.x[t[1]], m.x[t[0]]), sub(m.x[t[2]], m.x[t[0]])));
                    return s2;
                };
                const double ta = a.volume() / std::max(1e-30, areaOf(a)) / refVA, tb = b.volume() / std::max(1e-30, areaOf(b)) / refVA;
                if (std::min(ta, tb) < 0.12) { ok = false; why = QStringLiteral("a side is a thin shell"); }
            }
            if (!ok) {
                if (qEnvironmentVariableIsSet("CUT_DEBUG"))
                    fprintf(stderr, "CUTW  piece %d verts, part %d%s, cand kind %d at %.3f score %.3f: %s\n", n, hb, cx.main ? "" : " (limb)",
                            cd.kind, cd.c, cd.score, why.toUtf8().constData());
                continue;
            }
            if (cd.kind == 1) {
                // The plane through the axis: its two half-planes (as the seam).
                for (int h = 0; h < 2; ++h) {
                    const double vh = cd.c / kTwoPi + 0.5 * h;
                    Cut half;
                    half.part = hb;
                    half.kind = QStringLiteral("plane");
                    Trivariate::DSurface d = graphSurface(makeBulge(0.0, umin, umax, 1.0 - rHi, 1.0 - rLo, 1, 0.0, 0.0),
                                                          [vh](double s2, double t, double) { return V3{ s2, vh, t }; });
                    d.iso = 1;
                    half.surfaces << d;
                    R.cuts << half;
                    a.cuts.push_back({ int(R.cuts.size()) - 1, h == 0 ? +1 : -1 });
                    b.cuts.push_back({ int(R.cuts.size()) - 1, h == 0 ? -1 : +1 });
                }
            } else {
                R.cuts << ct;
                a.cuts.push_back({ int(R.cuts.size()) - 1, sideA });
                b.cuts.push_back({ int(R.cuts.size()) - 1, sideB });
            }
            ++cutsPerPart[hb];
            if (!cx.main) ++limbCuts;
            break;
        }
        if (!ok) {
            ++failures;
            R.notes << QStringLiteral("a piece could not be cut (%1) - left whole").arg(why);
            done.push_back(std::move(cur));
            continue;
        }
        todo.push_back(std::move(a));
        todo.push_back(std::move(b));
    }
    for (auto &m : todo) done.push_back(std::move(m));

    // Out: meshes, and per piece its cell in each part's trivariate it reaches.
    for (const PMesh &m : done) {
        Piece pc;
        for (const V3 &x : m.x) pc.mesh.addVertex(x[0], x[1], x[2]);
        for (const auto &tr : m.t) {
            pc.mesh.tris.push_back(uint32_t(tr[0]));
            pc.mesh.tris.push_back(uint32_t(tr[1]));
            pc.mesh.tris.push_back(uint32_t(tr[2]));
        }
        pc.mesh.computeBounds();
        pc.mesh.computeNormals();
        for (int l = 0; l < P; ++l) {
            if (!blocks[size_t(l)]) continue;
            std::vector<V3> xs;
            for (size_t i = 0; i < m.x.size(); ++i) if (m.lab[i] == l) xs.push_back(m.x[i]);
            if (xs.size() < 3) continue;
            const std::vector<V3> Q = mapInto(xs, l);
            PartCell cell;
            cell.part = l;
            double lo[3] = { 1e300, 1e300, 1e300 }, hi[3] = { -1e300, -1e300, -1e300 };
            std::vector<double> ang;
            for (const V3 &q : Q) {
                double u, v, w;
                fromQ(q, &u, &v, &w);
                lo[0] = std::min(lo[0], u); hi[0] = std::max(hi[0], u);
                lo[2] = std::min(lo[2], w); hi[2] = std::max(hi[2], w);
                if (std::hypot(q[1], q[2]) > 0.05) ang.push_back(v * kTwoPi);
            }
            double c0, c1;
            bool fullV;
            angularExtent(ang, &c0, &c1, &fullV);
            const double m2 = 0.02;
            cell.lo[0] = std::clamp(lo[0] - m2, 0.0, 1.0); cell.hi[0] = std::clamp(hi[0] + m2, 0.0, 1.0);
            cell.lo[2] = std::clamp(lo[2] - m2, 0.0, 1.0); cell.hi[2] = std::clamp(hi[2] + m2, 0.0, 1.0);
            if (fullV) { cell.lo[1] = 0.0; cell.hi[1] = 1.0; pc.cells.append(cell); }
            else {
                const double v0 = std::max(0.0, c0 / kTwoPi - m2), v1 = c1 / kTwoPi + m2;
                cell.lo[1] = v0;
                cell.hi[1] = std::min(v1, 1.0);
                pc.cells.append(cell);
                if (v1 > 1.0) { PartCell w = cell; w.lo[1] = 0.0; w.hi[1] = std::min(v1 - 1.0, v0); pc.cells.append(w); }
            }
        }
        for (const auto &cr : m.cuts) pc.cuts.append(qMakePair(cr.first, cr.second));
        R.piecesVolume += m.volume();
        R.openEdges += openEdgesOf(m);
        R.pieces.append(std::move(pc));
    }
    QStringList per;
    for (auto it = cutsPerPart.cbegin(); it != cutsPerPart.cend(); ++it) per << QStringLiteral("part %1: %2").arg(it.key()).arg(it.value());
    R.notes << QStringLiteral("limb cross-sections: %1 of the cuts").arg(limbCuts);
    R.notes << QStringLiteral("whole model cut: %1 piece(s)%5; cuts made in %2; their volume %3% of the model's; %4 open edge(s)")
                   .arg(R.pieces.size()).arg(per.join(QStringLiteral(", ")))
                   .arg(R.partVolume > 0 ? 100.0 * R.piecesVolume / R.partVolume : 0.0, 0, 'f', 3)
                   .arg(R.openEdges)
                   .arg(bend > 0 ? QStringLiteral(", curved cuts (bend %1)").arg(bend, 0, 'f', 2) : QString());
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

QVector<char> CutInD::trimTriangles(const MeshData &piece, const MeshData &surface)
{
    QVector<char> out(piece.triangleCount(), 0);
    if (surface.triangleCount() == 0) return out;
    const double size = std::max(meshSize(surface), 1e-12);
    const double tol = 1e-6 * size;
    // Buckets of the surface's triangles.
    V3 lo = { surface.bmin[0], surface.bmin[1], surface.bmin[2] };
    const double cell = size / 48.0;
    int nb[3];
    for (int a = 0; a < 3; ++a) nb[a] = int((surface.bmax[a] - surface.bmin[a]) / cell) + 3;
    auto cid = [&](double x, int a) { return std::clamp(int((x - lo[a]) / cell) + 1, 0, nb[a] - 1); };
    std::vector<std::vector<int>> bucket(size_t(nb[0]) * nb[1] * nb[2]);
    for (int t = 0; t < surface.triangleCount(); ++t) {
        int c0[3], c1[3];
        for (int a = 0; a < 3; ++a) {
            double mn = 1e300, mx = -1e300;
            for (int k = 0; k < 3; ++k) { const double x = surface.pos[3 * surface.tris[3 * t + k] + a]; mn = std::min(mn, x); mx = std::max(mx, x); }
            c0[a] = cid(mn - tol, a);
            c1[a] = cid(mx + tol, a);
        }
        for (int i = c0[0]; i <= c1[0]; ++i)
            for (int j = c0[1]; j <= c1[1]; ++j)
                for (int k = c0[2]; k <= c1[2]; ++k)
                    bucket[size_t(i) + size_t(nb[0]) * (size_t(j) + size_t(nb[1]) * k)].push_back(t);
    }
    auto onSurface = [&](const V3 &p) {
        const auto &b = bucket[size_t(cid(p[0], 0)) + size_t(nb[0]) * (size_t(cid(p[1], 1)) + size_t(nb[1]) * cid(p[2], 2))];
        for (int t : b) {
            const V3 a = posOf(surface, int(surface.tris[3 * t])), bb = posOf(surface, int(surface.tris[3 * t + 1])),
                     c = posOf(surface, int(surface.tris[3 * t + 2]));
            if (pointTriDist(p, a, bb, c) <= tol) return true;
        }
        return false;
    };
    for (int t = 0; t < piece.triangleCount(); ++t) {
        const V3 a = posOf(piece, int(piece.tris[3 * t])), b = posOf(piece, int(piece.tris[3 * t + 1])),
                 c = posOf(piece, int(piece.tris[3 * t + 2]));
        out[t] = onSurface(mul(add(add(a, b), c), 1.0 / 3.0)) && onSurface(a) && onSurface(b) && onSurface(c);
    }
    return out;
}

// ================================================================== contacts
namespace {

// One piece for point location: its triangles bucketed, per ray axis, on a
// grid over the other two axes.
struct PieceGrid {
    static constexpr int G = 40;
    std::vector<V3> P;
    std::vector<std::array<int, 3>> T;
    double lo[3] = { 0, 0, 0 }, hi[3] = { 0, 0, 0 };
    std::vector<std::vector<int>> cells[3];

    explicit PieceGrid(const MeshData &m)
    {
        for (int i = 0; i < m.vertexCount(); ++i) P.push_back({ m.pos[3 * i], m.pos[3 * i + 1], m.pos[3 * i + 2] });
        for (int t = 0; t < m.triangleCount(); ++t)
            T.push_back({ int(m.tris[3 * t]), int(m.tris[3 * t + 1]), int(m.tris[3 * t + 2]) });
        for (int a = 0; a < 3; ++a) { lo[a] = 1e300; hi[a] = -1e300; }
        for (const V3 &x : P) for (int a = 0; a < 3; ++a) { lo[a] = std::min(lo[a], x[a]); hi[a] = std::max(hi[a], x[a]); }
        for (int a = 0; a < 3; ++a) {
            const int b = (a + 1) % 3, c = (a + 2) % 3;
            cells[a].assign(size_t(G) * G, std::vector<int>());
            for (int t = 0; t < int(T.size()); ++t) {
                double mn[2] = { 1e300, 1e300 }, mx[2] = { -1e300, -1e300 };
                for (int k = 0; k < 3; ++k) {
                    const V3 &x = P[T[t][k]];
                    mn[0] = std::min(mn[0], x[b]); mx[0] = std::max(mx[0], x[b]);
                    mn[1] = std::min(mn[1], x[c]); mx[1] = std::max(mx[1], x[c]);
                }
                const int i0 = cell(mn[0], b), i1 = cell(mx[0], b), j0 = cell(mn[1], c), j1 = cell(mx[1], c);
                for (int i = i0; i <= i1; ++i)
                    for (int j = j0; j <= j1; ++j) cells[a][size_t(i) + size_t(G) * j].push_back(t);
            }
        }
    }
    int cell(double x, int axis) const
    {
        const double span = hi[axis] - lo[axis];
        if (!(span > 0)) return 0;
        return std::clamp(int((x - lo[axis]) / span * G), 0, G - 1);
    }
    bool inBox(const V3 &p) const
    {
        for (int a = 0; a < 3; ++a) if (p[a] < lo[a] || p[a] > hi[a]) return false;
        return true;
    }
    // Crossings of the ray from p along +axis.
    int crossings(const V3 &p, int a) const
    {
        const int b = (a + 1) % 3, c = (a + 2) % 3;
        int n = 0;
        for (int t : cells[a][size_t(cell(p[b], b)) + size_t(G) * cell(p[c], c)]) {
            const V3 &A = P[T[t][0]], &Bv = P[T[t][1]], &C = P[T[t][2]];
            const double e0 = (Bv[b] - A[b]) * (p[c] - A[c]) - (Bv[c] - A[c]) * (p[b] - A[b]);
            const double e1 = (C[b] - Bv[b]) * (p[c] - Bv[c]) - (C[c] - Bv[c]) * (p[b] - Bv[b]);
            const double e2 = (A[b] - C[b]) * (p[c] - C[c]) - (A[c] - C[c]) * (p[b] - C[b]);
            if (!((e0 > 0 && e1 > 0 && e2 > 0) || (e0 < 0 && e1 < 0 && e2 < 0))) continue;
            const double sum = e0 + e1 + e2;
            // Barycentric weights: e1 is opposite A, e2 opposite B, e0 opposite C.
            const double x = (e1 * A[a] + e2 * Bv[a] + e0 * C[a]) / sum;
            if (x > p[a]) ++n;
        }
        return n;
    }
    bool inside(const V3 &p) const
    {
        if (!inBox(p)) return false;
        int votes = 0;
        for (int a = 0; a < 3; ++a) votes += crossings(p, a) & 1;
        return votes >= 2;
    }
};

// A surface in D, evaluated (B-spline, open knots).
struct DEval {
    const Trivariate::DSurface &d;
    std::vector<double> U, V;
    explicit DEval(const Trivariate::DSurface &ds)
        : d(ds), U(ds.knots[0].begin(), ds.knots[0].end()), V(ds.knots[1].begin(), ds.knots[1].end()) {}
    double s0() const { return U[d.order[0] - 1]; }
    double s1() const { return U[d.n[0]]; }
    double t0() const { return V[d.order[1] - 1]; }
    double t1() const { return V[d.n[1]]; }
    V3 at(double s, double t) const
    {
        const std::vector<double> Ns = basisAt(U, d.order[0], d.n[0], s), Nt = basisAt(V, d.order[1], d.n[1], t);
        V3 p = { 0, 0, 0 };
        for (int j = 0; j < d.n[1]; ++j) {
            if (Nt[j] == 0.0) continue;
            for (int i = 0; i < d.n[0]; ++i) {
                const double w = Ns[i] * Nt[j];
                if (w == 0.0) continue;
                const int m = 3 * (i + d.n[0] * j);
                p = add(p, mul(V3{ d.ctrl[m], d.ctrl[m + 1], d.ctrl[m + 2] }, w));
            }
        }
        return p;
    }
};

}  // namespace

CutInD::Locate CutInD::locator(const QVector<MeshData> &pieces)
{
    auto grids = std::make_shared<std::vector<PieceGrid>>();
    grids->reserve(size_t(pieces.size()));
    for (const MeshData &m : pieces) grids->emplace_back(m);
    return [grids](const double *p) {
        const V3 q = { p[0], p[1], p[2] };
        for (int i = 0; i < int(grids->size()); ++i)
            if ((*grids)[size_t(i)].inside(q)) return i;
        return -1;
    };
}

QVector<CutInD::Contact> CutInD::cutContacts(const Cut &cut, const HarmonicFit::Result &block,
                                             const Locate &locate, int samples)
{
    QVector<Contact> out;
    if (!block.ok) return out;
    const Block B(block);
    const int S = std::max(4, samples);
    const double eps = 3e-3;                         // the step off the cut, in D
    for (const Trivariate::DSurface &ds : cut.surfaces) {
        if (ds.ctrl.size() != 3 * ds.n[0] * ds.n[1]) continue;
        const DEval e(ds);
        const double hs = 1e-6 * (e.s1() - e.s0()), ht = 1e-6 * (e.t1() - e.t0());
        for (int i = 0; i < S; ++i)
            for (int j = 0; j < S; ++j) {
                const double s = e.s0() + (e.s1() - e.s0()) * (i + 0.5) / S;
                const double t = e.t0() + (e.t1() - e.t0()) * (j + 0.5) / S;
                const V3 p = e.at(s, t);
                // Its normal in D, from the surface's own tangents.
                const V3 Ss = mul(sub(e.at(s + hs, t), e.at(s - hs, t)), 0.5 / hs);
                const V3 St = mul(sub(e.at(s, t + ht), e.at(s, t - ht)), 0.5 / ht);
                V3 nD = cross(Ss, St);
                const double l = len(nD);
                if (!(l > 1e-14)) continue;
                nD = mul(nD, 1.0 / l);
                const V3 pm = sub(p, mul(nD, eps)), pp = add(p, mul(nD, eps));
                // Off the block (u or w past its faces): M is clamped there.
                if (pm[0] < 0 || pm[0] > 1 || pp[0] < 0 || pp[0] > 1 || pm[2] < 0 || pm[2] > 1 || pp[2] < 0 || pp[2] > 1)
                    continue;
                const V3 xm = B.eval(pm[0], pm[1], pm[2]), xp = B.eval(pp[0], pp[1], pp[2]);
                const int a = locate(xm.data()), b = locate(xp.data());
                if (a < 0 || b < 0 || a == b) continue;
                const V3 Mu = B.d(p[0], p[1], p[2], 0), Mv = B.d(p[0], p[1], p[2], 1), Mw = B.d(p[0], p[1], p[2], 2);
                V3 n = add(add(mul(cross(Mv, Mw), nD[0]), mul(cross(Mw, Mu), nD[1])), mul(cross(Mu, Mv), nD[2]));
                if (dot(n, sub(xp, xm)) < 0) n = mul(n, -1.0);
                Contact c;
                c.a = a; c.b = b;
                for (int k = 0; k < 3; ++k) c.n[k] = n[k];
                out.append(c);
            }
    }
    return out;
}

QVector<CutInD::Contact> CutInD::capContacts(const MeshData &caps, const Locate &locate, double eps)
{
    QVector<Contact> out;
    for (int t = 0; t < caps.triangleCount(); ++t) {
        const V3 A = posOf(caps, int(caps.tris[3 * t])), Bv = posOf(caps, int(caps.tris[3 * t + 1])),
                 C = posOf(caps, int(caps.tris[3 * t + 2]));
        const V3 n = cross(sub(Bv, A), sub(C, A));
        const double l = len(n);
        if (!(l > 0)) continue;
        const V3 c = mul(add(add(A, Bv), C), 1.0 / 3.0);
        const V3 xm = sub(c, mul(n, eps / l)), xp = add(c, mul(n, eps / l));
        const int a = locate(xm.data()), b = locate(xp.data());
        if (a < 0 || b < 0 || a == b) continue;
        Contact k;
        k.a = a; k.b = b;
        for (int q = 0; q < 3; ++q) k.n[q] = n[q];
        out.append(k);
    }
    return out;
}
