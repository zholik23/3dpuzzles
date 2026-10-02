//
// HarmonicFit - implementation. See the header for the paper's steps; the
// sections below follow them in order.
//
#include "HarmonicFit.h"
#include "IritGuard.h"

#include <QElapsedTimer>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <random>
#include <unordered_map>
#include <vector>

extern "C" {
#include "inc_irit/triv_lib.h"
#include "inc_irit/cagd_lib.h"
}

namespace {

using V3 = std::array<double, 3>;
constexpr double kPi = 3.14159265358979323846;

inline V3 add(const V3 &a, const V3 &b) { return { a[0] + b[0], a[1] + b[1], a[2] + b[2] }; }
inline V3 sub(const V3 &a, const V3 &b) { return { a[0] - b[0], a[1] - b[1], a[2] - b[2] }; }
inline V3 mul(const V3 &a, double s)    { return { a[0] * s, a[1] * s, a[2] * s }; }
inline double dot(const V3 &a, const V3 &b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
inline V3 cross(const V3 &a, const V3 &b)
{
    return { a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0] };
}
inline double len(const V3 &a) { return std::sqrt(dot(a, a)); }

// ------------------------------------------------------------------ CG
// Preconditioned (Jacobi) conjugate gradients for a symmetric positive
// definite operator. x holds the initial guess. Returns the iterations used.
int solveCg(const std::function<void(const std::vector<double> &, std::vector<double> &)> &A,
            const std::vector<double> &diag, const std::vector<double> &b,
            std::vector<double> &x, int maxIt, double tol)
{
    const size_t n = b.size();
    std::vector<double> r(n), z(n), p(n), Ap(n);
    A(x, Ap);
    double bn = 0.0;
    for (size_t i = 0; i < n; ++i) { r[i] = b[i] - Ap[i]; bn += b[i] * b[i]; }
    bn = std::sqrt(bn);
    if (bn <= 0.0) bn = 1.0;
    double rz = 0.0;
    for (size_t i = 0; i < n; ++i) { z[i] = r[i] / diag[i]; p[i] = z[i]; rz += r[i] * z[i]; }
    int it = 0;
    for (; it < maxIt; ++it) {
        double rn = 0.0;
        for (size_t i = 0; i < n; ++i) rn += r[i] * r[i];
        if (std::sqrt(rn) < tol * bn) break;
        A(p, Ap);
        double pAp = 0.0;
        for (size_t i = 0; i < n; ++i) pAp += p[i] * Ap[i];
        if (pAp <= 0.0) break;
        const double alpha = rz / pAp;
        double rzNew = 0.0;
        for (size_t i = 0; i < n; ++i) {
            x[i] += alpha * p[i];
            r[i] -= alpha * Ap[i];
            z[i] = r[i] / diag[i];
            rzNew += r[i] * z[i];
        }
        const double beta = rzNew / rz;
        rz = rzNew;
        for (size_t i = 0; i < n; ++i) p[i] = z[i] + beta * p[i];
    }
    return it;
}

// ------------------------------------------------------------------ surface
struct Surf {
    std::vector<V3> p;
    std::vector<std::array<int, 3>> t;
    std::vector<std::vector<int>> vtris;   // vertex -> triangles
};

inline long long edgeKey(int a, int b)
{
    if (a > b) std::swap(a, b);
    return (static_cast<long long>(a) << 32) | static_cast<unsigned int>(b);
}

// Welds coincident vertices (STL soups, OBJ seams), drops degenerate triangles
// and keeps the largest connected component (stray eyes, floating parts).
bool buildSurface(const MeshData &m, Surf *s, int *boundaryEdges, int *genus, QString *err)
{
    const int nv = m.vertexCount();
    const int nt = m.triangleCount();
    if (nt < 4) { *err = QStringLiteral("The model has no triangles to fit."); return false; }

    double lo[3], hi[3];
    for (int k = 0; k < 3; ++k) { lo[k] = 1e300; hi[k] = -1e300; }
    for (int i = 0; i < nv; ++i)
        for (int k = 0; k < 3; ++k) {
            lo[k] = std::min(lo[k], double(m.pos[3 * i + k]));
            hi[k] = std::max(hi[k], double(m.pos[3 * i + k]));
        }
    const double diag = std::sqrt((hi[0] - lo[0]) * (hi[0] - lo[0]) + (hi[1] - lo[1]) * (hi[1] - lo[1]) +
                                  (hi[2] - lo[2]) * (hi[2] - lo[2]));
    const double q = std::max(diag, 1e-12) * 2e-6;

    std::unordered_map<long long, int> weld;
    std::vector<int> remap(nv);
    std::vector<V3> pts;
    for (int i = 0; i < nv; ++i) {
        long long key = 0;
        for (int k = 0; k < 3; ++k)
            key = (key << 21) | (static_cast<long long>(std::llround((m.pos[3 * i + k] - lo[k]) / q)) & 0x1fffff);
        auto it = weld.find(key);
        if (it == weld.end()) {
            weld.emplace(key, int(pts.size()));
            remap[i] = int(pts.size());
            pts.push_back({ m.pos[3 * i], m.pos[3 * i + 1], m.pos[3 * i + 2] });
        } else {
            remap[i] = it->second;
        }
    }

    std::vector<std::array<int, 3>> tris;
    for (int t = 0; t < nt; ++t) {
        const int a = remap[m.tris[3 * t]], b = remap[m.tris[3 * t + 1]], c = remap[m.tris[3 * t + 2]];
        if (a == b || b == c || c == a) continue;
        tris.push_back({ a, b, c });
    }

    // Largest component by triangle count (union-find over vertices).
    std::vector<int> par(pts.size());
    for (size_t i = 0; i < par.size(); ++i) par[i] = int(i);
    std::function<int(int)> find = [&](int x) {
        while (par[x] != x) { par[x] = par[par[x]]; x = par[x]; }
        return x;
    };
    for (const auto &t : tris) {
        par[find(t[1])] = find(t[0]);
        par[find(t[2])] = find(t[0]);
    }
    std::unordered_map<int, int> compCount;
    for (const auto &t : tris) ++compCount[find(t[0])];
    int best = -1, bestN = 0;
    for (const auto &kv : compCount)
        if (kv.second > bestN) { bestN = kv.second; best = kv.first; }

    std::vector<int> newIdx(pts.size(), -1);
    s->p.clear();
    s->t.clear();
    for (const auto &t : tris) {
        if (find(t[0]) != best) continue;
        std::array<int, 3> nt3;
        for (int k = 0; k < 3; ++k) {
            if (newIdx[t[k]] < 0) { newIdx[t[k]] = int(s->p.size()); s->p.push_back(pts[t[k]]); }
            nt3[k] = newIdx[t[k]];
        }
        s->t.push_back(nt3);
    }
    s->vtris.assign(s->p.size(), {});
    for (int t = 0; t < int(s->t.size()); ++t)
        for (int k = 0; k < 3; ++k) s->vtris[s->t[t][k]].push_back(t);

    std::unordered_map<long long, int> edges;
    for (const auto &t : s->t)
        for (int k = 0; k < 3; ++k) ++edges[edgeKey(t[k], t[(k + 1) % 3])];
    int bnd = 0;
    for (const auto &kv : edges) if (kv.second == 1) ++bnd;
    *boundaryEdges = bnd;
    const long long chi = static_cast<long long>(s->p.size()) - static_cast<long long>(edges.size()) +
                          static_cast<long long>(s->t.size());
    *genus = bnd == 0 ? int((2 - chi) / 2) : -1;
    return true;
}

// One midpoint subdivision (each triangle into four; geometry unchanged).
// mark: a new edge midpoint is marked when both ends are - so a cap marked
// u = 0 stays exactly the cap.
void subdivide(Surf *s, std::vector<char> *mark)
{
    std::unordered_map<long long, int> mid;
    auto midOf = [&](int a, int b) {
        const long long key = edgeKey(a, b);
        auto it = mid.find(key);
        if (it != mid.end()) return it->second;
        const int id = int(s->p.size());
        s->p.push_back(mul(add(s->p[a], s->p[b]), 0.5));
        mark->push_back((*mark)[a] && (*mark)[b]);
        mid.emplace(key, id);
        return id;
    };
    std::vector<std::array<int, 3>> out;
    out.reserve(s->t.size() * 4);
    for (const auto &t : s->t) {
        const int ab = midOf(t[0], t[1]), bc = midOf(t[1], t[2]), ca = midOf(t[2], t[0]);
        out.push_back({ t[0], ab, ca });
        out.push_back({ ab, t[1], bc });
        out.push_back({ ca, bc, t[2] });
        out.push_back({ ab, bc, ca });
    }
    s->t.swap(out);
    s->vtris.assign(s->p.size(), {});
    for (int t = 0; t < int(s->t.size()); ++t)
        for (int k = 0; k < 3; ++k) s->vtris[s->t[t][k]].push_back(t);
}

// Principal axes (largest two) of the vertex cloud, by power iteration.
void principalAxes(const Surf &s, V3 *mean, V3 *e1, V3 *e2)
{
    V3 c = { 0, 0, 0 };
    for (const V3 &p : s.p) c = add(c, p);
    c = mul(c, 1.0 / std::max<size_t>(1, s.p.size()));
    double C[3][3] = { { 0 } };
    for (const V3 &p : s.p) {
        const V3 d = sub(p, c);
        for (int a = 0; a < 3; ++a)
            for (int b = 0; b < 3; ++b) C[a][b] += d[a] * d[b];
    }
    auto power = [&](V3 x, const V3 *deflate) {
        for (int it = 0; it < 100; ++it) {
            if (deflate) x = sub(x, mul(*deflate, dot(x, *deflate)));
            V3 y = { 0, 0, 0 };
            for (int a = 0; a < 3; ++a)
                for (int b = 0; b < 3; ++b) y[a] += C[a][b] * x[b];
            if (deflate) y = sub(y, mul(*deflate, dot(y, *deflate)));
            const double l = len(y);
            if (l <= 0.0) break;
            x = mul(y, 1.0 / l);
        }
        return x;
    };
    *mean = c;
    *e1 = power({ 0.577, 0.577, 0.577 }, nullptr);
    V3 seed = std::fabs((*e1)[0]) < 0.9 ? V3{ 1, 0, 0 } : V3{ 0, 1, 0 };
    *e2 = power(seed, e1);
}

// ------------------------------------------------------------------ step 1: surface harmonic
// Cotangent Laplacian with Dirichlet 0 / 1 at the two poles. Negative cotangents
// (obtuse triangles) are clamped to a small positive weight so the discrete
// maximum principle holds - no spurious extrema, so every level is a loop.
using Nbr = std::vector<std::vector<std::pair<int, double>>>;

// Clamped cotangent weights of the surface Laplacian (see surfaceHarmonic).
Nbr cotanNbr(const Surf &s)
{
    const int n = int(s.p.size());
    std::unordered_map<long long, double> wEdge;
    for (const auto &t : s.t)
        for (int k = 0; k < 3; ++k) {
            const int a = t[k], b = t[(k + 1) % 3], c = t[(k + 2) % 3];
            const V3 ea = sub(s.p[a], s.p[c]), eb = sub(s.p[b], s.p[c]);
            const double cr = len(cross(ea, eb));
            wEdge[edgeKey(a, b)] += 0.5 * dot(ea, eb) / std::max(cr, 1e-30);
        }
    double mean = 0.0;
    int cnt = 0;
    for (const auto &kv : wEdge) if (kv.second > 0) { mean += kv.second; ++cnt; }
    mean = cnt ? mean / cnt : 1.0;
    const double floorW = 1e-3 * mean;

    std::vector<std::vector<std::pair<int, double>>> nbr(n);
    for (const auto &kv : wEdge) {
        const int a = int(kv.first >> 32), b = int(kv.first & 0xffffffff);
        const double w = std::max(kv.second, floorW);
        nbr[a].push_back({ b, w });
        nbr[b].push_back({ a, w });
    }
    return nbr;
}

std::vector<double> surfaceHarmonic(const Surf &s, int vMin, int vMax, const V3 &axis,
                                    int *iters, Nbr *outNbr, const std::vector<int> *zeros = nullptr)
{
    const int n = int(s.p.size());
    const Nbr nbr = cotanNbr(s);
    if (outNbr) *outNbr = nbr;

    std::vector<double> u(n, 0.0);
    u[vMax] = 1.0;
    std::vector<char> fixed(n, 0);
    fixed[vMin] = fixed[vMax] = 1;
    if (zeros) for (int z : *zeros) fixed[z] = 1;          // a cap: u = 0 on all of it
    std::vector<int> unk(n, -1), idx;
    for (int i = 0; i < n; ++i)
        if (!fixed[i]) { unk[i] = int(idx.size()); idx.push_back(i); }

    // Initial guess: position along the axis, so CG starts close.
    const double a0 = dot(s.p[vMin], axis), a1 = dot(s.p[vMax], axis);
    std::vector<double> x(idx.size()), b(idx.size(), 0.0), dg(idx.size(), 0.0);
    for (size_t k = 0; k < idx.size(); ++k) {
        const int i = idx[k];
        x[k] = std::clamp((dot(s.p[i], axis) - a0) / (a1 - a0), 0.0, 1.0);
        for (const auto &nw : nbr[i]) {
            dg[k] += nw.second;
            if (unk[nw.first] < 0) b[k] += nw.second * u[nw.first];
        }
        if (dg[k] <= 0.0) dg[k] = 1.0;
    }
    auto A = [&](const std::vector<double> &in, std::vector<double> &out) {
        for (size_t k = 0; k < idx.size(); ++k) {
            double v = dg[k] * in[k];
            for (const auto &nw : nbr[idx[k]])
                if (unk[nw.first] >= 0) v -= nw.second * in[unk[nw.first]];
            out[k] = v;
        }
    };
    *iters = solveCg(A, dg, b, x, 20000, 1e-9);
    for (size_t k = 0; k < idx.size(); ++k) u[idx[k]] = x[k];
    return u;
}

// ------------------------------------------------------------------ step 2: v, the conjugate harmonic
// v is harmonic on the surface cut along a seam from pole to pole, and jumps by
// exactly 1 across the seam - the discrete conjugate of u, as the paper's
// "v from two critical paths, made periodic". Along every u loop it grows
// monotonically once round, and the SAME v on neighbouring loops lines up, so
// the (u, v) net does not shear where a loop dips into a limb (arclength did).
//
// Unknowns are single-valued on the side A of the seam: a vertex on side B
// sees a seam vertex s as v_s + 1. The poles are singular and left out.
struct SeamV {
    std::vector<double> v;
    std::vector<char>   seam, sideB;
    int                 iters = 0;
};

bool conjugateV(const Surf &s, const Nbr &nbr, int vMin, int vMax, SeamV *out, QString *err)
{
    const int n = int(s.p.size());

    // Seam: shortest edge path from the min pole to the max pole.
    std::vector<double> dist(n, 1e300);
    std::vector<int> prev(n, -1);
    {
        std::vector<std::pair<double, int>> heap{ { 0.0, vMin } };
        dist[vMin] = 0.0;
        auto cmp = [](const std::pair<double, int> &a, const std::pair<double, int> &b) { return a.first > b.first; };
        while (!heap.empty()) {
            std::pop_heap(heap.begin(), heap.end(), cmp);
            const auto [d, x] = heap.back();
            heap.pop_back();
            if (d > dist[x]) continue;
            if (x == vMax) break;
            for (const auto &nw : nbr[x]) {
                const double nd = d + len(sub(s.p[nw.first], s.p[x]));
                if (nd < dist[nw.first]) {
                    dist[nw.first] = nd;
                    prev[nw.first] = x;
                    heap.push_back({ nd, nw.first });
                    std::push_heap(heap.begin(), heap.end(), cmp);
                }
            }
        }
    }
    if (prev[vMax] < 0) { *err = QStringLiteral("The two poles are not connected on the surface."); return false; }
    std::vector<int> path;
    for (int x = vMax; x >= 0; x = prev[x]) path.push_back(x);
    std::reverse(path.begin(), path.end());

    out->seam.assign(n, 0);
    out->sideB.assign(n, 0);
    for (int x : path) out->seam[x] = 1;

    // Side A of each inner seam vertex: its 1-ring walked (in triangle order)
    // from the next seam vertex round to the previous one; the rest is side B.
    for (size_t m = 1; m + 1 < path.size(); ++m) {
        const int sv = path[m], pv = path[m - 1], nx = path[m + 1];
        std::vector<char> isA(n, 0);
        int cur = nx;
        for (size_t guard = 0; guard < s.vtris[sv].size() + 2 && cur != pv; ++guard) {
            int nextV = -1;
            for (int t : s.vtris[sv]) {
                const auto &T = s.t[t];
                for (int k = 0; k < 3; ++k)
                    if (T[k] == sv && T[(k + 1) % 3] == cur) nextV = T[(k + 2) % 3];
                if (nextV >= 0) break;
            }
            if (nextV < 0) break;
            cur = nextV;
            if (cur != pv) isA[cur] = 1;
        }
        for (const auto &nw : nbr[sv]) {
            const int y = nw.first;
            if (!out->seam[y] && !isA[y]) out->sideB[y] = 1;
        }
    }

    // v seen from row vertex i for neighbour j.
    auto jump = [&](int i, int j) {
        if (out->seam[j] && out->sideB[i]) return 1.0;
        if (out->seam[i] && out->sideB[j]) return -1.0;
        return 0.0;
    };

    std::vector<int> unk(n, -1), idx;
    const int pin = path.size() > 2 ? path[1] : -1;
    for (int i = 0; i < n; ++i)
        if (i != vMin && i != vMax && i != pin) { unk[i] = int(idx.size()); idx.push_back(i); }
    out->v.assign(n, 0.0);
    std::vector<double> x(idx.size(), 0.0), b(idx.size(), 0.0), dg(idx.size(), 0.0);
    for (size_t k = 0; k < idx.size(); ++k) {
        const int i = idx[k];
        for (const auto &nw : nbr[i]) {
            if (nw.first == vMin || nw.first == vMax) continue;
            dg[k] += nw.second;
            b[k] += nw.second * jump(i, nw.first);   // pinned vertex: v = 0
        }
        if (dg[k] <= 0.0) dg[k] = 1.0;
    }
    auto A = [&](const std::vector<double> &in, std::vector<double> &o) {
        for (size_t k = 0; k < idx.size(); ++k) {
            double val = dg[k] * in[k];
            for (const auto &nw : nbr[idx[k]])
                if (unk[nw.first] >= 0) val -= nw.second * in[unk[nw.first]];
            o[k] = val;
        }
    };
    out->iters = solveCg(A, dg, b, x, 40000, 1e-10);
    for (size_t k = 0; k < idx.size(); ++k) out->v[idx[k]] = x[k];
    return true;
}

// ------------------------------------------------------------------ level loops
struct Loop {
    std::vector<V3> pts;
    std::vector<std::array<int, 2>> ends;   // the mesh edge each point lies on
    std::vector<double> tt;                 // position along it, from ends[0]
    V3     centroid = { 0, 0, 0 };
    double perim = 0.0;
};

void finishLoop(Loop *L)
{
    const size_t n = L->pts.size();
    L->perim = 0.0;
    L->centroid = { 0, 0, 0 };
    for (size_t i = 0; i < n; ++i) {
        const V3 &a = L->pts[i], &b = L->pts[(i + 1) % n];
        const double l = len(sub(b, a));
        L->perim += l;
        L->centroid = add(L->centroid, mul(add(a, b), 0.5 * l));
    }
    if (L->perim > 0) L->centroid = mul(L->centroid, 1.0 / L->perim);
}

// The closed loops of u = level on the surface, all oriented the same way
// relative to the surface normal and grad u (majority vote per loop, so a few
// flipped triangles do not matter). Which way round is settled later by the
// sign of det J.
std::vector<Loop> levelLoops(const Surf &s, const std::vector<double> &u, double level)
{
    std::unordered_map<long long, int> nodeOf;
    std::vector<V3> node;
    std::vector<std::array<int, 2>> nodeEnds;
    std::vector<double> nodeT;
    struct Link { int a, b; int sgn; };
    std::vector<Link> links;
    auto f = [&](int v) {
        const double d = u[v] - level;
        return d == 0.0 ? 1e-15 : d;
    };
    auto nodeFor = [&](int a, int b) {
        const long long key = edgeKey(a, b);
        auto it = nodeOf.find(key);
        if (it != nodeOf.end()) return it->second;
        const double fa = f(a), fb = f(b);
        const double t = fa / (fa - fb);
        const int id = int(node.size());
        node.push_back(add(s.p[a], mul(sub(s.p[b], s.p[a]), t)));
        nodeEnds.push_back({ a, b });
        nodeT.push_back(t);
        nodeOf.emplace(key, id);
        return id;
    };

    for (const auto &t : s.t) {
        const double fv[3] = { f(t[0]), f(t[1]), f(t[2]) };
        int cr[2], nc = 0;
        for (int k = 0; k < 3 && nc < 2; ++k)
            if ((fv[k] > 0) != (fv[(k + 1) % 3] > 0)) cr[nc++] = k;
        if (nc != 2) continue;
        const int n1 = nodeFor(t[cr[0]], t[(cr[0] + 1) % 3]);
        const int n2 = nodeFor(t[cr[1]], t[(cr[1] + 1) % 3]);
        // grad f in the triangle, then the walking direction n x grad f.
        const V3 &A = s.p[t[0]], &B = s.p[t[1]], &C = s.p[t[2]];
        const V3 nrm = cross(sub(B, A), sub(C, A));
        const double nn = dot(nrm, nrm);
        if (nn <= 0.0) continue;
        V3 g = add(add(mul(cross(nrm, sub(C, B)), fv[0]), mul(cross(nrm, sub(A, C)), fv[1])),
                   mul(cross(nrm, sub(B, A)), fv[2]));
        g = mul(g, 1.0 / nn);
        const V3 dir = cross(nrm, g);
        links.push_back({ n1, n2, dot(sub(node[n2], node[n1]), dir) >= 0 ? 1 : -1 });
    }

    std::vector<std::array<int, 2>> adj(node.size(), { -1, -1 });
    for (int l = 0; l < int(links.size()); ++l)
        for (int e = 0; e < 2; ++e) {
            const int nd = e ? links[l].b : links[l].a;
            if (adj[nd][0] < 0) adj[nd][0] = l; else if (adj[nd][1] < 0) adj[nd][1] = l;
        }

    std::vector<char> seen(node.size(), 0);
    std::vector<Loop> out;
    for (int start = 0; start < int(node.size()); ++start) {
        if (seen[start] || adj[start][0] < 0 || adj[start][1] < 0) continue;
        Loop L;
        int vote = 0, cur = start, viaLink = -1;
        bool closed = false;
        for (size_t guard = 0; guard <= node.size(); ++guard) {
            seen[cur] = 1;
            L.pts.push_back(node[cur]);
            L.ends.push_back(nodeEnds[cur]);
            L.tt.push_back(nodeT[cur]);
            const int l = adj[cur][0] == viaLink ? adj[cur][1] : adj[cur][0];
            if (l < 0) break;
            const int nxt = links[l].a == cur ? links[l].b : links[l].a;
            vote += links[l].a == cur ? links[l].sgn : -links[l].sgn;
            viaLink = l;
            if (nxt == start) { closed = true; break; }
            if (seen[nxt]) break;
            cur = nxt;
        }
        if (!closed || L.pts.size() < 3) continue;
        if (vote < 0) {
            std::reverse(L.pts.begin(), L.pts.end());
            std::reverse(L.ends.begin(), L.ends.end());
            std::reverse(L.tt.begin(), L.tt.end());
        }
        finishLoop(&L);
        out.push_back(std::move(L));
    }
    return out;
}

// Closest point on the closed polyline; returns the arclength position.
double closestOnLoop(const Loop &L, const V3 &x)
{
    double best = 1e300, bestS = 0.0, s = 0.0;
    const size_t n = L.pts.size();
    for (size_t i = 0; i < n; ++i) {
        const V3 &a = L.pts[i], &b = L.pts[(i + 1) % n];
        const V3 ab = sub(b, a);
        const double l2 = dot(ab, ab), l = std::sqrt(l2);
        const double t = l2 > 0 ? std::clamp(dot(sub(x, a), ab) / l2, 0.0, 1.0) : 0.0;
        const double d = len(sub(add(a, mul(ab, t)), x));
        if (d < best) { best = d; bestS = s + t * l; }
        s += l;
    }
    return bestS;
}

// v at a loop point on mesh edge e, seen consistently across the seam.
double vOnEdge(const SeamV &sv, const std::array<int, 2> &e, double t)
{
    double va = sv.v[e[0]], vb = sv.v[e[1]];
    if (sv.seam[e[0]] && sv.sideB[e[1]]) va += 1.0;
    if (sv.seam[e[1]] && sv.sideB[e[0]]) vb += 1.0;
    return va + t * (vb - va);
}

// How many times v winds round the loop: +-1 for the loop that separates the
// two poles (the main tube), 0 for a loop round a leg, a horn or an ear.
double loopWinding(const Loop &L, const SeamV &sv)
{
    const size_t n = L.pts.size();
    if (n < 3) return 0.0;
    double prev = vOnEdge(sv, L.ends[0], L.tt[0]), total = 0.0;
    for (size_t m = 1; m <= n; ++m) {
        const double raw = vOnEdge(sv, L.ends[m % n], L.tt[m % n]);
        double d = raw - prev;
        d -= std::round(d);
        total += d;
        prev = raw;
    }
    return total;
}

// The main-tube loop of a level: among the loops v winds round once, the one
// closest to the expected centroid and perimeter. -1 if none winds.
int mainLoop(const std::vector<Loop> &loops, const SeamV &sv, const V3 &c, double perim)
{
    int bi = -1;
    double bs = 1e300;
    for (int l = 0; l < int(loops.size()); ++l) {
        if (std::fabs(std::fabs(loopWinding(loops[l], sv)) - 1.0) > 0.25) continue;
        const double sc = len(sub(loops[l].centroid, c)) + std::fabs(loops[l].perim - perim) / (2 * kPi);
        if (sc < bs) { bs = sc; bi = l; }
    }
    return bi;
}

V3 pointOnLoop(const Loop &L, double s)
{
    const size_t n = L.pts.size();
    s = std::fmod(s, L.perim);
    if (s < 0) s += L.perim;
    for (size_t i = 0; i < n; ++i) {
        const V3 &a = L.pts[i], &b = L.pts[(i + 1) % n];
        const double l = len(sub(b, a));
        if (s <= l || i + 1 == n)
            return add(a, mul(sub(b, a), l > 0 ? std::min(1.0, s / l) : 0.0));
        s -= l;
    }
    return L.pts.front();
}

// ------------------------------------------------------------------ voxel grid
struct Grid {
    int nx = 0, ny = 0, nz = 0;
    V3 o = { 0, 0, 0 };
    double h = 1.0;
    int idx(int i, int j, int k) const { return i + nx * (j + ny * k); }
    size_t size() const { return size_t(nx) * ny * nz; }
    V3 pos(int i, int j, int k) const { return { o[0] + i * h, o[1] + j * h, o[2] + k * h }; }
    int dim(int a) const { return a == 0 ? nx : (a == 1 ? ny : nz); }

    double sample(const std::vector<double> &f, const V3 &x) const
    {
        double g[3];
        int i0[3];
        for (int a = 0; a < 3; ++a) {
            g[a] = (x[a] - o[a]) / h;
            i0[a] = std::clamp(int(std::floor(g[a])), 0, dim(a) - 2);
            g[a] = std::clamp(g[a] - i0[a], 0.0, 1.0);
        }
        double v = 0.0;
        for (int c = 0; c < 8; ++c) {
            const int di = c & 1, dj = (c >> 1) & 1, dk = (c >> 2) & 1;
            const double w = (di ? g[0] : 1 - g[0]) * (dj ? g[1] : 1 - g[1]) * (dk ? g[2] : 1 - g[2]);
            v += w * f[idx(i0[0] + di, i0[1] + dj, i0[2] + dk)];
        }
        return v;
    }
    V3 grad(const std::vector<double> &f, const V3 &x) const
    {
        V3 g;
        const double d = 0.5 * h;
        for (int a = 0; a < 3; ++a) {
            V3 p = x, m = x;
            p[a] += d;
            m[a] -= d;
            g[a] = (sample(f, p) - sample(f, m)) / (2 * d);
        }
        return g;
    }
};

// Inside by ray parity along each axis; majority of the three, so a ray that
// grazes an edge or vertex cannot flip a whole column.
std::vector<char> insideVoxels(const Surf &s, const Grid &G)
{
    std::vector<unsigned char> votes(G.size(), 0);
    const double jit[3] = { 0.01371 * G.h, 0.02917 * G.h, 0.00733 * G.h };
    for (int a = 0; a < 3; ++a) {
        const int b = (a + 1) % 3, c = (a + 2) % 3;
        const int nb = G.dim(b), nc = G.dim(c);
        std::vector<std::vector<double>> hits(size_t(nb) * nc);
        for (const auto &t : s.t) {
            const V3 &P = s.p[t[0]], &Q = s.p[t[1]], &R = s.p[t[2]];
            const double bmin = std::min({ P[b], Q[b], R[b] }), bmax = std::max({ P[b], Q[b], R[b] });
            const double cmin = std::min({ P[c], Q[c], R[c] }), cmax = std::max({ P[c], Q[c], R[c] });
            const int ib0 = std::max(0, int(std::ceil((bmin - G.o[b] - jit[b]) / G.h)));
            const int ib1 = std::min(nb - 1, int(std::floor((bmax - G.o[b] - jit[b]) / G.h)));
            const int ic0 = std::max(0, int(std::ceil((cmin - G.o[c] - jit[c]) / G.h)));
            const int ic1 = std::min(nc - 1, int(std::floor((cmax - G.o[c] - jit[c]) / G.h)));
            const double det = (Q[b] - P[b]) * (R[c] - P[c]) - (R[b] - P[b]) * (Q[c] - P[c]);
            if (std::fabs(det) < 1e-300) continue;
            for (int ib = ib0; ib <= ib1; ++ib)
                for (int ic = ic0; ic <= ic1; ++ic) {
                    const double yb = G.o[b] + ib * G.h + jit[b], yc = G.o[c] + ic * G.h + jit[c];
                    const double l1 = ((yb - P[b]) * (R[c] - P[c]) - (R[b] - P[b]) * (yc - P[c])) / det;
                    const double l2 = ((Q[b] - P[b]) * (yc - P[c]) - (yb - P[b]) * (Q[c] - P[c])) / det;
                    if (l1 < 0 || l2 < 0 || l1 + l2 > 1) continue;
                    hits[size_t(ib) + size_t(nb) * ic].push_back(P[a] + l1 * (Q[a] - P[a]) + l2 * (R[a] - P[a]));
                }
        }
        for (int ib = 0; ib < nb; ++ib)
            for (int ic = 0; ic < nc; ++ic) {
                auto &hs = hits[size_t(ib) + size_t(nb) * ic];
                if (hs.empty()) continue;
                std::sort(hs.begin(), hs.end());
                size_t h = 0;
                for (int ia = 0; ia < G.dim(a); ++ia) {
                    const double x = G.o[a] + ia * G.h;
                    while (h < hs.size() && hs[h] < x) ++h;
                    if (h & 1) {
                        int ijk[3];
                        ijk[a] = ia; ijk[b] = ib; ijk[c] = ic;
                        ++votes[G.idx(ijk[0], ijk[1], ijk[2])];
                    }
                }
            }
    }
    std::vector<char> in(G.size(), 0);
    for (size_t i = 0; i < in.size(); ++i) in[i] = votes[i] >= 2;

    // Largest 6-connected region only.
    std::vector<int> comp(G.size(), -1);
    int bestComp = -1, bestN = 0, nComp = 0;
    std::vector<int> stack;
    for (int k = 0; k < G.nz; ++k)
        for (int j = 0; j < G.ny; ++j)
            for (int i = 0; i < G.nx; ++i) {
                const int v = G.idx(i, j, k);
                if (!in[v] || comp[v] >= 0) continue;
                int n = 0;
                stack.assign(1, v);
                comp[v] = nComp;
                while (!stack.empty()) {
                    const int x = stack.back();
                    stack.pop_back();
                    ++n;
                    const int xi = x % G.nx, xj = (x / G.nx) % G.ny, xk = x / (G.nx * G.ny);
                    const int nb[6][3] = { { xi - 1, xj, xk }, { xi + 1, xj, xk }, { xi, xj - 1, xk },
                                           { xi, xj + 1, xk }, { xi, xj, xk - 1 }, { xi, xj, xk + 1 } };
                    for (const auto &q : nb) {
                        if (q[0] < 0 || q[1] < 0 || q[2] < 0 || q[0] >= G.nx || q[1] >= G.ny || q[2] >= G.nz) continue;
                        const int y = G.idx(q[0], q[1], q[2]);
                        if (in[y] && comp[y] < 0) { comp[y] = nComp; stack.push_back(y); }
                    }
                }
                if (n > bestN) { bestN = n; bestComp = nComp; }
                ++nComp;
            }
    for (size_t i = 0; i < in.size(); ++i) in[i] = in[i] && comp[i] == bestComp;
    return in;
}

// Closest point on triangle (Ericson, Real-Time Collision Detection 5.1.5);
// returns barycentric weights of a, b, c.
V3 closestBary(const V3 &p, const V3 &a, const V3 &b, const V3 &c)
{
    const V3 ab = sub(b, a), ac = sub(c, a), ap = sub(p, a);
    const double d1 = dot(ab, ap), d2 = dot(ac, ap);
    if (d1 <= 0 && d2 <= 0) return { 1, 0, 0 };
    const V3 bp = sub(p, b);
    const double d3 = dot(ab, bp), d4 = dot(ac, bp);
    if (d3 >= 0 && d4 <= d3) return { 0, 1, 0 };
    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) { const double v = d1 / (d1 - d3); return { 1 - v, v, 0 }; }
    const V3 cp = sub(p, c);
    const double d5 = dot(ab, cp), d6 = dot(ac, cp);
    if (d6 >= 0 && d5 <= d6) return { 0, 0, 1 };
    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) { const double w = d2 / (d2 - d6); return { 1 - w, 0, w }; }
    const double va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
        const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return { 0, 1 - w, w };
    }
    const double den = 1.0 / (va + vb + vc);
    const double v = vb * den, w = vc * den;
    return { 1 - v - w, v, w };
}

// u at the surface point closest to x: nearest vertex through a bucket grid,
// then the closest point on its triangles, u interpolated there.
struct SurfaceLookup {
    const Surf &s;
    const std::vector<double> &u;
    V3 o;
    double cell;
    int n[3];
    std::vector<std::vector<int>> bucket;

    SurfaceLookup(const Surf &surf, const std::vector<double> &uu, const Grid &G)
        : s(surf), u(uu), o(G.o), cell(2 * G.h)
    {
        n[0] = G.nx / 2 + 2; n[1] = G.ny / 2 + 2; n[2] = G.nz / 2 + 2;
        bucket.resize(size_t(n[0]) * n[1] * n[2]);
        for (int v = 0; v < int(s.p.size()); ++v) bucket[key(s.p[v])].push_back(v);
    }
    int clampI(int a, double x) const { return std::clamp(int((x - o[a]) / cell), 0, n[a] - 1); }
    size_t key(const V3 &p) const
    {
        return size_t(clampI(0, p[0])) + size_t(n[0]) * (clampI(1, p[1]) + size_t(n[1]) * clampI(2, p[2]));
    }
    double valueAt(const V3 &x) const
    {
        const int c[3] = { clampI(0, x[0]), clampI(1, x[1]), clampI(2, x[2]) };
        int bestV = -1;
        double best = 1e300;
        const int maxR = std::max({ n[0], n[1], n[2] });
        for (int r = 0; r <= maxR; ++r) {
            for (int dk = -r; dk <= r; ++dk)
                for (int dj = -r; dj <= r; ++dj)
                    for (int di = -r; di <= r; ++di) {
                        if (std::max({ std::abs(di), std::abs(dj), std::abs(dk) }) != r) continue;
                        const int q[3] = { c[0] + di, c[1] + dj, c[2] + dk };
                        if (q[0] < 0 || q[1] < 0 || q[2] < 0 || q[0] >= n[0] || q[1] >= n[1] || q[2] >= n[2]) continue;
                        for (int v : bucket[size_t(q[0]) + size_t(n[0]) * (q[1] + size_t(n[1]) * q[2])]) {
                            const double d = len(sub(s.p[v], x));
                            if (d < best) { best = d; bestV = v; }
                        }
                    }
            if (bestV >= 0 && (r - 1) * cell > best) break;
        }
        if (bestV < 0) return 0.0;
        double bestD = 1e300, val = u[bestV];
        for (int t : s.vtris[bestV]) {
            const auto &T = s.t[t];
            const V3 w = closestBary(x, s.p[T[0]], s.p[T[1]], s.p[T[2]]);
            const V3 q = add(add(mul(s.p[T[0]], w[0]), mul(s.p[T[1]], w[1])), mul(s.p[T[2]], w[2]));
            const double d = len(sub(q, x));
            if (d < bestD) { bestD = d; val = w[0] * u[T[0]] + w[1] * u[T[1]] + w[2] * u[T[2]]; }
        }
        return val;
    }
};

// Laplace on the voxels marked `unknown`; every other voxel keeps its value in
// f as a Dirichlet condition. 7-point stencil.
int solveGrid(const Grid &G, const std::vector<char> &unknown, std::vector<double> &f)
{
    std::vector<int> unk(G.size(), -1), idx;
    for (int v = 0; v < int(G.size()); ++v)
        if (unknown[v]) { unk[v] = int(idx.size()); idx.push_back(v); }
    const int off[6] = { -1, 1, -G.nx, G.nx, -G.nx * G.ny, G.nx * G.ny };
    std::vector<double> x(idx.size()), b(idx.size(), 0.0), dg(idx.size(), 6.0);
    for (size_t k = 0; k < idx.size(); ++k) {
        x[k] = f[idx[k]];
        for (int o : off) {
            const int y = idx[k] + o;
            if (unk[y] < 0) b[k] += f[y];
        }
    }
    auto A = [&](const std::vector<double> &in, std::vector<double> &out) {
        for (size_t k = 0; k < idx.size(); ++k) {
            double v = 6.0 * in[k];
            for (int o : off) {
                const int y = unk[idx[k] + o];
                if (y >= 0) v -= in[y];
            }
            out[k] = v;
        }
    };
    const int it = solveCg(A, dg, b, x, 5000, 1e-7);
    for (size_t k = 0; k < idx.size(); ++k) f[idx[k]] = x[k];
    return it;
}

// Cubic B-spline basis values at t on knot vector U (n control points).
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
    for (int m = 0; m < n + order; ++m)
        U[m] = std::clamp(double(m - order + 1) / inner, 0.0, 1.0);
    return U;
}

// ------------------------------------------------------------------ IRIT build
struct BuildCtx {
    const double *ctrl;
    int nu, nv, nw;
    TrivTVStruct *result;
};

void doBuild(void *vp)
{
    BuildCtx *c = static_cast<BuildCtx *>(vp);
    c->result = NULL;
    TrivTVStruct *tv = IritTrivBspTVNew(c->nu, c->nv + 3, c->nw, 4, 4, 2, CAGD_PT_E3_TYPE);
    if (tv == NULL) return;
    for (int k = 0; k < c->nw; ++k)
        for (int jj = 0; jj < c->nv + 3; ++jj)
            for (int i = 0; i < c->nu; ++i) {
                const int j = jj % c->nv;
                const double *p = c->ctrl + 3 * (i + c->nu * (j + c->nv * k));
                const int m = TRIV_MESH_UVW(tv, i, jj, k);
                tv->Points[1][m] = p[0];
                tv->Points[2][m] = p[1];
                tv->Points[3][m] = p[2];
            }
    const std::vector<double> U = openKnots(c->nu, 4), W = openKnots(c->nw, 2);
    for (int m = 0; m < c->nu + 4; ++m) tv->UKnotVector[m] = U[m];
    for (int m = 0; m < c->nw + 2; ++m) tv->WKnotVector[m] = W[m];
    // Float (uniform, unclamped) in v with the first three control points
    // repeated: a closed C2 loop whose valid domain is [0, 1].
    for (int m = 0; m < c->nv + 7; ++m) tv->VKnotVector[m] = double(m - 3) / c->nv;
    TrivTVStruct *open = IritTrivCnvrtFloat2OpenTV(tv);
    IritTrivTVFree(tv);
    c->result = open;
}

}  // namespace

// ================================================================== fit
HarmonicFit::Result HarmonicFit::fit(const MeshData &mesh, const Options &optIn)
{
    Result R;
    Options opt = optIn;
    opt.nu = std::max(4, opt.nu);
    opt.nv = std::max(4, opt.nv);
    opt.nw = std::max(2, opt.nw);
    opt.voxels = std::clamp(opt.voxels, 24, 160);

    Surf S;
    int bnd = 0;
    if (!buildSurface(mesh, &S, &bnd, &R.genus, &R.error)) return R;
    const int nEdges = int(S.t.size() * 3 / 2);
    if (bnd > std::max(8, nEdges / 100)) {
        R.error = QStringLiteral("The mesh is open (%1 boundary edges); the fit needs a closed surface.").arg(bnd);
        return R;
    }
    if (bnd > 0) R.notes << QStringLiteral("%1 boundary edges ignored (small holes)").arg(bnd);
    if (R.genus > 0)
        R.notes << QStringLiteral("genus %1: the method assumes genus 0 - handles are not represented").arg(R.genus);

    V3 lo = { 1e300, 1e300, 1e300 }, hi = { -1e300, -1e300, -1e300 };
    for (const V3 &p : S.p)
        for (int a = 0; a < 3; ++a) { lo[a] = std::min(lo[a], p[a]); hi[a] = std::max(hi[a], p[a]); }
    R.size = len(sub(hi, lo));
    for (const auto &t : S.t) R.meshVolume += dot(S.p[t[0]], cross(S.p[t[1]], S.p[t[2]])) / 6.0;
    R.meshVolume = std::fabs(R.meshVolume);

    // ---- step 1: poles and the surface harmonic
    V3 mean, e1, e2;
    principalAxes(S, &mean, &e1, &e2);
    int vMin = 0, vMax = 0;
    std::vector<int> capZeros;
    const V3 capC = { opt.capCentre[0], opt.capCentre[1], opt.capCentre[2] };
    if (opt.capStart) {
        // u = 0 on the whole cap: its fan centre and the ring round it.
        for (int v = 0; v < int(S.p.size()); ++v)
            if (len(sub(S.p[v], capC)) < len(sub(S.p[vMin], capC))) vMin = v;
        std::vector<char> mark(S.p.size(), 0);
        mark[vMin] = 1;
        for (int t : S.vtris[vMin])
            for (int k = 0; k < 3; ++k) mark[S.t[t][k]] = 1;
        // A small part (a hoof cut off at 108 vertices) is too coarse for the
        // harmonic v - its fit found no level loops. Subdivide it first; the
        // shape is unchanged and the cap stays marked.
        int subdiv = 0;
        while (S.p.size() < 1500 && subdiv < 3) { subdivide(&S, &mark); ++subdiv; }
        if (subdiv) R.notes << QStringLiteral("small part: subdivided %1 time(s) to %2 vertices").arg(subdiv).arg(S.p.size());
        // The far pole: farthest from the cap along the direction to the
        // part's centre - not simply the farthest point, which on spot's head
        // was a horn tip: the tube ran out through the horn and cut it short.
        V3 pc = { 0, 0, 0 };
        for (const V3 &q : S.p) pc = add(pc, q);
        pc = mul(pc, 1.0 / std::max<size_t>(1, S.p.size()));
        V3 dir = sub(pc, capC);
        dir = mul(dir, 1.0 / std::max(1e-30, len(dir)));
        for (int v = 0; v < int(S.p.size()); ++v) {
            if (mark[v] && v != vMin) capZeros.push_back(v);
            if (dot(sub(S.p[v], capC), dir) > dot(sub(S.p[vMax], capC), dir)) vMax = v;
        }
        e1 = sub(S.p[vMax], S.p[vMin]);
        e1 = mul(e1, 1.0 / std::max(1e-30, len(e1)));
    } else {
        for (int v = 0; v < int(S.p.size()); ++v) {
            if (dot(S.p[v], e1) < dot(S.p[vMin], e1)) vMin = v;
            if (dot(S.p[v], e1) > dot(S.p[vMax], e1)) vMax = v;
        }
    }
    int cgIt = 0;
    Nbr nbr;
    const std::vector<double> u = surfaceHarmonic(S, vMin, vMax, e1, &cgIt, &nbr,
                                                  opt.capStart ? &capZeros : nullptr);
    SeamV sv;
    if (!conjugateV(S, nbr, vMin, vMax, &sv, &R.error)) return R;
    R.notes << QStringLiteral("surface harmonics u, v: %1 vertices, %2 + %3 CG iterations")
                   .arg(S.p.size()).arg(cgIt).arg(sv.iters);

    // ---- step 2a: scan the levels, following one loop (the main tube)
    const int M = 240;
    std::vector<double> candL(M), candPerim(M, 0.0);
    std::vector<V3> candC(M);
    std::vector<int> candBranch(M, 0);
    std::vector<char> candOk(M, 0);
    {
        V3 prevC = S.p[vMin];
        double prevP = 0.0;
        for (int c = 0; c < M; ++c) {
            candL[c] = 0.004 + 0.992 * c / (M - 1);
            const std::vector<Loop> loops = levelLoops(S, u, candL[c]);
            const int bi = mainLoop(loops, sv, prevC, prevP);
            if (bi < 0) continue;
            candOk[c] = 1;
            candC[c] = loops[bi].centroid;
            candPerim[c] = loops[bi].perim;
            for (int l = 0; l < int(loops.size()); ++l)
                if (l != bi && loops[l].perim > 0.1 * loops[bi].perim) candBranch[c] = 1;
            prevC = candC[c];
            prevP = candPerim[c];
        }
    }
    double maxPerim = 0.0;
    for (int c = 0; c < M; ++c) if (candOk[c]) maxPerim = std::max(maxPerim, candPerim[c]);
    int c0 = 0, c1 = M - 1;
    const bool toPoles = opt.capStart || opt.closedEnds;
    if (toPoles)
        while (c0 < M && !candOk[c0]) ++c0;                   // the cap end is kept, not cut off
    else
        while (c0 < M && !(candOk[c0] && candPerim[c0] >= opt.capPerimeter * maxPerim)) ++c0;
    if (toPoles)
        while (c1 >= 0 && !candOk[c1]) --c1;                  // the far end runs to the tip
    else
        while (c1 >= 0 && !(candOk[c1] && candPerim[c1] >= opt.capPerimeter * maxPerim)) --c1;
    if (c0 >= c1) {
        R.error = QStringLiteral("No usable u levels on the surface (the harmonic field is degenerate).");
        return R;
    }

    // ---- step 3: voxel grid, volume harmonic uH
    Grid G;
    {
        const double ext = std::max({ hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2] });
        G.h = ext / opt.voxels;
        const int pad = 3;
        G.o = { lo[0] - pad * G.h, lo[1] - pad * G.h, lo[2] - pad * G.h };
        G.nx = int(std::ceil((hi[0] - lo[0]) / G.h)) + 1 + 2 * pad;
        G.ny = int(std::ceil((hi[1] - lo[1]) / G.h)) + 1 + 2 * pad;
        G.nz = int(std::ceil((hi[2] - lo[2]) / G.h)) + 1 + 2 * pad;
    }
    const std::vector<char> inside = insideVoxels(S, G);
    int nInside = 0;
    for (char c : inside) nInside += c;
    if (nInside < 200) {
        R.error = QStringLiteral("Only %1 voxels inside the model - is it closed, or too thin for %2 voxels?")
                      .arg(nInside).arg(opt.voxels);
        return R;
    }

    // Two layers of shell around the inside carry the boundary values.
    std::vector<char> kind(G.size(), 0);   // 0 far, 1 inside, 2 shell
    for (size_t v = 0; v < G.size(); ++v) if (inside[v]) kind[v] = 1;
    for (int layer = 0; layer < 2; ++layer) {
        std::vector<char> next = kind;
        for (int k = 1; k < G.nz - 1; ++k)
            for (int j = 1; j < G.ny - 1; ++j)
                for (int i = 1; i < G.nx - 1; ++i) {
                    const int v = G.idx(i, j, k);
                    if (kind[v] != 0) continue;
                    bool near = false;
                    for (int d = 0; d < 27 && !near; ++d) {
                        const int y = G.idx(i + d % 3 - 1, j + (d / 3) % 3 - 1, k + d / 9 - 1);
                        near = kind[y] != 0;
                    }
                    if (near) next[v] = 2;
                }
        kind.swap(next);
    }

    std::vector<double> uH(G.size(), 0.5);
    {
        SurfaceLookup look(S, u, G);
        for (int k = 0; k < G.nz; ++k)
            for (int j = 0; j < G.ny; ++j)
                for (int i = 0; i < G.nx; ++i)
                    if (kind[G.idx(i, j, k)] == 2) uH[G.idx(i, j, k)] = look.valueAt(G.pos(i, j, k));
    }
    std::vector<char> unknown(G.size(), 0);
    for (size_t v = 0; v < G.size(); ++v) unknown[v] = kind[v] == 1;
    const int itH = solveGrid(G, unknown, uH);
    R.notes << QStringLiteral("voxel grid %1x%2x%3, %4 inside, uH %5 CG iterations")
                   .arg(G.nx).arg(G.ny).arg(G.nz).arg(nInside).arg(itH);

    // Distance to the boundary (chamfer, voxel units), for the skeleton.
    std::vector<double> dist(G.size(), 0.0);
    for (size_t v = 0; v < G.size(); ++v) dist[v] = kind[v] == 1 ? 1e9 : 0.0;
    for (int pass = 0; pass < 2; ++pass) {
        const int s = pass == 0 ? 1 : -1;
        for (int kk = 1; kk < G.nz - 1; ++kk)
            for (int jj = 1; jj < G.ny - 1; ++jj)
                for (int ii = 1; ii < G.nx - 1; ++ii) {
                    const int i = s > 0 ? ii : G.nx - 1 - ii;
                    const int j = s > 0 ? jj : G.ny - 1 - jj;
                    const int k = s > 0 ? kk : G.nz - 1 - kk;
                    const int v = G.idx(i, j, k);
                    if (kind[v] != 1) continue;
                    double best = dist[v];
                    for (int d = 0; d < 27; ++d) {
                        const int di = d % 3 - 1, dj = (d / 3) % 3 - 1, dk = d / 9 - 1;
                        if (di == 0 && dj == 0 && dk == 0) continue;
                        best = std::min(best, dist[G.idx(i + di, j + dj, k + dk)] +
                                                  std::sqrt(double(di * di + dj * dj + dk * dk)));
                    }
                    dist[v] = best;
                }
    }
    auto isInside = [&](const V3 &x) {
        int q[3];
        for (int a = 0; a < 3; ++a) {
            q[a] = int(std::lround((x[a] - G.o[a]) / G.h));
            if (q[a] < 0 || q[a] >= G.dim(a)) return false;
        }
        return kind[G.idx(q[0], q[1], q[2])] == 1;
    };

    // ---- step 4: skeleton from the deepest voxel along +-grad uH
    std::vector<V3> skel;
    std::vector<double> skelU;
    {
        int seed = 0;
        for (size_t v = 0; v < G.size(); ++v) if (kind[v] == 1 && dist[v] > dist[seed]) seed = int(v);
        const V3 x0 = G.pos(seed % G.nx, (seed / G.nx) % G.ny, seed / (G.nx * G.ny));
        const double uLo = candL[c0], uHi = candL[c1];
        const int maxSteps = 40 * opt.voxels;
        auto trace = [&](double sgn) {
            std::vector<V3> out;
            V3 x = x0;
            for (int st = 0; st < maxSteps; ++st) {
                V3 g = G.grad(uH, x);
                const double gl = len(g);
                if (gl < 1e-12) break;
                const V3 mid = add(x, mul(g, sgn * 0.25 * G.h / gl));
                V3 gm = G.grad(uH, mid);
                const double gml = len(gm);
                if (gml < 1e-12) break;
                x = add(x, mul(gm, sgn * 0.5 * G.h / gml));
                // Keep centred: climb the distance field across the level set.
                const V3 n = mul(gm, 1.0 / gml);
                for (int c = 0; c < 2; ++c) {
                    V3 gd = G.grad(dist, x);
                    gd = sub(gd, mul(n, dot(gd, n)));
                    V3 stepV = mul(gd, 0.25 * G.h * G.h);
                    const double sl = len(stepV);
                    if (sl > 0.25 * G.h) stepV = mul(stepV, 0.25 * G.h / sl);
                    x = add(x, stepV);
                }
                if (!isInside(x)) break;
                out.push_back(x);
                const double uv = G.sample(uH, x);
                if ((sgn > 0 && uv > uHi + 0.01) || (sgn < 0 && uv < uLo - 0.01)) break;
            }
            return out;
        };
        std::vector<V3> back = trace(-1.0), fwd = trace(1.0);
        std::reverse(back.begin(), back.end());
        skel = back;
        skel.push_back(x0);
        skel.insert(skel.end(), fwd.begin(), fwd.end());
        double run = -1e300;
        for (const V3 &x : skel) {
            run = std::max(run, G.sample(uH, x));
            skelU.push_back(run);
        }
    }
    if (skel.size() < 4) {
        R.error = QStringLiteral("The skeleton could not be traced (uH has no gradient inside).");
        return R;
    }

    // A cap end: the skeleton starts at the cap's centre, u = 0, and ends at
    // the tip, u = 1.
    if (opt.capStart || opt.closedEnds) {
        skel.insert(skel.begin(), S.p[vMin]);
        skelU.insert(skelU.begin(), 0.0);
        skel.push_back(S.p[vMax]);
        skelU.push_back(1.0);
    }
    // The levels the skeleton reaches: the hex rows must end on it.
    while (c0 < c1 && candL[c0] < skelU.front()) ++c0;
    while (c1 > c0 && candL[c1] > skelU.back()) --c1;
    if (c1 - c0 < 4) {
        R.error = QStringLiteral("The skeleton spans only u %1..%2 - too short to fit.")
                      .arg(skelU.front(), 0, 'f', 3).arg(skelU.back(), 0, 'f', 3);
        return R;
    }
    for (int c = c0; c <= c1; ++c) R.branchLevels += candBranch[c];

    auto skelAt = [&](double L) {
        for (size_t m = 1; m < skel.size(); ++m)
            if (skelU[m] >= L) {
                const double du = skelU[m] - skelU[m - 1];
                const double t = du > 0 ? std::clamp((L - skelU[m - 1]) / du, 0.0, 1.0) : 0.0;
                return add(skel[m - 1], mul(sub(skel[m], skel[m - 1]), t));
            }
        return skel.back();
    };

    // ---- step 2b: the nu levels, evenly spaced along the loops' centroid path
    std::vector<double> arc(M, 0.0);
    for (int c = c0 + 1; c <= c1; ++c) arc[c] = arc[c - 1] + (candOk[c] && candOk[c - 1] ? len(sub(candC[c], candC[c - 1])) : 0.0);
    const int nu = opt.nu, nv = opt.nv, nw = opt.nw;
    std::vector<double> level(nu);
    std::vector<V3> levelC(nu);
    std::vector<double> levelP(nu);
    for (int i = 0; i < nu; ++i) {
        const double target = arc[c1] * i / (nu - 1);
        int c = c0;
        while (c < c1 && arc[c + 1] < target) ++c;
        const int cn = std::min(c + 1, c1);
        const double da = arc[cn] - arc[c];
        const double t = da > 0 ? std::clamp((target - arc[c]) / da, 0.0, 1.0) : 0.0;
        level[i] = candL[c] + t * (candL[cn] - candL[c]);
        levelC[i] = add(candC[c], mul(sub(candC[cn], candC[c]), t));
        levelP[i] = candPerim[c] + t * (candPerim[cn] - candPerim[c]);
    }
    // A part cut off by the split runs from its cap ring (u = 0) all the way
    // to its tip (u = 1): the last row collapses onto the tip vertex - a pole,
    // like the poles of Elber's sphere. Stopping at 12% of the widest loop
    // (the paper) left flat disks where hooves, snout and tail end.
    std::vector<char> collapsed(nu, 0);
    if (opt.capStart) {
        level[0] = 1e-6;                                      // the cap's ring itself
        level[nu - 1] = 1.0;
        collapsed[nu - 1] = 1;
    } else if (opt.closedEnds) {
        level[0] = 0.0;
        level[nu - 1] = 1.0;
        collapsed[0] = collapsed[nu - 1] = 1;
    }
    // The pole a collapsed row sits on.
    auto poleOf = [&](int i) { return (opt.closedEnds && !opt.capStart && i < nu / 2) ? S.p[vMin] : S.p[vMax]; };
    R.notes << QStringLiteral("u levels %1 .. %2 (%3)")
                   .arg(level.front(), 0, 'f', 3).arg(level.back(), 0, 'f', 3)
                   .arg(opt.capStart ? QStringLiteral("cap ring to tip")
                                     : opt.closedEnds ? QStringLiteral("pole to pole") : QStringLiteral("caps around the poles left out"));
    if (R.branchLevels > 0)
        R.notes << QStringLiteral("%1 of %2 scanned levels have more than one loop: the model branches, "
                                  "only the main tube is fitted").arg(R.branchLevels).arg(c1 - c0 + 1);

    // Boundary points X(u_i, v_j): where the conjugate harmonic v takes the
    // values j / nv on each loop. v is unwrapped round the loop (it grows by 1).
    std::vector<V3> X(size_t(nu) * nv);
    {
        for (int i = 0; i < nu; ++i) {
            if (collapsed[i]) {
                for (int j = 0; j < nv; ++j) X[size_t(i) + size_t(nu) * j] = poleOf(i);
                continue;
            }
            const std::vector<Loop> loops = levelLoops(S, u, level[i]);
            const int bi = mainLoop(loops, sv, levelC[i], levelP[i]);
            const bool nearPole = (opt.capStart && i > nu / 2) || (opt.closedEnds && (i <= 2 || i >= nu - 3));
            if (bi < 0 && nearPole) {                          // a tiny loop next to a pole: collapse it too
                collapsed[i] = 1;
                for (int j = 0; j < nv; ++j) X[size_t(i) + size_t(nu) * j] = poleOf(i);
                continue;
            }
            if (bi < 0) {
                R.error = QStringLiteral("Level u = %1 has no loop.").arg(level[i], 0, 'f', 3);
                return R;
            }
            Loop L = loops[bi];
            const size_t n = L.pts.size();
            std::vector<double> V(n + 1);
            V[0] = vOnEdge(sv, L.ends[0], L.tt[0]);
            for (size_t m = 1; m <= n; ++m) {
                const double raw = vOnEdge(sv, L.ends[m % n], L.tt[m % n]);
                double d = raw - V[m - 1];
                d -= std::round(d);                      // unwrap
                V[m] = V[m - 1] + d;
            }
            double total = V[n] - V[0];
            if (total < 0) {                             // walk the way v grows
                // Point m of the reversed loop is old point (n - m) % n; V
                // holds -v there, which grows along the reversed walk.
                std::vector<V3> P2(n);
                std::vector<double> V2(n + 1);
                for (size_t m = 0; m < n; ++m) P2[m] = L.pts[(n - m) % n];
                for (size_t m = 0; m <= n; ++m) V2[m] = -V[n - m];
                L.pts.swap(P2);
                V.swap(V2);
                total = -total;
            }
            if (std::fabs(total - 1.0) > 0.25 && nearPole) {
                collapsed[i] = 1;
                for (int j = 0; j < nv; ++j) X[size_t(i) + size_t(nu) * j] = poleOf(i);
                continue;
            }
            if (std::fabs(total - 1.0) > 0.25) {
                R.error = QStringLiteral("v does not wind once round the loop at u = %1 (%2) - "
                                         "the seam or the loop is broken there.")
                              .arg(level[i], 0, 'f', 3).arg(total, 0, 'f', 2);
                return R;
            }
            for (int j = 0; j < nv; ++j) {
                double tau = double(j) / nv;
                tau += std::ceil(V[0] - tau);            // into [V0, V0 + 1)
                size_t m = 0;
                while (m + 1 < n && V[m + 1] < tau) ++m;
                const double dv = V[m + 1] - V[m];
                const double f = dv > 0 ? std::clamp((tau - V[m]) / dv, 0.0, 1.0) : 0.0;
                X[size_t(i) + size_t(nu) * j] = add(L.pts[m], mul(sub(L.pts[(m + 1) % n], L.pts[m]), f));
            }
        }
    }

    // ---- step 5: w, 0 on the boundary and 1 on the skeleton
    std::vector<double> w(G.size(), 0.0);
    std::vector<char> wUnknown = unknown;
    for (size_t m = 0; m + 1 < skel.size(); ++m) {
        const double l = len(sub(skel[m + 1], skel[m]));
        const int steps = std::max(1, int(std::ceil(l / (0.5 * G.h))));
        for (int s = 0; s <= steps; ++s) {
            const V3 x = add(skel[m], mul(sub(skel[m + 1], skel[m]), double(s) / steps));
            int q[3];
            for (int a = 0; a < 3; ++a) q[a] = std::clamp(int(std::lround((x[a] - G.o[a]) / G.h)), 0, G.dim(a) - 1);
            const int v = G.idx(q[0], q[1], q[2]);
            if (kind[v] == 1) { w[v] = 1.0; wUnknown[v] = 0; }
        }
    }
    for (size_t v = 0; v < G.size(); ++v) if (wUnknown[v]) w[v] = 0.5;
    const int itW = solveGrid(G, wUnknown, w);
    R.notes << QStringLiteral("w harmonic: %1 CG iterations, skeleton %2 points").arg(itW).arg(skel.size());

    // ---- step 6: w-paths into the hex grid
    std::vector<std::vector<V3>> paths(size_t(nu) * nv);
    std::vector<char> straight(size_t(nu) * nv, 0);
    std::vector<V3> P(size_t(nu) * nv * nw);
    auto at = [&](int i, int j, int k) -> V3 & { return P[size_t(i) + size_t(nu) * (size_t(j) + size_t(nv) * k)]; };
    for (int i = 0; i < nu; ++i) {
        const V3 target = collapsed[i] ? poleOf(i) : skelAt(level[i]);
        for (int j = 0; j < nv; ++j) {
            const V3 x0 = X[size_t(i) + size_t(nu) * j];
            if (collapsed[i]) {                               // the tip: every layer on it
                paths[size_t(i) + size_t(nu) * j] = { x0, target };
                straight[size_t(i) + size_t(nu) * j] = 1;
                continue;
            }
            if (opt.capStart && i == 0) {
                // On the cap: straight from the ring to the centre - exactly
                // on the fan triangle between them.
                paths[size_t(i) + size_t(nu) * j] = { x0, target };
                straight[size_t(i) + size_t(nu) * j] = 1;
                continue;
            }
            std::vector<V3> path{ x0 };
            V3 x = x0;
            bool ok = false;
            double plen = 0.0;
            const double direct = len(sub(target, x0));
            // The paper's direction is grad w projected into the level uH = u_i.
            // Near the surface it starts along plain grad w instead - normal to
            // the surface, since w = 0 there - and blends into the projected
            // one over the first `normalStart` of the way: where the level set
            // meets the surface obliquely (a crease at a leg), a path that
            // starts inside the level set leaves sideways and folds the outer
            // hex cells.
            const double blendLen = opt.normalStart * direct;
            auto dirAt = [&](const V3 &y, double beta, V3 *out) {
                V3 g = G.grad(w, y);
                const double g0 = len(g);
                if (g0 < 1e-12) return false;
                g = mul(g, 1.0 / g0);
                V3 n = G.grad(uH, y);
                const double nl = len(n);
                V3 gp = g;
                if (nl > 1e-12) { n = mul(n, 1.0 / nl); gp = sub(g, mul(n, dot(g, n))); }
                const double gl = len(gp);
                if (gl < 1e-12) return false;
                const V3 d = add(mul(g, 1.0 - beta), mul(gp, beta / gl));
                const double dl = len(d);
                if (dl < 1e-12) return false;
                *out = mul(d, 1.0 / dl);
                return true;
            };
            for (int st = 0; st < 12 * opt.voxels; ++st) {
                const double beta = blendLen > 0 ? std::clamp(plen / blendLen, 0.0, 1.0) : 1.0;
                V3 d0, d1;
                if (!dirAt(x, beta, &d0) || !dirAt(add(x, mul(d0, 0.25 * G.h)), beta, &d1)) break;
                V3 y = add(x, mul(d1, 0.5 * G.h));
                // Stay on the level uH = u_i (drift correction, one Newton step),
                // pulled back in gradually as the blend completes.
                const V3 gu = G.grad(uH, y);
                const double gg = dot(gu, gu);
                if (gg > 1e-20 && beta > 0.0) {
                    V3 corr = mul(gu, -beta * (G.sample(uH, y) - level[i]) / gg);
                    const double cl = len(corr);
                    if (cl > 0.25 * G.h) corr = mul(corr, 0.25 * G.h / cl);
                    y = add(y, corr);
                }
                plen += len(sub(y, x));
                x = y;
                path.push_back(x);
                if (len(sub(x, target)) < 1.5 * G.h || G.sample(w, x) > 0.97) { ok = true; break; }
                if (plen > 3.0 * direct + 6.0 * G.h) break;
            }
            if (ok) {
                path.push_back(target);
            } else {
                path = { x0, target };
                straight[size_t(i) + size_t(nu) * j] = 1;
                ++R.fallbackPaths;
            }
            paths[size_t(i) + size_t(nu) * j] = std::move(path);
        }
    }

    // Resample every path where w crosses the same levels w_k, so hex layer k
    // lies on one w-isosurface - nested shells from the surface in to the
    // skeleton. (Resampling by arclength put layer 1 at different depths on
    // neighbouring paths and folded the outer cells.) w_k is the mean w at
    // arclength fraction k/(nw-1), which keeps the layers roughly even.
    {
        std::vector<std::vector<double>> wv(paths.size()), sv(paths.size());
        for (size_t q = 0; q < paths.size(); ++q) {
            const auto &pth = paths[q];
            wv[q].resize(pth.size());
            sv[q].assign(pth.size(), 0.0);
            double run = 0.0;
            for (size_t m = 0; m < pth.size(); ++m) {
                run = std::max(run, m == 0 ? 0.0 : (m + 1 == pth.size() ? 1.0 : G.sample(w, pth[m])));
                wv[q][m] = run;                              // monotone along the path
                if (m) sv[q][m] = sv[q][m - 1] + len(sub(pth[m], pth[m - 1]));
            }
        }
        std::vector<double> wk(nw, 0.0);
        wk[nw - 1] = 1.0;
        size_t traced = 0;
        for (size_t q = 0; q < paths.size(); ++q) traced += !straight[q];
        for (int k = 1; k < nw - 1; ++k) {
            double sum = 0.0;
            for (size_t q = 0; q < paths.size(); ++q) {
                if (straight[q]) continue;                   // w along a straight line means nothing
                const double t = sv[q].back() * k / (nw - 1);
                size_t m = 0;
                while (m + 2 < paths[q].size() && sv[q][m + 1] < t) ++m;
                const double ds = sv[q][m + 1] - sv[q][m];
                const double f = ds > 0 ? std::clamp((t - sv[q][m]) / ds, 0.0, 1.0) : 0.0;
                sum += wv[q][m] + f * (wv[q][m + 1] - wv[q][m]);
            }
            const double mean = traced ? sum / traced : double(k) / (nw - 1);
            wk[k] = std::clamp(mean, wk[k - 1] + 1e-3, 1.0 - 1e-3 * (nw - 1 - k));
        }
        // Straight paths (the cap row, fallbacks) take the traced paths' mean
        // arclength fraction at each w level.
        std::vector<double> fk(nw, 0.0);
        fk[nw - 1] = 1.0;
        for (int k = 1; k < nw - 1; ++k) {
            double sum = 0.0;
            for (size_t q = 0; q < paths.size(); ++q) {
                if (straight[q]) continue;
                size_t m = 0;
                while (m + 2 < paths[q].size() && wv[q][m + 1] < wk[k]) ++m;
                const double dw = wv[q][m + 1] - wv[q][m];
                const double f = dw > 0 ? std::clamp((wk[k] - wv[q][m]) / dw, 0.0, 1.0) : 0.0;
                sum += (sv[q][m] + f * (sv[q][m + 1] - sv[q][m])) / std::max(1e-30, sv[q].back());
            }
            fk[k] = traced ? sum / traced : double(k) / (nw - 1);
        }
        for (int i = 0; i < nu; ++i)
            for (int j = 0; j < nv; ++j) {
                const size_t q = size_t(i) + size_t(nu) * j;
                const auto &pth = paths[q];
                if (straight[q]) {
                    for (int k = 0; k < nw; ++k) at(i, j, k) = add(pth.front(), mul(sub(pth.back(), pth.front()), fk[k]));
                    continue;
                }
                size_t m = 0;
                for (int k = 0; k < nw; ++k) {
                    while (m + 2 < pth.size() && wv[q][m + 1] < wk[k]) ++m;
                    const double dw = wv[q][m + 1] - wv[q][m];
                    const double f = dw > 0 ? std::clamp((wk[k] - wv[q][m]) / dw, 0.0, 1.0) : 0.0;
                    at(i, j, k) = add(pth[m], mul(sub(pth[m + 1], pth[m]), f));
                }
            }
        QStringList ws;
        for (double x : wk) ws << QString::number(x, 'f', 3);
        R.notes << QStringLiteral("hex layers on w = %1").arg(ws.join(QStringLiteral(", ")));

    }
    if (R.fallbackPaths > 0)
        R.notes << QStringLiteral("%1 of %2 w-paths did not reach the skeleton and were replaced by a straight line")
                       .arg(R.fallbackPaths).arg(nu * nv);

    // Smoothing of the inner layers (the paper's w-path smoothing).
    for (int it = 0; it < opt.smooth; ++it) {
        std::vector<V3> Q = P;
        for (int k = 1; k < nw - 1; ++k)
            for (int j = 0; j < nv; ++j)
                for (int i = opt.capStart ? 1 : 0; i < nu; ++i) {   // the cap row stays on the cap
                    if (collapsed[i]) continue;                         // and the tip stays a point
                    V3 sum = { 0, 0, 0 };
                    int n = 0;
                    auto addN = [&](int a, int b, int c) { sum = add(sum, at(a, b, c)); ++n; };
                    if (i > 0) addN(i - 1, j, k);
                    if (i < nu - 1) addN(i + 1, j, k);
                    addN(i, (j + nv - 1) % nv, k);
                    addN(i, (j + 1) % nv, k);
                    addN(i, j, k - 1);
                    addN(i, j, k + 1);
                    Q[size_t(i) + size_t(nu) * (size_t(j) + size_t(nv) * k)] =
                        add(mul(at(i, j, k), 0.5), mul(sum, 0.5 / n));
                }
        P.swap(Q);
    }

    // Enclosing the part (for the trim): the outer row moves out along the
    // surface normal by delta, the cap row out past the cap, the tip past the
    // tip. The spline fitted to this grid then lies outside the part - pieces
    // cut from it are intersected with the part, which gives them exactly the
    // model's outer surface.
    if (opt.inflate > 0.0) {
        const double delta = opt.inflate * R.size;
        std::vector<V3> vn(S.p.size(), V3{ 0, 0, 0 });
        double v6 = 0.0;
        for (const auto &t : S.t) {
            const V3 nn = cross(sub(S.p[t[1]], S.p[t[0]]), sub(S.p[t[2]], S.p[t[0]]));
            for (int k = 0; k < 3; ++k) vn[t[k]] = add(vn[t[k]], nn);
            v6 += dot(S.p[t[0]], cross(S.p[t[1]], S.p[t[2]]));
        }
        const double sgn = v6 < 0 ? -1.0 : 1.0;             // outward
        for (V3 &x : vn) { const double l = len(x); if (l > 0) x = mul(x, sgn / l); }
        auto normalAt = [&](const V3 &x) {
            int best = 0;
            double bd = 1e300;
            for (int v = 0; v < int(S.p.size()); ++v) {
                const V3 d = sub(S.p[v], x);
                const double dd = dot(d, d);
                if (dd < bd) { bd = dd; best = v; }
            }
            return vn[best];
        };
        auto rowCentre = [&](int i) {
            V3 c = { 0, 0, 0 };
            for (int j = 0; j < nv; ++j) c = add(c, at(i, j, 0));
            return mul(c, 1.0 / nv);
        };
        V3 capOut = { 0, 0, 0 }, tipOut = { 0, 0, 0 };
        if (opt.capStart) {
            capOut = sub(S.p[vMin], rowCentre(1));
            capOut = mul(capOut, 1.0 / std::max(1e-30, len(capOut)));
            int iL = nu - 1;
            while (iL > 0 && collapsed[iL]) --iL;
            tipOut = sub(S.p[vMax], rowCentre(iL));
            tipOut = mul(tipOut, 1.0 / std::max(1e-30, len(tipOut)));
        }
        std::vector<V3> P0 = P;                                // the surface row before moving
        auto at0 = [&](int i, int j) { return P0[size_t(i) + size_t(nu) * size_t(j)]; };
        for (int i = 0; i < nu; ++i)
            for (int j = 0; j < nv; ++j) {
                if (collapsed[i]) {
                    for (int k = 0; k < nw; ++k) at(i, j, k) = add(at(i, j, k), mul(tipOut, delta));
                    continue;
                }
                if (opt.capStart && i == 0) {
                    for (int k = 0; k < nw; ++k) at(0, j, k) = add(at(0, j, k), mul(capOut, delta));
                    at(0, j, 0) = add(at(0, j, 0), mul(normalAt(at0(0, j)), delta));
                    continue;
                }
                at(i, j, 0) = add(at(i, j, 0), mul(normalAt(at0(i, j)), delta));
            }
        R.notes << QStringLiteral("enclosing: outer row moved out by %1% of the part's size").arg(100.0 * opt.inflate, 0, 'f', 1);
    }

    // Right-handed (u, v, w): det J > 0. Reverse v if the loops ran the other way.
    {
        double sum = 0.0;
        for (int k = 0; k + 1 < nw; ++k)
            for (int j = 0; j < nv; ++j)
                for (int i = 0; i + 1 < nu; ++i) {
                    const V3 &p = at(i, j, k);
                    sum += dot(sub(at(i + 1, j, k), p), cross(sub(at(i, (j + 1) % nv, k), p), sub(at(i, j, k + 1), p)));
                }
        if (sum < 0) {
            std::vector<V3> Q = P;
            for (int k = 0; k < nw; ++k)
                for (int j = 0; j < nv; ++j)
                    for (int i = 0; i < nu; ++i)
                        Q[size_t(i) + size_t(nu) * (size_t(j) + size_t(nv) * k)] = at(i, (nv - j) % nv, k);
            P.swap(Q);
        }
    }

    // Folded hex cells (a corner det <= 0), for the report. Corners on the axis
    // (k = nw-1) are degenerate by construction and not counted.
    {
        auto cornerDet = [&](int i, int j, int k, int di, int dj, int dk) {
            const V3 &p = at(i, j, k);
            const V3 a = mul(sub(at(i + di, j, k), p), di);
            const V3 b = mul(sub(at(i, (j + dj + nv) % nv, k), p), dj);
            const V3 c = mul(sub(at(i, j, k + dk), p), dk);
            return dot(a, cross(b, c));
        };
        int bad = 0;
        for (int k = 0; k + 1 < nw; ++k)
            for (int j = 0; j < nv; ++j)
                for (int i = 0; i + 1 < nu; ++i) {
                    bool fold = false;
                    for (int c = 0; c < 8 && !fold; ++c) {
                        const int ck = k + ((c >> 2) & 1);
                        if (ck == nw - 1) continue;
                        fold = cornerDet(i + (c & 1), (j + ((c >> 1) & 1)) % nv, ck, (c & 1) ? -1 : 1,
                                         ((c >> 1) & 1) ? -1 : 1, ((c >> 2) & 1) ? -1 : 1) <= 0;
                    }
                    bad += fold;
                }
        R.notes << QStringLiteral("hex grid: %1 of %2 cells folded").arg(bad).arg((nu - 1) * nv * (nw - 1));
    }

    // ---- step 7: iterative fit (eq. 2-4): each control point moves toward its
    // grid point by lambda * (p - S(greville)) / (B_i(u_i) * B_j(v_j)).
    const std::vector<double> U = openKnots(nu, 4);
    std::vector<std::vector<double>> Bu(nu);
    for (int i = 0; i < nu; ++i) {
        const double g = (U[i + 1] + U[i + 2] + U[i + 3]) / 3.0;
        Bu[i] = basisAt(U, 4, nu, g);
    }
    std::vector<V3> C = P;
    auto ctl = [&](int i, int j, int k) -> const V3 & { return C[size_t(i) + size_t(nu) * (size_t(j) + size_t(nv) * k)]; };
    const double tolAbs = opt.tol * R.size;
    for (R.iterations = 0; R.iterations < opt.maxIter; ++R.iterations) {
        std::vector<V3> D(C.size());
        double maxErr = 0.0;
        for (int k = 0; k < nw; ++k)
            for (int j = 0; j < nv; ++j)
                for (int i = 0; i < nu; ++i) {
                    V3 s = { 0, 0, 0 };
                    for (int l = std::max(0, i - 3); l <= std::min(nu - 1, i + 3); ++l) {
                        if (Bu[i][l] == 0.0) continue;
                        const V3 cv = add(add(mul(ctl(l, (j + nv - 1) % nv, k), 1.0 / 6), mul(ctl(l, j, k), 2.0 / 3)),
                                          mul(ctl(l, (j + 1) % nv, k), 1.0 / 6));
                        s = add(s, mul(cv, Bu[i][l]));
                    }
                    const V3 e = sub(at(i, j, k), s);
                    maxErr = std::max(maxErr, len(e));
                    D[size_t(i) + size_t(nu) * (size_t(j) + size_t(nv) * k)] = mul(e, 1.0 / (Bu[i][i] * (2.0 / 3)));
                }
        R.fitError = maxErr / R.size;
        if (maxErr < tolAbs) break;
        for (size_t m = 0; m < C.size(); ++m) C[m] = add(C[m], mul(D[m], opt.lambda));
    }
    R.notes << QStringLiteral("B-spline fit: %1 iterations, max error %2% of the model size")
                   .arg(R.iterations).arg(100.0 * R.fitError, 0, 'f', 2);

    R.nu = nu; R.nv = nv; R.nw = nw;
    R.ctrl.resize(int(C.size()) * 3);
    R.grid.resize(int(P.size()) * 3);
    for (size_t m = 0; m < C.size(); ++m)
        for (int a = 0; a < 3; ++a) { R.ctrl[int(3 * m) + a] = C[m][a]; R.grid[int(3 * m) + a] = P[m][a]; }
    R.ok = true;
    return R;
}

Trivariate HarmonicFit::toTrivariate(const Result &r, const QString &label, QString *error)
{
    if (!r.ok || r.ctrl.size() != 3 * r.nu * r.nv * r.nw) {
        if (error) *error = QStringLiteral("Nothing to build: the fit did not succeed.");
        return Trivariate();
    }
    BuildCtx c{ r.ctrl.constData(), r.nu, r.nv, r.nw, NULL };
    if (!IritGuard::run(&c, doBuild) || c.result == NULL) {
        if (error) *error = QStringLiteral("IRIT could not build the trivariate: %1").arg(IritGuard::lastError());
        return Trivariate();
    }
    return Trivariate::adopt(c.result, label);
}

HarmonicFit::Check HarmonicFit::checkJacobian(const Trivariate &tv, int n)
{
    Check ck;
    if (!tv.isValid()) return ck;
    double d[6];
    tv.domain(d);
    const double hs[3] = { (d[1] - d[0]) * 1e-4, (d[3] - d[2]) * 1e-4, (d[5] - d[4]) * 1e-4 };
    // det J at a point of D, one-sided at the faces.
    auto detAt = [&](const double p[3]) {
        double J[3][3];
        for (int a = 0; a < 3; ++a) {
            double q0[3] = { p[0], p[1], p[2] }, q1[3] = { p[0], p[1], p[2] };
            q0[a] = std::max(d[2 * a], p[a] - hs[a]);
            q1[a] = std::min(d[2 * a + 1], p[a] + hs[a]);
            double m0[3], m1[3];
            tv.evaluate(q0[0], q0[1], q0[2], m0);
            tv.evaluate(q1[0], q1[1], q1[2], m1);
            for (int b = 0; b < 3; ++b) J[b][a] = (m1[b] - m0[b]) / (q1[a] - q0[a]);
        }
        return J[0][0] * (J[1][1] * J[2][2] - J[1][2] * J[2][1]) -
               J[0][1] * (J[1][0] * J[2][2] - J[1][2] * J[2][0]) +
               J[0][2] * (J[1][0] * J[2][1] - J[1][1] * J[2][0]);
    };

    // Volume: midpoint rule.
    const double cellVol = (d[1] - d[0]) * (d[3] - d[2]) * (d[5] - d[4]) / (double(n) * n * n);
    for (int k = 0; k < n; ++k)
        for (int j = 0; j < n; ++j)
            for (int i = 0; i < n; ++i) {
                const double p[3] = { d[0] + (i + 0.5) / n * (d[1] - d[0]), d[2] + (j + 0.5) / n * (d[3] - d[2]),
                                      d[4] + (k + 0.5) / n * (d[5] - d[4]) };
                ck.volume += detAt(p) * cellVol;
            }

    // Sign: a grid that includes the faces (the analyzer samples them too),
    // except w = 1, the skeleton, where det J is 0 by construction.
    std::vector<double> dets;
    std::vector<std::array<double, 3>> pts;
    for (int k = 0; k < n; ++k)
        for (int j = 0; j <= n; ++j)
            for (int i = 0; i <= n; ++i) {
                const double p[3] = { d[0] + double(i) / n * (d[1] - d[0]), d[2] + double(j) / n * (d[3] - d[2]),
                                      d[4] + double(k) / n * (d[5] - d[4]) };
                const double det = detAt(p);
                dets.push_back(det);
                pts.push_back({ p[0], p[1], p[2] });
            }
    ck.samples = int(dets.size());
    double mean = 0.0;
    for (double x : dets) mean += x;
    mean /= std::max<size_t>(1, dets.size());
    // A fold is det J < 0. det J = 0 where a face collapses on purpose - the
    // tip of a part (a pole) - is not one; finite differences there give
    // values of either sign around zero.
    const double tolDet = 1e-4 * std::fabs(mean);
    for (size_t q = 0; q < dets.size(); ++q)
        if (dets[q] < -tolDet) {
            ++ck.nonPositive;
            if (ck.bad.size() < 30) ck.bad << pts[q][0] << pts[q][1] << pts[q][2];
        }
    const double mn = *std::min_element(dets.begin(), dets.end());
    ck.minRatio = mean != 0 ? mn / mean : 0.0;
    return ck;
}

HarmonicFit::Deviation HarmonicFit::surfaceDeviation(const Trivariate &tv, const MeshData &mesh, int n)
{
    Deviation dv;
    if (!tv.isValid() || mesh.vertexCount() == 0) return dv;
    double d[6];
    tv.domain(d);

    // Dense samples of the boundary: the w = 0 face and the two u ends.
    std::vector<V3> pts;
    auto put = [&](double u, double v, double w) {
        double p[3];
        tv.evaluate(u, v, w, p);
        pts.push_back({ p[0], p[1], p[2] });
    };
    for (int i = 0; i <= n; ++i)
        for (int j = 0; j < n; ++j)
            put(d[0] + (d[1] - d[0]) * i / n, d[2] + (d[3] - d[2]) * j / n, d[4]);
    const int ne = n / 2;
    for (int e = 0; e < 2; ++e)
        for (int j = 0; j < n; ++j)
            for (int k = 1; k <= ne; ++k)
                put(e ? d[1] : d[0], d[2] + (d[3] - d[2]) * j / n, d[4] + (d[5] - d[4]) * k / ne);

    V3 lo = { 1e300, 1e300, 1e300 }, hi = { -1e300, -1e300, -1e300 };
    for (int i = 0; i < mesh.vertexCount(); ++i)
        for (int a = 0; a < 3; ++a) {
            lo[a] = std::min(lo[a], double(mesh.pos[3 * i + a]));
            hi[a] = std::max(hi[a], double(mesh.pos[3 * i + a]));
        }
    for (const V3 &p : pts)
        for (int a = 0; a < 3; ++a) { lo[a] = std::min(lo[a], p[a]); hi[a] = std::max(hi[a], p[a]); }
    const double size = len(sub(hi, lo));
    const int B = 48;
    double cell = 0.0;
    for (int a = 0; a < 3; ++a) cell = std::max(cell, (hi[a] - lo[a]) / B);
    cell = std::max(cell, 1e-12);
    int nb[3];
    for (int a = 0; a < 3; ++a) nb[a] = int((hi[a] - lo[a]) / cell) + 1;
    auto cid = [&](const V3 &p, int a) { return std::clamp(int((p[a] - lo[a]) / cell), 0, nb[a] - 1); };
    std::vector<std::vector<int>> bucket(size_t(nb[0]) * nb[1] * nb[2]);
    for (int m = 0; m < int(pts.size()); ++m)
        bucket[size_t(cid(pts[m], 0)) + size_t(nb[0]) * (cid(pts[m], 1) + size_t(nb[1]) * cid(pts[m], 2))].push_back(m);

    // Every vertex (at most ~20000, evenly strided).
    const int nv = mesh.vertexCount();
    const int stride = std::max(1, nv / 20000);
    std::vector<double> dist;
    for (int i = 0; i < nv; i += stride) {
        const V3 x = { mesh.pos[3 * i], mesh.pos[3 * i + 1], mesh.pos[3 * i + 2] };
        const int c[3] = { cid(x, 0), cid(x, 1), cid(x, 2) };
        double best = 1e300;
        for (int r = 0; r < std::max({ nb[0], nb[1], nb[2] }); ++r) {
            for (int dk = -r; dk <= r; ++dk)
                for (int dj = -r; dj <= r; ++dj)
                    for (int di = -r; di <= r; ++di) {
                        if (std::max({ std::abs(di), std::abs(dj), std::abs(dk) }) != r) continue;
                        const int q[3] = { c[0] + di, c[1] + dj, c[2] + dk };
                        if (q[0] < 0 || q[1] < 0 || q[2] < 0 || q[0] >= nb[0] || q[1] >= nb[1] || q[2] >= nb[2]) continue;
                        for (int m : bucket[size_t(q[0]) + size_t(nb[0]) * (q[1] + size_t(nb[1]) * q[2])])
                            best = std::min(best, len(sub(pts[m], x)));
                    }
            if (best < 1e299 && (r - 1) * cell > best) break;
        }
        dist.push_back(best / size);
    }
    if (dist.empty()) return dv;
    double sum = 0.0;
    int over = 0;
    for (double x : dist) { sum += x; if (x > 0.01) ++over; }
    dv.mean = sum / dist.size();
    dv.over1 = double(over) / dist.size();
    std::sort(dist.begin(), dist.end());
    dv.p95 = dist[size_t(0.95 * (dist.size() - 1))];
    dv.max = dist.back();
    return dv;
}

namespace {

// The fitted control net evaluated without IRIT: open cubic in u, closed
// (float, wrapped) cubic in v, linear in w - the same B-spline toTrivariate
// builds, domain [0,1]^3.
struct Net {
    int nu = 0, nv = 0, nw = 0;
    std::vector<V3> C;                    // i + nu * (j + nv * k)
    std::vector<double> U, V, W;

    explicit Net(const HarmonicFit::Result &r) : nu(r.nu), nv(r.nv), nw(r.nw), C(size_t(r.nu) * r.nv * r.nw)
    {
        for (size_t m = 0; m < C.size(); ++m) C[m] = { r.ctrl[int(3 * m)], r.ctrl[int(3 * m) + 1], r.ctrl[int(3 * m) + 2] };
        U = openKnots(nu, 4);
        W = openKnots(nw, 2);
        V.resize(nv + 7);
        for (int m = 0; m < nv + 7; ++m) V[m] = double(m - 3) / nv;
    }
    V3 &at(int i, int j, int k) { return C[size_t(i) + size_t(nu) * (size_t(j) + size_t(nv) * k)]; }
    V3 eval(double u, double v, double w) const
    {
        u = std::clamp(u, 0.0, 1.0);
        v = std::clamp(v, 0.0, 1.0);
        w = std::clamp(w, 0.0, 1.0);
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
        lo[a] = std::max(0.0, lo[a] - h);
        hi[a] = std::min(1.0, hi[a] + h);
        return mul(sub(eval(hi[0], hi[1], hi[2]), eval(lo[0], lo[1], lo[2])), 1.0 / (hi[a] - lo[a]));
    }
    // Outward unit normal of the boundary face through (u, v, w); face 0 = w = 0,
    // 1 = u = 0, 2 = u = 1.
    V3 normal(double u, double v, double w, int face) const
    {
        const V3 mu = d(u, v, w, 0), mv = d(u, v, w, 1), mw = d(u, v, w, 2);
        V3 n;
        if (face == 0) { n = cross(mu, mv); if (dot(n, mw) > 0) n = mul(n, -1.0); }
        else { n = cross(mv, mw); if ((dot(n, mu) > 0) == (face == 1)) n = mul(n, -1.0); }
        const double l = len(n);
        return l > 0 ? mul(n, 1.0 / l) : V3{ 0, 0, 0 };
    }
};

}  // namespace

namespace {

// Boundary samples of a net (w = 0 side, u = 0 / u = 1 ends) with outward
// normals, and for every model vertex the nearest one and how far outside it
// the vertex lies (depth > 0: outside).
struct BoundaryProbe {
    struct Sample { V3 p, n; double u, v; int face; double w = 0.0; };
    std::vector<Sample> S;
    std::vector<int> nearest;     // per vertex
    std::vector<double> depth;    // per vertex

    BoundaryProbe(const Net &net, const std::vector<V3> &X)
    {
        const int Au = 5 * net.nu, Bv = 5 * net.nv, Cw = 4 * net.nw;
        for (int a = 0; a <= Au; ++a)
            for (int b = 0; b < Bv; ++b) {
                const double u = double(a) / Au, v = double(b) / Bv;
                S.push_back({ net.eval(u, v, 0.0), net.normal(u, v, 0.0, 0), u, v, 0 });
            }
        for (int e = 0; e < 2; ++e)
            for (int b = 0; b < Bv; ++b)
                for (int c = 0; c < Cw; ++c) {            // w = 1 is the axis: no normal there
                    const double v = double(b) / Bv, w = double(c) / Cw;
                    S.push_back({ net.eval(e, v, w), net.normal(e, v, w, 1 + e), double(e), v, 1 + e, w });
                }
        V3 lo = { 1e300, 1e300, 1e300 }, hi = { -1e300, -1e300, -1e300 };
        for (const auto &q : S) for (int a = 0; a < 3; ++a) { lo[a] = std::min(lo[a], q.p[a]); hi[a] = std::max(hi[a], q.p[a]); }
        for (const auto &x : X) for (int a = 0; a < 3; ++a) { lo[a] = std::min(lo[a], x[a]); hi[a] = std::max(hi[a], x[a]); }
        double cell = 0.0;
        for (int a = 0; a < 3; ++a) cell = std::max(cell, (hi[a] - lo[a]) / 40.0);
        cell = std::max(cell, 1e-12);
        int nb[3];
        for (int a = 0; a < 3; ++a) nb[a] = int((hi[a] - lo[a]) / cell) + 1;
        auto cid = [&](const V3 &p, int a) { return std::clamp(int((p[a] - lo[a]) / cell), 0, nb[a] - 1); };
        std::vector<std::vector<int>> bucket(size_t(nb[0]) * nb[1] * nb[2]);
        for (int m = 0; m < int(S.size()); ++m)
            bucket[size_t(cid(S[m].p, 0)) + size_t(nb[0]) * (cid(S[m].p, 1) + size_t(nb[1]) * cid(S[m].p, 2))].push_back(m);
        nearest.assign(X.size(), -1);
        depth.assign(X.size(), -1e300);
        for (size_t xi = 0; xi < X.size(); ++xi) {
            const V3 &x = X[xi];
            const int c[3] = { cid(x, 0), cid(x, 1), cid(x, 2) };
            double best = 1e300;
            int bi = -1;
            for (int rr = 0; rr < std::max({ nb[0], nb[1], nb[2] }); ++rr) {
                for (int dk = -rr; dk <= rr; ++dk)
                    for (int dj = -rr; dj <= rr; ++dj)
                        for (int di = -rr; di <= rr; ++di) {
                            if (std::max({ std::abs(di), std::abs(dj), std::abs(dk) }) != rr) continue;
                            const int q[3] = { c[0] + di, c[1] + dj, c[2] + dk };
                            if (q[0] < 0 || q[1] < 0 || q[2] < 0 || q[0] >= nb[0] || q[1] >= nb[1] || q[2] >= nb[2]) continue;
                            for (int m : bucket[size_t(q[0]) + size_t(nb[0]) * (q[1] + size_t(nb[1]) * q[2])]) {
                                const double dd = len(sub(S[m].p, x));
                                if (dd < best) { best = dd; bi = m; }
                            }
                        }
                if (bi >= 0 && (rr - 1) * cell > best) break;
            }
            nearest[xi] = bi;
            if (bi >= 0) depth[xi] = dot(sub(x, S[bi].p), S[bi].n);
        }
    }
};

}  // namespace

// The fit itself is left as it is (its det J, its cuts); material is ADDED
// around it: one new outer layer in w, offset along the fit's own surface
// normals by what each spot needs, and one new row past each u end,
// translated along the end's mean normal - a prism over the cap the fit left
// out. Pushing the fit's own layer outward instead was measured to bend it
// into folds (spot 493, bimba 19 folded samples) and never enclosed spot.
bool HarmonicFit::enclose(Result *r, const MeshData &mesh, QStringList *notes, double margin, int /*rounds*/)
{
    if (!r->ok || mesh.vertexCount() == 0) return false;
    const Net net(*r);
    const int nu = net.nu, nv = net.nv, nw = net.nw;
    const double gap = margin * r->size;

    std::vector<V3> X;
    {
        const int n = mesh.vertexCount();
        const int stride = std::max(1, n / 30000);
        for (int i = 0; i < n; i += stride)
            X.push_back({ mesh.pos[3 * i], mesh.pos[3 * i + 1], mesh.pos[3 * i + 2] });
    }
    const BoundaryProbe probe(net, X);

    // What each boundary sample needs: the deepest vertex outside it, plus the gap.
    std::vector<double> need(probe.S.size(), 0.0);
    double endNeed[2] = { 0.0, 0.0 };
    V3 endDir[2] = { { 0, 0, 0 }, { 0, 0, 0 } };
    for (const auto &q : probe.S)
        if (q.face > 0) endDir[q.face - 1] = add(endDir[q.face - 1], q.n);
    for (int e = 0; e < 2; ++e) {
        const double l = len(endDir[e]);
        if (l > 0) endDir[e] = mul(endDir[e], 1.0 / l);
    }
    for (size_t xi = 0; xi < X.size(); ++xi) {
        const int m = probe.nearest[xi];
        if (m < 0) continue;
        const double nd = probe.depth[xi] + gap;
        if (nd <= 0) continue;
        if (probe.S[m].face == 0) {
            need[m] = std::max(need[m], nd);
        } else {
            // Past an end: how far along the end's mean normal.
            const int e = probe.S[m].face - 1;
            endNeed[e] = std::max(endNeed[e], dot(sub(X[xi], probe.S[m].p), endDir[e]) + gap);
        }
    }

    // Shell thickness per outer control column: at least the gap, and the
    // largest need among the samples in its support facing the same way.
    const double su = 2.0 / (nu - 3), sv = 2.0 / nv;
    std::vector<double> t(size_t(nu) * nv, gap);
    std::vector<V3> nrm(size_t(nu) * nv);
    for (int i = 0; i < nu; ++i)
        for (int j = 0; j < nv; ++j) {
            const double gu = std::clamp((net.U[i + 1] + net.U[i + 2] + net.U[i + 3]) / 3.0, 0.0, 1.0);
            double gv = double(j - 1) / nv;
            if (gv < 0) gv += 1.0;
            const V3 n = net.normal(gu, gv, 0.0, 0);
            nrm[size_t(i) + size_t(nu) * j] = n;
            for (size_t m = 0; m < probe.S.size(); ++m) {
                const auto &q = probe.S[m];
                if (q.face != 0 || need[m] <= 0) continue;
                double dv = std::fabs(q.v - gv);
                dv = std::min(dv, 1.0 - dv);
                if (std::fabs(q.u - gu) <= su && dv <= sv && dot(q.n, n) > 0.5)
                    t[size_t(i) + size_t(nu) * j] = std::max(t[size_t(i) + size_t(nu) * j], need[m]);
            }
        }

    // New net: (nu + 2) x nv x (nw + 1).
    const int NU = nu + 2, NW = nw + 1;
    std::vector<V3> C(size_t(NU) * nv * NW);
    auto old = [&](int i, int j, int k) { return net.C[size_t(i) + size_t(nu) * (size_t(j) + size_t(nv) * k)]; };
    auto put = [&](int i, int j, int k, const V3 &p) { C[size_t(i) + size_t(NU) * (size_t(j) + size_t(nv) * k)] = p; };
    const double e0 = std::max(endNeed[0], gap), e1 = std::max(endNeed[1], gap);
    for (int j = 0; j < nv; ++j) {
        for (int k = 0; k < nw; ++k) {
            for (int i = 0; i < nu; ++i) put(i + 1, j, k + 1, old(i, j, k));
            put(0, j, k + 1, add(old(0, j, k), mul(endDir[0], e0)));
            put(NU - 1, j, k + 1, add(old(nu - 1, j, k), mul(endDir[1], e1)));
        }
        for (int i = 0; i < nu; ++i)
            put(i + 1, j, 0, add(old(i, j, 0), mul(nrm[size_t(i) + size_t(nu) * j], t[size_t(i) + size_t(nu) * j])));
        put(0, j, 0, add(add(old(0, j, 0), mul(endDir[0], e0)), mul(nrm[size_t(nu) * j], t[size_t(nu) * j])));
        put(NU - 1, j, 0, add(add(old(nu - 1, j, 0), mul(endDir[1], e1)),
                              mul(nrm[size_t(nu - 1) + size_t(nu) * j], t[size_t(nu - 1) + size_t(nu) * j])));
    }

    Result out = *r;
    out.nu = NU;
    out.nw = NW;
    out.ctrl.resize(int(C.size()) * 3);
    for (size_t m = 0; m < C.size(); ++m)
        for (int a = 0; a < 3; ++a) out.ctrl[int(3 * m) + a] = C[m][a];

    // Verify on the new net.
    const BoundaryProbe check(Net(out), X);
    int outside = 0;
    double worst = 0.0;
    for (size_t xi = 0; xi < X.size(); ++xi)
        if (check.depth[xi] > 0) { ++outside; worst = std::max(worst, check.depth[xi]); }

    double tmax = 0.0;
    for (double x : t) tmax = std::max(tmax, x);
    if (notes)
        *notes << QStringLiteral("enclosing: shell up to %1%, ends extended %2% / %3% of the model size; "
                                 "%4 of %5 model vertices outside%6")
                      .arg(100.0 * tmax / r->size, 0, 'f', 1)
                      .arg(100.0 * e0 / r->size, 0, 'f', 1).arg(100.0 * e1 / r->size, 0, 'f', 1)
                      .arg(outside).arg(X.size())
                      .arg(outside ? QStringLiteral(" (worst %1% out)").arg(100.0 * worst / r->size, 0, 'f', 1)
                                   : QString());
    *r = out;
    return outside == 0;
}

// ================================================================== Morse split
namespace {

// Dijkstra over mesh edges from `src`; stops past `radius` (if > 0).
std::vector<double> geodesic(const Surf &s, const Nbr &nbr, const std::vector<int> &src, double radius)
{
    std::vector<double> d(s.p.size(), 1e300);
    std::vector<std::pair<double, int>> heap;
    auto cmp = [](const std::pair<double, int> &a, const std::pair<double, int> &b) { return a.first > b.first; };
    for (int x : src) { d[x] = 0.0; heap.push_back({ 0.0, x }); }
    std::make_heap(heap.begin(), heap.end(), cmp);
    while (!heap.empty()) {
        std::pop_heap(heap.begin(), heap.end(), cmp);
        const auto [dx, x] = heap.back();
        heap.pop_back();
        if (dx > d[x]) continue;
        if (radius > 0 && dx > radius) continue;
        for (const auto &nw : nbr[x]) {
            const double nd = dx + len(sub(s.p[nw.first], s.p[x]));
            if (nd < d[nw.first]) { d[nw.first] = nd; heap.push_back({ nd, nw.first }); std::push_heap(heap.begin(), heap.end(), cmp); }
        }
    }
    return d;
}

}  // namespace

namespace {

// Ring length at levels r = k * step of a distance field d: the total length
// of the level segments over all triangles.
std::vector<double> ringLengths(const Surf &S, const std::vector<double> &d, double step, int levels)
{
    std::vector<double> P(levels, 0.0);
    for (const auto &t : S.t) {
        const double da = d[t[0]], db = d[t[1]], dc = d[t[2]];
        if (da > 1e299 || db > 1e299 || dc > 1e299) continue;
        const double dmin = std::min({ da, db, dc }), dmax = std::max({ da, db, dc });
        for (int k = std::max(1, int(std::ceil(dmin / step))); k <= std::min(levels - 1, int(std::floor(dmax / step))); ++k) {
            const double r = k * step;
            V3 q[2];
            int nq = 0;
            for (int e = 0; e < 3 && nq < 2; ++e) {
                const int x = t[e], y = t[(e + 1) % 3];
                const double fx = d[x] - r, fy = d[y] - r;
                if ((fx < 0) != (fy < 0)) q[nq++] = add(S.p[x], mul(sub(S.p[y], S.p[x]), fx / (fx - fy)));
            }
            if (nq == 2) P[k] += len(sub(q[1], q[0]));
        }
    }
    return P;
}

}  // namespace

namespace {

// A level set of a distance field at r = k * step: its length, how many loops
// it has (counting loops of at least 5% of the length) and how far it is from
// flat (distance from its best plane / its radius).
struct RingStat {
    double P = 0.0, plan = 1e9;
    int    comps = 0;
    V3     cen = { 0, 0, 0 };   // the ring's centre (length-weighted)
};

// Smallest eigenvalue of a symmetric 3x3 matrix (Jacobi sweeps).
double minEig3(double a[3][3])
{
    for (int sweep = 0; sweep < 40; ++sweep) {
        if (std::fabs(a[0][1]) + std::fabs(a[0][2]) + std::fabs(a[1][2]) < 1e-24) break;
        for (int p = 0; p < 2; ++p)
            for (int q = p + 1; q < 3; ++q) {
                if (std::fabs(a[p][q]) < 1e-300) continue;
                const double theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
                const double t = (theta >= 0 ? 1.0 : -1.0) / (std::fabs(theta) + std::sqrt(theta * theta + 1.0));
                const double c = 1.0 / std::sqrt(t * t + 1.0), sn = t * c;
                for (int k = 0; k < 3; ++k) {
                    const double akp = a[k][p], akq = a[k][q];
                    a[k][p] = c * akp - sn * akq;
                    a[k][q] = sn * akp + c * akq;
                }
                for (int k = 0; k < 3; ++k) {
                    const double apk = a[p][k], aqk = a[q][k];
                    a[p][k] = c * apk - sn * aqk;
                    a[q][k] = sn * apk + c * aqk;
                }
            }
    }
    return std::min({ a[0][0], a[1][1], a[2][2] });
}

std::vector<RingStat> ringStats(const Surf &S, const std::vector<double> &d, double step, int levels)
{
    struct Seg { long long ea, eb; V3 a, b; };
    std::vector<std::vector<Seg>> segs(levels);
    for (const auto &t : S.t) {
        const double da = d[t[0]], db = d[t[1]], dc = d[t[2]];
        if (da > 1e299 || db > 1e299 || dc > 1e299) continue;
        const double dmin = std::min({ da, db, dc }), dmax = std::max({ da, db, dc });
        for (int k = std::max(1, int(std::ceil(dmin / step))); k <= std::min(levels - 1, int(std::floor(dmax / step))); ++k) {
            const double r = k * step;
            V3 q[2];
            long long e[2];
            int nq = 0;
            for (int m = 0; m < 3 && nq < 2; ++m) {
                const int x = t[m], y = t[(m + 1) % 3];
                const double fx = d[x] - r, fy = d[y] - r;
                if ((fx < 0) != (fy < 0)) { e[nq] = edgeKey(x, y); q[nq++] = add(S.p[x], mul(sub(S.p[y], S.p[x]), fx / (fx - fy))); }
            }
            if (nq == 2) segs[k].push_back({ e[0], e[1], q[0], q[1] });
        }
    }
    std::vector<RingStat> out(levels);
    for (int k = 0; k < levels; ++k) {
        const auto &sg = segs[k];
        if (sg.empty()) continue;
        std::unordered_map<long long, int> id;
        std::vector<int> par;
        auto get = [&](long long e) {
            auto it = id.find(e);
            if (it != id.end()) return it->second;
            const int x = int(par.size());
            par.push_back(x);
            id.emplace(e, x);
            return x;
        };
        auto find = [&](int x) { while (par[x] != x) x = par[x] = par[par[x]]; return x; };
        double P = 0.0, W = 0.0;
        V3 c = { 0, 0, 0 };
        for (const Seg &g : sg) {
            const int a = find(get(g.ea)), b = find(get(g.eb));
            if (a != b) par[a] = b;
            const double l = len(sub(g.b, g.a));
            P += l;
            c = add(c, mul(add(g.a, g.b), 0.5 * l));
            W += l;
        }
        if (!(W > 0)) continue;
        c = mul(c, 1.0 / W);
        std::unordered_map<int, double> compLen;
        double C[3][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };
        for (const Seg &g : sg) {
            const double l = len(sub(g.b, g.a));
            compLen[find(get(g.ea))] += l;
            const V3 m = sub(mul(add(g.a, g.b), 0.5), c);
            for (int x = 0; x < 3; ++x)
                for (int y = 0; y < 3; ++y) C[x][y] += l * m[x] * m[y] / W;
        }
        RingStat rs;
        rs.P = P;
        rs.cen = c;
        // Loops under 15% of the ring are bumps (a teat, a knuckle), not a second branch.
        for (const auto &kv : compLen) if (kv.second >= 0.15 * P) ++rs.comps;
        const double rad = P / (2.0 * 3.14159265358979323846);
        rs.plan = rad > 0 ? std::sqrt(std::max(0.0, minEig3(C))) / rad : 1e9;
        out[k] = rs;
    }
    return out;
}

// One chosen cut: the ring d < r round a tip.
struct SplitChoice {
    int tip = -1;
    double r = 0.0, area = 0.0, neck = 0.0, plan = 0.0;
    std::vector<double> d;
    std::vector<double> field;   // > 0 inside the limb (its distance from the cut plane), -1e300 far away
};

// The shortlist: (energy, cuts) of up to opt.shortlist different splits, best first.
using SplitList = std::vector<std::pair<double, std::vector<SplitChoice>>>;

SplitList searchSplit(const Surf &S, const Nbr &nbr, const std::vector<double> &area, double totalArea,
                                     double size, int root, const std::vector<double> &g, const std::vector<int> &tips,
                                     const HarmonicFit::SplitOptions &opt, QStringList *notes)
{
    const int n = int(S.p.size()), T = int(tips.size());
    const double step = 0.005 * size;
    const int K = int(opt.maxLength * size / step) + 1;
    const int w = std::max(3, int(std::lround(0.08 * size / step)));
    const int levels = K + w + 2;
    const double kPi = 3.14159265358979323846;
    const bool debug = qEnvironmentVariableIsSet("SPLIT_DEBUG");
    double gap = 0.0;                                  // the longest mesh edge
    for (const auto &t : S.t)
        for (int m = 0; m < 3; ++m) gap = std::max(gap, len(sub(S.p[t[m]], S.p[t[(m + 1) % 3]])));
    const int k0 = std::max(2, int(std::ceil(opt.minPersistence * size / step)));

    // A candidate limb: a group of tips and the rings of its (multi-source)
    // distance field. A head is its muzzle, ears and horns together - a ring
    // grown from one tip of it is too wide to be a tube near the muzzle and
    // falls apart round the ears further on.
    struct Cand {
        std::vector<int> members;              // indices into tips
        std::vector<double> d;
        std::vector<double> unary, A, neck, plan;
        std::vector<int> order;
        std::vector<int> valid;                // radii (k) allowed
        int kTop = 0;                          // the largest allowed
        int joint = -1;                        // where the limb opens into the body (-1: none found)
        int firstJoint = -1;                   // its first joint (a knee before the hip): its own length
        double firstArea = 0.0;                // the area inside it
        bool planar = true;                    // cuts are planes across the axis (else rings round the tip)
        std::vector<V3> cs, tg;                // per level: the axis point and direction (away from the tip)
        std::vector<int> enter;                // per vertex: the first level whose cut holds it
    };
    const int NEVER = 1 << 29;
    int gapL = 1;                              // the gap between two limbs, in levels
    std::vector<Cand> C;
    auto evaluate = [&](Cand &c) {
        std::vector<int> src;
        for (int m : c.members) src.push_back(tips[m]);
        c.d = geodesic(S, nbr, src, (levels + 1) * step);
        const std::vector<RingStat> R = ringStats(S, c.d, step, levels);
        c.unary.assign(levels, HUGE_VAL);
        c.A.assign(levels, 0.0);
        c.neck.assign(levels, 0.0);
        c.plan.assign(levels, 0.0);
        if (!c.planar) {
        // Rings round the tip: the region at level k is {d < k step}.
        c.enter.assign(size_t(n), NEVER);
        for (int v = 0; v < n; ++v) if (c.d[size_t(v)] < 1e299) c.enter[size_t(v)] = int(c.d[size_t(v)] / step) + 1;
        c.order.clear();
        for (int v = 0; v < n; ++v) if (c.enter[size_t(v)] < NEVER) c.order.push_back(v);
        std::sort(c.order.begin(), c.order.end(), [&](int x, int y) { return c.d[size_t(x)] < c.d[size_t(y)]; });
        {
            double cum = 0.0;
            size_t q = 0;
            for (int k = 0; k < levels; ++k) {
                while (q < c.order.size() && c.d[size_t(c.order[q])] < k * step) cum += area[size_t(c.order[q++])];
                c.A[size_t(k)] = cum;
            }
        }
        const int ws = std::max(2, int(std::lround(0.02 * size / step)));
        auto growth = [&](int k) { return (R[k + ws].P - R[k].P) / (ws * step) / (2.0 * kPi); };
        int calm = -1, joint = -1;
        bool armed = false;
        for (int k = 1; k + ws < levels && k <= K; ++k) {
            if (!armed) {
                if (R[k].P > 0 && R[k].comps == 1 && growth(k) <= 0.4) { armed = true; if (calm < 0) calm = k; }
            } else if (growth(k) >= 0.6) {
                if (c.firstJoint < 0 || c.A[size_t(k)] <= 3.0 * c.firstArea) {
                    joint = k;
                    if (c.firstJoint < 0) { c.firstJoint = k; c.firstArea = c.A[size_t(k)]; }
                }
                armed = false;
            }
        }
        if (calm >= 0 && joint < 0) {
            double gmax = 0.4;
            for (int k = calm + 1; k + ws < levels && k <= K; ++k)
                if (growth(k) > gmax) { gmax = growth(k); joint = k; }
        }
        c.joint = joint;
        int broken = levels;
        {
            bool one = false;
            for (int k = 1; k < levels; ++k) {
                if (!(R[k].P > 0)) continue;
                if (R[k].comps == 1) one = true;
                else if (one) { broken = k; break; }
            }
        }
        for (int k = k0; k <= K; ++k) {
            const RingStat &rs = R[k];
            const double r = k * step;
            if (joint < 0 || k > joint || k >= broken) continue;
            if (rs.comps != 1 || !(rs.P > 0)) continue;
            if (c.A[size_t(k)] < opt.minArea * totalArea || c.A[size_t(k)] > opt.maxArea * totalArea) continue;
            if (c.members.size() > 1 && c.A[size_t(k)] < 0.05 * totalArea) continue;
            if (!(c.d[size_t(root)] > r + gap)) continue;
            if (r < 1.2 * rs.P / (2.0 * kPi)) continue;
            double beyond = 0.0;
            for (int j = k + 1; j <= k + w && j < levels; ++j) beyond = std::max(beyond, R[j].P);
            if (!(beyond > 0)) continue;
            c.neck[size_t(k)] = rs.P / beyond;
            c.plan[size_t(k)] = rs.plan;
            c.unary[size_t(k)] = c.neck[size_t(k)] + 2.0 * rs.plan;
            {
                const double rad = rs.P / (2.0 * kPi), tmin = opt.minStump * size;
                if (rad < tmin) c.unary[size_t(k)] += 2.0 * (1.0 - rad / tmin);
            }
            c.valid.push_back(k);
        }
        } else {
        // The limb's axis: the centres of its rings, smoothed; its direction
        // runs away from the tip. Cuts are PLANES across that axis - the
        // limb's true cross-sections. (Rings at equal distance over the
        // surface slanted on a bent hoof, grew at every knee and hock, and
        // touched the other leg before going round a hip.)
        std::vector<V3> cs(size_t(levels), V3{ 0, 0, 0 }), tg(size_t(levels), V3{ 0, 0, 0 });
        std::vector<char> hasAx(size_t(levels), 0);
        for (int k = 1; k < levels; ++k) {
            V3 sum = { 0, 0, 0 };
            int cnt = 0;
            for (int j = std::max(1, k - 2); j <= std::min(levels - 1, k + 2); ++j)
                if (R[j].P > 0) { sum = add(sum, R[j].cen); ++cnt; }
            if (cnt) { cs[size_t(k)] = mul(sum, 1.0 / cnt); hasAx[size_t(k)] = 1; }
        }
        for (int k = 1; k < levels; ++k) {
            if (!hasAx[size_t(k)]) continue;
            int lo = k, hi = k;
            for (int j = k - 1; j >= std::max(1, k - 3); --j) if (hasAx[size_t(j)]) lo = j;
            for (int j = k + 1; j <= std::min(levels - 1, k + 3); ++j) if (hasAx[size_t(j)]) hi = j;
            const V3 dd = sub(cs[size_t(hi)], cs[size_t(lo)]);
            if (len(dd) > 0) tg[size_t(k)] = mul(dd, 1.0 / len(dd)); else hasAx[size_t(k)] = 0;
        }
        c.cs = cs;
        c.tg = tg;
        // Each plane: the limb region = what is reachable from the tip on the
        // tip's side of it; clean = its outline is ONE loop (the plane cuts
        // round the limb only, not also into the body or the other leg).
        c.enter.assign(size_t(n), NEVER);
        std::vector<double> Pp(size_t(levels), 0.0), regionArea(size_t(levels), 0.0);
        std::vector<char> clean(size_t(levels), 0);
        std::vector<int> stamp(size_t(n), -1), tstamp(S.t.size(), -1), region;
        bool seenClean = false;
        int stopAt = levels;
        for (int k = 1; k < levels && k <= stopAt; ++k) {
            if (!hasAx[size_t(k)]) continue;
            const V3 pc = cs[size_t(k)], pt = tg[size_t(k)];
            auto sd = [&](int v) { return dot(sub(S.p[v], pc), pt); };
            region.clear();
            for (int s0 : src)
                if (sd(s0) < 0 && stamp[size_t(s0)] != k) { stamp[size_t(s0)] = k; region.push_back(s0); }
            for (size_t h = 0; h < region.size(); ++h)
                for (const auto &nw : nbr[size_t(region[h])]) {
                    const int u2 = nw.first;
                    if (stamp[size_t(u2)] != k && sd(u2) < 0) { stamp[size_t(u2)] = k; region.push_back(u2); }
                }
            if (region.empty() || stamp[size_t(root)] == k) { if (seenClean) break; continue; }
            double ra = 0.0;
            for (int v : region) ra += area[size_t(v)];
            regionArea[size_t(k)] = ra;
            if (ra > opt.maxArea * totalArea) break;               // spread over the body
            // The outline: segments of the triangles across the plane.
            std::unordered_map<long long, int> id;
            std::vector<int> par;
            auto get = [&](long long e) {
                auto it = id.find(e);
                if (it != id.end()) return it->second;
                const int x = int(par.size());
                par.push_back(x);
                id.emplace(e, x);
                return x;
            };
            auto findp = [&](int x) { while (par[size_t(x)] != x) x = par[size_t(x)] = par[size_t(par[size_t(x)])]; return x; };
            double P = 0.0;
            for (int v : region)
                for (int ti : S.vtris[size_t(v)]) {
                    if (tstamp[size_t(ti)] == k) continue;
                    tstamp[size_t(ti)] = k;
                    const auto &t = S.t[size_t(ti)];
                    V3 q[2];
                    long long e[2];
                    int nq = 0;
                    for (int m = 0; m < 3 && nq < 2; ++m) {
                        const int x = t[size_t(m)], y = t[size_t((m + 1) % 3)];
                        const bool ix = stamp[size_t(x)] == k, iy = stamp[size_t(y)] == k;
                        if (ix == iy) continue;
                        const double fx = sd(x), fy = sd(y);
                        const double tt = std::clamp(fx / (fx - fy), 0.0, 1.0);
                        e[nq] = edgeKey(x, y);
                        q[nq++] = add(S.p[x], mul(sub(S.p[y], S.p[x]), tt));
                    }
                    if (nq != 2) continue;
                    const int pa = findp(get(e[0])), pb = findp(get(e[1]));
                    if (pa != pb) par[size_t(pa)] = pb;
                    P += len(sub(q[1], q[0]));
                }
            int loops = 0;
            for (size_t x = 0; x < par.size(); ++x) if (findp(int(x)) == int(x)) ++loops;
            for (int v : region) if (c.enter[size_t(v)] == NEVER) c.enter[size_t(v)] = k;
            if (loops == 1) { clean[size_t(k)] = 1; Pp[size_t(k)] = P; seenClean = true; }
            else if (seenClean && stopAt == levels) stopAt = std::min(levels - 1, k + w);   // keep a few for the neck test
        }

        // The limb's joint, on its cross-sections: past the tip's cap they grow
        // slowly along the limb (calm), then fast where the limb opens into
        // what it hangs on. A knee or hock is a joint too; the limb may reach
        // up to the last one while it stays limb-sized, and the reward below
        // pulls it there.
        const int ws = std::max(2, int(std::lround(0.02 * size / step)));
        auto growth = [&](int k) {
            if (!clean[size_t(k)] || k + ws >= levels || !clean[size_t(k + ws)]) return 1e9;   // the section broke: as fast as it gets
            return (Pp[size_t(k + ws)] - Pp[size_t(k)]) / (ws * step) / (2.0 * kPi);
        };
        int calm = -1, joint = -1;
        bool armed = false;
        for (int k = 1; k + ws < levels && k <= K; ++k) {
            if (!armed) {
                if (clean[size_t(k)] && growth(k) <= 0.4) { armed = true; if (calm < 0) calm = k; }
            } else if (growth(k) >= 0.6) {
                if (c.firstJoint < 0 || regionArea[size_t(k)] <= 3.0 * c.firstArea) {
                    joint = k;
                    if (c.firstJoint < 0) { c.firstJoint = k; c.firstArea = regionArea[size_t(k)]; }
                }
                armed = false;
            }
        }
        if (calm >= 0 && joint < 0) {
            double gmax = 0.4;
            for (int k = calm + 1; k + ws < levels && k <= K; ++k)
                if (clean[size_t(k)] && growth(k) < 1e8 && growth(k) > gmax) { gmax = growth(k); joint = k; }
        }
        c.joint = joint;
        // A tube: once the sections are clean, they stay clean up to the cut.
        int broken = levels;
        {
            bool one = false;
            for (int k = 1; k < levels; ++k) {
                if (!hasAx[size_t(k)]) continue;
                if (clean[size_t(k)]) one = true;
                else if (one) { broken = k; break; }
            }
        }
        // The limb's own width: the median cross-section along its calm stretch.
        double Pref = 1e300;
        {
            std::vector<double> calmP;
            const int kEnd = c.firstJoint > 0 ? c.firstJoint : joint;
            for (int k = std::max(1, calm); calm >= 0 && k <= kEnd && k < levels; ++k) if (clean[size_t(k)]) calmP.push_back(Pp[size_t(k)]);
            if (!calmP.empty()) { std::nth_element(calmP.begin(), calmP.begin() + long(calmP.size() / 2), calmP.end()); Pref = calmP[calmP.size() / 2]; }
        }
        c.A = regionArea;
        c.order.clear();
        for (int v = 0; v < n; ++v) if (c.enter[size_t(v)] < NEVER) c.order.push_back(v);
        std::sort(c.order.begin(), c.order.end(), [&](int x, int y) { return c.enter[size_t(x)] < c.enter[size_t(y)]; });
        QString dbg;
        for (int k = k0; k <= K; ++k) {
            const double r = k * step;
            if (joint < 0 || k > joint) { dbg += QLatin1Char('J'); continue; }
            if (k >= broken) { dbg += QLatin1Char('B'); continue; }
            if (!clean[size_t(k)]) { dbg += QLatin1Char('C'); continue; }
            if (c.A[size_t(k)] < opt.minArea * totalArea || c.A[size_t(k)] > opt.maxArea * totalArea) { dbg += QLatin1Char('A'); continue; }
            // A group is a head (with its ears and horns), not a patch of one:
            // at least 5% of the area.
            if (c.members.size() > 1 && c.A[size_t(k)] < 0.05 * totalArea) { dbg += QLatin1Char('S'); continue; }
            if (r < 1.2 * Pp[size_t(k)] / (2.0 * kPi)) { dbg += QLatin1Char('T'); continue; } // a bump, not a tube
            // Not wider than the limb itself: a plane higher up stays one loop
            // while it slices into the belly (a big flat face on the body).
            if (Pp[size_t(k)] > 1.8 * Pref) { dbg += QLatin1Char('W'); continue; }
            double beyond = 0.0;
            for (int j = k + 1; j <= k + w && j < levels; ++j) beyond = std::max(beyond, Pp[size_t(j)]);
            if (!(beyond > 0)) beyond = Pp[size_t(k)];
            c.neck[size_t(k)] = Pp[size_t(k)] / beyond;
            c.plan[size_t(k)] = 0.0;                                      // a plane is flat
            c.unary[size_t(k)] = c.neck[size_t(k)];
            {
                const double rad = Pp[size_t(k)] / (2.0 * kPi), tmin = opt.minStump * size;
                if (rad < tmin) c.unary[size_t(k)] += 2.0 * (1.0 - rad / tmin);    // too thin for a joint
            }
            c.valid.push_back(k);
            dbg += QString::number(std::min(9, int(5.0 * c.unary[size_t(k)])));
        }
        }
        // Reward for reaching up the limb: a leg that widens slowly into the
        // body has the same neck all the way up, and was cut at the hoof.
        if (!c.valid.empty()) {
            c.kTop = c.valid.back();
            // As strong as the neck term: along a leg the neck barely changes,
            // and a leg should end at its joint.
            for (int k : c.valid) c.unary[k] += 1.0 * (1.0 - double(k) / c.kTop);
        }
        if (notes && debug) {

            QStringList m;
            for (int t : c.members) m << QString::number(t);
            QString vk;
            for (int k : c.valid) vk += QString::number(k) + QLatin1Char(' ');
            *notes << QStringLiteral("DBG group {%1} %2: valid levels %3").arg(m.join(QLatin1Char(',')))
                          .arg(c.planar ? QStringLiteral("planes") : QStringLiteral("rings")).arg(vk);
        }
    };
    for (int i = 0; i < T; ++i) {
        Cand c;
        c.members = { i };
        c.planar = false;
        evaluate(c);
        C.push_back(std::move(c));
    }
    if (notes && debug)
        for (int i = 0; i < T; ++i)
            *notes << QStringLiteral("DBG tip %1 at (%2, %3, %4), joint at %5% of the size")
                          .arg(i).arg(S.p[tips[i]][0], 0, 'f', 2).arg(S.p[tips[i]][1], 0, 'f', 2).arg(S.p[tips[i]][2], 0, 'f', 2)
                          .arg(C[size_t(i)].joint < 0 ? -1.0 : 100.0 * C[size_t(i)].joint * step / size, 0, 'f', 1);
    // Groups: single linkage on the tips' geodesic distances (a head: muzzle,
    // ears, horns). Only tips that can be cut on their own join (a bump - an
    // udder, a nose - is simply inside whatever region covers it), and at most
    // one of them may be a big limb (3% of the area or more at its joint):
    // two legs never make one part.
    // Three kinds of tip: a limb (it can be cut on its own: a leg, an ear);
    // a bulb far out from the body's centre (a muzzle: not a tube itself, it
    // has to be covered by a group - the head); a bump near the centre (an
    // udder), which is left alone.
    std::vector<char> significant(T, 0);
    std::vector<double> tipSize(T, 0.0);
    double gFar = 0.0;
    for (int t : tips) gFar = std::max(gFar, g[t]);
    // The same tips with planar cuts: either kind may win a limb.
    for (int i = 0; i < T; ++i) {
        Cand c;
        c.members = { i };
        c.planar = true;
        evaluate(c);
        if (!c.valid.empty()) C.push_back(std::move(c));
    }
    for (int i = 0; i < T; ++i) {
        bool planeOk = false;
        for (size_t ci = size_t(T); ci < C.size(); ++ci) if (C[ci].members == std::vector<int>{ i }) planeOk = true;
        const bool limb = !C[size_t(i)].valid.empty() || planeOk;
        const bool bulb = !limb && g[tips[i]] >= 0.6 * gFar;
        significant[i] = limb || bulb;
        // How long its own limb is (tip to joint): a thin arm is still a big
        // limb (by area it was small, and joined the head).
        if (!C[size_t(i)].valid.empty())
            tipSize[i] = (C[size_t(i)].firstJoint > 0 ? C[size_t(i)].firstJoint : C[size_t(i)].kTop) * step;
        else if (limb) tipSize[i] = 0.10 * size;   // a limb only as planes: count it as long
        if (notes && debug)
            *notes << QStringLiteral("DBG tip %1: %2 (%3 of the farthest tip from the centre)").arg(i)
                          .arg(limb ? QStringLiteral("limb") : bulb ? QStringLiteral("bulb") : QStringLiteral("bump"))
                          .arg(g[tips[i]] / std::max(1e-30, gFar), 0, 'f', 2);
    }
    {
        std::vector<std::vector<int>> groups;
        std::vector<char> alive;
        for (int i = 0; i < T; ++i) { groups.push_back({ i }); alive.push_back(significant[i]); }
        auto bigCount = [&](const std::vector<int> &x, const std::vector<int> &y) {
            int b = 0;
            for (int i : x) b += tipSize[i] >= 0.10 * size;
            for (int i : y) b += tipSize[i] >= 0.10 * size;
            return b;
        };
        auto dist = [&](const std::vector<int> &x, const std::vector<int> &y) {
            double best = 1e300;
            for (int i : x) for (int j : y) best = std::min(best, C[size_t(i)].d[tips[j]]);
            return best;
        };
        for (;;) {
            int bx = -1, by = -1;
            double bd = opt.maxLength * size;
            for (size_t x = 0; x < groups.size(); ++x)
                for (size_t y = x + 1; y < groups.size(); ++y) {
                    // At most one big (long) limb: a head with its ears and
                    // horns, never two legs, never an arm with the head. A
                    // group must also be big itself (5% of the area, below).
                    if (!alive[x] || !alive[y] || bigCount(groups[x], groups[y]) > 1) continue;
                    const double dd = dist(groups[x], groups[y]);
                    if (dd < bd) { bd = dd; bx = int(x); by = int(y); }
                }
            if (bx < 0) break;
            std::vector<int> u = groups[size_t(bx)];
            u.insert(u.end(), groups[size_t(by)].begin(), groups[size_t(by)].end());
            std::sort(u.begin(), u.end());
            alive[size_t(bx)] = alive[size_t(by)] = 0;
            groups.push_back(u);
            alive.push_back(1);
            for (int kind = 0; kind < 2; ++kind) {
                Cand c;
                c.members = u;
                c.planar = kind == 1;
                evaluate(c);
                if (!c.valid.empty()) C.push_back(std::move(c));
            }
        }
    }
    const int NC = int(C.size());

    // Pairs: over the region inside candidate j's ring k, the nearest and
    // farthest points from candidate i's sources.
    std::vector<std::vector<std::vector<double>>> Mn(NC, std::vector<std::vector<double>>(NC, std::vector<double>(levels, 1e300)));
    std::vector<std::vector<std::vector<double>>> Mx(NC, std::vector<std::vector<double>>(NC, std::vector<double>(levels, -1e300)));
    // (by the level at which each vertex enters candidate i's cut)
    gapL = std::max(1, int(std::ceil(gap / step)));
    for (int j = 0; j < NC; ++j) {
        std::vector<double> mn(NC, 1e300), mx(NC, -1e300);
        size_t q = 0;
        for (int k = 0; k < levels; ++k) {
            while (q < C[j].order.size() && C[j].enter[size_t(C[j].order[q])] <= k) {
                const int v = C[j].order[q++];
                for (int i = 0; i < NC; ++i) {
                    const double e = C[i].enter[size_t(v)];
                    mn[i] = std::min(mn[i], e); mx[i] = std::max(mx[i], e);
                }
            }
            for (int i = 0; i < NC; ++i) { Mn[j][i][k] = mn[i]; Mx[j][i][k] = mx[i]; }
        }
    }
    auto hasTip = [&](int c, int t) { return std::find(C[c].members.begin(), C[c].members.end(), t) != C[c].members.end(); };
    auto pairOk = [&](int i, int ki, int j, int kj) {
        // apart: no vertex of one is in the other's cut within a gap
        if (Mn[j][i][kj] > ki + gapL && Mn[i][j][ki] > kj + gapL) return true;
        // Inside another limb only a single limb (an ear on the head) - a
        // group inside a group cut the head in layers.
        if (C[j].members.size() == 1 && Mx[j][i][kj] <= ki - gapL && C[j].A[kj] < C[i].A[ki]) return true;   // j inside i
        if (C[i].members.size() == 1 && Mx[i][j][ki] <= kj - gapL && C[i].A[ki] < C[j].A[kj]) return true;   // i inside j
        return false;
    };
    // A whole state: every chosen pair apart or nested.
    auto validState = [&](const std::vector<int> &st) {
        for (int i = 0; i < NC; ++i) {
            if (st[i] <= 0) continue;
            for (int j = i + 1; j < NC; ++j)
                if (st[j] > 0 && (C[i].members == C[j].members || !pairOk(i, st[i], j, st[j]))) return false;
        }
        return true;
    };
    // Energy: every tip that can be a limb pays for the smallest chosen limb
    // holding it, or noCut if none does (bumps pay nothing).
    // The largest limb any single-tip candidate can make of each tip: a cut
    // is rewarded for how much of it it takes, whether ring or plane (each
    // kind's own "reach the joint" reward is measured against its own joint,
    // so a short ring cut at a hoof could beat a plane taking the whole leg).
    std::vector<double> maxA(size_t(T), 0.0);
    for (int c = 0; c < NC; ++c)
        if (C[c].members.size() == 1 && !C[c].valid.empty())
            maxA[size_t(C[c].members[0])] = std::max(maxA[size_t(C[c].members[0])], C[c].A[size_t(C[c].kTop)]);
    auto energy = [&](const std::vector<int> &st) {
        double E = 0.0;
        for (int t = 0; t < T; ++t) {
            if (!significant[t]) continue;
            int best = -1;
            for (int c = 0; c < NC; ++c)
                if (st[c] > 0 && hasTip(c, t) && (best < 0 || C[c].A[st[c]] < C[best].A[st[best]])) best = c;
            if (best < 0) { E += opt.noCut; continue; }
            E += C[best].unary[st[best]];
            if (C[best].members.size() == 1 && maxA[size_t(t)] > 0)
                E += 0.6 * (1.0 - std::min(1.0, C[best].A[st[best]] / maxA[size_t(t)]));
        }
        return E;
    };

    std::vector<int> best(NC, 0);
    double bestE = energy(best);
    int accepted = 0, tried = 0;
    for (int rs = 0; rs < std::max(1, opt.searchRestarts); ++rs) {
        std::mt19937 rng(opt.seed + 7919u * quint32(rs));
        std::uniform_real_distribution<double> U01(0.0, 1.0);
        std::vector<int> st(NC, 0);
        double E = energy(st);
        const int N = std::max(100, opt.searchIters);
        const double T0 = 0.5, T1 = 0.002;
        for (int it = 0; it < N; ++it) {
            const double temp = T0 * std::pow(T1 / T0, double(it) / N);
            const int i = int(rng() % quint32(NC));
            if (C[i].valid.empty()) continue;
            int nk;
            const double m = U01(rng);
            if (m < 0.25) nk = 0;
            else if (m < 0.6 || st[i] == 0) nk = C[i].valid[rng() % C[i].valid.size()];
            else nk = st[i] + int(rng() % 7u) - 3;
            if (nk == st[i]) continue;
            if (nk > 0 && (nk >= levels || !(C[i].unary[nk] < HUGE_VAL))) continue;
            ++tried;
            const int old = st[i];
            st[i] = nk;
            if (!validState(st)) { st[i] = old; continue; }
            const double En = energy(st);
            const double dE = En - E;
            if (dE <= 0.0 || U01(rng) < std::exp(-dE / temp)) {
                E = En;
                ++accepted;
                if (E < bestE - 1e-12) { bestE = E; best = st; }
            } else {
                st[i] = old;
            }
        }
    }
    // Polish: the best radius for each candidate in turn (one may be held
    // fixed), until nothing improves.
    auto polish = [&](std::vector<int> st, int fixed) {
        double E = energy(st);
        for (bool improved = true; improved;) {
            improved = false;
            for (int i = 0; i < NC; ++i) {
                if (i == fixed) continue;
                std::vector<int> ks = C[i].valid;
                ks.push_back(0);
                for (int k : ks) {
                    std::vector<int> t = st;
                    t[i] = k;
                    if (!validState(t)) continue;
                    const double En = energy(t);
                    if (En < E - 1e-12) { E = En; st = t; improved = true; }
                }
            }
        }
        return std::make_pair(E, st);
    };
    std::vector<std::pair<double, std::vector<int>>> pool;
    pool.push_back(polish(best, -1));
    // Alternatives round the best: each chosen cut moved 1.5% or 3% of the
    // size down or up, or dropped, and the rest re-optimised round it.
    {
        const std::vector<int> top = pool[0].second;
        for (int i = 0; i < NC; ++i) {
            if (top[i] <= 0) continue;
            for (int dk : { -6, -3, 3, 6, -100000 }) {
                const int k = dk == -100000 ? 0 : top[i] + dk;
                if (k != 0 && (k < 0 || k >= levels || !(C[i].unary[k] < HUGE_VAL))) continue;
                std::vector<int> st = top;
                st[i] = k;
                if (!validState(st)) continue;
                pool.push_back(polish(st, i));
            }
        }
    }
    std::sort(pool.begin(), pool.end(), [](const auto &x, const auto &y) { return x.first < y.first; });
    std::vector<std::pair<double, std::vector<int>>> kept;
    for (const auto &cand : pool) {
        bool distinct = true;
        for (const auto &k : kept) {
            int diff = 0;
            for (int i = 0; i < NC; ++i) diff += (cand.second[i] > 0) != (k.second[i] > 0) ? 100 : std::abs(cand.second[i] - k.second[i]);
            if (diff < 4) { distinct = false; break; }          // under 2% of the size apart in total
        }
        if (distinct) kept.push_back(cand);
        if (int(kept.size()) >= std::max(1, opt.shortlist)) break;
    }

    SplitList out;
    for (const auto &kp : kept) {
        std::vector<SplitChoice> cuts;
        for (int i = 0; i < NC; ++i) {
            if (kp.second[i] <= 0) continue;
            SplitChoice c;
            c.tip = tips[C[i].members.front()];
            c.r = kp.second[i] * step;
            c.area = C[i].A[kp.second[i]];
            c.neck = C[i].neck[kp.second[i]];
            c.plan = C[i].plan[kp.second[i]];
            c.d = C[i].d;
            // The region on the tip's side of the plane, reachable from the
            // tip(s): field = distance from the plane there (and at its rim,
            // for the exact crossing), -1e300 elsewhere.
            if (!C[i].planar) {
                const double r = kp.second[i] * step;
                c.field.assign(size_t(n), -1e300);
                for (int v = 0; v < n; ++v) if (C[i].d[size_t(v)] < 1e299) c.field[size_t(v)] = r - C[i].d[size_t(v)];
            } else {
                const int k = kp.second[i];
                const V3 pc = C[i].cs[size_t(k)], pt = C[i].tg[size_t(k)];
                auto sd = [&](int v) { return dot(sub(S.p[v], pc), pt); };
                c.field.assign(size_t(n), -1e300);
                std::vector<char> in(size_t(n), 0);
                std::vector<int> reg;
                for (int m : C[i].members)
                    if (sd(tips[m]) < 0 && !in[size_t(tips[m])]) { in[size_t(tips[m])] = 1; reg.push_back(tips[m]); }
                for (size_t h = 0; h < reg.size(); ++h)
                    for (const auto &nw : nbr[size_t(reg[h])])
                        if (!in[size_t(nw.first)] && sd(nw.first) < 0) { in[size_t(nw.first)] = 1; reg.push_back(nw.first); }
                for (int v : reg) {
                    c.field[size_t(v)] = -sd(v);
                    for (const auto &nw : nbr[size_t(v)]) if (!in[size_t(nw.first)]) c.field[size_t(nw.first)] = -sd(nw.first);
                }
            }
            cuts.push_back(std::move(c));
        }
        out.push_back({ kp.first, std::move(cuts) });
    }
    if (notes) {
        int sig = 0;
        for (char c : significant) sig += c;
        *notes << QStringLiteral("split search (simulated annealing): %1 tips in %2 candidate limbs (tips and groups of "
                                 "tips), %3 tips could be cut; best: %4 cut(s), energy %5 (no cuts: %6); %7 of %8 moves "
                                 "accepted, %9 restart(s)")
                      .arg(T).arg(NC).arg(sig).arg(out.empty() ? 0 : int(out[0].second.size()))
                      .arg(out.empty() ? 0.0 : out[0].first, 0, 'f', 3).arg(sig * opt.noCut, 0, 'f', 3)
                      .arg(accepted).arg(tried).arg(std::max(1, opt.searchRestarts));
        QStringList es;
        for (const auto &o : out) es << QString::number(o.first, 'f', 3);
        *notes << QStringLiteral("shortlist: %1 different split(s), energies %2 (from %3 tried)")
                      .arg(out.size()).arg(es.join(QStringLiteral(", "))).arg(pool.size());
        const int pk = std::clamp(opt.pick, 0, std::max(0, int(kept.size()) - 1));
        if (!kept.empty())
            for (int i = 0; i < NC; ++i) {
                const int k = kept[size_t(pk)].second[i];
                if (k <= 0) continue;
                *notes << QStringLiteral("  cut %1% of the size from %2 tip(s) (%6): neck %3, planarity %4, %5% of the area")
                              .arg(100.0 * k * step / size, 0, 'f', 1).arg(C[i].members.size())
                              .arg(C[i].neck[k], 0, 'f', 2).arg(C[i].plan[k], 0, 'f', 3)
                              .arg(100.0 * C[i].A[k] / totalArea, 0, 'f', 1).arg(C[i].planar ? QStringLiteral("plane") : QStringLiteral("ring"));
            }
    }
    return out;
}

}  // namespace

// Two stages, one cut pass:
//  1. WHICH limbs: the merge tree of a harmonic Morse function f (0 on a
//     patch of the body round the root, 1 at the tips). Regions meeting at a
//     saddle: a branch meeting the body is a limb; two similar branches are
//     both limbs; a much smaller branch on a bigger one is a limb alone.
//     This found spot's head, four legs and udder, with exact volumes - but
//     cut the legs near the hoof: f's level lines do not run round a leg at
//     its joint.
//  2. WHERE: from each limb's tip, geodesic rings grow up the limb. If the
//     ring length levels off and then grows by neckJump within 15% of the
//     size (a leg flaring into the body), the cut moves up to that ring -
//     provided the ring's region holds the merge-tree region, stays under
//     maxArea and does not reach the body's centre. A limb without such a
//     joint (a head: widens, then narrows at the neck) keeps its f cut.
// Every limb is cut along ONE field (f or its ring distance), and limbs whose
// regions touch are dropped (smaller first), so each triangle is split by one
// field and both sides get the same cut. (Cutting in rounds and re-finding
// tips on the rest carved spot's body into 16 pieces; overlapping rings broke
// armadillo's cuts.)
bool HarmonicFit::splitLimbs(const MeshData &mesh, const SplitOptions &opt, QVector<Part> *parts,
                             QStringList *notes, QString *error)
{
    parts->clear();
    Surf S;
    int bnd = 0, genus = -1;
    QString err;
    if (!buildSurface(mesh, &S, &bnd, &genus, &err)) { if (error) *error = err; return false; }
    if (bnd > 0) { if (error) *error = QStringLiteral("The mesh is open (%1 boundary edges).").arg(bnd); return false; }
    const int n = int(S.p.size());
    const Nbr nbr = cotanNbr(S);

    V3 lo = { 1e300, 1e300, 1e300 }, hi = { -1e300, -1e300, -1e300 };
    for (const V3 &q : S.p) for (int k = 0; k < 3; ++k) { lo[k] = std::min(lo[k], q[k]); hi[k] = std::max(hi[k], q[k]); }
    const double size = len(sub(hi, lo));
    std::vector<double> area(n, 0.0);
    double totalArea = 0.0;
    for (const auto &t : S.t) {
        const double A = 0.5 * len(cross(sub(S.p[t[1]], S.p[t[0]]), sub(S.p[t[2]], S.p[t[0]])));
        for (int k = 0; k < 3; ++k) area[t[k]] += A / 3.0;
        totalArea += A;
    }

    // Root: the vertex nearest the volume centroid (on the body).
    V3 cen = { 0, 0, 0 };
    double vol = 0.0;
    for (const auto &t : S.t) {
        const double v6 = dot(S.p[t[0]], cross(S.p[t[1]], S.p[t[2]]));
        vol += v6;
        cen = add(cen, mul(add(add(S.p[t[0]], S.p[t[1]]), S.p[t[2]]), v6 / 4.0));
    }
    cen = mul(cen, 1.0 / vol);
    int root = 0;
    for (int v = 0; v < n; ++v) if (len(sub(S.p[v], cen)) < len(sub(S.p[root], cen))) root = v;

    // Tips: local maxima of the geodesic distance from the root, farthest
    // first, one per `tipRadius` neighbourhood.
    const std::vector<double> g = geodesic(S, nbr, { root }, 0.0);
    std::vector<int> maxima;
    for (int v = 0; v < n; ++v) {
        bool isMax = g[v] < 1e299;
        for (const auto &nw : nbr[v]) if (g[nw.first] > g[v]) { isMax = false; break; }
        if (isMax) maxima.push_back(v);
    }
    std::sort(maxima.begin(), maxima.end(), [&](int x, int y) { return g[x] > g[y]; });
    std::vector<int> tips;
    {
        std::vector<char> covered(n, 0);
        for (int c : maxima) {
            if (covered[c]) continue;
            tips.push_back(c);
            const std::vector<double> d = geodesic(S, nbr, { c }, opt.tipRadius * size);
            for (int v = 0; v < n; ++v) if (d[v] <= opt.tipRadius * size) covered[v] = 1;
        }
    }

    std::vector<double> f(n, 0.5);
    // A limb: vertices v with field[v] > thr (for a ring: field = -distance).
    struct Limb {
        std::vector<double> field; double thr; double area; int tip; bool ring; double length;
        std::vector<double> fField; double fThr; double fArea;      // the merge-tree cut, kept as a fallback
        std::vector<double> dist;                                   // ring distance from the tip
        int parent = -1;                                            // the limb it sits on (a horn on the head)
    };
    std::vector<Limb> limbs;
    int moved = 0, shrunk = 0, dropped = 0, bumps = 0, nested = 0;
    if (opt.search) {
        // The search: every limb a ring round its tip, all chosen together.
        SplitList list = searchSplit(S, nbr, area, totalArea, size, root, g, tips, opt, notes);
        if (opt.pick < 0 || opt.pick >= int(list.size())) {
            if (error) *error = QStringLiteral("the search found %1 different split(s); no split %2").arg(list.size()).arg(opt.pick + 1);
            return false;
        }
        if (notes)
            *notes << QStringLiteral("split %1 of %2 on the shortlist (energy %3)").arg(opt.pick + 1).arg(list.size())
                          .arg(list[size_t(opt.pick)].first, 0, 'f', 3);
        std::vector<SplitChoice> &ch = list[size_t(opt.pick)].second;
        for (SplitChoice &c : ch) {
            Limb L;
            L.dist = std::move(c.d);
            // The cut is a plane across the limb: field = distance from it on
            // the limb's side, so the mesh is split exactly on the plane.
            L.field = std::move(c.field);
            L.thr = 0.0;
            L.area = c.area;
            L.tip = c.tip;
            L.ring = true;
            L.length = c.r;
            L.fField = L.field; L.fThr = L.thr; L.fArea = L.area;
            limbs.push_back(std::move(L));
        }
        // Smaller first (a nested limb is labelled before the one it sits on);
        // its parent: the smallest limb holding all of it.
        std::sort(limbs.begin(), limbs.end(), [](const Limb &x, const Limb &y) { return x.area < y.area; });
        moved = int(limbs.size());
        for (size_t j = 0; j < limbs.size(); ++j)
            for (size_t i = j + 1; i < limbs.size(); ++i) {
                bool inside = true;
                for (int v = 0; v < n && inside; ++v)
                    if (limbs[j].field[v] > limbs[j].thr && !(limbs[i].field[v] > limbs[i].thr)) inside = false;
                if (inside) { limbs[j].parent = int(i); ++nested; break; }
            }
    } else {
    // ---- stage 1: the Morse function and its merge tree
    double gmax = 0.0;
    for (double x : g) if (x < 1e299) gmax = std::max(gmax, x);
    {
        std::vector<char> fixed(n, 0);
        for (int v = 0; v < n; ++v) if (g[v] <= 0.15 * gmax) { fixed[v] = 1; f[v] = 0.0; }
        for (int t : tips) { fixed[t] = 1; f[t] = 1.0; }
        std::vector<int> unk(n, -1), idx;
        for (int v = 0; v < n; ++v) if (!fixed[v]) { unk[v] = int(idx.size()); idx.push_back(v); }
        std::vector<double> x(idx.size(), 0.5), b(idx.size(), 0.0), dg(idx.size(), 0.0);
        for (size_t k = 0; k < idx.size(); ++k) {
            for (const auto &nw : nbr[idx[k]]) { dg[k] += nw.second; if (fixed[nw.first]) b[k] += nw.second * f[nw.first]; }
            if (dg[k] <= 0) dg[k] = 1.0;
        }
        auto A = [&](const std::vector<double> &in, std::vector<double> &o) {
            for (size_t k = 0; k < idx.size(); ++k) {
                double val = dg[k] * in[k];
                for (const auto &nw : nbr[idx[k]]) if (unk[nw.first] >= 0) val -= nw.second * in[unk[nw.first]];
                o[k] = val;
            }
        };
        solveCg(A, dg, b, x, 40000, 1e-10);
        for (size_t k = 0; k < idx.size(); ++k) f[idx[k]] = x[k];
    }

    {
        std::vector<int> order(n);
        for (int v = 0; v < n; ++v) order[v] = v;
        std::sort(order.begin(), order.end(), [&](int x, int y) { return f[x] > f[y]; });
        std::vector<int> comp(n, -1);
        std::vector<std::vector<int>> members;
        std::vector<double> compArea, compBirth;
        std::vector<int> compTip, redirect;
        std::vector<char> leaf;
        auto findC = [&](int c) { while (redirect[c] != c) c = redirect[c] = redirect[redirect[c]]; return c; };
        struct Found { std::vector<int> verts; double level; double area; int tip; };
        std::vector<Found> found;
        for (int v : order) {
            std::vector<int> touching;
            for (const auto &nw : nbr[v])
                if (comp[nw.first] >= 0) {
                    const int c = findC(comp[nw.first]);
                    if (std::find(touching.begin(), touching.end(), c) == touching.end()) touching.push_back(c);
                }
            if (touching.empty()) {
                const int c = int(members.size());
                members.push_back({ v });
                compArea.push_back(area[v]);
                compBirth.push_back(f[v]);
                compTip.push_back(v);
                leaf.push_back(1);
                redirect.push_back(c);
                comp[v] = c;
                continue;
            }
            std::sort(touching.begin(), touching.end(), [&](int x, int y) { return compArea[x] > compArea[y]; });
            const int keep = touching[0];
            auto significant = [&](int c) {
                return compArea[c] >= opt.minArea * totalArea &&
                       len(sub(S.p[compTip[c]], S.p[v])) >= opt.minPersistence * size;
            };
            auto cut = [&](int c) {
                found.push_back({ members[c], f[v] + 1e-3 * (compBirth[c] - f[v]), compArea[c], compTip[c] });
            };
            for (size_t m = 1; m < touching.size(); ++m) {
                const int c = touching[m];
                if (significant(c)) {
                    if (!leaf[keep]) {
                        if (leaf[c]) cut(c);
                    } else if (leaf[c]) {
                        if (compArea[c] < 0.35 * compArea[keep]) {
                            cut(c);                               // a small branch on a bigger one (a horn on the head)
                        } else if (compArea[c] >= 0.015 * totalArea && compArea[keep] >= 0.015 * totalArea) {
                            cut(c);                               // two real limbs meeting (legs at the chest):
                            if (significant(keep)) cut(keep);     // both end, what they form is body
                            leaf[keep] = 0;
                        }
                        // else: two small similar branches (two toes of one hoof, two
                        // horns meeting on the forehead) are one feature - they join
                        // and stay a branch. Cutting both split a hoof in half and
                        // turned spot's head into body.
                    } else {
                        if (significant(keep)) cut(keep);
                        leaf[keep] = 0;
                    }
                }
                members[keep].insert(members[keep].end(), members[c].begin(), members[c].end());
                members[c].clear();
                members[c].shrink_to_fit();
                compArea[keep] += compArea[c];
                redirect[c] = keep;
            }
            members[keep].push_back(v);
            compArea[keep] += area[v];
            comp[v] = keep;
        }
        for (auto &fd : found) {
            if (fd.area > opt.maxArea * totalArea) continue;
            // The merge-tree region is {f > level} containing the tip; as a
            // field, f itself (vertices of other branches above the level are
            // masked out below).
            std::vector<double> fld(n, -1e300);
            for (int v : fd.verts) fld[v] = f[v];
            Limb L{ fld, fd.level, fd.area, fd.tip, false, 0.0, fld, fd.level, fd.area, {} };
            limbs.push_back(std::move(L));
        }
    }

    // ---- stage 2: move a limb's cut up to its joint, if its ring profile has one
    const double step = 0.005 * size;
    const double minLen = opt.minPersistence * size, maxLen = opt.maxLength * size;
    const int K = int(maxLen / step) + 1;
    const int wCalm = std::max(2, int(std::lround(0.05 * size / step)));
    const int wRise = std::max(2, int(std::lround(0.15 * size / step)));
    for (Limb &L : limbs) {
        std::vector<double> d = geodesic(S, nbr, { L.tip }, maxLen + 0.2 * size);
        const std::vector<double> P = ringLengths(S, d, step, K + wRise + 2);
        // The joint: from the limb's narrowest calm ring, the first ring that is
        // neckJump times wider. Spot's legs level off near the hoof and then
        // flare steadily into the belly - no sudden jump - so "grows by X
        // within Y" found the start of the flare, barely above the f cut.
        int kCut = -1;
        const int k0 = std::max(wCalm, int(std::ceil(minLen / step)));
        int kCalm = -1;
        for (int k = k0; k <= K; ++k)
            if (P[k] > 0 && P[k] - P[k - wCalm] <= opt.neckCalm * P[k]) { kCalm = k; break; }
        if (kCalm >= 0) {
            double pmin = P[kCalm];
            for (int k = kCalm + 1; k <= K; ++k) {
                if (P[k] <= 0) continue;
                pmin = std::min(pmin, P[k]);
                if (P[k] >= opt.neckJump * pmin) { kCut = k; break; }
            }
        }
        if (kCut < 0) continue;
        const double r = (kCut + 0.5) * step;
        // The ring must stay on this limb: no other limb's tip inside it.
        // (Testing the root instead failed on spot, whose root - the surface
        // point nearest the centroid - is on the belly between the legs; any
        // tip failed too - a hoof can have two.)
        bool otherTip = false;
        for (const Limb &M : limbs) if (&M != &L && d[M.tip] < r) { otherTip = true; break; }
        if (otherTip) continue;
        double a = 0.0;
        bool holds = true;
        for (int v = 0; v < n; ++v) {
            if (d[v] < r) a += area[v];
            if (L.field[v] > L.thr && d[v] >= r) holds = false;
        }
        if (!holds || a <= L.area || a > opt.maxArea * totalArea) continue;
        for (int v = 0; v < n; ++v) L.field[v] = d[v] < 1e299 ? -d[v] : -1e300;
        L.dist = d;
        L.thr = -r;
        L.area = a;
        L.ring = true;
        L.length = r;
        ++moved;
    }

    // Limbs whose regions (plus one ring of vertices) touch, smaller first: a
    // moved-up ring that meets a neighbour (spot's front legs meet at the
    // chest) shrinks step by step, then falls back to its merge-tree cut;
    // only if that clashes too is the limb dropped.
    std::sort(limbs.begin(), limbs.end(), [](const Limb &x, const Limb &y) { return x.area < y.area; });
    // owner[v]: index into keep of the limb holding v, and whether it is cut
    // by a ring. Two limbs cut by the same f may touch (a triangle between
    // them is split by f); anything involving a ring needs a gap of one ring
    // of vertices, so every triangle meets at most one ring.
    std::vector<int> owner(n, -1);
    std::vector<Limb> keep;
    // A smaller limb lying wholly inside L - its region and every neighbour of
    // it inside L's region (a horn on the head) - is nested, not a clash: it is
    // cut off first and L is cut from what is left.
    auto nestedIn = [&](int o, const Limb &L) {
        for (int v = 0; v < n; ++v) {
            if (owner[v] != o) continue;
            if (!(L.field[v] > L.thr)) return false;
            for (const auto &nw : nbr[v])
                if (owner[nw.first] != o && !(L.field[nw.first] > L.thr)) return false;
        }
        return true;
    };
    std::vector<int> children;
    auto clashes = [&](const Limb &L) {
        children.clear();
        std::vector<char> checked(keep.size(), 0), inside(keep.size(), 0);
        auto isChild = [&](int o) {
            if (!checked[o]) { checked[o] = 1; inside[o] = nestedIn(o, L); if (inside[o]) children.push_back(o); }
            return bool(inside[o]);
        };
        for (int v = 0; v < n; ++v) {
            if (!(L.field[v] > L.thr)) continue;
            if (owner[v] >= 0 && !isChild(owner[v])) return true;
            for (const auto &nw : nbr[v]) {
                const int o = owner[nw.first];
                if (o >= 0 && (L.ring || keep[o].ring) && !isChild(o)) return true;
            }
        }
        return false;
    };
    for (auto &L : limbs) {
        // A limb must be a tube, not a bump (spot's udder: 1% of the area but
        // short and wide - its fit found no usable levels, leaving a hole; a
        // hind hoof nearly as wide as long folded 12% of its samples): at
        // least as long as its cut is wide.
        {
            V3 bc = { 0, 0, 0 };
            int nb = 0;
            std::vector<int> region, border;
            for (int v = 0; v < n; ++v) {
                if (!(L.field[v] > L.thr)) continue;
                region.push_back(v);
                for (const auto &nw : nbr[v])
                    if (!(L.field[nw.first] > L.thr)) { border.push_back(v); bc = add(bc, S.p[v]); ++nb; break; }
            }
            if (nb > 0) {
                bc = mul(bc, 1.0 / nb);
                double rad = 0.0, length = 0.0;
                for (int v : border) rad += len(sub(S.p[v], bc));
                rad /= nb;
                for (int v : region) length = std::max(length, len(sub(S.p[v], bc)));
                if (length < 0.6 * 2.0 * rad) { ++bumps; continue; }
            }
        }
        bool clash = clashes(L);
        if (clash && L.ring) {
            const double r0 = -L.thr;
            for (int s = 1; s <= 8 && clash; ++s) {
                const double r = r0 * (1.0 - 0.06 * s);
                bool holds = true;
                for (int v = 0; v < n && holds; ++v) if (L.fField[v] > L.fThr && L.dist[v] >= r) holds = false;
                if (!holds) break;                          // would no longer hold the merge-tree region
                L.thr = -r;
                clash = clashes(L);
            }
            if (!clash) { ++shrunk; L.length = -L.thr; }
            else {
                L.field = L.fField; L.thr = L.fThr; L.area = L.fArea; L.ring = false; L.length = 0.0;
                clash = clashes(L);
            }
        }
        if (clash) { ++dropped; continue; }
        if (L.ring) {
            L.area = 0.0;
            for (int v = 0; v < n; ++v) if (L.field[v] > L.thr) L.area += area[v];
        }
        const int me = int(keep.size());
        for (int o : children)
            if (keep[o].parent < 0) { keep[o].parent = me; ++nested; }
        for (int v = 0; v < n; ++v)
            if (L.field[v] > L.thr && owner[v] < 0) owner[v] = me;   // a nested child keeps its vertices
        keep.push_back(std::move(L));
    }
    limbs.swap(keep);
    }  // the rules (opt.search off)

    // Labels: smaller limbs were kept first, so a horn keeps its vertices and
    // the head gets the rest of its region.
    std::vector<int> label(n, 0);
    for (size_t l = 0; l < limbs.size(); ++l)
        for (int v = 0; v < n; ++v)
            if (label[v] == 0 && limbs[l].field[v] > limbs[l].thr) label[v] = int(l) + 1;

    // Cut: every mixed triangle has the body and one limb; split along that
    // limb's field. For a merge-tree limb its field is f, which is defined on
    // the body side too.
    const int nParts = int(limbs.size()) + 1;
    std::vector<MeshData> pm(nParts);
    std::vector<std::unordered_map<long long, uint32_t>> ptIdx(nParts);
    auto addPt = [&](int part, long long key, const V3 &q) {
        auto it = ptIdx[part].find(key);
        if (it != ptIdx[part].end()) return it->second;
        const uint32_t id = pm[part].addVertex(q[0], q[1], q[2]);
        ptIdx[part].emplace(key, id);
        return id;
    };
    auto tri = [&](int part, long long k0, const V3 &p0, long long k1, const V3 &p1, long long k2, const V3 &p2) {
        if (len(cross(sub(p1, p0), sub(p2, p0))) <= 1e-30) return;
        pm[part].tris.push_back(addPt(part, k0, p0));
        pm[part].tris.push_back(addPt(part, k1, p1));
        pm[part].tris.push_back(addPt(part, k2, p2));
    };
    auto value = [&](int limb, int v) {
        const Limb &L = limbs[limb - 1];
        if (!L.ring) return f[v];                       // f is the field everywhere
        return L.field[v] > -1e299 ? L.field[v] : -1e6 * size;
    };
    auto crossing = [&](int x, int y, int limb) {
        const double thr = limbs[limb - 1].thr;
        const double fx = value(limb, x), fy = value(limb, y);
        const double t = fy != fx ? std::clamp((thr - fx) / (fy - fx), 0.0, 1.0) : 0.5;
        return add(S.p[x], mul(sub(S.p[y], S.p[x]), t));
    };
    int unsplit = 0;
    for (const auto &t : S.t) {
        const int la = label[t[0]], lb = label[t[1]], lc = label[t[2]];
        if (la == lb && lb == lc) { tri(la, t[0], S.p[t[0]], t[1], S.p[t[1]], t[2], S.p[t[2]]); continue; }
        if (la != lb && lb != lc && la != lc) { ++unsplit; tri(la, t[0], S.p[t[0]], t[1], S.p[t[1]], t[2], S.p[t[2]]); continue; }
        const int odd = la == lb ? 2 : (la == lc ? 1 : 0);
        const int o = t[odd], x = t[(odd + 1) % 3], y = t[(odd + 2) % 3];
        const int lo_ = label[o], lr = label[x];
        int ring = lo_ != 0 ? lo_ : lr;
        if (lo_ != 0 && lr != 0) {
            if (limbs[lo_ - 1].parent == lr - 1) ring = lo_;          // a nested limb: its own boundary
            else if (limbs[lr - 1].parent == lo_ - 1) ring = lr;
            else if (limbs[lr - 1].thr > limbs[lo_ - 1].thr) ring = lr;   // two f-limbs: the higher level
        }
        const V3 px = crossing(o, x, ring), py = crossing(o, y, ring);
        const long long kx = -1 - edgeKey(o, x), ky = -1 - edgeKey(o, y);
        tri(lo_, o, S.p[o], kx, px, ky, py);
        tri(lr, kx, px, x, S.p[x], y, S.p[y]);
        tri(lr, kx, px, y, S.p[y], ky, py);
    }

    // Cap every boundary loop with a fan from its centroid; both sides of a
    // cut see the same loop, so they get the same cap.
    std::vector<QVector<Cap>> partCaps(nParts);
    int caps = 0;
    for (int part = 0; part < nParts; ++part) {
        MeshData &m = pm[part];
        std::unordered_map<long long, int> ecount;
        std::unordered_map<uint32_t, uint32_t> nextOf;
        for (int tI = 0; tI < m.triangleCount(); ++tI)
            for (int k = 0; k < 3; ++k) ++ecount[edgeKey(int(m.tris[3 * tI + k]), int(m.tris[3 * tI + (k + 1) % 3]))];
        for (int tI = 0; tI < m.triangleCount(); ++tI)
            for (int k = 0; k < 3; ++k) {
                const uint32_t x = m.tris[3 * tI + k], y = m.tris[3 * tI + (k + 1) % 3];
                if (ecount[edgeKey(int(x), int(y))] == 1) nextOf[x] = y;
            }
        std::unordered_map<uint32_t, char> done;
        for (const auto &kv : nextOf) {
            if (done.count(kv.first)) continue;
            std::vector<uint32_t> loop;
            uint32_t cur = kv.first;
            for (size_t guard = 0; guard <= nextOf.size(); ++guard) {
                if (done.count(cur)) break;
                done[cur] = 1;
                loop.push_back(cur);
                auto it = nextOf.find(cur);
                if (it == nextOf.end()) break;
                cur = it->second;
            }
            if (loop.size() < 3) continue;
            V3 c = { 0, 0, 0 };
            for (uint32_t id : loop) c = add(c, { m.pos[3 * id], m.pos[3 * id + 1], m.pos[3 * id + 2] });
            c = mul(c, 1.0 / loop.size());
            const uint32_t ci = m.addVertex(c[0], c[1], c[2]);
            Cap cap;
            for (int k = 0; k < 3; ++k) cap.centre[k] = c[k];
            for (size_t q = 0; q < loop.size(); ++q) {
                const uint32_t x = loop[q], y = loop[(q + 1) % loop.size()];
                m.tris.push_back(y); m.tris.push_back(x); m.tris.push_back(ci);
                const V3 px = { m.pos[3 * x], m.pos[3 * x + 1], m.pos[3 * x + 2] };
                const V3 py = { m.pos[3 * y], m.pos[3 * y + 1], m.pos[3 * y + 2] };
                cap.area += 0.5 * len(cross(sub(px, c), sub(py, c)));
            }
            partCaps[part].append(cap);
            ++caps;
        }
        m.computeBounds();
        m.computeNormals();
    }

    for (int part = 0; part < nParts; ++part) {
        Part P;
        P.mesh = std::move(pm[part]);
        P.name = part == 0 ? QStringLiteral("body") : QStringLiteral("limb %1").arg(part);
        P.level = part == 0 ? 0.0 : limbs[part - 1].length;
        double a = 0.0;
        for (int v = 0; v < n; ++v) if (label[v] == part) a += area[v];
        P.areaFrac = a / totalArea;
        P.caps = partCaps[part];
        if (P.mesh.triangleCount() > 0) parts->append(std::move(P));
    }
    if (notes) {
        int ringKept = 0;
        for (const Limb &L : limbs) ringKept += L.ring;
        *notes << QStringLiteral("limb split: %1 part(s) - body + %2 limb(s); %3 tips; %4 of %5 cut(s) at a joint "
                                 "(%6 shrunk to clear a neighbour); %7 limb(s) dropped (touching a smaller one); "
                                 "%9 bump(s) left on the body, %10 nested in a bigger limb%8")
                      .arg(parts->size()).arg(parts->size() - 1).arg(tips.size()).arg(ringKept).arg(moved).arg(shrunk).arg(dropped)
                      .arg(unsplit ? QStringLiteral("; %1 triangle(s) where three parts meet not split").arg(unsplit)
                                   : QString()).arg(bumps).arg(nested);
        for (const Part &P : *parts)
            *notes << QStringLiteral("  %1: %2% of the area, %3 cap(s), %4 triangles")
                          .arg(P.name).arg(100.0 * P.areaFrac, 0, 'f', 1).arg(P.caps.size())
                          .arg(P.mesh.triangleCount());
    }
    return true;
}

void HarmonicFit::snapParts(QVector<Result> *results, const QVector<Part> &parts, QStringList *notes)
{
    auto same = [](const Cap &a, const Cap &b) {
        return std::fabs(a.centre[0] - b.centre[0]) < 1e-5 && std::fabs(a.centre[1] - b.centre[1]) < 1e-5 &&
               std::fabs(a.centre[2] - b.centre[2]) < 1e-5;
    };
    auto largest = [](const Part &P) -> const Cap * {
        const Cap *big = nullptr;
        for (const auto &c : P.caps) if (!big || c.area > big->area) big = &c;
        return big;
    };
    for (int c = 0; c < parts.size() && c < results->size(); ++c) {
        Result &child = (*results)[c];
        const Cap *k = largest(parts[c]);
        if (!child.ok || !k) continue;
        // Small parts (ears, horns: under 1% of the area) are not snapped: the
        // move is large for them (2.4% of the model size on spot's ears) and
        // folded them; a small part is one piece anyway - its exact mesh.
        if (parts[c].areaFrac < 0.01) continue;
        // The part on the other side of this cap.
        int par = -1;
        for (int p = 0; p < parts.size(); ++p) {
            if (p == c) continue;
            for (const auto &q : parts[p].caps) if (same(q, *k)) { par = p; break; }
            if (par >= 0) break;
        }
        if (par < 0 || par >= results->size() || !(*results)[par].ok) continue;
        const Cap *pk = largest(parts[par]);
        if (pk && same(*pk, *k)) continue;                    // both blocks end on it: nothing to do
        const Result &parent = (*results)[par];

        Net pn(parent), cn(child);
        const int nv = cn.nv, nw = cn.nw, nu = cn.nu;
        std::vector<V3> X;
        for (int kk = 0; kk < nw; ++kk)
            for (int j = 0; j < nv; ++j) X.push_back(cn.at(0, j, kk));
        X.push_back({ k->centre[0], k->centre[1], k->centre[2] });   // last: the cap's centre
        const BoundaryProbe probe(pn, X);
        // Every point goes onto the ONE parent face the cap centre is nearest
        // to (a leg: the body's side). Letting each point pick its own nearest
        // sample sent some to the body's end face - moves of up to 27% of the
        // model size on spot.
        const int capFace = probe.nearest.back() >= 0 ? probe.S[probe.nearest.back()].face : 0;
        X.pop_back();
        const double maxMove = 0.04 * child.size;

        // Nearest point on the parent's boundary: the nearest sample, then a
        // few Gauss-Newton steps on that face's two parameters.
        double moveMax = 0.0, moveSum = 0.0;
        std::vector<V3> delta(X.size(), V3{ 0, 0, 0 });
        for (size_t q = 0; q < X.size(); ++q) {
            // Nearest sample on the cap's face.
            int m = -1;
            double bd = 1e300;
            for (int si = 0; si < int(probe.S.size()); ++si) {
                if (probe.S[si].face != capFace) continue;
                const double dd = len(sub(probe.S[si].p, X[q]));
                if (dd < bd) { bd = dd; m = si; }
            }
            if (m < 0) continue;
            const auto &smp = probe.S[m];
            double uvw[3] = { smp.u, smp.v, smp.face == 0 ? 0.0 : smp.w };
            const int a0 = smp.face == 0 ? 0 : 1, a1 = smp.face == 0 ? 1 : 2;   // the face's parameters
            for (int it = 0; it < 6; ++it) {
                const V3 pt = pn.eval(uvw[0], uvw[1], uvw[2]);
                const V3 r = sub(X[q], pt);
                const V3 d0 = pn.d(uvw[0], uvw[1], uvw[2], a0), d1 = pn.d(uvw[0], uvw[1], uvw[2], a1);
                const double A = dot(d0, d0), B = dot(d0, d1), Cc = dot(d1, d1);
                const double b0 = dot(d0, r), b1 = dot(d1, r);
                const double det = A * Cc - B * B;
                if (std::fabs(det) < 1e-30) break;
                uvw[a0] = std::clamp(uvw[a0] + (Cc * b0 - B * b1) / det, 0.0, 1.0);
                uvw[a1] = std::clamp(uvw[a1] + (A * b1 - B * b0) / det, 0.0, 1.0);
            }
            delta[q] = sub(pn.eval(uvw[0], uvw[1], uvw[2]), X[q]);
            const double dl = len(delta[q]);
            if (dl > maxMove) delta[q] = mul(delta[q], maxMove / dl);   // never more than 4% of the size
            moveMax = std::max(moveMax, len(delta[q]));
            moveSum += len(delta[q]);
        }
        static const double fade[4] = { 1.0, 0.75, 0.5, 0.25 };
        for (int kk = 0; kk < nw; ++kk)
            for (int j = 0; j < nv; ++j) {
                const V3 &dq = delta[size_t(kk) * nv + j];
                for (int i = 0; i < 4 && i < nu; ++i) cn.at(i, j, kk) = add(cn.at(i, j, kk), mul(dq, fade[i]));
            }
        for (size_t m = 0; m < cn.C.size(); ++m)
            for (int a = 0; a < 3; ++a) child.ctrl[int(3 * m) + a] = cn.C[m][a];
        if (notes)
            *notes << QStringLiteral("%1 joined to %2: its cap row moved onto %2's surface by %3% on average, "
                                     "at most %4% of the model size")
                          .arg(parts[c].name, parts[par].name)
                          .arg(100.0 * moveSum / std::max<size_t>(1, X.size()) / child.size, 0, 'f', 2)
                          .arg(100.0 * moveMax / child.size, 0, 'f', 2);
    }
}

int HarmonicFit::outsideCount(const Result &r, const MeshData &mesh, double *worst)
{
    if (worst) *worst = 0.0;
    if (!r.ok || mesh.vertexCount() == 0) return -1;
    std::vector<V3> X;
    const int n = mesh.vertexCount();
    const int stride = std::max(1, n / 30000);
    for (int i = 0; i < n; i += stride) X.push_back({ mesh.pos[3 * i], mesh.pos[3 * i + 1], mesh.pos[3 * i + 2] });
    const BoundaryProbe probe(Net(r), X);
    int out = 0;
    for (size_t q = 0; q < X.size(); ++q)
        if (probe.depth[q] > 0) {
            ++out;
            if (worst) *worst = std::max(*worst, probe.depth[q] / std::max(1e-30, r.size));
        }
    return out;
}

int HarmonicFit::pushToEnclose(Result *r, const MeshData &mesh, int rounds, double gapFrac)
{
    if (!r->ok || mesh.vertexCount() == 0) return -1;
    std::vector<V3> X;
    const int nverts = mesh.vertexCount();
    const int stride = std::max(1, nverts / 30000);
    for (int i = 0; i < nverts; i += stride) X.push_back({ mesh.pos[3 * i], mesh.pos[3 * i + 1], mesh.pos[3 * i + 2] });
    const double gap = gapFrac * r->size;

    Net net(*r);
    const int nu = net.nu, nv = net.nv, nw = net.nw;
    std::vector<V3> bestC = net.C;
    int bestOut = INT32_MAX;
    for (int round = 0; round <= rounds; ++round) {
        const BoundaryProbe probe(net, X);
        std::vector<double> need(probe.S.size(), 0.0);
        int out = 0;
        for (size_t q = 0; q < X.size(); ++q) {
            if (probe.nearest[q] < 0 || probe.depth[q] <= 0) continue;
            ++out;
            need[probe.nearest[q]] = std::max(need[probe.nearest[q]], probe.depth[q] + gap);
        }
        if (out < bestOut) { bestOut = out; bestC = net.C; }
        if (out == 0 || round == rounds) break;

        // Side: each outer control point, by the largest need in its support
        // among samples facing its way, along its own normal.
        const double su = 2.0 / std::max(1, nu - 3), sv = 2.0 / nv;
        std::vector<V3> move(size_t(nu) * nv, V3{ 0, 0, 0 });
        for (int i = 0; i < nu; ++i)
            for (int j = 0; j < nv; ++j) {
                const double gu = std::clamp((net.U[i + 1] + net.U[i + 2] + net.U[i + 3]) / 3.0, 0.0, 1.0);
                double gv = double(j - 1) / nv;
                if (gv < 0) gv += 1.0;
                const V3 n = net.normal(gu, gv, 0.0, 0);
                double off = 0.0;
                for (size_t m = 0; m < probe.S.size(); ++m) {
                    const auto &smp = probe.S[m];
                    if (smp.face != 0 || need[m] <= 0) continue;
                    double dv = std::fabs(smp.v - gv);
                    dv = std::min(dv, 1.0 - dv);
                    if (std::fabs(smp.u - gu) <= su && dv <= sv && dot(smp.n, n) > 0.5) off = std::max(off, need[m]);
                }
                move[size_t(i) + size_t(nu) * j] = mul(n, off);
            }
        for (int i = 0; i < nu; ++i)
            for (int j = 0; j < nv; ++j) net.at(i, j, 0) = add(net.at(i, j, 0), move[size_t(i) + size_t(nu) * j]);
        // Ends: a whole end row along the end's mean normal.
        for (int e = 0; e < 2; ++e) {
            double off = 0.0;
            V3 dir = { 0, 0, 0 };
            for (size_t m = 0; m < probe.S.size(); ++m)
                if (probe.S[m].face == 1 + e) {
                    dir = add(dir, probe.S[m].n);
                    off = std::max(off, need[m]);
                }
            const double l = len(dir);
            if (off <= 0 || l <= 1e-12) continue;
            dir = mul(dir, off / l);
            const int i = e ? nu - 1 : 0;
            for (int k = 0; k < nw; ++k)
                for (int j = 0; j < nv; ++j) net.at(i, j, k) = add(net.at(i, j, k), dir);
        }
    }
    for (size_t m = 0; m < bestC.size(); ++m)
        for (int a = 0; a < 3; ++a) r->ctrl[int(3 * m) + a] = bestC[m][a];
    return bestOut;
}

MeshData HarmonicFit::envelope(const MeshData &part, double offset, int voxels, QString *error)
{
    Surf S;
    int bnd = 0, genus = -1;
    QString err;
    if (!buildSurface(part, &S, &bnd, &genus, &err)) { if (error) *error = err; return MeshData(); }
    V3 lo = { 1e300, 1e300, 1e300 }, hi = { -1e300, -1e300, -1e300 };
    for (const V3 &q : S.p) for (int a = 0; a < 3; ++a) { lo[a] = std::min(lo[a], q[a]); hi[a] = std::max(hi[a], q[a]); }
    const double size = len(sub(hi, lo));
    const double ext = std::max({ hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2] });
    Grid G;
    G.h = ext / std::max(16, voxels);
    const double offVox = offset * size / G.h;
    const int pad = int(std::ceil(offVox)) + 4;
    G.o = { lo[0] - pad * G.h, lo[1] - pad * G.h, lo[2] - pad * G.h };
    G.nx = int(std::ceil((hi[0] - lo[0]) / G.h)) + 1 + 2 * pad;
    G.ny = int(std::ceil((hi[1] - lo[1]) / G.h)) + 1 + 2 * pad;
    G.nz = int(std::ceil((hi[2] - lo[2]) / G.h)) + 1 + 2 * pad;
    const std::vector<char> inside = insideVoxels(S, G);

    // Distance (voxels) from the inside region, chamfered over 26 neighbours.
    std::vector<double> dist(G.size(), 1e9);
    for (size_t v = 0; v < G.size(); ++v) if (inside[v]) dist[v] = 0.0;
    for (int pass = 0; pass < 4; ++pass) {
        const int sgn = (pass % 2 == 0) ? 1 : -1;
        for (int kk = 1; kk < G.nz - 1; ++kk)
            for (int jj = 1; jj < G.ny - 1; ++jj)
                for (int ii = 1; ii < G.nx - 1; ++ii) {
                    const int i = sgn > 0 ? ii : G.nx - 1 - ii, j = sgn > 0 ? jj : G.ny - 1 - jj, k = sgn > 0 ? kk : G.nz - 1 - kk;
                    const int v = G.idx(i, j, k);
                    double best = dist[v];
                    for (int d = 0; d < 27; ++d) {
                        const int di = d % 3 - 1, dj = (d / 3) % 3 - 1, dk = d / 9 - 1;
                        if (!di && !dj && !dk) continue;
                        best = std::min(best, dist[G.idx(i + di, j + dj, k + dk)] + std::sqrt(double(di * di + dj * dj + dk * dk)));
                    }
                    dist[v] = best;
                }
    }
    // The field: > 0 inside the envelope; softened once so the surface is not
    // a staircase (the offset keeps it outside the part).
    std::vector<double> f(G.size());
    for (size_t v = 0; v < G.size(); ++v) f[v] = offVox - dist[v];
    for (int pass = 0; pass < 2; ++pass) {
        std::vector<double> g = f;
        for (int k = 1; k < G.nz - 1; ++k)
            for (int j = 1; j < G.ny - 1; ++j)
                for (int i = 1; i < G.nx - 1; ++i) {
                    double sum = 0.0;
                    for (int d = 0; d < 27; ++d) sum += f[G.idx(i + d % 3 - 1, j + (d / 3) % 3 - 1, k + d / 9 - 1)];
                    g[G.idx(i, j, k)] = sum / 27.0;
                }
        f.swap(g);
    }

    // Marching tetrahedra (six per cube, the same split in every cube, so
    // neighbouring cubes agree and the surface is watertight).
    static const int tets[6][4] = { { 0, 1, 3, 7 }, { 0, 1, 5, 7 }, { 0, 2, 3, 7 },
                                    { 0, 2, 6, 7 }, { 0, 4, 5, 7 }, { 0, 4, 6, 7 } };
    MeshData out;
    std::unordered_map<long long, uint32_t> edgePt;
    auto corner = [&](int i, int j, int k, int c) { return G.idx(i + (c & 1), j + ((c >> 1) & 1), k + ((c >> 2) & 1)); };
    auto point = [&](int a, int b) {
        const long long key = edgeKey(a, b);
        auto it = edgePt.find(key);
        if (it != edgePt.end()) return it->second;
        const double t = f[a] / (f[a] - f[b]);
        auto pos = [&](int v) { return G.pos(v % G.nx, (v / G.nx) % G.ny, v / (G.nx * G.ny)); };
        const V3 p = add(pos(a), mul(sub(pos(b), pos(a)), t));
        const uint32_t id = out.addVertex(p[0], p[1], p[2]);
        edgePt.emplace(key, id);
        return id;
    };
    auto posV = [&](int v) { return G.pos(v % G.nx, (v / G.nx) % G.ny, v / (G.nx * G.ny)); };
    auto emitTri = [&](uint32_t a, uint32_t b, uint32_t c, int inV, int outV) {
        const V3 pa = { out.pos[3 * a], out.pos[3 * a + 1], out.pos[3 * a + 2] };
        const V3 pb = { out.pos[3 * b], out.pos[3 * b + 1], out.pos[3 * b + 2] };
        const V3 pc = { out.pos[3 * c], out.pos[3 * c + 1], out.pos[3 * c + 2] };
        const V3 n = cross(sub(pb, pa), sub(pc, pa));
        if (dot(n, sub(posV(outV), posV(inV))) < 0) std::swap(b, c);   // face outward
        if (a == b || b == c || a == c) return;
        out.tris.push_back(a); out.tris.push_back(b); out.tris.push_back(c);
    };
    for (int k = 0; k < G.nz - 1; ++k)
        for (int j = 0; j < G.ny - 1; ++j)
            for (int i = 0; i < G.nx - 1; ++i)
                for (const auto &tt : tets) {
                    int v[4], in[4], out4[4], ni = 0, no = 0;
                    for (int c = 0; c < 4; ++c) {
                        v[c] = corner(i, j, k, tt[c]);
                        if (f[v[c]] > 0) in[ni++] = v[c]; else out4[no++] = v[c];
                    }
                    if (ni == 0 || ni == 4) continue;
                    if (ni == 1) {
                        emitTri(point(in[0], out4[0]), point(in[0], out4[1]), point(in[0], out4[2]), in[0], out4[0]);
                    } else if (ni == 3) {
                        emitTri(point(out4[0], in[0]), point(out4[0], in[1]), point(out4[0], in[2]), in[0], out4[0]);
                    } else {
                        const uint32_t a = point(in[0], out4[0]), b = point(in[0], out4[1]);
                        const uint32_t c = point(in[1], out4[0]), d = point(in[1], out4[1]);
                        emitTri(a, b, d, in[0], out4[0]);
                        emitTri(a, d, c, in[0], out4[0]);
                    }
                }

    // Taubin smoothing (no shrinking).
    const int nvtx = out.vertexCount();
    std::vector<std::vector<int>> nb(nvtx);
    for (int t = 0; t < out.triangleCount(); ++t)
        for (int k = 0; k < 3; ++k) {
            const int a = int(out.tris[3 * t + k]), b = int(out.tris[3 * t + (k + 1) % 3]);
            nb[a].push_back(b);
            nb[b].push_back(a);
        }
    for (auto &l : nb) { std::sort(l.begin(), l.end()); l.erase(std::unique(l.begin(), l.end()), l.end()); }
    for (int it = 0; it < 12; ++it) {
        const double lam = (it % 2 == 0) ? 0.5 : -0.53;
        QVector<float> np = out.pos;
        for (int v = 0; v < nvtx; ++v) {
            if (nb[v].empty()) continue;
            double m[3] = { 0, 0, 0 };
            for (int w : nb[v]) for (int a = 0; a < 3; ++a) m[a] += out.pos[3 * w + a];
            for (int a = 0; a < 3; ++a) {
                m[a] /= nb[v].size();
                np[3 * v + a] = float(out.pos[3 * v + a] + lam * (m[a] - out.pos[3 * v + a]));
            }
        }
        out.pos.swap(np);
    }
    out.computeBounds();
    out.computeNormals();
    if (out.triangleCount() == 0 && error) *error = QStringLiteral("the envelope came out empty");
    return out;
}

HarmonicFit::SplitHealth HarmonicFit::splitHealth(const QVector<Part> &parts)
{
    SplitHealth h;
    QElapsedTimer tm;
    tm.start();
    for (const Part &P : parts) {
        // The full enclosing fit (envelope + pole-to-pole fit, thicker if the
        // part sticks out), coarse.
        Options o;
        o.nu = 16;
        o.nv = 16;
        o.nw = 4;
        o.voxels = 40;
        o.maxIter = 15;
        o.closedEnds = true;
        double off = 0.05, usedOff = 0.0;
        Result best;
        int bestOut = std::numeric_limits<int>::max(), usedTry = 0;
        for (int t = 0; t < 3; ++t, off *= 1.6) {
            QString err;
            const MeshData env = envelope(P.mesh, off, 32, &err);
            if (env.triangleCount() == 0) break;
            Result r = fit(env, o);
            if (!r.ok) continue;
            const int outN = outsideCount(r, P.mesh);
            if (outN < bestOut) { bestOut = outN; best = r; usedTry = t; usedOff = off; }
            if (outN == 0) break;
        }
        if (!best.ok) {
            ++h.failed;
            h.lines << QStringLiteral("%1: no trial fit").arg(P.name);
            continue;
        }
        QString err;
        const Trivariate tv = toTrivariate(best, P.name, &err);
        const Check ck = tv.isValid() ? checkJacobian(tv, 14) : Check();
        if (!tv.isValid()) ++h.failed;
        if (ck.nonPositive > 0) { ++h.folded; h.folds += ck.nonPositive; }
        h.retries += usedTry;
        h.outside += bestOut;
        h.lines << QStringLiteral("%1: %2 fold(s) of %3 samples, envelope %4%, %5 vertex(es) outside")
                       .arg(P.name).arg(ck.nonPositive).arg(ck.samples).arg(100.0 * usedOff, 0, 'f', 1).arg(bestOut);
    }
    h.score = 1e7 * h.failed + 1e5 * h.folded + 100.0 * h.folds + 10.0 * h.retries + 0.1 * h.outside;
    h.ok = true;
    h.ms = tm.elapsed();
    return h;
}
