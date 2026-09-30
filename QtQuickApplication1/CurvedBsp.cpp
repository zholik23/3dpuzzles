//
// CurvedBsp - implementation.
//
#include "CurvedBsp.h"

#include <QRandomGenerator>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <vector>

namespace {

constexpr double kTwoPi = 6.283185307179586;

}  // namespace

CurvedBsp CurvedBsp::build(const Trivariate &tv, int pieces, double bend, int waves,
                           quint32 seed)
{
    CurvedBsp b;
    if (!tv.isValid())
        return b;
    tv.domain(b.m_dom);

    // The app's BSP, unchanged: which boxes, split where. Only the shape of
    // each split is new.
    b.m_tree = PuzzleDivider::buildBspTree(b.m_dom, std::max(1, pieces), 0.25, seed);
    const int nNodes = b.m_tree.size();
    b.m_split.resize(nNodes);
    b.m_bulge.resize(nNodes);

    bend = std::clamp(bend, 0.0, 0.9);
    waves = std::max(1, waves);
    QRandomGenerator rng(seed ^ 0x9e3779b9u);

    // Parents are appended before their children, so one pass in index order
    // sees every node's bulges before it is split.
    for (int n = 0; n < nNodes; ++n) {
        const PuzzleDivider::BspNode &node = b.m_tree[n];
        if (node.isLeaf())
            continue;
        const PuzzleDivider::BspNode &c0 = b.m_tree[node.child[0]];

        Split s;
        for (int a = 0; a < 3; ++a)
            if (std::fabs(c0.box.hi[a] - node.box.hi[a]) > 1e-12) s.axis = a;
        const int a = s.axis;
        s.at = c0.box.hi[a];
        for (int k = 0; k < 3; ++k) { s.lo[k] = node.box.lo[k]; s.hi[k] = node.box.hi[k]; }

        // Room between the split and the cell's own faces along the split axis,
        // those faces possibly bent inward by an earlier split.
        const double roomLo = s.at - node.box.lo[a] - b.m_bulge[n].b[2 * a];
        const double roomHi = node.box.hi[a] - s.at - b.m_bulge[n].b[2 * a + 1];
        s.amp = bend * std::max(0.0, std::min(roomLo, roomHi));
        s.waves = waves;
        s.phase[0] = kTwoPi * rng.generateDouble();
        s.phase[1] = kTwoPi * rng.generateDouble();
        b.m_split[n] = s;
        b.m_internal.append(n);

        // The children inherit the cell's bent faces, plus the new one.
        b.m_bulge[node.child[0]] = b.m_bulge[n];
        b.m_bulge[node.child[1]] = b.m_bulge[n];
        b.m_bulge[node.child[0]].b[2 * a + 1] = s.amp;
        b.m_bulge[node.child[1]].b[2 * a] = s.amp;
    }

    b.m_leafNode = PuzzleDivider::leavesOf(b.m_tree, nullptr);
    b.m_pieceOf = QVector<int>(nNodes, -1);
    for (int i = 0; i < b.m_leafNode.size(); ++i)
        b.m_pieceOf[b.m_leafNode[i]] = i;
    return b;
}

void CurvedBsp::domain(double d[6]) const
{
    for (int k = 0; k < 6; ++k) d[k] = m_dom[k];
}

double CurvedBsp::g(int node, const double p[3]) const
{
    const Split &s = m_split[node];
    const int a = s.axis, bb = (a + 1) % 3, cc = (a + 2) % 3;
    const double sb = (p[bb] - s.lo[bb]) / (s.hi[bb] - s.lo[bb]);
    const double sc = (p[cc] - s.lo[cc]) / (s.hi[cc] - s.lo[cc]);
    const double f = std::sin(kTwoPi * s.waves * sb + s.phase[0]) *
                     std::sin(kTwoPi * s.waves * sc + s.phase[1]);
    return (p[a] - s.at) - s.amp * f;
}

void CurvedBsp::gradG(int node, const double p[3], double out[3]) const
{
    const Split &s = m_split[node];
    const int a = s.axis, bb = (a + 1) % 3, cc = (a + 2) % 3;
    const double Lb = s.hi[bb] - s.lo[bb], Lc = s.hi[cc] - s.lo[cc];
    const double ab = kTwoPi * s.waves * (p[bb] - s.lo[bb]) / Lb + s.phase[0];
    const double ac = kTwoPi * s.waves * (p[cc] - s.lo[cc]) / Lc + s.phase[1];
    out[a]  = 1.0;
    out[bb] = -s.amp * std::cos(ab) * (kTwoPi * s.waves / Lb) * std::sin(ac);
    out[cc] = -s.amp * std::sin(ab) * std::cos(ac) * (kTwoPi * s.waves / Lc);
}

double CurvedBsp::pointOnSplit(int node, double pb, double pc) const
{
    const Split &s = m_split[node];
    const int a = s.axis, bb = (a + 1) % 3, cc = (a + 2) % 3;
    double p[3];
    p[a] = s.at; p[bb] = pb; p[cc] = pc;
    return s.at + (p[a] - s.at) - g(node, p);   // g = (p_a - at) - A f  =>  at + A f
}

int CurvedBsp::classify(const double p[3], double *margin) const
{
    if (m_tree.isEmpty())
        return -1;
    double m = std::numeric_limits<double>::max();
    int n = 0;
    while (!m_tree[n].isLeaf()) {
        const double gv = g(n, p);
        m = std::min(m, std::fabs(gv));
        n = m_tree[n].child[gv >= 0.0 ? 1 : 0];
    }
    if (margin) *margin = m;
    return m_pieceOf[n];
}

bool CurvedBsp::isInSubtree(int node, int root) const
{
    for (int n = node; n >= 0; n = m_tree[n].parent)
        if (n == root) return true;
    return false;
}

double CurvedBsp::pieceField(int piece, const double p[3]) const
{
    double v = std::numeric_limits<double>::max();
    for (int a = 0; a < 3; ++a)
        v = std::min(v, std::min(p[a] - m_dom[2 * a], m_dom[2 * a + 1] - p[a]));
    int n = m_leafNode[piece];
    while (m_tree[n].parent >= 0) {
        const int par = m_tree[n].parent;
        const double side = (m_tree[par].child[1] == n) ? 1.0 : -1.0;
        v = std::min(v, side * g(par, p));
        n = par;
    }
    return v;
}

// Marching tetrahedra over an n^3 grid of D (plus one layer of padding, so the
// domain boundary is a real zero crossing). Six tetrahedra per cube along the
// main diagonal - no case tables - and vertices shared along grid edges.
bool CurvedBsp::meshPiece(const Trivariate &tv, int piece, int n, MeshData *out) const
{
    *out = MeshData();
    if (piece < 0 || piece >= pieceCount() || n < 2)
        return false;

    const int N = n + 3;                               // nodes per axis
    double h[3], x0[3];
    for (int a = 0; a < 3; ++a) {
        h[a] = (m_dom[2 * a + 1] - m_dom[2 * a]) / n;
        x0[a] = m_dom[2 * a] - h[a];
    }
    const auto nodeIdx = [N](int i, int j, int k) { return (size_t(k) * N + j) * N + i; };
    const auto nodePos = [&](size_t id, double q[3]) {
        const int i = int(id % N), j = int((id / N) % N), k = int(id / (size_t(N) * N));
        q[0] = x0[0] + i * h[0]; q[1] = x0[1] + j * h[1]; q[2] = x0[2] + k * h[2];
    };

    std::vector<double> F(size_t(N) * N * N);
    bool any = false;
    for (int k = 0; k < N; ++k)
        for (int j = 0; j < N; ++j)
            for (int i = 0; i < N; ++i) {
                double q[3] = { x0[0] + i * h[0], x0[1] + j * h[1], x0[2] + k * h[2] };
                const double v = pieceField(piece, q);
                F[nodeIdx(i, j, k)] = v;
                any = any || v > 0.0;
            }
    if (!any)
        return false;

    std::unordered_map<uint64_t, uint32_t> edgeVert;
    std::vector<std::array<double, 3>> dpos;           // vertex positions in D

    const auto vertexOn = [&](size_t a, size_t b) -> uint32_t {
        const uint64_t key = (uint64_t(std::min(a, b)) << 32) | uint64_t(std::max(a, b));
        const auto it = edgeVert.find(key);
        if (it != edgeVert.end())
            return it->second;
        double qa[3], qb[3];
        nodePos(a, qa);
        nodePos(b, qb);
        const double fa = F[a], fb = F[b];
        const double t = fa / (fa - fb);
        std::array<double, 3> q;
        for (int c = 0; c < 3; ++c) {
            q[c] = qa[c] + t * (qb[c] - qa[c]);
            q[c] = std::clamp(q[c], m_dom[2 * c], m_dom[2 * c + 1]);
        }
        double r[3];
        tv.evaluate(q[0], q[1], q[2], r);
        const uint32_t id = out->addVertex(r[0], r[1], r[2]);
        dpos.push_back(q);
        edgeVert.emplace(key, id);
        return id;
    };

    // Kuhn subdivision: corner c = x + 2y + 4z.
    static const int kTet[6][4] = { { 0, 1, 3, 7 }, { 0, 1, 5, 7 }, { 0, 2, 3, 7 },
                                    { 0, 2, 6, 7 }, { 0, 4, 5, 7 }, { 0, 4, 6, 7 } };

    const auto addTri = [&](uint32_t v0, uint32_t v1, uint32_t v2, const double inside[3]) {
        // Face the triangle away from the inside (in D; M keeps orientation
        // where det J > 0).
        const auto &p0 = dpos[v0], &p1 = dpos[v1], &p2 = dpos[v2];
        const double e1[3] = { p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2] };
        const double e2[3] = { p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2] };
        const double nn[3] = { e1[1] * e2[2] - e1[2] * e2[1],
                               e1[2] * e2[0] - e1[0] * e2[2],
                               e1[0] * e2[1] - e1[1] * e2[0] };
        const double d = nn[0] * (inside[0] - p0[0]) + nn[1] * (inside[1] - p0[1]) +
                         nn[2] * (inside[2] - p0[2]);
        if (v0 == v1 || v1 == v2 || v0 == v2)
            return;
        out->tris.push_back(v0);
        if (d > 0.0) { out->tris.push_back(v2); out->tris.push_back(v1); }
        else         { out->tris.push_back(v1); out->tris.push_back(v2); }
    };

    for (int k = 0; k < N - 1; ++k)
        for (int j = 0; j < N - 1; ++j)
            for (int i = 0; i < N - 1; ++i) {
                size_t c[8];
                int in = 0;
                for (int q = 0; q < 8; ++q) {
                    c[q] = nodeIdx(i + (q & 1), j + ((q >> 1) & 1), k + ((q >> 2) & 1));
                    if (F[c[q]] > 0.0) ++in;
                }
                if (in == 0 || in == 8)
                    continue;
                for (const auto &t : kTet) {
                    size_t v[4] = { c[t[0]], c[t[1]], c[t[2]], c[t[3]] };
                    size_t ins[4], outs[4];
                    int ni = 0, no = 0;
                    for (size_t w : v) (F[w] > 0.0 ? ins[ni++] : outs[no++]) = w;
                    if (ni == 0 || ni == 4)
                        continue;
                    double ctr[3] = { 0, 0, 0 };
                    for (int q = 0; q < ni; ++q) {
                        double qp[3];
                        nodePos(ins[q], qp);
                        for (int cc = 0; cc < 3; ++cc) ctr[cc] += qp[cc] / ni;
                    }
                    if (ni == 1) {
                        addTri(vertexOn(ins[0], outs[0]), vertexOn(ins[0], outs[1]),
                             vertexOn(ins[0], outs[2]), ctr);
                    } else if (ni == 3) {
                        addTri(vertexOn(outs[0], ins[0]), vertexOn(outs[0], ins[1]),
                             vertexOn(outs[0], ins[2]), ctr);
                    } else {
                        const uint32_t q0 = vertexOn(ins[0], outs[0]);
                        const uint32_t q1 = vertexOn(ins[0], outs[1]);
                        const uint32_t q2 = vertexOn(ins[1], outs[1]);
                        const uint32_t q3 = vertexOn(ins[1], outs[0]);
                        addTri(q0, q1, q2, ctr);
                        addTri(q0, q2, q3, ctr);
                    }
                }
            }

    if (out->tris.isEmpty())
        return false;
    out->sourceKind = QStringLiteral("curved BSP piece (meshed in D, mapped through M)");
    out->finalize();
    return true;
}
