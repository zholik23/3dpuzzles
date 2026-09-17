//
// IritSolid - implementation: MeshData to IRIT polygons with a chosen winding,
// plus the welding and orientation fixes the booleans need.
//

#include "IritSolid.h"

#include <cmath>
#include <QHash>
#include <QSet>
#include <algorithm>
#include <cmath>

namespace IritSolid {

// Signed volume by the divergence theorem: positive when the mesh is wound
// outward, negative when inward.
double signedVolume(const MeshData &m)
{
    double v = 0.0;
    for (int t = 0; t + 2 < m.tris.size(); t += 3) {
        const float *a = &m.pos[m.tris[t + 0] * 3];
        const float *b = &m.pos[m.tris[t + 1] * 3];
        const float *c = &m.pos[m.tris[t + 2] * 3];
        v += double(a[0]) * (double(b[1]) * double(c[2]) - double(b[2]) * double(c[1]))
           - double(a[1]) * (double(b[0]) * double(c[2]) - double(b[2]) * double(c[0]))
           + double(a[2]) * (double(b[0]) * double(c[1]) - double(b[1]) * double(c[0]));
    }
    return v / 6.0;
}

// MeshData to IRIT polygons. Measures how the mesh arrived and corrects it to
// the winding the caller asked for, since IRIT decides inside from outside by
// the winding.
IritPrsrObjectStruct *fromMesh(const MeshData &m, Winding w)
{
    IritPrsrPolygonStruct *head = NULL;

    const bool isInward = signedVolume(m) < 0.0;
    const bool flip = isInward != (w == Winding::Inward);

    for (int t = 0; t + 2 < m.tris.size(); t += 3) {
        const int i0 = 0, i1 = flip ? 2 : 1, i2 = flip ? 1 : 2;
        const float *p[3] = { &m.pos[m.tris[t + i0] * 3],
                              &m.pos[m.tris[t + i1] * 3],
                              &m.pos[m.tris[t + i2] * 3] };

        IritPrsrVertexStruct *v2 = IritPrsrAllocVertex2(NULL);
        IritPrsrVertexStruct *v1 = IritPrsrAllocVertex2(v2);
        IritPrsrVertexStruct *v0 = IritPrsrAllocVertex2(v1);
        IritPrsrVertexStruct *v[3] = { v0, v1, v2 };
        (void)v2;

        for (int k = 0; k < 3; ++k)
            for (int c = 0; c < 3; ++c)
                v[k] -> Coord[c] = IrtRType(p[k][c]);

        IritPrsrPolygonStruct *poly = IritPrsrAllocPolygon(0, v0, head);
        if (!IritPrsrUpdatePolyPlane(poly)) {
            poly -> PVertex = NULL;
            IritPrsrFreeVertexList(v0);
            IritPrsrFreePolygon(poly);
            continue;
        }
        head = poly;
    }

    if (head == NULL)
        return NULL;

    IritPrsrOpenPolysToClosed(head);
    return IritPrsrGenPOLYObject(head);
}

// Closes every vertex list back on itself. The booleans build adjacencies and
// reject a NULL-terminated list outright.
void closeLists(IritPrsrObjectStruct *o)
{
    for (; o != NULL; o = o -> Pnext)
        if (IRIT_PRSR_IS_POLY_OBJ(o) && o -> U.Pl != NULL)
            IritPrsrOpenPolysToClosed(o -> U.Pl);
}

bool isClosed(const MeshData &m)
{
    QSet<quint64> dir;
    for (int t = 0; t + 2 < m.tris.size(); t += 3) {
        dir.insert((quint64(m.tris[t + 0]) << 32) | m.tris[t + 1]);
        dir.insert((quint64(m.tris[t + 1]) << 32) | m.tris[t + 2]);
        dir.insert((quint64(m.tris[t + 2]) << 32) | m.tris[t + 0]);
    }
    for (const quint64 k : dir) {
        const quint64 back = (quint64(quint32(k & 0xffffffffu)) << 32)
                           | quint32(k >> 32);
        if (!dir.contains(back))
            return false;
    }
    return true;
}

// Welds vertices within eps. Hashes on a grid of side eps and looks across the
// 27-cell neighbourhood, since two near points can fall in different cells.
void weldClose(MeshData *m, double eps)
{
    const int nVert = int(m->pos.size() / 3);
    if (nVert == 0)
        return;

    if (eps <= 0.0) {
        double diag = 0.0;
        for (int a = 0; a < 3; ++a) {
            const double d = double(m->bmax[a]) - double(m->bmin[a]);
            diag += d * d;
        }
        eps = 1e-6 * std::sqrt(diag);
        if (eps <= 0.0)
            return;
    }

    const double inv = 1.0 / eps;
    QHash<quint64, QVector<int>> grid;
    grid.reserve(nVert);
    const auto cellKey = [](qint64 x, qint64 y, qint64 z) {
        return (quint64(x & 0x1FFFFF) << 42) | (quint64(y & 0x1FFFFF) << 21) |
                quint64(z & 0x1FFFFF);
    };

    QVector<int> remap(nVert, -1);
    QVector<float> kept;
    kept.reserve(m->pos.size());

    for (int v = 0; v < nVert; ++v) {
        const double x = m->pos[v * 3 + 0];
        const double y = m->pos[v * 3 + 1];
        const double z = m->pos[v * 3 + 2];
        const qint64 cx = qint64(std::floor(x * inv));
        const qint64 cy = qint64(std::floor(y * inv));
        const qint64 cz = qint64(std::floor(z * inv));

        int found = -1;
        for (qint64 dx = -1; dx <= 1 && found < 0; ++dx)
        for (qint64 dy = -1; dy <= 1 && found < 0; ++dy)
        for (qint64 dz = -1; dz <= 1 && found < 0; ++dz) {
            for (int c : grid.value(cellKey(cx + dx, cy + dy, cz + dz))) {
                const double ex = double(kept[c * 3 + 0]) - x;
                const double ey = double(kept[c * 3 + 1]) - y;
                const double ez = double(kept[c * 3 + 2]) - z;
                if (ex * ex + ey * ey + ez * ez <= eps * eps) { found = c; break; }
            }
        }

        if (found < 0) {
            found = int(kept.size() / 3);
            kept.append(float(x)); kept.append(float(y)); kept.append(float(z));
            grid[cellKey(cx, cy, cz)].append(found);
        }
        remap[v] = found;
    }

    QVector<uint32_t> tris;
    tris.reserve(m->tris.size());
    for (int t = 0; t + 2 < m->tris.size(); t += 3) {
        const uint32_t a = remap[m->tris[t + 0]];
        const uint32_t b = remap[m->tris[t + 1]];
        const uint32_t c = remap[m->tris[t + 2]];
        if (a == b || b == c || a == c)
            continue;
        tris.append(a); tris.append(b); tris.append(c);
    }

    m->pos  = kept;
    m->tris = tris;
    m->edges.clear();
    m->finalize();
}

// Makes every shell consistently wound, deciding per shell rather than once for
// the mesh: a piece can be several lumps, and an inside-out one hides inside a
// larger correct one and silently subtracts when measured.
void orientConsistently(MeshData *m)
{
    weldClose(m);

    const int nTri = m->triangleCount();
    if (nTri < 2)
        return;

    QHash<quint64, QVector<int>> edgeTris;
    edgeTris.reserve(nTri * 3);
    const auto key = [](uint32_t a, uint32_t b) {
        const uint32_t lo = qMin(a, b), hi = qMax(a, b);
        return (quint64(lo) << 32) | hi;
    };
    for (int t = 0; t < nTri; ++t)
        for (int e = 0; e < 3; ++e)
            edgeTris[key(m->tris[t * 3 + e], m->tris[t * 3 + (e + 1) % 3])].append(t);

    QVector<bool> seen(nTri, false);
    QVector<int>  comp(nTri, -1);
    int nComp = 0;

    for (int root = 0; root < nTri; ++root) {
        if (seen[root])
            continue;
        seen[root] = true;
        const int cid = nComp++;
        comp[root] = cid;

        QVector<int> stack;
        stack.append(root);
        while (!stack.isEmpty()) {
            const int t = stack.takeLast();
            for (int e = 0; e < 3; ++e) {
                const uint32_t a = m->tris[t * 3 + e];
                const uint32_t b = m->tris[t * 3 + (e + 1) % 3];
                for (int n : edgeTris.value(key(a, b))) {
                    if (n == t || seen[n])
                        continue;
                    bool sameDir = false;
                    for (int f = 0; f < 3; ++f)
                        if (m->tris[n * 3 + f] == a &&
                            m->tris[n * 3 + (f + 1) % 3] == b)
                            sameDir = true;
                    if (sameDir)
                        std::swap(m->tris[n * 3 + 1], m->tris[n * 3 + 2]);
                    seen[n] = true;
                    comp[n] = cid;
                    stack.append(n);
                }
            }
        }
    }

    QVector<double> vol(nComp, 0.0);
    for (int t = 0; t < nTri; ++t) {
        const float *A = &m->pos[m->tris[t * 3 + 0] * 3];
        const float *B = &m->pos[m->tris[t * 3 + 1] * 3];
        const float *C = &m->pos[m->tris[t * 3 + 2] * 3];
        vol[comp[t]] +=
              double(A[0]) * (double(B[1]) * double(C[2]) - double(B[2]) * double(C[1]))
            - double(A[1]) * (double(B[0]) * double(C[2]) - double(B[2]) * double(C[0]))
            + double(A[2]) * (double(B[0]) * double(C[1]) - double(B[1]) * double(C[0]));
    }
    for (int t = 0; t < nTri; ++t)
        if (vol[comp[t]] < 0.0)
            std::swap(m->tris[t * 3 + 1], m->tris[t * 3 + 2]);

    m->computeNormals();
}

}
