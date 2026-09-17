//
// MaterialField - implementation: the voxel fill, its 3D prefix sum, and the O(1)
// box queries and connectivity tests the splitter asks for.
//

#include "MaterialField.h"

#include <QHash>

#include <algorithm>
#include <cmath>
#include <cstdio>

// Fills the voxel grid by column parity - one pass over the triangles - then
// builds the 3D prefix sum the queries below read.
MaterialField MaterialField::build(const MeshData &mesh, int maxRes)
{
    MaterialField f;
    if (mesh.triangleCount() == 0)
        return f;

    double ext[3], org[3];
    double longest = 0.0;
    for (int a = 0; a < 3; ++a) {
        org[a] = double(mesh.bmin[a]);
        ext[a] = double(mesh.bmax[a]) - org[a];
        longest = qMax(longest, ext[a]);
    }
    if (!(longest > 0.0))
        return f;

    maxRes = qBound(8, maxRes, 256);
    for (int a = 0; a < 3; ++a) {
        f.m_n[a]    = qBound(4, int(std::lround(maxRes * ext[a] / longest)), maxRes);
        f.m_cell[a] = ext[a] / f.m_n[a];
        if (!(f.m_cell[a] > 0.0))
            return MaterialField();
    }

    const int nx = f.m_n[0], ny = f.m_n[1], nz = f.m_n[2];

    QVector<QVector<float>> crossings(nx * ny);

    for (int t = 0; t + 2 < mesh.tris.size(); t += 3) {
        const float *A = &mesh.pos[mesh.tris[t + 0] * 3];
        const float *B = &mesh.pos[mesh.tris[t + 1] * 3];
        const float *C = &mesh.pos[mesh.tris[t + 2] * 3];

        const double ax = A[0] - org[0], ay = A[1] - org[1];
        const double bx = B[0] - org[0], by = B[1] - org[1];
        const double cx = C[0] - org[0], cy = C[1] - org[1];
        const double det = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
        if (std::fabs(det) < 1e-12)
            continue;

        const double loX = qMin(ax, qMin(bx, cx)), hiX = qMax(ax, qMax(bx, cx));
        const double loY = qMin(ay, qMin(by, cy)), hiY = qMax(ay, qMax(by, cy));

        const int i0 = qMax(0,      int(std::floor(loX / f.m_cell[0] - 0.5)));
        const int i1 = qMin(nx - 1, int(std::ceil (hiX / f.m_cell[0] - 0.5)));
        const int j0 = qMax(0,      int(std::floor(loY / f.m_cell[1] - 0.5)));
        const int j1 = qMin(ny - 1, int(std::ceil (hiY / f.m_cell[1] - 0.5)));

        const double jx = 0.5 + 1.0 / 512.0, jy = 0.5 + 1.0 / 337.0;
        for (int i = i0; i <= i1; ++i) {
            const double px = (i + jx) * f.m_cell[0];
            for (int j = j0; j <= j1; ++j) {
                const double py = (j + jy) * f.m_cell[1];

                const double w0 = ((bx - px) * (cy - py) - (by - py) * (cx - px)) / det;
                const double w1 = ((cx - px) * (ay - py) - (cy - py) * (ax - px)) / det;
                const double w2 = 1.0 - w0 - w1;
                if (w0 < 0.0 || w1 < 0.0 || w2 < 0.0)
                    continue;

                const double z = w0 * (double(A[2]) - org[2])
                               + w1 * (double(B[2]) - org[2])
                               + w2 * (double(C[2]) - org[2]);
                crossings[i * ny + j].append(float(z));
            }
        }
    }

    QVector<quint8> occ(nx * ny * nz, 0);
    for (int i = 0; i < nx; ++i)
        for (int j = 0; j < ny; ++j) {
            QVector<float> &zs = crossings[i * ny + j];
            if (zs.size() < 2)
                continue;
            std::sort(zs.begin(), zs.end());

            const int pairs = zs.size() & ~1;
            for (int p = 0; p + 1 < pairs; p += 2) {
                const int k0 = qMax(0,      int(std::ceil (zs[p]     / f.m_cell[2] - 0.5)));
                const int k1 = qMin(nz - 1, int(std::floor(zs[p + 1] / f.m_cell[2] - 0.5)));
                for (int k = k0; k <= k1; ++k)
                    occ[(i * ny + j) * nz + k] = 1;
            }
        }

    f.m_occ = occ;

    const double vox = f.m_cell[0] * f.m_cell[1] * f.m_cell[2];
    f.m_sum.assign(qsizetype(nx + 1) * (ny + 1) * (nz + 1), 0.0);
    for (int i = 1; i <= nx; ++i)
        for (int j = 1; j <= ny; ++j)
            for (int k = 1; k <= nz; ++k) {
                const double here = occ[((i - 1) * ny + (j - 1)) * nz + (k - 1)] ? vox : 0.0;
                f.m_sum[f.sumIndex(i, j, k)] =
                      here
                    + f.m_sum[f.sumIndex(i - 1, j,     k    )]
                    + f.m_sum[f.sumIndex(i,     j - 1, k    )]
                    + f.m_sum[f.sumIndex(i,     j,     k - 1)]
                    - f.m_sum[f.sumIndex(i - 1, j - 1, k    )]
                    - f.m_sum[f.sumIndex(i - 1, j,     k - 1)]
                    - f.m_sum[f.sumIndex(i,     j - 1, k - 1)]
                    + f.m_sum[f.sumIndex(i - 1, j - 1, k - 1)];
            }
    return f;
}

double MaterialField::total() const
{
    if (!isValid())
        return 0.0;
    return m_sum[sumIndex(m_n[0], m_n[1], m_n[2])];
}

// Voxel index range for a box, half-open: voxels whose centre lies inside.
bool MaterialField::range(const double lo[3], const double hi[3],
                          int a[3], int b[3]) const
{
    for (int d = 0; d < 3; ++d) {
        a[d] = qBound(0, int(std::ceil (lo[d] / m_cell[d] - 0.5)),     m_n[d]);
        b[d] = qBound(0, int(std::floor(hi[d] / m_cell[d] - 0.5)) + 1, m_n[d]);
        if (b[d] <= a[d])
            return false;
    }
    return true;
}

// Counts the separate lumps of material inside a box.
int MaterialField::lumpStats(const double lo[3], const double hi[3],
                             int *biggest, int *total) const
{
    *biggest = *total = 0;
    if (!isValid())
        return 0;
    int a[3], b[3];
    if (!range(lo, hi, a, b))
        return 0;

    const int nx = b[0] - a[0], ny = b[1] - a[1], nz = b[2] - a[2];
    QVector<quint8> seen(qsizetype(nx) * ny * nz, 0);
    int lumps = 0;

    for (int i = a[0]; i < b[0]; ++i)
    for (int j = a[1]; j < b[1]; ++j)
    for (int k = a[2]; k < b[2]; ++k) {
        if (!m_occ[occIndex(i, j, k)])
            continue;
        ++*total;
        const int start = ((i - a[0]) * ny + (j - a[1])) * nz + (k - a[2]);
        if (seen[start])
            continue;
        ++lumps;
        int size = 0;
        int lo_i = 1 << 30, hi_i = -1, lo_j = 1 << 30, hi_j = -1, lo_k = 1 << 30, hi_k = -1;
        QVector<int> stack;
        stack.append(start);
        seen[start] = 1;
        while (!stack.isEmpty()) {
            const int idx = stack.takeLast();
            ++size;
            const int li = idx / (ny * nz), lj = (idx / nz) % ny, lk = idx % nz;
            lo_i = qMin(lo_i, li); hi_i = qMax(hi_i, li);
            lo_j = qMin(lo_j, lj); hi_j = qMax(hi_j, lj);
            lo_k = qMin(lo_k, lk); hi_k = qMax(hi_k, lk);
            static const int st[6][3] = { {1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1} };
            for (int d = 0; d < 6; ++d) {
                const int ni = li + st[d][0], nj = lj + st[d][1], nk = lk + st[d][2];
                if (ni < 0 || nj < 0 || nk < 0 || ni >= nx || nj >= ny || nk >= nz)
                    continue;
                const int nIdx = (ni * ny + nj) * nz + nk;
                if (seen[nIdx] || !m_occ[occIndex(a[0]+ni, a[1]+nj, a[2]+nk)])
                    continue;
                seen[nIdx] = 1;
                stack.append(nIdx);
            }
        }
        if (size > *biggest) *biggest = size;
        if (size > 500)
            std::printf("          lump %d: %d voxels  i[%d..%d] j[%d..%d] k[%d..%d]\n",
                        lumps, size, lo_i, hi_i, lo_j, hi_j, lo_k, hi_k);
    }
    *total = 0;
    for (int i = a[0]; i < b[0]; ++i)
    for (int j = a[1]; j < b[1]; ++j)
    for (int k = a[2]; k < b[2]; ++k)
        if (m_occ[occIndex(i, j, k)]) ++*total;
    return lumps;
}

// Whether the material in a box is a single lump: seed at the first occupied
// voxel and flood through 6-neighbours, confined to the box.
bool MaterialField::isConnected(const double lo[3], const double hi[3]) const
{
    if (!isValid())
        return true;

    int a[3], b[3];
    if (!range(lo, hi, a, b))
        return true;

    const int nx = b[0] - a[0], ny = b[1] - a[1], nz = b[2] - a[2];

    int total = 0, seed = -1;
    for (int i = a[0]; i < b[0]; ++i)
        for (int j = a[1]; j < b[1]; ++j)
            for (int k = a[2]; k < b[2]; ++k)
                if (m_occ[occIndex(i, j, k)]) {
                    ++total;
                    if (seed < 0)
                        seed = ((i - a[0]) * ny + (j - a[1])) * nz + (k - a[2]);
                }
    if (total <= 1)
        return true;

    QVector<quint8> seen(qsizetype(nx) * ny * nz, 0);
    QVector<int> stack;
    stack.append(seed);
    seen[seed] = 1;
    int reached = 0;

    while (!stack.isEmpty()) {
        const int idx = stack.takeLast();
        ++reached;
        const int li = idx / (ny * nz);
        const int lj = (idx / nz) % ny;
        const int lk = idx % nz;

        static const int step[6][3] = { {1,0,0}, {-1,0,0}, {0,1,0},
                                        {0,-1,0}, {0,0,1}, {0,0,-1} };
        for (int d = 0; d < 6; ++d) {
            const int ni = li + step[d][0];
            const int nj = lj + step[d][1];
            const int nk = lk + step[d][2];
            if (ni < 0 || nj < 0 || nk < 0 || ni >= nx || nj >= ny || nk >= nz)
                continue;
            const int nIdx = (ni * ny + nj) * nz + nk;
            if (seen[nIdx] || !m_occ[occIndex(a[0] + ni, a[1] + nj, a[2] + nk)])
                continue;
            seen[nIdx] = 1;
            stack.append(nIdx);
        }
    }
    return reached == total;
}

// Material volume in a box: inclusion-exclusion over the eight corners of the
// prefix sum, so O(1) however large the box.
double MaterialField::volumeIn(const double lo[3], const double hi[3]) const
{
    if (!isValid())
        return 0.0;

    int a[3], b[3];
    if (!range(lo, hi, a, b))
        return 0.0;

    return m_sum[sumIndex(b[0], b[1], b[2])]
         - m_sum[sumIndex(a[0], b[1], b[2])]
         - m_sum[sumIndex(b[0], a[1], b[2])]
         - m_sum[sumIndex(b[0], b[1], a[2])]
         + m_sum[sumIndex(a[0], a[1], b[2])]
         + m_sum[sumIndex(a[0], b[1], a[2])]
         + m_sum[sumIndex(b[0], a[1], a[2])]
         - m_sum[sumIndex(a[0], a[1], a[2])];
}
