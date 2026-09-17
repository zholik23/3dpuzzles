//
// MeshData - implementation: bounds, normals, edges, and the derived measures.
//

#include "MeshData.h"
#include <QHash>
#include <cmath>
#include <limits>

void MeshData::computeBounds()
{
    if (pos.isEmpty()) {
        bmin[0] = bmin[1] = bmin[2] = 0.0f;
        bmax[0] = bmax[1] = bmax[2] = 0.0f;
        return;
    }
    for (int k = 0; k < 3; ++k) {
        bmin[k] =  std::numeric_limits<float>::max();
        bmax[k] = -std::numeric_limits<float>::max();
    }
    for (int i = 0; i + 2 < pos.size(); i += 3)
        for (int k = 0; k < 3; ++k) {
            const float v = pos[i + k];
            if (v < bmin[k]) bmin[k] = v;
            if (v > bmax[k]) bmax[k] = v;
        }
}

void MeshData::computeNormals()
{
    triNrm.resize(tris.size());
    for (int t = 0; t + 2 < tris.size(); t += 3) {
        const float *a = &pos[tris[t + 0] * 3];
        const float *b = &pos[tris[t + 1] * 3];
        const float *c = &pos[tris[t + 2] * 3];
        const float ux = b[0] - a[0], uy = b[1] - a[1], uz = b[2] - a[2];
        const float vx = c[0] - a[0], vy = c[1] - a[1], vz = c[2] - a[2];
        float nx = uy * vz - uz * vy;
        float ny = uz * vx - ux * vz;
        float nz = ux * vy - uy * vx;
        const float len = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (len > 1e-20f) { nx /= len; ny /= len; nz /= len; }
        else              { nx = 0.0f; ny = 0.0f; nz = 1.0f; }
        triNrm[t + 0] = nx; triNrm[t + 1] = ny; triNrm[t + 2] = nz;
    }
}

void MeshData::buildEdges()
{
    edges.clear();
    QHash<quint64, char> seen;
    seen.reserve(tris.size());

    auto add = [&](uint32_t a, uint32_t b) {
        if (a == b) return;
        const uint32_t lo = a < b ? a : b, hi = a < b ? b : a;
        const quint64 key = (static_cast<quint64>(lo) << 32) | hi;
        if (seen.contains(key)) return;
        seen.insert(key, 1);
        edges.push_back(lo);
        edges.push_back(hi);
    };

    for (int t = 0; t + 2 < tris.size(); t += 3) {
        add(tris[t + 0], tris[t + 1]);
        add(tris[t + 1], tris[t + 2]);
        add(tris[t + 2], tris[t + 0]);
    }
    edges += polylineEdges;
}

float MeshData::diagonal() const
{
    const float dx = bmax[0] - bmin[0];
    const float dy = bmax[1] - bmin[1];
    const float dz = bmax[2] - bmin[2];
    const float d = std::sqrt(dx * dx + dy * dy + dz * dz);
    return d > 1e-12f ? d : 1.0f;
}

void MeshData::centroid(float c[3]) const
{
    for (int k = 0; k < 3; ++k)
        c[k] = 0.5f * (bmin[k] + bmax[k]);
}
