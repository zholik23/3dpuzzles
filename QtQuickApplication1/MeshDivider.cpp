#include "MeshDivider.h"

#include <QHash>
#include <QSet>
#include <QRandomGenerator>
#include <algorithm>
#include <cmath>

namespace {

struct V3 { double x, y, z;
    double operator[](int i) const { return i == 0 ? x : (i == 1 ? y : z); }
};

inline V3 lerp(const V3 &a, const V3 &b, double t)
{
    return { a.x + (b.x - a.x) * t,
             a.y + (b.y - a.y) * t,
             a.z + (b.z - a.z) * t };
}

// Sutherland-Hodgman against one axis-aligned half-space. `keepAbove` keeps the
// side with coordinate >= val.
void clipHalfSpace(const QVector<V3> &in, QVector<V3> &out,
                   int axis, double val, bool keepAbove)
{
    out.clear();
    const int n = in.size();
    if (n == 0)
        return;

    for (int i = 0; i < n; ++i) {
        const V3 &a = in[i];
        const V3 &b = in[(i + 1) % n];
        const double da = keepAbove ? a[axis] - val : val - a[axis];
        const double db = keepAbove ? b[axis] - val : val - b[axis];

        if (da >= 0.0)
            out.append(a);
        if ((da >= 0.0) != (db >= 0.0)) {
            const double t = da / (da - db);
            V3 p = lerp(a, b, t);
            p = { axis == 0 ? val : p.x,          // pin exactly onto the plane,
                  axis == 1 ? val : p.y,          // so cap detection can match
                  axis == 2 ? val : p.z };        // on equality later
            out.append(p);
        }
    }
}

// Clips a triangle to a box. The result of clipping a convex polygon by planes
// is convex, so the caller can fan-triangulate it.
void clipToBox(const V3 tri[3], const double lo[3], const double hi[3],
               QVector<V3> &poly, QVector<V3> &scratch)
{
    poly.clear();
    poly.append(tri[0]); poly.append(tri[1]); poly.append(tri[2]);

    for (int a = 0; a < 3 && !poly.isEmpty(); ++a) {
        clipHalfSpace(poly, scratch, a, lo[a], true);
        poly.swap(scratch);
        if (poly.isEmpty())
            return;
        clipHalfSpace(poly, scratch, a, hi[a], false);
        poly.swap(scratch);
    }
}

// ---------------------------------------------------------------- capping --

struct VKey {
    float x, y, z;
    bool operator==(const VKey &o) const { return x == o.x && y == o.y && z == o.z; }
};
inline size_t qHash(const VKey &k, size_t seed = 0) { return qHashMulti(seed, k.x, k.y, k.z); }

// Ear clipping of one planar loop, projected by dropping `axis`.
void earClip(const QVector<uint32_t> &loop, const QVector<float> &pos,
             int axis, bool flip, QVector<uint32_t> *tris)
{
    const int n = loop.size();
    if (n < 3)
        return;

    const int u = (axis + 1) % 3;
    const int v = (axis + 2) % 3;

    QVector<double> px(n), py(n);
    for (int i = 0; i < n; ++i) {
        px[i] = double(pos[loop[i] * 3 + u]);
        py[i] = double(pos[loop[i] * 3 + v]);
    }

    double area2 = 0.0;
    for (int i = 0, j = n - 1; i < n; j = i++)
        area2 += px[j] * py[i] - px[i] * py[j];

    QVector<int> ring(n);
    for (int i = 0; i < n; ++i)
        ring[i] = (area2 >= 0.0) ? i : (n - 1 - i);

    const auto cross2 = [&](int a, int b, int c) {
        return (px[b] - px[a]) * (py[c] - py[a]) - (py[b] - py[a]) * (px[c] - px[a]);
    };

    int remaining = n, guard = 2 * n;
    while (remaining > 3 && guard-- > 0) {
        bool clipped = false;
        for (int i = 0; i < remaining; ++i) {
            const int ia = ring[(i + remaining - 1) % remaining];
            const int ib = ring[i];
            const int ic = ring[(i + 1) % remaining];
            if (cross2(ia, ib, ic) <= 0.0)
                continue;

            bool ear = true;
            for (int k = 0; k < remaining && ear; ++k) {
                const int ip = ring[k];
                if (ip == ia || ip == ib || ip == ic)
                    continue;
                if (cross2(ia, ib, ip) >= 0.0 &&
                    cross2(ib, ic, ip) >= 0.0 &&
                    cross2(ic, ia, ip) >= 0.0)
                    ear = false;
            }
            if (!ear)
                continue;

            if (flip) { tris->append(loop[ic]); tris->append(loop[ib]); tris->append(loop[ia]); }
            else      { tris->append(loop[ia]); tris->append(loop[ib]); tris->append(loop[ic]); }
            ring.remove(i);
            --remaining;
            guard = 2 * remaining;
            clipped = true;
            break;
        }
        if (!clipped)
            break;
    }
    for (int i = 1; i + 1 < remaining; ++i) {
        if (flip) { tris->append(loop[ring[i + 1]]); tris->append(loop[ring[i]]); tris->append(loop[ring[0]]); }
        else      { tris->append(loop[ring[0]]); tris->append(loop[ring[i]]); tris->append(loop[ring[i + 1]]); }
    }
}

// Closes the cut faces of one clipped cell.
//
// The open boundary of the clipped shell only gives PART of each cut face's
// outline - the part where the model's own surface meets the plane. Where the
// solid runs all the way into a cell corner, the rest of the outline is the
// cell face's own border, and no triangle exists along it. So the loop is
// walked in two alternating modes: follow the shell's open edges while they
// last, then run along the face border until the next open edge starts.
//
bool pointInside(const MeshData &m, const double p[3]);   // defined below

// Returns true if the piece is a closed solid afterwards. Attempting a face
// that had no hole is harmless - the walk simply finds nothing to do - so the
// only meaningful verdict is whether any open edge is left at the end.
bool capCell(MeshData &out, const double lo[3], const double hi[3],
             bool inwardWinding, const MeshData &source)
{
    // Directed boundary edges: an edge with no opposite-direction twin is on
    // the open rim. The cap traverses it the other way round, the way the
    // missing triangle would have.
    const auto dirKey = [](uint32_t x, uint32_t y) {
        return (quint64(x) << 32) | y;
    };

    // Each directed edge remembers the third vertex of the triangle it came
    // from. That is what tells a genuine hole from a face the shell already
    // covers: if the source triangle lies IN the cell face, the missing
    // material is not on that face at all, and capping there would just
    // duplicate the shell with the opposite winding.
    QHash<quint64, uint32_t> directed;
    for (int t = 0; t + 2 < out.tris.size(); t += 3) {
        const uint32_t a = out.tris[t + 0], b = out.tris[t + 1], c = out.tris[t + 2];
        directed.insert(dirKey(a, b), c);
        directed.insert(dirKey(b, c), a);
        directed.insert(dirKey(c, a), b);
    }

    struct RimEdge { uint32_t from, to, apex; };
    QVector<RimEdge> rim;                        // already reversed for the cap
    for (auto it = directed.constBegin(); it != directed.constEnd(); ++it) {
        const uint32_t x = uint32_t(it.key() >> 32);
        const uint32_t y = uint32_t(it.key() & 0xffffffffu);
        if (!directed.contains(dirKey(y, x)))
            rim.append({ y, x, it.value() });
    }
    if (rim.isEmpty())
        return true;                             // already closed

    for (int f = 0; f < 6; ++f) {
        const int    axis = f / 2;
        const bool   high = (f % 2) != 0;
        const double val  = high ? hi[axis] : lo[axis];

        // 2D frame chosen so that counter-clockwise in (u, v) means the cap's
        // normal points out of the cell.
        const int u = high ? (axis + 1) % 3 : (axis + 2) % 3;
        const int v = high ? (axis + 2) % 3 : (axis + 1) % 3;

        const double u0 = qMin(lo[u], hi[u]), u1 = qMax(lo[u], hi[u]);
        const double v0 = qMin(lo[v], hi[v]), v1 = qMax(lo[v], hi[v]);
        const double du = u1 - u0, dv = v1 - v0;
        if (!(du > 0.0) || !(dv > 0.0))
            continue;

        // Vertex coordinates are stored as float, while the cell bounds are
        // doubles. A padded bound like -1e-5 does not survive the round trip,
        // so every "is it on this plane / border" test compares in float.
        const float fval = float(val);
        const auto onPlane = [&](uint32_t i) {
            return out.pos[i * 3 + axis] == fval;
        };

        QHash<uint32_t, uint32_t> forward;
        for (const RimEdge &e : rim)
            if (onPlane(e.from) && onPlane(e.to) && !onPlane(e.apex))
                forward.insert(e.from, e.to);
        // A face with no model surface on it is either wholly outside the
        // material or wholly inside it. Filling the "wholly inside" case by
        // hand was tried and consistently over-filled on finer grids, so it is
        // left alone: an honestly open piece beats invented material.
        if (forward.isEmpty())
            continue;

        const float fu0 = float(u0), fu1 = float(u1);
        const float fv0 = float(v0), fv1 = float(v1);

        // Perimeter parameter, counter-clockwise from (u0, v0).
        const auto param = [&](double pu, double pv) -> double {
            if (float(pv) == fv0) return pu - u0;
            if (float(pu) == fu1) return du + (pv - v0);
            if (float(pv) == fv1) return du + dv + (u1 - pu);
            return 2.0 * du + dv + (v1 - pv);
        };
        const auto onBorder = [&](uint32_t i) {
            const float pu = out.pos[i * 3 + u], pv = out.pos[i * 3 + v];
            return pu == fu0 || pu == fu1 || pv == fv0 || pv == fv1;
        };

        // The four corners, as vertices, created on demand.
        QHash<VKey, quint32> vindex;
        for (int i = 0; i < out.vertexCount(); ++i)
            vindex.insert(VKey { out.pos[i * 3 + 0], out.pos[i * 3 + 1], out.pos[i * 3 + 2] },
                          quint32(i));
        const auto vertexAt = [&](double pu, double pv) {
            double c[3];
            c[axis] = val; c[u] = pu; c[v] = pv;
            const VKey k { float(c[0]) + 0.0f, float(c[1]) + 0.0f, float(c[2]) + 0.0f };
            const auto it = vindex.constFind(k);
            if (it != vindex.constEnd())
                return uint32_t(*it);
            const uint32_t id = out.addVertex(k.x, k.y, k.z);
            vindex.insert(k, id);
            return id;
        };

        const uint32_t corner[4] = { vertexAt(u0, v0), vertexAt(u1, v0),
                                     vertexAt(u1, v1), vertexAt(u0, v1) };
        const double perim = 2.0 * (du + dv);

        // Which way the rim runs depends on the source mesh's winding, which we
        // cannot assume - an inward-wound STL is still a perfectly good STL. So
        // build the loops both ways and keep the one that comes out
        // counter-clockwise, i.e. enclosing the material rather than the rest
        // of the cell face. Going the wrong way round picks up the far corners
        // and caps the complement of the piece.
        struct Attempt { bool ok; double area; QVector<QVector<uint32_t>> loops; };

        // The rim direction is NOT a free choice - it is fixed by the source
        // mesh's winding, since the cap traverses each open edge the opposite
        // way from the triangle that owns it. Only the border closure has to
        // be matched to it: an inward-wound mesh gives clockwise cap loops, so
        // the run along the cell face border has to go clockwise as well, and
        // the finished loop has negative area rather than positive.
        const auto build = [&](bool borderCW) {
            Attempt res { true, 0.0, {} };
            const QHash<uint32_t, uint32_t> &nextOf = forward;

            struct Stop { double t; uint32_t vert; };
            QVector<Stop> stops;
            for (int c = 0; c < 4; ++c)
                stops.append({ param(out.pos[corner[c] * 3 + u],
                                     out.pos[corner[c] * 3 + v]), corner[c] });
            for (auto it = nextOf.constBegin(); it != nextOf.constEnd(); ++it)
                if (onBorder(it.key()))
                    stops.append({ param(out.pos[it.key() * 3 + u],
                                         out.pos[it.key() * 3 + v]), it.key() });

            QHash<uint32_t, bool> started;
            for (auto it = nextOf.constBegin(); it != nextOf.constEnd() && res.ok; ++it) {
                if (started.value(it.key(), false))
                    continue;

                QVector<uint32_t> loop;
                uint32_t cur = it.key();
                const uint32_t first = cur;
                int guard = 4 * (nextOf.size() + stops.size()) + 16;

                while (guard-- > 0) {
                    loop.append(cur);
                    started[cur] = true;

                    const auto nx = nextOf.constFind(cur);
                    if (nx != nextOf.constEnd()) {
                        cur = *nx;
                    }
                    else if (onBorder(cur)) {
                        // Run along the face border, the same way round as the
                        // rim, to the next vertex that matters.
                        const double t = param(out.pos[cur * 3 + u], out.pos[cur * 3 + v]);
                        uint32_t nextVert = uint32_t(-1);
                        double   bestGap  = perim * 2.0;
                        for (const Stop &st : stops) {
                            if (st.vert == cur)
                                continue;
                            double gap = borderCW ? (t - st.t) : (st.t - t);
                            if (gap <= 1e-12)
                                gap += perim;
                            if (gap < bestGap) { bestGap = gap; nextVert = st.vert; }
                        }
                        if (nextVert == uint32_t(-1)) { res.ok = false; break; }
                        cur = nextVert;
                    }
                    else {
                        res.ok = false;          // dead end away from the border
                        break;
                    }

                    if (cur == first)
                        break;
                }
                if (guard <= 0)
                    res.ok = false;

                if (res.ok && loop.size() >= 3) {
                    double a2 = 0.0;
                    for (int i = 0, j = loop.size() - 1; i < loop.size(); j = i++)
                        a2 += double(out.pos[loop[j] * 3 + u]) * double(out.pos[loop[i] * 3 + v])
                            - double(out.pos[loop[i] * 3 + u]) * double(out.pos[loop[j] * 3 + v]);
                    res.area += 0.5 * a2;
                    res.loops.append(loop);
                }
            }
            return res;
        };

        // Which way the rim runs is fixed by the source mesh's winding, which
        // was measured once from its signed volume. Guessing per face does not
        // work: the loop going the wrong way round encloses the leftover strip
        // of the cell face, and that is a positively-wound region too.
        const auto areaOk = [&](const Attempt &x) {
            return x.ok && !x.loops.isEmpty() &&
                   (inwardWinding ? x.area < 0.0 : x.area > 0.0);
        };

        Attempt a = build(inwardWinding);
        if (!areaOk(a)) {
            Attempt b = build(!inwardWinding);
            if (areaOk(b))
                a = b;
        }

        const bool faceOk = areaOk(a);
        if (faceOk) {
            // earClip always emits counter-clockwise in its own (axis+1,
            // axis+2) frame, which faces +axis. The low face has to look the
            // other way - and so does everything, if the source mesh is wound
            // inward, because the cap has to agree with the shell it closes.
            for (const QVector<uint32_t> &loop : a.loops)
                earClip(loop, out.pos, axis, high == inwardWinding, &out.tris);
        }

    }

    // Final verdict: is every edge now shared by two triangles?
    QSet<quint64> after;
    for (int t = 0; t + 2 < out.tris.size(); t += 3) {
        after.insert(dirKey(out.tris[t + 0], out.tris[t + 1]));
        after.insert(dirKey(out.tris[t + 1], out.tris[t + 2]));
        after.insert(dirKey(out.tris[t + 2], out.tris[t + 0]));
    }
    for (const quint64 k : after) {
        const uint32_t x = uint32_t(k >> 32);
        const uint32_t y = uint32_t(k & 0xffffffffu);
        if (!after.contains(dirKey(y, x)))
            return false;
    }
    return true;
}

// Six times the signed volume of a closed mesh. Positive means the triangles
// are wound counter-clockwise seen from outside - the usual convention. A file
// wound the other way is still perfectly valid, and the caps have to follow it.
double signedVolume6(const MeshData &m)
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
    return v;
}

// Ray cast from p along a deliberately skew direction, counting crossings of
// the model. An odd count means p is inside.
//
// This is what makes a fine division of a solid work at all: a cell buried in
// the middle of the material contains no surface triangles, so clipping alone
// yields nothing for it. Such a cell is not empty - it is completely full.
bool pointInside(const MeshData &m, const double p[3])
{
    // Skew on purpose, so the ray is very unlikely to graze an edge or vertex.
    const double d[3] = { 0.5773502691, 0.5773502692, 0.5773502693 };
    int crossings = 0;

    for (int t = 0; t + 2 < m.tris.size(); t += 3) {
        const float *a = &m.pos[m.tris[t + 0] * 3];
        const float *b = &m.pos[m.tris[t + 1] * 3];
        const float *c = &m.pos[m.tris[t + 2] * 3];

        const double e1[3] = { b[0] - a[0], b[1] - a[1], b[2] - a[2] };
        const double e2[3] = { c[0] - a[0], c[1] - a[1], c[2] - a[2] };
        const double pv[3] = { d[1] * e2[2] - d[2] * e2[1],
                               d[2] * e2[0] - d[0] * e2[2],
                               d[0] * e2[1] - d[1] * e2[0] };
        const double det = e1[0] * pv[0] + e1[1] * pv[1] + e1[2] * pv[2];
        if (det > -1e-12 && det < 1e-12)
            continue;                            // ray parallel to the triangle

        const double inv = 1.0 / det;
        const double tv[3] = { p[0] - a[0], p[1] - a[1], p[2] - a[2] };
        const double uu = (tv[0] * pv[0] + tv[1] * pv[1] + tv[2] * pv[2]) * inv;
        if (uu < 0.0 || uu > 1.0)
            continue;

        const double qv[3] = { tv[1] * e1[2] - tv[2] * e1[1],
                               tv[2] * e1[0] - tv[0] * e1[2],
                               tv[0] * e1[1] - tv[1] * e1[0] };
        const double vv = (d[0] * qv[0] + d[1] * qv[1] + d[2] * qv[2]) * inv;
        if (vv < 0.0 || uu + vv > 1.0)
            continue;

        const double hit = (e2[0] * qv[0] + e2[1] * qv[1] + e2[2] * qv[2]) * inv;
        if (hit > 1e-9)
            ++crossings;
    }
    return (crossings & 1) != 0;
}

// Emits a closed axis-aligned box, wound to match the source mesh.
void emitBox(MeshData &out, const double lo[3], const double hi[3], bool inwardWinding)
{
    uint32_t v[8];
    for (int i = 0; i < 8; ++i)
        v[i] = out.addVertex((i & 1) ? hi[0] : lo[0],
                             (i & 2) ? hi[1] : lo[1],
                             (i & 4) ? hi[2] : lo[2]);

    // Each face as two triangles, wound counter-clockwise seen from outside.
    static const int face[6][4] = {
        { 0, 2, 3, 1 },   // z = lo   (looking from -z)
        { 4, 5, 7, 6 },   // z = hi
        { 0, 1, 5, 4 },   // y = lo
        { 2, 6, 7, 3 },   // y = hi
        { 0, 4, 6, 2 },   // x = lo
        { 1, 3, 7, 5 }    // x = hi
    };
    for (int f = 0; f < 6; ++f) {
        const int *q = face[f];
        if (inwardWinding) {
            out.tris.push_back(v[q[0]]); out.tris.push_back(v[q[2]]); out.tris.push_back(v[q[1]]);
            out.tris.push_back(v[q[0]]); out.tris.push_back(v[q[3]]); out.tris.push_back(v[q[2]]);
        }
        else {
            out.tris.push_back(v[q[0]]); out.tris.push_back(v[q[1]]); out.tris.push_back(v[q[2]]);
            out.tris.push_back(v[q[0]]); out.tris.push_back(v[q[2]]); out.tris.push_back(v[q[3]]);
        }
    }
}

// Splits triangles lying on a cell face until no edge is longer than `target`.
//
// This only matters for curved cuts: a cut face is created flat, and if it stays
// one big quad then un-warping just tilts that quad instead of bending it. Both
// triangles sharing an edge compute the same midpoint from the same two
// endpoints, and midpoints are welded, so no T-junctions appear.
void subdivideCutFaces(MeshData &m, const double lo[3], const double hi[3],
                       double target)
{
    if (!(target > 0.0) || m.tris.isEmpty())
        return;

    QHash<VKey, quint32> index;
    for (int i = 0; i < m.vertexCount(); ++i)
        index.insert(VKey { m.pos[i * 3 + 0], m.pos[i * 3 + 1], m.pos[i * 3 + 2] },
                     quint32(i));

    const auto midpoint = [&](uint32_t a, uint32_t b) {
        const float x = 0.5f * (m.pos[a * 3 + 0] + m.pos[b * 3 + 0]);
        const float y = 0.5f * (m.pos[a * 3 + 1] + m.pos[b * 3 + 1]);
        const float z = 0.5f * (m.pos[a * 3 + 2] + m.pos[b * 3 + 2]);
        const VKey k { x + 0.0f, y + 0.0f, z + 0.0f };
        const auto it = index.constFind(k);
        if (it != index.constEnd())
            return uint32_t(*it);
        const uint32_t id = m.addVertex(x, y, z);
        index.insert(k, id);
        return id;
    };

    // A triangle is on a cut face when all three of its vertices share one
    // coordinate that equals a cell bound.
    const auto onCutFace = [&](uint32_t a, uint32_t b, uint32_t c) {
        for (int ax = 0; ax < 3; ++ax) {
            const float f0 = float(lo[ax]), f1 = float(hi[ax]);
            const float pa = m.pos[a * 3 + ax], pb = m.pos[b * 3 + ax],
                        pc = m.pos[c * 3 + ax];
            if ((pa == f0 && pb == f0 && pc == f0) ||
                (pa == f1 && pb == f1 && pc == f1))
                return true;
        }
        return false;
    };

    const auto longestEdge = [&](uint32_t a, uint32_t b, uint32_t c) {
        const auto d2 = [&](uint32_t x, uint32_t y) {
            const double dx = double(m.pos[x * 3 + 0]) - m.pos[y * 3 + 0];
            const double dy = double(m.pos[x * 3 + 1]) - m.pos[y * 3 + 1];
            const double dz = double(m.pos[x * 3 + 2]) - m.pos[y * 3 + 2];
            return dx * dx + dy * dy + dz * dz;
        };
        return std::sqrt(qMax(d2(a, b), qMax(d2(b, c), d2(c, a))));
    };

    // Each pass quarters the offending triangles; six is far more resolution
    // than any sane wave needs, and it bounds the growth.
    for (int pass = 0; pass < 6; ++pass) {
        QVector<uint32_t> next;
        next.reserve(m.tris.size());
        bool changed = false;

        for (int t = 0; t + 2 < m.tris.size(); t += 3) {
            const uint32_t a = m.tris[t + 0], b = m.tris[t + 1], c = m.tris[t + 2];
            if (!onCutFace(a, b, c) || longestEdge(a, b, c) <= target) {
                next.push_back(a); next.push_back(b); next.push_back(c);
                continue;
            }
            const uint32_t ab = midpoint(a, b);
            const uint32_t bc = midpoint(b, c);
            const uint32_t ca = midpoint(c, a);
            next.push_back(a);  next.push_back(ab); next.push_back(ca);
            next.push_back(ab); next.push_back(b);  next.push_back(bc);
            next.push_back(ca); next.push_back(bc); next.push_back(c);
            next.push_back(ab); next.push_back(bc); next.push_back(ca);
            changed = true;
        }
        m.tris = next;
        if (!changed)
            break;
    }
}

// Evenly spaced interior cut positions for one axis.
void evenPlanes(double lo, double hi, int n, QVector<double> *out)
{
    for (int s = 1; s < n; ++s)
        out->append(lo + (hi - lo) * double(s) / double(n));
}

} // namespace

// ------------------------------------------------------------ spec builders --

MeshDivisionSpec MeshDivider::uniform(const MeshData &mesh, const int counts[3])
{
    MeshDivisionSpec spec;
    for (int a = 0; a < 3; ++a)
        evenPlanes(mesh.bmin[a], mesh.bmax[a], qMax(1, counts[a]), &spec.planes[a]);
    spec.note = QStringLiteral("uniform %1 x %2 x %3")
                    .arg(spec.cells(0)).arg(spec.cells(1)).arg(spec.cells(2));
    return spec;
}

MeshDivisionSpec MeshDivider::jittered(const MeshData &mesh, const int counts[3],
                                       double jitter, quint32 seed)
{
    MeshDivisionSpec spec = uniform(mesh, counts);
    jitter = qBound(0.0, jitter, 0.45);          // beyond 0.45 cuts can cross

    QRandomGenerator rng(seed);
    for (int a = 0; a < 3; ++a) {
        const int n = spec.cells(a);
        if (n < 2)
            continue;
        const double slab = (mesh.bmax[a] - mesh.bmin[a]) / double(n);
        for (double &p : spec.planes[a])
            p += (rng.generateDouble() * 2.0 - 1.0) * jitter * slab;
        std::sort(spec.planes[a].begin(), spec.planes[a].end());
    }
    spec.note = QStringLiteral("jittered %1 x %2 x %3 (%4%, seed %5)")
                    .arg(spec.cells(0)).arg(spec.cells(1)).arg(spec.cells(2))
                    .arg(int(jitter * 100)).arg(seed);
    return spec;
}

MeshDivisionSpec MeshDivider::toBuildVolume(const MeshData &mesh,
                                            const double budget[3],
                                            int maxCellsPerAxis)
{
    MeshDivisionSpec spec;
    maxCellsPerAxis = qBound(1, maxCellsPerAxis, 64);

    for (int a = 0; a < 3; ++a) {
        const double extent = double(mesh.bmax[a]) - double(mesh.bmin[a]);
        if (!(extent > 0.0) || budget[a] <= 0.0)
            continue;
        int n = int(std::ceil(extent / budget[a] - 1e-9));
        n = qMax(n, 1);
        if (n > maxCellsPerAxis) {
            n = maxCellsPerAxis;
            spec.capped = true;
        }
        evenPlanes(mesh.bmin[a], mesh.bmax[a], n, &spec.planes[a]);
    }

    double worst[3];
    for (int a = 0; a < 3; ++a)
        worst[a] = (double(mesh.bmax[a]) - double(mesh.bmin[a])) / double(spec.cells(a));

    spec.note = QStringLiteral("fit to %1 x %2 x %3 -> %4 x %5 x %6 cells "
                               "(cell %7 x %8 x %9)")
                    .arg(budget[0], 0, 'g', 3).arg(budget[1], 0, 'g', 3)
                    .arg(budget[2], 0, 'g', 3)
                    .arg(spec.cells(0)).arg(spec.cells(1)).arg(spec.cells(2))
                    .arg(worst[0], 0, 'g', 3).arg(worst[1], 0, 'g', 3)
                    .arg(worst[2], 0, 'g', 3);
    if (spec.capped)
        spec.note += QStringLiteral(" - OVER BUDGET, raise the cell limit above %1")
                         .arg(maxCellsPerAxis);
    return spec;
}


MeshDivisionSpec MeshDivider::balanced(const MeshData &mesh, const int counts[3])
{
    MeshDivisionSpec spec;

    const int nTri = mesh.triangleCount();
    if (nTri == 0)
        return uniform(mesh, counts);

    // Triangle centroid and area along each axis, then cuts at equal-area
    // quantiles. Where the model's material is unevenly spread, this is what
    // stops one piece being nearly empty and its neighbour carrying everything.
    struct Item { double at; double w; };

    for (int a = 0; a < 3; ++a) {
        const int n = qMax(1, counts[a]);
        if (n < 2)
            continue;

        QVector<Item> items;
        items.reserve(nTri);
        double total = 0.0;

        for (int t = 0; t < nTri; ++t) {
            const float *p0 = &mesh.pos[mesh.tris[t * 3 + 0] * 3];
            const float *p1 = &mesh.pos[mesh.tris[t * 3 + 1] * 3];
            const float *p2 = &mesh.pos[mesh.tris[t * 3 + 2] * 3];

            const double ux = p1[0] - p0[0], uy = p1[1] - p0[1], uz = p1[2] - p0[2];
            const double vx = p2[0] - p0[0], vy = p2[1] - p0[1], vz = p2[2] - p0[2];
            const double cx = uy * vz - uz * vy;
            const double cy = uz * vx - ux * vz;
            const double cz = ux * vy - uy * vx;
            const double area = 0.5 * std::sqrt(cx * cx + cy * cy + cz * cz);
            if (!(area > 0.0))
                continue;

            items.append({ (double(p0[a]) + double(p1[a]) + double(p2[a])) / 3.0, area });
            total += area;
        }
        if (items.isEmpty() || !(total > 0.0)) {
            evenPlanes(mesh.bmin[a], mesh.bmax[a], n, &spec.planes[a]);
            continue;
        }

        std::sort(items.begin(), items.end(),
                  [](const Item &x, const Item &y) { return x.at < y.at; });

        double accrued = 0.0;
        int    next    = 1;
        for (int i = 0; i < items.size() && next < n; ++i) {
            accrued += items[i].w;
            while (next < n && accrued >= total * double(next) / double(n)) {
                spec.planes[a].append(items[i].at);
                ++next;
            }
        }
        // Any cut that landed on the boundary would make an empty slab.
        for (double &p : spec.planes[a])
            p = qBound(double(mesh.bmin[a]) + 1e-9, p, double(mesh.bmax[a]) - 1e-9);
        std::sort(spec.planes[a].begin(), spec.planes[a].end());
    }

    spec.note = QStringLiteral("area-balanced %1 x %2 x %3")
                    .arg(spec.cells(0)).arg(spec.cells(1)).arg(spec.cells(2));
    return spec;
}

// ------------------------------------------------------------------ divide --

bool MeshDivider::divide(const MeshData &mesh, const MeshDivisionSpec &spec,
                         QVector<PuzzlePiece> *pieces, QString *report,
                         double cutDetail)
{
    pieces->clear();
    if (mesh.triangleCount() == 0) {
        if (report) *report = QStringLiteral("The model has no triangles to divide.");
        return false;
    }

    // Ordered cell boundaries per axis, padded so nothing falls outside.
    // No padding on the outer bounds. Clipping keeps points exactly on a plane,
    // so the model's extremes survive - and a padded cell would leave a sliver
    // of empty face beyond the material, which makes the cut outline ambiguous.
    QVector<double> b[3];
    for (int a = 0; a < 3; ++a) {
        b[a].append(double(mesh.bmin[a]));
        for (double p : spec.planes[a])
            if (p > mesh.bmin[a] && p < mesh.bmax[a])
                b[a].append(p);
        b[a].append(double(mesh.bmax[a]));
        std::sort(b[a].begin(), b[a].end());
    }

    const int n[3] = { b[0].size() - 1, b[1].size() - 1, b[2].size() - 1 };
    const int nCells = n[0] * n[1] * n[2];
    if (nCells < 1) {
        if (report) *report = QStringLiteral("The cut planes left no cells.");
        return false;
    }

    // Bucket each triangle into the cells its bbox overlaps, so each cell only
    // clips the triangles that can possibly reach it.
    QVector<QVector<int>> bucket(nCells);
    const auto cellAt = [&](int i, int j, int k) { return (i * n[1] + j) * n[2] + k; };
    const auto slabOf = [&](int a, double v) {
        int lo = 0, hi = n[a] - 1;
        while (lo < hi && v >= b[a][lo + 1]) ++lo;
        while (hi > lo && v <  b[a][hi])     --hi;
        return qBound(0, lo, n[a] - 1);
    };

    const int nTri = mesh.triangleCount();
    for (int t = 0; t < nTri; ++t) {
        double lo[3] = {  1e300,  1e300,  1e300 };
        double hi[3] = { -1e300, -1e300, -1e300 };
        for (int c = 0; c < 3; ++c) {
            const float *p = &mesh.pos[mesh.tris[t * 3 + c] * 3];
            for (int a = 0; a < 3; ++a) {
                lo[a] = qMin(lo[a], double(p[a]));
                hi[a] = qMax(hi[a], double(p[a]));
            }
        }
        int i0 = slabOf(0, lo[0]), i1 = slabOf(0, hi[0]);
        int j0 = slabOf(1, lo[1]), j1 = slabOf(1, hi[1]);
        int k0 = slabOf(2, lo[2]), k1 = slabOf(2, hi[2]);
        for (int i = i0; i <= i1; ++i)
            for (int j = j0; j <= j1; ++j)
                for (int k = k0; k <= k1; ++k)
                    bucket[cellAt(i, j, k)].append(t);
    }

    // Winding is a property of the file, measured once.
    const bool inwardWinding = signedVolume6(mesh) < 0.0;

    int openPieces = 0, empty = 0;

    QVector<V3> poly, scratch;
    for (int i = 0; i < n[0]; ++i)
        for (int j = 0; j < n[1]; ++j)
            for (int k = 0; k < n[2]; ++k) {
                const double lo[3] = { b[0][i],     b[1][j],     b[2][k]     };
                const double hi[3] = { b[0][i + 1], b[1][j + 1], b[2][k + 1] };

                PuzzlePiece piece;
                piece.i = i; piece.j = j; piece.k = k;
                for (int a = 0; a < 3; ++a) { piece.p0[a] = lo[a]; piece.p1[a] = hi[a]; }

                MeshData &out = piece.mesh;
                out.sourceKind = mesh.sourceKind;

                QHash<VKey, quint32> index;
                const auto vertexFor = [&](const V3 &p) {
                    const VKey key { float(p.x) + 0.0f, float(p.y) + 0.0f, float(p.z) + 0.0f };
                    const auto it = index.constFind(key);
                    if (it != index.constEnd())
                        return uint32_t(*it);
                    const uint32_t id = out.addVertex(key.x, key.y, key.z);
                    index.insert(key, id);
                    return id;
                };

                for (int t : bucket[cellAt(i, j, k)]) {
                    V3 tri[3];
                    for (int c = 0; c < 3; ++c) {
                        const float *p = &mesh.pos[mesh.tris[t * 3 + c] * 3];
                        tri[c] = { double(p[0]), double(p[1]), double(p[2]) };
                    }
                    clipToBox(tri, lo, hi, poly, scratch);
                    if (poly.size() < 3)
                        continue;

                    const uint32_t a0 = vertexFor(poly[0]);
                    for (int q = 1; q + 1 < poly.size(); ++q) {
                        const uint32_t a1 = vertexFor(poly[q]);
                        const uint32_t a2 = vertexFor(poly[q + 1]);
                        if (a0 == a1 || a1 == a2 || a0 == a2)
                            continue;
                        out.tris.push_back(a0);
                        out.tris.push_back(a1);
                        out.tris.push_back(a2);
                    }
                }

                if (out.tris.isEmpty()) {
                    // No surface crossed this cell. Either it is outside the
                    // model, or it is buried inside it and completely full.
                    const double mid[3] = { 0.5 * (lo[0] + hi[0]),
                                            0.5 * (lo[1] + hi[1]),
                                            0.5 * (lo[2] + hi[2]) };
                    if (!pointInside(mesh, mid)) {
                        ++empty;
                        continue;
                    }
                    emitBox(out, lo, hi, inwardWinding);
                    out.finalize();
                    for (int a = 0; a < 3; ++a) {
                        piece.centre[a] = 0.5f * (out.bmin[a] + out.bmax[a]);
                        piece.size[a]   = out.bmax[a] - out.bmin[a];
                    }
                    pieces->append(std::move(piece));
                    continue;
                }

                // ---- cap the cuts ------------------------------------------
                // A cut face has to be closed with new material, or the piece
                // is a shell rather than a solid. capCell walks the open
                // boundary and fills each cell face it lies on.
                if (!capCell(out, lo, hi, inwardWinding, mesh))
                    ++openPieces;

                subdivideCutFaces(out, lo, hi, cutDetail);
                out.finalize();
                if (out.isEmpty()) {
                    ++empty;
                    continue;
                }
                for (int a = 0; a < 3; ++a) {
                    piece.centre[a] = 0.5f * (out.bmin[a] + out.bmax[a]);
                    piece.size[a]   = out.bmax[a] - out.bmin[a];
                }
                pieces->append(std::move(piece));
            }

    if (pieces->isEmpty()) {
        if (report) *report = QStringLiteral("Clipping produced no pieces.");
        return false;
    }

    if (report) {
        QStringList notes;
        if (empty > 0)
            notes << QStringLiteral("%1 of %2 cells were empty (the model does not "
                                    "reach them)").arg(empty).arg(nCells);
        if (openPieces > 0)
            notes << QStringLiteral("%1 piece(s) are still open shells - the cut "
                                    "outline was not a simple loop there (a hollow "
                                    "wall or an interior void), so it was left open "
                                    "rather than filled wrongly").arg(openPieces);
        *report = notes.join(QStringLiteral("; "));
    }
    return true;
}


// ------------------------------------------------------------ curved cuts --

MeshData MeshDivider::warp(const MeshData &mesh, const CutWarp &w)
{
    MeshData out = mesh;
    if (!w.active() || mesh.pos.isEmpty())
        return out;

    // Normalisation uses the ORIGINAL bounds, and unwarp() is handed the same
    // ones, so the two maps are exact inverses.
    const double bmin[3] = { mesh.bmin[0], mesh.bmin[1], mesh.bmin[2] };
    const double bmax[3] = { mesh.bmax[0], mesh.bmax[1], mesh.bmax[2] };
    const double diag    = mesh.diagonal();

    for (int i = 0; i + 2 < out.pos.size(); i += 3) {
        double p[3] = { out.pos[i], out.pos[i + 1], out.pos[i + 2] };
        w.apply(p, bmin, bmax, diag);
        out.pos[i]     = float(p[0]);
        out.pos[i + 1] = float(p[1]);
        out.pos[i + 2] = float(p[2]);
    }
    out.computeBounds();
    out.computeNormals();
    return out;
}

void MeshDivider::unwarp(QVector<PuzzlePiece> *pieces, const CutWarp &w,
                         const float bminF[3], const float bmaxF[3],
                         double diagonal)
{
    if (!w.active())
        return;

    const double bmin[3] = { bminF[0], bminF[1], bminF[2] };
    const double bmax[3] = { bmaxF[0], bmaxF[1], bmaxF[2] };

    for (PuzzlePiece &piece : *pieces) {
        MeshData &m = piece.mesh;
        for (int i = 0; i + 2 < m.pos.size(); i += 3) {
            double p[3] = { m.pos[i], m.pos[i + 1], m.pos[i + 2] };
            w.invert(p, bmin, bmax, diagonal);
            m.pos[i]     = float(p[0]);
            m.pos[i + 1] = float(p[1]);
            m.pos[i + 2] = float(p[2]);
        }
        // Positions moved, so bounds, normals and the edge list are all stale.
        m.finalize();
        for (int a = 0; a < 3; ++a) {
            piece.centre[a] = 0.5f * (m.bmin[a] + m.bmax[a]);
            piece.size[a]   = m.bmax[a] - m.bmin[a];
        }
    }
}

bool MeshDivider::divideCells(const MeshData &mesh, const QVector<CellBox> &cells,
                              QVector<PuzzlePiece> *pieces, QString *report,
                              double cutDetail)
{
    pieces->clear();
    if (mesh.triangleCount() == 0) {
        if (report) *report = QStringLiteral("The model has no triangles to divide.");
        return false;
    }
    if (cells.isEmpty()) {
        if (report) *report = QStringLiteral("No cells to cut into.");
        return false;
    }

    const bool inwardWinding = signedVolume6(mesh) < 0.0;
    const int  nTri = mesh.triangleCount();

    // Triangle bounds once, so each cell only clips what can reach it.
    QVector<float> tlo(nTri * 3), thi(nTri * 3);
    for (int t = 0; t < nTri; ++t)
        for (int a = 0; a < 3; ++a) {
            float lo =  1e30f, hi = -1e30f;
            for (int c = 0; c < 3; ++c) {
                const float v = mesh.pos[mesh.tris[t * 3 + c] * 3 + a];
                lo = qMin(lo, v);
                hi = qMax(hi, v);
            }
            tlo[t * 3 + a] = lo;
            thi[t * 3 + a] = hi;
        }

    int openPieces = 0, empty = 0;
    QVector<V3> poly, scratch;

    for (int ci = 0; ci < cells.size(); ++ci) {
        const double *lo = cells[ci].lo;
        const double *hi = cells[ci].hi;

        PuzzlePiece piece;
        piece.i = ci;                    // BSP leaves have no grid position;
        piece.j = 0; piece.k = 0;        // the index is the identity
        for (int a = 0; a < 3; ++a) { piece.p0[a] = lo[a]; piece.p1[a] = hi[a]; }

        MeshData &out = piece.mesh;
        out.sourceKind = mesh.sourceKind;

        QHash<VKey, quint32> index;
        const auto vertexFor = [&](const V3 &p) {
            const VKey key { float(p.x) + 0.0f, float(p.y) + 0.0f, float(p.z) + 0.0f };
            const auto it = index.constFind(key);
            if (it != index.constEnd())
                return uint32_t(*it);
            const uint32_t id = out.addVertex(key.x, key.y, key.z);
            index.insert(key, id);
            return id;
        };

        for (int t = 0; t < nTri; ++t) {
            bool overlaps = true;
            for (int a = 0; a < 3 && overlaps; ++a)
                if (double(thi[t * 3 + a]) < lo[a] || double(tlo[t * 3 + a]) > hi[a])
                    overlaps = false;
            if (!overlaps)
                continue;

            V3 tri[3];
            for (int c = 0; c < 3; ++c) {
                const float *p = &mesh.pos[mesh.tris[t * 3 + c] * 3];
                tri[c] = { double(p[0]), double(p[1]), double(p[2]) };
            }
            clipToBox(tri, lo, hi, poly, scratch);
            if (poly.size() < 3)
                continue;

            const uint32_t a0 = vertexFor(poly[0]);
            for (int q = 1; q + 1 < poly.size(); ++q) {
                const uint32_t a1 = vertexFor(poly[q]);
                const uint32_t a2 = vertexFor(poly[q + 1]);
                if (a0 == a1 || a1 == a2 || a0 == a2)
                    continue;
                out.tris.push_back(a0);
                out.tris.push_back(a1);
                out.tris.push_back(a2);
            }
        }

        if (out.tris.isEmpty()) {
            // No surface crossed this cell: it is either outside the model or
            // buried inside it and completely full.
            const double mid[3] = { 0.5 * (lo[0] + hi[0]),
                                    0.5 * (lo[1] + hi[1]),
                                    0.5 * (lo[2] + hi[2]) };
            if (!pointInside(mesh, mid)) {
                ++empty;
                continue;
            }
            emitBox(out, lo, hi, inwardWinding);
        }
        else {
            if (!capCell(out, lo, hi, inwardWinding, mesh))
                ++openPieces;
            subdivideCutFaces(out, lo, hi, cutDetail);
        }

        out.finalize();
        if (out.isEmpty()) {
            ++empty;
            continue;
        }
        for (int a = 0; a < 3; ++a) {
            piece.centre[a] = 0.5f * (out.bmin[a] + out.bmax[a]);
            piece.size[a]   = out.bmax[a] - out.bmin[a];
        }
        pieces->append(std::move(piece));
    }

    if (pieces->isEmpty()) {
        if (report) *report = QStringLiteral("Clipping produced no pieces.");
        return false;
    }
    if (report) {
        QStringList notes;
        if (empty > 0)
            notes << QStringLiteral("%1 of %2 cells were empty (the model does not "
                                    "reach them)").arg(empty).arg(cells.size());
        if (openPieces > 0)
            notes << QStringLiteral("%1 piece(s) are still open shells - the cut "
                                    "outline was not a simple loop there, so it was "
                                    "left open rather than filled wrongly")
                     .arg(openPieces);
        *report = notes.join(QStringLiteral("; "));
    }
    return true;
}
