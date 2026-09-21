//
// JointRotation - implementation: rotate the real mesh, reject on bounding boxes,
// then triangle-against-triangle for whatever survives.
//
#include "JointRotation.h"

#include <QtGlobal>

#include <cmath>

namespace {

const double kPi = 3.14159265358979323846;

struct V3 { double x, y, z; };

inline V3 sub(const V3 &a, const V3 &b) { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
inline V3 cross(const V3 &a, const V3 &b)
{
    return { a.y * b.z - a.z * b.y,
             a.z * b.x - a.x * b.z,
             a.x * b.y - a.y * b.x };
}
inline double dot(const V3 &a, const V3 &b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline double len2(const V3 &a) { return dot(a, a); }

// Projection of a triangle onto an axis.
inline void span(const V3 &ax, const V3 &p0, const V3 &p1, const V3 &p2,
                 double *lo, double *hi)
{
    const double a = dot(ax, p0), b = dot(ax, p1), c = dot(ax, p2);
    *lo = qMin(a, qMin(b, c));
    *hi = qMax(a, qMax(b, c));
}

// Separating-axis test for two triangles. Thirteen axes: the two face normals
// and the nine edge-pair cross products, plus a degenerate guard. Returns true
// when they overlap by more than eps.
//
// Touching is not overlap: eps is subtracted from the intersection, so two
// pieces resting face to face do not read as a collision.
bool triTriOverlap(const V3 &a0, const V3 &a1, const V3 &a2,
                   const V3 &b0, const V3 &b1, const V3 &b2,
                   double eps)
{
    V3 axes[13];
    int n = 0;

    const V3 ae[3] = { sub(a1, a0), sub(a2, a1), sub(a0, a2) };
    const V3 be[3] = { sub(b1, b0), sub(b2, b1), sub(b0, b2) };

    axes[n++] = cross(ae[0], ae[1]);          // face normal of A
    axes[n++] = cross(be[0], be[1]);          // face normal of B
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            axes[n++] = cross(ae[i], be[j]);

    for (int i = 0; i < n; ++i) {
        const double l2 = len2(axes[i]);
        if (l2 < 1e-18)
            continue;                          // parallel edges: not separating

        const double inv = 1.0 / std::sqrt(l2);
        const V3 ax = { axes[i].x * inv, axes[i].y * inv, axes[i].z * inv };

        double alo, ahi, blo, bhi;
        span(ax, a0, a1, a2, &alo, &ahi);
        span(ax, b0, b1, b2, &blo, &bhi);

        // A gap on any axis means no overlap at all.
        if (alo > bhi - eps || blo > ahi - eps)
            return false;
    }
    return true;
}

inline bool boxesOverlap(const float alo[3], const float ahi[3],
                         const float blo[3], const float bhi[3], double eps)
{
    for (int a = 0; a < 3; ++a)
        if (double(alo[a]) > double(bhi[a]) - eps ||
            double(blo[a]) > double(ahi[a]) - eps)
            return false;
    return true;
}

inline V3 vertexOf(const MeshData &m, quint32 i)
{
    return { double(m.pos[i * 3 + 0]),
             double(m.pos[i * 3 + 1]),
             double(m.pos[i * 3 + 2]) };
}

// Pulls every vertex toward the mesh centre by `d`, so pieces that merely rest
// against each other are not reported as colliding.
void shrink(MeshData *m, double d)
{
    if (d <= 0.0 || m->pos.isEmpty())
        return;

    double c[3] = { 0, 0, 0 };
    const int nv = int(m->pos.size() / 3);
    for (int v = 0; v < nv; ++v)
        for (int a = 0; a < 3; ++a)
            c[a] += double(m->pos[v * 3 + a]);
    for (int a = 0; a < 3; ++a)
        c[a] /= double(qMax(1, nv));

    for (int v = 0; v < nv; ++v) {
        double dir[3];
        double l = 0.0;
        for (int a = 0; a < 3; ++a) {
            dir[a] = double(m->pos[v * 3 + a]) - c[a];
            l += dir[a] * dir[a];
        }
        l = std::sqrt(l);
        if (l <= d)
            continue;
        const double k = (l - d) / l;
        for (int a = 0; a < 3; ++a)
            m->pos[v * 3 + a] = float(c[a] + dir[a] * k);
    }
    m->computeBounds();
}

}

// The piece's mesh turned about one axis. Only the two coordinates across the
// axis move, which is what makes the joint axis meaningful: geometry that is
// rotationally symmetric about it does not move at all.
MeshData JointRotation::rotated(const MeshData &m, int axis,
                                const double centre[3], double deg)
{
    MeshData out = m;
    if (deg == 0.0 || out.pos.isEmpty())
        return out;

    const int u = (axis + 1) % 3;
    const int v = (axis + 2) % 3;
    const double t = deg * kPi / 180.0;
    const double c = std::cos(t), s = std::sin(t);

    const int nv = int(out.pos.size() / 3);
    for (int i = 0; i < nv; ++i) {
        const double pu = double(out.pos[i * 3 + u]) - centre[u];
        const double pv = double(out.pos[i * 3 + v]) - centre[v];
        out.pos[i * 3 + u] = float(centre[u] + pu * c - pv * s);
        out.pos[i * 3 + v] = float(centre[v] + pu * s + pv * c);
    }
    out.computeBounds();
    return out;
}

JointRotation::Result JointRotation::test(const QVector<PuzzlePiece> &pieces,
                                          int                         moving,
                                          const JointPlacement       &place,
                                          const QVector<bool>        &present,
                                          const Params               &params)
{
    Result r;

    if (moving < 0 || moving >= pieces.size()) {
        r.note = QStringLiteral("no such piece");
        return r;
    }
    if (pieces[moving].mesh.triangleCount() == 0) {
        r.note = QStringLiteral("piece has no geometry to turn");
        return r;
    }

    // Scale the clearance to the model, not to the piece: a fixed millimetre
    // figure means something different on a 5 mm part and a 600 mm one.
    double diag = 0.0;
    {
        float lo[3] = {  1e30f,  1e30f,  1e30f };
        float hi[3] = { -1e30f, -1e30f, -1e30f };
        for (const PuzzlePiece &p : pieces)
            for (int a = 0; a < 3; ++a) {
                lo[a] = qMin(lo[a], p.mesh.bmin[a]);
                hi[a] = qMax(hi[a], p.mesh.bmax[a]);
            }
        for (int a = 0; a < 3; ++a) {
            const double e = double(hi[a]) - double(lo[a]);
            diag += e * e;
        }
        diag = std::sqrt(qMax(0.0, diag));
    }
    const double eps = qMax(1e-9, diag * params.clearanceFrac);

    MeshData base = pieces[moving].mesh;
    shrink(&base, eps);

    const int samples = qMax(2, params.samples);
    const double step = params.angleDeg / double(samples);

    r.maxAngleDeg = 0.0;
    r.clear       = true;

    for (int s = 1; s <= samples; ++s) {
        const double deg = step * double(s);
        const MeshData turned = rotated(base, place.axis, place.at, deg);

        bool hit = false;
        for (int other = 0; other < pieces.size() && !hit; ++other) {
            if (other == moving)
                continue;
            if (other < present.size() && !present[other])
                continue;                       // not placed yet: not in the way
            const MeshData &om = pieces[other].mesh;
            if (om.triangleCount() == 0)
                continue;

            if (!boxesOverlap(turned.bmin, turned.bmax, om.bmin, om.bmax, eps)) {
                ++r.rejected;
                continue;
            }
            ++r.tested;

            for (int ta = 0; ta + 2 < turned.tris.size() && !hit; ta += 3) {
                const V3 a0 = vertexOf(turned, turned.tris[ta + 0]);
                const V3 a1 = vertexOf(turned, turned.tris[ta + 1]);
                const V3 a2 = vertexOf(turned, turned.tris[ta + 2]);

                for (int tb = 0; tb + 2 < om.tris.size(); tb += 3) {
                    const V3 b0 = vertexOf(om, om.tris[tb + 0]);
                    const V3 b1 = vertexOf(om, om.tris[tb + 1]);
                    const V3 b2 = vertexOf(om, om.tris[tb + 2]);

                    if (triTriOverlap(a0, a1, a2, b0, b1, b2, eps)) {
                        hit = true;
                        break;
                    }
                }
            }

            if (hit) {
                r.blockedBy    = other;
                r.blockedAtDeg = deg;
            }
        }

        if (hit) {
            r.clear = false;
            break;
        }
        r.maxAngleDeg = deg;
    }

    if (r.clear)
        r.note = QStringLiteral("turns the full %1 deg with no contact")
                     .arg(params.angleDeg, 0, 'f', 1);
    else
        r.note = QStringLiteral("stopped at %1 deg of %2 by piece %3")
                     .arg(r.blockedAtDeg, 0, 'f', 1)
                     .arg(params.angleDeg, 0, 'f', 1)
                     .arg(r.blockedBy);
    return r;
}

QVector<JointRotation::Result> JointRotation::testAll(
        const QVector<PuzzlePiece>              &pieces,
        const QVector<int>                      &order,
        const QVector<QVector<JointPlacement> > &places,
        const Params                            &params)
{
    QVector<Result> out;
    QVector<bool> present(pieces.size(), false);

    for (int k = 0; k < order.size(); ++k) {
        const int piece = order[k];
        if (piece < 0 || piece >= pieces.size())
            continue;

        // Test against what is already seated, then add this piece.
        if (piece < places.size() && !places[piece].isEmpty()) {
            Result r = test(pieces, piece, places[piece].first(), present, params);
            out.append(r);
        }
        else {
            Result r;
            r.clear = true;
            r.note  = QStringLiteral("no joint on this piece");
            out.append(r);
        }
        present[piece] = true;
    }
    return out;
}

QStringList JointRotation::describe(const QVector<Result> &results,
                                    const QVector<int>    &order)
{
    QStringList out;

    int blocked = 0;
    double worst = 360.0;
    for (const Result &r : results)
        if (!r.clear) {
            ++blocked;
            worst = qMin(worst, r.maxAngleDeg);
        }

    int tested = 0, rejected = 0;
    for (const Result &r : results) { tested += r.tested; rejected += r.rejected; }

    out << QStringLiteral("ROTATE   %1 of %2 joint(s) can seat; tested on the "
                          "trimmed geometry, not bounding boxes")
               .arg(results.size() - blocked).arg(results.size());

    // Without these two numbers a vacuous run looks like a clean one: if no
    // triangle test ever ran, "can seat" only means the bounding boxes never
    // came near each other.
    out << QStringLiteral("         %1 triangle test(s) run, %2 neighbour(s) "
                          "dismissed on bounding box alone")
               .arg(tested).arg(rejected);
    if (tested == 0)
        out << QStringLiteral("         WARNING: no triangle test ran - every "
                              "neighbour was rejected on its bounding box, so "
                              "this result proves nothing about the geometry");

    if (blocked > 0)
        out << QStringLiteral("         worst case turns only %1 deg before "
                              "contact").arg(worst, 0, 'f', 1);

    for (int i = 0; i < results.size(); ++i) {
        const int piece = (i < order.size()) ? order[i] : i;
        out << QStringLiteral("         piece %1: %2")
                   .arg(piece, 3).arg(results[i].note);
    }

    out << QStringLiteral("         a bayonet needs the sweeping part to be "
                          "rotationally symmetric about the joint axis - a "
                          "square boss cannot turn in a snug pocket at any angle");
    return out;
}
