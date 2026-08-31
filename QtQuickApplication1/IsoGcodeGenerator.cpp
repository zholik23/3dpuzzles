// ====================================================================
// GCodeGenerator.cpp  —  ISOPARAMETRIC SLICER
//   * Hausdorff-validated Kåsa–Späth arc fitting
//   * Multi-wall iso shells via parameter-offset grouping
// ====================================================================
//
// Public API (unchanged — matches main.cpp's three call sites):
//
//   GenerateDirectGCode(srf, renderer, out, printSolidRoof);
//   GenerateDirectGCodeMultiSurface(surfaces, renderer, out, printSolidRoof);
//   GenerateDirectGCodeWithSupport(modelSurfaces, supportMesh,
//                                  renderer, out, printSolidRoof);
//
// What this produces:
//   A single-bead-thick conformal SHELL on each input surface, optionally
//   N parallel walls thick, with arcs fitted to horizontal iso curves
//   and 3D G1 moves elsewhere.
//
// What this does NOT produce:
//   Infill, top caps, floor caps. These are PLANAR concepts. For a
//   complete solid part, combine this iso shell with a planar slicer
//   for infill/caps (see comments at the bottom of this file).
//
// IRIT API used:
//   IritCagdSrfDomain, IritCagdCrvFromSrf, IritCagdCrv2Polyline,
//   IritCagdCrvFree, IritCagdPolylineFree, IritCagdSrfBBox.
//   Strip the "Irit" prefix on any function the linker can't find.
//
// ====================================================================

#include "GCodeGenerator.h"
#include <QFile>
#include <QTextStream>
#include <QDebug>
#include <QElapsedTimer>
#include <QMap>
#include <QHash>
#include <QVector3D>
#include <math.h>
#include <vector>
#include <algorithm>
#include <numeric>
#include <clipper\Clipper2-main\CPP\Clipper2Lib\include\clipper2/clipper.h>
using namespace Clipper2Lib;  // <--- ADD THIS
// ====================================================================
// CONFIGURATION
// ====================================================================
namespace IsoConfig {

    // --- SLICING DIRECTION ---
    constexpr CagdSrfDirType SLICE_DIRECTION = CAGD_CONST_V_DIR;

    // --- ISO-CURVE COUNT / SAMPLING ---
    // Number of PRIMARY iso "tracks" per surface. Each track will then
    // be expanded into NUM_WALLS parallel iso curves.
    constexpr int    NUM_PRIMARY_ISO_TRACKS = 250;
    constexpr int    SAMPLES_PER_CURVE = 500;

    // --- WALLS ---
    // Each PRIMARY track becomes NUM_WALLS adjacent iso curves printed
    // consecutively, so the surface is built up NUM_WALLS beads thick.
    // For NUM_WALLS = 1 you get the original single-bead shell.
    constexpr int    NUM_WALLS = 1;
    // Parameter offset between walls within one track, expressed as a
    // FRACTION of the per-track spacing. 0.0–1.0; smaller = tighter.
    // E.g. with 250 tracks and a U domain of [0,1], the per-track
    // spacing is 1/250 = 0.004. A WALL_PARAM_FRACTION of 0.2 makes the
    // wall offset 0.2 * 0.004 = 0.0008 in parameter space.
    constexpr double WALL_PARAM_FRACTION = 0.25;

    // --- 3D SIMPLIFICATION ---
    constexpr bool   ENABLE_3D_SIMPLIFY = true;
    constexpr double RDP_EPSILON_3D = 0.005;
    constexpr double MIN_3D_POINT_DIST = 0.01;

    // --- KÅSA–SPÄTH + HAUSDORFF ARC FITTING ---
    constexpr bool   ENABLE_ARC_FITTING = true;
    constexpr double PLANARITY_Z_TOLERANCE = 0.005;
    constexpr double HAUSDORFF_EPSILON = 0.010;  // mm
    constexpr int    HAUSDORFF_ARC_SAMPLES = 32;     // samples along fitted arc
    constexpr double MIN_ARC_RADIUS = 0.5;
    constexpr double MAX_ARC_RADIUS = 50000.0;
    constexpr double MIN_ARC_CHORD = 0.4;
    constexpr int    MIN_POINTS_FOR_ARC = 6;
    constexpr double MIN_ARC_TO_CHORD_RATIO = 1.0001;

    // --- EXTRUSION ---
    constexpr double FILAMENT_DIAMETER = 1.75;
    constexpr double EXTRUSION_WIDTH = 0.42;
    //constexpr double FALLBACK_LAYER_HEIGHT = 0.2;
    constexpr bool   USE_LOCAL_LAYER_HEIGHT = true;

    // --- BED ---
    constexpr double BED_CENTER_X = 100.0;
    constexpr double BED_CENTER_Y = 100.0;

    // --- SPEEDS (mm/min) ---
    constexpr double PRINT_FEED = 2400.0;
    constexpr double TRAVEL_FEED = 9000.0;
    constexpr double Z_LIFT_FEED = 1200.0;

    // --- RETRACT / TRAVEL ---
    constexpr double RETRACT_LENGTH = 0.6;
    constexpr double RETRACT_FEED = 2400.0;
    constexpr double Z_HOP_HEIGHT = 0.4;
    constexpr double RETRACT_THRESHOLD = 1.5;

    // --- PATH ORDERING ---
    constexpr bool   ENABLE_ZIGZAG = true;

    // --- SUPPORTS (still planar) ---
    constexpr double SUPPORT_LAYER_HEIGHT = 0.2;
    constexpr double SUPPORT_PRINT_FEED = 3600.0;
    constexpr double SUPPORT_FLOW_FACTOR = 0.85;

    // --- CUSTOM INFILL (no Clipper2) ---
    // Density 0.0 disables infill. 0.20 means ~20% infill — hatching
    // line spacing = EXTRUSION_WIDTH / INFILL_DENSITY (e.g. 0.42/0.20 = 2.1 mm).
    constexpr double INFILL_DENSITY = 0.08;
    // For multi-wall (offset) shells, increase NUM_WALLS above. Each
    // additional wall is an extra iso curve at WALL_PARAM_FRACTION
    // parameter offset — no Clipper2 polygon offset needed.
    // --- CLIPPER2 SETTINGS (NEW) ---
    constexpr int    CLIPPER_INNER_WALLS = 2;  // Number of offset inner walls
    //constexpr double INFILL_DENSITY = 0.20;
    constexpr double INFILL_SPACING = 5.0;
    constexpr double LAYER_HEIGHT_MM = 0.2;          // the "true" print layer height
    constexpr double FALLBACK_LAYER_HEIGHT = LAYER_HEIGHT_MM;  // keep existing alias

}

// ====================================================================
// SUPPORT MESH IMPLEMENTATION
// ====================================================================
void SupportMesh::addTriangle(float x0, float y0, float z0,
    float x1, float y1, float z1,
    float x2, float y2, float z2)
{
    SupportTriangle tri;
    tri.v0[0] = x0; tri.v0[1] = y0; tri.v0[2] = z0;
    tri.v1[0] = x1; tri.v1[1] = y1; tri.v1[2] = z1;
    tri.v2[0] = x2; tri.v2[1] = y2; tri.v2[2] = z2;
    tri.minZ = std::min({ z0, z1, z2 });
    tri.maxZ = std::max({ z0, z1, z2 });
    if (tri.minZ < globalMinZ) globalMinZ = tri.minZ;
    if (tri.maxZ > globalMaxZ) globalMaxZ = tri.maxZ;
    triangles.append(tri);
}

void SupportMesh::buildIndex() {
    const int n = triangles.size();
    sortedByMinZ.resize(n);
    std::iota(sortedByMinZ.begin(), sortedByMinZ.end(), 0);
    std::sort(sortedByMinZ.begin(), sortedByMinZ.end(),
        [this](int a, int b) { return triangles[a].minZ < triangles[b].minZ; });
    qDebug() << "   [SupportMesh] Built Z-index for" << n << "triangles"
        << "Z:" << globalMinZ << "to" << globalMaxZ;
}

// ====================================================================
// PLANAR SUPPORT SLICER (used only for the support prelude)
// ====================================================================
struct SliceSegment { float x1, y1, x2, y2; };

static QList<QList<QPointF>> sliceSupportMeshAtZ(
    const SupportMesh& mesh, double sliceZ,
    double modelCX, double modelCY, double bedCX, double bedCY)
{
    QList<QList<QPointF>> result;
    if (mesh.isEmpty()) return result;

    const float z = static_cast<float>(sliceZ);
    const auto& tris = mesh.triangles;
    const auto& idx = mesh.sortedByMinZ;

    int lo = 0, hi = idx.size();
    while (lo < hi) {
        int mid = (lo + hi) / 2;
        if (tris[idx[mid]].minZ > z) hi = mid; else lo = mid + 1;
    }

    QList<SliceSegment> segments;
    segments.reserve(lo / 4);

    for (int ii = 0; ii < lo; ++ii) {
        const SupportTriangle& tri = tris[idx[ii]];
        if (tri.maxZ < z) continue;
        float pts[4]; int ptCount = 0;
        const float* verts[3] = { tri.v0, tri.v1, tri.v2 };
        for (int e = 0; e < 3 && ptCount < 4; ++e) {
            const float* va = verts[e];
            const float* vb = verts[(e + 1) % 3];
            if ((va[2] < z && vb[2] >= z) || (vb[2] < z && va[2] >= z)) {
                float dz = vb[2] - va[2];
                if (std::abs(dz) < 1e-10f) continue;
                float t = (z - va[2]) / dz;
                pts[ptCount * 2 + 0] = va[0] + t * (vb[0] - va[0]);
                pts[ptCount * 2 + 1] = va[1] + t * (vb[1] - va[1]);
                ptCount++;
            }
        }
        if (ptCount == 2) {
            SliceSegment seg;
            seg.x1 = pts[0] - modelCX + bedCX;
            seg.y1 = pts[1] - modelCY + bedCY;
            seg.x2 = pts[2] - modelCX + bedCX;
            seg.y2 = pts[3] - modelCY + bedCY;
            segments.append(seg);
        }
    }
    if (segments.isEmpty()) return result;

    const double snapTol = 0.01;
    const double invTol = 1.0 / snapTol;
    auto snapKey = [invTol](float x, float y) -> int64_t {
        int64_t ix = static_cast<int64_t>(floor(x * invTol));
        int64_t iy = static_cast<int64_t>(floor(y * invTol));
        return ix * 73856093LL ^ iy * 19349669LL;
        };

    struct EndRef { int segIdx; bool isEnd2; };
    QHash<int64_t, QList<EndRef>> adjacency;
    for (int i = 0; i < segments.size(); ++i) {
        adjacency[snapKey(segments[i].x1, segments[i].y1)].append({ i, false });
        adjacency[snapKey(segments[i].x2, segments[i].y2)].append({ i, true });
    }

    QVector<bool> used(segments.size(), false);
    for (int startSeg = 0; startSeg < segments.size(); ++startSeg) {
        if (used[startSeg]) continue;
        used[startSeg] = true;
        const auto& s0 = segments[startSeg];
        QList<QPointF> chain;
        chain.append(QPointF(s0.x1, s0.y1));
        chain.append(QPointF(s0.x2, s0.y2));

        bool extended = true;
        while (extended) {
            extended = false;
            float tx = chain.last().x(), ty = chain.last().y();
            auto it = adjacency.find(snapKey(tx, ty));
            if (it != adjacency.end()) {
                for (const auto& ref : it.value()) {
                    if (used[ref.segIdx]) continue;
                    const auto& ns = segments[ref.segIdx];
                    float nx = ref.isEnd2 ? ns.x1 : ns.x2;
                    float ny = ref.isEnd2 ? ns.y1 : ns.y2;
                    used[ref.segIdx] = true;
                    chain.append(QPointF(nx, ny));
                    extended = true;
                    break;
                }
            }
            if (extended) continue;
            float hx = chain.first().x(), hy = chain.first().y();
            it = adjacency.find(snapKey(hx, hy));
            if (it != adjacency.end()) {
                for (const auto& ref : it.value()) {
                    if (used[ref.segIdx]) continue;
                    const auto& ns = segments[ref.segIdx];
                    float nx = ref.isEnd2 ? ns.x1 : ns.x2;
                    float ny = ref.isEnd2 ? ns.y1 : ns.y2;
                    used[ref.segIdx] = true;
                    chain.prepend(QPointF(nx, ny));
                    extended = true;
                    break;
                }
            }
        }
        if (chain.size() >= 3) result.append(chain);
    }
    return result;
}

// ====================================================================
// G-CODE TEMPLATE READER
// ====================================================================
static QMap<double, QString> globalZBuffer;

QString readGCodeTemplate(const QString& filePath) {
    QString templateContent;
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qDebug() << "   WARNING: Could not open template file:" << filePath;
        if (filePath.contains("start"))
            return "; --- Start G-Code Missing ---\nG21\nG90\nM83\n";
        else
            return "; --- End G-Code Missing ---\nM400\n";
    }
    else {
        QTextStream in(&file);
        templateContent = in.readAll();
    }

    if (filePath.contains("end")) {
        QString assembled;
        QTextStream s(&assembled);
        for (auto it = globalZBuffer.begin(); it != globalZBuffer.end(); ++it) {
            s << "; --- CAD Z: " << it.key() << " ---\n";
            s << "G1 Z" << it.key() << " F1200\n";
            s << it.value();
        }
        globalZBuffer.clear();
        return assembled + templateContent;
    }
    return templateContent;
}

// ====================================================================
// 3D / 2D GEOMETRY HELPERS
// ====================================================================
static inline double dist3D(const QVector3D& a, const QVector3D& b) {
    double dx = a.x() - b.x();
    double dy = a.y() - b.y();
    double dz = a.z() - b.z();
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

static inline double dist2D(double ax, double ay, double bx, double by) {
    double dx = ax - bx, dy = ay - by;
    return std::sqrt(dx * dx + dy * dy);
}

static double pointToLine3D(const QVector3D& A, const QVector3D& B, const QVector3D& C)
{
    QVector3D ab = B - A;
    QVector3D ac = C - A;
    double abLen2 = QVector3D::dotProduct(ab, ab);
    if (abLen2 < 1e-12) return ac.length();
    QVector3D cross = QVector3D::crossProduct(ab, ac);
    return cross.length() / std::sqrt(abLen2);
}

static QList<QVector3D> rdp3D(const QList<QVector3D>& pts, double eps)
{
    const int n = pts.size();
    if (n < 3) return pts;
    QVector<bool> keep(n, false);
    keep[0] = keep[n - 1] = true;

    std::vector<std::pair<int, int>> stack;
    stack.push_back({ 0, n - 1 });
    while (!stack.empty()) {
        auto [s, e] = stack.back(); stack.pop_back();
        if (e <= s + 1) continue;
        double maxD = 0.0; int maxI = -1;
        for (int i = s + 1; i < e; ++i) {
            double d = pointToLine3D(pts[s], pts[e], pts[i]);
            if (d > maxD) { maxD = d; maxI = i; }
        }
        if (maxD > eps && maxI > 0) {
            keep[maxI] = true;
            stack.push_back({ s, maxI });
            stack.push_back({ maxI, e });
        }
    }
    QList<QVector3D> out; out.reserve(n);
    for (int i = 0; i < n; ++i) if (keep[i]) out.append(pts[i]);
    return out;
}

static QList<QVector3D> dropDuplicates3D(const QList<QVector3D>& pts, double minDist)
{
    QList<QVector3D> out;
    if (pts.isEmpty()) return out;
    out.append(pts.first());
    for (int i = 1; i < pts.size(); ++i) {
        if (dist3D(out.last(), pts[i]) >= minDist) out.append(pts[i]);
    }
    return out;
}

// ====================================================================
// KÅSA–SPÄTH ALGEBRAIC CIRCLE FIT
// ====================================================================
struct CircleFit {
    bool   valid;
    double cx, cy;
    double radius;
};

static CircleFit fitCircleKasaSpath(const QList<QVector3D>& pts2D,
    int startIdx, int endIdx)
{
    CircleFit r; r.valid = false;
    const int n = endIdx - startIdx + 1;
    if (n < 3) return r;

    double Sx = 0, Sy = 0, Sxx = 0, Syy = 0, Sxy = 0;
    double Sxxx = 0, Syyy = 0, Sxyy = 0, Syxx = 0;
    for (int i = startIdx; i <= endIdx; ++i) {
        double x = pts2D[i].x(), y = pts2D[i].y();
        Sx += x; Sy += y;
        Sxx += x * x; Syy += y * y; Sxy += x * y;
        Sxxx += x * x * x; Syyy += y * y * y;
        Sxyy += x * y * y; Syxx += y * x * x;
    }
    double N = double(n);
    double m[3][3] = { {Sxx, Sxy, Sx}, {Sxy, Syy, Sy}, {Sx, Sy, N} };
    double rhs[3] = { -(Sxxx + Sxyy), -(Syxx + Syyy), -(Sxx + Syy) };

    double det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    if (std::abs(det) < 1e-20) return r;

    double inv[3][3];
    inv[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) / det;
    inv[0][1] = -(m[0][1] * m[2][2] - m[0][2] * m[2][1]) / det;
    inv[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) / det;
    inv[1][0] = -(m[1][0] * m[2][2] - m[1][2] * m[2][0]) / det;
    inv[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) / det;
    inv[1][2] = -(m[0][0] * m[1][2] - m[0][2] * m[1][0]) / det;
    inv[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) / det;
    inv[2][1] = -(m[0][0] * m[2][1] - m[0][1] * m[2][0]) / det;
    inv[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) / det;

    double D = inv[0][0] * rhs[0] + inv[0][1] * rhs[1] + inv[0][2] * rhs[2];
    double E = inv[1][0] * rhs[0] + inv[1][1] * rhs[1] + inv[1][2] * rhs[2];
    double F = inv[2][0] * rhs[0] + inv[2][1] * rhs[1] + inv[2][2] * rhs[2];

    double cx = -D / 2.0, cy = -E / 2.0;
    double rad23 = cx * cx + cy * cy - F;
    if (rad23 <= 0.0) return r;

    r.valid = true;
    r.cx = cx; r.cy = cy; r.radius = std::sqrt(rad23);
    return r;
}

// ====================================================================
// HAUSDORFF DISTANCE — input polyline vs. fitted arc
// ====================================================================
//
//   H(P, A) = max( sup_p∈P inf_a∈A d(p,a) ,  sup_a∈A inf_p∈P d(a,p) )
//
// We need BOTH directions:
//   d1 = max over input points of distance to the fitted arc
//        (catches points that don't lie on the circle)
//   d2 = max over arc samples of distance to nearest input point
//        (catches the fitted arc bulging into empty space between
//         input points — pure max-deviation can't see this)
//
// Cost: O(N * M) where N = input points, M = arc samples. With our
// HAUSDORFF_ARC_SAMPLES = 32 and N < 500, that's ~16k ops per
// candidate fit. Fast enough.
// ====================================================================

// Signed angle from center to point (atan2, range [-pi, pi]).
static inline double angleOf(double px, double py, double cx, double cy) {
    return std::atan2(py - cy, px - cx);
}

// Test whether angle `a` lies on the arc going from a0 to a1 in the
// specified direction. All angles in radians.
static bool angleOnArc(double a, double a0, double a1, bool clockwise) {
    // Normalise to [0, 2pi)
    auto norm = [](double x) {
        const double TWO_PI = 2.0 * M_PI;
        while (x < 0.0)     x += TWO_PI;
        while (x >= TWO_PI) x -= TWO_PI;
        return x;
        };
    double na = norm(a);
    double na0 = norm(a0);
    double na1 = norm(a1);

    if (clockwise) {
        // Going from a0 down to a1 (decreasing).
        double sweep = norm(na0 - na1);
        double pos = norm(na0 - na);
        return pos <= sweep + 1e-12;
    }
    else {
        // Going from a0 up to a1 (increasing).
        double sweep = norm(na1 - na0);
        double pos = norm(na - na0);
        return pos <= sweep + 1e-12;
    }
}

// Distance from point (px,py) to the closest point of the fitted arc.
// If the foot of the perpendicular falls inside the arc, it's
// |dist_to_center - R|; otherwise it's distance to the nearer endpoint.
static double pointToArcDistance(double px, double py,
    double cx, double cy, double R,
    double a0, double a1, bool clockwise,
    double sx, double sy, double ex, double ey)
{
    double angleP = angleOf(px, py, cx, cy);
    if (angleOnArc(angleP, a0, a1, clockwise)) {
        double r = std::sqrt((px - cx) * (px - cx) + (py - cy) * (py - cy));
        return std::abs(r - R);
    }
    else {
        double dS = std::hypot(px - sx, py - sy);
        double dE = std::hypot(px - ex, py - ey);
        return std::min(dS, dE);
    }
}

// Bidirectional Hausdorff distance between the input polyline
// segment pts[startIdx..endIdx] and the fitted arc.
static double hausdorffPolylineArc(const QList<QVector3D>& pts,
    int startIdx, int endIdx,
    double cx, double cy, double R,
    bool clockwise, int arcSamples)
{
    // Arc endpoints and angles.
    double sx = pts[startIdx].x(), sy = pts[startIdx].y();
    double ex = pts[endIdx].x(), ey = pts[endIdx].y();
    double a0 = angleOf(sx, sy, cx, cy);
    double a1 = angleOf(ex, ey, cx, cy);

    // ---- d1: max input-point-to-arc distance ----
    double d1 = 0.0;
    for (int i = startIdx; i <= endIdx; ++i) {
        double d = pointToArcDistance(pts[i].x(), pts[i].y(),
            cx, cy, R, a0, a1, clockwise,
            sx, sy, ex, ey);
        if (d > d1) d1 = d;
    }

    // ---- d2: max arc-sample-to-nearest-input-point distance ----
    // Walk the arc from a0 to a1 in the specified direction.
    double sweep;
    if (clockwise) {
        sweep = a0 - a1;
        if (sweep < 0) sweep += 2.0 * M_PI;
        sweep = -sweep;             // negative => decreasing angle
    }
    else {
        sweep = a1 - a0;
        if (sweep < 0) sweep += 2.0 * M_PI;
    }

    double d2 = 0.0;
    for (int k = 0; k <= arcSamples; ++k) {
        double t = double(k) / double(arcSamples);
        double angle = a0 + t * sweep;
        double qx = cx + R * std::cos(angle);
        double qy = cy + R * std::sin(angle);

        double minD = std::numeric_limits<double>::max();
        for (int i = startIdx; i <= endIdx; ++i) {
            double d = std::hypot(qx - pts[i].x(), qy - pts[i].y());
            if (d < minD) minD = d;
        }
        if (minD > d2) d2 = minD;
    }

    return std::max(d1, d2);
}

// ====================================================================
// ISO-CURVE TYPES
// ====================================================================
struct IsoPath {
    QList<QVector3D> points;
    double  param;
    int     surfaceIdx;
    int     trackIdx;      // which "primary track" this path belongs to
    int     wallIdx;       // 0..NUM_WALLS-1 within the track
    double  avgZ;
    double  zVariance;
    bool    isHorizontal;
    double  planeZ;
};

// ====================================================================
// EXTRACT ISO-CURVES FROM ONE SURFACE
// ====================================================================
// For each PRIMARY track we extract NUM_WALLS adjacent iso curves at
// parameter offsets, so the surface is built up multiple beads thick.
// ====================================================================
static QVector<IsoPath> extractIsoPathsFromSurface(
    CagdSrfStruct* srf, int surfaceIdx,
    int numTracks, int samplesPerCurve,
    int numWalls, double wallParamFraction,
    CagdSrfDirType dir,
    double modelCX, double modelCY,
    double bedCX, double bedCY,
    double zOffset)
{
    QVector<IsoPath> result;
    if (!srf || numTracks <= 0 || numWalls <= 0) return result;

    CagdRType uMin = 0, uMax = 0, vMin = 0, vMax = 0;
    IritCagdSrfDomain(srf, &uMin, &uMax, &vMin, &vMax);

    CagdRType tMin = (dir == CAGD_CONST_U_DIR) ? uMin : vMin;
    CagdRType tMax = (dir == CAGD_CONST_U_DIR) ? uMax : vMax;
    if (std::abs(tMax - tMin) < 1e-9) {
        qDebug() << "   [Iso] Degenerate domain on surface" << surfaceIdx;
        return result;
    }
    const double inset = (tMax - tMin) * 1e-5;
    tMin += inset;
    tMax -= inset;

    // Per-track spacing in parameter space.
    double perTrack = (numTracks == 1) ? 0.0 : (tMax - tMin) / double(numTracks - 1);
    double wallParStep = perTrack * wallParamFraction;

    result.reserve(numTracks * numWalls);

    for (int t = 0; t < numTracks; ++t) {
        double trackFrac = (numTracks == 1) ? 0.5
            : double(t) / double(numTracks - 1);
        double trackParam = tMin + (tMax - tMin) * trackFrac;

        for (int w = 0; w < numWalls; ++w) {
            CagdRType tParam = trackParam + double(w) * wallParStep;
            // Don't run off the domain.
            if (tParam > tMax) tParam = tMax;
            if (tParam < tMin) tParam = tMin;

            CagdCrvStruct* iso = IritCagdCrvFromSrf(srf, tParam, dir);
            if (!iso) continue;

            CagdPolylineStruct* poly = IritCagdCrv2Polyline(iso, samplesPerCurve, FALSE);
            IritCagdCrvFree(iso);
            if (!poly) continue;

            IsoPath path;
            path.param = tParam;
            path.surfaceIdx = surfaceIdx;
            path.trackIdx = t;
            path.wallIdx = w;
            path.points.reserve(poly->Length);

            double zSum = 0.0, zMin = 1e18, zMax = -1e18;
            for (int k = 0; k < poly->Length; ++k) {
                double x = poly->Polyline[k].Pt[0];
                double y = poly->Polyline[k].Pt[1];
                double z = poly->Polyline[k].Pt[2];
                x = x - modelCX + bedCX;
                y = y - modelCY + bedCY;
                z = z + zOffset;
                path.points.append(QVector3D(static_cast<float>(x),
                    static_cast<float>(y),
                    static_cast<float>(z)));
                zSum += z;
                if (z < zMin) zMin = z;
                if (z > zMax) zMax = z;
            }
            IritCagdPolylineFree(poly);

            if (path.points.size() < 2) continue;
            path.avgZ = zSum / path.points.size();
            path.zVariance = zMax - zMin;
            path.isHorizontal = (path.zVariance < IsoConfig::PLANARITY_Z_TOLERANCE);
            path.planeZ = path.avgZ;

            path.points = dropDuplicates3D(path.points, IsoConfig::MIN_3D_POINT_DIST);
            if (IsoConfig::ENABLE_3D_SIMPLIFY && path.points.size() > 3) {
                path.points = rdp3D(path.points, IsoConfig::RDP_EPSILON_3D);
            }
            if (path.points.size() >= 2) result.append(path);
        }
    }

    return result;
}

// ====================================================================
// PATH ORDERING — group walls together within tracks; zigzag tracks
// ====================================================================
// Walls of the same (surface, track) MUST print consecutively so the
// printer lays down all walls of one region before moving to the next.
// We sort tracks by avgZ-of-primary-wall, then within each track keep
// wall order, and apply zigzag between tracks.
// ====================================================================
static void orderPathsForPrinting(QVector<IsoPath>& paths)
{
    if (paths.size() < 2) return;

    // Group by (surfaceIdx, trackIdx).
    struct TrackGroup {
        QVector<IsoPath> walls;     // ordered by wallIdx (0..NUM_WALLS-1)
        double avgZ;
    };
    QHash<qint64, TrackGroup> groups;
    auto key = [](int s, int t) -> qint64 {
        return (qint64(s) << 32) | qint64(uint(t));
        };

    for (const auto& p : paths) {
        qint64 k = key(p.surfaceIdx, p.trackIdx);
        groups[k].walls.append(p);
    }
    for (auto it = groups.begin(); it != groups.end(); ++it) {
        // Sort walls within track by wallIdx.
        std::sort(it.value().walls.begin(), it.value().walls.end(),
            [](const IsoPath& a, const IsoPath& b) {
                return a.wallIdx < b.wallIdx;
            });
        // avgZ of the group = avgZ of the first wall (the primary curve).
        it.value().avgZ = it.value().walls.first().avgZ;
    }

    // Order groups by avgZ.
    QVector<TrackGroup> ordered;
    ordered.reserve(groups.size());
    for (auto it = groups.begin(); it != groups.end(); ++it)
        ordered.append(it.value());
    std::sort(ordered.begin(), ordered.end(),
        [](const TrackGroup& a, const TrackGroup& b) {
            return a.avgZ < b.avgZ;
        });

    // Zigzag between groups: greedy nearest-neighbour on group endpoints.
    if (IsoConfig::ENABLE_ZIGZAG && ordered.size() > 1) {
        QVector<TrackGroup> rerouted;
        rerouted.reserve(ordered.size());
        rerouted.append(ordered.first());
        ordered.removeFirst();

        while (!ordered.isEmpty()) {
            const QVector3D& cursor =
                rerouted.last().walls.last().points.last();
            int bestIdx = -1;
            bool bestReverse = false;
            double bestDist = std::numeric_limits<double>::max();
            double currentZ = ordered.first().avgZ;


            for (int i = 0; i < ordered.size(); ++i) {
                if (std::abs(ordered[i].avgZ - currentZ) > 0.1) break;

                const auto& firstW = ordered[i].walls.first();

                const auto& lastW = ordered[i].walls.last();
                double dFront = dist3D(cursor, firstW.points.first());
                double dBack = dist3D(cursor, firstW.points.last());

                if (dFront < bestDist) { bestDist = dFront; bestIdx = i; bestReverse = false; }
                if (dBack < bestDist) { bestDist = dBack;  bestIdx = i; bestReverse = true; }
            }

            if (bestIdx < 0) break;
            TrackGroup g = ordered[bestIdx];
            ordered.removeAt(bestIdx);
            if (bestReverse) {
                // Reverse both the wall order and each wall's points.
                //std::reverse(g.walls.begin(), g.walls.end());
                for (auto& w : g.walls)
                    std::reverse(w.points.begin(), w.points.end());

            }
            rerouted.append(g);
        }
        ordered = rerouted;
    }

    // Flatten back into paths.
    paths.clear();
    for (const auto& g : ordered)
        for (const auto& w : g.walls)
            paths.append(w);
}

// ====================================================================
// LOCAL LAYER HEIGHT
// ====================================================================
static inline double nearestDistanceToPath(
    const QVector3D& p, const QList<QVector3D>& prev)
{
    if (prev.isEmpty()) return IsoConfig::FALLBACK_LAYER_HEIGHT;
    double best = std::numeric_limits<double>::max();
    for (const auto& q : prev) {
        double d = dist3D(p, q);
        if (d < best) best = d;
    }
    return (best > 1e-6) ? best : IsoConfig::FALLBACK_LAYER_HEIGHT;
}

// ====================================================================
// G-CODE EMISSION
// ====================================================================
struct EmitStats {
    int    totalG1 = 0;
    int    totalG2G3 = 0;
    int    totalTravel = 0;
    int    pointsIn = 0;
    int    pointsOut = 0;
    int    arcsFitted = 0;
    int    arcsRejectedByHausdorff = 0;
    double totalPrintTimeSec = 0.0;
    double totalPathLen = 0.0;
};

static inline double filamentArea() {
    return M_PI * std::pow(IsoConfig::FILAMENT_DIAMETER * 0.5, 2.0);
}

static inline double eForSegment(double segLen, double localLh) {
    double bead = IsoConfig::EXTRUSION_WIDTH * localLh;
    return segLen * bead / filamentArea();
}

// Try to fit ONE arc to points [startIdx..endIdx] of a horizontal iso-curve.
// Uses Kåsa-Späth for the algebraic fit and BIDIRECTIONAL HAUSDORFF for
// the acceptance test.
static bool tryFitHorizontalArc(const QList<QVector3D>& pts,
    int startIdx, int endIdx,
    double& outCx, double& outCy,
    double& outR, bool& outClockwise,
    EmitStats& stats)
{
    int n = endIdx - startIdx + 1;
    if (n < IsoConfig::MIN_POINTS_FOR_ARC) return false;

    // --- Kåsa-Späth algebraic fit ---
    CircleFit cf = fitCircleKasaSpath(pts, startIdx, endIdx);
    if (!cf.valid) return false;
    if (cf.radius < IsoConfig::MIN_ARC_RADIUS)  return false;
    if (cf.radius > IsoConfig::MAX_ARC_RADIUS)  return false;

    // --- Chord & angle checks ---
    double chord = dist2D(pts[startIdx].x(), pts[startIdx].y(),
        pts[endIdx].x(), pts[endIdx].y());
    if (chord < IsoConfig::MIN_ARC_CHORD) return false;

    double halfC = chord * 0.5;
    if (halfC >= cf.radius) return false;
    double arcLen = 2.0 * cf.radius * std::asin(halfC / cf.radius);
    if (arcLen / chord < IsoConfig::MIN_ARC_TO_CHORD_RATIO) return false;

    // --- Direction (CW vs CCW) ---
    int midIdx = (startIdx + endIdx) / 2;
    double v1x = pts[midIdx].x() - pts[startIdx].x();
    double v1y = pts[midIdx].y() - pts[startIdx].y();
    double v2x = pts[endIdx].x() - pts[startIdx].x();
    double v2y = pts[endIdx].y() - pts[startIdx].y();
    bool clockwise = ((v1x * v2y - v1y * v2x) < 0.0);

    // --- HAUSDORFF acceptance test ---
    double H = hausdorffPolylineArc(pts, startIdx, endIdx,
        cf.cx, cf.cy, cf.radius,
        clockwise, IsoConfig::HAUSDORFF_ARC_SAMPLES);
    if (H > IsoConfig::HAUSDORFF_EPSILON) {
        stats.arcsRejectedByHausdorff++;
        return false;
    }

    outCx = cf.cx; outCy = cf.cy; outR = cf.radius;
    outClockwise = clockwise;
    return true;
}

static void emitHorizontalCurveArcs(const QList<QVector3D>& pts,
    double localLh,
    QTextStream& out, EmitStats& s,
    QVector3D& cursor)
{
    int n = pts.size();
    if (n < 2) return;


    // ====================================================================
    // FULL CIRCLE FAST-PATH (Bypasses the zero-chord bug)
    // ====================================================================
    bool isClosed = (dist2D(pts.first().x(), pts.first().y(), pts.last().x(), pts.last().y()) < 0.05);

    if (isClosed && n >= IsoConfig::MIN_POINTS_FOR_ARC) {
        CircleFit cf = fitCircleKasaSpath(pts, 0, n - 1);
        if (cf.valid && cf.radius >= IsoConfig::MIN_ARC_RADIUS && cf.radius <= IsoConfig::MAX_ARC_RADIUS) {

            // Check Hausdorff max deviation for the full circle
            double maxErr = 0;
            for (int k = 0; k < n; ++k) {
                double r = std::hypot(pts[k].x() - cf.cx, pts[k].y() - cf.cy);
                double err = std::abs(r - cf.radius);
                if (err > maxErr) maxErr = err;
            }

            if (maxErr <= IsoConfig::HAUSDORFF_EPSILON) {
                // IT IS A PERFECT FULL CIRCLE! 
                // Determine direction using polygon Shoelace formula
                double area = 0;
                for (int k = 0; k < n - 1; ++k) {
                    area += (pts[k].x() * pts[k + 1].y() - pts[k + 1].x() * pts[k].y());
                }
                bool clockwise = (area < 0);
                QString cmd = clockwise ? "G2" : "G3";

                double I = cf.cx - pts.first().x();
                double J = cf.cy - pts.first().y();
                double midX = cf.cx + I; // Opposite side of the circle
                double midY = cf.cy + J;

                double halfCircumference = M_PI * cf.radius;
                double eHalf = eForSegment(halfCircumference, localLh);
                double circumference = 2.0 * M_PI * cf.radius;
                double eFull = eForSegment(circumference, localLh);
                out << cmd << " X" << pts.first().x() << " Y" << pts.first().y() << " Z" << pts.first().z()
                    << " I" << I << " J" << J << " E" << eFull << " F" << IsoConfig::PRINT_FEED << "\n";
                // Emit as two 180-degree arcs (Safest for all 3D printer firmwares)
               /* out << cmd << " X" << midX << " Y" << midY << " Z" << pts.first().z()
                    << " I" << I << " J" << J << " E" << eHalf << " F" << IsoConfig::PRINT_FEED << "\n";

                out << cmd << " X" << pts.last().x() << " Y" << pts.last().y() << " Z" << pts.last().z()
                    << " I" << -I << " J" << -J << " E" << eHalf << " F" << IsoConfig::PRINT_FEED << "\n";

                s.totalG2G3 += 2; s.arcsFitted += 2; s.pointsOut += 2;
                s.totalPathLen += 2.0 * halfCircumference;
                cursor = pts.last();*/
                s.totalG2G3 += 1; s.arcsFitted += 1; s.pointsOut += 1; // Only 1 command!
                s.totalPathLen += circumference;
                cursor = pts.first();
                return; // Layer completely finished in 2 lines of code!
            }
        }
    }
    // ====================================================================




    int i = 0;
    while (i < n - 1) {
        int best = -1;
        double bestCx = 0, bestCy = 0, bestR = 0; bool bestCW = false;

        int j = std::min(i + IsoConfig::MIN_POINTS_FOR_ARC - 1, n - 1);
        while (j < n) {
            double cx, cy, r; bool cw;
            if (tryFitHorizontalArc(pts, i, j, cx, cy, r, cw, s)) {
                best = j;
                bestCx = cx; bestCy = cy; bestR = r; bestCW = cw;
                j++;
            }
            else {
                break;
            }
        }

        if (best > i) {
            const QVector3D& start = pts[i];
            const QVector3D& end = pts[best];
            double I = bestCx - start.x();
            double J = bestCy - start.y();
            double halfC = 0.5 * dist2D(start.x(), start.y(), end.x(), end.y());
            double arcLen = (halfC < bestR)
                ? 2.0 * bestR * std::asin(halfC / bestR)
                : dist2D(start.x(), start.y(), end.x(), end.y());
            double e = eForSegment(arcLen, localLh);
            out << (bestCW ? "G2" : "G3")
                << " X" << end.x() << " Y" << end.y() << " Z" << end.z()
                << " I" << I << " J" << J
                << " E" << e << " F" << IsoConfig::PRINT_FEED << "\n";
            s.totalG2G3++; s.arcsFitted++; s.pointsOut++;
            s.totalPathLen += arcLen;
            if (IsoConfig::PRINT_FEED > 0)
                s.totalPrintTimeSec += arcLen / (IsoConfig::PRINT_FEED / 60.0);
            cursor = end;
            i = best;
        }
        else {
            const QVector3D& p = pts[i + 1];
            double segLen = dist3D(cursor, p);
            if (segLen > 1e-6) {
                double e = eForSegment(segLen, localLh);
                out << "G1 X" << p.x() << " Y" << p.y() << " Z" << p.z()
                    << " E" << e << " F" << IsoConfig::PRINT_FEED << "\n";
                s.totalG1++; s.pointsOut++;
                s.totalPathLen += segLen;
                if (IsoConfig::PRINT_FEED > 0)
                    s.totalPrintTimeSec += segLen / (IsoConfig::PRINT_FEED / 60.0);
                cursor = p;
            }
            i++;
        }
    }
}

static void emit3DCurve(const QList<QVector3D>& pts, double localLh,
    QTextStream& out, EmitStats& s, QVector3D& cursor)
{
    for (int k = 1; k < pts.size(); ++k) {
        const QVector3D& p = pts[k];
        double segLen = dist3D(cursor, p);
        if (segLen < 1e-6) continue;
        double e = eForSegment(segLen, localLh);
        out << "G1 X" << p.x() << " Y" << p.y() << " Z" << p.z()
            << " E" << e << " F" << IsoConfig::PRINT_FEED << "\n";
        s.totalG1++; s.pointsOut++;
        s.totalPathLen += segLen;
        if (IsoConfig::PRINT_FEED > 0)
            s.totalPrintTimeSec += segLen / (IsoConfig::PRINT_FEED / 60.0);
        cursor = p;
    }
}

static void emitIsoGCode(const QVector<IsoPath>& paths,
    QTextStream& out, EmitStats& stats,
    IritRenderer* renderer)
{
    QVector3D cursor(-1000.0f, -1000.0f, -1000.0f);
    bool primed = false;
    bool inG17 = false;

    auto addTime = [&](double mm, double feed) {
        if (feed > 0) stats.totalPrintTimeSec += mm / (feed / 60.0);
        };

    const QList<QVector3D>* prevPathPoints = nullptr;
    int lastTrackIdx = -1, lastSurfaceIdx = -1;

    for (int pi = 0; pi < paths.size(); ++pi) {
        const IsoPath& path = paths[pi];
        if (path.points.size() < 2) continue;

        stats.pointsIn += path.points.size();
        const QVector3D& start = path.points.first();

        bool newTrack = (path.trackIdx != lastTrackIdx ||
            path.surfaceIdx != lastSurfaceIdx);
        if (newTrack) {
            out << "; --- Track surf=" << path.surfaceIdx
                << " idx=" << path.trackIdx
                << " param=" << QString::number(path.param, 'f', 5)
                << (path.isHorizontal ? " HORIZONTAL" : "")
                << " ---\n";
        }
        out << "; wall " << path.wallIdx << " of "
            << IsoConfig::NUM_WALLS << "\n";

        if (!primed) {
            out << "G0 X" << start.x() << " Y" << start.y()
                << " Z" << (start.z() + IsoConfig::Z_HOP_HEIGHT)
                << " F" << IsoConfig::TRAVEL_FEED << "\n";
            out << "G0 Z" << start.z() << " F" << IsoConfig::Z_LIFT_FEED << "\n";
            out << "G1 E0.8 F2400 ; Prime\n";
            addTime(0.8, 1800.0);
            primed = true;
            stats.totalTravel++;
        }
        else {
            double travel = dist3D(cursor, start);
            if (travel > IsoConfig::RETRACT_THRESHOLD) {
                out << "G1 E-" << IsoConfig::RETRACT_LENGTH
                    << " F" << IsoConfig::RETRACT_FEED << " ; Retract\n";
                out << "G0 Z" << (std::max(cursor.z(), start.z()) + IsoConfig::Z_HOP_HEIGHT)
                    << " F" << IsoConfig::Z_LIFT_FEED << "\n";
                out << "G0 X" << start.x() << " Y" << start.y()
                    << " F" << IsoConfig::TRAVEL_FEED << "\n";
                out << "G0 Z" << start.z()
                    << " F" << IsoConfig::Z_LIFT_FEED << "\n";
                out << "G1 E" << IsoConfig::RETRACT_LENGTH
                    << " F" << IsoConfig::RETRACT_FEED << " ; Prime\n";
                addTime(IsoConfig::RETRACT_LENGTH * 2, IsoConfig::RETRACT_FEED);
                addTime(travel, IsoConfig::TRAVEL_FEED);
            }
            else {
                out << "G0 X" << start.x() << " Y" << start.y()
                    << " Z" << start.z()
                    << " F" << IsoConfig::TRAVEL_FEED << "\n";
                addTime(travel, IsoConfig::TRAVEL_FEED);
            }
            stats.totalTravel++;
        }
        cursor = start;

        // Local layer height — for the FIRST wall of a track, measure
        // against the previous track. For subsequent walls within the
        // same track, the previous wall is closer (in mm), which is the
        // right reference for bead width.
        double localLh = IsoConfig::FALLBACK_LAYER_HEIGHT;
        if (IsoConfig::USE_LOCAL_LAYER_HEIGHT && prevPathPoints) {
            double sum = 0; int cnt = 0;
            for (const auto& p : path.points) {
                sum += nearestDistanceToPath(p, *prevPathPoints);
                cnt++;
            }
            if (cnt > 0) localLh = sum / cnt;
            if (localLh < 0.05) localLh = 0.05;
            if (localLh > 1.0)  localLh = 1.0;
        }

        if (IsoConfig::ENABLE_ARC_FITTING && path.isHorizontal) {
            if (!inG17) {
                out << "G17 ; XY arc plane\n";
                inG17 = true;
            }
            emitHorizontalCurveArcs(path.points, localLh, out, stats, cursor);
        }
        else {
            emit3DCurve(path.points, localLh, out, stats, cursor);
        }

        if (renderer) {
            for (int k = 1; k < path.points.size(); ++k) {
                const QVector3D& a = path.points[k - 1];
                const QVector3D& b = path.points[k];
                renderer->addContourPoint(a.x(), a.y(), a.z());
                renderer->addContourPoint(b.x(), b.y(), b.z());
            }
        }

        prevPathPoints = &path.points;
        lastTrackIdx = path.trackIdx;
        lastSurfaceIdx = path.surfaceIdx;
    }
}

// ====================================================================
// PLANAR SUPPORT PRELUDE
// ====================================================================
static void emitPlanarSupportPrelude(SupportMesh& supportMesh,
    double modelCX, double modelCY, double zOffset,
    QTextStream& out, EmitStats& stats)
{
    if (supportMesh.isEmpty()) return;
    supportMesh.buildIndex();

    const double minZ = supportMesh.globalMinZ;
    const double maxZ = supportMesh.globalMaxZ;
    const double lh = IsoConfig::SUPPORT_LAYER_HEIGHT;
    const double eMul = (IsoConfig::EXTRUSION_WIDTH * lh) / filamentArea()
        * IsoConfig::SUPPORT_FLOW_FACTOR;
    const double feed = IsoConfig::SUPPORT_PRINT_FEED;

    QList<double> heights;
    for (double z = minZ + 0.001; z < maxZ; z += lh) heights.append(z);
    if (heights.isEmpty()) return;

    out << "; ============================================\n";
    out << "; SUPPORT PRELUDE — " << heights.size() << " planar layers\n";
    out << "; ============================================\n";

    QVector3D last(-1000.0f, -1000.0f, -1000.0f);
    auto addTime = [&](double mm, double f) {
        if (f > 0) stats.totalPrintTimeSec += mm / (f / 60.0);
        };

    bool primed = false;
    for (double z : heights) {
        double printZ = z + zOffset;
        QList<QList<QPointF>> polys = sliceSupportMeshAtZ(
            supportMesh, z, modelCX, modelCY,
            IsoConfig::BED_CENTER_X, IsoConfig::BED_CENTER_Y);
        if (polys.isEmpty()) continue;

        out << "; --- Support layer Z=" << printZ << " ---\n";
        out << "G1 Z" << printZ << " F" << IsoConfig::Z_LIFT_FEED << "\n";

        for (const auto& poly : polys) {
            if (poly.size() < 2) continue;
            const QPointF& s = poly.first();
            QVector3D start(static_cast<float>(s.x()),
                static_cast<float>(s.y()),
                static_cast<float>(printZ));
            if (!primed) {
                out << "G0 X" << start.x() << " Y" << start.y()
                    << " F" << IsoConfig::TRAVEL_FEED << "\n";
                out << "G1 E0.8 F2400 ; Prime\n";
                primed = true;
            }
            else {
                double travel = dist3D(last, start);
                if (travel > IsoConfig::RETRACT_THRESHOLD) {
                    out << "G1 E-" << IsoConfig::RETRACT_LENGTH
                        << " F" << IsoConfig::RETRACT_FEED << "\n";
                    out << "G0 X" << start.x() << " Y" << start.y()
                        << " F" << IsoConfig::TRAVEL_FEED << "\n";
                    out << "G1 E" << IsoConfig::RETRACT_LENGTH
                        << " F" << IsoConfig::RETRACT_FEED << "\n";
                    addTime(travel, IsoConfig::TRAVEL_FEED);
                }
                else {
                    out << "G0 X" << start.x() << " Y" << start.y()
                        << " F" << IsoConfig::TRAVEL_FEED << "\n";
                    addTime(travel, IsoConfig::TRAVEL_FEED);
                }
            }
            last = start;

            for (int i = 1; i < poly.size(); ++i) {
                const QPointF& p = poly[i];
                double segLen = std::hypot(p.x() - last.x(), p.y() - last.y());
                if (segLen < 1e-5) continue;
                double e = segLen * eMul;
                out << "G1 X" << p.x() << " Y" << p.y()
                    << " E" << e
                    << " F" << feed << "\n";
                addTime(segLen, feed);
                stats.totalG1++; stats.pointsOut++;
                last = QVector3D(static_cast<float>(p.x()),
                    static_cast<float>(p.y()),
                    static_cast<float>(printZ));
            }
        }
    }

    out << "G1 Z" << (maxZ + 1.0) << " F" << IsoConfig::Z_LIFT_FEED
        << " ; lift before iso phase\n";
    out << "; --- END SUPPORT PRELUDE ---\n\n";
}

// ====================================================================
// STATISTICS
// ====================================================================
static void printIsoStatistics(QTextStream& out, const EmitStats& s,
    int numPaths, int totalPointsIn)
{
    int hours = s.totalPrintTimeSec / 3600;
    int minutes = (int(s.totalPrintTimeSec) % 3600) / 60;
    int seconds = int(s.totalPrintTimeSec) % 60;
    QString timeStr = QString("%1h %2m %3s").arg(hours)
        .arg(minutes, 2, 10, QChar('0'))
        .arg(seconds, 2, 10, QChar('0'));
    int totalCmds = s.totalG1 + s.totalG2G3;
    double compression = (totalPointsIn > 0)
        ? (1.0 - double(totalCmds) / double(totalPointsIn)) * 100.0 : 0.0;
    double arcRatio = (totalCmds > 0)
        ? (double(s.totalG2G3) / totalCmds) * 100.0 : 0.0;

    qDebug() << "   =========================================";
    qDebug() << "   ISO SLICING STATISTICS:";
    qDebug() << "   Iso curves emitted:" << numPaths
        << "(" << IsoConfig::NUM_WALLS << "walls per track)";
    qDebug() << "   Sample points (raw):" << totalPointsIn;
    qDebug() << "   G1 (line) commands:" << s.totalG1;
    qDebug() << "   G2/G3 (arc) commands:" << s.totalG2G3;
    qDebug() << "   Arcs accepted (Hausdorff):" << s.arcsFitted;
    qDebug() << "   Arcs rejected by Hausdorff:" << s.arcsRejectedByHausdorff;
    qDebug() << "   Travel transitions:" << s.totalTravel;
    qDebug() << "   Compression:" << QString::number(compression, 'f', 1) << "%";
    qDebug() << "   Arc ratio:" << QString::number(arcRatio, 'f', 1) << "%";
    qDebug() << "   Extrusion path:"
        << QString::number(s.totalPathLen, 'f', 1) << "mm";
    qDebug() << "   Estimated print time:" << timeStr;
    qDebug() << "   =========================================";

    out << "\n; --- ISO SLICING STATISTICS ---\n";
    out << "; Estimated Print Time: " << timeStr << "\n";
    out << "; Iso curves emitted: " << numPaths
        << " (" << IsoConfig::NUM_WALLS << " walls per track)\n";
    out << "; Sample points (raw): " << totalPointsIn << "\n";
    out << "; G1 commands: " << s.totalG1 << "\n";
    out << "; G2/G3 commands: " << s.totalG2G3 << "\n";
    out << "; Arcs accepted: " << s.arcsFitted << "\n";
    out << "; Arcs rejected by Hausdorff: " << s.arcsRejectedByHausdorff << "\n";
    out << "; Travel transitions: " << s.totalTravel << "\n";
    out << "; Compression: " << QString::number(compression, 'f', 1) << "%\n";
    out << "; Arc ratio: " << QString::number(arcRatio, 'f', 1) << "%\n";
    out << "; Extrusion path: " << QString::number(s.totalPathLen, 'f', 1) << " mm\n";
    out << "; ------------------------------\n";
}
// ====================================================================
// CUSTOM INFILL (no Clipper2 dependency)
// ====================================================================
//
// Algorithm — line clipping by even-odd rule:
//
//   At each Z layer, you have a set of closed iso rings. For a torus
//   the layer has 2 rings (outer + inner). For a cylinder, 1 ring.
//   For arbitrary topology, any number of nested rings.
//
//   We cast parallel "hatching" lines across the layer's bounding box
//   at a 45° or 135° angle (alternating per layer for cross-hatching).
//   For each hatching line:
//
//     1. Find every intersection of the line with every ring edge
//        across every ring in this layer.
//     2. Sort the intersection points by their parameter t along the
//        infinite line.
//     3. By the Jordan curve theorem, the parity of the crossing
//        count tells you which segments are inside vs. outside the
//        filled region. With an even number of sorted intersections
//        t0 < t1 < t2 < t3 < …, the segments
//             [t0,t1], [t2,t3], [t4,t5], …
//        lie inside the filled region. The segments
//             [t1,t2], [t3,t4], …
//        lie inside a HOLE and are skipped.
//
//   This works regardless of ring winding direction because even-odd
//   only cares about crossing count, not orientation. That's the key
//   property that lets it handle the torus hole correctly without any
//   Clipper2 Union step.
//
// Notes:
//   - Filters to wallIdx == 0 to ignore the multi-wall iso duplicates
//     so they don't create spurious "extra rings".
//   - Skips lines with odd intersection counts (corner-grazing
//     numerical edge cases — safer to drop than misinterpret).
//   - Output segments are open polylines (2 points each); the iso
//     emitter prints them as travel-then-extrude moves.
//
// ====================================================================

// Returns true and writes parameter t along the ray (ox,oy)+t*(dx,dy)
// at which it crosses the segment [(ax,ay)–(bx,by)]. Returns false if
// the line and segment are parallel or the intersection lies outside
// the segment's [0,1] parameter range.
static bool rayCrossesSegment(double ox, double oy, double dx, double dy,
    double ax, double ay, double bx, double by,
    double& tOut)
{
    double ex = bx - ax, ey = by - ay;
    double det = ex * dy - dx * ey;
    if (std::abs(det) < 1e-12) return false;   // parallel — no crossing

    double rx = ax - ox, ry = ay - oy;
    double t = (-rx * ey + ex * ry) / det;     // parameter along the ray
    double s = (dx * ry - dy * rx) / det;      // parameter along the segment

    if (s < -1e-9 || s > 1.0 + 1e-9) return false;
    tOut = t;
    return true;
}

static QVector<IsoPath> generateCustomInfill(const QVector<IsoPath>& layerPaths,
    double infillSpacing,
    double angleDeg)
{
    QVector<IsoPath> result;
    if (layerPaths.isEmpty() || infillSpacing <= 0.0) return result;

    // ---- 1. Collect closed iso rings (wallIdx == 0 only) ----
    struct Ring { QVector<QVector3D> pts; };
    QVector<Ring> rings;
    double minX = 1e9, maxX = -1e9;
    double minY = 1e9, maxY = -1e9;

    double planeZ = layerPaths.first().planeZ;
    int    surfIdx = layerPaths.first().surfaceIdx;
    int    trackIdx = layerPaths.first().trackIdx;
    double param = layerPaths.first().param;

    for (const auto& path : layerPaths) {
        if (path.wallIdx != 0) continue;       // skip multi-wall duplicates
        if (path.points.size() < 3) continue;

        Ring r;
        r.pts.reserve(path.points.size() + 1);
        for (const auto& p : path.points) {
            r.pts.append(p);
            if (p.x() < minX) minX = p.x();
            if (p.x() > maxX) maxX = p.x();
            if (p.y() < minY) minY = p.y();
            if (p.y() > maxY) maxY = p.y();
        }
        // Force closure: if last point != first point, append first.
        QVector3D a = r.pts.first(), b = r.pts.last();
        if (std::hypot(a.x() - b.x(), a.y() - b.y()) > 1e-4) {
            r.pts.append(a);
        }
        rings.append(r);
    }
    if (rings.isEmpty()) return result;

    // ---- 2. Hatching direction & extents ----
    double angleRad = angleDeg * M_PI / 180.0;
    double dx = std::cos(angleRad);
    double dy = std::sin(angleRad);
    double px = -dy;   // perpendicular: spacing direction
    double py = dx;

    double cx = (minX + maxX) * 0.5;
    double cy = (minY + maxY) * 0.5;
    double bboxRadius = std::hypot(maxX - cx, maxY - cy) + 1.0;

    // ---- 3. Cast hatching lines, find crossings, emit segments ----
    for (double d = -bboxRadius; d <= bboxRadius; d += infillSpacing) {
        // Origin of this hatching line: one point on it.
        double ox = cx + d * px;
        double oy = cy + d * py;

        // Find all ring-edge crossings with this infinite line.
        QVector<double> tCross;
        tCross.reserve(8);
        for (const auto& ring : rings) {
            for (int i = 0; i < ring.pts.size() - 1; ++i) {
                double t;
                if (rayCrossesSegment(ox, oy, dx, dy,
                    ring.pts[i].x(), ring.pts[i].y(),
                    ring.pts[i + 1].x(), ring.pts[i + 1].y(),
                    t))
                {
                    tCross.append(t);
                }
            }
        }
        if (tCross.size() < 2) continue;
        std::sort(tCross.begin(), tCross.end());

        // Dedupe values that are essentially the same (vertex hits).
        QVector<double> tClean;
        tClean.reserve(tCross.size());
        tClean.append(tCross.first());
        for (int i = 1; i < tCross.size(); ++i) {
            if (std::abs(tCross[i] - tClean.last()) > 1e-5) {
                tClean.append(tCross[i]);
            }
        }
        if (tClean.size() < 2) continue;

        // Need even count for clean inside/outside pairing. Odd count
        // means a corner-grazing intersection; skip the line.
        if (tClean.size() % 2 != 0) continue;

        // Emit one segment per pair: [t0,t1], [t2,t3], …
        for (int i = 0; i + 1 < tClean.size(); i += 2) {
            double t0 = tClean[i];
            double t1 = tClean[i + 1];
            if (std::abs(t1 - t0) < 1e-4) continue;   // too short

            QVector3D start(static_cast<float>(ox + t0 * dx),
                static_cast<float>(oy + t0 * dy),
                static_cast<float>(planeZ));
            QVector3D end(static_cast<float>(ox + t1 * dx),
                static_cast<float>(oy + t1 * dy),
                static_cast<float>(planeZ));

            IsoPath seg;
            seg.surfaceIdx = surfIdx;
            seg.trackIdx = trackIdx;
            seg.param = param;
            seg.planeZ = planeZ;
            seg.avgZ = planeZ;
            seg.zVariance = 0.0;
            seg.isHorizontal = true;
            seg.wallIdx = 99;             // sentinel for "infill"
            seg.points.append(start);
            seg.points.append(end);
            result.append(seg);
        }
    }

    return result;
}

// ====================================================================
// VOLUMETRIC BOUNDARY EXTRACTOR (Using GCodeGenerator.cpp IRIT API)
// ====================================================================
static QVector<IsoPath> extractPlanarContoursAtZ(
    const QList<CagdSrfStruct*>& surfaces,
    double cadZ,
    double modelCX, double modelCY,
    double bedCX, double bedCY,
    double zOffset)
{
    QVector<IsoPath> boundaries;
    IrtPlnType plane = { 0.0, 0.0, 1.0, -cadZ };

    // Use the same resolution parameter type as your original slicer
    double slicingResolution = IsoConfig::SAMPLES_PER_CURVE;

    for (int i = 0; i < surfaces.size(); ++i) {
        CagdSrfStruct* srf = surfaces[i];
        if (!srf) continue;

        // Exact call from GCodeGenerator.cpp
        IritPrsrPolygonStruct* polys = IritUserCntrSrfWithPlane(srf, plane, slicingResolution, 0, 1);
        if (!polys) continue;

        // Traverse the polygon linked list
        for (IritPrsrPolygonStruct* pl = polys; pl != nullptr; pl = pl->Pnext) {
            IsoPath boundary;
            boundary.surfaceIdx = i;
            boundary.planeZ = cadZ + zOffset;
            boundary.isHorizontal = true;
            boundary.trackIdx = 0;
            boundary.param = 0.0;

            // CRITICAL: Set wallIdx to 0 so generateCustomInfill accepts it as a boundary!
            boundary.wallIdx = 0;

            // Traverse the vertex linked list
            for (IritPrsrVertexStruct* v = pl->PVertex; v != nullptr; v = v->Pnext) {
                double wx = v->Coord[0] - modelCX + bedCX;
                double wy = v->Coord[1] - modelCY + bedCY;
                boundary.points.append(QVector3D(static_cast<float>(wx),
                    static_cast<float>(wy),
                    static_cast<float>(cadZ + zOffset)));
            }

            // Ensure the polygon loop is perfectly closed for the Even-Odd raycaster
            if (boundary.points.size() > 0) {
                double d = std::hypot(boundary.points.first().x() - boundary.points.last().x(),
                    boundary.points.first().y() - boundary.points.last().y());
                if (d > 1e-4) {
                    boundary.points.append(boundary.points.first());
                }
            }

            if (boundary.points.size() >= 3) {
                boundaries.append(boundary);
            }
        }
        // Free memory exactly as done in GCodeGenerator.cpp
        IritPrsrFreePolygonList(polys);
    }
    return boundaries;
}

// ====================================================================
// HYBRID ENGINE: IRIT Planar Extraction + Clipper2 Offsets & Infill
// ====================================================================
static QVector<IsoPath> processPlanarLayerWithClipper(
    const QList<CagdSrfStruct*>& surfaces,
    double cadZ, int unifiedTrackIdx,
    double modelCX, double modelCY,
    double bedCX, double bedCY, double zOffset)
{
    QVector<IsoPath> newPaths;
    const double SCALE = 1000000.0;
    Paths64 layerBoundaries;
    double minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9;

    IrtPlnType plane = { 0.0, 0.0, 1.0, -cadZ };
    double slicingResolution = IsoConfig::SAMPLES_PER_CURVE;

    // 1. EXTRACT TRUE PLANAR BOUNDARIES FROM IRIT
    for (int i = 0; i < surfaces.size(); ++i) {
        if (!surfaces[i]) continue;
        IritPrsrPolygonStruct* polys = IritUserCntrSrfWithPlane(surfaces[i], plane, slicingResolution, 0, 1);
        if (!polys) continue;

        for (IritPrsrPolygonStruct* pl = polys; pl != nullptr; pl = pl->Pnext) {
            Path64 clipperPath;
            for (IritPrsrVertexStruct* v = pl->PVertex; v != nullptr; v = v->Pnext) {
                double wx = v->Coord[0] - modelCX + bedCX;
                double wy = v->Coord[1] - modelCY + bedCY;
                clipperPath.push_back(Point64(wx * SCALE, wy * SCALE));

                if (wx < minX) minX = wx; if (wx > maxX) maxX = wx;
                if (wy < minY) minY = wy; if (wy > maxY) maxY = wy;
            }
            if (clipperPath.size() >= 3) {
                layerBoundaries.push_back(clipperPath);
            }
        }
        IritPrsrFreePolygonList(polys);
    }

    if (layerBoundaries.empty()) return newPaths;

    Paths64 currentBoundary = layerBoundaries;
    Paths64 innermostBoundary = layerBoundaries;
    double printZ = cadZ + zOffset;

    // 2. CLIPPER2 OFFSET WALLS
    ClipperOffset offsetter;
    double offsetDist = -IsoConfig::EXTRUSION_WIDTH * SCALE;

    for (int w = 0; w < IsoConfig::CLIPPER_INNER_WALLS; ++w) {
        offsetter.Clear();
        offsetter.AddPaths(currentBoundary, JoinType::Miter, EndType::Polygon);
        Paths64 innerWall;
        offsetter.Execute(offsetDist, innerWall);

        if (innerWall.empty()) break;

        for (const auto& poly : innerWall) {
            IsoPath wp;
            wp.surfaceIdx = 0;
            wp.trackIdx = unifiedTrackIdx; // Lock to the group
            wp.param = 0.0;
            wp.planeZ = printZ;
            wp.avgZ = printZ;
            wp.zVariance = 0.0;
            wp.isHorizontal = true;
            wp.wallIdx = w + 1; // 1, 2, etc. (0 is the pure Isoparametric Skin)

            for (const auto& pt : poly) {
                wp.points.append(QVector3D(pt.x / SCALE, pt.y / SCALE, printZ));
            }
            if (!wp.points.isEmpty()) wp.points.append(wp.points.first()); // Close loop
            newPaths.append(wp);
        }
        currentBoundary = innerWall;
        innermostBoundary = innerWall;
    }

    // 3. CLIPPER2 ORTHOGONAL GRID INFILL (Ported from GCodeGenerator.cpp)
    if (IsoConfig::INFILL_SPACING > 0.0 && !innermostBoundary.empty()) {
        Paths64 grid;
        double inf = IsoConfig::INFILL_SPACING; // Millimeter spacing

        // Convert to Clipper's integer scale
        double imnX = minX, imxX = maxX, imnY = minY, imxY = maxY;

        // Draw horizontal lines across the bounding box
        for (double y = imnY; y <= imxY; y += inf) {
            Path64 h;
            h.push_back(Point64(imnX * SCALE, y * SCALE));
            h.push_back(Point64(imxX * SCALE, y * SCALE));
            grid.push_back(h);
        }

        // Draw vertical lines across the bounding box
        for (double x = imnX; x <= imxX; x += inf) {
            Path64 v;
            v.push_back(Point64(x * SCALE, imnY * SCALE));
            v.push_back(Point64(x * SCALE, imxY * SCALE));
            grid.push_back(v);
        }

        // Use Clipper to intersect the square grid with the donut hole/boundaries
        Clipper64 clipper;
        clipper.AddOpenSubject(grid);
        clipper.AddClip(innermostBoundary);
        Paths64 closedRes, openRes;
        clipper.Execute(ClipType::Intersection, FillRule::EvenOdd, closedRes, openRes);

        // Convert the trimmed lines back to IsoPaths for the G-code emitter
        int infillWallIdx = 99;
        for (const auto& line : openRes) {
            if (line.size() < 2) continue;
            IsoPath infillPath;
            infillPath.surfaceIdx = 0;
            infillPath.trackIdx = unifiedTrackIdx;
            infillPath.param = 0.0;
            infillPath.planeZ = printZ;
            infillPath.avgZ = printZ;
            infillPath.zVariance = 0.0;
            infillPath.isHorizontal = true;
            infillPath.wallIdx = infillWallIdx++;

            for (const auto& pt : line) {
                infillPath.points.append(QVector3D(pt.x / SCALE, pt.y / SCALE, printZ));
            }
            newPaths.append(infillPath);
        }
    }

    return newPaths;
}
// ====================================================================
// NEAREST-NEIGHBOUR CHAINING for infill segments
// ====================================================================
//
// Clipper2 / our hand-rolled even-odd infill emits segments in scan
// order (one hatching line after another), which means consecutive
// segments are often on opposite sides of the part. The printer ends
// up doing 5–10 mm travel jumps between every pair of segments, which
// either (a) bloats the file with retract+Z-hop ceremony, or (b)
// strings plastic across empty space if those gaps are skipped.
//
// This pass reorders segments and reverses individual ones so that
// each segment's end is close to the next segment's start. Most
// inter-segment gaps then collapse to ~0.2 mm (the hatching offset),
// which is below any sensible retract threshold — so retracts vanish
// naturally AND the print is clean.
//
// Uses a uniform grid for O(N) average-case neighbour lookups instead
// of the O(N²) you'd get from naive linear search.
// ====================================================================
static QVector<IsoPath> chainInfillSegmentsNN(const QVector<IsoPath>& segments)
{
    QVector<IsoPath> result;
    const int n = segments.size();
    if (n <= 1) return segments;

    // ---- 1. Build a uniform grid of segment endpoints. ----
    //    Each segment contributes two endpoint refs: (segIdx, isEnd).
    //    Cell size ~ 2x typical hatch spacing keeps avg neighbours small.
    double minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9;
    for (const auto& s : segments) {
        if (s.points.size() < 2) continue;
        const auto& a = s.points.first();
        const auto& b = s.points.last();
        minX = std::min({ minX, double(a.x()), double(b.x()) });
        maxX = std::max({ maxX, double(a.x()), double(b.x()) });
        minY = std::min({ minY, double(a.y()), double(b.y()) });
        maxY = std::max({ maxY, double(a.y()), double(b.y()) });
    }
    const double cellSize = 8.0;  // mm
    auto cellOf = [&](double x, double y) -> qint64 {
        int cx = int(std::floor((x - minX) / cellSize));
        int cy = int(std::floor((y - minY) / cellSize));
        return (qint64(cx) << 32) | qint64(uint(cy));
        };

    struct EndRef { int segIdx; bool isEnd; };
    QHash<qint64, QList<EndRef>> grid;
    for (int i = 0; i < n; ++i) {
        if (segments[i].points.size() < 2) continue;
        grid[cellOf(segments[i].points.first().x(), segments[i].points.first().y())]
            .append({ i, false });
        grid[cellOf(segments[i].points.last().x(), segments[i].points.last().y())]
            .append({ i, true });
    }

    QVector<bool> used(n, false);
    result.reserve(n);

    // ---- 2. Seed: segment whose start is closest to bed bottom-left. ----
    int seedIdx = -1;
    double seedDist = std::numeric_limits<double>::max();
    for (int i = 0; i < n; ++i) {
        if (segments[i].points.size() < 2) continue;
        const auto& a = segments[i].points.first();
        double d = std::hypot(a.x() - minX, a.y() - minY);
        if (d < seedDist) { seedDist = d; seedIdx = i; }
    }
    if (seedIdx < 0) return segments;

    result.append(segments[seedIdx]);
    used[seedIdx] = true;

    // ---- 3. Greedy chain. ----
    auto current = [&]() -> QVector3D { return result.last().points.last(); };

    while (true) {
        QVector3D cursor = current();
        int bestSeg = -1;
        bool bestReverse = false;
        double bestDist = std::numeric_limits<double>::max();

        // Search the cell containing `cursor` and the 8 around it
        // (3×3 = 9 cells). Anything outside this neighbourhood is
        // farther than cellSize, so it can't beat a hit closer than
        // cellSize. If we find no hit, expand to 5×5, then fall back
        // to linear (rare).
        for (int expand = 1; expand <= 3 && bestSeg < 0; ++expand) {
            int cx0 = int(std::floor((cursor.x() - minX) / cellSize));
            int cy0 = int(std::floor((cursor.y() - minY) / cellSize));
            for (int dx = -expand; dx <= expand; ++dx) {
                for (int dy = -expand; dy <= expand; ++dy) {
                    qint64 key = (qint64(cx0 + dx) << 32) | qint64(uint(cy0 + dy));
                    auto it = grid.find(key);
                    if (it == grid.end()) continue;
                    for (const auto& ref : it.value()) {
                        if (used[ref.segIdx]) continue;
                        const auto& s = segments[ref.segIdx];
                        const auto& ep = ref.isEnd ? s.points.last() : s.points.first();
                        double d = std::hypot(ep.x() - cursor.x(), ep.y() - cursor.y());
                        if (d < bestDist) {
                            bestDist = d;
                            bestSeg = ref.segIdx;
                            // If we landed on the "end" endpoint, the
                            // segment needs to be reversed so its start
                            // is at the near side.
                            bestReverse = ref.isEnd;
                        }
                    }
                }
            }
        }

        // Last-resort fallback: linear scan. Only happens when remaining
        // segments are isolated (e.g., on a separate island in the layer).
        if (bestSeg < 0) {
            for (int i = 0; i < n; ++i) {
                if (used[i]) continue;
                if (segments[i].points.size() < 2) continue;
                const auto& a = segments[i].points.first();
                const auto& b = segments[i].points.last();
                double dA = std::hypot(a.x() - cursor.x(), a.y() - cursor.y());
                double dB = std::hypot(b.x() - cursor.x(), b.y() - cursor.y());
                if (dA < bestDist) { bestDist = dA; bestSeg = i; bestReverse = false; }
                if (dB < bestDist) { bestDist = dB; bestSeg = i; bestReverse = true; }
            }
        }
        if (bestSeg < 0) break;  // all used

        IsoPath next = segments[bestSeg];
        if (bestReverse) std::reverse(next.points.begin(), next.points.end());
        result.append(next);
        used[bestSeg] = true;
    }

    return result;
}

// ====================================================================
// CORE PIPELINE
// ====================================================================
static void runIsoPipeline(const QList<CagdSrfStruct*>& surfaces,
    SupportMesh* supportMesh,
    IritRenderer* renderer,
    QTextStream& out,
    bool generateInfill)
{
    QElapsedTimer timer; timer.start();
    out.setRealNumberNotation(QTextStream::FixedNotation);
    out.setRealNumberPrecision(5);

    double gMinX = 1e9, gMaxX = -1e9, gMinY = 1e9, gMaxY = -1e9, gMinZ = 1e9, gMaxZ = -1e9;
    for (auto* s : surfaces) {
        if (!s) continue;
        CagdBBoxStruct B; IritCagdSrfBBox(s, &B);
        if (B.Min[0] < gMinX) gMinX = B.Min[0];
        if (B.Max[0] > gMaxX) gMaxX = B.Max[0];
        if (B.Min[1] < gMinY) gMinY = B.Min[1];
        if (B.Max[1] > gMaxY) gMaxY = B.Max[1];
        if (B.Min[2] < gMinZ) gMinZ = B.Min[2];
        if (B.Max[2] > gMaxZ) gMaxZ = B.Max[2];
    }
    double modelCX = (gMinX + gMaxX) * 0.5;
    double modelCY = (gMinY + gMaxY) * 0.5;
    double zOffset = -gMinZ;
    qDebug() << "   [Iso] Model centre:" << modelCX << modelCY
        << " Z range:" << gMinZ << "to" << gMaxZ;
    qDebug() << "   [Iso] Walls per track:" << IsoConfig::NUM_WALLS
        << " | Tracks per surface:" << IsoConfig::NUM_PRIMARY_ISO_TRACKS;

    EmitStats stats;

    if (supportMesh && !supportMesh->isEmpty()) {
        qDebug() << "   [Iso] Emitting planar support prelude ("
            << supportMesh->count() << "triangles)";
        emitPlanarSupportPrelude(*supportMesh, modelCX, modelCY, zOffset, out, stats);
    }

    QElapsedTimer t1; t1.start();
    QVector<IsoPath> allPaths;
    int totalPointsIn = 0;
    for (int i = 0; i < surfaces.size(); ++i) {
        CagdSrfStruct* srf = surfaces[i];
        if (!srf) continue;
        QVector<IsoPath> paths = extractIsoPathsFromSurface(
            srf, i,
            IsoConfig::NUM_PRIMARY_ISO_TRACKS,
            IsoConfig::SAMPLES_PER_CURVE,
            IsoConfig::NUM_WALLS,
            IsoConfig::WALL_PARAM_FRACTION,
            IsoConfig::SLICE_DIRECTION,
            modelCX, modelCY,
            IsoConfig::BED_CENTER_X, IsoConfig::BED_CENTER_Y, zOffset);
        for (const auto& p : paths) totalPointsIn += p.points.size();
        allPaths.append(paths);
        if ((i + 1) % 10 == 0 || i == surfaces.size() - 1) {
            qDebug() << "   [Iso] Surface" << (i + 1) << "/" << surfaces.size()
                << " paths so far:" << allPaths.size()
                << "(" << t1.elapsed() << "ms)";
        }
    }
    qDebug() << "   [Iso] Extraction done in" << t1.elapsed() << "ms,"
        << allPaths.size() << "iso paths total.";
    // --- HYBRID PASS: Z-binned planar walls + chained infill ---
    if (generateInfill && IsoConfig::INFILL_SPACING > 0.0) {
        qDebug() << "   [Iso] Hybrid Pass: Z-binned planar boundaries + Clipper2 + NN chaining";

        QVector<IsoPath> finalPaths;
        finalPaths.reserve(allPaths.size() * 2);

        // ----------------------------------------------------------
        // STEP 1. Bin every horizontal iso path into TRUE LAYERS.
        // ----------------------------------------------------------
        // Many iso curves can land within one real layer (especially
        // near the poles of a torus where iso curves bunch up). We
        // index them by floor(printZ / LAYER_HEIGHT) so all curves
        // within one real layer share a bucket.
        const double LH = IsoConfig::LAYER_HEIGHT_MM;
        QMap<int, QVector<IsoPath>> realLayers;          // layerIdx -> iso paths
        QMap<int, double>           layerPrintZ;         // layerIdx -> representative Z

        for (auto& path : allPaths) {
            if (!path.isHorizontal) {
                finalPaths.append(path);                 // 3D iso curves pass through
                continue;
            }
            // Snap microscopic float noise; bin to real layer index.
            int layerIdx = int(std::round(path.planeZ / LH));
            double snappedZ = layerIdx * LH;
            path.planeZ = snappedZ;
            path.avgZ = snappedZ;
            for (auto& pt : path.points) {
                pt.setZ(static_cast<float>(snappedZ));
            }
            realLayers[layerIdx].append(path);
            if (!layerPrintZ.contains(layerIdx))
                layerPrintZ[layerIdx] = snappedZ;
        }

        qDebug() << "   [Iso] Iso curves binned into" << realLayers.size()
            << "real layers (was" << allPaths.size() << "raw paths)";

        // ----------------------------------------------------------
        // STEP 2. For each REAL layer, do walls + infill EXACTLY ONCE.
        // ----------------------------------------------------------
        int wallsGenerated = 0, infillSegsGenerated = 0;

        for (auto it = realLayers.begin(); it != realLayers.end(); ++it) {
            int    layerIdx = it.key();
            double printZ = it.value().first().planeZ;
            double cadZ = printZ - zOffset;
            int    unifiedTrackIdx = layerIdx + 1000000;

            // (a) Lock all iso skin curves for this layer into one print group.
            for (auto& p : it.value()) {
                p.trackIdx = unifiedTrackIdx;
                finalPaths.append(p);
            }

            // (b) Generate planar walls + infill ONCE for this real layer.
            //     Cross-hatch direction alternates per layer.
            QVector<IsoPath> clipperPaths = processPlanarLayerWithClipper(
                surfaces, cadZ, unifiedTrackIdx,
                modelCX, modelCY,
                IsoConfig::BED_CENTER_X, IsoConfig::BED_CENTER_Y,
                zOffset);

            // (c) Split inner walls from infill so we can chain just the infill.
            QVector<IsoPath> wallsOnly, infillOnly;
            for (const auto& p : clipperPaths) {
                if (p.wallIdx >= 99) infillOnly.append(p);
                else                 wallsOnly.append(p);
            }

            // Walls print in their natural concentric order (Clipper emits
            // outermost-first, which is correct).
            finalPaths.append(wallsOnly);
            wallsGenerated += wallsOnly.size();

            // Infill gets nearest-neighbour chained so consecutive segments
            // are physically close — kills the retract ceremony naturally.
            if (!infillOnly.isEmpty()) {
                QVector<IsoPath> chained = chainInfillSegmentsNN(infillOnly);
                finalPaths.append(chained);
                infillSegsGenerated += chained.size();
            }
        }

        qDebug() << "   [Iso] Hybrid pass produced:"
            << wallsGenerated << "wall paths,"
            << infillSegsGenerated << "infill segments (chained)";

        allPaths = finalPaths;
    }
    else {
        qDebug() << "   [Iso] Infill SKIPPED (printSolidRoof = false)";
    }
// ----------------------------------------------------
    if (allPaths.isEmpty()) {
        qDebug() << "   [Iso] WARNING: no iso paths extracted!";
        return;
    }

    QElapsedTimer t2; t2.start();
    orderPathsForPrinting(allPaths);
    qDebug() << "   [Iso] Path ordering in" << t2.elapsed() << "ms";

    QElapsedTimer t3; t3.start();
    emitIsoGCode(allPaths, out, stats, renderer);
    qDebug() << "   [Iso] G-code emission in" << t3.elapsed() << "ms";

    printIsoStatistics(out, stats, allPaths.size(), totalPointsIn);
    qDebug() << "   [Iso] Total iso slicing:" << timer.elapsed() << "ms";
}

// ====================================================================
// PUBLIC API
// ====================================================================
void GenerateDirectGCode(CagdSrfStruct* srf, IritRenderer* renderer,
    QTextStream& out, bool printSolidRoof)
{
    qDebug() << "   [Iso] Single-surface isoparametric slicing"
        << "| infill =" << (printSolidRoof ? "ON" : "OFF");
    QList<CagdSrfStruct*> list;
    list.append(srf);
    runIsoPipeline(list, nullptr, renderer, out, printSolidRoof);
}

void GenerateDirectGCodeMultiSurface(
    const QList<CagdSrfStruct*>& surfaces,
    IritRenderer* renderer, QTextStream& out, bool printSolidRoof)
{
    qDebug() << "   [Iso] Multi-surface isoparametric slicing,"
        << surfaces.size() << "surfaces"
        << "| infill =" << (printSolidRoof ? "ON" : "OFF");
    if (surfaces.isEmpty()) return;
    runIsoPipeline(surfaces, nullptr, renderer, out, printSolidRoof);
}

void GenerateDirectGCodeWithSupport(
    const QList<CagdSrfStruct*>& modelSurfaces,
    SupportMesh& supportMesh,
    IritRenderer* renderer, QTextStream& out, bool printSolidRoof)
{
    qDebug() << "   [Iso] Iso slicing with planar support prelude:"
        << modelSurfaces.size() << "surfaces,"
        << supportMesh.count() << "support triangles"
        << "| infill =" << (printSolidRoof ? "ON" : "OFF");
    runIsoPipeline(modelSurfaces, &supportMesh, renderer, out, printSolidRoof);
}

// ====================================================================
// ROADMAP TO INFILL / FLOOR / ROOF
// ====================================================================
// To turn this into a complete slicer for solid parts, the cleanest path
// is a HYBRID architecture:
//
// 1. Keep the existing PLANAR slicer (your previous GCodeGenerator.cpp)
//    but configure it to skip the OUTERMOST wall. Its output becomes:
//      - inner walls (NUM_PERIMETERS - 1 of them)
//      - infill at each Z
//      - solid floor at the bottom N layers
//      - solid roof at the top N layers
//
// 2. Run the planar slicer first to lay down the interior.
//
// 3. Then run THIS iso slicer to wrap the outer surface. The iso shell
//    becomes the visible outermost wall with conformal quality.
//
// Both pipelines write to the same QTextStream. The planar layers are
// already sorted by Z via globalZBuffer; the iso paths come after and
// are also bottom-up ordered, so the printer builds bottom planar
// layers, then bottom iso paths, etc.
//
// Modifications required:
//   - Planar slicer: add a NUM_OUTER_WALLS_TO_SKIP config (set to 1).
//     Currently it always emits perimeters from outermost inward;
//     it would skip the outermost pass.
//   - This iso slicer: nothing — it already handles its own pipeline.
//   - main.cpp: call the planar function, then call this iso function.
// ====================================================================
