
#include "GCodeGenerator.h"
#include <QFile>
#include <QTextStream>
#include <QStandardPaths>
#include <QDebug>
#include <clipper\Clipper2-main\CPP\Clipper2Lib\include\clipper2/clipper.h>
#include <math.h>
#include <vector>
#include <algorithm>
#include <QMap>
#include <QHash>
#include <QElapsedTimer>
#include <QThread>
#include <QMutex>
#include <QtConcurrent>
#include <QFuture>
#include <QFutureSynchronizer>
#include <functional>
#include <numeric>

// ====================================================================
// CONFIGURABLE PARAMETERS FOR PATH OPTIMIZATION
// ====================================================================
namespace GCodeConfig {
    constexpr bool ENABLE_PATH_OPTIMIZATION = true;
    constexpr bool ENABLE_ARC_FITTING = true;
    // LINE OPTIMIZATION
    constexpr double LINE_COLLINEARITY_EPSILON = 0.015;
    constexpr double MIN_POINT_DISTANCE = 0.05;

    // ARC FITTING PARAMETERS
    // IRIT contour points have ~0.03-0.05mm positional noise from tessellation.
    // Tolerances must be ABOVE this noise floor or every arc fails validation.
    constexpr double ARC_FITTING_EPSILON = 0.06;      // Max point-to-circle deviation (mm)
    constexpr double MIN_ARC_RADIUS = 2.0;             // Reject very tight arcs
    constexpr double MAX_ARC_RADIUS = 3000.0;          // Reject near-straight arcs
    constexpr double MIN_CHORD_LENGTH = 0.5;            // No tiny arcs
    constexpr double MAX_ARC_ANGLE = M_PI * 0.667;     // Max 120° per arc
    constexpr double MIN_ARC_ANGLE = 0.03;              // Reject near-zero arcs

    // --- ARC VALIDATION ---
    constexpr double MIN_ARC_TO_CHORD_RATIO = 1.001;   // Must be noticeably curved
    constexpr double MIN_ARC_PHYSICAL_LENGTH = 1.0;     // Minimum 1mm arc
    constexpr int    MIN_POINTS_IN_ARC = 5;             // 5+ points for reliable fit
    constexpr double ARC_MIDPOINT_TOLERANCE = 0.15;     // Midpoint check (mm) — above IRIT noise

    constexpr int    MAX_POINTS_TO_CHECK_ARC = 150;

    // CORNER PROTECTION
    constexpr double MIN_CORNER_ANGLE_RAD = 0.5;

    // INFILL PARAMETERS
    constexpr double INFILL_SPACING = 2.5;
    constexpr double MIN_INFILL_SEGMENT_LENGTH = 1.0;
    constexpr double RETRACT_THRESHOLD = 1.5;

    // SOLID SURFACE QUALITY
    constexpr int    NUM_SOLID_FLOOR_LAYERS = 3;
    constexpr int    NUM_SOLID_ROOF_LAYERS = 3;
    constexpr double SOLID_FILL_LINE_SPACING_FACTOR = 0.88;
    constexpr double SOLID_FILL_BOUNDARY_OVERLAP = 0.35;

    // PARALLEL SLICING TUNING
    // How many layers per parallel batch. Smaller = more parallelism overhead.
    // Larger = fewer threads active if layer count isn't divisible.
    // 0 = auto (use hardware_concurrency).
    constexpr int    PARALLEL_BATCH_SIZE = 0;

    // SUPPORT-SPECIFIC PARAMETERS
    constexpr int    SUPPORT_PERIMETERS = 1;           // 1 wall (easy removal)
    constexpr double SUPPORT_INFILL_SPACING = 4.0;     // Sparse lines (mm)
    constexpr double SUPPORT_FLOW_MULTIPLIER = 0.85;   // 85% flow (weaker bond)
    constexpr double SUPPORT_PRINT_SPEED = 3600.0;     // mm/min (faster)
    constexpr int    SUPPORT_AIR_GAP_LAYERS = 2;       // Gap between support top & model
    constexpr double SUPPORT_INTERFACE_SPACING = 0.8;   // Dense interface layer (mm)
}

// ====================================================================
// GEOMETRIC HELPER STRUCTURES
// ====================================================================
struct ArcData {
    bool isArc;
    double centerX, centerY;
    double radius;
    bool isClockwise;
    double arcLength;
    double startAngle, endAngle;
};

struct OptimizedSegment {
    enum Type { LINE, ARC_CW, ARC_CCW };
    Type type;
    QPointF start;
    QPointF end;
    double centerX, centerY;
    double radius;
    double arcLength;
};

// Helper struct to hold all contour polygons for a layer
struct LayerContours {
    QList<QList<QPointF>> allPolygons;
    bool isConcentricCircles;
    QList<double> radii;
    double centerX, centerY;
};

// ====================================================================
// PARALLEL SLICING: Pre-sliced layer data
// ====================================================================
// Each layer is sliced independently in a worker thread, producing this
// struct. The main thread then processes them sequentially for G-code.
struct PreSlicedLayer {
    int layerIdx;
    double cadZ;
    double machineZ;
    LayerContours contours;           // Model contours only
    QList<QList<QPointF>> supportPolygons;  // Support contours (separate)
    QList<QPair<QVector3D, QVector3D>> vizPoints;
    bool hasContours;
    bool hasSupportContours;
};

// ====================================================================
// SUPPORT MESH IMPLEMENTATION — triangle storage + Z-sorted index
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
// FAST POLYGON MESH SLICER — O(active_triangles) per layer
// ====================================================================
// For each Z height, finds all triangles that span that Z,
// computes the plane-edge intersection segment for each, then
// chains segments into closed/open contour polylines.
//
// With the Z-sorted index, we binary-search to the first triangle
// whose minZ <= sliceZ, then scan forward while minZ <= sliceZ.
// Triangles with maxZ < sliceZ are skipped. This gives O(A) per
// layer where A = active triangles at that Z (typically ~1-5% of total).
// ====================================================================

struct SliceSegment {
    float x1, y1, x2, y2;
};

static QList<QList<QPointF>> sliceSupportMeshAtZ(
    const SupportMesh& mesh,
    double sliceZ,
    double modelCenterX, double modelCenterY,
    double bedCenterX, double bedCenterY)
{
    QList<QList<QPointF>> result;
    if (mesh.isEmpty()) return result;

    const float z = static_cast<float>(sliceZ);
    const auto& tris = mesh.triangles;
    const auto& idx = mesh.sortedByMinZ;

    // Binary search: find first triangle with minZ <= z
    int lo = 0, hi = idx.size();
    while (lo < hi) {
        int mid = (lo + hi) / 2;
        if (tris[idx[mid]].minZ > z) hi = mid;
        else lo = mid + 1;
    }
    // lo = first index where minZ > z, so all indices [0, lo) have minZ <= z

    // Collect intersection segments
    QList<SliceSegment> segments;
    segments.reserve(lo / 4); // Rough estimate

    for (int ii = 0; ii < lo; ++ii) {
        const SupportTriangle& tri = tris[idx[ii]];
        if (tri.maxZ < z) continue; // Triangle entirely below this plane

        // Find 2 intersection points where edges cross z
        float pts[4]; // x1,y1, x2,y2
        int ptCount = 0;

        // Check each of the 3 edges
        const float* verts[3] = { tri.v0, tri.v1, tri.v2 };
        for (int e = 0; e < 3 && ptCount < 4; ++e) {
            const float* va = verts[e];
            const float* vb = verts[(e + 1) % 3];

            // Edge crosses z if one vertex is above and one below (or on) z
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
            // Transform to bed coordinates
            SliceSegment seg;
            seg.x1 = pts[0] - modelCenterX + bedCenterX;
            seg.y1 = pts[1] - modelCenterY + bedCenterY;
            seg.x2 = pts[2] - modelCenterX + bedCenterX;
            seg.y2 = pts[3] - modelCenterY + bedCenterY;
            segments.append(seg);
        }
    }

    if (segments.isEmpty()) return result;

    // Chain segments into polylines using spatial hashing
    // Each segment endpoint gets a hash key; endpoints within 0.01mm merge.
    const double snapTol = 0.01;
    const double invTol = 1.0 / snapTol;
    auto snapKey = [invTol](float x, float y) -> int64_t {
        int64_t ix = static_cast<int64_t>(floor(x * invTol));
        int64_t iy = static_cast<int64_t>(floor(y * invTol));
        return ix * 73856093LL ^ iy * 19349669LL;
        };

    // Build adjacency: for each endpoint hash, list of (segIdx, which_end)
    struct EndRef { int segIdx; bool isEnd2; };
    QHash<int64_t, QList<EndRef>> adjacency;
    for (int i = 0; i < segments.size(); ++i) {
        const auto& s = segments[i];
        adjacency[snapKey(s.x1, s.y1)].append({ i, false });
        adjacency[snapKey(s.x2, s.y2)].append({ i, true });
    }

    // Walk chains
    QVector<bool> used(segments.size(), false);
    for (int startSeg = 0; startSeg < segments.size(); ++startSeg) {
        if (used[startSeg]) continue;
        used[startSeg] = true;

        QList<QPointF> chain;
        const auto& s = segments[startSeg];
        chain.append(QPointF(s.x1, s.y1));
        chain.append(QPointF(s.x2, s.y2));

        // Extend forward from endpoint
        bool extended = true;
        while (extended) {
            extended = false;
            float cx = chain.last().x(), cy = chain.last().y();
            int64_t key = snapKey(cx, cy);
            auto it = adjacency.find(key);
            if (it != adjacency.end()) {
                for (const auto& ref : it.value()) {
                    if (used[ref.segIdx]) continue;
                    const auto& ns = segments[ref.segIdx];
                    float nx, ny;
                    if (!ref.isEnd2) { // matched on (x1,y1), extend to (x2,y2)
                        nx = ns.x2; ny = ns.y2;
                    }
                    else { // matched on (x2,y2), extend to (x1,y1)
                        nx = ns.x1; ny = ns.y1;
                    }
                    used[ref.segIdx] = true;
                    chain.append(QPointF(nx, ny));
                    extended = true;
                    break;
                }
            }
        }

        // Extend backward from start
        extended = true;
        while (extended) {
            extended = false;
            float cx = chain.first().x(), cy = chain.first().y();
            int64_t key = snapKey(cx, cy);
            auto it = adjacency.find(key);
            if (it != adjacency.end()) {
                for (const auto& ref : it.value()) {
                    if (used[ref.segIdx]) continue;
                    const auto& ns = segments[ref.segIdx];
                    float nx, ny;
                    if (!ref.isEnd2) {
                        nx = ns.x2; ny = ns.y2;
                    }
                    else {
                        nx = ns.x1; ny = ns.y1;
                    }
                    used[ref.segIdx] = true;
                    chain.prepend(QPointF(nx, ny));
                    extended = true;
                    break;
                }
            }
        }

        if (chain.size() >= 3) {
            result.append(chain);
        }
    }

    return result;
}

// ====================================================================
// G-CODE FILE READER + Z-BUFFER REORDER
// ====================================================================
static QMap<double, QString> globalZBuffer;

QString readGCodeTemplate(const QString& filePath) {
    QString templateContent;
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qDebug() << "   WARNING: Could not open template file:" << filePath;
        if (filePath.contains("start")) return "; --- Start G-Code Missing ---\nG21\nG90\nM83\n";
        else return "; --- End G-Code Missing ---\nM400\n";
    }
    else {
        QTextStream in(&file);
        templateContent = in.readAll();
    }

    if (filePath.contains("end")) {
        QString assembledGCode;
        QTextStream assembleStream(&assembledGCode);

        for (auto it = globalZBuffer.begin(); it != globalZBuffer.end(); ++it) {
            double z = it.key();
            assembleStream << "; --- CAD Z: " << z << " ---\n";
            assembleStream << "G1 Z" << z << " F1200\n";
            assembleStream << it.value();
        }
        globalZBuffer.clear();
        return assembledGCode + templateContent;
    }

    return templateContent;
}

// ====================================================================
// BASIC GEOMETRIC FUNCTIONS
// ====================================================================
static inline double pointDistance(const QPointF& a, const QPointF& b) {
    double dx = b.x() - a.x(), dy = b.y() - a.y();
    return sqrt(dx * dx + dy * dy);
}

static inline double crossProduct2D(const QPointF& O, const QPointF& A, const QPointF& B) {
    return (A.x() - O.x()) * (B.y() - O.y()) - (A.y() - O.y()) * (B.x() - O.x());
}

static double angleBetweenVectors(const QPointF& v1, const QPointF& v2) {
    double len1 = sqrt(v1.x() * v1.x() + v1.y() * v1.y());
    double len2 = sqrt(v2.x() * v2.x() + v2.y() * v2.y());
    if (len1 < 1e-9 || len2 < 1e-9) return 0;
    double dot = (v1.x() * v2.x() + v1.y() * v2.y()) / (len1 * len2);
    dot = std::max(-1.0, std::min(1.0, dot));
    return acos(dot);
}

static bool isSharpCorner(const QPointF& before, const QPointF& at, const QPointF& after, double minAngle) {
    QPointF v1(at.x() - before.x(), at.y() - before.y());
    QPointF v2(after.x() - at.x(), after.y() - at.y());
    return angleBetweenVectors(v1, v2) > minAngle;
}

static double pointToLineDistance(const QPointF& A, const QPointF& B, const QPointF& C) {
    double ABx = B.x() - A.x();
    double ABy = B.y() - A.y();
    double ACx = C.x() - A.x();
    double ACy = C.y() - A.y();
    double lenAB = sqrt(ABx * ABx + ABy * ABy);
    if (lenAB < 1e-10) return sqrt(ACx * ACx + ACy * ACy);
    double crossProduct = ABx * ACy - ABy * ACx;
    return std::abs(crossProduct) / lenAB;
}

static QList<QPointF> removeDuplicatePoints(const QList<QPointF>& input, double minDist) {
    if (input.size() <= 1) return input;
    QList<QPointF> result;
    result.reserve(input.size());
    result.append(input[0]);
    for (int i = 1; i < input.size(); ++i) {
        if (pointDistance(result.last(), input[i]) >= minDist) {
            result.append(input[i]);
        }
    }
    return result;
}

// ====================================================================
// RAMER-DOUGLAS-PEUCKER LINE SIMPLIFICATION  —  O(n log n) average
// ====================================================================
// Replaces the old O(n²) optimizeStraightLines() and the expanding
// arePointsCollinear() window in optimizePath().
//
// How it works:
//   1. Draw line from first point to last point
//   2. Find point with maximum perpendicular distance from that line
//   3. If max distance > epsilon, recursively simplify both halves
//   4. Otherwise, the whole segment is "straight enough" — keep only endpoints
//
// This is O(n log n) on average for typical contour data (O(n²) worst case
// for pathological inputs, but those don't occur in CNC contours).
// ====================================================================

// Iterative RDP using an explicit stack (avoids stack overflow on
// very long contours with thousands of points).
static void rdpSimplifyIterative(
    const QList<QPointF>& points,
    double epsilon,
    std::vector<bool>& keep)
{
    const int n = points.size();
    if (n <= 2) {
        for (int i = 0; i < n; ++i) keep[i] = true;
        return;
    }

    keep[0] = true;
    keep[n - 1] = true;

    // Stack stores (startIdx, endIdx) pairs to process
    struct Range { int start, end; };
    std::vector<Range> stack;
    stack.reserve(32); // log2(n) typical depth
    stack.push_back({ 0, n - 1 });

    while (!stack.empty()) {
        Range r = stack.back();
        stack.pop_back();

        if (r.end - r.start < 2) continue;

        // Find point with maximum distance from line (r.start -> r.end)
        const QPointF& A = points[r.start];
        const QPointF& B = points[r.end];
        double ABx = B.x() - A.x();
        double ABy = B.y() - A.y();
        double lenAB = sqrt(ABx * ABx + ABy * ABy);

        double maxDist = 0.0;
        int maxIdx = r.start;

        if (lenAB < 1e-10) {
            // Degenerate: start == end, find farthest point
            for (int i = r.start + 1; i < r.end; ++i) {
                double dx = points[i].x() - A.x();
                double dy = points[i].y() - A.y();
                double dist = sqrt(dx * dx + dy * dy);
                if (dist > maxDist) {
                    maxDist = dist;
                    maxIdx = i;
                }
            }
        }
        else {
            double invLenAB = 1.0 / lenAB;
            for (int i = r.start + 1; i < r.end; ++i) {
                double ACx = points[i].x() - A.x();
                double ACy = points[i].y() - A.y();
                double dist = std::abs(ABx * ACy - ABy * ACx) * invLenAB;
                if (dist > maxDist) {
                    maxDist = dist;
                    maxIdx = i;
                }
            }
        }

        if (maxDist > epsilon) {
            keep[maxIdx] = true;
            // Push both halves onto the stack
            // Push right half first so left is processed first (optional ordering)
            if (maxIdx + 1 < r.end)
                stack.push_back({ maxIdx, r.end });
            if (r.start + 1 < maxIdx)
                stack.push_back({ r.start, maxIdx });
        }
        // else: all points between r.start and r.end are within epsilon
        //       of the line — they get discarded (keep[] stays false)
    }
}

// Public API: returns simplified point list
static QList<QPointF> rdpSimplify(const QList<QPointF>& input, double epsilon) {
    if (input.size() <= 2) return input;

    std::vector<bool> keep(input.size(), false);
    rdpSimplifyIterative(input, epsilon, keep);

    QList<QPointF> result;
    result.reserve(input.size() / 2); // Typical 50% reduction
    for (int i = 0; i < input.size(); ++i) {
        if (keep[i]) result.append(input[i]);
    }
    return result;
}

// ====================================================================
// ARC FITTING — with INCREMENTAL least-squares circle fit
// ====================================================================
// The old version recomputed all sums from scratch every iteration.
// This version maintains running sums and just adds the new point,
// turning O(k²) per arc attempt into O(k).
// ====================================================================

struct IncrementalCircleFitter {
    int n;
    double sumX, sumY;
    double Suu, Suv, Svv, Suuu, Svvv, Suvv, Svuu;
    // Store individual coordinates for max-error check
    // (we need them for the final validation pass)
    const QList<QPointF>* points;
    int startIdx;

    void init(const QList<QPointF>* pts, int start) {
        points = pts;
        startIdx = start;
        n = 0;
        sumX = sumY = 0;
        Suu = Suv = Svv = Suuu = Svvv = Suvv = Svuu = 0;
    }

    // Call this for each point in order: startIdx, startIdx+1, ...
    // IMPORTANT: must be called sequentially, one point at a time.
    // The sums use the CURRENT mean, so we have to recompute from
    // scratch when the mean shifts. For efficiency, we store raw sums
    // and compute the centered moments at solve time.
    double rawSumX, rawSumY, rawSumXX, rawSumXY, rawSumYY;
    double rawSumXXX, rawSumYYY, rawSumXYY, rawSumYXX;

    void initRaw(const QList<QPointF>* pts, int start) {
        points = pts;
        startIdx = start;
        n = 0;
        rawSumX = rawSumY = 0;
        rawSumXX = rawSumXY = rawSumYY = 0;
        rawSumXXX = rawSumYYY = rawSumXYY = rawSumYXX = 0;
    }

    void addPoint(int idx) {
        double x = (*points)[idx].x();
        double y = (*points)[idx].y();
        n++;
        rawSumX += x;
        rawSumY += y;
        rawSumXX += x * x;
        rawSumXY += x * y;
        rawSumYY += y * y;
        rawSumXXX += x * x * x;
        rawSumYYY += y * y * y;
        rawSumXYY += x * y * y;
        rawSumYXX += y * x * x;
    }

    // Solve for circle center and radius using current accumulated sums.
    // Returns max error (max distance of any point from the fitted circle).
    double solve(int endIdx, double& outCx, double& outCy, double& outRadius) {
        if (n < 3) return 1e9;

        double meanX = rawSumX / n;
        double meanY = rawSumY / n;

        // Compute centered moments from raw sums:
        // Suu = sum((x - meanX)^2) = sumXX - 2*meanX*sumX + n*meanX^2
        double cSuu = rawSumXX - 2.0 * meanX * rawSumX + n * meanX * meanX;
        double cSvv = rawSumYY - 2.0 * meanY * rawSumY + n * meanY * meanY;
        double cSuv = rawSumXY - meanX * rawSumY - meanY * rawSumX + n * meanX * meanY;

        // Suuu = sum((x-mx)^3) = sumXXX - 3*mx*sumXX + 3*mx^2*sumX - n*mx^3
        double cSuuu = rawSumXXX - 3.0 * meanX * rawSumXX
            + 3.0 * meanX * meanX * rawSumX
            - n * meanX * meanX * meanX;
        double cSvvv = rawSumYYY - 3.0 * meanY * rawSumYY
            + 3.0 * meanY * meanY * rawSumY
            - n * meanY * meanY * meanY;

        // Suvv = sum((x-mx)*(y-my)^2)
        // = sum(x*y^2) - 2*my*sum(x*y) + my^2*sum(x) - mx*sum(y^2) + 2*mx*my*sum(y) - n*mx*my^2
        double cSuvv = rawSumXYY - 2.0 * meanY * rawSumXY + meanY * meanY * rawSumX
            - meanX * rawSumYY + 2.0 * meanX * meanY * rawSumY
            - n * meanX * meanY * meanY;
        double cSvuu = rawSumYXX - 2.0 * meanX * rawSumXY + meanX * meanX * rawSumY
            - meanY * rawSumXX + 2.0 * meanX * meanY * rawSumX
            - n * meanX * meanX * meanY;

        double denom = 2.0 * (cSuu * cSvv - cSuv * cSuv);
        if (std::abs(denom) < 1e-10) return 1e9;

        double uc = (cSvv * (cSuuu + cSuvv) - cSuv * (cSvvv + cSvuu)) / denom;
        double vc = (cSuu * (cSvvv + cSvuu) - cSuv * (cSuuu + cSuvv)) / denom;
        outCx = uc + meanX;
        outCy = vc + meanY;
        outRadius = sqrt(uc * uc + vc * vc + (cSuu + cSvv) / n);

        // Compute max error
        double maxError = 0;
        for (int i = startIdx; i <= endIdx; ++i) {
            double dx = (*points)[i].x() - outCx;
            double dy = (*points)[i].y() - outCy;
            double dist = sqrt(dx * dx + dy * dy);
            double error = std::abs(dist - outRadius);
            if (error > maxError) maxError = error;
        }
        return maxError;
    }
};

static bool isClockwiseArc(const QList<QPointF>& points, int startIdx, int endIdx, double cx, double cy) {
    double signedArea = 0;
    for (int i = startIdx; i < endIdx; ++i) {
        double x1 = points[i].x() - cx; double y1 = points[i].y() - cy;
        double x2 = points[i + 1].x() - cx; double y2 = points[i + 1].y() - cy;
        signedArea += (x1 * y2 - x2 * y1);
    }
    return signedArea < 0;
}

static double calculateArcLength(const QPointF& start, const QPointF& end, double cx, double cy, double radius, bool clockwise) {
    double startAngle = atan2(start.y() - cy, start.x() - cx);
    double endAngle = atan2(end.y() - cy, end.x() - cx);
    double deltaAngle = endAngle - startAngle;
    if (clockwise) { if (deltaAngle > 0) deltaAngle -= 2.0 * M_PI; }
    else { if (deltaAngle < 0) deltaAngle += 2.0 * M_PI; }
    return std::abs(deltaAngle) * radius;
}

// ====================================================================
// TRY TO FIT ARC — OPTIMIZED with incremental circle fit + early exit
// ====================================================================
static int tryFitArcExtended(const QList<QPointF>& points, int startIdx, double epsilon, OptimizedSegment& outSeg) {
    int maxPoints = qMin(startIdx + GCodeConfig::MAX_POINTS_TO_CHECK_ARC, points.size() - 1);
    if (startIdx + 2 > maxPoints) return 0;

    if (startIdx + 2 < points.size()) {
        if (isSharpCorner(points[startIdx], points[startIdx + 1], points[startIdx + 2], GCodeConfig::MIN_CORNER_ANGLE_RAD)) {
            return 0;
        }
    }

    int bestEndIdx = -1;
    double bestCx = 0, bestCy = 0, bestRadius = 0;

    // INCREMENTAL circle fitter — O(1) per new point instead of O(k)
    IncrementalCircleFitter fitter;
    fitter.initRaw(&points, startIdx);

    // Seed with first two points
    fitter.addPoint(startIdx);
    fitter.addPoint(startIdx + 1);

    // Track the sign of the first cross product for inflection detection
    double firstCross = 0;
    bool haveFirstCross = false;

    for (int endIdx = startIdx + 2; endIdx <= maxPoints; ++endIdx) {

        // Quick inflection check on the last 3 points
        if (endIdx >= startIdx + 3) {
            double cross = crossProduct2D(points[endIdx - 2], points[endIdx - 1], points[endIdx]);
            if (std::abs(cross) > 0.05) {
                if (!haveFirstCross) {
                    firstCross = crossProduct2D(points[startIdx], points[startIdx + 1], points[startIdx + 2]);
                    haveFirstCross = true;
                }
                if ((firstCross >= 0) != (cross >= 0)) break;
            }
        }

        // Sharp corner at the new point
        if (endIdx < points.size() - 1) {
            if (isSharpCorner(points[endIdx - 1], points[endIdx], points[endIdx + 1], GCodeConfig::MIN_CORNER_ANGLE_RAD))
                break;
        }

        // Incrementally add the new point (O(1))
        fitter.addPoint(endIdx);

        // Solve for the circle (O(k) for max-error, but the solve itself is O(1))
        double cx, cy, radius;
        double fitError = fitter.solve(endIdx, cx, cy, radius);

        if (fitError > epsilon) break;

        if (radius < GCodeConfig::MIN_ARC_RADIUS) continue;
        if (radius > GCodeConfig::MAX_ARC_RADIUS) continue;

        double startAngle = atan2(points[startIdx].y() - cy, points[startIdx].x() - cx);
        double endAngle = atan2(points[endIdx].y() - cy, points[endIdx].x() - cx);
        double deltaAngle = std::abs(endAngle - startAngle);
        if (deltaAngle > M_PI) deltaAngle = 2.0 * M_PI - deltaAngle;

        if (deltaAngle > GCodeConfig::MAX_ARC_ANGLE) break;
        if (deltaAngle < GCodeConfig::MIN_ARC_ANGLE) continue;

        double physicalArcLength = radius * deltaAngle;
        if (physicalArcLength < GCodeConfig::MIN_ARC_PHYSICAL_LENGTH) continue;

        double chordLength = pointDistance(points[startIdx], points[endIdx]);
        if (chordLength < GCodeConfig::MIN_CHORD_LENGTH) continue;

        double arcToChordRatio = physicalArcLength / chordLength;
        if (arcToChordRatio < GCodeConfig::MIN_ARC_TO_CHORD_RATIO) continue;

        bestEndIdx = endIdx;
        bestCx = cx;
        bestCy = cy;
        bestRadius = radius;
    }

    if (bestEndIdx < 0) return 0;

    // Check if really an arc or just a straight line
    // Use quick check: perpendicular distance from midpoint to chord
    {
        int mid = (startIdx + bestEndIdx) / 2;
        double midDist = pointToLineDistance(points[startIdx], points[bestEndIdx], points[mid]);
        if (midDist < GCodeConfig::LINE_COLLINEARITY_EPSILON) return 0;
    }

    if ((bestEndIdx - startIdx + 1) < GCodeConfig::MIN_POINTS_IN_ARC) {
        return 0;
    }

    bool clockwise = isClockwiseArc(points, startIdx, bestEndIdx, bestCx, bestCy);
    outSeg.type = clockwise ? OptimizedSegment::ARC_CW : OptimizedSegment::ARC_CCW;
    outSeg.start = points[startIdx];
    outSeg.end = points[bestEndIdx];
    outSeg.centerX = bestCx;
    outSeg.centerY = bestCy;
    outSeg.radius = bestRadius;
    outSeg.arcLength = calculateArcLength(outSeg.start, outSeg.end, bestCx, bestCy, bestRadius, clockwise);

    return bestEndIdx - startIdx + 1;
}

// ====================================================================
// CONTOUR SMOOTHING — double-buffered (no extra allocations per pass)
// ====================================================================
static QList<QPointF> smoothContour(const QList<QPointF>& input, int passes = 1) {
    if (input.size() < 5) return input;

    QList<QPointF> buf1 = input;
    QList<QPointF> buf2;
    buf2.reserve(input.size());

    for (int pass = 0; pass < passes; ++pass) {
        QList<QPointF>& src = (pass % 2 == 0) ? buf1 : buf2;
        QList<QPointF>& dst = (pass % 2 == 0) ? buf2 : buf1;
        dst.clear();
        dst.append(src[0]);
        for (int i = 1; i < src.size() - 1; ++i) {
            double sx = src[i - 1].x() * 0.25 + src[i].x() * 0.50 + src[i + 1].x() * 0.25;
            double sy = src[i - 1].y() * 0.25 + src[i].y() * 0.50 + src[i + 1].y() * 0.25;
            dst.append(QPointF(sx, sy));
        }
        dst.append(src.last());
    }
    return (passes % 2 == 0) ? buf1 : buf2;
}

// ====================================================================
// POST-PROCESS SEGMENTS: merge tiny G1 gaps between arcs
// ====================================================================
static QList<OptimizedSegment> postProcessSegments(const QList<OptimizedSegment>& input) {
    if (input.size() < 2) return input;

    QList<OptimizedSegment> result;
    result.reserve(input.size());

    for (int i = 0; i < input.size(); ++i) {
        const auto& seg = input[i];

        if (seg.type == OptimizedSegment::LINE && seg.arcLength < 0.3) {
            bool prevIsArc = (i > 0 && input[i - 1].type != OptimizedSegment::LINE);
            bool nextIsArc = (i + 1 < input.size() && input[i + 1].type != OptimizedSegment::LINE);

            if (prevIsArc && nextIsArc) {
                if (!result.isEmpty()) {
                    result.last().end = seg.end;
                    auto& prev = result.last();
                    bool cw = (prev.type == OptimizedSegment::ARC_CW);
                    prev.arcLength = calculateArcLength(prev.start, prev.end,
                        prev.centerX, prev.centerY, prev.radius, cw);
                }
                continue;
            }

            if (prevIsArc && i == input.size() - 1) {
                if (!result.isEmpty()) {
                    result.last().end = seg.end;
                    auto& prev = result.last();
                    bool cw = (prev.type == OptimizedSegment::ARC_CW);
                    prev.arcLength = calculateArcLength(prev.start, prev.end,
                        prev.centerX, prev.centerY, prev.radius, cw);
                }
                continue;
            }
        }

        if (!result.isEmpty()) {
            OptimizedSegment adjusted = seg;
            adjusted.start = result.last().end;
            result.append(adjusted);
        }
        else {
            result.append(seg);
        }
    }

    return result;
}

// ====================================================================
// UNIFIED CURVATURE-BASED PATH OPTIMIZER
// ====================================================================
// Replaces the old RDP → arc fitting → post-process pipeline.
// Computes curvature FIRST, segments by curvature, then applies
// the right tool (RDP for lines, circle fit for arcs).
// ====================================================================

static double mengerCurvature(const QPointF& p0, const QPointF& p1, const QPointF& p2) {
    double cross = (p1.x() - p0.x()) * (p2.y() - p0.y())
        - (p1.y() - p0.y()) * (p2.x() - p0.x());
    double d01 = pointDistance(p0, p1);
    double d12 = pointDistance(p1, p2);
    double d02 = pointDistance(p0, p2);
    double denom = d01 * d12 * d02;
    if (denom < 1e-12) return 0.0;
    return 2.0 * cross / denom;
}

static std::vector<int> findCorners(
    const QList<QPointF>& pts,
    const std::vector<double>& curvature,
    double cornerAngleRad,
    double curvatureLineThreshold)
{
    std::vector<int> corners;
    corners.push_back(0);

    for (int i = 1; i < (int)pts.size() - 1; ++i) {
        QPointF v1(pts[i].x() - pts[i - 1].x(), pts[i].y() - pts[i - 1].y());
        QPointF v2(pts[i + 1].x() - pts[i].x(), pts[i + 1].y() - pts[i].y());
        double len1 = sqrt(v1.x() * v1.x() + v1.y() * v1.y());
        double len2 = sqrt(v2.x() * v2.x() + v2.y() * v2.y());

        if (len1 > 1e-9 && len2 > 1e-9) {
            double dot = (v1.x() * v2.x() + v1.y() * v2.y()) / (len1 * len2);
            dot = std::max(-1.0, std::min(1.0, dot));
            if (acos(dot) > cornerAngleRad) { corners.push_back(i); continue; }
        }

        if (i > 0 && i < (int)curvature.size()) {
            double kP = curvature[i - 1], kC = curvature[i];
            if (std::abs(kP) > curvatureLineThreshold &&
                std::abs(kC) > curvatureLineThreshold &&
                (kP > 0) != (kC > 0)) {
                corners.push_back(i);
            }
        }
    }

    corners.push_back((int)pts.size() - 1);
    std::sort(corners.begin(), corners.end());
    corners.erase(std::unique(corners.begin(), corners.end()), corners.end());
    return corners;
}

enum SegClass { SEG_LINE, SEG_ARC, SEG_MIXED };

static SegClass classifySegment(
    const std::vector<double>& curvature,
    int startIdx, int endIdx,
    double lineThreshold, double uniformityFactor)
{
    if (endIdx - startIdx < 2) return SEG_LINE;

    int kS = std::max(startIdx, 0);
    int kE = std::min(endIdx, (int)curvature.size());
    if (kS >= kE) return SEG_LINE;

    double kMin = 1e9, kMax = -1e9, kAbsMax = 0;
    int straightCount = 0, total = 0;

    for (int i = kS; i < kE; ++i) {
        double k = curvature[i], ka = std::abs(k);
        if (ka > kAbsMax) kAbsMax = ka;
        if (k < kMin) kMin = k;
        if (k > kMax) kMax = k;
        if (ka < lineThreshold) straightCount++;
        total++;
    }

    if (total == 0) return SEG_LINE;
    if ((double)straightCount / total > 0.8 || kAbsMax < lineThreshold) return SEG_LINE;

    bool sameSign = (kMin >= 0 && kMax >= 0) || (kMin <= 0 && kMax <= 0);
    if (sameSign && kAbsMax > lineThreshold) {
        double kAbsMin = 1e9;
        for (int i = kS; i < kE; ++i) {
            double ka = std::abs(curvature[i]);
            if (ka > lineThreshold && ka < kAbsMin) kAbsMin = ka;
        }
        if (kAbsMin < 1e9 && kAbsMax / kAbsMin < uniformityFactor) return SEG_ARC;
    }
    return SEG_MIXED;
}

struct StableCircleFitter {
    double originX, originY;
    int n;
    double rawSumX, rawSumY, rawSumXX, rawSumXY, rawSumYY;
    double rawSumXXX, rawSumYYY, rawSumXYY, rawSumYXX;
    const QList<QPointF>* points;
    int startIdx;

    void init(const QList<QPointF>* pts, int start, int end) {
        points = pts; startIdx = start; n = 0;
        originX = ((*pts)[start].x() + (*pts)[end].x()) * 0.5;
        originY = ((*pts)[start].y() + (*pts)[end].y()) * 0.5;
        rawSumX = rawSumY = 0; rawSumXX = rawSumXY = rawSumYY = 0;
        rawSumXXX = rawSumYYY = rawSumXYY = rawSumYXX = 0;
    }

    void addPoint(int idx) {
        double x = (*points)[idx].x() - originX;
        double y = (*points)[idx].y() - originY;
        n++;
        rawSumX += x;    rawSumY += y;
        rawSumXX += x * x; rawSumXY += x * y; rawSumYY += y * y;
        rawSumXXX += x * x * x; rawSumYYY += y * y * y;
        rawSumXYY += x * y * y; rawSumYXX += y * x * x;
    }

    double solve(int endIdx, double& outCx, double& outCy, double& outRadius) {
        if (n < 3) return 1e9;
        double mx = rawSumX / n, my = rawSumY / n;
        double Suu = rawSumXX - 2 * mx * rawSumX + n * mx * mx;
        double Svv = rawSumYY - 2 * my * rawSumY + n * my * my;
        double Suv = rawSumXY - mx * rawSumY - my * rawSumX + n * mx * my;
        double Suuu = rawSumXXX - 3 * mx * rawSumXX + 3 * mx * mx * rawSumX - n * mx * mx * mx;
        double Svvv = rawSumYYY - 3 * my * rawSumYY + 3 * my * my * rawSumY - n * my * my * my;
        double Suvv = rawSumXYY - 2 * my * rawSumXY + my * my * rawSumX
            - mx * rawSumYY + 2 * mx * my * rawSumY - n * mx * my * my;
        double Svuu = rawSumYXX - 2 * mx * rawSumXY + mx * mx * rawSumY
            - my * rawSumXX + 2 * mx * my * rawSumX - n * mx * mx * my;
        double denom = 2.0 * (Suu * Svv - Suv * Suv);
        if (std::abs(denom) < 1e-10) return 1e9;
        double uc = (Svv * (Suuu + Suvv) - Suv * (Svvv + Svuu)) / denom;
        double vc = (Suu * (Svvv + Svuu) - Suv * (Suuu + Suvv)) / denom;
        outCx = uc + mx + originX;
        outCy = vc + my + originY;
        outRadius = sqrt(uc * uc + vc * vc + (Suu + Svv) / n);
        double maxErr = 0;
        for (int i = startIdx; i <= endIdx; ++i) {
            double dx = (*points)[i].x() - outCx, dy = (*points)[i].y() - outCy;
            double err = std::abs(sqrt(dx * dx + dy * dy) - outRadius);
            if (err > maxErr) maxErr = err;
        }
        return maxErr;
    }
};

static bool fitArcToRegion(const QList<QPointF>& pts, int startIdx, int endIdx,
    double epsilon, OptimizedSegment& outSeg)
{
    int numPts = endIdx - startIdx + 1;
    if (numPts < GCodeConfig::MIN_POINTS_IN_ARC) return false;

    // --- Circle fit ---
    StableCircleFitter fitter;
    fitter.init(&pts, startIdx, endIdx);
    for (int i = startIdx; i <= endIdx; ++i) fitter.addPoint(i);
    double cx, cy, radius;
    double fitError = fitter.solve(endIdx, cx, cy, radius);
    if (fitError > epsilon) return false;
    if (radius < GCodeConfig::MIN_ARC_RADIUS || radius > GCodeConfig::MAX_ARC_RADIUS) return false;

    // --- Angle & length checks ---
    double sa = atan2(pts[startIdx].y() - cy, pts[startIdx].x() - cx);
    double ea = atan2(pts[endIdx].y() - cy, pts[endIdx].x() - cx);
    double da = std::abs(ea - sa);
    if (da > M_PI) da = 2 * M_PI - da;
    if (da > GCodeConfig::MAX_ARC_ANGLE || da < GCodeConfig::MIN_ARC_ANGLE) return false;
    double physLen = radius * da;
    if (physLen < GCodeConfig::MIN_ARC_PHYSICAL_LENGTH) return false;
    double chord = pointDistance(pts[startIdx], pts[endIdx]);
    if (chord < GCodeConfig::MIN_CHORD_LENGTH) return false;
    if (physLen / chord < GCodeConfig::MIN_ARC_TO_CHORD_RATIO) return false;

    // --- Determine arc direction ---
    double signedArea = 0;
    for (int i = startIdx; i < endIdx; ++i) {
        double x1 = pts[i].x() - cx, y1 = pts[i].y() - cy;
        double x2 = pts[i + 1].x() - cx, y2 = pts[i + 1].y() - cy;
        signedArea += (x1 * y2 - x2 * y1);
    }
    bool cw = signedArea < 0;

    // --- MIDPOINT VALIDATION ---
    // The arc's geometric midpoint must be close to the actual point midpoint.
    // This catches the "wrong-side bulge" where the circle fits the endpoints
    // but curves in the wrong direction between them.
    {
        // Actual midpoint of the original points
        int midIdx = (startIdx + endIdx) / 2;
        QPointF actualMid = pts[midIdx];

        // Predicted midpoint on the fitted arc
        double midAngle;
        double startAngle = atan2(pts[startIdx].y() - cy, pts[startIdx].x() - cx);
        double endAngle = atan2(pts[endIdx].y() - cy, pts[endIdx].x() - cx);
        double sweep = endAngle - startAngle;
        if (cw) { if (sweep > 0) sweep -= 2 * M_PI; }
        else { if (sweep < 0) sweep += 2 * M_PI; }
        midAngle = startAngle + sweep * 0.5;

        QPointF arcMid(cx + radius * cos(midAngle), cy + radius * sin(midAngle));

        double midError = pointDistance(actualMid, arcMid);
        if (midError > GCodeConfig::ARC_MIDPOINT_TOLERANCE) return false;
    }

    // --- QUARTER-POINT VALIDATION ---
    // Also check at 25% and 75% to catch S-shaped deviations
    {
        double startAngle = atan2(pts[startIdx].y() - cy, pts[startIdx].x() - cx);
        double endAngle = atan2(pts[endIdx].y() - cy, pts[endIdx].x() - cx);
        double sweep = endAngle - startAngle;
        if (cw) { if (sweep > 0) sweep -= 2 * M_PI; }
        else { if (sweep < 0) sweep += 2 * M_PI; }

        int q1Idx = startIdx + numPts / 4;
        int q3Idx = startIdx + (numPts * 3) / 4;

        double q1Angle = startAngle + sweep * 0.25;
        double q3Angle = startAngle + sweep * 0.75;

        QPointF arcQ1(cx + radius * cos(q1Angle), cy + radius * sin(q1Angle));
        QPointF arcQ3(cx + radius * cos(q3Angle), cy + radius * sin(q3Angle));

        double q1Error = pointDistance(pts[q1Idx], arcQ1);
        double q3Error = pointDistance(pts[q3Idx], arcQ3);

        // Quarter-point tolerance is slightly looser than midpoint
        double qTol = GCodeConfig::ARC_MIDPOINT_TOLERANCE * 1.5;
        if (q1Error > qTol || q3Error > qTol) return false;
    }

    // --- DIRECTION CONSISTENCY ---
    // All points must be on the same side of the chord line.
    // If any point crosses the chord, the arc is invalid.
    {
        double chordDx = pts[endIdx].x() - pts[startIdx].x();
        double chordDy = pts[endIdx].y() - pts[startIdx].y();
        int positive = 0, negative = 0;
        for (int i = startIdx + 1; i < endIdx; ++i) {
            double cross = chordDx * (pts[i].y() - pts[startIdx].y())
                - chordDy * (pts[i].x() - pts[startIdx].x());
            if (cross > 0.001) positive++;
            else if (cross < -0.001) negative++;
        }
        // If points are on both sides of the chord, reject
        if (positive > 0 && negative > 0) return false;
    }

    // --- All checks passed — emit the arc ---
    outSeg.type = cw ? OptimizedSegment::ARC_CW : OptimizedSegment::ARC_CCW;
    outSeg.start = pts[startIdx]; outSeg.end = pts[endIdx];
    outSeg.centerX = cx; outSeg.centerY = cy; outSeg.radius = radius;
    double a1 = atan2(outSeg.start.y() - cy, outSeg.start.x() - cx);
    double a2 = atan2(outSeg.end.y() - cy, outSeg.end.x() - cx);
    double d = a2 - a1;
    if (cw) { if (d > 0) d -= 2 * M_PI; }
    else { if (d < 0) d += 2 * M_PI; }
    outSeg.arcLength = std::abs(d) * radius;
    return true;
}

static void emitLineSegments(const QList<QPointF>& pts, int startIdx, int endIdx,
    double lineEpsilon, QList<OptimizedSegment>& out)
{
    if (endIdx <= startIdx) return;
    if (endIdx - startIdx <= 2) {
        OptimizedSegment seg; seg.type = OptimizedSegment::LINE;
        seg.start = pts[startIdx]; seg.end = pts[endIdx];
        seg.arcLength = pointDistance(seg.start, seg.end);
        out.append(seg); return;
    }
    QList<QPointF> sub;
    sub.reserve(endIdx - startIdx + 1);
    for (int i = startIdx; i <= endIdx; ++i) sub.append(pts[i]);
    QList<QPointF> simplified = rdpSimplify(sub, lineEpsilon);
    for (int i = 0; i < simplified.size() - 1; ++i) {
        OptimizedSegment seg; seg.type = OptimizedSegment::LINE;
        seg.start = simplified[i]; seg.end = simplified[i + 1];
        seg.arcLength = pointDistance(seg.start, seg.end);
        out.append(seg);
    }
}

static void processMixedSegment(const QList<QPointF>& pts,
    const std::vector<double>& curvature, int startIdx, int endIdx,
    double lineEpsilon, double arcEpsilon, double lineThreshold,
    QList<OptimizedSegment>& out)
{
    int runStart = startIdx;
    bool runIsCurved = (startIdx < (int)curvature.size()) ?
        (std::abs(curvature[startIdx]) > lineThreshold) : false;

    for (int i = startIdx + 1; i <= endIdx; ++i) {
        bool curved = (i < (int)curvature.size()) ?
            (std::abs(curvature[i]) > lineThreshold) : false;

        if (curved != runIsCurved && i > runStart + 1) {
            if (runIsCurved) {
                OptimizedSegment arcSeg;
                if (fitArcToRegion(pts, runStart, i - 1, arcEpsilon, arcSeg))
                    out.append(arcSeg);
                else emitLineSegments(pts, runStart, i - 1, lineEpsilon, out);
            }
            else {
                emitLineSegments(pts, runStart, i - 1, lineEpsilon, out);
            }
            runStart = i - 1;
            runIsCurved = curved;
        }
    }

    if (runStart < endIdx) {
        if (runIsCurved) {
            OptimizedSegment arcSeg;
            if (fitArcToRegion(pts, runStart, endIdx, arcEpsilon, arcSeg))
                out.append(arcSeg);
            else emitLineSegments(pts, runStart, endIdx, lineEpsilon, out);
        }
        else {
            emitLineSegments(pts, runStart, endIdx, lineEpsilon, out);
        }
    }
}

static QList<OptimizedSegment> optimizePathCurvature(
    const QList<QPointF>& inputPoints, double lineEpsilon, double arcEpsilon)
{
    QList<OptimizedSegment> segments;
    if (inputPoints.size() < 2) return segments;

    QList<QPointF> pts = removeDuplicatePoints(inputPoints, GCodeConfig::MIN_POINT_DISTANCE);
    if (pts.size() < 2) return segments;

    const int n = pts.size();

    // ---- STEP 1: Compute raw signed curvature ----
    std::vector<double> rawK(n, 0.0);
    for (int i = 1; i < n - 1; ++i)
        rawK[i] = mengerCurvature(pts[i - 1], pts[i], pts[i + 1]);

    // ---- STEP 2: Smooth curvature (5-point moving average) ----
    // Raw Menger curvature on dense points is very noisy.
    // Smoothing eliminates false inflection points from jitter.
    const int smoothWindow = 5;
    const int halfW = smoothWindow / 2;
    std::vector<double> curvature(n, 0.0);
    for (int i = halfW; i < n - halfW; ++i) {
        double sum = 0;
        for (int j = i - halfW; j <= i + halfW; ++j)
            sum += rawK[j];
        curvature[i] = sum / smoothWindow;
    }

    // ---- STEP 3: Find corners with minimum spacing ----
    // Only mark REAL sharp angles (not noise-induced inflections).
    // Require minimum 2mm (or 8 points) between consecutive corners.
    double minCornerSpacing = 2.0; // mm
    int minCornerPoints = 8;       // points

    std::vector<int> corners;
    corners.push_back(0);

    for (int i = 1; i < n - 1; ++i) {
        // Check minimum distance from previous corner
        int prevCorner = corners.back();
        double distFromPrev = 0;
        for (int k = prevCorner; k < i && k < n - 1; ++k)
            distFromPrev += pointDistance(pts[k], pts[k + 1]);
        if (distFromPrev < minCornerSpacing && (i - prevCorner) < minCornerPoints)
            continue;

        // (a) Sharp turning angle
        QPointF v1(pts[i].x() - pts[i - 1].x(), pts[i].y() - pts[i - 1].y());
        QPointF v2(pts[i + 1].x() - pts[i].x(), pts[i + 1].y() - pts[i].y());
        double len1 = sqrt(v1.x() * v1.x() + v1.y() * v1.y());
        double len2 = sqrt(v2.x() * v2.x() + v2.y() * v2.y());

        if (len1 > 1e-9 && len2 > 1e-9) {
            double dot = (v1.x() * v2.x() + v1.y() * v2.y()) / (len1 * len2);
            dot = std::max(-1.0, std::min(1.0, dot));
            if (acos(dot) > GCodeConfig::MIN_CORNER_ANGLE_RAD) {
                corners.push_back(i);
                continue;
            }
        }

        // (b) Inflection: curvature sign flip on SMOOTHED values
        // Use a higher threshold than lineThreshold to avoid noise
        double inflectionThreshold = 0.05; // κ > 0.05 means R < 20mm — real curve
        if (i >= halfW && i < n - halfW) {
            double kP = curvature[i - 1], kC = curvature[i];
            if (std::abs(kP) > inflectionThreshold &&
                std::abs(kC) > inflectionThreshold &&
                (kP > 0) != (kC > 0)) {
                corners.push_back(i);
            }
        }
    }

    corners.push_back(n - 1);
    std::sort(corners.begin(), corners.end());
    corners.erase(std::unique(corners.begin(), corners.end()), corners.end());

    // ---- STEP 4: Classify and process each segment ----
    double lineThreshold = 1.0 / GCodeConfig::MAX_ARC_RADIUS;
    double uniformity = 5.0; // Allow 5:1 curvature variation within an arc

    segments.reserve(corners.size() * 2);

    for (int ci = 0; ci + 1 < (int)corners.size(); ++ci) {
        int sI = corners[ci], eI = corners[ci + 1];
        if (eI <= sI) continue;

        // Single-edge segment
        if (eI - sI == 1) {
            OptimizedSegment seg; seg.type = OptimizedSegment::LINE;
            seg.start = pts[sI]; seg.end = pts[eI];
            seg.arcLength = pointDistance(seg.start, seg.end);
            segments.append(seg);
            continue;
        }

        // Try arc fit FIRST — the geometric validation checks (midpoint,
        // quarter-point, direction) will reject bad fits.
        // This catches clean arcs that classifySegment might call MIXED
        // due to curvature smoothing artifacts at segment boundaries.
        if (eI - sI >= GCodeConfig::MIN_POINTS_IN_ARC) {
            OptimizedSegment arcSeg;
            if (fitArcToRegion(pts, sI, eI, arcEpsilon, arcSeg)) {
                segments.append(arcSeg);
                continue;
            }
        }

        // Arc fit failed — classify by curvature
        SegClass cls = classifySegment(curvature, sI, eI, lineThreshold, uniformity);

        switch (cls) {
        case SEG_LINE:
            emitLineSegments(pts, sI, eI, lineEpsilon, segments);
            break;
        case SEG_ARC: {
            OptimizedSegment arcSeg;
            if (eI - sI >= GCodeConfig::MIN_POINTS_IN_ARC &&
                fitArcToRegion(pts, sI, eI, arcEpsilon, arcSeg)) {
                segments.append(arcSeg);
            }
            else {
                emitLineSegments(pts, sI, eI, lineEpsilon, segments);
            }
            break;
        }
        case SEG_MIXED:
            processMixedSegment(pts, curvature, sI, eI,
                lineEpsilon, arcEpsilon, lineThreshold, segments);
            break;
        }
    }

    return postProcessSegments(segments);
}

// ====================================================================
// OLD PATH OPTIMIZER — kept for reference / fallback
// ====================================================================
// Pipeline:
//   1. Remove duplicate points
//   2. RDP simplify (O(n log n)) — replaces the old O(n²) collinear scan
//   3. Arc fitting on the simplified points
//   4. Post-process to merge tiny G1 scars
// ====================================================================
static QList<OptimizedSegment> optimizePath(const QList<QPointF>& inputPoints, double lineEpsilon, double arcEpsilon) {
    QList<OptimizedSegment> segments;
    if (inputPoints.size() < 2) return segments;

    // Step 1: Remove exact/near duplicates
    QList<QPointF> cleaned = removeDuplicatePoints(inputPoints, GCodeConfig::MIN_POINT_DISTANCE);
    if (cleaned.size() < 2) return segments;

    // Step 2: RDP simplification — O(n log n) average
    // This replaces the old O(n²) arePointsCollinear expanding window.
    // After RDP, consecutive points that are collinear within epsilon
    // have been merged, so we only need to do arc fitting on the survivors.
    QList<QPointF> simplified = rdpSimplify(cleaned, lineEpsilon);
    if (simplified.size() < 2) return segments;

    segments.reserve(simplified.size() / 2);

    // Step 3: Walk through simplified points — try arc fitting first,
    //         then emit straight-line segments for non-arc sections.
    //         Since RDP already removed collinear points, each pair of
    //         consecutive simplified points IS a valid straight segment.
    int i = 0;
    while (i < simplified.size() - 1) {
        OptimizedSegment arcSeg;
        int pointsConsumed = 0;

        if (GCodeConfig::ENABLE_ARC_FITTING) {
            // Arc fitting needs the ORIGINAL (non-RDP) points for accuracy.
            // Find the range of original points corresponding to
            // simplified[i] .. simplified[i+k]

            // Map simplified points back to indices in 'cleaned'
            // For arc fitting, we work on the cleaned (but un-simplified) data
            // because RDP may have removed points that define the arc curvature.

            // Find where simplified[i] is in 'cleaned'
            int cleanedStart = -1;
            for (int ci = 0; ci < cleaned.size(); ++ci) {
                if (cleaned[ci] == simplified[i]) { cleanedStart = ci; break; }
            }

            if (cleanedStart >= 0 && cleanedStart < cleaned.size() - 2) {
                pointsConsumed = tryFitArcExtended(cleaned, cleanedStart, arcEpsilon, arcSeg);
                if (pointsConsumed >= 3) {
                    segments.append(arcSeg);
                    // Advance i in the simplified list to the point matching arcSeg.end
                    QPointF arcEnd = arcSeg.end;
                    while (i < simplified.size() - 1 && simplified[i] != arcEnd) {
                        // Also check approximate match (floating point)
                        if (pointDistance(simplified[i], arcEnd) < 0.001) break;
                        i++;
                    }
                    if (i < simplified.size() - 1) i++; // Move past arc end
                    else break;
                    continue;
                }
            }
        }

        // No arc found — emit a straight line between consecutive simplified points
        OptimizedSegment seg;
        seg.type = OptimizedSegment::LINE;
        seg.start = simplified[i];
        seg.end = simplified[i + 1];
        seg.arcLength = pointDistance(seg.start, seg.end);
        segments.append(seg);
        i++;
    }

    return postProcessSegments(segments);
}



// ====================================================================
// CONTOUR EXTRACTION FROM IRIT (thread-safe — no renderer calls)
// ====================================================================
static LayerContours extractContoursFromIrit(
    IritPrsrPolygonStruct* contourPolys,
    double modelCenterX, double modelCenterY,
    double bedCenterX, double bedCenterY,
    QList<QPair<QVector3D, QVector3D>>* vizPoints = nullptr,
    double vizZ = 0.0)
{
    LayerContours result;
    result.isConcentricCircles = true;
    result.centerX = 0;
    result.centerY = 0;

    int polyCount = 0;
    for (IritPrsrPolygonStruct* pl = contourPolys; pl != nullptr; pl = pl->Pnext) {
        polyCount++;
        QList<QPointF> poly;
        double minCx = 1e6, maxCx = -1e6, minCy = 1e6, maxCy = -1e6;
        int pointCount = 0;

        for (IritPrsrVertexStruct* v = pl->PVertex; v != nullptr; v = v->Pnext) {
            double bx = v->Coord[0] - modelCenterX + bedCenterX;
            double by = v->Coord[1] - modelCenterY + bedCenterY;
            poly.append(QPointF(bx, by));
            if (bx < minCx) minCx = bx; if (bx > maxCx) maxCx = bx;
            if (by < minCy) minCy = by; if (by > maxCy) maxCy = by;
            pointCount++;

            if (vizPoints && v->Pnext != nullptr) {
                vizPoints->append(qMakePair(
                    QVector3D(v->Coord[0], v->Coord[1], vizZ),
                    QVector3D(v->Pnext->Coord[0], v->Pnext->Coord[1], vizZ)
                ));
            }
        }

        if (pointCount <= 5) result.isConcentricCircles = false;

        if (result.isConcentricCircles) {
            double cX = (minCx + maxCx) / 2.0;
            double cY = (minCy + maxCy) / 2.0;
            double r = (maxCx - minCx) / 2.0;

            for (const auto& pt : poly) {
                double dist = sqrt(pow(pt.x() - cX, 2) + pow(pt.y() - cY, 2));
                if (std::abs(dist - r) > 0.5) { result.isConcentricCircles = false; break; }
            }

            if (polyCount == 1) { result.centerX = cX; result.centerY = cY; }
            else if (std::abs(cX - result.centerX) > 0.5 || std::abs(cY - result.centerY) > 0.5) {
                result.isConcentricCircles = false;
            }
            if (result.isConcentricCircles) result.radii.append(r);
        }

        if (poly.size() > 1 && std::abs(poly.first().x() - poly.last().x()) < 0.001
            && std::abs(poly.first().y() - poly.last().y()) < 0.001)
            poly.removeLast();

        if (poly.size() >= 3) result.allPolygons.append(poly);
    }

    return result;
}




// ====================================================================
// STATISTICS PRINTER
// ====================================================================
static void printStatistics(QTextStream& out,
    double totalPrintTimeSec, int originalPointCount, int optimizedCommandCount,
    int totalG1Commands, int totalG2G3Commands)
{
    int hours = totalPrintTimeSec / 3600;
    int minutes = (static_cast<int>(totalPrintTimeSec) % 3600) / 60;
    int seconds = static_cast<int>(totalPrintTimeSec) % 60;
    QString timeString = QString("%1h %2m %3s").arg(hours).arg(minutes, 2, 10, QChar('0')).arg(seconds, 2, 10, QChar('0'));

    double compressionRatio = (originalPointCount > 0) ?
        (1.0 - (double)optimizedCommandCount / originalPointCount) * 100.0 : 0.0;
    double arcRatio = (optimizedCommandCount > 0) ?
        (double)totalG2G3Commands / optimizedCommandCount * 100.0 : 0.0;

    qDebug() << "   =========================================";
    qDebug() << "   OPTIMIZATION STATISTICS:";
    qDebug() << "   Original points:" << originalPointCount;
    qDebug() << "   Optimized commands:" << optimizedCommandCount;
    qDebug() << "   Compression:" << QString::number(compressionRatio, 'f', 1) << "%";
    qDebug() << "   G1 (line) commands:" << totalG1Commands;
    qDebug() << "   G2/G3 (arc) commands:" << totalG2G3Commands;
    qDebug() << "   Arc ratio:" << QString::number(arcRatio, 'f', 1) << "%";
    qDebug() << "   =========================================";
    qDebug() << "   Estimated Print Time:" << timeString;
    qDebug() << "   =========================================";

    out << "\n; --- SLICING STATISTICS ---\n";
    out << "; Estimated Print Time: " << timeString << "\n";
    out << "; Original points: " << originalPointCount << "\n";
    out << "; Optimized commands: " << optimizedCommandCount << "\n";
    out << "; Compression: " << QString::number(compressionRatio, 'f', 1) << "%\n";
    out << "; G1 commands: " << totalG1Commands << "\n";
    out << "; G2/G3 commands: " << totalG2G3Commands << "\n";
    out << "; Arc ratio: " << QString::number(arcRatio, 'f', 1) << "%\n";
    out << "; --------------------------\n";
}



// ====================================================================
// LAYER SLICING INFRASTRUCTURE — IRIT-SAFE 3-PHASE ARCHITECTURE
// ====================================================================
//
// IRIT's internal functions (IritUserCntrSrfWithPlane, IritPrsrFreePolygonList,
// etc.) use global state and are NOT thread-safe. Calling them from multiple
// threads causes pointer corruption (VList = 0xFFFFFFFFFFFFFFFF crash).
//
// Architecture:
//   PHASE 1 (SEQUENTIAL):  All IRIT calls on main thread only.
//                           Stores pure Qt data (no IRIT pointers survive).
//
//   PHASE 2 (PARALLEL):    Contour smoothing, Clipper2 offsets, RDP,
//                           and arc fitting — fully thread-safe.
//
//   PHASE 3 (SEQUENTIAL):  G-code emission with lastX/lastY tracking.
// ====================================================================

// Pre-processed wall: ready-to-emit segments from Phase 2
struct PreProcessedWall {
    QList<OptimizedSegment> segments;
    QPointF startPoint;
    bool isInnerPerimeter;
    int originalPointCount;
};

struct PreProcessedInfillLine {
    double x1, y1, x2, y2;
    double length;
};

struct PreProcessedLayer {
    int layerIdx;
    double cadZ, machineZ;
    bool hasContours;

    bool isConcentricCircles;
    QList<double> sortedRadii;
    double centerX, centerY;

    QList<PreProcessedWall> walls;

    bool hasSolidFill;
    QList<PreProcessedInfillLine> solidFillLines;

    bool hasGridInfill;
    QList<PreProcessedInfillLine> gridInfillLines;

    // Circle infill mask (for non-solid circle layers)
    bool hasCircleInfillMask;
    double circleInfillROut, circleInfillRIn;

    bool isTopOrBottom;
    QList<QPair<QVector3D, QVector3D>> vizPoints;

    // === SUPPORT-SPECIFIC DATA (processed separately) ===
    bool hasSupport;
    QList<PreProcessedWall> supportWalls;           // 1 perimeter per merged region
    QList<PreProcessedInfillLine> supportInfillLines; // Sparse line fill
    bool isSupportInterface;                         // Dense layer near model
};


// ====================================================================
// PHASE 1: Sequential IRIT slicing (single surface)
// ====================================================================
static QVector<PreSlicedLayer> sequentialSlice_Single(
    CagdSrfStruct* srf,
    const QList<double>& cadHeights,
    double cadBaseZ, double layerHeight, double slicingResolution,
    double modelCenterX, double modelCenterY,
    double bedCenterX, double bedCenterY)
{
    const int N = cadHeights.size();
    QVector<PreSlicedLayer> results(N);
    QElapsedTimer t; t.start();

    for (int i = 0; i < N; ++i) {
        double z = cadHeights[i];
        PreSlicedLayer& L = results[i];
        L.layerIdx = i;
        L.cadZ = z;
        L.machineZ = layerHeight + (z - cadBaseZ);
        L.hasContours = false;
        L.hasSupportContours = false;

        IrtPlnType plane = { 0.0, 0.0, 1.0, -z };
        IritPrsrPolygonStruct* polys = IritUserCntrSrfWithPlane(srf, plane, slicingResolution, 0, 1);
        if (polys) {
            L.contours = extractContoursFromIrit(polys, modelCenterX, modelCenterY,
                bedCenterX, bedCenterY, &L.vizPoints, z);
            L.hasContours = !L.contours.allPolygons.isEmpty();
            IritPrsrFreePolygonList(polys);
        }
        if ((i + 1) % 50 == 0 || i == N - 1)
            qDebug() << "   Phase 1:" << (i + 1) << "/" << N << "layers (" << t.elapsed() << "ms)";
    }
    qDebug() << "   Phase 1 (IRIT slicing) done in" << t.elapsed() << "ms";
    return results;
}

// ====================================================================
// PHASE 1: Sequential IRIT slicing (multi-surface)
// ====================================================================
static QVector<PreSlicedLayer> sequentialSlice_Multi(
    const QList<CagdSrfStruct*>& surfaces,
    const QList<double>& cadHeights,
    double cadBaseZ, double layerHeight, double slicingResolution,
    double modelCenterX, double modelCenterY,
    double bedCenterX, double bedCenterY, double mathEpsilon)
{
    // ================================================================
    // Z-SWEEP ALGORITHM: Active Surface List
    // ================================================================
    // Instead of checking every surface against every layer (O(L×S)),
    // sort surfaces by zMin. As the slicing plane moves up:
    //   - ADD surfaces whose zMin <= currentZ (binary search: O(log S))
    //   - REMOVE surfaces whose zMax < currentZ (amortized O(1) each)
    //   - SLICE only active surfaces (O(A) where A << S)
    //
    // Total: O(S log S) sort + O(L × A) slicing, where A = avg active
    // For 1000 layers × 20000 surfaces with ~5% active: 1M vs 20M checks.
    // ================================================================

    struct SInfo {
        CagdSrfStruct* srf;
        double zMin, zMax;
        int originalIdx; // For stable ordering
    };

    // Step 1: Build surface info and sort by zMin
    std::vector<SInfo> sortedSurfaces;
    sortedSurfaces.reserve(surfaces.size());
    for (int i = 0; i < surfaces.size(); ++i) {
        CagdBBoxStruct B; IritCagdSrfBBox(surfaces[i], &B);
        sortedSurfaces.push_back({ surfaces[i], B.Min[2], B.Max[2], i });
    }
    std::sort(sortedSurfaces.begin(), sortedSurfaces.end(),
        [](const SInfo& a, const SInfo& b) { return a.zMin < b.zMin; });

    const int N = cadHeights.size();
    const int S = sortedSurfaces.size();
    QVector<PreSlicedLayer> results(N);
    QElapsedTimer t; t.start();

    // Step 2: Z-sweep with active surface list
    int nextToAdd = 0;                          // Index into sortedSurfaces
    std::vector<const SInfo*> activeSurfaces;    // Currently active
    activeSurfaces.reserve(std::min(S, 500));    // Typical active count
    int totalSliceCalls = 0;
    int maxActive = 0;

    for (int i = 0; i < N; ++i) {
        double z = cadHeights[i];
        PreSlicedLayer& L = results[i];
        L.layerIdx = i;
        L.cadZ = z;
        L.machineZ = layerHeight + (z - cadBaseZ);
        L.hasContours = false;
        L.hasSupportContours = false;

        LayerContours& merged = L.contours;
        merged.isConcentricCircles = true;
        merged.centerX = merged.centerY = 0;
        bool first = true;
        int totalPoly = 0;

        // ADD: Sweep forward — add surfaces whose zMin <= z + epsilon
        while (nextToAdd < S &&
            sortedSurfaces[nextToAdd].zMin <= z + mathEpsilon) {
            activeSurfaces.push_back(&sortedSurfaces[nextToAdd]);
            nextToAdd++;
        }

        // REMOVE: Evict surfaces whose zMax < z (they're below the plane now)
        // Swap-and-pop for O(1) removal
        for (int j = static_cast<int>(activeSurfaces.size()) - 1; j >= 0; --j) {
            if (activeSurfaces[j]->zMax < z - mathEpsilon * 0.1) {
                activeSurfaces[j] = activeSurfaces.back();
                activeSurfaces.pop_back();
            }
        }

        if (static_cast<int>(activeSurfaces.size()) > maxActive)
            maxActive = static_cast<int>(activeSurfaces.size());

        // SLICE: Only active surfaces (typically 5-15% of total)
        for (const auto* s : activeSurfaces) {
            IrtPlnType plane = { 0.0, 0.0, 1.0, -z };
            IritPrsrPolygonStruct* polys = IritUserCntrSrfWithPlane(
                s->srf, plane, slicingResolution, 0, 1);
            totalSliceCalls++;
            if (!polys) continue;

            LayerContours c = extractContoursFromIrit(polys, modelCenterX, modelCenterY,
                bedCenterX, bedCenterY, &L.vizPoints, z);
            merged.allPolygons.append(c.allPolygons);
            if (!c.isConcentricCircles) { merged.isConcentricCircles = false; }
            else {
                merged.radii.append(c.radii);
                if (first) { merged.centerX = c.centerX; merged.centerY = c.centerY; first = false; }
                else if (std::abs(c.centerX - merged.centerX) > 0.5 ||
                    std::abs(c.centerY - merged.centerY) > 0.5) merged.isConcentricCircles = false;
            }
            totalPoly += c.allPolygons.size();
            IritPrsrFreePolygonList(polys);
        }
        L.hasContours = (totalPoly > 0);

        if ((i + 1) % 50 == 0 || i == N - 1)
            qDebug() << "   Phase 1:" << (i + 1) << "/" << N
            << "layers, active:" << activeSurfaces.size()
            << "(" << t.elapsed() << "ms)";
    }
    qDebug() << "   Phase 1 (Z-sweep multi-surface) done in" << t.elapsed() << "ms";
    qDebug() << "     Total IRIT slice calls:" << totalSliceCalls
        << "(vs brute-force:" << (N * S) << ")";
    qDebug() << "     Max active surfaces at any layer:" << maxActive
        << "(of" << S << "total)";
    return results;
}


// ====================================================================
// PHASE 2: Parallel contour processing (smoothing, offset, RDP, arcs)
// No IRIT pointers — fully thread-safe.
// ====================================================================
static QVector<PreProcessedLayer> parallelProcess(
    const QVector<PreSlicedLayer>& sliced,
    double extrusionWidth, double eMultiplier, double layerHeight,
    bool printSolidRoof, double startZ, double maxZ)
{
    const int N = sliced.size();
    QVector<PreProcessedLayer> out(N);
    QElapsedTimer t; t.start();

    int nThreads = QThread::idealThreadCount();
    qDebug() << "   Phase 2: parallel processing with" << nThreads << "threads";

    QVector<int> idx(N);
    std::iota(idx.begin(), idx.end(), 0);

    QtConcurrent::blockingMap(idx, [&](int li) {
        const PreSlicedLayer& sl = sliced[li];
        PreProcessedLayer& pp = out[li];
        pp.layerIdx = sl.layerIdx;
        pp.cadZ = sl.cadZ;
        pp.machineZ = sl.machineZ;
        pp.hasContours = sl.hasContours;
        pp.vizPoints = sl.vizPoints;
        pp.hasSolidFill = false;
        pp.hasGridInfill = false;
        pp.hasCircleInfillMask = false;
        pp.hasSupport = false;
        pp.isSupportInterface = false;

        // Skip only if we have NEITHER model contours NOR support contours
        if (!sl.hasContours && !sl.hasSupportContours) return;

        // --- MODEL CONTOUR PROCESSING ---
        if (sl.hasContours) {
            const LayerContours& c = sl.contours;
            bool isSolidFloor = (sl.cadZ <= startZ + layerHeight * GCodeConfig::NUM_SOLID_FLOOR_LAYERS);
            bool isSolidRoof = printSolidRoof && (sl.cadZ >= maxZ - layerHeight * GCodeConfig::NUM_SOLID_ROOF_LAYERS);
            pp.isTopOrBottom = isSolidFloor || isSolidRoof;
            pp.isConcentricCircles = c.isConcentricCircles;
            pp.centerX = c.centerX;
            pp.centerY = c.centerY;

            // Circle layers: just sort radii
            if (c.isConcentricCircles && !c.radii.isEmpty()) {
                pp.sortedRadii = c.radii;
                std::sort(pp.sortedRadii.begin(), pp.sortedRadii.end(),
                    [](double a, double b) { return a > b; });
                // Don't return here — still need to process support below
            }
            else {
                auto offsetPoly = [&](const QList<QPointF>& poly, double d) {
                    Clipper2Lib::PathsD ps; Clipper2Lib::PathD p;
                    for (const auto& pt : poly) p.push_back({ pt.x(), pt.y() });
                    ps.push_back(p);
                    auto sol = Clipper2Lib::InflatePaths(ps, -d, Clipper2Lib::JoinType::Round, Clipper2Lib::EndType::Polygon);
                    QList<QPointF> r;
                    if (!sol.empty()) for (const auto& pt : sol[0]) r.append(QPointF(pt.x, pt.y));
                    return r;
                    };

                int numPerimeters = 3;
                Clipper2Lib::PathsD innerClipMask;
                QList<QList<QPointF>> innermostPolys;

                for (const auto& poly : c.allPolygons) {
                    QList<QPointF> smoothed = smoothContour(poly, 2);

                    for (int p = 0; p < numPerimeters; ++p) {
                        double offset = p * extrusionWidth;
                        QList<QPointF> wall = (p == 0) ? smoothed : offsetPoly(smoothed, offset);
                        if (wall.size() < 3) continue;

                        PreProcessedWall pw;
                        pw.originalPointCount = wall.size();
                        pw.isInnerPerimeter = (p > 0);

                        // Seam hiding
                        if (wall.size() > 3) {
                            int best = 0; double bestS = -1e9;
                            QPointF tgt(10000.0, 10000.0);
                            for (int w = 0; w < wall.size(); ++w) {
                                int prev = (w == 0) ? wall.size() - 1 : w - 1;
                                int next = (w + 1) % wall.size();
                                QPointF v1(wall[w].x() - wall[prev].x(), wall[w].y() - wall[prev].y());
                                QPointF v2(wall[next].x() - wall[w].x(), wall[next].y() - wall[w].y());
                                double l1 = sqrt(v1.x() * v1.x() + v1.y() * v1.y());
                                double l2 = sqrt(v2.x() * v2.x() + v2.y() * v2.y());
                                double as = 0;
                                if (l1 > 1e-6 && l2 > 1e-6) {
                                    double d = std::max(-1.0, std::min(1.0, (v1.x() * v2.x() + v1.y() * v2.y()) / (l1 * l2)));
                                    as = 1.0 - d;
                                }
                                double dt = sqrt(pow(wall[w].x() - tgt.x(), 2) + pow(wall[w].y() - tgt.y(), 2));
                                double sc = as * 10000.0 - dt;
                                if (sc > bestS) { bestS = sc; best = w; }
                            }
                            if (best > 0) {
                                QList<QPointF> rot; rot.reserve(wall.size());
                                for (int w = best; w < wall.size(); ++w) rot.append(wall[w]);
                                for (int w = 0; w < best; ++w) rot.append(wall[w]);
                                wall = rot;
                            }
                        }

                        pw.startPoint = wall[0];

                        // Close + overlap
                        QList<QPointF> closed = wall;
                        closed.append(wall[0]);
                        if (wall.size() > 2) {
                            double dx = wall[1].x() - wall[0].x(), dy = wall[1].y() - wall[0].y();
                            double len = sqrt(dx * dx + dy * dy);
                            if (len > 0.01) {
                                double t = std::min(0.3 / len, 0.5);
                                closed.append(QPointF(wall[0].x() + dx * t, wall[0].y() + dy * t));
                            }
                        }

                        // RDP + arc fitting
                        if (GCodeConfig::ENABLE_PATH_OPTIMIZATION) {
                            pw.segments = optimizePathCurvature(closed,
                                GCodeConfig::LINE_COLLINEARITY_EPSILON,
                                GCodeConfig::ARC_FITTING_EPSILON);
                        }
                        else {
                            for (int k = 0; k < closed.size() - 1; ++k) {
                                OptimizedSegment s;
                                s.type = OptimizedSegment::LINE;
                                s.start = closed[k]; s.end = closed[k + 1];
                                s.arcLength = pointDistance(s.start, s.end);
                                pw.segments.append(s);
                            }
                        }

                        pp.walls.append(pw);

                        if (p == numPerimeters - 1) {
                            innermostPolys.append(wall);
                            Clipper2Lib::PathD cp;
                            for (auto& pt : wall) cp.push_back({ pt.x(), pt.y() });
                            innerClipMask.push_back(cp);
                        }
                    }
                }

                // Solid fill for top/bottom
                if (pp.isTopOrBottom && !innermostPolys.isEmpty()) {
                    pp.hasSolidFill = true;
                    double sp = extrusionWidth * GCodeConfig::SOLID_FILL_LINE_SPACING_FACTOR;
                    double bo = extrusionWidth * GCodeConfig::SOLID_FILL_BOUNDARY_OVERLAP;

                    double mnX = 1e6, mxX = -1e6;
                    for (const auto& poly : innermostPolys)
                        for (const auto& pt : poly) { if (pt.x() < mnX) mnX = pt.x(); if (pt.x() > mxX) mxX = pt.x(); }

                    for (double sx = mnX + (sp / 2.0); sx <= mxX - (sp / 2.0); sx += sp) {
                        QList<double> ints;
                        for (const auto& poly : innermostPolys)
                            for (int ii = 0; ii < poly.size(); ++ii) {
                                QPointF p1 = poly[ii], p2 = poly[(ii + 1) % poly.size()];
                                if ((p1.x() < sx && p2.x() >= sx) || (p2.x() < sx && p1.x() >= sx))
                                    ints.append(p1.y() + (p2.y() - p1.y()) * (sx - p1.x()) / (p2.x() - p1.x()));
                            }
                        std::sort(ints.begin(), ints.end());
                        for (int ii = 0; ii + 1 < ints.size(); ii += 2) {
                            PreProcessedInfillLine l;
                            l.x1 = sx; l.y1 = ints[ii] - bo; l.x2 = sx; l.y2 = ints[ii + 1] + bo;
                            l.length = std::abs(l.y2 - l.y1);
                            pp.solidFillLines.append(l);
                        }
                    }
                }

                // Grid infill for middle layers
                if (!pp.isTopOrBottom && printSolidRoof && !innerClipMask.empty()) {
                    double inf = GCodeConfig::INFILL_SPACING;
                    double imnX = 1e6, imxX = -1e6, imnY = 1e6, imxY = -1e6;
                    for (const auto& pa : innerClipMask)
                        for (const auto& pt : pa) {
                            if (pt.x < imnX) imnX = pt.x; if (pt.x > imxX) imxX = pt.x;
                            if (pt.y < imnY) imnY = pt.y; if (pt.y > imxY) imxY = pt.y;
                        }
                    Clipper2Lib::PathsD raw;
                    for (double y = imnY; y <= imxY; y += inf) {
                        Clipper2Lib::PathD h; h.push_back({ imnX,y }); h.push_back({ imxX,y }); raw.push_back(h);
                    }
                    for (double x = imnX; x <= imxX; x += inf) {
                        Clipper2Lib::PathD v; v.push_back({ x,imnY }); v.push_back({ x,imxY }); raw.push_back(v);
                    }
                    Clipper2Lib::ClipperD cl;
                    cl.AddOpenSubject(raw);
                    cl.AddClip(innerClipMask);
                    Clipper2Lib::PathsD dc, trimmed;
                    cl.Execute(Clipper2Lib::ClipType::Intersection, Clipper2Lib::FillRule::EvenOdd, dc, trimmed);

                    pp.hasGridInfill = true;
                    for (const auto& pa : trimmed) {
                        if (pa.size() < 2) continue;
                        PreProcessedInfillLine l;
                        l.x1 = pa[0].x; l.y1 = pa[0].y; l.x2 = pa[pa.size() - 1].x; l.y2 = pa[pa.size() - 1].y;
                        l.length = sqrt(pow(l.x2 - l.x1, 2) + pow(l.y2 - l.y1, 2));
                        if (l.length >= GCodeConfig::MIN_INFILL_SEGMENT_LENGTH)
                            pp.gridInfillLines.append(l);
                    }
                }
            } // end else (complex shapes)
        } // end if (sl.hasContours)

        // ============================================================
        // SUPPORT CONTOUR PROCESSING — direct emission, no Clipper2
        // ============================================================
        // The microstructure tiles ARE the support pattern. Just emit
        // the raw contours as G1 moves — no union, no offset, no infill.
        pp.hasSupport = false;
        pp.isSupportInterface = false;

        if (sl.hasSupportContours && !sl.supportPolygons.isEmpty()) {
            for (const auto& poly : sl.supportPolygons) {
                if (poly.size() < 3) continue;

                // Skip tiny contours (noise from degenerate triangle intersections)
                double perim = 0;
                for (int k = 0; k < poly.size() - 1; ++k)
                    perim += pointDistance(poly[k], poly[k + 1]);
                if (perim < 1.0) continue; // Skip contours smaller than 1mm

                QList<QPointF> wall = poly;
                // Close the contour if not already closed
                if (pointDistance(wall.first(), wall.last()) > 0.01)
                    wall.append(wall.first());

                PreProcessedWall pw;
                pw.originalPointCount = wall.size();
                pw.isInnerPerimeter = false;
                pw.startPoint = wall[0];

                // Simple line segments — no arc fitting needed for support
                for (int k = 0; k < wall.size() - 1; ++k) {
                    OptimizedSegment s;
                    s.type = OptimizedSegment::LINE;
                    s.start = wall[k]; s.end = wall[k + 1];
                    s.arcLength = pointDistance(s.start, s.end);
                    if (s.arcLength > 0.05) // Skip micro-segments
                        pw.segments.append(s);
                }

                if (!pw.segments.isEmpty()) {
                    pp.supportWalls.append(pw);
                    pp.hasSupport = true;
                }
            }
        }
        });

    qDebug() << "   Phase 2 (parallel processing) done in" << t.elapsed() << "ms";
    return out;
}


// ====================================================================
// PHASE 3: Sequential G-code emission
// ====================================================================
static void emitGCode(
    const QVector<PreProcessedLayer>& layers,
    double extrusionWidth, double eMultiplier, double layerHeight,
    double& lastX, double& lastY, double& totalPrintTimeSec,
    int& totalG1Commands, int& totalG2G3Commands,
    int& originalPointCount, int& optimizedCommandCount)
{
    auto addTime = [&](double d, double f) { if (f > 0) totalPrintTimeSec += d / (f / 60.0); };

    auto smartMove = [&](double x, double y, bool noRetract, QTextStream& o) {
        if (lastX == -1000.0) {
            o << "G0 X" << x << " Y" << y << " F9000\n";
            o << "G1 E0.8 F2400 ; Prime\n";
            addTime(0.8, 1800.0);
        }
        else {
            double d = sqrt(pow(x - lastX, 2) + pow(y - lastY, 2));
            if (d > GCodeConfig::RETRACT_THRESHOLD && !noRetract) {
                o << "G1 E-0.6 F2400 ; Retract\n";
                o << "G0 X" << x << " Y" << y << " F9000\n";
                o << "G1 E0.6 F2400 ; Prime\n";
                addTime(0.6, 1800.0); addTime(d, 9000.0); addTime(0.6, 1800.0);
            }
            else {
                o << "G0 X" << x << " Y" << y << " F9000\n"; addTime(d, 9000.0);
            }
        }
        lastX = x; lastY = y;
        };

    auto emitSegs = [&](const QList<OptimizedSegment>& segs, QTextStream& o) {
        for (const auto& s : segs) {
            optimizedCommandCount++;
            if (s.type == OptimizedSegment::LINE) {
                o << "G1 X" << s.end.x() << " Y" << s.end.y()
                    << " E" << (s.arcLength * eMultiplier) << " F2400\n";
                addTime(s.arcLength, 2400.0); totalG1Commands++;
            }
            else {
                double I = s.centerX - lastX, J = s.centerY - lastY;
                o << (s.type == OptimizedSegment::ARC_CW ? "G2" : "G3")
                    << " X" << s.end.x() << " Y" << s.end.y()
                    << " I" << I << " J" << J
                    << " E" << (s.arcLength * eMultiplier) << " F2400\n";
                addTime(s.arcLength, 2400.0); totalG2G3Commands++;
            }
            lastX = s.end.x(); lastY = s.end.y();
        }
        };

    for (const auto& pp : layers) {
        if (!pp.hasContours && !pp.hasSupport) continue;

        QString layerStr;
        QTextStream lo(&layerStr);
        lo.setRealNumberNotation(QTextStream::FixedNotation);
        lo.setRealNumberPrecision(5);

        lo << "; --- CAD Z: " << pp.cadZ << " | Machine Z: " << pp.machineZ << " ---\n";
        lo << "G1 Z" << pp.machineZ << " F1200\n";
        addTime(layerHeight, 1200.0);

        // Circle perimeters
        if (pp.isConcentricCircles && !pp.sortedRadii.isEmpty()) {
            int numP = 3;
            for (int i = 0; i < pp.sortedRadii.size(); i += 2) {
                double rO = pp.sortedRadii[i];
                double rI = (i + 1 < pp.sortedRadii.size()) ? pp.sortedRadii[i + 1] : 0.0;
                double cx = pp.centerX, cy = pp.centerY;

                auto circle = [&](double r) {
                    if (r < 0.2) return;
                    double sx = cx + r;
                    smartMove(sx, cy, false, lo);
                    lo << "G3 X" << sx << " Y" << cy << " I" << (-r)
                        << " J0.00000 E" << ((2 * M_PI * r) * eMultiplier) << " F2400\n";
                    addTime(2 * M_PI * r, 1800.0);
                    lastX = sx; lastY = cy;
                    totalG2G3Commands++; optimizedCommandCount++;
                    };

                if (pp.isTopOrBottom) {
                    for (double r = rO; r >= rI && r > 0.2; r -= extrusionWidth) circle(r);
                    if (rI > 0.2) circle(rI);
                }
                else {
                    for (int p = 0; p < numP; p++) {
                        double r = rO - p * extrusionWidth;
                        if (r > rI + numP * extrusionWidth || rI < 0.2) circle(r);
                    }
                    if (rI > 0.2) for (int p = 0; p < numP; p++) circle(rI + p * extrusionWidth);
                }
            }
        }
        // Complex walls — sorted by nearest-neighbor to minimize travel
        else {
            // Build index of unprinted walls
            std::vector<int> wallOrder;
            wallOrder.reserve(pp.walls.size());
            std::vector<bool> wallPrinted(pp.walls.size(), false);

            double curX = lastX, curY = lastY;

            for (int wi = 0; wi < pp.walls.size(); ++wi) {
                // Find nearest unprinted wall to current nozzle position
                int bestIdx = -1;
                double bestDist = 1e9;

                for (int wj = 0; wj < pp.walls.size(); ++wj) {
                    if (wallPrinted[wj]) continue;
                    const auto& w = pp.walls[wj];
                    double dx = w.startPoint.x() - curX;
                    double dy = w.startPoint.y() - curY;
                    double d = dx * dx + dy * dy;

                    // Inner perimeters near current position get priority
                    // (keeps them with their outer perimeter)
                    if (w.isInnerPerimeter && d < extrusionWidth * extrusionWidth * 16.0)
                        d *= 0.1; // 10x priority boost

                    if (d < bestDist) { bestDist = d; bestIdx = wj; }
                }

                if (bestIdx < 0) break;
                wallPrinted[bestIdx] = true;
                wallOrder.push_back(bestIdx);

                // Update position to where this wall ends
                const auto& w = pp.walls[bestIdx];
                if (!w.segments.isEmpty()) {
                    curX = w.segments.last().end.x();
                    curY = w.segments.last().end.y();
                }
                else {
                    curX = w.startPoint.x();
                    curY = w.startPoint.y();
                }
            }

            // Emit walls in sorted order
            for (int idx : wallOrder) {
                const auto& w = pp.walls[idx];
                originalPointCount += w.originalPointCount;
                double t = (lastX > -999.0) ?
                    sqrt(pow(w.startPoint.x() - lastX, 2) + pow(w.startPoint.y() - lastY, 2)) : 1e6;
                bool skip = w.isInnerPerimeter && (t < extrusionWidth * 4.0);
                smartMove(w.startPoint.x(), w.startPoint.y(), skip, lo);
                emitSegs(w.segments, lo);
            }

            // Solid fill
            if (pp.hasSolidFill) {
                bool alt = true;
                for (const auto& l : pp.solidFillLines) {
                    double sx, sy, ex, ey;
                    if (alt) { sx = l.x1; sy = l.y1; ex = l.x2; ey = l.y2; }
                    else { sx = l.x2; sy = l.y2; ex = l.x1; ey = l.y1; }
                    smartMove(sx, sy, false, lo);
                    lo << "G1 X" << ex << " Y" << ey << " E" << (l.length * eMultiplier) << " F2400\n";
                    addTime(l.length, 1800.0);
                    lastX = ex; lastY = ey;
                    totalG1Commands++; optimizedCommandCount++;
                    alt = !alt;
                }
            }
        }

        // Grid infill (sorted by nearest-neighbor using current lastX/lastY)
        if (pp.hasGridInfill && !pp.gridInfillLines.isEmpty()) {
            struct IS { double x1, y1, x2, y2, len; bool used; };
            std::vector<IS> sg;
            sg.reserve(pp.gridInfillLines.size());
            for (const auto& l : pp.gridInfillLines) sg.push_back({ l.x1,l.y1,l.x2,l.y2,l.length,false });

            for (size_t i = 0; i < sg.size(); ++i) {
                int bi = -1; double bd = 1e9; bool rev = false;
                for (size_t j = 0; j < sg.size(); ++j) {
                    if (sg[j].used) continue;
                    double m = std::min(std::abs(sg[j].x1 - lastX) + std::abs(sg[j].y1 - lastY),
                        std::abs(sg[j].x2 - lastX) + std::abs(sg[j].y2 - lastY));
                    if (m > bd * 1.42) continue;
                    double d1 = (sg[j].x1 - lastX) * (sg[j].x1 - lastX) + (sg[j].y1 - lastY) * (sg[j].y1 - lastY);
                    double d2 = (sg[j].x2 - lastX) * (sg[j].x2 - lastX) + (sg[j].y2 - lastY) * (sg[j].y2 - lastY);
                    if (d1 < bd) { bd = d1; bi = static_cast<int>(j); rev = false; }
                    if (d2 < bd) { bd = d2; bi = static_cast<int>(j); rev = true; }
                }
                if (bi < 0) break;
                sg[bi].used = true;
                double sx, sy, ex, ey;
                if (rev) { sx = sg[bi].x2; sy = sg[bi].y2; ex = sg[bi].x1; ey = sg[bi].y1; }
                else { sx = sg[bi].x1; sy = sg[bi].y1; ex = sg[bi].x2; ey = sg[bi].y2; }
                double td = sqrt(pow(sx - lastX, 2) + pow(sy - lastY, 2));
                smartMove(sx, sy, (td < GCodeConfig::RETRACT_THRESHOLD * 2.0), lo);
                lo << "G1 X" << ex << " Y" << ey << " E" << (sg[bi].len * eMultiplier) << " F2400\n";
                addTime(sg[bi].len, 2400.0);
                lastX = ex; lastY = ey;
                totalG1Commands++; optimizedCommandCount++;
            }
        }

        // === SUPPORT G-CODE EMISSION (separate flow & speed) ===
        if (pp.hasSupport) {
            double supportEMul = eMultiplier * GCodeConfig::SUPPORT_FLOW_MULTIPLIER;
            double supportSpeed = GCodeConfig::SUPPORT_PRINT_SPEED;

            lo << "; --- SUPPORT ---\n";

            // Sort support walls by nearest-neighbor
            std::vector<int> swOrder;
            swOrder.reserve(pp.supportWalls.size());
            std::vector<bool> swPrinted(pp.supportWalls.size(), false);
            double swX = lastX, swY = lastY;

            for (int si = 0; si < pp.supportWalls.size(); ++si) {
                int best = -1; double bd = 1e9;
                for (int sj = 0; sj < pp.supportWalls.size(); ++sj) {
                    if (swPrinted[sj]) continue;
                    double dx = pp.supportWalls[sj].startPoint.x() - swX;
                    double dy = pp.supportWalls[sj].startPoint.y() - swY;
                    double d = dx * dx + dy * dy;
                    if (d < bd) { bd = d; best = sj; }
                }
                if (best < 0) break;
                swPrinted[best] = true;
                swOrder.push_back(best);
                const auto& w = pp.supportWalls[best];
                if (!w.segments.isEmpty()) { swX = w.segments.last().end.x(); swY = w.segments.last().end.y(); }
                else { swX = w.startPoint.x(); swY = w.startPoint.y(); }
            }

            for (int idx : swOrder) {
                const auto& w = pp.supportWalls[idx];
                originalPointCount += w.originalPointCount;
                smartMove(w.startPoint.x(), w.startPoint.y(), false, lo);
                for (const auto& s : w.segments) {
                    optimizedCommandCount++;
                    if (s.type == OptimizedSegment::LINE) {
                        lo << "G1 X" << s.end.x() << " Y" << s.end.y()
                            << " E" << (s.arcLength * supportEMul)
                            << " F" << supportSpeed << "\n";
                        addTime(s.arcLength, supportSpeed);
                        totalG1Commands++;
                    }
                    else {
                        double I = s.centerX - lastX, J = s.centerY - lastY;
                        lo << (s.type == OptimizedSegment::ARC_CW ? "G2" : "G3")
                            << " X" << s.end.x() << " Y" << s.end.y()
                            << " I" << I << " J" << J
                            << " E" << (s.arcLength * supportEMul)
                            << " F" << supportSpeed << "\n";
                        addTime(s.arcLength, supportSpeed);
                        totalG2G3Commands++;
                    }
                    lastX = s.end.x(); lastY = s.end.y();
                }
            }

            lo << "; --- END SUPPORT ---\n";
        }

        globalZBuffer[pp.machineZ] += layerStr;
    }
}


// ====================================================================
// SINGLE-SURFACE G-CODE GENERATION
// ====================================================================
void GenerateDirectGCode(CagdSrfStruct* srf, IritRenderer* renderer, QTextStream& out, bool printSolidRoof) {
    QElapsedTimer timer; timer.start();
    qDebug() << "   Direct Slicing (3-phase: seq IRIT + parallel process + seq emit)";

    double totalPrintTimeSec = 0;
    int totalG1 = 0, totalG2G3 = 0, origPts = 0, optCmds = 0;

    out.setRealNumberNotation(QTextStream::FixedNotation);
    out.setRealNumberPrecision(5);

    CagdBBoxStruct BBox; IritCagdSrfBBox(srf, &BBox);
    double minZ = BBox.Min[2], maxZ = BBox.Max[2];
    double modelCX = 0, modelCY = 0, bedCX = 100.0, bedCY = 100.0;
    double lh = 0.2, res = 500, filD = 1.75, ew = 0.42;
    double eMul = (ew * lh) / (M_PI * pow(filD / 2.0, 2));

    QList<double> heights;
    double baseOff = 0.4, eps = 0.000137;
    double startZ = minZ + baseOff + eps;
    if (startZ > maxZ) startZ = minZ + 0.005;
    for (double z = startZ; z < maxZ; z += lh) heights.append(z);
    heights.append(maxZ - 0.005);
    double cadBaseZ = heights.isEmpty() ? minZ : heights.first();

    // PHASE 1
    auto sliced = sequentialSlice_Single(srf, heights, cadBaseZ, lh, res, modelCX, modelCY, bedCX, bedCY);

    // PHASE 2
    auto processed = parallelProcess(sliced, ew, eMul, lh, printSolidRoof, startZ, maxZ);

    // Free sliced data (contours already copied into processed)
    sliced.clear();

    // PHASE 3
    QElapsedTimer p3; p3.start();
    double lastX = -1000.0, lastY = -1000.0;
    QList<QPair<QVector3D, QVector3D>> allViz;
    for (const auto& pp : processed) allViz.append(pp.vizPoints);

    emitGCode(processed, ew, eMul, lh, lastX, lastY, totalPrintTimeSec,
        totalG1, totalG2G3, origPts, optCmds);
    qDebug() << "   Phase 3 (G-code emit) done in" << p3.elapsed() << "ms";

    for (const auto& pair : allViz) {
        renderer->addContourPoint(pair.first.x(), pair.first.y(), pair.first.z());
        renderer->addContourPoint(pair.second.x(), pair.second.y(), pair.second.z());
    }

    printStatistics(out, totalPrintTimeSec, origPts, optCmds, totalG1, totalG2G3);
    qDebug() << "   Total slicing completed in" << timer.elapsed() << "ms";
}


// ====================================================================
// MULTI-SURFACE BATCH G-CODE GENERATION
// ====================================================================
void GenerateDirectGCodeMultiSurface(
    const QList<CagdSrfStruct*>& surfaces,
    IritRenderer* renderer, QTextStream& out, bool printSolidRoof)
{
    QElapsedTimer timer; timer.start();
    if (surfaces.isEmpty()) return;

    qDebug() << "   Multi-Surface Slicing (3-phase, " << surfaces.size() << " surfaces)";

    double totalPrintTimeSec = 0;
    int totalG1 = 0, totalG2G3 = 0, origPts = 0, optCmds = 0;

    out.setRealNumberNotation(QTextStream::FixedNotation);
    out.setRealNumberPrecision(5);

    double gMinX = 1e9, gMaxX = -1e9, gMinY = 1e9, gMaxY = -1e9, gMinZ = 1e9, gMaxZ = -1e9;
    for (auto* s : surfaces) {
        CagdBBoxStruct B; IritCagdSrfBBox(s, &B);
        if (B.Min[0] < gMinX) gMinX = B.Min[0]; if (B.Max[0] > gMaxX) gMaxX = B.Max[0];
        if (B.Min[1] < gMinY) gMinY = B.Min[1]; if (B.Max[1] > gMaxY) gMaxY = B.Max[1];
        if (B.Min[2] < gMinZ) gMinZ = B.Min[2]; if (B.Max[2] > gMaxZ) gMaxZ = B.Max[2];
    }

    double modelCX = 0, modelCY = 0, bedCX = 100.0, bedCY = 100.0;
    double lh = 0.2, res = 150, filD = 1.75, ew = 0.42;
    double eMul = (ew * lh) / (M_PI * pow(filD / 2.0, 2));

    QList<double> heights;
    double baseOff = 0.4, eps = 0.000137;
    double startZ = gMinZ + baseOff + eps;
    if (startZ > gMaxZ) startZ = gMinZ + 0.005;
    for (double z = startZ; z < gMaxZ; z += lh) heights.append(z);
    heights.append(gMaxZ - 0.005);
    double cadBaseZ = heights.isEmpty() ? gMinZ : heights.first();

    qDebug() << "   Z range:" << gMinZ << "to" << gMaxZ << "| Layers:" << heights.size();

    // PHASE 1 (sequential — IRIT is not thread-safe)
    auto sliced = sequentialSlice_Multi(surfaces, heights, cadBaseZ, lh, res,
        modelCX, modelCY, bedCX, bedCY, eps);

    // PHASE 2 (parallel — only Qt/Clipper2, fully thread-safe)
    auto processed = parallelProcess(sliced, ew, eMul, lh, printSolidRoof, startZ, gMaxZ);
    sliced.clear();

    // PHASE 3 (sequential — G-code emission with travel optimization)
    QElapsedTimer p3; p3.start();
    double lastX = -1000.0, lastY = -1000.0;
    QList<QPair<QVector3D, QVector3D>> allViz;
    for (const auto& pp : processed) allViz.append(pp.vizPoints);

    emitGCode(processed, ew, eMul, lh, lastX, lastY, totalPrintTimeSec,
        totalG1, totalG2G3, origPts, optCmds);
    qDebug() << "   Phase 3 (G-code emit) done in" << p3.elapsed() << "ms";

    for (const auto& pair : allViz) {
        renderer->addContourPoint(pair.first.x(), pair.first.y(), pair.first.z());
        renderer->addContourPoint(pair.second.x(), pair.second.y(), pair.second.z());
    }

    printStatistics(out, totalPrintTimeSec, origPts, optCmds, totalG1, totalG2G3);
    qDebug() << "   Total multi-surface slicing in" << timer.elapsed() << "ms ("
        << surfaces.size() << " surfaces)";
}


// ====================================================================
// MODEL + SUPPORT SLICING — NURBS for model, polygon for supports
// ====================================================================
// Phase 1: For each layer:
//   a) Slice model surfaces with IRIT (sequential, slow, but only a few surfaces)
//   b) Slice support mesh with fast polygon slicer (milliseconds)
//   c) Merge support contours into the layer's contour list
//
// Phases 2 & 3 are identical to the model-only path.
// ====================================================================

static QVector<PreSlicedLayer> sequentialSlice_WithSupport(
    const QList<CagdSrfStruct*>& modelSurfaces,
    SupportMesh& supportMesh,
    const QList<double>& cadHeights,
    double cadBaseZ, double layerHeight, double slicingResolution,
    double modelCenterX, double modelCenterY,
    double bedCenterX, double bedCenterY, double mathEpsilon)
{
    // Z-SWEEP for model surfaces
    struct SInfo { CagdSrfStruct* srf; double zMin, zMax; };
    std::vector<SInfo> sortedSurfaces;
    sortedSurfaces.reserve(modelSurfaces.size());
    for (auto* s : modelSurfaces) {
        CagdBBoxStruct B; IritCagdSrfBBox(s, &B);
        sortedSurfaces.push_back({ s, B.Min[2], B.Max[2] });
    }
    std::sort(sortedSurfaces.begin(), sortedSurfaces.end(),
        [](const SInfo& a, const SInfo& b) { return a.zMin < b.zMin; });

    const int N = cadHeights.size();
    const int S = static_cast<int>(sortedSurfaces.size());
    QVector<PreSlicedLayer> results(N);
    QElapsedTimer t; t.start();
    QElapsedTimer iritTime, supportTime;
    qint64 totalIritMs = 0, totalSupportMs = 0;
    int totalSupportContours = 0, totalSliceCalls = 0;

    qDebug() << "   Phase 1: slicing" << modelSurfaces.size()
        << "model surfaces (Z-sweep) +" << supportMesh.count()
        << "support triangles (polygon slicer)";

    int nextToAdd = 0;
    std::vector<const SInfo*> activeSurfaces;
    activeSurfaces.reserve(std::min(S, 500));

    for (int i = 0; i < N; ++i) {
        double z = cadHeights[i];
        PreSlicedLayer& L = results[i];
        L.layerIdx = i;
        L.cadZ = z;
        L.machineZ = layerHeight + (z - cadBaseZ);
        L.hasContours = false;
        L.hasSupportContours = false;

        LayerContours& merged = L.contours;
        merged.isConcentricCircles = false;
        merged.centerX = merged.centerY = 0;
        int totalPoly = 0;

        // --- (a) Z-SWEEP: Slice MODEL surfaces with IRIT ---
        iritTime.start();
        while (nextToAdd < S && sortedSurfaces[nextToAdd].zMin <= z + mathEpsilon) {
            activeSurfaces.push_back(&sortedSurfaces[nextToAdd]);
            nextToAdd++;
        }
        for (int j = static_cast<int>(activeSurfaces.size()) - 1; j >= 0; --j) {
            if (activeSurfaces[j]->zMax < z - mathEpsilon * 0.1) {
                activeSurfaces[j] = activeSurfaces.back();
                activeSurfaces.pop_back();
            }
        }
        for (const auto* s : activeSurfaces) {
            IrtPlnType plane = { 0.0, 0.0, 1.0, -z };
            IritPrsrPolygonStruct* polys = IritUserCntrSrfWithPlane(
                s->srf, plane, slicingResolution, 0, 1);
            totalSliceCalls++;
            if (!polys) continue;
            LayerContours c = extractContoursFromIrit(
                polys, modelCenterX, modelCenterY,
                bedCenterX, bedCenterY, &L.vizPoints, z);
            merged.allPolygons.append(c.allPolygons);
            totalPoly += c.allPolygons.size();
            IritPrsrFreePolygonList(polys);
        }
        totalIritMs += iritTime.elapsed();

        // --- (b) Slice SUPPORT MESH with fast polygon slicer ---
        supportTime.start();
        if (!supportMesh.isEmpty() && z >= supportMesh.globalMinZ && z <= supportMesh.globalMaxZ) {
            QList<QList<QPointF>> supportContours = sliceSupportMeshAtZ(
                supportMesh, z, modelCenterX, modelCenterY, bedCenterX, bedCenterY);
            if (!supportContours.isEmpty()) {
                L.supportPolygons = supportContours;
                L.hasSupportContours = true;
                totalSupportContours += supportContours.size();
                for (const auto& contour : supportContours) {
                    for (int ci = 0; ci < contour.size() - 1; ++ci) {
                        float vx1 = contour[ci].x() - bedCenterX + modelCenterX;
                        float vy1 = contour[ci].y() - bedCenterY + modelCenterY;
                        float vx2 = contour[ci + 1].x() - bedCenterX + modelCenterX;
                        float vy2 = contour[ci + 1].y() - bedCenterY + modelCenterY;
                        L.vizPoints.append(qMakePair(
                            QVector3D(vx1, vy1, z), QVector3D(vx2, vy2, z)));
                    }
                }
            }
        }
        totalSupportMs += supportTime.elapsed();

        L.hasContours = (totalPoly > 0);

        if ((i + 1) % 50 == 0 || i == N - 1)
            qDebug() << "   Phase 1:" << (i + 1) << "/" << N
            << "layers, active:" << activeSurfaces.size()
            << "(" << t.elapsed() << "ms)";
    }

    qDebug() << "   Phase 1 done in" << t.elapsed() << "ms";
    qDebug() << "     IRIT calls:" << totalSliceCalls
        << "(vs brute-force:" << (N * S) << ")";
    qDebug() << "     IRIT (model) slicing:" << totalIritMs << "ms";
    qDebug() << "     Polygon (support) slicing:" << totalSupportMs << "ms";
    qDebug() << "     Support contours generated:" << totalSupportContours;

    // --- AIR GAP: Remove support from the top N layers of each support column ---
    // Only remove support where it's about to TOUCH the model (support-top),
    // not everywhere model exists. A "support top" is a layer that has support
    // but one of the next N layers above has model WITHOUT support.
    int gapLayers = GCodeConfig::SUPPORT_AIR_GAP_LAYERS;
    int gapRemoved = 0;
    for (int i = 0; i < N; ++i) {
        if (!results[i].hasSupportContours) continue;

        // Check if this support layer is near a "support top":
        // any layer within gapLayers above has model but no support
        bool nearModelSurface = false;
        for (int j = i + 1; j <= i + gapLayers && j < N; ++j) {
            if (results[j].hasContours && !results[j].hasSupportContours) {
                nearModelSurface = true;
                break;
            }
        }

        if (nearModelSurface) {
            results[i].supportPolygons.clear();
            results[i].hasSupportContours = false;
            gapRemoved++;
        }
    }
    if (gapRemoved > 0)
        qDebug() << "     Air gap: removed support from" << gapRemoved << "layers at support tops";

    return results;
}


// ====================================================================
// PUBLIC API: Model + Support G-Code Generation
// ====================================================================
void GenerateDirectGCodeWithSupport(
    const QList<CagdSrfStruct*>& modelSurfaces,
    SupportMesh& supportMesh,
    IritRenderer* renderer,
    QTextStream& out,
    bool printSolidRoof)
{
    QElapsedTimer timer; timer.start();
    if (modelSurfaces.isEmpty()) return;

    qDebug() << "   Model+Support Slicing (" << modelSurfaces.size()
        << " model surfaces +" << supportMesh.count() << " support triangles)";

    double totalPrintTimeSec = 0;
    int totalG1 = 0, totalG2G3 = 0, origPts = 0, optCmds = 0;

    out.setRealNumberNotation(QTextStream::FixedNotation);
    out.setRealNumberPrecision(5);

    // Compute unified bounding box (model + support)
    double gMinX = 1e9, gMaxX = -1e9, gMinY = 1e9, gMaxY = -1e9, gMinZ = 1e9, gMaxZ = -1e9;
    for (auto* s : modelSurfaces) {
        CagdBBoxStruct B; IritCagdSrfBBox(s, &B);
        if (B.Min[0] < gMinX) gMinX = B.Min[0]; if (B.Max[0] > gMaxX) gMaxX = B.Max[0];
        if (B.Min[1] < gMinY) gMinY = B.Min[1]; if (B.Max[1] > gMaxY) gMaxY = B.Max[1];
        if (B.Min[2] < gMinZ) gMinZ = B.Min[2]; if (B.Max[2] > gMaxZ) gMaxZ = B.Max[2];
    }
    // Extend Z range to include supports
    if (!supportMesh.isEmpty()) {
       // if (supportMesh.globalMinZ < gMinZ) gMinZ = supportMesh.globalMinZ;
        if (supportMesh.globalMaxZ > gMaxZ) gMaxZ = supportMesh.globalMaxZ;
    }

    double modelCX = 0, modelCY = 0, bedCX = 100.0, bedCY = 100.0;
    double lh = 0.2, res = 150, filD = 1.75, ew = 0.42;
    double eMul = (ew * lh) / (M_PI * pow(filD / 2.0, 2));

    QList<double> heights;
    double baseOff = 0.4, eps = 0.000137;
    double startZ = gMinZ + baseOff + eps;
    if (startZ > gMaxZ) startZ = gMinZ + 0.005;
    for (double z = startZ; z < gMaxZ; z += lh) heights.append(z);
    heights.append(gMaxZ - 0.005);
    double cadBaseZ = heights.isEmpty() ? gMinZ : heights.first();

    qDebug() << "   Z range:" << gMinZ << "to" << gMaxZ << "| Layers:" << heights.size();

    // Build support mesh Z-index
    if (!supportMesh.isEmpty()) {
        supportMesh.buildIndex();
    }

    // PHASE 1: Sequential slicing (IRIT for model, polygon for support)
    auto sliced = sequentialSlice_WithSupport(
        modelSurfaces, supportMesh, heights, cadBaseZ, lh, res,
        modelCX, modelCY, bedCX, bedCY, eps);

    // PHASE 2: Parallel contour processing (smoothing, offset, RDP, arcs)
    auto processed = parallelProcess(sliced, ew, eMul, lh, printSolidRoof, startZ, gMaxZ);
    sliced.clear();

    // PHASE 3: Sequential G-code emission
    QElapsedTimer p3; p3.start();
    double lastX = -1000.0, lastY = -1000.0;
    QList<QPair<QVector3D, QVector3D>> allViz;
    for (const auto& pp : processed) allViz.append(pp.vizPoints);

    emitGCode(processed, ew, eMul, lh, lastX, lastY, totalPrintTimeSec,
        totalG1, totalG2G3, origPts, optCmds);
    qDebug() << "   Phase 3 (G-code emit) done in" << p3.elapsed() << "ms";

    for (const auto& pair : allViz) {
        renderer->addContourPoint(pair.first.x(), pair.first.y(), pair.first.z());
        renderer->addContourPoint(pair.second.x(), pair.second.y(), pair.second.z());
    }

    printStatistics(out, totalPrintTimeSec, origPts, optCmds, totalG1, totalG2G3);
    qDebug() << "   Total model+support slicing in" << timer.elapsed() << "ms";
}
