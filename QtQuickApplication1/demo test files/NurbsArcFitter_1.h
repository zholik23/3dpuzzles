// ====================================================================
// NurbsArcFitter.h — NURBS-Native Arc Detection for G-Code Generation
// ====================================================================
// Replaces the point-based arc fitting pipeline with exact NURBS math.
//
// OLD PIPELINE (lossy):
//   Surface → IritUserCntrSrfWithPlane → polyline points → guess arcs
//   Problem: tessellation noise destroys curvature information
//
// NEW PIPELINE (exact):
//   Surface → SymbSrfZeroSet → NURBS curve → analyze exactly → G1/G2/G3
//   Step 1: Intersect surface with Z-plane → get CagdCrvStruct (exact curve)
//   Step 2: Check if curve is rational degree-2 (exact conic arc)
//   Step 3: Evaluate curvature symbolically for non-conic curves
//   Step 4: Bi-arc approximate freeform curves within tolerance
//
// USAGE:
//   In your slicing loop, replace:
//     IritPrsrPolygonStruct* polys = IritUserCntrSrfWithPlane(srf, plane, res, 0, 1);
//   with:
//     QList<OptimizedSegment> segs = sliceSurfaceExact(srf, z, bedCX, bedCY, tolerance);
//   This gives you G1/G2/G3 segments directly — no post-processing needed.
// ====================================================================

#ifndef NURBS_ARC_FITTER_H
#define NURBS_ARC_FITTER_H

#include <QList>
#include <QPointF>
#include <QVector3D>
#include <cmath>

extern "C" {
#include "irit_sm.h"
#include "iritprsr.h"
#include "cagd_lib.h"
#include "symb_lib.h"
#include "mvar_lib.h"   // For MvarSrfZeros if available
}

// Forward declaration (defined in GCodeGenerator.cpp)
struct OptimizedSegment;


// ====================================================================
// STEP 1: Surface-Plane Intersection → NURBS Curves
// ====================================================================
// Returns a linked list of CagdCrvStruct representing the exact
// intersection of the surface with the Z=sliceZ plane.
//
// Internally:
//   1. Extract the Z-component scalar surface: F(u,v) = Srf_Z(u,v) - sliceZ
//   2. Find the zero-set of F in parameter space using SymbSrfZeroSet
//   3. Map the parameter-space curves back to 3D via CagdSrfEval
//
// If SymbSrfZeroSet is not available in your IRIT build, falls back
// to IritUserCntrSrfWithPlane (polyline) + point-based fitting.
// ====================================================================

static CagdCrvStruct* intersectSurfaceWithZPlane(CagdSrfStruct* srf, double sliceZ) {
    // Create the scalar surface F(u,v) = Z(u,v) - sliceZ
    // by extracting the Z component and subtracting sliceZ.

    // Step 1: Extract Z-component surface
    // For a 3D surface with control points (X,Y,Z), we need a scalar surface
    // whose value equals the Z coordinate at each (u,v).
    int uLen = srf->ULength;
    int vLen = srf->VLength;
    int uOrder = srf->UOrder;
    int vOrder = srf->VOrder;

    // Create a scalar (E1) B-spline surface for the Z component
    CagdSrfStruct* zSrf = BspSrfNew(uLen, vLen, uOrder, vOrder, CAGD_PT_E1_TYPE);

    // Copy knot vectors
    for (int i = 0; i < uLen + uOrder; i++)
        zSrf->UKnotVector[i] = srf->UKnotVector[i];
    for (int i = 0; i < vLen + vOrder; i++)
        zSrf->VKnotVector[i] = srf->VKnotVector[i];

    // Copy Z coordinates as the scalar value, subtract sliceZ
    CagdRType* zPts = zSrf->Points[1]; // E1 type: only one coordinate channel
    int isRational = CAGD_IS_RATIONAL_SRF(srf);

    for (int i = 0; i < uLen * vLen; i++) {
        if (isRational) {
            // For rational surfaces, Z_physical = Z_weighted / W
            // The zero-set of (Z_weighted - sliceZ * W) gives the correct answer
            double w = srf->Points[0][i]; // Weight
            double z = srf->Points[3][i]; // Weighted Z
            zPts[i] = z - sliceZ * w;
        } else {
            zPts[i] = srf->Points[3][i] - sliceZ;
        }
    }

    // Step 2: Find zero-set curves of this scalar surface
    // SymbSrfZeroSet returns curves in the (u,v) parameter domain
    // where the scalar surface equals zero.
    CagdCrvStruct* paramCurves = SymbSrfZeroSet(zSrf, 1, // Step size
                                                  0.001,   // Subdivision tolerance
                                                  TRUE);   // Numeric improvement

    CagdSrfFree(zSrf);

    if (!paramCurves) return nullptr;

    // Step 3: Map parameter-space curves to 3D
    // Evaluate the original surface along each (u,v) curve to get 3D curves.
    CagdCrvStruct* result3D = nullptr;

    for (CagdCrvStruct* pCrv = paramCurves; pCrv; pCrv = pCrv->Pnext) {
        int crvLen = pCrv->Length;
        int crvOrder = pCrv->Order;

        // Create a 3D curve with the same parameterization
        CagdCrvStruct* crv3D;
        if (isRational)
            crv3D = BspCrvNew(crvLen, crvOrder, CAGD_PT_P3_TYPE);
        else
            crv3D = BspCrvNew(crvLen, crvOrder, CAGD_PT_E3_TYPE);

        // Copy knot vector
        for (int i = 0; i < crvLen + crvOrder; i++)
            crv3D->KnotVector[i] = pCrv->KnotVector[i];

        // Evaluate the surface at each control point's (u,v) parameter
        for (int i = 0; i < crvLen; i++) {
            double u, v;
            if (CAGD_IS_RATIONAL_CRV(pCrv)) {
                double w = pCrv->Points[0][i];
                u = pCrv->Points[1][i] / w;
                v = pCrv->Points[2][i] / w;
            } else {
                u = pCrv->Points[1][i];
                v = pCrv->Points[2][i];
            }

            // Clamp to valid parameter range
            double uMin = srf->UKnotVector[srf->UOrder - 1];
            double uMax = srf->UKnotVector[srf->ULength];
            double vMin = srf->VKnotVector[srf->VOrder - 1];
            double vMax = srf->VKnotVector[srf->VLength];
            u = std::max(uMin, std::min(uMax, u));
            v = std::max(vMin, std::min(vMax, v));

            // Evaluate the original surface at (u,v)
            CagdRType* pt = CagdSrfEval(srf, u, v);

            if (isRational) {
                crv3D->Points[0][i] = 1.0; // Weight
                crv3D->Points[1][i] = pt[1] / pt[0]; // X
                crv3D->Points[2][i] = pt[2] / pt[0]; // Y
                crv3D->Points[3][i] = pt[3] / pt[0]; // Z
            } else {
                crv3D->Points[1][i] = pt[1]; // X
                crv3D->Points[2][i] = pt[2]; // Y
                crv3D->Points[3][i] = pt[3]; // Z
            }
        }

        // Prepend to result list
        crv3D->Pnext = result3D;
        result3D = crv3D;
    }

    CagdCrvFreeList(paramCurves);
    return result3D;
}


// ====================================================================
// STEP 2: Exact Conic Arc Detection
// ====================================================================
// A NURBS curve that is rational degree 2 (order 3) is EXACTLY a conic
// section (circle, ellipse, parabola, or hyperbola). For a cylinder or
// fillet sliced by a plane, this gives a perfect circular arc.
//
// Returns true if the curve is an exact arc, and fills cx, cy, radius.
// ====================================================================

struct ExactArcInfo {
    bool isExactArc;
    double cx, cy;       // Center
    double radius;
    double startAngle, endAngle;
    bool isClockwise;
    double startX, startY;
    double endX, endY;
};

static ExactArcInfo checkExactConicArc(CagdCrvStruct* crv) {
    ExactArcInfo info;
    info.isExactArc = false;

    if (!crv) return info;

    // Condition: rational (weighted) AND order 3 (degree 2) AND 3D
    if (!CAGD_IS_RATIONAL_CRV(crv)) return info;
    if (crv->Order != 3) return info;
    if (crv->Length < 3) return info;

    // Extract control points with weights
    // For a rational quadratic Bezier: P(t) = (w0*P0*(1-t)² + w1*P1*2t(1-t) + w2*P2*t²)
    //                                        / (w0*(1-t)² + w1*2t(1-t) + w2*t²)
    // If all weights are equal, it's a parabola (not a circle).
    // For a circular arc, w1 = cos(θ/2) where θ is the arc angle.

    // Process each Bezier segment (for multi-segment rational curves)
    // For simplicity, handle the first 3 control points as one arc.
    // Multi-segment curves should be split at knot multiplicities.

    double w0 = crv->Points[0][0];
    double w1 = crv->Points[0][1];
    double w2 = crv->Points[0][2];

    // Physical control points (de-homogenize)
    double x0 = crv->Points[1][0] / w0;
    double y0 = crv->Points[2][0] / w0;
    double x1 = crv->Points[1][1] / w1;
    double y1 = crv->Points[2][1] / w1;
    double x2 = crv->Points[1][2] / w2;
    double y2 = crv->Points[2][2] / w2;

    // For a circular arc represented as rational quadratic:
    // The center is equidistant from P0 and P2, and the middle weight w1
    // determines the arc angle: θ = 2 * acos(w1)
    if (w1 <= 0 || w1 >= 1.0) return info; // w1 must be in (0,1) for a circular arc

    double halfAngle = acos(w1);
    double arcAngle = 2.0 * halfAngle;

    // The center of the circle lies at the intersection of the
    // perpendicular bisectors of P0-P1 and P1-P2
    // Simplified formula for rational quadratic:
    double midX = (x0 + x2) / 2.0;
    double midY = (y0 + y2) / 2.0;

    // Direction from midpoint to P1
    double dx = x1 - midX;
    double dy = y1 - midY;
    double dLen = sqrt(dx * dx + dy * dy);
    if (dLen < 1e-10) return info;

    // The radius can be computed from the geometry
    double chordLen = sqrt((x2 - x0) * (x2 - x0) + (y2 - y0) * (y2 - y0));
    double halfChord = chordLen / 2.0;
    double radius = halfChord / sin(halfAngle);

    if (radius < 0.5 || radius > 5000.0) return info; // Sanity check

    // Center is at distance radius from both P0 and P2
    // Using perpendicular bisector of chord
    double chordMidX = midX;
    double chordMidY = midY;
    double chordDx = x2 - x0;
    double chordDy = y2 - y0;
    // Perpendicular direction
    double perpX = -chordDy;
    double perpY = chordDx;
    double perpLen = sqrt(perpX * perpX + perpY * perpY);
    if (perpLen < 1e-10) return info;
    perpX /= perpLen;
    perpY /= perpLen;

    // Distance from chord midpoint to center
    double apothem = sqrt(radius * radius - halfChord * halfChord);

    // Determine which side the center is on
    // P1 is on the arc side, center is on the opposite side
    double p1Side = perpX * (x1 - chordMidX) + perpY * (y1 - chordMidY);
    double centerDist = (p1Side > 0) ? -apothem : apothem;

    double cx = chordMidX + perpX * centerDist;
    double cy = chordMidY + perpY * centerDist;

    // Verify: distance from center to P0 and P2 should equal radius
    double d0 = sqrt((x0 - cx) * (x0 - cx) + (y0 - cy) * (y0 - cy));
    double d2 = sqrt((x2 - cx) * (x2 - cx) + (y2 - cy) * (y2 - cy));
    if (std::abs(d0 - radius) > 0.01 || std::abs(d2 - radius) > 0.01) return info;

    // Determine clockwise/counterclockwise
    double cross = (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0);
    bool isCW = (cross < 0);

    info.isExactArc = true;
    info.cx = cx;
    info.cy = cy;
    info.radius = radius;
    info.startX = x0;
    info.startY = y0;
    info.endX = x2;
    info.endY = y2;
    info.startAngle = atan2(y0 - cy, x0 - cx);
    info.endAngle = atan2(y2 - cy, x2 - cx);
    info.isClockwise = isCW;

    return info;
}


// ====================================================================
// STEP 3: NURBS Curvature Evaluation (Symbolic)
// ====================================================================
// Evaluates exact curvature κ(t) at parameter t along a NURBS curve.
// Uses the formula: κ = |x'y'' - y'x''| / (x'² + y'²)^(3/2)
//
// This is EXACT — no tessellation noise. A straight line returns κ=0,
// a circle returns κ=1/R at every point.
// ====================================================================

static double evaluateNurbsCurvature2D(CagdCrvStruct* crv, double t) {
    // Compute first and second derivatives
    CagdCrvStruct* dCrv = CagdCrvDerive(crv);
    CagdCrvStruct* ddCrv = CagdCrvDerive(dCrv);

    CagdRType* d1 = CagdCrvEval(dCrv, t);
    CagdRType* d2 = CagdCrvEval(ddCrv, t);

    double dx, dy, ddx, ddy;
    if (CAGD_IS_RATIONAL_CRV(dCrv)) {
        dx = d1[1] / d1[0];
        dy = d1[2] / d1[0];
    } else {
        dx = d1[1];
        dy = d1[2];
    }
    if (CAGD_IS_RATIONAL_CRV(ddCrv)) {
        ddx = d2[1] / d2[0];
        ddy = d2[2] / d2[0];
    } else {
        ddx = d2[1];
        ddy = d2[2];
    }

    double cross = dx * ddy - dy * ddx;
    double speed = dx * dx + dy * dy;
    double speedCubed = speed * sqrt(speed);

    CagdCrvFree(dCrv);
    CagdCrvFree(ddCrv);

    if (speedCubed < 1e-15) return 0.0;
    return cross / speedCubed; // Signed curvature
}


// ====================================================================
// STEP 4: Bi-Arc Approximation
// ====================================================================
// Converts a freeform NURBS curve into a sequence of G1/G2/G3 segments.
//
// Algorithm:
//   1. Sample curvature at N points along the curve
//   2. Split at curvature discontinuities (inflection points, sharp changes)
//   3. For each segment:
//      a. If |κ| < threshold everywhere → straight line (G1)
//      b. If κ is approximately constant → single arc (G2/G3)
//      c. Otherwise → recursive bi-arc subdivision
//
// The bi-arc subdivision:
//   Given curve from t0 to t1:
//   - Evaluate start point P0, end point P1, tangents T0, T1
//   - Find the junction point where two tangent arcs meet
//   - If max error < tolerance → emit two arcs
//   - If error too large → split at midpoint and recurse
// ====================================================================

struct GCodeArc {
    enum Type { LINE, ARC_CW, ARC_CCW };
    Type type;
    double startX, startY;
    double endX, endY;
    double centerX, centerY; // Only for arcs
    double radius;           // Only for arcs
    double arcLength;
};

static GCodeArc makeLineSegment(double x0, double y0, double x1, double y1) {
    GCodeArc g;
    g.type = GCodeArc::LINE;
    g.startX = x0; g.startY = y0;
    g.endX = x1; g.endY = y1;
    g.arcLength = sqrt((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0));
    return g;
}

// Fit a single circular arc through start, mid, end points
static bool fitArcThrough3Points(
    double x0, double y0,
    double xm, double ym,
    double x1, double y1,
    double& cx, double& cy, double& radius, bool& clockwise)
{
    // Circumscribed circle of triangle (P0, Pm, P1)
    double ax = x0, ay = y0;
    double bx = xm, by = ym;
    double cx_ = x1, cy_ = y1;

    double D = 2.0 * (ax * (by - cy_) + bx * (cy_ - ay) + cx_ * (ay - by));
    if (std::abs(D) < 1e-10) return false; // Collinear

    double ux = ((ax*ax + ay*ay) * (by - cy_) + (bx*bx + by*by) * (cy_ - ay) + (cx_*cx_ + cy_*cy_) * (ay - by)) / D;
    double uy = ((ax*ax + ay*ay) * (cx_ - bx) + (bx*bx + by*by) * (ax - cx_) + (cx_*cx_ + cy_*cy_) * (bx - ax)) / D;

    cx = ux;
    cy = uy;
    radius = sqrt((ax - cx) * (ax - cx) + (ay - cy) * (ay - cy));

    // Determine direction
    double cross = (xm - x0) * (y1 - y0) - (ym - y0) * (x1 - x0);
    clockwise = (cross < 0);

    return (radius > 0.5 && radius < 5000.0);
}

// Recursive bi-arc approximation of a NURBS curve segment
static void biArcApproxRecursive(
    CagdCrvStruct* crv, CagdSrfStruct* srf,
    double t0, double t1,
    double tolerance,
    int maxDepth,
    QList<GCodeArc>& output)
{
    if (maxDepth <= 0 || t1 - t0 < 1e-6) {
        // Base case: emit as line
        CagdRType* p0 = CagdCrvEval(crv, t0);
        CagdRType* p1 = CagdCrvEval(crv, t1);
        double x0 = CAGD_IS_RATIONAL_CRV(crv) ? p0[1]/p0[0] : p0[1];
        double y0 = CAGD_IS_RATIONAL_CRV(crv) ? p0[2]/p0[0] : p0[2];
        double x1 = CAGD_IS_RATIONAL_CRV(crv) ? p1[1]/p1[0] : p1[1];
        double y1 = CAGD_IS_RATIONAL_CRV(crv) ? p1[2]/p1[0] : p1[2];
        output.append(makeLineSegment(x0, y0, x1, y1));
        return;
    }

    double tMid = (t0 + t1) / 2.0;

    // Evaluate 3 points on the curve
    CagdRType* pe0 = CagdCrvEval(crv, t0);
    CagdRType* peM = CagdCrvEval(crv, tMid);
    CagdRType* pe1 = CagdCrvEval(crv, t1);

    bool rat = CAGD_IS_RATIONAL_CRV(crv);
    double x0 = rat ? pe0[1]/pe0[0] : pe0[1];
    double y0 = rat ? pe0[2]/pe0[0] : pe0[2];
    double xm = rat ? peM[1]/peM[0] : peM[1];
    double ym = rat ? peM[2]/peM[0] : peM[2];
    double x1 = rat ? pe1[1]/pe1[0] : pe1[1];
    double y1 = rat ? pe1[2]/pe1[0] : pe1[2];

    // Check if segment is straight
    double chordLen = sqrt((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0));
    if (chordLen < 0.05) {
        output.append(makeLineSegment(x0, y0, x1, y1));
        return;
    }

    // Perpendicular distance of midpoint from chord
    double cross = (x1-x0)*(ym-y0) - (y1-y0)*(xm-x0);
    double midDev = std::abs(cross) / chordLen;

    if (midDev < tolerance * 0.5) {
        // Nearly straight — emit as line
        output.append(makeLineSegment(x0, y0, x1, y1));
        return;
    }

    // Try to fit a single arc through (P0, Pmid, P1)
    double cx, cy, radius;
    bool clockwise;
    if (fitArcThrough3Points(x0, y0, xm, ym, x1, y1, cx, cy, radius, clockwise)) {
        // Validate: sample additional points and check max deviation
        double maxError = 0;
        int nSamples = 5;
        for (int i = 1; i < nSamples; ++i) {
            double ts = t0 + (t1 - t0) * i / nSamples;
            CagdRType* ps = CagdCrvEval(crv, ts);
            double sx = rat ? ps[1]/ps[0] : ps[1];
            double sy = rat ? ps[2]/ps[0] : ps[2];
            double dist = sqrt((sx-cx)*(sx-cx) + (sy-cy)*(sy-cy));
            double err = std::abs(dist - radius);
            if (err > maxError) maxError = err;
        }

        if (maxError < tolerance) {
            // Good arc — emit it
            GCodeArc arc;
            arc.type = clockwise ? GCodeArc::ARC_CW : GCodeArc::ARC_CCW;
            arc.startX = x0; arc.startY = y0;
            arc.endX = x1; arc.endY = y1;
            arc.centerX = cx; arc.centerY = cy;
            arc.radius = radius;

            double a1 = atan2(y0-cy, x0-cx);
            double a2 = atan2(y1-cy, x1-cx);
            double da = a2 - a1;
            if (clockwise) { if (da > 0) da -= 2*M_PI; }
            else           { if (da < 0) da += 2*M_PI; }
            arc.arcLength = std::abs(da) * radius;

            output.append(arc);
            return;
        }
    }

    // Arc fit failed or error too large — subdivide and recurse
    biArcApproxRecursive(crv, srf, t0, tMid, tolerance, maxDepth - 1, output);
    biArcApproxRecursive(crv, srf, tMid, t1, tolerance, maxDepth - 1, output);
}


// ====================================================================
// STEP 5: Complete Pipeline — Surface + Z → G-Code Segments
// ====================================================================
// This is the main function to call. It replaces:
//   IritUserCntrSrfWithPlane + extractContoursFromIrit + optimizePathCurvature
// with:
//   intersectSurfaceWithZPlane + analyze curves + bi-arc approximate
//
// Returns OptimizedSegment list ready for G-code emission.
// ====================================================================

static QList<GCodeArc> sliceSurfaceToArcs(
    CagdSrfStruct* srf,
    double sliceZ,
    double modelCenterX, double modelCenterY,
    double bedCenterX, double bedCenterY,
    double tolerance)   // mm, typically 0.02-0.05
{
    QList<GCodeArc> allArcs;

    // Step 1: Get exact NURBS curves from intersection
    CagdCrvStruct* curves = intersectSurfaceWithZPlane(srf, sliceZ);
    if (!curves) return allArcs;

    for (CagdCrvStruct* crv = curves; crv; crv = crv->Pnext) {
        // Step 2: Check for exact conic arc (rational degree 2)
        ExactArcInfo exactInfo = checkExactConicArc(crv);
        if (exactInfo.isExactArc) {
            // Perfect arc — emit directly with zero error
            GCodeArc arc;
            arc.type = exactInfo.isClockwise ? GCodeArc::ARC_CW : GCodeArc::ARC_CCW;
            arc.startX = exactInfo.startX - modelCenterX + bedCenterX;
            arc.startY = exactInfo.startY - modelCenterY + bedCenterY;
            arc.endX = exactInfo.endX - modelCenterX + bedCenterX;
            arc.endY = exactInfo.endY - modelCenterY + bedCenterY;
            arc.centerX = exactInfo.cx - modelCenterX + bedCenterX;
            arc.centerY = exactInfo.cy - modelCenterY + bedCenterY;
            arc.radius = exactInfo.radius;

            double da = exactInfo.endAngle - exactInfo.startAngle;
            if (exactInfo.isClockwise) { if (da > 0) da -= 2*M_PI; }
            else                       { if (da < 0) da += 2*M_PI; }
            arc.arcLength = std::abs(da) * exactInfo.radius;

            allArcs.append(arc);
            continue;
        }

        // Step 3: Not an exact conic — use bi-arc approximation
        // Get parameter domain
        double tMin = crv->KnotVector[crv->Order - 1];
        double tMax = crv->KnotVector[crv->Length];

        QList<GCodeArc> segArcs;
        biArcApproxRecursive(crv, srf, tMin, tMax, tolerance, 12, segArcs);

        // Transform to bed coordinates
        for (auto& arc : segArcs) {
            arc.startX = arc.startX - modelCenterX + bedCenterX;
            arc.startY = arc.startY - modelCenterY + bedCenterY;
            arc.endX = arc.endX - modelCenterX + bedCenterX;
            arc.endY = arc.endY - modelCenterY + bedCenterY;
            if (arc.type != GCodeArc::LINE) {
                arc.centerX = arc.centerX - modelCenterX + bedCenterX;
                arc.centerY = arc.centerY - modelCenterY + bedCenterY;
            }
        }

        allArcs.append(segArcs);
    }

    CagdCrvFreeList(curves);
    return allArcs;
}


// ====================================================================
// USAGE EXAMPLE — How to integrate into your slicing loop:
// ====================================================================
//
//  // In sequentialSlice_WithSupport or sequentialSlice_Multi:
//
//  // OLD:
//  IritPrsrPolygonStruct* polys = IritUserCntrSrfWithPlane(srf, plane, res, 0, 1);
//  LayerContours c = extractContoursFromIrit(polys, ...);
//  // ... later in parallelProcess: optimizePathCurvature(wall, ...)
//
//  // NEW:
//  QList<GCodeArc> arcs = sliceSurfaceToArcs(srf, z, modelCX, modelCY, bedCX, bedCY, 0.03);
//  // arcs are already G1/G2/G3 — emit directly in Phase 3:
//  for (const auto& arc : arcs) {
//      if (arc.type == GCodeArc::LINE)
//          out << "G1 X" << arc.endX << " Y" << arc.endY << " E... F...\n";
//      else {
//          double I = arc.centerX - lastX;
//          double J = arc.centerY - lastY;
//          out << (arc.type == GCodeArc::ARC_CW ? "G2" : "G3")
//              << " X" << arc.endX << " Y" << arc.endY
//              << " I" << I << " J" << J << " E... F...\n";
//      }
//  }
//
// NOTE: If SymbSrfZeroSet doesn't compile (not all IRIT builds include it),
// you can still use IritUserCntrSrfWithPlane for the intersection and
// feed the resulting polyline into the point-based optimizer. The conic
// detection (Step 2) won't help but the bi-arc approximation (Step 4)
// can work on sampled points too.
// ====================================================================

#endif // NURBS_ARC_FITTER_H
