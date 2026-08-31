#ifndef GCODEGENERATOR_H
#define GCODEGENERATOR_H
#include <QTextStream>
#include "IritRenderer.h"
#include <QMutex>
#include <QVector>
#include <QVector3D>

extern "C" {
#include "irit_sm.h"
#include "iritprsr.h"
#include "allocate.h"
#include "cagd_lib.h"
#include "symb_lib.h" 
#include "user_lib.h"
#include "bool_lib.h"
#include "geom_lib.h"
}

#ifdef mkdir
#undef mkdir
#endif
#ifdef rmdir
#undef rmdir
#endif

// ====================================================================
// SUPPORT POLYGON MESH — tessellated support triangles for fast slicing
// ====================================================================
// Instead of slicing 12,000+ NURBS surfaces with expensive IritUserCntrSrfWithPlane,
// support trivariates are tessellated to triangles and sliced with a simple
// plane-triangle intersector: O(active_triangles) per layer vs O(surfaces * NURBS_cost).
//
// For 12,287 support surfaces (~60K triangles), this reduces per-layer slicing
// from ~30 seconds to ~2 milliseconds.
// ====================================================================
struct SupportTriangle {
    float v0[3], v1[3], v2[3];
    float minZ, maxZ;
};

struct SupportMesh {
    QVector<SupportTriangle> triangles;
    QVector<int> sortedByMinZ;
    double globalMinZ;
    double globalMaxZ;

    SupportMesh() : globalMinZ(1e9), globalMaxZ(-1e9) {}

    void addTriangle(float x0, float y0, float z0,
                     float x1, float y1, float z1,
                     float x2, float y2, float z2);
    void buildIndex();
    bool isEmpty() const { return triangles.isEmpty(); }
    int count() const { return triangles.size(); }
};

QString readGCodeTemplate(const QString& filePath);

// Single-surface slicing (model only, no supports)
void GenerateDirectGCode(CagdSrfStruct* srf, IritRenderer* renderer, QTextStream& out, bool printSolidRoof);

// Multi-surface slicing (model only, no supports)
void GenerateDirectGCodeMultiSurface(
    const QList<CagdSrfStruct*>& surfaces,
    IritRenderer* renderer,
    QTextStream& out,
    bool printSolidRoof
);

// Multi-surface + support mesh slicing
// Model surfaces use IRIT NURBS slicer (slow but accurate).
// Support mesh uses fast polygon slicer (fast, polygon-based).
void GenerateDirectGCodeWithSupport(
    const QList<CagdSrfStruct*>& modelSurfaces,
    SupportMesh& supportMesh,
    IritRenderer* renderer,
    QTextStream& out,
    bool printSolidRoof
);

#endif // GCODEGENERATOR_H
