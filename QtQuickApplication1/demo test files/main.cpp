#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QDebug>
#include <QQuickItem>
#include <QOpenGLFunctions>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "IritRenderer.h"
#include <QFile>
#include <QTextStream>
#include <QStandardPaths>
#include <QUrl>
#include "GCodeGenerator.h"
#include <QFileInfo>
#include <QElapsedTimer>
#include <QDir>
#include <QHash>

#ifndef __WINNT__
#define __WINNT__
#endif

extern "C" {
#include "irit_sm.h"
#include "iritprsr.h"
#include "allocate.h"
#include "cagd_lib.h"
#include "symb_lib.h"
#include "user_lib.h"
#include "triv_lib.h"
#include "attribut.h"
#include "mdl_lib.h"
}

// Undefine IRIT macros that collide with Qt
#ifdef mkdir
#undef mkdir
#endif
#ifdef rmdir
#undef rmdir
#endif


// ====================================================================
// VISUALIZATION HELPERS
// ====================================================================
static void VisualizeSurface(CagdSrfStruct* srf, IritRenderer* renderer) {
    IritPrsrPolygonStruct* surfaceMesh = IritSymbSrf2Polygons(srf, 50, TRUE, TRUE, TRUE);
    if (surfaceMesh != nullptr) {
        for (IritPrsrPolygonStruct* pl = surfaceMesh; pl != nullptr; pl = pl->Pnext) {
            IritPrsrVertexStruct* v1 = pl->PVertex;
            IritPrsrVertexStruct* v2 = (v1 != nullptr) ? v1->Pnext : nullptr;
            IritPrsrVertexStruct* v3 = (v2 != nullptr) ? v2->Pnext : nullptr;
            if (v1 && v2 && v3) {
                QVector3D p1(v1->Coord[0], v1->Coord[1], v1->Coord[2]);
                QVector3D p2(v2->Coord[0], v2->Coord[1], v2->Coord[2]);
                QVector3D p3(v3->Coord[0], v3->Coord[1], v3->Coord[2]);
                renderer->addModelTriangle(p1, p2, p3);
            }
        }
        IritPrsrFreePolygonList(surfaceMesh);
    }
}

static void VisualizeMesh(IritPrsrPolygonStruct* firstPoly, IritRenderer* renderer) {
    for (IritPrsrPolygonStruct* pl = firstPoly; pl != nullptr; pl = pl->Pnext) {
        IritPrsrVertexStruct* v1 = pl->PVertex;
        IritPrsrVertexStruct* v2 = (v1 != nullptr) ? v1->Pnext : nullptr;
        IritPrsrVertexStruct* v3 = (v2 != nullptr) ? v2->Pnext : nullptr;
        if (v1 && v2 && v3) {
            renderer->addModelTriangle(
                QVector3D(v1->Coord[0], v1->Coord[1], v1->Coord[2]),
                QVector3D(v2->Coord[0], v2->Coord[1], v2->Coord[2]),
                QVector3D(v3->Coord[0], v3->Coord[1], v3->Coord[2]));
        }
    }
}


// ====================================================================
// OVERHANG DETECTION
// ====================================================================
static bool detectOverhangs(IritPrsrPolygonStruct* polys, double thresholdDeg) {
    for (IritPrsrPolygonStruct* pl = polys; pl != nullptr; pl = pl->Pnext) {
        if (IRIT_PRSR_HAS_PLANE_POLY(pl)) {
            if (pl->Plane[2] < 0.0) {
                double slopeAngle = asin(fabs(pl->Plane[2])) * (180.0 / M_PI);
                if (slopeAngle < thresholdDeg) {
                    return true;
                }
            }
        }
    }
    return false;
}


// ====================================================================
// SUPPORT STRUCTURE EXTRACTION — tessellates ALL types to triangles
// ====================================================================
// Instead of converting support tiles to NURBS surfaces (12,000+ surfaces
// that each need expensive NURBS slicing), we tessellate everything to
// triangles and store in a SupportMesh for fast polygon-based slicing.
//
// Polygon slicer: O(active_triangles) per layer ~2ms
// NURBS slicer:   O(surfaces × NURBS_cost) per layer ~30s
// ====================================================================

struct SupportExtractionStats {
    int trivarCount = 0;
    int surfaceCount = 0;
    int polyCount = 0;
    int triangleCount = 0;
    int listCount = 0;
    int unknownCount = 0;
    double minZ = 1e9, maxZ = -1e9;
};

// Helper: add an IRIT polygon list to the SupportMesh as triangles
static void addPolygonsToMesh(IritPrsrPolygonStruct* polys, SupportMesh& mesh,
    IritRenderer* renderer, SupportExtractionStats& stats) {
    for (IritPrsrPolygonStruct* pl = polys; pl; pl = pl->Pnext) {
        IritPrsrVertexStruct* v1 = pl->PVertex;
        IritPrsrVertexStruct* v2 = v1 ? v1->Pnext : nullptr;
        IritPrsrVertexStruct* v3 = v2 ? v2->Pnext : nullptr;
        if (v1 && v2 && v3) {
            mesh.addTriangle(
                v1->Coord[0], v1->Coord[1], v1->Coord[2],
                v2->Coord[0], v2->Coord[1], v2->Coord[2],
                v3->Coord[0], v3->Coord[1], v3->Coord[2]);
            stats.triangleCount++;

            // Also fan-tessellate quads and higher-order polygons
            IritPrsrVertexStruct* vn = v3->Pnext;
            IritPrsrVertexStruct* prev = v3;
            while (vn) {
                mesh.addTriangle(
                    v1->Coord[0], v1->Coord[1], v1->Coord[2],
                    prev->Coord[0], prev->Coord[1], prev->Coord[2],
                    vn->Coord[0], vn->Coord[1], vn->Coord[2]);
                stats.triangleCount++;
                prev = vn;
                vn = vn->Pnext;
            }

            // Visualization
            renderer->addModelTriangle(
                QVector3D(v1->Coord[0], v1->Coord[1], v1->Coord[2]),
                QVector3D(v2->Coord[0], v2->Coord[1], v2->Coord[2]),
                QVector3D(v3->Coord[0], v3->Coord[1], v3->Coord[2]));
        }
    }
}

static void extractSupportToMesh(
    IritPrsrObjectStruct* obj,
    SupportMesh& mesh,
    IritRenderer* renderer,
    SupportExtractionStats& stats)
{
    if (!obj) return;

    switch (obj->ObjType) {
    case IRIT_PRSR_OBJ_LIST_OBJ: {
        stats.listCount++;
        for (int i = 0; ; i++) {
            IritPrsrObjectStruct* child = IritPrsrListObjectGet(obj, i);
            if (!child) break;
            extractSupportToMesh(child, mesh, renderer, stats);
        }
        break;
    }

    case IRIT_PRSR_OBJ_TRIVAR: {
        stats.trivarCount++;
        // Tessellate trivariate boundary to polygons
        CagdSrfStruct* bndrySrfs = IritTrivBndrySrfsFromTVs(
            obj->U.Trivars, 1e-6, TRUE, FALSE, FALSE);
        for (CagdSrfStruct* srf = bndrySrfs; srf; srf = srf->Pnext) {
            IritPrsrPolygonStruct* polys = IritSymbSrf2Polygons(srf, 10, TRUE, TRUE, TRUE);
            if (polys) {
                addPolygonsToMesh(polys, mesh, renderer, stats);
                IritPrsrFreePolygonList(polys);
            }
        }
        // Free boundary surfaces — all data is now in triangles
        // If IritCagdSrfFreeList doesn't compile, try: CagdSrfFreeList(bndrySrfs);
        if (bndrySrfs) IritCagdSrfFreeList(bndrySrfs);
        break;
    }

    case IRIT_PRSR_OBJ_SURFACE: {
        stats.surfaceCount++;
        for (CagdSrfStruct* srf = obj->U.Srfs; srf; srf = srf->Pnext) {
            IritPrsrPolygonStruct* polys = IritSymbSrf2Polygons(srf, 10, TRUE, TRUE, TRUE);
            if (polys) {
                addPolygonsToMesh(polys, mesh, renderer, stats);
                IritPrsrFreePolygonList(polys);
            }
        }
        break;
    }

    case IRIT_PRSR_OBJ_TRIMSRF: {
        stats.surfaceCount++;
        for (TrimSrfStruct* ts = obj->U.TrimSrfs; ts; ts = ts->Pnext) {
            IritPrsrPolygonStruct* polys = IritSymbSrf2Polygons(ts->Srf, 10, TRUE, TRUE, TRUE);
            if (polys) {
                addPolygonsToMesh(polys, mesh, renderer, stats);
                IritPrsrFreePolygonList(polys);
            }
        }
        break;
    }

    case IRIT_PRSR_OBJ_POLY: {
        stats.polyCount++;
        if (obj->U.Pl) {
            addPolygonsToMesh(obj->U.Pl, mesh, renderer, stats);
        }
        break;
    }

    default:
        stats.unknownCount++;
        break;
    }
}


// ====================================================================
// MESH STITCHING — welds vertices within tolerance to close seam gaps
// ====================================================================
// When multiple NURBS surfaces are tessellated independently, shared
// boundary edges have slightly different vertex positions on each side.
// This creates a non-watertight mesh that breaks inside/outside tests.
//
// Fix: quantize vertex coordinates into a spatial hash grid and merge
// vertices that fall into the same cell. O(V) time, O(V) space.
// ====================================================================
static int stitchMeshVertices(IritPrsrPolygonStruct* polys, double tolerance) {
    if (!polys) return 0;

    // Step 1: Collect all vertices with hash keys
    struct VtxEntry {
        IritPrsrVertexStruct* vtx;
        int64_t hashKey;
    };

    double invTol = 1.0 / tolerance;
    auto quantize = [invTol](double v) -> int64_t {
        return static_cast<int64_t>(floor(v * invTol));
        };
    auto makeKey = [quantize](double x, double y, double z) -> int64_t {
        int64_t ix = quantize(x), iy = quantize(y), iz = quantize(z);
        // Combine using large primes to minimize collisions
        return ix * 73856093LL ^ iy * 19349669LL ^ iz * 83492791LL;
        };

    // Map from hash key to the "representative" vertex coordinates
    struct RepVertex {
        double x, y, z;
        int count;
    };
    QHash<int64_t, RepVertex> vertexMap;

    int totalVertices = 0;
    int mergedVertices = 0;

    // Step 2: First pass — establish representative vertex for each cell
    for (IritPrsrPolygonStruct* pl = polys; pl; pl = pl->Pnext) {
        for (IritPrsrVertexStruct* v = pl->PVertex; v; v = v->Pnext) {
            totalVertices++;
            int64_t key = makeKey(v->Coord[0], v->Coord[1], v->Coord[2]);

            auto it = vertexMap.find(key);
            if (it == vertexMap.end()) {
                // First vertex in this cell — it becomes the representative
                vertexMap.insert(key, { v->Coord[0], v->Coord[1], v->Coord[2], 1 });
            }
            else {
                // Cell already has a representative — count it
                it->count++;
            }
        }
    }

    // Step 3: Second pass — snap all vertices to their cell's representative
    for (IritPrsrPolygonStruct* pl = polys; pl; pl = pl->Pnext) {
        for (IritPrsrVertexStruct* v = pl->PVertex; v; v = v->Pnext) {
            int64_t key = makeKey(v->Coord[0], v->Coord[1], v->Coord[2]);
            auto it = vertexMap.find(key);
            if (it != vertexMap.end() && it->count > 1) {
                // Snap to representative position
                double dx = v->Coord[0] - it->x;
                double dy = v->Coord[1] - it->y;
                double dz = v->Coord[2] - it->z;
                if (dx * dx + dy * dy + dz * dz > 0) {
                    v->Coord[0] = it->x;
                    v->Coord[1] = it->y;
                    v->Coord[2] = it->z;
                    mergedVertices++;
                }
            }
        }
    }

    // Step 4: Recompute polygon plane equations (normals) after vertex changes
    for (IritPrsrPolygonStruct* pl = polys; pl; pl = pl->Pnext) {
        IritPrsrVertexStruct* v1 = pl->PVertex;
        IritPrsrVertexStruct* v2 = v1 ? v1->Pnext : nullptr;
        IritPrsrVertexStruct* v3 = v2 ? v2->Pnext : nullptr;
        if (v1 && v2 && v3) {
            double ux = v2->Coord[0] - v1->Coord[0];
            double uy = v2->Coord[1] - v1->Coord[1];
            double uz = v2->Coord[2] - v1->Coord[2];
            double vx = v3->Coord[0] - v1->Coord[0];
            double vy = v3->Coord[1] - v1->Coord[1];
            double vz = v3->Coord[2] - v1->Coord[2];
            // Cross product
            double nx = uy * vz - uz * vy;
            double ny = uz * vx - ux * vz;
            double nz = ux * vy - uy * vx;
            double len = sqrt(nx * nx + ny * ny + nz * nz);
            if (len > 1e-10) {
                pl->Plane[0] = nx / len;
                pl->Plane[1] = ny / len;
                pl->Plane[2] = nz / len;
                pl->Plane[3] = -(pl->Plane[0] * v1->Coord[0] +
                    pl->Plane[1] * v1->Coord[1] +
                    pl->Plane[2] * v1->Coord[2]);
            }
        }
    }

    return mergedVertices;
}


// ====================================================================
// MESH DIAGNOSTICS — count holes and check quality
// ====================================================================
static void diagnoseMesh(IritPrsrPolygonStruct* polys) {
    int polyCount = 0;
    int vertexCount = 0;
    int downFacingCount = 0;
    double minZ = 1e9, maxZ = -1e9;

    for (IritPrsrPolygonStruct* pl = polys; pl; pl = pl->Pnext) {
        polyCount++;
        for (IritPrsrVertexStruct* v = pl->PVertex; v; v = v->Pnext) {
            vertexCount++;
            if (v->Coord[2] < minZ) minZ = v->Coord[2];
            if (v->Coord[2] > maxZ) maxZ = v->Coord[2];
        }
        if (IRIT_PRSR_HAS_PLANE_POLY(pl) && pl->Plane[2] < -0.1)
            downFacingCount++;
    }

    qDebug() << "   [Mesh] Polygons:" << polyCount
        << "Vertices:" << vertexCount
        << "Z:" << minZ << "to" << maxZ
        << "Down-facing:" << downFacingCount;
}


// ====================================================================
// SUPPORT GENERATION PIPELINE
// ====================================================================
static void generateSupports(
    IritPrsrObjectStruct* polyObj,
    SupportMesh& supportMesh,
    IritRenderer* renderer)
{
    QElapsedTimer supportTimer;
    supportTimer.start();

    // --- STEP 0: Diagnose and stitch the polygon mesh ---
    qDebug() << "   [Support] Mesh before stitching:";
    diagnoseMesh(polyObj->U.Pl);

    int merged = stitchMeshVertices(polyObj->U.Pl, 0.01);
    qDebug() << "   [Support] Vertex stitching: merged" << merged << "vertices";

    qDebug() << "   [Support] Mesh after stitching:";
    diagnoseMesh(polyObj->U.Pl);

    // Set Bndry attribute for boundary surface extraction
    IritMiscAttrSetObjectIntAttrib(polyObj, "Bndry", TRUE);

    // --- STEP 1: Configure support parameters for FDM ---
    IritUserMicroGenAMSupportParamStruct Params;
    memset(&Params, 0, sizeof(Params));

    Params.TileSize = 10;
    Params.TileCntrThickness = 0.42;
    Params.TileEndEdgeThickness = 0.42;
    Params.TileDiagEdgeSlope = 55;
    Params.TileHaveVertBars = TRUE;

    Params.SupportExtent[0] = Params.TileSize * 0.5;
    Params.SupportExtent[1] = Params.TileSize * 0.5;
    Params.SupportExtent[2] = Params.TileSize *
        tan(IRIT_DEG2RAD(Params.TileDiagEdgeSlope)) * 1.2;

    Params.TipMaxLength = Params.TileSize * 3.0;
    Params.TipMinSlope = 45.0;
    Params.ModelMinSlope = 45.0;
    Params.OutputType = 0;

    qDebug() << "   [Support] Calling IritUserMicroGenAMSupport...";

    IritPrsrObjectStruct* supportStruct = IritUserMicroGenAMSupport(polyObj, &Params);

    if (supportStruct == nullptr) {
        qDebug() << "   [Support] WARNING: IritUserMicroGenAMSupport returned NULL!";
        return;
    }

    qDebug() << "   [Support] Generation completed in" << supportTimer.elapsed() << "ms";
    qDebug() << "   [Support] Tessellating support structure to polygon mesh...";

    QElapsedTimer tessTimer; tessTimer.start();

    // Extract and tessellate ALL support geometry into triangles
    SupportExtractionStats stats;
    extractSupportToMesh(supportStruct, supportMesh, renderer, stats);

    // Build Z-sorted index for fast per-layer slicing
    if (!supportMesh.isEmpty()) {
        supportMesh.buildIndex();
    }

    qDebug() << "   [Support] Tessellation completed in" << tessTimer.elapsed() << "ms";

    // Diagnostic report
    qDebug() << "   =========================================";
    qDebug() << "   SUPPORT EXTRACTION REPORT:";
    qDebug() << "   Lists traversed:" << stats.listCount;
    qDebug() << "   Trivar tiles found:" << stats.trivarCount;
    qDebug() << "   Surface objects found:" << stats.surfaceCount;
    qDebug() << "   Polygon objects found:" << stats.polyCount;
    qDebug() << "   Triangles in mesh:" << stats.triangleCount;
    qDebug() << "   Support Z range:" << supportMesh.globalMinZ
        << "to" << supportMesh.globalMaxZ;
    qDebug() << "   =========================================";

    if (stats.triangleCount == 0) {
        qDebug() << "   [Support] WARNING: No triangles extracted!";
        qDebug() << "   [Support] Try reducing TileSize to 1.5 or 1.0";
    }

    // Free the support structure — all data is now in the SupportMesh
    IritPrsrFreeObject(supportStruct);
}
// ====================================================================
// RECURSIVE SURFACE EXTRACTOR
// Digs through nested IGES Shells/Manifolds to find the raw geometry
// ====================================================================
static void ExtractSurfaces(IritPrsrObjectStruct* obj, QList<CagdSrfStruct*>& allSurfaces, IritRenderer* renderer) {
    if (!obj) return;

    // Iterate through the linked list at the current level
    for (IritPrsrObjectStruct* current = obj; current != nullptr; current = current->Pnext) {

        if (current->ObjType == IRIT_PRSR_OBJ_SURFACE) {
            // Found raw surfaces
            qDebug() << "   -> Found Surface. :DD";
            for (CagdSrfStruct* srf = current->U.Srfs; srf != nullptr; srf = srf->Pnext) {
                VisualizeSurface(srf, renderer);
                allSurfaces.append(srf);
            }
        }
        else if (current->ObjType == IRIT_PRSR_OBJ_TRIMSRF) {
            // Found trimmed surfaces
            qDebug() << "   -> Found Trimmed Surface. :D";
            for (TrimSrfStruct* trimSrf = current->U.TrimSrfs; trimSrf != nullptr; trimSrf = trimSrf->Pnext) {
                CagdSrfStruct* baseSrf = trimSrf->Srf;
                VisualizeSurface(baseSrf, renderer);
                allSurfaces.append(baseSrf);
            }
        }
        else if (current->ObjType == IRIT_PRSR_OBJ_LIST_OBJ) {
            // Found a nested container (like a MANIFOLD or SHELL).
            // We must dig deeper into it.
            for (int i = 0; ; i++) {
                IritPrsrObjectStruct* child = IritPrsrListObjectGet(current, i);
                if (!child) break; // Reached the end of this list
                ExtractSurfaces(child, allSurfaces, renderer); // Recursive call
            }

        }
        else if (current->ObjType == IRIT_PRSR_OBJ_MODEL) {
            qDebug() << "   -> Found B-Rep MODEL. Extracting surfaces...";

            // IRIT 'Model' objects contain a linked list of models...
            for (MdlModelStruct* mdl = current->U.Mdls; mdl != nullptr; mdl = mdl->Pnext) {
                // ...and each model contains a linked list of MdlTrimSrfStructs
                for (MdlTrimSrfStruct* mTrim = mdl->TrimSrfList; mTrim != nullptr; mTrim = mTrim->Pnext) {

                    // MdlTrimSrfStruct points DIRECTLY to the base CagdSrfStruct
                    if (mTrim->Srf) {
                        CagdSrfStruct* baseSrf = mTrim->Srf;

                        VisualizeSurface(baseSrf, renderer);
                        allSurfaces.append(baseSrf); // Append to your slicer's list!
                    }
                }
            }
        }

        else {
            // It's something else (like curves or point clouds), ignore it quietly
             qDebug() << "   -> Skipping non-surface object type:(((" << current->ObjType;
        }
    }
}

// ====================================================================
// MAIN GEOMETRY ANALYSIS PIPELINE
// ====================================================================
static void AnalyzeIritGeometry(
    IritPrsrObjectStruct* firstObj,
    IritRenderer* renderer,
    const QString& inputFilePath,
    bool printSolidRoof)
{
    renderer->clearAll();
    QElapsedTimer totalTimer;
    totalTimer.start();

    QFileInfo fileInfo(inputFilePath);
    QString baseName = fileInfo.baseName();

    QString desktopPath = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation);
    QString outputDir = desktopPath + "/from IRIT to Gcode";
    QDir().mkpath(outputDir);
    QString outputPath = outputDir + "/" + baseName + ".gcode";

    QFile gcodeFile(outputPath);
    if (!gcodeFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        qDebug() << "   Error: Cannot create G-code file at:" << outputPath;
        return;
    }
    QTextStream out(&gcodeFile);

    // 1. WRITE START G-CODE
    out << readGCodeTemplate("start_template.gcode") << "\n";

    // 2. COLLECT ALL SURFACES from the file
    QList<CagdSrfStruct*> allSurfaces;
    bool surfaceFound = false;
    qDebug() << "-> Extracting surfaces :)";
    //ExtractSurfaces(firstObj, allSurfaces, renderer);
    for (IritPrsrObjectStruct* current = firstObj; current != nullptr; current = current->Pnext) {
        if (current->ObjType == IRIT_PRSR_OBJ_SURFACE) {
            surfaceFound = true;
            qDebug() << "\n-> Found Surface from IRIT file.";
            for (CagdSrfStruct* srf = current->U.Srfs; srf != nullptr; srf = srf->Pnext) {
                VisualizeSurface(srf, renderer);
                allSurfaces.append(srf);
            }
        }
        else if (current->ObjType == IRIT_PRSR_OBJ_TRIMSRF) {
            surfaceFound = true;
            qDebug() << "-> Found Trimmed Surface.";
            for (TrimSrfStruct* trimSrf = current->U.TrimSrfs; trimSrf != nullptr; trimSrf = trimSrf->Pnext) {
                CagdSrfStruct* baseSrf = trimSrf->Srf;
                VisualizeSurface(baseSrf, renderer);
                allSurfaces.append(baseSrf);
            }
        }
        else if (current->ObjType == IRIT_PRSR_OBJ_MODEL) {
            qDebug() << "   -> Found B-Rep MODEL. Extracting surfaces...";
            surfaceFound = true;
            // IRIT 'Model' objects contain a linked list of models...
            for (MdlModelStruct* mdl = current->U.Mdls; mdl != nullptr; mdl = mdl->Pnext) {
                // ...and each model contains a linked list of MdlTrimSrfStructs
                for (MdlTrimSrfStruct* mTrim = mdl->TrimSrfList; mTrim != nullptr; mTrim = mTrim->Pnext) {

                    // MdlTrimSrfStruct points DIRECTLY to the base CagdSrfStruct
                    if (mTrim->Srf) {
                        CagdSrfStruct* baseSrf = mTrim->Srf;

                        VisualizeSurface(baseSrf, renderer);
                        allSurfaces.append(baseSrf); // Append to your slicer's list!
                    }
                }
            }
        }
        else {
            qDebug() << "\n-> Warning: Object type" << current->ObjType << "is not a surface.";
        }
    }

    if (!surfaceFound || allSurfaces.isEmpty()) {
        qDebug() << "\n-> CRITICAL ERROR: No CAD surfaces found in file.";
        gcodeFile.close();
        return;
    }

    qDebug() << "-> Found" << allSurfaces.size() << "model surfaces.";

    // 3. OVERHANG DETECTION + SUPPORT GENERATION
    qDebug() << "-> Analyzing mesh for overhangs...";

    // Build a polygon mesh from all surfaces for overhang analysis
   IritPrsrObjectStruct* polyObj = IritPrsrGenPOLYObject(nullptr);
    for (CagdSrfStruct* srf : allSurfaces) {
        IritPrsrPolygonStruct* mesh = IritSymbSrf2Polygons(srf, 50, TRUE, TRUE, TRUE);
        if (mesh) {
            IritPrsrPolygonStruct* last = mesh;
            while (last->Pnext) last = last->Pnext;
            last->Pnext = polyObj->U.Pl;
            polyObj->U.Pl = mesh;
        }
    }

    bool needsSupport = detectOverhangs(polyObj->U.Pl, 45.0);
    SupportMesh supportMesh; // Empty by default

    if (needsSupport) {
        qDebug() << "-> Overhangs detected! Generating microstructure supports...";
       // generateSupports(polyObj, supportMesh, renderer);
    }
    else {
        qDebug() << "-> No steep overhangs detected. Skipping support generation.";
    }

    // Free the analysis mesh — support data is now in SupportMesh (pure triangles)
    IritPrsrFreeObject(polyObj);

    // 4. SLICE AND GENERATE G-CODE
    // Model surfaces use IRIT NURBS slicer (slow but accurate, only a few surfaces).
    // Support mesh uses fast polygon slicer (O(active_tris) per layer, milliseconds).
    qDebug() << "\n-> Model surfaces:" << allSurfaces.size()
        << "| Support triangles:" << supportMesh.count();

    if (!supportMesh.isEmpty()) {
        // Model + Support: two-track slicing
        qDebug() << "-> Using DUAL slicer (NURBS model + polygon support)";
        GenerateDirectGCodeWithSupport(allSurfaces, supportMesh, renderer, out, printSolidRoof);
    }
    else if (allSurfaces.size() == 1) {
        qDebug() << "-> Single surface, using standard slicing.";
        GenerateDirectGCode(allSurfaces[0], renderer, out, printSolidRoof);
    }
    else {
        qDebug() << "-> " << allSurfaces.size() << " surfaces, using BATCH multi-surface slicing.";
        GenerateDirectGCodeMultiSurface(allSurfaces, renderer, out, printSolidRoof);
    }

    // 5. WRITE END G-CODE
    out << readGCodeTemplate("end_template.gcode") << "\n";

    gcodeFile.close();
    qDebug() << "  Done. Total processing time:" << totalTimer.elapsed() << "ms";
    qDebug() << "  Saved as:" << outputPath;
}


// ====================================================================
// APPLICATION ENTRY POINT
// ====================================================================
int main(int argc, char* argv[])
{
#if defined(Q_OS_WIN) && QT_VERSION_CHECK(5, 6, 0) <= QT_VERSION && QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
#endif

    QGuiApplication app(argc, argv);
    qmlRegisterType<IritRenderer>("CustomGeometry", 1, 0, "IritRenderer");

    QQmlApplicationEngine engine;
    engine.load(QUrl(QStringLiteral("qrc:/qt/qml/qtquickapplication1/main.qml")));
    if (engine.rootObjects().isEmpty()) return -1;

    qDebug() << "=== IRIT test start ===";

    QObject* rootObject = engine.rootObjects().first();
    IritRenderer* renderer = rootObject->findChild<IritRenderer*>("myIritRenderer");

    if (renderer != nullptr) {
        qDebug() << "Renderer initialized. Waiting for file selection...";

        QObject::connect(renderer, &IritRenderer::fileRequested,
            [renderer](const QString& localPath, bool printSolidRoof) {

                QByteArray ba = localPath.toLocal8Bit();
                const char* currentFilePath = ba.constData();
                const char* fileNames[1] = { currentFilePath };

                qDebug() << "\nLoading file:" << localPath;

                IritPrsrObjectStruct* parsedObject = IritPrsrGetDataFiles(
                    (const char**)fileNames, 1, TRUE, FALSE);

                if (parsedObject != nullptr) {
                    qDebug() << " File successfully read.";
                    AnalyzeIritGeometry(parsedObject, renderer, localPath, printSolidRoof);
                    // NOTE: Don't free parsedObject if allSurfaces still references its
                    // surfaces. If support generation created new surface objects,
                    // those are owned by supportStruct (not freed for this reason).
                    // The original model surfaces are owned by parsedObject.
                    // For clean exit, you'd need a more sophisticated ownership model.
                    IritPrsrFreeObjectList(parsedObject);
                }
                else {
                    qDebug() << "Error: IRIT cannot read file at:" << localPath;
                }
            });
    }
    else {
        qDebug() << "error: Cannot find IritRenderer in QML. Check objectName!";
    }

    return app.exec();
}
