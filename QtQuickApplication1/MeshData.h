#pragma once
//
// MeshData - the single geometry container every loader produces and the
// viewer consumes. Deliberately dumb: flat float/index arrays, no IRIT
// types, no Qt Quick types. Increment 4 will hand one of these per puzzle
// piece to the same viewer.
//
#include <QString>
#include <QVector>
#include <cstdint>

struct MeshData {
    QVector<float>    pos;      // 3 floats per vertex
    QVector<uint32_t> tris;     // 3 indices per triangle
    QVector<uint32_t> edges;    // 2 indices per edge (deduped tri edges + polylines)
    QVector<float>    triNrm;   // 3 floats per triangle, unit face normal

    // Polyline/curve segments are collected here during parsing and merged
    // into `edges` by buildEdges(). Kept apart so they survive dedup.
    QVector<uint32_t> polylineEdges;

    float bmin[3] = { 0, 0, 0 };
    float bmax[3] = { 0, 0, 0 };

    QString sourceKind;             // "STL", "OBJ", "IRIT", "IGES", ...
    int     objectCount   = 0;      // top-level IRIT objects seen
    int     freeformCount = 0;      // freeform objects that had to be tessellated
    int     polygonCount  = 0;      // source polygons before triangulation

    int  vertexCount()   const { return pos.size() / 3; }
    int  triangleCount() const { return tris.size() / 3; }
    int  edgeCount()     const { return edges.size() / 2; }
    bool isEmpty()       const { return tris.isEmpty() && edges.isEmpty(); }

    uint32_t addVertex(double x, double y, double z) {
        const uint32_t i = static_cast<uint32_t>(pos.size() / 3);
        pos.push_back(static_cast<float>(x));
        pos.push_back(static_cast<float>(y));
        pos.push_back(static_cast<float>(z));
        return i;
    }

    void computeBounds();
    void computeNormals();
    void buildEdges();          // dedupe triangle edges, then append polylineEdges
    void finalize() { computeBounds(); computeNormals(); buildEdges(); }

    float diagonal() const;
    void  centroid(float c[3]) const;
};
