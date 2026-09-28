#pragma once
//
// PuzzleDivider - cuts a trivariate's parameter domain into cuboid cells and
// extracts each cell as its own sub-trivariate. One cell = one puzzle piece.
//
// This is Elber's construction minus the joints. The division is non-uniform by
// construction: a DivisionSpec is a sorted list of interior split parameters per
// axis, and uniform is the special case where those splits are evenly spaced.
//
#include "MeshData.h"
#include "Trivariate.h"

#include <QRandomGenerator>
#include <QString>
#include <QVector>

struct PuzzlePiece {
    int      i = 0, j = 0, k = 0;
    double   p0[3] = { 0, 0, 0 };
    double   p1[3] = { 0, 0, 0 };
    MeshData mesh;
    float    centre[3] = { 0, 0, 0 };
    float    size[3]   = { 0, 0, 0 };
    quint32  tint      = 0;     // 0 = the palette colour for i, j, k

    float largestSide() const;
};

struct DivisionSpec {
    QVector<double> splits[3];
    QString         note;
    bool            capped = false;

    int cells(int axis) const { return splits[axis].size() + 1; }
    int cellCount()     const { return cells(0) * cells(1) * cells(2); }
};

class MaterialField;

struct CellBox {
    double lo[3] = { 0, 0, 0 };
    double hi[3] = { 0, 0, 0 };

    double extent(int axis) const { return hi[axis] - lo[axis]; }
    double volume() const { return extent(0) * extent(1) * extent(2); }
};

class PuzzleDivider {
public:
    static QVector<CellBox> buildBspCells(const double domain[6], int targetPieces,
                                          double splitJitter, quint32 seed,
                                          double minSide = 0.0,
                                          const MaterialField *material = nullptr);

    struct BspNode {
        CellBox box;
        int     child[2] = { -1, -1 };
        int     parent   = -1;
        bool    isLeaf() const { return child[0] < 0; }
    };

    static QVector<BspNode> buildBspTree(const double domain[6], int targetPieces,
                                         double splitJitter, quint32 seed,
                                         double minSide = 0.0,
                                         const MaterialField *material = nullptr);

    static QVector<int> leavesOf(const QVector<BspNode> &tree,
                                 QVector<CellBox> *boxes);

    static void collapse(QVector<BspNode> &tree, int node);

    static bool splitLeaf(QVector<BspNode> &tree, int node,
                          double splitJitter, double minSide,
                          QRandomGenerator *rng);

    struct Neighbours { int a, b, axis; };
    static QVector<Neighbours> adjacencyOfBoxes(const QVector<PuzzlePiece> &pieces,
                                                double eps);

    static DivisionSpec uniform(const Trivariate &tv, const int counts[3]);

    static DivisionSpec jittered(const Trivariate &tv, const int counts[3],
                                 double jitter, quint32 seed);

    static DivisionSpec toBuildVolume(const Trivariate &tv, const double budget[3],
                                      int maxCellsPerAxis);

    static DivisionSpec splitsFromArcLength(const Trivariate &tv,
                                            const double budget[3],
                                            int maxCellsPerAxis);

    static bool divideCells(const Trivariate &tv, const QVector<CellBox> &cells,
                            double fineNess, QVector<PuzzlePiece> *pieces,
                            QString *error);

    static bool divide(const Trivariate &tv, const DivisionSpec &spec,
                       double fineNess, QVector<PuzzlePiece> *pieces,
                       QString *error);

    static QVector<Neighbours> adjacency(const QVector<PuzzlePiece> &pieces,
                                         const DivisionSpec &spec);
};
