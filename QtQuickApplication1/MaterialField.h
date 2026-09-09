#pragma once
//
// MaterialField - how much of the model lies inside an axis-aligned box.
//
// The reason this exists: the BSP splits the CAGE, and the cage is a box while
// the model is not. Choosing cuts by cage volume means the splitter has never
// looked at the model, so it happily places a cell in thin air - an armadillo
// fills only 10.7% of its own bounding box, so most of the cage is empty. Those
// cells produce nothing, get dropped, and a request for 6 pieces returns 5.
//
// With this, the splitter can ask "how much material is in this box?" and cut
// where the model actually is. Since every cell is an axis-aligned box, the
// answer comes from a 3D prefix sum over a voxel grid: eight lookups, O(1), so
// the search can test hundreds of candidate planes per cut without cost.
//
// Coordinates are LOCAL: (0,0,0) is the model's minimum corner. That matches the
// domain the BSP runs on, which is sized to the model's world extents.
//
#include "MeshData.h"

#include <QVector>

class MaterialField {
public:
    // Voxelises `mesh`. `maxRes` is the resolution of the longest axis; the
    // others are scaled to keep voxels roughly cubic.
    static MaterialField build(const MeshData &mesh, int maxRes = 96);

    bool   isValid() const { return !m_sum.isEmpty(); }
    double total()   const;

    // Material volume inside the local-space box [lo, hi]. Quantised to the
    // voxel grid, which is ample for deciding where to cut.
    double volumeIn(const double lo[3], const double hi[3]) const;

    // Voxel volume - the granularity of any answer above.
    double voxelVolume() const { return m_cell[0] * m_cell[1] * m_cell[2]; }

    // True when the material inside the box forms ONE connected lump.
    //
    // A cut that severs a piece produces something nobody can print or joint -
    // a "piece" made of a slice of thigh and a slice of tail with air between.
    // Asking this before accepting a cut prevents that, where the alternative is
    // detecting it after the Boolean and repairing it. Resolution-limited: a
    // connection thinner than a voxel is not seen.
    bool isConnected(const double lo[3], const double hi[3]) const;

private:
    int             m_n[3]    = { 0, 0, 0 };
    double          m_cell[3] = { 0, 0, 0 };
    QVector<double> m_sum;            // (n0+1)(n1+1)(n2+1) inclusive prefix sums
    QVector<quint8> m_occ;            // n0*n1*n2 occupancy, for connectivity

    // Voxel index range whose centres lie inside [lo, hi]; false when empty.
    bool range(const double lo[3], const double hi[3], int a[3], int b[3]) const;

    int occIndex(int i, int j, int k) const
    {
        return (i * m_n[1] + j) * m_n[2] + k;
    }

    int sumIndex(int i, int j, int k) const
    {
        return (i * (m_n[1] + 1) + j) * (m_n[2] + 1) + k;
    }
};
