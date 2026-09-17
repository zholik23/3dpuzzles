#pragma once
//
// MaterialField - how much of the model lies inside an axis-aligned box.
//
// The BSP splits the cage, which is a box, while the model is not: an armadillo
// fills only 10.7% of its own bounding box. Cuts chosen by cage volume therefore
// place cells in thin air, and a request for 6 pieces comes back with 5. With
// this, cuts follow the material instead.
//
// Every cell is an axis-aligned box, so a 3D prefix sum over a voxel grid answers
// each query with eight lookups - O(1), cheap enough to test hundreds of
// candidate planes per cut. Coordinates are local: (0,0,0) is the model's
// minimum corner.
//
#include "MeshData.h"

#include <QVector>

class MaterialField {
public:
    static MaterialField build(const MeshData &mesh, int maxRes = 96);

    bool   isValid() const { return !m_sum.isEmpty(); }
    double total()   const;

    double volumeIn(const double lo[3], const double hi[3]) const;

    int    dim(int axis)  const { return m_n[axis]; }
    double side(int axis) const { return m_cell[axis]; }
    int    cellCount()    const { return m_n[0] * m_n[1] * m_n[2]; }
    int    filledCount()  const
    {
        int n = 0;
        for (quint8 v : m_occ) n += v ? 1 : 0;
        return n;
    }

    double voxelVolume() const { return m_cell[0] * m_cell[1] * m_cell[2]; }

    bool isConnected(const double lo[3], const double hi[3]) const;

    int lumpStats(const double lo[3], const double hi[3],
                  int *biggest, int *total) const;

private:
    int             m_n[3]    = { 0, 0, 0 };
    double          m_cell[3] = { 0, 0, 0 };
    QVector<double> m_sum;
    QVector<quint8> m_occ;

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
