#pragma once
//
// CutWarp - curved, irregular cuts without cutting by curved surfaces: warp the
// model, cut it with ordinary planes, un-warp the pieces.
//
// Each step displaces one coordinate by a smooth function of only the other two,
// so the Jacobian is triangular with 1s on the diagonal - determinant exactly 1
// at any amplitude. It cannot fold, and it inverts by running the steps
// backwards. Two rounds are used so that parallel cuts bend differently instead
// of repeating one surface shifted along.
//
#include <QtGlobal>
#include <QString>

struct CutWarp {
    static constexpr int kRounds = 2;

    bool    enabled = false;
    double  amount  = 0.12;
    quint32 seed    = 7;

    double grid[kRounds][3][4][4] = {};

    void reseed();

    bool active() const { return enabled && amount != 0.0; }

    void apply (double p[3], const double bmin[3], const double bmax[3],
                double diagonal) const;
    void invert(double p[3], const double bmin[3], const double bmax[3],
                double diagonal) const;

    QString describe() const;
};
