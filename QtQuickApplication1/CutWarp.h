#pragma once
//
// CutWarp - makes the cuts irregular and curved instead of flat planes.
//
// The trick is not to cut with curved surfaces, which is hard, but to warp the
// model, cut it with ordinary planes, and un-warp the pieces. The interfaces
// then carry the inverse of the warp.
//
// Why this warp is safe: each step displaces ONE coordinate by a smooth
// function of only the other two,
//
//     x' = x + f0(y , z )
//     y' = y + f1(x', z )
//     z' = z + f2(x', y')
//
// so the Jacobian is triangular with 1s on the diagonal - determinant exactly
// 1, everywhere, at any amplitude. It can never fold on itself, and the inverse
// is the same steps run backwards. The pieces therefore still tile the model
// exactly, and the model's own surface lands back precisely where it started.
//
// Why TWO rounds: in a single round the x displacement depends only on (y, z),
// so every cut perpendicular to x is the same surface merely shifted along -
// parallel cuts come out identical, which does not look random at all. Running
// the three shears a second time with different fields makes the x
// displacement depend on x as well, so each cut bends differently. Composing
// two invertible maps is still invertible, so nothing is given up.
//
#include <QtGlobal>
#include <QString>

struct CutWarp {
    static constexpr int kRounds = 2;

    bool    enabled = false;
    double  amount  = 0.12;      // fraction of the model's bbox diagonal
    quint32 seed    = 7;

    // Per round, per axis, a 4x4 grid of control heights in [-1, 1], read as a
    // bicubic Bezier. Bezier patches stay inside the range of their controls,
    // so `amount` is a genuine bound on how far a cut can bend.
    double grid[kRounds][3][4][4] = {};

    // Fills the control grids from `seed`. Call after changing the seed.
    void reseed();

    bool active() const { return enabled && amount != 0.0; }

    // In-place warp / un-warp of one world point. `bmin`/`bmax` are the
    // ORIGINAL model bounds and must be the same for both calls.
    void apply (double p[3], const double bmin[3], const double bmax[3],
                double diagonal) const;
    void invert(double p[3], const double bmin[3], const double bmax[3],
                double diagonal) const;

    QString describe() const;
};
