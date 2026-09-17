//
// CutWarp - implementation: the bicubic Bernstein displacement fields and the
// forward and inverse shear passes.
//

#include "CutWarp.h"

#include <QRandomGenerator>
#include <cmath>

namespace {

// Cubic Bernstein basis, for the control-point patch.
inline void bernstein3(double u, double b[4])
{
    const double v = 1.0 - u;
    b[0] = v * v * v;
    b[1] = 3.0 * u * v * v;
    b[2] = 3.0 * u * u * v;
    b[3] = u * u * u;
}

inline double normalise(double v, double lo, double hi)
{
    const double d = hi - lo;
    return (d > 1e-12) ? (v - lo) / d : 0.5;
}

inline double patch(const double g[4][4], double s, double t)
{
    double bs[4], bt[4];
    bernstein3(qBound(0.0, s, 1.0), bs);
    bernstein3(qBound(0.0, t, 1.0), bt);
    double v = 0.0;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            v += g[i][j] * bs[i] * bt[j];
    return v;
}

}

void CutWarp::reseed()
{
    QRandomGenerator rng(seed);
    for (int r = 0; r < kRounds; ++r)
        for (int a = 0; a < 3; ++a)
            for (int i = 0; i < 4; ++i)
                for (int j = 0; j < 4; ++j)
                    grid[r][a][i][j] = rng.generateDouble() * 2.0 - 1.0;
}

// The forward warp: the three shears, run twice, with the budget split across
// the rounds so the total excursion stays near `amount`.
void CutWarp::apply(double p[3], const double bmin[3], const double bmax[3],
                    double diagonal) const
{
    if (!active())
        return;
    const double amp = amount * diagonal / double(kRounds);

    for (int r = 0; r < kRounds; ++r)
        for (int a = 0; a < 3; ++a) {
            const int b = (a + 1) % 3, c = (a + 2) % 3;
            p[a] += amp * patch(grid[r][a],
                                normalise(p[b], bmin[b], bmax[b]),
                                normalise(p[c], bmin[c], bmax[c]));
        }
}

// Exactly the forward loops in reverse - rounds backwards, and the axes
// backwards within each round - so the same displacement comes straight off.
void CutWarp::invert(double p[3], const double bmin[3], const double bmax[3],
                     double diagonal) const
{
    if (!active())
        return;
    const double amp = amount * diagonal / double(kRounds);

    for (int r = kRounds - 1; r >= 0; --r)
        for (int a = 2; a >= 0; --a) {
            const int b = (a + 1) % 3, c = (a + 2) % 3;
            p[a] -= amp * patch(grid[r][a],
                                normalise(p[b], bmin[b], bmax[b]),
                                normalise(p[c], bmin[c], bmax[c]));
        }
}

QString CutWarp::describe() const
{
    if (!active())
        return QStringLiteral("straight cuts");
    return QStringLiteral("curved cuts %1%, seed %2")
               .arg(amount * 100.0, 0, 'g', 3).arg(seed);
}
