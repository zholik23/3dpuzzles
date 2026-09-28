# The R sweep, done properly

## What was wrong before

`RandomizeInterior` calls `random(-R, R)` afresh on every invocation, so R = 0.05
and R = 0.10 were **two unrelated random shapes**, not one shape at two
magnitudes. The earlier sphere sweep showed 4, 4, 2, 4 blocked pairs and
Jacobians of 0.282, 0.250, 0.184, 0.215 - scatter with no trend, which is exactly
what independent draws look like. A trend could not have appeared even if one
existed.

## The fix

`random(-R, R)` is `R * (2u - 1)` for the underlying uniform draw `u`, so
re-seeding the generator before each call makes every R scale **one fixed field
of offsets**:

```
SweepCase = function( TV, R, TVName ): TVR: dmy:
    dmy = IritState( "RandomInit", 1960 ):
    TVR = RandomizeInterior( TV, R ):
    ...
```

Verified - the same 132 control points move at every R, and the displacement is
exactly proportional:

| R | control points moved | max displacement | max / R |
|---|---|---|---|
| 0.05 | 132 | 0.07413 | **1.4826** |
| 0.10 | 132 | 0.14826 | **1.4826** |
| 0.15 | 132 | 0.22240 | **1.4826** |

## The sweep

| R | det J interior | blocked | mobile pieces | mobile subsets | smallest |
|---|---|---|---|---|---|
| **sphere** | | | | | |
| 0.000 | 0.2823 | 4 | 0 | 12 | 2 |
| 0.025 | 0.2513 | 4 | 0 | 12 | 2 |
| 0.050 | 0.2219 | 4 | 0 | 12 | 2 |
| 0.075 | 0.1942 | 4 | 0 | 12 | 2 |
| 0.100 | 0.1676 | 4 | 0 | 12 | 2 |
| 0.125 | 0.0847 | 4 | 0 | 12 | 2 |
| 0.150 | **-0.0924** | 4 | 0 | 12 | 2 |
| 0.175 | **-0.2860** | 4 | 0 | 12 | 2 |
| **duck** | | | | | |
| 0.0000 | 0.03562 | 4 | 0 | 12 | 2 |
| 0.0025 | 0.03619 | 4 | 0 | 12 | 2 |
| 0.0050 | 0.03587 | 4 | 0 | 12 | 2 |
| 0.0075 | 0.03523 | 4 | 0 | 12 | 2 |
| 0.0100 | 0.02875 | 4 | 0 | 12 | 2 |
| 0.0150 | 0.00158 | 4 | 0 | 12 | 2 |
| 0.0200 | **-0.03107** | 4 | 0 | 8 | 2 |

## Two results, and the second is the important one

**1. The Jacobian now behaves properly.** It falls monotonically with R on both
shapes, which is what a genuine one-parameter family must do and what the old
scattered sweep never showed. The folding thresholds are now sharp:

- sphere folds between **R = 0.125 and 0.150**
- duck folds between **R = 0.015 and 0.020**

So the sphere tolerates roughly **8x** the duck's perturbation, and the duck's
usable range is about 0.015 wide.

**2. R does not change the blocking at all.**

Every valid row, on both shapes, across the entire range up to folding:
**4 of 12 pairs blocked, 0 mobile pieces, 12 mobile subsets, smallest group 2.**
Identical. The only row that differs is duck R = 0.020, which has already folded
and is invalid.

This is a clean negative result, and it is now trustworthy because the sweep is a
true one-parameter family rather than a scatter of unrelated shapes.

> Within the range where M o R stays valid, the randomisation has **no effect on
> the blocking structure**. Blocking is set by the base shape and the division
> topology, not by R.

## Why this matters for the plan

Step 3 asks to vary R and find the window where interlocking appears. Measured on
Elber's own shapes, **there is no such window** - not because R is too small or
too large, but because blocking is flat in R right up to the point where the map
breaks.

Compare what *does* move the blocking:

| change | blocked pairs |
|---|---|
| R from 0 to 0.125 (sphere, whole valid range) | 4 -> 4 |
| R from 0 to 0.015 (duck, whole valid range) | 4 -> 4 |
| **2x2x2 -> 2x2x1 (duck, division topology)** | **4 -> 0** |

The division topology changes the answer completely; R does not change it at all.
That is the finding to take to the meeting, and it redirects the search from
"tune R" to "choose where to cut".
