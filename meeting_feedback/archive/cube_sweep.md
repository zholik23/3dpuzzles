# The cube sweep — R isolated

## Why the cube is the right experiment

The sphere shell is **already curved at R = 0**, so any locking it shows is a mix
of the base shape and the randomisation, and the two cannot be separated.

A cube trivariate is **affine at R = 0**. It must therefore reproduce the flat
control exactly — and it does:

| | median free dirs | blocked | mobile |
|---|---|---|---|
| flat control (Box primitive) | 2000 | 0 | 8 |
| **cube trivariate, R = 0** | **2000** | **0** | **8** |

2000 of 4000 is exactly half the sphere of directions, which is what a perfectly
planar interface must allow. So the cube starts from a provably flat baseline,
and **everything that follows is attributable to R alone**.

That is also a second independent validation of the analyser, on a path that does
not touch the built-in primitive.

## The sweep

| R | det J interior | blocked pairs | median free dirs | mobile pieces | single key | valid? |
|---|---|---|---|---|---|---|
| 0.00 | 1.000 | 0 | 2000 | 8 | no | yes |
| 0.02 | 0.911 | 0 | 1879 | 8 | no | yes |
| 0.05 | 0.782 | 0 | 1704 | 8 | no | yes |
| 0.10 | 0.591 | 0 | 1437 | 8 | no | yes |
| 0.15 | 0.396 | 1 | 892 | 5 | no | yes |
| 0.20 | 0.389 | 1 | 795 | 3 | no | yes |
| 0.21 | 0.248 | 1 | — | 0 | no | yes |
| **0.22** | **0.202** | **3** | — | **1** | **YES** | **yes** |
| **0.23** | **0.235** | **2** | — | **1** | **YES** | **yes** |
| 0.24 | 0.291 | 3 | — | 2 | no | yes |
| 0.25 | 0.202 | 2 | 371 | 0 | no | yes |
| 0.30 | **−0.043** | 1 | 189 | 1 | (yes) | **NO — folds** |
| 0.35 | 0.002 | 6 | 56 | 0 | no | marginal |

## The result

**A cube, with only the interior of the map randomised and no joints anywhere,
gives a single-key interlocking puzzle at R = 0.22 and R = 0.23, with a positive
Jacobian throughout the interior.**

This is the strongest form of the claim, because the R = 0 row proves the
starting point was flat and freely separable.

## The Jacobian criterion earned its keep

R = 0.30 reports a single key — and is **rejected**, because its interior
Jacobian is −0.043: the map folds. Without that check it would have been the
headline, and it would have been wrong.

That is a concrete argument for keeping "positive Jacobian of M∘R" as an
acceptance criterion rather than an afterthought.

## Important caveat: this is not a dose-response curve

`RandomizeInterior(TV, R)` calls `random(-R, R)` afresh for every control point
**and every call**, drawing new numbers from one global stream. So R = 0.21 and
R = 0.22 are **different random shapes**, not the same perturbation at two
magnitudes.

That is why mobility jitters — 3, 0, 1, 1, 2, 0 — rather than descending cleanly.
The broad trend across the whole sweep (2000 → 56 median free directions) is
real, but no single step between neighbouring R values should be read as
causation.

**Fix for the next round:** draw the unit offsets once, store them, and scale
that one fixed perturbation by R. Then the sweep becomes a genuine curve for a
single shape, and "the window is R ∈ [0.22, 0.23]" becomes a statement about that
shape rather than about six unrelated draws.

Until that is done, the honest claim is:

> At R ≈ 0.2, randomised cube trivariates produce single-key configurations with
> a valid Jacobian — found in 2 of the 6 draws sampled in that range.

## Compared with the sphere

The sphere was non-monotonic (R = 0 was *more* locked than R = 0.05) because its
base curvature already locks pieces before any randomisation. The cube shows that
this was a property of the sphere's geometry, **not** of R. R itself behaves the
way one would expect: more perturbation, fewer escape directions, until the map
folds.
