# Verification — is the script actually correct?

"Exit code 0" is not correctness. This is what was checked and what it found.

## First, what the results depend on

**The analyser never reads the tile files.** It loads the *trivariate* and
performs its own 2×2×2 division of the parameter domain. So any bug in the tile
generation (`PuzPlainTile`, `tdeform`, the 6 MB `puz2x2x2_*.itd` files) cannot
affect the blocking table. The correctness question reduces to:

1. are the saved trivariates what we think they are, and
2. is the analysis of them right.

## 1. The trivariates — checked, correct

From the `.itd` header of `tv2x2x2_a_r005.itd`:

```
[TRIVAR BSPLINE 13 8 4   4 4 2   E3
  [KV 0 0 0 0 0.25 0.25 0.25 0.5 0.5 0.5 0.75 0.75 0.75 1 1 1 1]
  [KV 0 0 0 0 0.25 0.5 0.5 0.75 1 1 1 1]
  [KV 0 0 0.33333 0.66667 1 1]
```

- **Domain is [0,1]³ on all three axes.** So cutting at 0 / 0.5 / 1 for a 2×2×2
  division is correct, and the tiles (scaled by `sxyz(1/2,1/2,1/2)` from a
  [0,2]³ grid) land exactly on the domain.
- 13 × 8 × 4 control points, orders 4, 4, 2.
- **Randomisation confirmed**: 132 control-point lines differ between `r000` and
  `r020`, while the **boundary control points are byte-identical** — so the outer
  shape is preserved and only the interior warps, which is what `R` is supposed
  to do.

### One oddity in Elber's `RandomizeInterior`, inherited verbatim

```
if ( x != 0 && x != nth( Sizes, 1 ) - 1, x = x + random( -R, R ) ):
```

This compares a control point's **geometric x coordinate** against the **mesh
size** (`Sizes` is `ffmsize`, i.e. 13). That test only makes sense for a
trivariate whose control points sit on an integer lattice, such as a box from
0..n−1. For a sphere shell the coordinates are values like −0.138, so the guard
never fires.

It is **harmless here**: the loop bounds `for i = 1, 1, uLen - 2` already exclude
the boundary control points, which is why the boundary came out byte-identical.
Worth knowing before reusing the function on a different trivariate.

## 2. The analyser — a real bug, found and fixed

The Jacobian was sampled at **cell centres only**, `(i+0.5)/g`, so it never
touched the domain boundary. Shape A is a surface of revolution, which is
**degenerate at its poles** — and the poles sit exactly on the boundary `v = 0`
and `v = 1`. A centre-only grid reports a healthy Jacobian for a map that is
singular there.

Fixed: the grid now includes the boundary, records **where** the minimum occurs,
and reports the strictly-interior minimum separately.

### What the corrected check shows

| case | min det J (with boundary) | where | min det J (interior) | reading |
|---|---|---|---|---|
| Box | 8 | — | 8 | flat, healthy |
| Cylinder | 4.36e−05 | (0.000, 0.000, 0.333) | 1.445 | axis degeneracy, healthy inside |
| A R=0.00 | −5.9e−19 | (0.417, **0.000**, 1.000) | **0.2823** | pole; numerically zero, not a fold |
| A R=0.05 | −3.8e−17 | (0.417, **0.000**, 0.333) | **0.2447** | pole; valid case |
| A R=0.20 | **−0.6725** | (0.750, 0.667, 0.583) | **−0.6725** | **genuine fold, well inside** |

The distinction matters and only became visible after the fix:

- **−5.9e−19 and −3.8e−17 are machine zero**, located exactly at `v = 0`. That is
  the pole of a surface of revolution — intrinsic to the representation, present
  in any sphere-like trivariate, and *not* the map folding over itself.
- **−0.6725 at (0.75, 0.667, 0.583)** is deep in the interior and is a genuine
  fold. R = 0.20 is properly invalid.

So the **interior minimum is the meaningful fold test**, and the boundary value
tells you about representation degeneracies instead.

## 3. Do the conclusions survive?

**Yes, and they are better supported now.**

- **Single key at shape A, R = 0.05** — stands. Interior det J = 0.2447 > 0, so
  the map is valid where it matters.
- **R = 0.20 excluded** — stands, and is now *demonstrated* to be a real interior
  fold rather than assumed from a boundary artefact.
- **The cylinder result** — unaffected. Interior det J = 1.445, perfectly
  healthy, and it still falls apart with 0 blocked pairs. The single-curvature
  finding in `base_shapes.md` is the strongest result here and does not depend on
  the Jacobian at all.

## 4. Still unverified

- The earlier "Sphere and Torus have negative Jacobian" claim was made with the
  old centre-only sampling and has **not** been re-examined with the
  interior/boundary split. Their −2.376 and −0.973 are far too large to be
  machine-zero pole effects, so they are probably genuine — but that should be
  re-run before being quoted.
- The deformed tile files have not been inspected geometrically. They do not feed
  the results, but they are what would be viewed in GuIrit, so they should be
  opened once before showing them to anyone.
- Still one random seed throughout.
