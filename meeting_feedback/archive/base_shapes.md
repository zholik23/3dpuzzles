# Base shapes — and a correction to the first conclusion

## Why this file exists

The first run used two "shapes" that were the same sphere shell with the inner
surface scaled differently:

```
ShapeA = MakeShellTV( 0.25, 0.5 )
ShapeB = MakeShellTV( 0.50, 0.25 )
```

That is one shape twice, not "a couple of base shapes". Rerun on genuinely
different trivariates, the conclusion needs refining.

## Genuinely different base shapes, 2×2×2, no pins

| base shape | min det J | pairs | blocked | median free dirs | mobile pieces | verdict |
|---|---|---|---|---|---|---|
| Box | 8 | 12 | 0 | 2000 | 8 | flat — falls apart |
| **Cylinder** | 0.766 | 12 | **0** | **2000** | **8** | **curved, and still falls apart** |
| Cone | 0.0022 | 12 | 3 | 2000 | 4 | partly blocked |
| Sphere | **−2.376** | 12 | 4 | 1027 | 0 | *invalid — map folds at the poles* |
| Torus | **−0.973** | 12 | 0 | 797 | 4 | *invalid — map folds; also genus 1* |
| sphere shell (shape A, R=0.05) | 0.101 | 12 | 3 | 1859 | **1** | **single key** |

## The correction

**"Curvature creates locking" is too strong. It has to be the right kind of
curvature.**

The cylinder is the case that proves it. It is genuinely curved in R³, its
Jacobian is healthy (0.766), and it still behaves *exactly* like the flat box:
0 blocked pairs, 8 mobile pieces, median 2000 free directions — the flat-interface
value.

The reason is visible in the geometry. Cutting a cylinder's parameter domain
2×2×2 gives three kinds of interface:

- cuts in the **angular** parameter → flat radial planes
- cuts in the **height** parameter → flat discs
- cuts in the **radial** parameter → cylindrical surfaces

The cylindrical ones are curved, but their normals all point radially, so they
all lie **perpendicular to the cylinder's axis**. Every one of them satisfies
`d·n = 0` for `d` along the axis. So the piece simply slides out along the axis,
and the curvature buys nothing.

That is **single curvature**: the normals spread in one direction only. To close
off every escape you need the normals to spread in **two** directions —
**double curvature**, which is what a sphere shell's radial interfaces have.

## The claim, restated precisely

> An interface blocks translation when its normals spread in two independent
> directions. Single-curvature interfaces (cylinders, cones, anything
> developable) leave an escape along the ruling direction, exactly as a plane
> leaves an escape along its normal.

This is a sharper and more useful statement than "curvature locks", and it says
something actionable about **which trivariates are worth using**: a base shape
whose parameter lines are developable will not interlock, however curved it
looks.

## Also worth noting

**Two of the five primitives have a negative Jacobian.** Sphere folds at the
poles (a sphere trivariate has collapsed edges there) and Torus folds too. Both
are excluded, not analysed — and the fact that the check rejected two of five
standard primitives is a good argument for having it.

**The cone is marginal**, `min det J = 0.0022`, near-degenerate at the apex. It
reports 3 blocked pairs and 4 mobile pieces, but a Jacobian that close to zero
means the map is nearly singular there and the result should be treated as
suspect rather than as evidence.

## What this changes about the plan

- The base shape is not a free parameter. It decides whether interlocking is
  **possible at all**, before R is tuned.
- A useful screening test exists and is cheap: for each interface, is the normal
  spread two-dimensional? If every interface is developable, stop — that
  trivariate cannot interlock by curvature.
- The harmonic-field construction of the trivariate (Martin, Cohen & Kirby)
  becomes more interesting, not less: the question is whether harmonic level sets
  give doubly-curved interfaces.
