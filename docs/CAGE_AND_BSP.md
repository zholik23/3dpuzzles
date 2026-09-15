# The Cage and the BSP Division

How a polygonal model becomes puzzle pieces: what the trivariate cage is, how it
is built, how the BSP splits it, which IRIT functions do the work, and how the
requested piece count is now honoured exactly.

For the same pipeline with the responsible code quoted stage by stage, see
`DIVISION_WALKTHROUGH.md`.

This documents what the code does today. Where a design choice was forced by a
measurement, the measurement is given.

---

## 0. The pipeline

```
  Mp                Mc                 cells              sub-trivariates
  mesh   ────────▶  trivariate  ────▶  in parameter ────▶ tessellated to  ────▶  ∩ Mp  ────▶  pieces
  (OBJ/STL)         cage (box)         space (BSP)        boxy meshes           Boolean
   step 1            step 1             step 2              step 3               step 4
```

This is Elber's Section 5 (Figure 14). The essential idea is that **the cage does
not need to resemble the model**. The division happens in the cage's continuous
volumetric domain; the model's detail is recovered at the end by a Boolean
intersection. A crude box cage is therefore legitimate, not a placeholder for
something better.

| Step | Entry point | File |
|---|---|---|
| 1. Build the cage | `Trivariate::boundingCage` | `Trivariate.cpp` |
| 2. Split the domain | `PuzzleDivider::buildBspCells` | `PuzzleDivider.cpp` |
| 3. Extract + tessellate | `PuzzleDivider::divideCells` | `PuzzleDivider.cpp` |
| 4. Intersect with Mp | `CageBoolean::intersectAll` | `CageBoolean.cpp` |

Orchestrated by `AppController::divideRandom` → `runTrivCellDivision`.

---

## 1. Step 1 — the trivariate B-spline cage

### What a trivariate is

A trivariate B-spline is a volume-valued function

```
    S(u, v, w) → (x, y, z),        (u, v, w) ∈ [0,1]³
```

a tensor product of B-spline bases in three parameters. Where a surface maps a 2D
patch into space, a trivariate maps a 3D *block* into space. That matters here
for one reason: **a box in parameter space is a solid, not a shell.** Slicing the
parameter domain gives solid sub-volumes for free, with no capping or
watertightness problem to solve.

### How it is built

`Trivariate::boundingCage(mesh, error)`:

1. Take the mesh's axis-aligned bounds, `MeshData::bmin` / `bmax`.
2. Pad any axis whose extent is below `1e-9` by 0.5% of the model diagonal. A
   flat plate or a planar sketch would otherwise give a degenerate trivariate
   with zero thickness in one direction.
3. Call `IritTrivNSPrimBox(lo[0], lo[1], lo[2], hi[0], hi[1], hi[2])` inside
   `IritGuard::run`, so an IRIT fatal error becomes a returned error rather than
   a process exit.
4. Adopt the result through `Trivariate::adopt`, which takes ownership.

The cage that comes back is **trilinear**: orders 2/2/2, a 2×2×2 grid of control
points, domain `u[0,1] v[0,1] w[0,1]`. The app prints exactly this in its status
line.

### Two consequences of it being a box

**It is affine and axis-aligned.** The map from parameter space to world space is
diagonal — `x = lerp(bmin[0], bmax[0], u)` and so on. A box in parameter space is
therefore a box in world space. Everything in Step 2 relies on this.

**It has no interior freedom.** All 8 control points are corners of the box.
There is nothing to pull inward, so this cage cannot be shrink-wrapped as it
stands. Tightening it (Elber's Figure 14b) would first require
`IritTrivTVDegreeRaise` and `IritTrivTVRefineAtParams` to create an actual
control grid, then closest-point projection of the boundary layer, with
`IritTrivTVEvalJacobian` checked after each move to keep the parameterisation
valid.

### Why not fit a trivariate to the mesh directly

Because it does not work on the models in question. `IritTrivFitTV2PolyMesh`
needs tube topology plus a medial axis, and cannot produce a conforming
trivariate for a limbed model such as the armadillo. A genus-0 bust is more
tractable. But the fit is not needed: **the Boolean in Step 4 supplies the
detail**, which is the whole point of Elber's construction.

---

## 2. Step 2 — BSP division of the parameter domain

### Principle

Binary space partitioning. Begin with one cell covering the whole domain.
Repeatedly choose a leaf cell and cut it with a single axis-aligned plane into
two children. Each cut converts one leaf into two, so **n−1 cuts produce exactly
n leaves**. The leaves are the cells; they tile the domain with no overlap and no
gap.

### Why not a grid

A grid is built from *global* cut planes that run the full width of the model.
Two things follow, and both are wrong for a puzzle:

- Piece sizes are locked to rows and columns — change one plane and a whole slab
  of pieces changes with it.
- Every interior piece has exactly six neighbours.

Splitting one cell at a time frees the sizes from each other and makes the
adjacency irregular: a large piece can border several small ones. A grid can
never do that. It also means an arbitrary piece count is reachable, where a grid
only reaches products `nu × nv × nw`.

### The loop, in `buildBspTree`

```
while (leaves.size() < targetPieces):
    pick the leaf holding the most MODEL material
    try axes longest-first
    score 24 candidate planes, cheapest cost wins
    reject a cut that empties or severs a half
    if no axis worked: mark the leaf "exhausted"
```

Each decision is there for a reason:

**Leaf choice is weighted by volume.** Picking uniformly at random keeps
re-splitting whatever is already smallest, ending with a cloud of slivers beside
one untouched block. Weighting by volume drives the split toward the big cells.

**Axes are tried longest first.** Cutting the longest axis keeps pieces blocky
rather than slab-like. Falling through to a shorter axis is what still allows a
cell that is already thin in its longest direction to be divided at all.

**The cut position comes from the cost, not from a draw.** Candidates span
`[minSide/extent, 1 − minSide/extent]` — the whole range the printability floor
allows — and the cheapest is taken. The jitter window that used to bound this is
gone: it was the same arbitrary 15–85% threshold expressed as a position, and it
would have quietly bounded the search.

**`minSide` is derived, not tuned.** When passed 0 it defaults to

```
    minSide = 0.45 · ∛(V_root / targetPieces)
```

— 45% of the side of an average piece. It scales with both the model and the
requested count, so it needs no units and no retuning when either changes. A cell
is cut only where **both** halves clear it.

**Exhaustion is tracked.** A cell too short on every axis is flagged and given
weight 0, so the loop stops reconsidering it. When no cuttable leaf remains, the
loop breaks — deliberately finishing under target rather than manufacturing
slivers.

### The split follows the material, not the cage

Everything above describes *where* cells are cut. What decides it is the
`MaterialField` — a voxelisation of Mp with a 3D prefix sum, so the material
inside any axis-aligned box costs eight lookups.

Without it the splitter has never seen the model, and it will happily place a
cell in thin air: an armadillo fills only 10.7% of its own bounding box. With it,
three things change:

1. **The cell holding the most material is split, every time.** Weighting by
   material and then *sampling* is not enough: it lets the biggest cell simply
   never come up again, and on the armadillo at 6 pieces one piece kept 80% of
   the model because five of the cuts landed elsewhere. Always taking the
   largest bounds that — a piece can only stay large if it was large one cut
   ago, and it will be picked again. A cell with no material has weight 0 and is
   never chosen at all.
2. **The cut position minimises a cost, with no threshold anywhere.** 24 planes
   are scored across the cell and the cheapest is taken. The live term is
   balance — `|left − right| / cellMaterial` — whose optimum is an even split,
   worked out per cell from the material actually present. This replaced an
   earlier rule that accepted any plane leaving each side between 15% and 85% of
   the material, which was a threshold nobody could defend and which chose
   randomly inside its window. The cost is a weighted sum so further terms
   (thinness, disconnection) can be added; they are stubbed at weight 0.
3. **A cut is accepted only if both halves keep material** above a floor of
   `max(4 voxels, 5% of an average piece)`. If not, the next axis is tried.
4. **A cut that would sever a piece is rejected.** `MaterialField::isConnected`
   flood-fills the voxels inside each half; if either comes back as more than one
   lump, the cut is refused. This is done in two passes — the first demands
   connected halves, the second drops that requirement if no axis could satisfy
   it — because a severed piece can be repaired afterwards whereas a missing one
   cannot. It is resolution-limited: a connection thinner than a voxel is not
   seen, so downstream repair in `CageBoolean` stays as the backstop.

**The guarantee.** The root cell holds all the material, and every accepted cut
leaves material on both sides, so by induction **every leaf holds material**.
There are no empty cells to drop, and the piece count comes out as asked.

Measured on the armadillo — the worst case, since it is nearly 90% air:

| Asked | Cells | Pieces | Dropped |
|---|---|---|---|
| 6 | 6 | **6** | 0 |
| 7 | 7 | **7** | 0 |
| 10 | 10 | **10** | 0 |

Balancing material also narrows the size spread, since that is what the cut is
now balancing. On the armadillo at 6 pieces:

| Splitter | Smallest | Largest | Spread |
|---|---|---|---|
| by cage volume | 496 | 87,092 | 175× |
| by material, sampled leaf | 4,486 | 117,650 | 26× |
| by material, largest leaf, random share | 24,167 | 59,272 | 2.8× |
| by material, largest leaf, minimum cost | 30,073 | 57,314 | **1.91×** |

### Splitting in world proportions

`AppController::divideRandom` does not run the BSP on `[0,1]³` directly. It:

1. builds a domain whose extents are the model's **world** extents,
2. runs `buildBspCells` on that,
3. maps each cell back to parameter space per axis.

Without this, "cut the longest axis" and the `minSide` floor would be measured in
parameter units, where every axis is 1 long regardless of the model's real shape
— and a parameter-cubic cell would come out as a long world slab on any model
whose bounding box is not a cube.

This is exact **only because the cage is a box** (Section 1). With a deformed
cage, a parameter box maps to a curved region of varying size and this mapping
would have to go through the Jacobian instead.

### Functions

| Function | Role |
|---|---|
| `buildBspTree(domain, target, jitter, seed, minSide)` | the split, kept as a tree |
| `leavesOf(tree, boxes)` | flattens the tree to cells |
| `buildBspCells(...)` | the two above, combined |
| `collapse(tree, node)` | merges a leaf and its sibling back into the parent box exactly — used by the mesh path to absorb crumbs |
| `splitLeaf(tree, node, ...)` | re-splits an oversized leaf |
| `adjacencyOfBoxes(pieces, eps)` | geometric neighbours: shared plane **plus** real overlap area, so edge and corner contact is correctly rejected |

The tree is kept rather than discarded because a cell and its sibling always
merge back into their parent box *exactly*. Two arbitrary adjacent cells have no
such box, which is what makes crumb absorption possible at all.

---

## 3. Step 3 — cells to solid meshes

`PuzzleDivider::divideCells` walks the cells. For each, `doRegion` narrows the
trivariate one axis at a time:

```c
IritTrivTVRegionFromTV(cur, p0[a], p1[a], TRIV_CONST_U_DIR);   // then V, then W
```

Three calls, each consuming the previous result, leaving a sub-trivariate for
that cell. All of it runs inside `IritGuard::run`.

Each sub-trivariate is then tessellated by `Trivariate::tessellate`:

```
IritTrivTVCopy  →  IritPrsrGenTRIVARObject  →  IritMesh::tessellate
                                            →  IritPrsrConvertFreeFormHierachy
```

The copy is necessary because `IritPrsrGenTRIVARObject` takes ownership and
`IritMesh::tessellate` then consumes the object.

### The trap in this step

What comes back is the trivariate's **six boundary surfaces**, tessellated
independently — *not* an oriented solid. For a box cage cell that is 12 triangles
which look right and are not: neighbouring faces disagree about which way is out,
and the mesh is not closed.

Measured on a sphere at 8 pieces, before repair: every cell reported
`closed NO`, with signed volumes ranging from −82% to +129% of the box the cell
is supposed to fill exactly.

`IritSolid::orientConsistently` repairs this, and must run before any Boolean:

1. `weldClose` merges vertices within `1e-6 × diagonal`. IRIT evaluates each
   boundary surface separately, so a corner reached from the u face and from the
   v face differs in the last few bits; `MeshData::finalize` welds on exact
   equality only, which is not enough here.
2. A breadth-first walk over shared edges flips any triangle that traverses a
   shared edge in the same direction as its neighbour.
3. Each connected shell is then flipped if its own signed volume is negative —
   **per shell, not globally**. A piece can be several disjoint lumps, and a
   single global test lets a negative lump hide inside a larger positive one.
   That bug cost exactly two lump-volumes on a later merge before it was found.

After repair, every cell measures exactly 100% of its box.

---

## 4. Step 4 — the Boolean intersection

`CageBoolean::intersectAll`, one `IritBooleanAND(piece, model)` per cell, each
inside `IritGuard::run`, each against a **fresh copy** of the model because IRIT's
booleans consume their operands.

**Winding is inverted on purpose.** `IritSolid::fromMesh(mesh, Winding::Inward)`
— IRIT builds a polygon's plane from its vertex order, and the sign is the
opposite of the right-hand rule that `signedVolume` uses. Feeding IRIT a
conventionally outward-wound solid inverts both containment tests and the
intersection silently computes a **union**: AND of a bounding box with an
inscribed sphere returned the box (64000, every box polygon, no sphere polygon)
instead of the sphere. It reported "8 clean, 0 failed" while returning 704% of
the model volume.

The correctness check is volume: the trimmed pieces must sum to the model's own
volume. On every sound model tested they sum to exactly 100%.

**A caveat on what "the model's volume" means.** The divergence integral counts
every shell it is handed, so a mesh carrying interior geometry reads high, and
the pieces then look as though they lost material when they did not. `beast.obj`
is such a mesh. Two independent measures agree against it:

| Model | Divergence | Voxel fill | Trimmed pieces |
|---|---|---|---|
| sphere | 33,272 | 32,807 (99%) | 33,272 |
| armadillo | 237,930 | 237,710 (100%) | 237,930 |
| **beast** | 822,450 | **527,590 (64%)** | **528,310** |

The voxel fill (`MaterialField`, parity down each column) and the Boolean agree
to 0.14% on beast, and both disagree with the divergence integral by the same
36%. The division is right; the reference number is wrong. When a run reports a
low percentage, compare the two `MODEL` figures before suspecting the Boolean —
if they differ, the input mesh has interior structure and needs repair before it
is printed, not before it is divided.

Three outcomes are distinguished, and the distinction is made by the guard rather
than by guesswork — `IritGuard` replaces IRIT's fatal handler, so a real failure
always arrives as an error, and an empty result from a *successful* call means
the cell and the model really are disjoint:

| Outcome | Meaning | What happens |
|---|---|---|
| `Ok` | real geometry | becomes the piece |
| `EmptyCell` | cell holds no material | **piece dropped** |
| `Failed` | boolean errored | piece keeps its boxy cage shape, reported |

Afterwards each result is split into connected components
(`CageBoolean::components`), because a convex cell intersected with a non-convex
model can return several disjoint lumps. Slivers below `1e-5 × model volume` are
discarded as numerical debris; the largest component becomes the piece; any other
real lump is reattached to the neighbour it touches. `multiPart` re-checks the
finished pieces and must be 0.

---

## 5. Piece counts, and why they used to come up short

**The BSP loop is exact.** It is `while (leaves.size() < targetPieces)` and every
split adds exactly one leaf, so it produces exactly N cells whenever the geometry
allows:

```
CAGE  asked 10 -> 10 BSP cell(s) -> 10 boxy cage piece(s)
```

### The old failure: empty cage cells

Before the split became material-aware, cells were chosen and cut by **cage
volume**. The cage is a box and the model is not, so a cell could contain no
material at all. The Boolean correctly returned nothing and the piece was
dropped, giving 5 pieces for a request of 6:

```
V-rep · recursive split · 6 of 6 cells · 5 of 6 cells filled
```

Whether it happened depended on how much of its bounding box the model fills:

| Model | Model ÷ cage volume | Empty cells at 10 pieces (old) |
|---|---|---|
| sphere | 52% | 0 |
| armadillo | **10.7%** | 1 |

An armadillo is mostly air — limbs, a tail, the gaps between them — so a cell
landing in nothing was likely. Asking for *fewer* pieces made it worse, because
cells were larger and reached further into the corners.

**This no longer happens.** The material-aware split cannot produce an empty
cell (Section 2), so nothing is dropped and the count is exact.

### What can still change the count

Two things remain, both reported in the log:

**The minimum-size floor.** If no leaf can be cut with both halves clearing
`minSide`, the loop breaks early:

```
below target: the rest would have been under the minimum piece size
```

**Connectivity repair, which can raise the count.** A detached lump merged into a
neighbour changes nothing, but a lump touching nothing is kept as its own piece:

```
connectedness: 2 sliver(s) discarded as noise, 0 unioned + 5 welded, 1 kept standalone
```

### Reading the log

`CAGE asked N -> M BSP cell(s)` tells you whether the splitter met the target; if
`M < N` it is the size floor. If `M == N` but a different number of pieces comes
out, the `BOOLEAN` line says why — `dropped`, `failed`, or `kept standalone`.

## 6. Known limitations

**(a) Every cut is an axis-aligned plane.** BSP splits only on axis-aligned
planes, and the cage is a box, so nothing about the division is freeform. The
V-rep machinery is genuine but its shape freedom is unused until the cage hugs
the model.

**(b) Piece sizes are bounded, not equalised.** Splitting the largest cell and
minimising the imbalance cost brought the armadillo at 6 pieces to a **1.91×**
spread (from 175×). What remains comes from the cut having to be placed on a
plane that also keeps both halves connected and above the printable floor, so
the cheapest candidate is not always available — the logs show this as a cut
chosen at "rank 2 of 24". Tightening it further means adding a term to the cost,
not adjusting a window.

**(c) Debris is discarded, not merged.** A component below 0.5% of an average
piece is dropped rather than promoted to a piece of its own — a 29-unit speck
against a 238,000 model cannot be printed or handled. The volume this throws away
is reported whenever it exceeds a rounding error (0.013% on the armadillo at
seed 5). The alternative, keeping it, is what used to turn a request for 6 pieces
into 7.

**(d) The reported adjacency graph is wrong for BSP pieces.**
`AppController::describePieces` calls `PuzzleDivider::adjacency()`, which finds
neighbours by stepping grid coordinates `(i+1, j, k)`. But `divideCells` stores a
*linear* index in `i` with `j = k = 0`, so the result is a chain of exactly `n−1`
edges — visible in the UI as "9 of 10 cells filled · 8 shared faces". The correct
routine, `adjacencyOfBoxes()`, exists and is simply not called on this path. This
matters because the blocking graph and assembly order run on that adjacency.

---

## 7. Function reference

| Function | File | Role |
|---|---|---|
| `IritTrivNSPrimBox` | IRIT `triv_lib` | builds the box trivariate cage |
| `IritTrivTVRegionFromTV` | IRIT `triv_lib` | extracts a sub-trivariate for one cell, one axis per call |
| `IritTrivTVCopy` / `IritTrivTVFree` | IRIT `triv_lib` | ownership around the tessellator |
| `IritPrsrGenTRIVARObject` | IRIT `prsr_lib` | wraps a trivariate as an object (takes ownership) |
| `IritPrsrConvertFreeFormHierachy` | IRIT `prsr_lib` | tessellates to polygons (consumes the object) |
| `IritBooleanAND` | IRIT `bool_lib` | the Section 5 intersection |
| `IritBoolSetFatalErrorFunc` | IRIT `bool_lib` | lets `IritGuard` trap failures |
| `IritTrivTVDegreeRaise`, `IritTrivTVRefineAtParams`, `IritTrivTVEvalJacobian` | IRIT `triv_lib` | *not yet used* — needed for a tight cage |
| `Trivariate::boundingCage` | `Trivariate.cpp` | Step 1 |
| `PuzzleDivider::buildBspCells` / `buildBspTree` | `PuzzleDivider.cpp` | Step 2 |
| `MaterialField::build` / `volumeIn` | `MaterialField.cpp` | voxel grid + 3D prefix sum; material in any box in O(1) |
| `PuzzleDivider::divideCells` | `PuzzleDivider.cpp` | Step 3 |
| `PuzzleDivider::adjacencyOfBoxes` | `PuzzleDivider.cpp` | correct neighbour graph |
| `CageBoolean::intersectAll` | `CageBoolean.cpp` | Step 4 |
| `CageBoolean::components` | `CageBoolean.cpp` | connectivity split |
| `IritSolid::orientConsistently` / `weldClose` | `IritSolid.cpp` | repairs tessellator output |
| `IritSolid::fromMesh` | `IritSolid.cpp` | MeshData → IRIT, with explicit winding |
| `IritGuard::run` | `IritGuard.cpp` | traps IRIT fatal errors **and** `SIGABRT` |
| `AppController::divideRandom` | `AppController.cpp` | orchestrates, maps world ↔ parameter |
| `PieceExport::save` | `PieceExport.cpp` | writes `.itd` / `.obj` / `.stl` |
