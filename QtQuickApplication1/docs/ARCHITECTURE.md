# Puzzle Divider — code documentation

An application that takes a 3D solid, divides it into printable pieces, plans how
they join, and checks whether the result can actually be assembled. It extends
Gershon Elber, *On the Construction of Freeform Volumetric 3D Puzzles* (2025).

The research contribution is the **assembly planner** — Elber lists automatic
assemblability verification as an open problem. The divider and the joints are
supporting machinery.

---

## Contents

1. [Layer map](#1-layer-map)
2. [Data model](#2-data-model)
3. [Loading](#3-loading)
4. [Dividing](#4-dividing)
5. [Planning](#5-planning)
6. [Joints](#6-joints)
7. [Viewing](#7-viewing)
8. [The app shell](#8-the-app-shell)
9. [Command line](#9-command-line)
10. [Building](#10-building)
11. [Known limitations](#11-known-limitations)

---

## 1. Layer map

Each arrow is a one-way dependency. Nothing below reaches back up.

```
                       main.cpp  ── CLI modes, or ──►  QML  ◄──  AppController
                                                                      │
      ┌───────────────────────────────────────────────────────────────┤
      ▼                    ▼                  ▼                       ▼
  CadLoader           PuzzleDivider      Planner::*              MeshView
  Trivariate          MeshDivider        (Graph, Blocking,       (renderer)
      │               AssemblyDivider     Order, Joints)
      │                    │              AssemblyPlanner
      ▼                    │                   │
   IritMesh                │                   ▼
   IritGuard               │              IritJoint
      │                    │                   │
      ▼                    ▼                   ▼
  ┌──────────────────── MeshData ────────────────────┐
  └───────────────────────────────────────────────────┘
                          │
                          ▼
                    IRIT C libraries
```

Two rules hold throughout:

- **`MeshData` is the only currency.** Every module takes and returns it. No
  IRIT type appears in any header except `IritMesh.h` and `IritJoint.cpp`.
- **The planner has no kernel dependency at all.** It works on the pieces'
  bounding boxes, so it can be tested with no geometry kernel in the loop.

---

## 2. Data model

### `MeshData` — `MeshData.h/.cpp`

Flat arrays, deliberately dumb. No Qt Quick types, no IRIT types.

| Field | Meaning |
|---|---|
| `pos` | 3 floats per vertex |
| `tris` | 3 indices per triangle |
| `edges` | 2 indices per edge, deduped |
| `triNrm` | unit face normal per triangle |
| `polylineEdges` | curve segments, merged into `edges` by `buildEdges()` |
| `bmin` / `bmax` | bounding box |

`finalize()` runs `computeBounds`, `computeNormals`, `buildEdges` in that order.

> **Call `finalize()` before testing `isEmpty()`.** `buildEdges()` is what moves
> polyline segments into `edges`; a curve-only file looks empty until it runs.
> That ordering was a real bug once.

Vertices are welded on **exact float equality**, with `-0.0` normalised to `0.0`
so signed zero can't split a seam. STL repeats every vertex per facet, so welding
is what turns a triangle soup into a mesh with a meaningful edge list.

### `PuzzlePiece` — `PuzzleDivider.h`

| Field | Meaning |
|---|---|
| `i, j, k` | grid cell index; for a BSP leaf, `i` is the leaf index and `j/k` are 0 |
| `p0`, `p1` | the **cell box** — parameter space for a trivariate, world space for a mesh |
| `mesh` | the piece's own geometry |
| `centre`, `size` | physical bbox centre and extent |

`p0`/`p1` is the *cell*, not the piece's bounding box. For a curved solid they
differ a lot — the cell is a box, the material inside it is not. Confusing the
two is what broke joint placement (§11).

### `CellBox` — `PuzzleDivider.h`

One leaf of a recursive split: an axis-aligned box. Still a box, so region
extraction and box clipping both work on it unchanged.

---

## 3. Loading

### `CadLoader` — one entry point for every format

```cpp
static bool load(const QString &path, MeshData *out, QString *error);
```

| Format | Reader |
|---|---|
| `.stl` | `IritPrsrSTLLoadFile` — binary/ASCII auto-detected |
| `.obj` | `IritPrsrOBJLoadFile` |
| `.itd .ibd .imd` | `IritPrsrGetObjects2`, freeforms tessellated |
| `.igs .iges` | `IritPrsrIgesLoadFile` |
| `.stp .step` | recognised and refused — IRIT has no STEP reader |

**Binary STL detection** does not use the `"solid"` header text: binary files
very often start with it too. It checks the size instead — a valid binary STL is
exactly `84 + 50N` bytes with `N` at offset 80.

When a file parses but yields nothing displayable, the error names what was
actually in it. `data/sphere8.itd` reports *"Trivar (scalar E1 field, no boundary
surface)"* rather than a bare failure.

### `IritGuard` — stops IRIT killing the process

IRIT's default fatal handlers print to stderr and call `exit()`. In a GUI that
means a malformed file makes the window vanish with no message.

`installHandlers()` replaces them on all eleven linked libraries with handlers
that record the message and `longjmp` out.

```cpp
bool IritGuard::run(void *ctx, void (*fn)(void *ctx));
```

Takes a plain function pointer and a `void*` because `longjmp` cannot safely
unwind C++ frames that own destructors. **Put only POD in the context struct and
keep C++ objects outside.**

### `IritMesh` — IRIT geometry to `MeshData`

```cpp
bool tessellate(IritPrsrObjectStruct *objs, MeshData *out,
                double fineNess, QString *error);   // CONSUMES objs
```

Tessellates freeforms, harvests polygons, ear-clips them.

> `TriangleOnly` is deliberately **off** in the call to
> `IritPrsrConvertFreeFormHierachy`. That flag routes through
> `IritGeomConvertPolysToTriangles` → `IritGeomConvexPolyObjectN`, which
> access-violates on valid inputs — `data/pl_cncyl.itd` and `data/pl_sold3.itd`
> both kill the process. Ear clipping here does the job and also handles
> non-convex faces, which a triangle fan does not.

---

## 4. Dividing

Two dividers, answering different questions.

### `PuzzleDivider` — Elber's method, on a trivariate

Divides a trivariate's **parameter domain** and region-extracts each cell with
`IritTrivTVRegionFromTV`. Every piece is a genuine solid sub-trivariate. Needs a
V-rep; an STL has none.

| Function | Cuts placed |
|---|---|
| `uniform` | equal parameter steps |
| `jittered` | equal, then nudged — Elber's difficulty randomisation |
| `splitsFromArcLength` | equal *physical* arc length per cell |
| `toBuildVolume` | arc length, then tightened until cells really fit |
| `buildBspCells` | recursive split into N boxes |
| `adjacencyOfBoxes` | face-sharing pairs among arbitrary boxes |

**`toBuildVolume` tightens each axis separately.** A single shared factor made
one overshooting axis drag the other two down with it, subdividing axes that were
already inside the limit — 175 pieces where 36 would do. It also needs a
tolerance: without one, landing 0.02% over counts as failure and the loop runs
every pass and still reports failure.

The measurement is per **parameter** axis, not per world axis. Measuring the
cell's world bbox was wrong: for anything but a box, world X picks up
contributions from u, v and w at once, so an overshoot could never be attributed
to the axis that caused it.

**`buildBspCells`** picks a cell in proportion to its **volume** and always cuts
its **longest** axis, near the middle. Both matter: uniform random picking
re-splits whatever is smallest and ends with a cloud of slivers; a random axis
lets a cell get sliced the same way repeatedly into a wafer.

### `MeshDivider` — clipping the model itself

Clips the loaded mesh against world-space cells, so pieces keep the model's real
shape. This exists because a bounding-cage trivariate turns every model into a
box — a torus comes out cuboid.

```cpp
bool divide(mesh, spec, pieces, report, cutDetail);       // a grid
bool divideCells(mesh, cells, pieces, report, cutDetail); // arbitrary boxes (BSP)
```

Three things are less obvious than they look:

**No padding on the outer bounds.** Clipping keeps points exactly on a plane, so
the model's extremes survive. A padded cell leaves a sliver of empty face beyond
the material and makes the cut outline ambiguous.

**Cells buried inside solid material contain no surface triangles.** They aren't
empty, they're full. Detected by ray casting (`pointInside`) and emitted as a
full box. Without this, a fine division of a solid loses its interior.

**Capping** closes cut faces. The open boundary only gives *part* of each cut's
outline — where the solid runs into a cell corner, the rest is the cell face's
own border and no triangle exists along it. So `capCell` walks in two alternating
modes: follow the shell's open edges, then run along the face border to the next
one. Which way round the rim runs is fixed by the mesh's winding, measured once
from its signed volume; guessing per face does not work, because the loop going
the wrong way encloses the leftover strip of cell face and *that is a positively
wound region too*.

A cut face is only capped when its outline is a **simple loop**. Otherwise it is
left open and reported — never filled speculatively.

### `CutWarp` — curved cuts

Warps the model, cuts it with planes, un-warps the pieces, so the interfaces
carry the inverse of the warp. Each step displaces one coordinate by a function
of only the other two, so the Jacobian is triangular with 1s on the diagonal —
determinant exactly 1, at any amplitude. It can never fold.

Two rounds are used. In one round the x displacement depends only on (y, z), so
every cut perpendicular to x is the same surface merely shifted along.

> ⚠️ **Off by default, and it should stay off for coarse meshes.** Clipping
> creates vertices by linear interpolation along an edge *in warped space*. A
> point interpolated between two points on a curved surface isn't on that
> surface, so un-warping doesn't return it to the original flat face. On a
> 12-triangle cube with 10-unit edges the error is large enough to visibly bulge
> the outer shape. Fixing it properly means subdividing the model before warping,
> not just the cut faces.

### `AssemblyDivider` — split, test, repair

```cpp
static bool splitToTarget(const MeshData &solid, int targetPieces,
                          quint32 seed, DividedSolid *out, QString *error);
```

**Stage A only so far.** Owns no geometry: `buildBspCells`, `divideCells` and
`adjacencyOfBoxes` are all called unchanged. What it adds is the signed direction
of each contact — `adjacencyOfBoxes` reports only that two pieces meet across the
X axis, not which side, and the side is what makes a direction blocked.

It also records the union of the piece boxes against the original solid's extent,
which is the cheapest check that dividing hasn't altered the outer shape.

Stage B (test) and Stage C (repair) are unbuilt here — but see §5, the `Planner`
namespace covers much of the same ground.

---

## 5. Planning

`Planner::*` — four stages, no kernel dependency.

### Stage 1 — `PlannerGraph`

`Planner::build(pieces, eps)` → `Graph` of `Node` (boxes) and `Contact` (real
face contacts, with side, plane position and contact area).

Directions are one fixed enum the whole planner shares:

```cpp
enum Direction { PosX = 0, NegX, PosY, NegY, PosZ, NegZ, DirectionCount };
```

### Stage 2 — `PlannerBlocking`

The Directional Blocking Graph, behind an abstract `BlockingModel`.

> **Placeholder model.** Purely translational: a piece is blocked in a direction
> if a neighbour sits on that face. Exact for straight-line withdrawal of
> axis-aligned boxes (Wilson & Latombe 1994), but it knows nothing about
> rotation, swept volume, or pieces that are in the way without touching.

The seam is deliberately wider than "does B block A" — `blocked()` takes the
whole present-piece set, because a swept-volume test must be free to consult
pieces that never touch the one being moved. A pairwise interface would have to
be thrown away when that lands.

Every model must supply `caveat()`, printed with every plan so a result is never
read as stronger than the model behind it.

### Stage 3 — `PlannerOrder`

`Planner::extract(graph, model)` → `Plan`.

Disassembly is the easier direction to search, so pieces are taken **off** and
the order reversed. The direction a piece left along is, reversed, the direction
it must be inserted along.

Greedy monotone removal, **no backtracking** — so a failure does not prove the
puzzle is unassemblable, only that this strategy didn't find an order.

### Stage 4 — `PlannerJoints`

A face contact only stops a piece leaving *through that face*. A **joint** is far
stronger: a peg in a hole slides along the peg and nowhere else, so a piece with
pegs on two different axes cannot move at all.

> That is why "a pin on every shared face" is not a puzzle — it is a welded
> block. The order has to be decided first and the joints placed to suit it.

`allContacts()` gives the naive everything-jointed set; `chooseAlongOrder()`
places joints that follow the plan instead of fighting it;
`explainOverConstrained()` says why a set deadlocks.

### `AssemblyPlanner` — the earlier, standalone planner

Spanning tree over the adjacency graph, plus approach-path and rotation-sweep
checks. Superseded by `Planner::*` for graph work, but it is where the rotation
finding came from (§11).

---

## 6. Joints

### `IritJoint` — Elber's pin, via IRIT booleans

Builds one joint solid and **booleans** it onto the piece: union for a pin,
subtract a slightly larger copy for the hole. The profile is Elber's, verbatim
from `PuzTile()` in `C:\irit\irit\scripts\puz_vol.irt` — seven circles of radius
0.2 lofted at scales 1.0 / 0.75 / 0.775 / 0.8 / 0.8 / 0.6 / 0.0. It swells then
necks, so it snaps past the mouth of the hole and is held by the neck.

Two of those sections are load-bearing for the **boolean**, not the shape:

- `Crc * tz(-0.02)` puts the first section *below* the face so the tool really
  crosses the boundary. A tool flush at z = 0 only touches, and IRIT reports
  *"objects in a subtraction operation failed to intersect"* and hands the first
  operand back unchanged.
- `Crc * sc(0.0)` closes the top. An open tube is not a solid and cannot be a
  boolean operand at all.

Pin and hole come from the **same loft at the same place** — only the operation
and the clearance scale differ, so the two always match.

Interface is `MeshData` in, `MeshData` out; only the `.cpp` needs IRIT headers.

---

## 7. Viewing

`MeshView` — a `QQuickPaintedItem` with a **software z-buffer rasteriser**. Qt
Quick 3D is not installed, and a painter's-algorithm wireframe cannot survive an
exploded assembly where pieces overlap constantly.

Everything is a list of parts: a loaded model is a one-part list, a divided puzzle
is one part per piece. Exploding, per-piece colour and hidden-surface removal all
fall out of the same path.

- **Orthographic on purpose** — no near plane means no clipping code and no way
  for a model at an awkward scale to vanish.
- **Two-sided shading and no backface culling.** Dividing by the *signed* area
  normalises the winding away, so "all three barycentric weights ≥ 0" means
  inside for front- and back-facing triangles alike. Open surfaces stay solid.
- **Depth-tested wireframe.** Edges are drawn into the same image and tested
  against the same z-buffer; drawn flat on top, the far side shows through.
- **Two edge colours, chosen per pixel.** A dark line reads as a crease on a lit
  surface but is invisible against the background, so anything still at the
  z-buffer's clear value gets the light pen.
- **Edges are dropped above one per 8 pixels.** Past that the wireframe stops
  describing the surface and just darkens it.
- **Explode** adds a fixed step along the offset direction as well as the offset
  itself, or pieces near the centre barely move.

---

## 8. The app shell

`AppController` owns the loaded mesh, the trivariate, the pieces and the cut
warp, and exposes them to QML. `main.qml` is the UI.

Three division modes:

| Mode | Input | Why |
|---|---|---|
| Uniform | Grid X/Y/Z | it *is* a grid |
| **Random** (default) | Pieces, Curve %, Seed | recursive BSP; a target count is all it needs |
| Max piece size | mm limit | count follows from the limit |

> On a **mesh**, "Max piece size" comes out evenly spaced — in world space
> `n = ceil(extent / limit)` equal slabs already satisfies a size limit, so there
> is nothing for the spacing to be non-uniform about. Unequal cuts on a mesh have
> to be imposed (Random). On a **trivariate** the same limit gives genuinely
> unequal cuts, because equal parameter steps are not equal physical sizes.

Loading is **synchronous on the GUI thread** — a large STL will visibly stall the
window.

---

## 9. Command line

Every stage is reachable without the UI, so a regression in one is never confused
with a regression in another.

```sh
--probe FILE...                          # loader only; exit code = failures
--render MODEL OUT.png                   # rasteriser only
--divide SOURCE MODE ARGS [--png OUT]    # trivariate division
--meshdivide MODEL MODE ARGS [--png OUT] # mesh division
--assemble MODEL [N] [SEED]              # Stage A: split + contact graph
```

`--divide` sources: a primitive (`Sphere`/`Torus`/`Cylinder`/`Cone`/`Box`), a
`.itd`, or `cage:MODEL`. Modes: `uniform NU NV NW`, `jitter NU NV NW PCT SEED`,
`fit BX BY BZ MAX`.

`--meshdivide` modes: `uniform NX NY NZ`, `jitter NX NY NZ PCT SEED`,
`balanced NX NY NZ`, `fit BX BY BZ MAX`, `bsp N SEED`.

Sub-flags: `--turn DEG`, `--joints`, `--pinmin`, `--sink`, `--noclear`,
`--plan`, `--full`, `--stress`.

`VOL` in `--meshdivide` compares summed piece volume against the model's. **It
only proves anything when every piece is closed** — the divergence integral over
an open shell is not a volume, so a run with open pieces can hit 100% by
accident. The line says so when that applies.

---

## 10. Building

Native `.vcxproj`, **not CMake**. Qt 6.6.2 MSVC, IRIT linked as C libraries.

- Any `Q_OBJECT` header needs **Item Type = moc** in Visual Studio or it won't
  link.
- `$(MSBuildProjectDirectory)` is on the include path so `AssemblyDivider/` can
  include the parent headers plainly.
- **Release does not link.** The prebuilt IRIT release libraries were compiled
  `/GL` with an older MSVC, so LTCG fails with `C1900` / `LNK1257`. Fix is to
  rebuild IRIT's release libs with the current toolset, or build them without
  `/GL`. Debug is unaffected.
- IRIT here uses **`Irit`-prefixed** names — `IritTrivTVDomain`,
  `IritTrivTVRegionFromTV`, `IritTrivTVEvalToData`, `IritTrivBndrySrfsFromTVs`,
  `CAGD_IS_RATIONAL_PT`. Some differ from classic IRIT. Verify against the
  headers rather than assuming.

`IritTrivTVEvalToData` writes an IRIT control-point vector: index 0 is the
weight, 1..3 are x, y, z. Divide through by the weight when
`CAGD_IS_RATIONAL_PT(PType)`.

---

## 11. Known limitations

Stated plainly, because several were found the expensive way.

**Curved solids give open shells.** All 9 pieces of a torus at N = 9 are
non-watertight; the cube's 9 are closed. Clipping a surface mesh gives surface
pieces, and capping declines whenever a cut outline is not a simple loop. An open
shell has no volume, can't be sliced for printing, and can't have a joint
booleaned into it. This is the main blocker before printing.

**Whole-piece rotation is infeasible in a snug packing.** Measured on a 24-piece
BSP cube, varying the seating rotation:

| rotation | 720° | 360° | 180° | 90° | 30° | 5° | 1° |
|---|---|---|---|---|---|---|---|
| collisions | 68 | 68 | 56 | 45 | 45 | 43 | **42** |

It still fails at one degree. A cuboid's corners sit at the half-diagonal, which
is strictly greater than the half-width, so any nonzero rotation swings them into
whatever is face-adjacent. Clearance doesn't rescue it — 0.2 mm on a piece with
r ≈ 3.5 mm buys about 3°. **Reducing the angle helps at the margin but never
crosses the line.** The way out is to stop rotating the *piece*: a captive collar
or separate spiral key that turns while both pieces stay still.

*Caveat:* that check uses the AABB of sampled poses, which over-estimates, and
assumes zero clearance. It is conservative — it can reject an assembly that would
fit, never accept one that jams. The real swept-volume check would settle it.

**Joint placement on curved pieces is unsolved.** The removed `JointGeometry`
placed joints on the **cell bounding-box** face. For box pieces that is the
contact surface; for a torus segment the box is mostly empty space, so joints
landed beside the material. For clipped curved pieces the contact surface isn't a
flat rectangle at all — it's a curved patch whose outline is whatever the model
cut out of the cell face, and it can be disconnected.

**The blocking model is translational.** No rotation, no swept volume, no
non-touching obstruction. Any result is *"assemblable under translational DBG;
twist/swept-volume check pending"* — never "guaranteed collision-free".

**Order extraction is greedy with no backtracking.** A failure does not prove a
puzzle unassemblable.

**Piece count is a target, not exact.** Cells falling outside the model hold no
material and are dropped.

**Scalar (E1) trivariates have no geometry.** `data/sphere8.itd` and
`sphere16.itd` are volumetric functions with no boundary surface. Correctly
refused, not a bug.
