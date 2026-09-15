# Building the assembly check

A guide for wiring the three stages you asked about — adjacency, directional
blocking, and the peeling order test — into the cage/V-rep pipeline.

**Read this first: all three already exist in your repo, fully written.** They
were built on 2026-09-02 and they do exactly what you described. What is missing
is that they are never run on the cage path. That is the work.

---

## What exists, and where

| Stage | You described it as | Lives in | Entry point |
|---|---|---|---|
| 1 | which pieces share a real face | `PlannerGraph.h/.cpp` | `Planner::build(pieces, eps, minArea)` |
| 2 | which directions each piece can move | `PlannerBlocking.h/.cpp` | `Planner::TranslationalBlocking` |
| 3 | the peeling test | `PlannerOrder.h/.cpp` | `Planner::extract(graph, model)` |

There is also `plannerCli(args, pieces)` in `main.cpp:417` which runs all three
and prints the logs, and `PlannerJoints.h/.cpp`, which is stage 2 again with
joints added.

None of it depends on IRIT. The planner works on the pieces' boxes, so you can
test it without a geometry kernel in the loop.

---

## Step 0 — watch it run before you write anything

The mesh path already calls all three:

```
--meshdivide MODEL bsp 8 --plan          # the three stages, first 8 pieces
--meshdivide MODEL bsp 8 --plan --full   # every piece
--meshdivide MODEL bsp 8 --plan --stress # force the failure branch, see below
```

Spend ten minutes reading that output before writing code. It is the thing you
are asking to build, already running, and the log format shows you what each
stage produces.

---

## Step 1 — wire it to the cage path

This is the actual missing piece, and in the CLI it is **one line**.

### The CLI

In `main.cpp`, `cageCli()` finishes the Boolean at **line 1022**:

```cpp
const CageBoolean::Result r = CageBoolean::intersectAll(&pieces, model, 12.0);
```

After the reporting that follows it, add:

```cpp
plannerCli(a, pieces);      // stages 1-3, printed; no-ops unless --plan is passed
```

`plannerCli` is already declared above `cageCli`, so nothing else changes. Then:

```
--cage MODEL 6 7 --plan
```

### The app

In `AppController::runTrivCellDivision` (`AppController.cpp:361`), the Boolean
runs at **line 374**. After it, and after `m_pieces` is final, add:

```cpp
const Planner::Graph graph = Planner::build(m_pieces, 1e-6, 0.0);
const Planner::TranslationalBlocking model;
const Planner::Plan plan = Planner::extract(graph, model);

for (const QString &line : plan.describe(8))
    qDebug().noquote() << line;
```

`AppController.cpp` already includes what it needs — `applyJoints()` at line 762
uses the same three calls — so this compiles as-is.

---

## Step 2 — fix the adjacency number the UI reports

`AppController::describePieces` (**line 452**) computes the shared-face count
with the wrong function:

```cpp
const int shared = PuzzleDivider::adjacency(m_pieces, DivisionSpec()).size();
```

`adjacency()` finds neighbours by stepping grid coordinates `(i+1, j, k)`. BSP
cells have no grid coordinates — `divideCells` stores a *linear* index in `i`
with `j = k = 0` — so it returns a chain of exactly `n−1` edges. That is why the
UI says "9 of 10 cells filled · 8 shared faces".

Change it to the geometric one:

```cpp
const int shared = PuzzleDivider::adjacencyOfBoxes(m_pieces, 1e-6).size();
```

The planner already uses `adjacencyOfBoxes`, so this only affects what the UI
displays — but the displayed number is currently meaningless.

---

## What each stage actually does

### Stage 1 — `Planner::build`

Does not re-derive the pairing. It calls `PuzzleDivider::adjacencyOfBoxes`,
which already finds boxes meeting on a plane **with real overlap there**, and
already rejects edge-only and corner-only touches. What `build` adds is what
stages 2 and 3 need:

```cpp
struct Contact {
    int    lowSide, highSide;   // lowSide is on the smaller side of the plane
    int    axis;                // 0/1/2 — the shared face's normal
    double plane;               // where that face sits along axis
    double area;                // overlap area
    double ext[2];              // the face's two side lengths
    double depth;               // thinnest of the two pieces along axis
};
```

The orientation is the whole content of the edge: **from `lowSide` the
neighbour lies along `+axis`; from `highSide`, along `−axis`.** That is your
"if B sits on A's +X side then A is blocked in +X", stored once.

`graph.describe()` prints every piece, its neighbours, and the direction each
lies in.

### Stage 2 — `BlockingModel`

Six directions, one bit each:

```cpp
enum Direction { PosX = 0, NegX, PosY, NegY, PosZ, NegZ };
typedef unsigned DirMask;
```

```cpp
virtual DirMask blocked(const Graph &g, int piece,
                        const QVector<bool> &present) const = 0;
```

`TranslationalBlocking` is the one you described: walk the piece's contacts, and
for each neighbour still `present`, set the bit for the direction that neighbour
lies in. For axis-aligned boxes that is exact for straight-line withdrawal.

**To write your own**, subclass `BlockingModel` and implement four methods:
`name()`, `caveat()`, `blocked()`, `blockers()`. `caveat()` is not optional — it
is one line stating what the model does *not* cover, and it is printed with
every plan so a result is never read as stronger than the model behind it.

Note the interface takes the **whole present set**, not a neighbour pair. That
is deliberate: a swept-volume test has to be free to consult pieces that never
touch the piece being moved, and a pairwise interface would have to be thrown
away when that lands.

### Stage 3 — `Planner::extract`

Exactly your peeling loop: find a piece with at least one free direction, take
it off, repeat.

```cpp
struct Plan {
    QVector<Step> removal;    // disassembly, first piece off first
    QVector<Step> assembly;   // the reverse, directions flipped
    QVector<int>  stuck;      // pieces still present when it gave up
    bool          complete;
    QString       modelName, modelCaveat;
};
```

`Planner::replay(g, otherModel, plan)` checks an order found under one model
against another. Use it when you add joints — it validates *the order*, instead
of letting a fresh search quietly pick a different order that suits the new
model.

---

## The thing to understand before you trust a result

**Without joints, a box partition always peels.** Sort the pieces by their box's
minimum x, descending, and remove them in that order: nothing is ever in front,
because nothing has a larger x. So `extract` will report `complete = true` on
every division you produce, every time.

This is confirmed by your own runs — the BSP cube (8 pieces, 16 contacts) and
the BSP torus (19 pieces, 39 contacts) both order completely.

That is why `AlwaysBlocking` exists in `PlannerBlocking.h`. It blocks every
direction of any piece that still has a neighbour, purely so the
**non-assemblable branch of stage 3 can be exercised at all** — with
`--plan --stress`. Without it that branch would never run and you would not know
whether it worked.

So: **the assembly test is not a filter on divisions.** Running it on the cage
path will tell you "assemblable" every single time, and that answer is correct
and uninformative. It becomes a real test only once joints constrain motion,
which is `JointedBlocking` in `PlannerJoints.h`.

Wire it anyway — you need the plumbing, the logs, and the baseline. Just do not
read the green result as evidence of anything yet.

---

## Traps

**Contact area is cell-face area, not material contact.** Two cage cells can
share a face where no material actually touches, because the pieces were trimmed
back by the Boolean. The graph will still call them neighbours. This
over-reports blocking, which is the safe direction — it never claims a piece is
free when it is not — but it is pessimistic, and it is the first thing to fix if
the planner ever reports a deadlock you cannot see.

**Units differ by path.** `Planner::build` reads `PuzzlePiece::p0/p1`. On the
cage path those are the cell's box in the trivariate's **parameter domain**
(0..1); on the mesh path they are world coordinates. Topology and direction are
unaffected — the box cage's map is diagonal — but `area` and `depth` are in
parameter units on the cage path, so do not compare them across paths or use
them as a physical threshold.

**Greedy, no backtracking.** `extract` never reconsiders a removal. So
`complete = false` means *this greedy order failed*, not *no order exists*. Do
not report it as proof of a deadlock.

**Translational only.** No rotation, no swept volume, no pieces that are in the
way without touching. Anything the planner blesses is assemblable *under that
model* — say it that way in the paper.

---

## Suggested order

1. Run `--meshdivide MODEL bsp 8 --plan` and read the output.
2. Add the one line to `cageCli`, run `--cage MODEL 6 7 --plan`.
3. Fix `describePieces` to use `adjacencyOfBoxes`.
4. Add the planner call to `runTrivCellDivision` so the app logs it too.
5. Only then move to joints — that is where the test starts having something to
   say.
