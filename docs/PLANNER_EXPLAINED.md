# How the assembly planner works

*Plain explanation first, then the code that does it, function by function.*

The planner answers one question: **can these pieces be put together, and in
what order?** It works in three stages, each a separate file:

| Stage | Question | File |
|---|---|---|
| 1 | Which pieces touch, and on which face? | `PlannerGraph.cpp` |
| 2 | Which way can each piece still move? | `PlannerBlocking.cpp` |
| 3 | What order takes them all apart? | `PlannerOrder.cpp` |

Reverse stage 3 and you have the assembly order.

Nothing in these three files touches IRIT. They work on the pieces' **boxes**,
so the planner can be run and tested without a geometry kernel.

---

## The one idea to hold on to

Every piece is an axis-aligned box. Two boxes that share a face are in each
other's way along **one axis only**. That is the whole model:

> A piece cannot move in a direction if a neighbour is sitting on that face.

Everything below is bookkeeping around that sentence.

---

## Stage 1 — who touches whom

### The idea

Two pieces touch when a face of one lies on a face of the other **and** the
faces actually overlap. Meeting at an edge, or at a single corner, does not
count — nothing there can block a slide.

```
   touching (shared face)        edge only          corner only
   +-----+-----+                 +-----+            +-----+
   |  A  |  B  |                 |  A  |            |  A  |
   |     |     |                 +-----+-----+      +-----+-----+
   +-----+-----+                       |  B  |            |  B  |
      counts                           +-----+            +-----+
                                    does not count     does not count
```

### The code — `PuzzleDivider::adjacencyOfBoxes`

Every pair of pieces, every axis:

```cpp
for (int i = 0; i < pieces.size(); ++i)
    for (int j = i + 1; j < pieces.size(); ++j) {
        const PuzzlePiece &A = pieces[i], &B = pieces[j];

        for (int a = 0; a < 3; ++a) {
            const bool meets = std::fabs(A.p1[a] - B.p0[a]) <= eps ||
                               std::fabs(B.p1[a] - A.p0[a]) <= eps;
            if (!meets)
                continue;
```

`p0` is a box's low corner, `p1` its high corner. `meets` asks: is A's **high**
face on axis `a` in the same place as B's **low** face, or the other way round?
`eps` (1e-6) absorbs floating-point noise.

Then the overlap test on the other two axes:

```cpp
            bool overlaps = true;
            for (int b = 0; b < 3 && overlaps; ++b) {
                if (b == a)
                    continue;
                const double lo = qMax(A.p0[b], B.p0[b]);
                const double hi = qMin(A.p1[b], B.p1[b]);
                if (hi - lo <= eps)
                    overlaps = false;
            }
            if (overlaps) {
                out.append({ i, j, a });
                break;
            }
```

`lo`/`hi` are the overlapping interval on axis `b`. **This is where edge and
corner contacts are thrown away**: a corner touch has zero overlap on both
remaining axes, an edge touch on one, so `hi - lo <= eps` and `overlaps` goes
false.

The `break` matters too: once a pair is recorded on one axis, stop. Two boxes
can only share a real face on one axis.

Result: a list of `{ a, b, axis }` triples.

### Turning that into the graph — `Planner::build`

`adjacencyOfBoxes` says *that* two pieces touch. The graph adds **which side each
one is on**, which is what stage 2 needs.

```cpp
int lo = l.a, hi = l.b;
if (std::fabs(pieces[hi].p1[axis] - pieces[lo].p0[axis]) <= eps)
    qSwap(lo, hi);
```

After this, `lo` is the piece on the **low** side of the shared plane and `hi`
the one on the **high** side. The swap fires when it was the other way round.

Then the contact is measured and stored:

```cpp
Contact c;
c.lowSide  = lo;
c.highSide = hi;
c.axis     = axis;
c.plane    = 0.5 * (A.p1[axis] + B.p0[axis]);
c.area     = area;
c.depth    = qMin(A.p1[axis] - A.p0[axis], B.p1[axis] - B.p0[axis]);

const int idx = g.contacts.size();
g.contacts.append(c);
g.incident[lo].append(idx);
g.incident[hi].append(idx);
```

`incident[p]` is the list of contact indices touching piece `p` — the adjacency
list. `area` is used by the joint code to decide whether a face is big enough to
carry a peg; `minArea` can reject slivers.

### What you get

```cpp
struct Contact {
    int    lowSide, highSide;   // the two pieces
    int    axis;                // 0 = X, 1 = Y, 2 = Z
    double plane;               // where the shared face is
    double area, ext[2], depth; // how big the contact is
};
```

**This is figure 2** (`2_adjacency_graph.png`): one line per contact, coloured by
`axis`.

---

## Stage 2 — which ways can a piece move

### The idea

Six directions: ±X, ±Y, ±Z. For one piece, walk its contacts. Each contact with
a piece **that is still present** blocks exactly one direction: the one pointing
into that neighbour.

### Directions are integers

```cpp
enum Direction { PosX = 0, NegX, PosY, NegY, PosZ, NegZ, DirectionCount };

inline int dirAxis(int d)     { return d >> 1; }        // 0,1 -> X   2,3 -> Y
inline int dirSign(int d)     { return (d & 1) ? -1 : +1; }
inline int dirOpposite(int d) { return d ^ 1; }         // flip the low bit
```

Even = positive, odd = negative. `dirOpposite` is a single XOR because of that
layout — this is why reversing a removal order into an assembly order is free.

### The key line in the whole planner

```cpp
int Graph::directionFrom(int contactIndex, int piece) const
{
    const Contact &c = contacts[contactIndex];
    if (piece == c.lowSide)  return c.axis * 2 + 0;   // blocked going +
    if (piece == c.highSide) return c.axis * 2 + 1;   // blocked going -
    return -1;
}
```

If you are on the **low** side of a shared plane, the neighbour is above you, so
you are blocked in the **positive** direction on that axis. If you are on the
**high** side, blocked in the **negative**. That one line converts geometry into
blocking.

### The blocking mask

Six bits, one per direction:

```cpp
typedef unsigned DirMask;

inline bool    isBlocked(DirMask m, int d)   { return ((m >> d) & 1u) != 0u; }
inline DirMask withBlocked(DirMask m, int d) { return m | (1u << d); }
inline bool    allBlocked(DirMask m)         { return (m & 0x3Fu) == 0x3Fu; }
```

`0x3F` is six bits set — all directions blocked.

### The test — `TranslationalBlocking::blocked`

```cpp
DirMask m = 0;
for (int ci : g.incident[piece]) {
    const int other = g.otherSide(ci, piece);
    if (other < 0 || other >= present.size() || !present[other])
        continue;                       // already removed: no longer in the way
    m = withBlocked(m, g.directionFrom(ci, piece));
}
return m;
```

Start with nothing blocked, walk the contacts, set a bit for each neighbour
**still present**. That `!present[other]` line is what makes the puzzle come
apart: as pieces leave, directions open up.

**This is figure 3** (`3_blocking.png`): one row per piece, six columns, red
where a bit is set, naming the blocker from `blockers()`.

### Why it is an interface, not a function

```cpp
class BlockingModel {
    virtual DirMask blocked(const Graph &g, int piece,
                            const QVector<bool> &present) const = 0;
    virtual QVector<int> blockers(const Graph &g, int piece, int dir,
                                  const QVector<bool> &present) const = 0;
};
```

Three implementations exist: `TranslationalBlocking` (the real one),
`AlwaysBlocking` (blocks everything — used to check the search itself reports
failure properly), and `JointedBlocking` in `PlannerJoints.cpp` (a mated peg
leaves exactly one direction).

Note the signature takes **the whole `present` set**, not a pair of pieces. That
is deliberate: a swept-volume test has to consult pieces that never touch the one
being moved. Replacing this class is the research step.

---

## Stage 3 — the order

### The idea

Disassembly is easier to search than assembly, so take pieces **off** and reverse
the result at the end.

### The loop — `Planner::extract`

```cpp
QVector<bool> present(n, true);

while (remaining > 0) {
    int pick = -1, pickDir = -1;

    for (int i = 0; i < n && pick < 0; ++i) {
        if (!present[i]) continue;
        const DirMask m = model.blocked(g, i, present);
        for (int d = 0; d < DirectionCount; ++d)
            if (!isBlocked(m, d)) { pick = i; pickDir = d; break; }
    }

    if (pick < 0) { /* stuck: record every remaining piece */ }

    plan.removal.append({ pick, pickDir });
    present[pick] = false;
    --remaining;
}
```

Take the **first** piece with any free direction. No scoring, no backtracking.

Then reverse, flipping each direction:

```cpp
for (int i = plan.removal.size() - 1; i >= 0; --i)
    plan.assembly.append({ plan.removal[i].piece,
                           dirOpposite(plan.removal[i].dir) });
```

The direction a piece left along is, reversed, the direction it is inserted
along. **This is figure 4** (`4_removal_order.png`).

So `4 (-X)` means: piece 4 came off first, sliding in −X. Read the line backwards
to build the puzzle.

### Checking the answer — `replay`

`extract` produces an order; `replay` proves it:

```cpp
QVector<bool> present(g.pieceCount(), true);

for (int i = 0; i < plan.removal.size(); ++i) {
    const Step &s = plan.removal[i];
    const DirMask m = model.blocked(g, s.piece, present);
    if (isBlocked(m, s.dir))
        return i;                       // step i does not hold
    present[s.piece] = false;
}
return -1;                              // every step holds
```

It re-walks the plan from a clean state and returns the index of the first step
that fails, or −1. On failure it names the blockers and the directions that *were*
free. A plan that survives `replay` is a witness, not a claim.

---

## What the result does and does not mean

Written into `Plan::describe()`:

- **Success is real.** Each step was checked against the pieces still present at
  that moment.
- **Failure is not a proof.** `describe()` says so in as many words: *"greedy, no
  backtracking - this is a failure of this search, not a proof that no order
  exists"*. It also only covers **monotone** sequences: one piece at a time,
  straight out, never moved again.
- **For box partitions, success is nearly free.** The piece with the largest
  `hi[x]` cannot have a neighbour on its `+X` face, because such a neighbour
  would need a larger `hi[x]` still. So some piece is always removable and the
  loop always finishes. A "yes" carries little information until joints restrict
  motion or a swept test replaces this one.

Every string the planner produces carries the qualifier *"assemblable under
translational blocking; rotational/swept check pending."*

---

## Reading the code in order

1. `PlannerGraph.h` — `Direction`, `Contact`, `Graph`. Small; read it first.
2. `PuzzleDivider::adjacencyOfBoxes` — the touching test.
3. `PlannerGraph.cpp` → `build`, `directionFrom`, `otherSide`.
4. `PlannerBlocking.h` — `DirMask` helpers and the `BlockingModel` interface.
5. `PlannerBlocking.cpp` → `TranslationalBlocking::blocked`.
6. `PlannerOrder.cpp` → `extract`, then `replay`, then `Plan::describe`.
7. `PlannerJoints.cpp` — only after the above; it is a fourth stage that reuses
   the same interface.

## Questions you should be able to answer

- *Why is a corner touch not an edge in the graph?* Zero overlap on both other
  axes, so `hi - lo <= eps`.
- *Why does the low-side piece get the positive direction?* The neighbour is
  above it on that axis.
- *Why `present` and not just the graph?* Blocking changes as pieces leave; that
  is the whole mechanism.
- *Why an interface for blocking?* So a swept-volume test can replace it without
  touching stages 1 and 3.
- *Why is a successful order weak evidence here?* Box partitions always have a
  free piece.
