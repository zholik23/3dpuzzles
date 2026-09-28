# DBG results — with subassemblies

## What changed

The earlier analyser only asked *"can one piece be pulled out?"*. That is not
enough to call an assembly interlocking: **a group of pieces can slide out
together while no single member could move alone.**

The analyser now enumerates **every proper non-empty subset** (all 254 of them
for 8 pieces) and asks whether each can translate out as a rigid group. An
interface buried inside the group imposes nothing, because both of its pieces
travel together — only interfaces crossing the group's boundary constrain it.

That distinction is the difference between a pile of pairwise facts and a real
blocking graph.

## Results

| case | det J int | pairs | blocked | mobile **pieces** | mobile **subsets** | smallest group | single key |
|---|---|---|---|---|---|---|---|
| planar control | 8 | 12 | 0 | 8 | 104 | 1 | no |
| cube R=0.22 | 0.202 | 12 | 3 | **1** | 4 | 1 | **yes** (piece 1) |
| cube R=0.23 | 0.235 | 12 | 2 | **1** | 6 | 1 | **yes** (piece 4) |
| cube R=0.25 | 0.202 | 12 | 2 | **0** | **2** | **4** | no |
| sphere A R=0.05 | 0.245 | 12 | 3 | **1** | 14 | 1 | **yes** (piece 7) |
| duck 2×2×2 | 0.036 | 12 | 4 | **0** | **12** | **2** | no |
| duck 2×2×1 | 0.036 | 4 | 0 | 4 | 12 | 1 | no |

## The correction this forces

**"0 mobile pieces" does not mean interlocked.** Two cases prove it:

- **duck 2×2×2** — no single piece can be removed, but **12 groups can**, the
  smallest being a **pair**. Two pieces slide out together.
- **cube R=0.25** — no single piece moves, but a **group of 4** comes away.

I previously described the duck as "over-locked — cannot be disassembled". That
was **wrong**, and wrong in the direction that matters: it is disassemblable, just
not one piece at a time. Only the subset search could have caught it.

## What is actually true across the experiment

**Single key — exactly one piece removable first:** cube R=0.22, cube R=0.23,
sphere R=0.05. This is what the professor asked for in step 4, and it is real.

**Fully interlocked — no subset of any size removable: none.** Every case has at
least one removable group. So nothing here is interlocked in the strict sense,
and no case should be described that way.

The honest phrasing for the strongest cases:

> Cube trivariate, R = 0.22, no joints: exactly **one piece** can be withdrawn
> first, and 3 further groups are also removable. Interior Jacobian 0.202.

Note the tension between the two columns: a single mobile *piece* is a good
puzzle-like property, but several mobile *groups* means a solver has more than
one opening move. A strict single-key puzzle wants **mobile subsets = 1**.

## The planar control still validates

104 mobile subsets, smallest size 1, 0 blocked pairs, all 8 pieces mobile — a
flat box partition falls apart in many ways at once, exactly as it must. The
control continues to behave.

## Cost

The subset search is cheap because "which directions work" is stored as a
**bitset per interface**, one bit per direction. A group's mobility is then a
bitwise AND across its boundary interfaces rather than a loop over thousands of
sampled normals. All 254 subsets × 4000 directions finish in well under a second.

For more than 12 pieces the search is capped at groups of 4 and the report says
so — absence of a mobile group is then **not** a proof.

## What to do next

1. **Search for mobile subsets = 1**, not just mobile pieces = 1. That is the
   real single-key condition and nothing in the current sweep achieves it.
2. Fix the random offsets and scale them by R, so the sweep is one shape rather
   than independent draws (see `cube_sweep.md`).
3. Then level *k* (Chen et al. 2022), which needs the disassembly tree rather
   than a single-step test.
