# Interlocking - spot.obj

Division: Cut in D (curved: bend 0.25) : 3 of 3 pieces (body 2, limb 9 1) · 7 part(s) glued on · volume 100.000% of the model · 0 open edge(s) · seed 7 · 37 ms

**INTERLOCKING with a single key: only piece 2 can come out first**

## How it was checked
- Translational directional blocking graph (first-order: the first small step along a direction), 4000 sampled directions; clearance decided exactly (Gilbert).
- Contacts on the cuts: each exact cut surface S(s,t) in D sampled 24 x 24; n_D = S_s x S_t; the pieces on its two sides found by stepping 0.003 off it in D and locating both points in the model; normal in the model n' = n_u (M_v x M_w) + n_v (M_w x M_u) + n_w (M_u x M_v) (= det J J^-T n_D), turned to point from a to b.
- Contacts on the joint caps between limbs (flat, not cuts in D): the cap triangles' normals.
- 279 contact samples (151 on cuts, 128 on caps); sampling 14 ms, total 17 ms.
- Groups tested: all.

## Results
- touching pairs: 2; blocked in every direction: 1
- pieces that slide out alone: 1 (piece 2)
- groups that can slide out: 2
- smallest: {2} along (0.600, 0.434, -0.672), clearance 61.98 deg
- key level: 1 (keys in order: 2) - every state had a unique key, down to the last two pieces
- one way to take it apart: {2} along (0.600, 0.434, -0.672) - then STOPS: what is left is locked

## Pieces alone
| piece | free directions (of 4000) | clearance |
|---|---|---|
| 0 | 0 | blocked |
| 1 | 0 | blocked |
| 2 | 1225 | 61.98 deg |

## Touching pairs
| pair | normals | spread (deg) | free directions | blocked | clearance (deg) |
|---|---|---|---|---|---|
| 0-1 | 151 | 154.9 | 0 | **yes** | 0.00 |
| 0-2 | 128 | 35.8 | 1225 | no | 61.98 |

## Every way it opens (smaller side, up to 64)
- {2} along (0.600, 0.434, -0.672)