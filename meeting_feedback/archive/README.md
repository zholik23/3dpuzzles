# Archive

Superseded documents, kept for the record. **Do not quote numbers from here.**

They describe either my first script (`irit/unused/puz2x2x2_nopins.irt` - the
cube and the two scaled sphere shells, which are out of scope) or runs made
before two analyser fixes:

- pole normals: at a pole the normal is round-off, and with R > 0 it was not
  exactly zero, so fake normals blocked directions that are free
  (`../results/dbg_verification.md`)
- the direction set now contains every direction's exact opposite, so a group
  and the rest of the puzzle always get the same answer

| file | what it was |
|---|---|
| `results_README_old.md` | how to regenerate results with the old script |
| `base_shapes.md` | the two sphere variants with different inner scaling |
| `cube_sweep.md` | the synthetic cube sweep - the only single keys ever found, on a shape outside scope |
| `dbg_results.md` | first DBG-with-groups run, old sphere numbers |
| `sweep_both_shapes.md` | the re-seeded R sweep, before the fixes |
| `verification.md` | checks of the old script's trivariates |

Current numbers: `../results/blocking_table.md` (regenerate with `python ../make_tables.py`).
