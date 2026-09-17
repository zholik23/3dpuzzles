# Material-aware BSP division for IRIT — installation guide

*Everything needed to add the `PUZBSP` command and the GuIrit **BSP Volumetric**
panel to a clean IRIT tree, what to change in the existing sources, and what has
and has not been tested.*

The division follows Elber, *On the Construction of Freeform Volumetric 3D
Puzzles* (2025), §5: a cell partition of the bounding region, then a per-piece
Boolean intersection against the model. What is added here is the **cell
partition**: cuts chosen by a cost function evaluated against the material
actually inside each cell, rather than a uniform grid.

---

## 1. What to send

Seven new files. None of them replaces anything in a clean IRIT tree.

| File | Size | What it is |
|---|---|---|
| `irit/ext_lib/PuzBspCore.c` | 32 KB | the division core: voxel field, 3D prefix sum, connectivity, cost-function BSP, cell → sub-trivariate, cell box, Boolean trimming |
| `irit/ext_lib/PuzBspCore.h` | 4.3 KB | its interface, plus `PuzBspCellWire` and `PuzBspLastReport` |
| `irit/ext_lib/puzbsp.c` | 1.5 KB | the thin `PUZBSP` wrapper the interpreter calls |
| `extra/guirit/Src/GuIritDllBspPuzzle/GuIritDllBspPuzzle.cpp` | 5.2 KB | the GuIrit panel |
| `extra/guirit/Src/GuIritDllBspPuzzle/GuIritDllBspPuzzle.def` | 28 B | exports `_IrtMdlrDllRegister` |
| `extra/guirit/Src/GuIritDllBspPuzzle/Icons/IconBspPuzzle.xpm` | 4.6 KB | the button icon |
| `extra/guirit/Src/WindowsVC2026/GuIritDllBspPuzzle/GuIritDllBspPuzzle.vcxproj` | 17 KB | MSVC project for the panel (adapt the folder name for other VS versions) |

`PuzBspCore.c` is plain C99. It uses only IRIT's own API — no Qt, no C++, no
GuIrit dependency — so `ext_lib` stays buildable on its own.

**Do not copy `geom_lib/ogl_depth_peel.c`.** It appears in our change log, but
only because that file had been locally stubbed out on this machine and had to
be restored before `Irit64.dll` would link. It has nothing to do with BSP.

Skip every `*.before-*` file; those are local backups.

---

## 2. Five edits to existing IRIT sources

All five are additions. None changes existing behaviour.

### 2.1 `irit/inc_irit/ext_lib.h` — declare the function

After `IritExtExampleFunction` (line 33 in our tree):

```c
IritPrsrObjectStruct *IritExtPuzBsp(IritPrsrObjectStruct *PolyObj, IrtRType *RNumPieces, IrtRType *RVoxelRes, IrtRType *ROutput);
```

### 2.2 `irit/irit/inptprsl.h` — the token

Add `IP_PUZBSP` as the **last** entry of `ObjValueFuncType`, immediately before
the terminator:

```c
    IP_JOINCRVCRV,
    IP_PUZBSP,

    IRIT_PRSR_OBJ_VAL_LAST
} ObjValueFuncType;
```

### 2.3 `irit/irit/inptevl0.c` — the table row

Add as the **last** row of `ObjFuncTable[]` (give the previous row a trailing
comma):

```c
    { IPNP(PUZBSP),     IPNP2(IritExtPuzBsp),         4,        { POLY_EXPR, NUMERIC_EXPR, NUMERIC_EXPR, NUMERIC_EXPR }, OLST_EXPR }
};
```

> **Both must be last, and in the same order.** The evaluator looks the row up
> by arithmetic on the token — `ObjFuncTable[NodeKind - IRIT_PRSR_OBJ_FUNC_OFFSET]`
> (`inptevl1.c:99`) — so the enum and the table are parallel arrays. Appending at
> the end of both leaves every existing index untouched; inserting in the middle
> of either silently rebinds other commands.

### 2.4 `irit/dll_defs/irit_all_dll_defs.txt` — the exports

Four names, anywhere in the list:

```
IritExtPuzBsp
PuzBspBuildCells
PuzBspMakePieces
PuzBspCellBox
```

Only needed for the Windows DLL build. The GuIrit panel calls the last three
directly, which is why they are exported and not just the command.

### 2.5 `extra/guirit/Src/GuIrit/guirit64.cfg` — the button

After the *Slotted Tube* line in the 3D Puzzles menu:

```
    "Applications.3D Puzzles.BSP\nVolumetric",		"IRT_MDLR_PUZZLE_BSP",			"Type:REGULAR",
```

> GuIrit loads every DLL in `GuIritData64\Extensions` and calls its
> `_IrtMdlrDllRegister`, but a registered function gets a button **only** if the
> cfg lists its string id. Without this line the plugin loads and does nothing
> visible. The id must match `"IRT_MDLR_PUZZLE_BSP"` in the plugin's function
> table exactly.
>
> On Windows note that `GuIritSetup*.bat` copies this cfg over the installed
> one, so edit the source copy or the change is lost on the next setup run.

---

## 3. Build-file entries

The two new `.c` files must be added to whichever build system is used.

| Build file | Line to have |
|---|---|
| `ext_lib/Makefile.am` | `explfunc.c puzbsp.c PuzBspCore.c` |
| `ext_lib/makefile.wnt` | `OBJS =  explfunc.$(IRIT_OBJ_PF) puzbsp.$(IRIT_OBJ_PF) PuzBspCore.$(IRIT_OBJ_PF)` |
| `ext_lib/makefile.unx` | `OBJS =  explfunc.o$(IRIT_OBJ_PF) puzbsp.o$(IRIT_OBJ_PF) PuzBspCore.o$(IRIT_OBJ_PF)` |
| `windowsVC20xx/ext_lib/ext_lib.vcxproj` | `<ClCompile Include="..\..\ext_lib\puzbsp.c" />` and the same for `PuzBspCore.c` |

**`makefile.unx` is the one to watch.** In our tree it was never updated — it
still reads `OBJS = explfunc.o$(IRIT_OBJ_PF)`. A Unix build with that line
produces an `ext_lib` without the command and then fails to link
`IritExtPuzBsp`. `Makefile.am` and `makefile.wnt` were updated; the Unix
makefile was not.

Build order: `ext_lib`, then the interpreter (`inpt_lib`), then `irit`. On
Windows also relink `Irit64.dll`, and relink `GuIrit64.exe` if the command
should be available inside GuIrit — GuIrit statically links the interpreter's
function table, so a new command does not reach it through the DLL.

> **Rebuild the panel too, whenever `PuzBspCore.c` changes.** The plugin project
> compiles the core straight into itself:
>
> ```
> <ClCompile Include="c:\irit\irit\ext_lib\PuzBspCore.c" />
> ```
>
> so the panel carries its **own copy** and does not pick up changes through
> `Irit64.dll`. Miss this and the symptom is quietly confusing: the `PUZBSP`
> command shows the new behaviour while the panel still shows the old, from the
> same tree. We hit exactly that — the command reported stage timings and the
> panel did not, until the plugin was rebuilt.
>
> To check which code a binary really holds, search it for a string only the new
> version prints, rather than trusting timestamps.

---

## 4. Trying it

### As a script command

```
a = LOAD("solid.itd");
p = PUZBSP(a, 8, 96, 2);
view(p, 1);
```

Four arguments:

| # | Argument | Meaning |
|---|---|---|
| 1 | solid | a **closed, watertight polygonal** object — polygons, not polylines |
| 2 | pieces | exact number of pieces |
| 3 | voxel resolution | voxels along the longest axis; 96 is the default used in our app |
| 4 | output | `0` cells, `1` sub-trivariates, `2` pieces trimmed to the model |

It prints a report to **stderr**: how many pieces were made, how many Booleans
failed or came back empty, what the pieces sum to as a fraction of the model,
and how long each stage took.

```
8 of 8 pieces made - 0 failed, 0 empty - pieces sum to 100.0% of the model
volume - 12.34s total (voxelise 0.21, cuts 0.16, trim 11.97)
```

The times come from `IritMiscCPUTime`, which reports wall-clock seconds on
Windows and CPU seconds on Unix. The Boolean trim normally dominates: it is one
intersection per piece against the whole model, so it grows with piece count
times mesh size, while choosing the cuts stays in the tens of milliseconds.
Quote those two separately — the partition is the contribution here, the trim is
the cost of Elber's §5 construction and a uniform grid would pay it too.

One imprecision worth knowing: the "trim" figure times the whole piece-building
call, so for output modes `0` and `1` it is really "build the output" rather
than trimming. Those paths take milliseconds, so it does not mislead.

A short puzzle never fails silently. Note there is no spacing argument — the
command leaves pieces assembled.

### As a GuIrit panel

**Applications → 3D Puzzles → BSP Volumetric.** Six fields: name, object,
pieces, voxel resolution, output, and a spacing gap the script command does not
expose. Read the `BspPzl:` line in General Output: about **100 %** of the model
volume means the pieces are real; several hundred percent means the cells came
back untrimmed. That line is the same report the command prints, so it carries
the stage timings too.

---

## 5. What is verified, and what is not

**Measured, in `irit64.exe`:**

- a cube into 8 → 8 pieces, summing to **100.0 %** of the model volume, 0 Booleans failed;
- an armadillo into 10 → 10 pieces, **100.0 %**, with the orientation reversal engaged.

**Not yet exercised:** the command has never been run from *inside* GuIrit's
interpreter — only from `irit64.exe`. The panel has not been re-run since the
coplanar fix was deployed. Both are expected to work and neither has been
demonstrated.

**Scope:** this is the *division*. The assembly planner, the blocking graph and
the joints live in our Qt application and are not part of this port.

---

## 6. Two IRIT Boolean behaviours the core works around

Worth knowing, because both produced wrong output before they were handled and
both fail quietly rather than loudly.

1. **Winding.** IRIT's Booleans expect inward-facing normals. If the model's
   orientation disagrees with the cell box's, the intersection returns the whole
   box instead of the trimmed piece — pieces look like cubes and the volume sum
   comes out at several hundred percent. The core probes the signed volume of a
   cell box against the model's and calls `IritPrsrReverseObject` when they
   disagree.
2. **Coplanar faces.** Cell faces that lie exactly on the model's bounding box
   are coplanar with the model's own faces, which the Boolean handles poorly. The
   core sets `IritBoolSetHandleCoplanarPoly(TRUE)` and inflates outer cell faces
   by ε = 1e-4 × the largest extent. Before this, an 8-cell split of a cube
   returned 7 pieces at 88.3 %; after it, 8 pieces at 100.0 %.

A Boolean that fails or returns an empty or degenerate result is reported with
the piece index and the reason, then skipped — it never aborts the run.
