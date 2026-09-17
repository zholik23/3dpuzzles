# Installing the BSP puzzle division into IRIT

Adds the `PUZBSP` script command and the GuIrit **BSP Volumetric** panel.

Five steps: copy the files, add five lines to existing sources, add the new
sources to the build, build, check.

---

## 1. Copy the files

Copy each file from this package to the same relative path in your IRIT tree.
None of them overwrites anything.

| From this package | To |
|---|---|
| `irit/ext_lib/PuzBspCore.c` | `irit/ext_lib/` |
| `irit/ext_lib/PuzBspCore.h` | `irit/ext_lib/` |
| `irit/ext_lib/puzbsp.c` | `irit/ext_lib/` |
| `extra/guirit/Src/GuIritDllBspPuzzle/GuIritDllBspPuzzle.cpp` | `extra/guirit/Src/GuIritDllBspPuzzle/` |
| `extra/guirit/Src/GuIritDllBspPuzzle/GuIritDllBspPuzzle.def` | `extra/guirit/Src/GuIritDllBspPuzzle/` |
| `extra/guirit/Src/GuIritDllBspPuzzle/Icons/IconBspPuzzle.xpm` | `extra/guirit/Src/GuIritDllBspPuzzle/Icons/` |
| `extra/guirit/Src/WindowsVC2026/GuIritDllBspPuzzle/GuIritDllBspPuzzle.vcxproj` | `extra/guirit/Src/WindowsVC<your version>/GuIritDllBspPuzzle/` |

The last four are only needed for the GuIrit panel. For the script command
alone, the three `ext_lib` files are enough.

---

## 2. Add five lines to existing sources

All five are additions.

**2.1 — `irit/inc_irit/ext_lib.h`**, with the other declarations:

```c
IritPrsrObjectStruct *IritExtPuzBsp(IritPrsrObjectStruct *PolyObj, IrtRType *RNumPieces, IrtRType *RVoxelRes, IrtRType *ROutput);
```

**2.2 — `irit/irit/inptprsl.h`**, as the **last** entry of `ObjValueFuncType`:

```c
    IP_PUZBSP,

    IRIT_PRSR_OBJ_VAL_LAST
} ObjValueFuncType;
```

**2.3 — `irit/irit/inptevl0.c`**, as the **last** row of `ObjFuncTable[]`
(add a comma to the row above it):

```c
    { IPNP(PUZBSP),     IPNP2(IritExtPuzBsp),         4,        { POLY_EXPR, NUMERIC_EXPR, NUMERIC_EXPR, NUMERIC_EXPR }, OLST_EXPR }
};
```

> 2.2 and 2.3 must both be **last**, and in the same order as each other. The
> evaluator indexes the table by arithmetic on the token
> (`ObjFuncTable[NodeKind - IRIT_PRSR_OBJ_FUNC_OFFSET]`, `inptevl1.c`), so the
> enum and the table are parallel arrays. Inserting in the middle of either
> silently rebinds other commands.

**2.4 — `irit/dll_defs/irit_all_dll_defs.txt`** (Windows DLL build only),
anywhere in the list:

```
IritExtPuzBsp
PuzBspBuildCells
PuzBspMakePieces
PuzBspCellBox
```

**2.5 — `extra/guirit/Src/GuIrit/guirit64.cfg`** (panel only), after the
*Slotted Tube* line:

```
    "Applications.3D Puzzles.BSP\nVolumetric",		"IRT_MDLR_PUZZLE_BSP",			"Type:REGULAR",
```

> Edit the copy under `Src/GuIrit/`. On Windows `GuIritSetup*.bat` copies it
> over the installed one.

---

## 3. Add the new sources to the build

| Build file | Line |
|---|---|
| `ext_lib/Makefile.am` | `explfunc.c puzbsp.c PuzBspCore.c` |
| `ext_lib/makefile.unx` | `OBJS =  explfunc.o$(IRIT_OBJ_PF) puzbsp.o$(IRIT_OBJ_PF) PuzBspCore.o$(IRIT_OBJ_PF)` |
| `ext_lib/makefile.wnt` | `OBJS =  explfunc.$(IRIT_OBJ_PF) puzbsp.$(IRIT_OBJ_PF) PuzBspCore.$(IRIT_OBJ_PF)` |
| `windowsVC20xx/ext_lib/ext_lib.vcxproj` | `<ClCompile Include="..\..\ext_lib\puzbsp.c" />` and the same for `PuzBspCore.c` |

Use whichever your platform builds with. Miss this and `ext_lib` compiles
without the command, then the link fails on `IritExtPuzBsp`.

---

## 4. Build

In this order:

1. `ext_lib`
2. `inpt_lib`
3. `irit` → `irit64.exe`

On Windows also:

4. relink `Irit64.dll` (`iritDll`)
5. relink `GuIrit64.exe` — only if you want `PUZBSP` inside GuIrit's own
   interpreter; it links the interpreter's function table statically
6. build `GuIritDllBspPuzzle` and copy the resulting
   `GuIritDllBspPuzzle_64.dll` into `GuIritData64/Extensions/`

Close GuIrit and `irit64.exe` before steps 4–6; a running process holds those
files open and the link fails.

> **Rebuild the panel (step 6) whenever `PuzBspCore.c` changes.** The plugin
> project compiles the core directly:
>
> ```
> <ClCompile Include="c:\irit\irit\ext_lib\PuzBspCore.c" />
> ```
>
> so it keeps its own copy and does not pick up changes through `Irit64.dll`.
> Skip it and the command shows new behaviour while the panel shows old.

---

## 5. Check it works

**Script command** — in `irit64.exe`:

```
a = box( vector( 0, 0, 0 ), 1, 1, 1 );
p = PUZBSP( a, 8, 32, 2 );
```

Expected on stderr:

```
PUZBSP: 8 of 8 pieces made - 0 failed, 0 empty - pieces sum to 100.0% of the
model volume - 0.02s total (voxelise 0.02, cuts 0.00, trim 0.00)
```

Arguments: solid, piece count, voxel resolution of the longest axis, output
mode (`0` cell boxes, `1` sub-trivariates, `2` pieces trimmed to the model).
The input must be a closed polygonal solid.

**Panel** — GuIrit → **Applications → 3D Puzzles → BSP Volumetric**. The
`BspPzl:` line in General Output carries the same report.

---

## Troubleshooting

| Symptom | Cause |
|---|---|
| Link fails on `IritExtPuzBsp` | step 3 missed for your build system |
| `PUZBSP` unknown in the interpreter | steps 2.2 / 2.3, or `inpt_lib` not rebuilt |
| Button missing in GuIrit | step 2.5, or the cfg was overwritten by `GuIritSetup*.bat` |
| Panel behaves differently from the command | step 6 skipped after changing `PuzBspCore.c` |
| Pieces come back as whole boxes | the input is not a closed solid, or its normals are inconsistent |
| `LNK1104` while relinking | GuIrit or `irit64.exe` still running |
