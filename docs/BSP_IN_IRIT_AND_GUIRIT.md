# BSP division inside IRIT and GuIrit

*How to move the material-aware BSP division out of the Qt app and into IRIT
itself — as a GuIrit panel, as an IRIT script function, or both.*

> **Already done on this machine.** The GuIrit panel from Route A is built and
> installed, and the Route B source edits are in place. What was changed, and
> what is still blocked, is recorded in
> [`BSP_IN_IRIT_WHAT_CHANGED.md`](BSP_IN_IRIT_WHAT_CHANGED.md). Read this guide
> for how and why; read that one for the current state of the tree.

Everything here was checked against the IRIT tree at `C:\irit\irit` and the
GuIrit source at `C:\irit\extra\guirit\Src` on this machine. File paths, symbol
names, table rows and line numbers are real. Where a statement is inference
rather than something read in the source, it says so — and every such place
comes with a check you can run.

---

## Contents

1. [The fact that decides everything](#1-the-fact-that-decides-everything)
2. [Four routes, compared](#2-four-routes-compared)
3. [Recommended plan](#3-recommended-plan)
4. [Step 0 — a Qt-free core](#4-step-0--a-qt-free-core)
5. [Route A — a GuIrit panel (plugin DLL)](#5-route-a--a-guirit-panel-plugin-dll)
6. [Route B — an IRIT built-in script function](#6-route-b--an-irit-built-in-script-function)
7. [Route C — pure `.irt` script](#7-route-c--pure-irt-script)
8. [Route D — exchange files (works today)](#8-route-d--exchange-files-works-today)
9. [Pitfalls already paid for](#9-pitfalls-already-paid-for)
10. [Verification protocol](#10-verification-protocol)
11. [Every file touched](#11-every-file-touched)

---

## 1. The fact that decides everything

GuIrit is not one program. It is three kinds of binary, and which one you
change decides how much has to be rebuilt.

| Binary | What is inside | Evidence |
|---|---|---|
| `GuIrit64.exe` | the wxWidgets GUI **and the IRIT script interpreter, statically linked** | `GuIrit.vcxproj` links `IritInpt64.lib`; the interpreter's table strings `TREGION`, `RegionFromTrivarObject`, `EXAMPLEFUNC` are present in the exe |
| `Irit64.dll` | every geometry library — Triv, Bool, Geom, Prsr, … **and `ext_lib`** | `$(IritLibs64)` in `windowsVC2026\IRIT_SM\GlobalRules.props` begins with `IritExt64.lib`; the same table strings are **absent** from the DLL |
| `GuIritData64\Extensions\GuIritDll*_64.dll` | plugins — panels, icons, menu blocks | each exports exactly one symbol, `_IrtMdlrDllRegister`; GuIrit loads every DLL in `AppExtensionsDir "GuIritData64/Extensions"` (`guirit64.cfg:58`) and calls that symbol (`GuIrit\Frame\IrtDspGuiApp.cpp:751`) |

```
        ┌──────────────────────────── GuIrit64.exe ────────────────────────────┐
        │   wxWidgets GUI              IRIT interpreter  (IritInpt64.lib)      │
        │        │                     function tables   (irit\inptevl0.c)     │
        └────────┼───────────────────────────────┬──────────────────────────────┘
                 │ loads every DLL at start       │ every geometry call
                 ▼                                ▼
   Extensions\GuIritDll*_64.dll  ─────────►  Irit64.dll
   (_IrtMdlrDllRegister)                     Triv · Bool · Geom · Prsr · … · ext_lib
```

Three consequences follow, and the whole guide is built on them:

- **A new script function** is a row in the interpreter's table, and that table
  is compiled into `GuIrit64.exe`. Adding one means relinking `GuIrit64.exe` —
  rebuilding IRIT alone is not enough for GuIrit to see it.
- **A new panel** is a plugin DLL. Adding one means building one DLL and
  dropping it into `Extensions`. Nothing else is rebuilt.
- **Every geometry call the BSP needs is already exported** by `Irit64.dll` —
  checked in `irit64.def`: `IritTrivTVRegionFromTV`, `IritTrivNSPrimBox`,
  `IritTrivTVDomain`, `IritBooleanAND`, `IritPrsrGenTRIVARObject`,
  `IritPrsrGenLISTObject`, `IritPrsrListObjectInsert`,
  `IritGeomPrimGenBOXObject` (line 3109), `IritGeomPrimSetGeneratePrimType`
  (line 3107).

> **Not a shortcut:** `GuIritMdlrDllRegisterInterpFunc` reads as if it adds a
> script function. It does not. It registers a whole alternative *interpreter
> console* — a name, a prompt and a file extension
> (`GuIritDllExtensions.cpp:186-213`). It is how `GuIritDllPython` adds Python
> to GuIrit.

**Everything required to build is already on this machine:** the GuIrit
solution `C:\irit\extra\guirit\Src\WindowsVC2026\GuIrit.sln`, wxWidgets 3.2.2.1
at `C:\c\wxWindows\wxWidgets-3.2.2.1-VC19`, glew at `C:\c\glew\glew-1.10.0`.
And IRIT has been rebuilt here before — `Irit64.dll` is dated 2026-05-04, and
`IritTriv64.lib`, `iritInpt64.lib`, `IritExt64.lib` are dated 2026-09-01,
against shipped binaries from 2026-03-19. So the toolchain works; the libs are
simply newer than the DLL that should contain them (see §5.9).

There is also a natural home already waiting: **`GuIritDllPuzzles`**, Elber's
"3D Puzzles" block, with four entries — Linear, Rounded and Custom slice, and
Slotted Tube. All four take a polygonal object and return a list of pieces,
which is exactly the shape of a BSP puzzle function.

---

## 2. Four routes, compared

| | **A. GuIrit panel** | **B. IRIT built-in** | **C. `.irt` script** | **D. exchange file** |
|---|---|---|---|---|
| What you get | a *BSP Puzzle* button and dialog in its own block | `PUZBSP(model, n, res, output)` callable from any script — in `irit64.exe` **and** in GuIrit | a script function with the same call | the Qt app's `.itd` loaded into GuIrit |
| Speed | native | native | interpreted; material measured by Booleans, no voxels | — |
| What is rebuilt | **one DLL** | ext_lib → `Irit64.dll` → inpt_lib → `irit64.exe` → **`GuIrit64.exe`** | nothing | nothing |
| Edits Elber's sources | no | yes — `inptprsl.h`, `inptevl0.c`, `ext_lib.h`, `irit_all_dll_defs.txt` | no | no |
| Callable from a script | no | **yes** | yes | no |
| Keeps the connectivity guarantee | yes | yes | **no** — no voxel flood fill | yes (computed in the app) |
| Survives an IRIT upgrade | rebuild the DLL | re-apply the edits | yes | yes |
| Effort | medium | high | low–medium | zero |

---

## 3. Recommended plan

1. **Today — Route D.** Divide in the Qt app, save `.itd`, load in GuIrit. No
   code.
2. **Write the core once** (§4): plain C, no Qt. A polygonal IRIT object goes
   in; exactly *N* cell boxes come out.
3. **Route A next.** One DLL means the fastest edit–build–test loop, a real GUI
   home beside Elber's puzzles, and no edits to his code.
4. **Route B when a script needs it** — calling the BSP from a batch `.irt`, or
   from inside `puz_vol.irt`. It reuses the same core file; only the glue
   differs. Once B exists, the plugin can call the core through `Irit64.dll`
   instead of carrying its own copy.
5. **Route C** only as a throwaway prototype, or on a machine with no compiler.

> **One rule across all of it:** the algorithm lives in **one** `.c` file.
> Compile that file into the plugin *or* export it from `Irit64.dll` — never
> keep two diverging copies. It is the same principle as the Qt app's "never
> re-derive adjacency".

---

## 4. Step 0 — a Qt-free core

**Where the core lives.** Two new files, `PuzBspCore.h` and `PuzBspCore.c`.
Create them in the plugin's folder first,
`C:\irit\extra\guirit\Src\GuIritDllBspPuzzle\` — make the folder now if §5.1 has
not been done yet. If you later do Route B, **move** both files into
`C:\irit\irit\ext_lib\` (§6), so only one copy ever exists.

**How to read the code blocks.** Every block in §4–§8 carries one of these
labels:

| Label | Meaning |
|---|---|
| **NEW FILE** | create this file; the block is its content, or the part named |
| **EDIT FILE** | the file already exists; add exactly what the label says, where it says |
| **RUN** | a command to type, not file content |
| **REFERENCE** | shown to explain; do not paste it anywhere |

### 4.1 What to port, and what not to

Port only the **decision** — where to cut. IRIT already owns the geometry:
region extraction, Booleans, tessellation.

| From the Qt app | Where | Becomes |
|---|---|---|
| `MaterialField::build` | `MaterialField.cpp:9-134` | `PuzBspFieldBuild()` |
| `MaterialField::volumeIn` | `MaterialField.cpp:275` | `PuzBspFieldVolumeIn()` |
| `MaterialField::isConnected` | `MaterialField.cpp:219` | `PuzBspFieldIsConnected()` |
| `PuzzleDivider::buildBspTree` — leaf choice, 24 candidates, cost, connectivity gate | `PuzzleDivider.cpp:630-850` | `PuzBspBuildCells()` |
| world → parameter mapping | `AppController.cpp:561` | `PuzBspCellToTV()` |

**Do not port** `CageBoolean`'s lump repair, `IritSolid`, `PieceExport` or the
planner. They exist because the app rebuilds polygons from `MeshData`; inside
IRIT you work on IRIT's own objects and most of that problem does not arise
(§9 says which part still does).

### 4.2 Qt to plain C

| Qt | Plain C |
|---|---|
| `QVector<double>` | `IrtRType *` from `IritMalloc`, released with `IritFree` |
| `QVector<quint8>` | `unsigned char *` |
| `QVector<QVector<float>> crossings` | two passes over the triangles: count crossings per column, then fill one flat array with per-column offsets |
| `std::sort` of crossings / candidates | `qsort` with a comparator |
| `qBound`, `qMax`, `qMin` | small `static` helpers in the file — do not assume IRIT macro names |
| `qEnvironmentVariableIsSet("BSP_LOG")` | a `Verbose` argument |
| `std::printf` logging | `fprintf(stderr, …)` in ext_lib, as `ext_lib\explfunc.c` does; `GuIritMdlrDllPrintf` in a plugin |
| `QRandomGenerator rng(seed)` | **not needed** on the material path — see below |

> **The port is deterministic, so it can be checked exactly.** On the material
> path the random draw at `PuzzleDivider.cpp:744` is always overwritten by the
> chosen candidate at `PuzzleDivider.cpp:818`, and an axis with no usable
> candidate is skipped with `continue` before that line. The leaf to split is
> always the fullest one, never sampled. So **cut positions do not depend on the
> RNG at all**, and a faithful C port must reproduce the Qt app's cuts to
> printed precision. That is the test in §10. (The RNG does still matter on the
> volume-only fallback, when no field can be built.)

Carry these two lines across **verbatim** — each one fixed a measured failure:

**NEW FILE** · `PuzBspCore.c` (created in §4.4) · inside `PuzBspFieldBuild()`,
at the same two places they sit in `MaterialField::build`

```c
/* MaterialField.cpp:72 - sample off the voxel centre, or a UV sphere's
   meridians leave an empty curtain of columns that splits the model. */
const double jx = 0.5 + 1.0 / 512.0, jy = 0.5 + 1.0 / 337.0;

/* MaterialField.cpp:104 - drop an unpaired crossing instead of flooding
   the rest of the column. */
const int pairs = count & ~1;
```

### 4.3 Reading an IRIT polygon object

The voxel fill only needs triangles. The structure fields were checked in
`inc_irit\iritprsr.h`: an object's `U.Pl` is a `Pnext` chain of
`IritPrsrPolygonStruct` (line 425); each polygon's `PVertex` is a `Pnext` chain
of `IritPrsrVertexStruct` (line 377) holding `Coord`.

**NEW FILE** · `PuzBspCore.c` · near the top, below the `#include` lines. It is
`static`: used only inside this file.

```c
/* Fan-triangulates every polygon of a polygonal object. */
static void PuzBspForEachTriangle(const IritPrsrObjectStruct *PObj,
                                  void (*Tri)(const IrtPtType A,
                                              const IrtPtType B,
                                              const IrtPtType C,
                                              void *Data),
                                  void *Data)
{
    const IritPrsrPolygonStruct *Pl;

    for (Pl = PObj -> U.Pl; Pl != NULL; Pl = Pl -> Pnext) {
        const IritPrsrVertexStruct
            *V0 = Pl -> PVertex,
            *V;

        if (V0 == NULL || V0 -> Pnext == NULL)
            continue;

        /* IRIT vertex lists may be CIRCULAR - stop on returning to V0 as
           well as on NULL, or this loops forever. */
        for (V = V0 -> Pnext;
             V -> Pnext != NULL && V -> Pnext != V0;
             V = V -> Pnext)
            Tri(V0 -> Coord, V -> Coord, V -> Pnext -> Coord, Data);
    }
}
```

Guard the input first with `IRIT_PRSR_IS_POLY_OBJ(Obj)` (`iritprsr.h:496`) **and**
`IRIT_PRSR_IS_POLYGON_OBJ(Obj)` (`iritprsr.h:563`) — polylines and point lists
share the POLY object type, and only the second test excludes them.

Two notes on correctness:

- **Winding does not matter here.** The parity fill uses only the *heights* of
  crossings, never their orientation.
- **A fan is exact only for convex polygons.** STL input is triangles already.
  If a model may carry non-convex polygons, triangulate first — but **do not use
  `IritGeomConvertPolysToTriangles`**: the Qt app found it access-violates on
  valid inputs (`IritMesh.cpp:30-33`). Port the app's ear clipper instead.

### 4.4 The core's interface

**NEW FILE** · `C:\irit\extra\guirit\Src\GuIritDllBspPuzzle\PuzBspCore.h` · the whole file

```c
/* PuzBspCore.h - shared by the plugin (Route A) and ext_lib (Route B). */

#include "inc_irit/irit_sm.h"
#include "inc_irit/iritprsr.h"
#include "inc_irit/triv_lib.h"

#if defined(__cplusplus) || defined(c_plusplus)
extern "C" {
#endif

typedef struct PuzBspCellStruct {
    IrtRType Lo[3], Hi[3];      /* Local world box; (0,0,0) = model's min. */
} PuzBspCellStruct;

/* Splits a polygonal solid into exactly NumPieces axis-aligned cells by the
   material-aware BSP.  MaxRes is the voxel resolution of the longest axis
   (96 in the Qt app).  Returns the cell count and sets *Cells (IritMalloc -
   caller calls IritFree), or 0 on failure.  ModelMin receives the offset
   that turns local cell coordinates back into world coordinates.          */
int PuzBspBuildCells(const IritPrsrObjectStruct *PolyModel,
                     int NumPieces,
                     int MaxRes,
                     PuzBspCellStruct **Cells,
                     IrtRType ModelMin[3],
                     IrtRType ModelExt[3]);

/* The sub-trivariate of Cage covering one cell: map the cell into Cage's
   parameter domain, then three IritTrivTVRegionFromTV calls, one per
   direction - exactly doRegion() in PuzzleDivider.cpp:330.               */
TrivTVStruct *PuzBspCellToTV(const TrivTVStruct *Cage,
                             const PuzBspCellStruct *Cell,
                             const IrtRType ModelExt[3]);

/* One cell as a polygonal box in world coordinates: the live preview in
   GuIrit, and the Boolean operand when trimming.  Body: step 1 of 5.6.    */
IritPrsrObjectStruct *PuzBspCellBox(const PuzBspCellStruct *Cell,
                                    const IrtRType ModelMin[3]);

/* All pieces as one list object.  Output: 0 = cell boxes,
   1 = sub-trivariates, 2 = pieces trimmed to the model (Elber Section 5).
   Spacing moves pieces apart; 0 keeps them assembled.  Body: 5.6.        */
IritPrsrObjectStruct *PuzBspMakePieces(const IritPrsrObjectStruct *Model,
                                       const PuzBspCellStruct *Cells,
                                       int n,
                                       const IrtRType ModelMin[3],
                                       const IrtRType ModelExt[3],
                                       int Output,
                                       IrtRType Spacing,
                                       const char *Name);

#if defined(__cplusplus) || defined(c_plusplus)
}
#endif
```

**NEW FILE** · `C:\irit\extra\guirit\Src\GuIritDllBspPuzzle\PuzBspCore.c` · starts with
`#include "PuzBspCore.h"`, then the `static` helpers from §4.2–4.3, then the
body of every function declared above.

For `PuzBspCellToTV`, take the argument order of `IritTrivTVDomain`
(`triv_lib.h:509`) from `Trivariate::domain` in the Qt app rather than from
memory, and **do not assume the cage's domain is [0,1]** — the app reads it and
maps into it for a reason. The cage itself is
`IritTrivNSPrimBox(MinX, MinY, MinZ, MaxX, MaxY, MaxZ)`, in that argument order
(`Trivariate.cpp:160`).

The `extern "C"` guard is not decoration: the plugin is C++, ext_lib is C, and
both must link the same symbols.

---

## 5. Route A — a GuIrit panel (plugin DLL)

Modelled line for line on `GuIritDllPuzzles`, which is the closest working
example on the machine.

**Files for this route**

| Action | File | Section |
|---|---|---|
| create folder | `C:\irit\extra\guirit\Src\GuIritDllBspPuzzle\` | 5.1 |
| create | `…\GuIritDllBspPuzzle\GuIritDllBspPuzzle.cpp` | 5.3 – 5.7 |
| create | `…\GuIritDllBspPuzzle\GuIritDllBspPuzzle.def` | 5.2 |
| create | `…\GuIritDllBspPuzzle\PuzBspCore.h` and `PuzBspCore.c` | 4, 5.6 |
| create — copy an icon, redraw it | `…\GuIritDllBspPuzzle\Icons\IconBspPuzzle.xpm` | 5.3 |
| create — copy a project, edit it | `C:\irit\extra\guirit\Src\WindowsVC2026\GuIritDllBspPuzzle\GuIritDllBspPuzzle.vcxproj` | 5.1 |
| edit — add the project | `C:\irit\extra\guirit\Src\WindowsVC2026\GuIrit.sln` | 5.1 |
| edit — optional | `C:\irit\extra\guirit\Src\RunTime\GuIritData\GuIritinit.irt` | 5.8 |

Nothing in Elber's plugins or in IRIT itself is edited on this route.

**Order inside `GuIritDllBspPuzzle.cpp`**, top to bottom — C++ needs each thing
declared before it is used:

1. the `#include` lines and the icon include (§5.3)
2. the callback's forward declaration and the table (§5.3)
3. the local-data class (§5.4)
4. the callback (§5.5)
5. `_IrtMdlrDllRegister` (§5.7)

### 5.1 Create the project

Keep it a **separate** DLL rather than a fifth row in Elber's Puzzles DLL: an
IRIT/GuIrit update then cannot overwrite your work, and you never have to merge.

1. Copy `C:\irit\extra\guirit\Src\GuIritDllPuzzles\` →
   `C:\irit\extra\guirit\Src\GuIritDllBspPuzzle\`, and delete the copied
   sources you will not use.
2. Copy `Src\WindowsVC2026\GuIritDllPuzzles\GuIritDllPuzzles.vcxproj` →
   `Src\WindowsVC2026\GuIritDllBspPuzzle\GuIritDllBspPuzzle.vcxproj`.
3. In the new `.vcxproj`: point the `ClCompile` entries at your files, change
   `ModuleDefinitionFile` to your `.def`, and **give it a new `ProjectGuid`** —
   a duplicate GUID makes Visual Studio treat the two projects as one.
4. Open `GuIrit.sln` → *Add → Existing Project*.

What the copy inherits, checked in the Example and Puzzles projects:

| Setting | Value |
|---|---|
| Links | `GuIritDllExtensions_64.lib;irit64.lib` (Release x64) |
| Includes | `..\..\GuIritDllExtensions;..\..\GuIrit\Include;..\..\GuIrit\Modeler;c:\Irit\Irit` |
| Runtime | `MultiThreadedDLL` — the same CRT as `GuIrit64.exe` |
| Output | `$(ExeDir64)$(ProjectName)_64.dll`, with `ExeDir64 = ..\ntbin64\` → `Src\WindowsVC2026\ntbin64\GuIritDllBspPuzzle_64.dll` |

Add `PuzBspCore.c` to the project as a C file alongside the `.cpp`; mixing is
fine in a `.vcxproj`.

### 5.2 The export file

**NEW FILE** · `C:\irit\extra\guirit\Src\GuIritDllBspPuzzle\GuIritDllBspPuzzle.def` · the whole file,
identical to every plugin's:

```
EXPORTS
_IrtMdlrDllRegister
```

### 5.3 The table row

`IrtMdlrFuncTableStruct` (`GuIrit\Modeler\IrtMdlr.h:376`) describes one button:
its id, name prefix, icon, string id, names, help, callback, flags, return type
and parameters.

**NEW FILE** · `C:\irit\extra\guirit\Src\GuIritDllBspPuzzle\GuIritDllBspPuzzle.cpp` · the top of the file
(items 1–2 of the order above)

```cpp
#include "IrtDspBasicDefs.h"
#include "IrtMdlr.h"
#include "IrtMdlrFunc.h"
#include "IrtMdlrDll.h"
#include "GuIritDllExtensions.h"
#include "PuzBspCore.h"

#include "Icons/IconBspPuzzle.xpm"

static void IrtMdlrBspPuzzle(IrtMdlrFuncInfoClass *FI);

IRT_DSP_STATIC_DATA IrtMdlrFuncTableStruct IrtMdlrBspPuzzleFuncTable[] =
{
    { 0,                                  /* FuncId                        */
      "BspPzl",                           /* FuncNameDefPrefix - new names  */
      IconBspPuzzle,                      /* FuncIcon                      */
      "IRT_MDLR_PUZZLE_BSP",              /* StrId - unique across GuIrit  */
      "BspPzl",                           /* FName - used as GuIrit_BspPzl */
      "BSP Volumetric Puzzle",            /* FuncName - button caption     */
      "Divides a watertight polygonal solid into exactly N pieces.\n"
      "Cuts are chosen by a cost function against the material inside\n"
      "each cell, and a cut is only accepted if both halves stay one lump.",
      IrtMdlrBspPuzzle, NULL,
      IRT_MDLR_PARAM_HIDE_GEOM_PARAM_DFLT_ON |
          IRT_MDLR_PARAM_INTERMEDIATE_UPDATE_DFLT_ON,
      IRT_MDLR_OLST_EXPR,                 /* returns a list of pieces      */
      6, IRT_MDLR_PARAM_EXACT,
      { IRT_MDLR_STRING_EXPR, IRT_MDLR_POLY_EXPR, IRT_MDLR_INTEGER_EXPR,
        IRT_MDLR_INTEGER_EXPR, IRT_MDLR_SELECTION_EXPR, IRT_MDLR_NUMERIC_EXPR },
      { "Name", "Object", "Pieces", "Voxel\nRes.", "Output", "Spacing" },
      { "Resulting object's name",
        "Watertight polygonal solid to divide",
        "Exact number of pieces",
        "Voxel resolution of the longest axis (96 in the Qt app)",
        "Cells, sub-trivariates, or pieces trimmed to the model",
        "Gap between pieces in the result; 0 keeps them assembled" } }
};
```

For the icon, copy `Icons\IconPuzzleLinearSlice.xpm`, redraw it, and **rename the
array inside the file** to match the `IconBspPuzzle` name used above.

### 5.4 Binding dialog fields to C++ members

Each dialog field is written straight into a member of a local-data class
through `ParamVals[i]`. Index 0 is the name, handled by the base class; your
fields start at 1, in the same order as the table row. This mirrors
`IrtMdlrPuzzleLinearSliceLclClass` (`GuIritDllPuzzles.h:41-80`), including the
member types.

**NEW FILE, continued** · `GuIritDllBspPuzzle.cpp` · directly below the table

```cpp
IRT_DSP_STATIC_DATA const char
    *IrtMdlrBspOutputStr = "Cells;Sub-trivariates;Trimmed pieces";

class IrtMdlrBspPuzzleLclClass: public IrtMdlrLclDataClass
{
    public:
        IrtMdlrBspPuzzleLclClass(IrtMdlrFuncInfoClass *FI):
            IrtMdlrLclDataClass(FI),
            Object(),
            NumPieces(8),
            VoxelRes(96),
            Output(IrtMdlrSelectExprClass(IrtMdlrBspOutputStr, 2)),
            Spacing(0.0)
        {
            ParamVals[1] = (void *) &Object;      /* IRT_MDLR_POLY_EXPR      */
            ParamVals[2] = (void *) &NumPieces;   /* IRT_MDLR_INTEGER_EXPR   */
            ParamVals[3] = (void *) &VoxelRes;    /* IRT_MDLR_INTEGER_EXPR   */
            ParamVals[4] = (void *) &Output;      /* IRT_MDLR_SELECTION_EXPR */
            ParamVals[5] = (void *) &Spacing;     /* IRT_MDLR_NUMERIC_EXPR   */
        }

        IrtMdlrObjectExprClass Object;
        unsigned int NumPieces;
        unsigned int VoxelRes;
        IrtMdlrSelectExprClass Output;
        IrtRType Spacing;
};
```

The second argument of `IrtMdlrSelectExprClass` is the initial selection —
LinearSlice passes `2` against `"X axis;Y axis;Z axis"`. Here `2` selects
*Trimmed pieces*. (Read from the pattern, not from the class source; if the
dialog opens on a different entry, that argument is the one to change.)

### 5.5 The callback

The pattern is `IrtMdlrPuzzleLinearSlice` (`GuIritDllPuzzles.cpp:698`) and
`IrtMdlrPuzBooleansAndFinish` (`:644`): set input domains on the first
invocation; **preview cheaply** while the user edits; do the expensive Booleans
only on **OK** or **Apply**.

**NEW FILE, continued** · `GuIritDllBspPuzzle.cpp` · below the local-data class

```cpp
static void IrtMdlrBspPuzzle(IrtMdlrFuncInfoClass *FI)
{
    IRT_MDLR_DLL_LCL_DATA_INIT(FI, IrtMdlrBspPuzzleLclClass, TRUE);

    if (FI -> CnstrctState == IRT_MDLR_CNSTRCT_STATE_CANCEL)
        return;

    if (FI -> InvocationNumber == 0) {
        GuIritMdlrDllSetIntInputDomain(FI, 1, 512, 2);          /* Pieces    */
        GuIritMdlrDllSetIntInputDomain(FI, 8, 256, 3);          /* Voxel res */
        GuIritMdlrDllSetRealInputDomain(FI, 0.0, IRIT_INFNTY, 5); /* Spacing */
    }

    const IritPrsrObjectStruct
        *Model = LclData -> Object.GetIPObj();

    if (Model == NULL)
        return;                                 /* nothing selected yet     */

    if (!IRIT_PRSR_IS_POLY_OBJ(Model) || !IRIT_PRSR_IS_POLYGON_OBJ(Model)) {
        GuIritMdlrDllPrintf(FI, IRT_DSP_LOG_WARNING,
                            "BspPzl: select a polygonal solid.");
        return;
    }

    PuzBspCellStruct *Cells = NULL;
    IrtRType ModelMin[3], ModelExt[3];
    const int n = PuzBspBuildCells(Model, (int) LclData -> NumPieces,
                                   (int) LclData -> VoxelRes,
                                   &Cells, ModelMin, ModelExt);

    GuIritMdlrDllClearThisModelingFuncTempDisplayObjs(FI);

    if (n <= 0) {
        GuIritMdlrDllPrintf(FI, IRT_DSP_LOG_WARNING,
                            "BspPzl: could not voxelise the object - "
                            "is it a closed solid?");
        return;
    }

    if (FI -> CnstrctState == IRT_MDLR_CNSTRCT_STATE_OK ||
        FI -> CnstrctState == IRT_MDLR_CNSTRCT_STATE_APPLY) {
        IritPrsrObjectStruct
            *Result = PuzBspMakePieces(Model, Cells, n, ModelMin, ModelExt,
                                       (int) LclData -> Output.GetIndex(),
                                       LclData -> Spacing,
                                       LclData -> GetName());

        if (Result != NULL)
            GuIritMdlrDllInsertModelingFuncObj(FI, Result);
        else
            GuIritMdlrDllPrintf(FI, IRT_DSP_LOG_WARNING,
                                "BspPzl: no piece survived trimming.");
    }
    else {
        /* Live preview: the cells only.  Voxelising is cheap enough to
           rerun on every edit; the Booleans are not.                    */
        for (int i = 0; i < n; i++)
            GuIritMdlrDllAddTempDisplayObject(FI,
                                              PuzBspCellBox(&Cells[i], ModelMin),
                                              FALSE, 3,
                                              IRT_DSP_GEOM_HIGHLIGHT3, FALSE);
    }

    IritFree(Cells);
}
```

Every GuIrit call above appears in Elber's Puzzles plugin with these arguments;
the prototypes are in `GuIritDllExtensions\GuIritDllExtensions.h` —
`GuIritMdlrDllPrintf` (line 165), `GuIritMdlrDllSetIntInputDomain` (220),
`GuIritMdlrDllSetRealInputDomain` (225), `GuIritMdlrDllInsertModelingFuncObj`
(414).

### 5.6 Making the pieces

**NEW FILE** · `PuzBspCore.c` · the bodies of `PuzBspMakePieces` and
`PuzBspCellBox`, both already declared in `PuzBspCore.h` (§4.4). They live in the
core rather than in the plugin so Route B uses them unchanged — do **not** make
them `static`.

What it does, step by step:

1. **Cell box as polygons** — this is the body of `PuzBspCellBox`.
   `IritGeomPrimGenBOXObject(Pt, WidthX, WidthY,
   WidthZ)` (`geom_lib.h:1203`) with `Pt = ModelMin + Cell.Lo` and widths
   `Cell.Hi − Cell.Lo`. IRIT's primitives can come out polygonal or freeform;
   the switch is `IritGeomPrimSetGeneratePrimType` (`geom_lib.h:1200`), which
   returns the previous setting — restore it afterwards.
   > ⚠️ **Unverified encoding.** The parameter is named `PolygonalPrimitive`,
   > while the script state maps *Polygonal* to `0`
   > (`GuIrit_PrimitiveType` in `GuIritinit.irt`). Those may not use the same
   > convention. Read the setter's implementation in `geom_lib`, and assert
   > `IRIT_PRSR_IS_POLY_OBJ(Box)` on the result either way.
2. **Output 0** — append the box.
3. **Output 1** — build the cage once with `IritTrivNSPrimBox` over the model's
   bounding box, then `PuzBspCellToTV` per cell, wrapped with
   `IritPrsrGenTRIVARObject`. These are the genuine V-rep pieces.
4. **Output 2** — `IritBooleanAND` of a **copy** of the model with the cell box.
   For a bounding cage this is exact, because every sub-trivariate of a box cage
   *is* an axis-aligned box. Once the cage is a fitted trivariate
   (`POLYMESH2TV` in script, `IritFitTV2PolyMesh` in `irit\freefrm10.c:2558`),
   the cell is no longer a box and you must polygonise the sub-trivariate
   instead.
   > **Copy the model for every Boolean.** The Qt app measured that IRIT's
   > Booleans modify their operands (`CageBoolean.cpp:155`). For how an
   > in-GuIrit plugin calls Booleans against the selected object, read
   > `IrtMdlrPuzComputeBooleans` in `GuIritDllPuzzles.cpp` — it is the closest
   > working reference on the machine.
5. **Assemble the result** exactly as `IrtMdlrPuzBooleansAndFinish` does:
   `IritPrsrGenListObject(Name, FirstPiece, NULL)` (`allocate.h:204`), then
   `IritPrsrListObjectAppend(List, Piece)` (`allocate.h:90`) for the rest. Name
   each piece (`piece_000`, …) so it can be picked out in GuIrit's object tree.
6. **Spacing** — translate each piece away from the model centre along its
   cell-centre offset, scaled by `Spacing`: the app's explode. Leave it at 0 for
   anything you intend to measure or reassemble.

A failed Boolean must fail **locally**: log which piece and why with
`GuIritMdlrDllPrintf`, skip it, continue — the same rule the Qt app follows.

### 5.7 Registration

**NEW FILE, continued** · `GuIritDllBspPuzzle.cpp` · the end of the file

```cpp
extern "C" bool _IrtMdlrDllRegister(void)
{
    GuIritMdlrDllRegister(IrtMdlrBspPuzzleFuncTable,
                          sizeof(IrtMdlrBspPuzzleFuncTable) /
                              sizeof(IrtMdlrFuncTableStruct),
                          "BSP Puzzles",          /* block name in the GUI */
                          IconBspPuzzle);
    return true;
}
```

Signature from `GuIritDllExtensions.h:158`: table, count, block name, block
icon, and a defaulted version. `extern "C"` matters — the loader looks the
symbol up by its unmangled name.

### 5.8 The `GuIrit_` wrapper is optional

After a panel function runs, GuIrit writes a line of the form
`ObjName = GuIrit_<FName>( params )` to its script log
(`GuIrit\Modeler\IrtMdlrFunc.cpp:865-874`). A matching script function in
`GuIritinit.irt` makes that line re-executable.

It is **not required** for the panel to work: Elber's four Puzzles entries have
no wrapper at all — `GuIrit_LinPzlSlc`, `GuIrit_RndPzlSlc` and `GuIrit_SltTbPzl`
each have zero matches in `GuIritinit.irt`, while Trivariate entries such as
`GuIrit_RuledTV` do have one.

Add one only after Route B exists, so it has something to call. Match its
parameter list to what GuIrit actually logs — run the panel once and copy the
logged line, rather than guessing whether the name field is included.

**EDIT FILE** (optional) ·
`C:\irit\extra\guirit\Src\RunTime\GuIritData\GuIritinit.irt` · append the
`GuIrit_BspPzl = function( ... ): ...` definition at the end of the file.

> ⚠️ **Edit the source copy.** `C:\irit\GuIritSetup2026.bat` robocopies
> `C:\irit\extra\guirit\Src\RunTime\GuIritData` over the installed
> `GuIritData64`. An edit made to
> `C:\irit\irit\windowsVC2026\ntbin64\GuIritData64\GuIritinit.irt` is silently
> overwritten the next time setup runs. Edit
> `Src\RunTime\GuIritData\GuIritinit.irt`.

### 5.9 Build and deploy

1. In `GuIrit.sln`, build **Release | x64** of `GuIritDllBspPuzzle` →
   `Src\WindowsVC2026\ntbin64\GuIritDllBspPuzzle_64.dll`.
2. Copy it into `C:\irit\irit\windowsVC2026\ntbin64\GuIritData64\Extensions\`
   — by hand, or by running `C:\irit\GuIritSetup2026.bat`, whose fifth line
   copies every `*.dll` from that output folder into `Extensions`. Note the
   script also recopies `GuIrit64.exe` and the whole `GuIritData` tree.
3. Start `GuIrit64.exe`. A *BSP Puzzles* block should appear. If it does not,
   GuIrit's log reports `Failed to register Guirit DLL ext. "<name>"`
   (`IrtDspGuiApp.cpp:754`) — almost always a missing `extern "C"` or `.def`.

**Pair the builds.** `_64.dll` goes with `GuIrit64.exe` and `Irit64.dll`;
`_D64.dll` with `GuIritD64.exe` and `IritD64.dll`. Mixing a Debug plugin into a
Release GuIrit loads two C runtimes into one process.

**Relink `Irit64.dll` first if you have rebuilt IRIT's libraries.** Here the
libs (2026-09-01) are newer than the DLL (2026-05-04). A plugin compiled against
the current headers should run against a DLL built from the same tree.

---

## 6. Route B — an IRIT built-in script function

This follows Elber's own procedure in `C:\irit\irit\ext_lib\README.1st` step by
step, with the two things it leaves out for GuIrit: the DLL export, and the
relink of `GuIrit64.exe`.

The result is a script function:

**REFERENCE** · how it will be called from any `.irt` script once built; not a file

```
Pieces = PUZBSP( Model, NumPieces, VoxelRes, Output );
```

**Files for this route**

| Action | File | Section |
|---|---|---|
| create | `C:\irit\irit\ext_lib\puzbsp.c` | 6.1 |
| move here from the plugin folder | `C:\irit\irit\ext_lib\PuzBspCore.h` and `PuzBspCore.c` | 4 |
| edit | `C:\irit\irit\inc_irit\ext_lib.h` | 6.2 |
| edit | `C:\irit\irit\windowsVC2026\ext_lib\ext_lib.vcxproj` | 6.3 |
| edit | `C:\irit\irit\ext_lib\makefile.wnt` and `Makefile.am` | 6.3 |
| edit | `C:\irit\irit\irit\inptprsl.h` | 6.4 |
| edit | `C:\irit\irit\irit\inptevl0.c` | 6.5 |
| edit, then run `make_defs.cmd` | `C:\irit\irit\dll_defs\irit_all_dll_defs.txt` | 6.6 |
| edit — plugin: drop its copy of `PuzBspCore.c`, include `"ext_lib/PuzBspCore.h"` | `GuIritDllBspPuzzle.vcxproj`, `GuIritDllBspPuzzle.cpp` | 6.6 |
| create | `C:\irit\irit\scripts\puzbsp_test.irt` | 6.8 |
| edit | `C:\irit\irit\docs\irit.src` | 6.9 |

### 6.1 The C function

New file `C:\irit\irit\ext_lib\puzbsp.c`. Interpreter functions receive numbers
as `IrtRType *` and objects as `IritPrsrObjectStruct *`, and return a new object
or `NULL` — compare `RegionFromTrivarObject` (`irit\freefrm4.c:786`), the C side
of `TREGION`.

**NEW FILE** · `C:\irit\irit\ext_lib\puzbsp.c` · the whole file

```c
/******************************************************************************
* puzbsp.c - material-aware BSP division of a polygonal solid, for the IRIT   *
* interpreter.  The algorithm itself is in PuzBspCore.c, shared with GuIrit.  *
******************************************************************************/

#include <stdio.h>
#include "inc_irit/irit_sm.h"
#include "inc_irit/iritprsr.h"
#include "inc_irit/allocate.h"
#include "inc_irit/ext_lib.h"
#include "PuzBspCore.h"

/*****************************************************************************
* DESCRIPTION:                                                               M
*   Divides a polygonal solid into exactly NumPieces pieces.                 M
*                                                                            *
* PARAMETERS:                                                                M
*   PolyObj:    Watertight polygonal solid.                                  M
*   RNumPieces: Number of pieces.                                            M
*   RVoxelRes:  Voxel resolution of the longest axis.                        M
*   ROutput:    0 = cell boxes, 1 = sub-trivariates, 2 = trimmed pieces.     M
*                                                                            *
* RETURN VALUE:                                                              M
*   IritPrsrObjectStruct *:  A list object of pieces, or NULL on failure.    M
*                                                                            *
* KEYWORDS:                                                                  M
*   IritExtPuzBsp                                                            M
*****************************************************************************/
IritPrsrObjectStruct *IritExtPuzBsp(IritPrsrObjectStruct *PolyObj,
                                    IrtRType *RNumPieces,
                                    IrtRType *RVoxelRes,
                                    IrtRType *ROutput)
{
    PuzBspCellStruct *Cells = NULL;
    IrtRType ModelMin[3], ModelExt[3];
    IritPrsrObjectStruct *Result;
    int n;

    if (!IRIT_PRSR_IS_POLY_OBJ(PolyObj) || !IRIT_PRSR_IS_POLYGON_OBJ(PolyObj)) {
        fprintf(stderr, "PUZBSP: expected a polygonal solid\n");
        return NULL;
    }

    n = PuzBspBuildCells(PolyObj, (int) *RNumPieces, (int) *RVoxelRes,
                         &Cells, ModelMin, ModelExt);
    if (n <= 0) {
        fprintf(stderr, "PUZBSP: could not voxelise - is it a closed solid?\n");
        return NULL;
    }

    Result = PuzBspMakePieces(PolyObj, Cells, n, ModelMin, ModelExt,
                              (int) *ROutput, 0.0, "PuzBsp");
    IritFree(Cells);
    return Result;
}
```

The comment block uses IRIT's `M` markers on purpose: they are how the
programmer's manual is extracted from the sources (see `docs\prog_man`). The
plain casts `(int) *RNumPieces` avoid depending on interpreter-private macros
from ext_lib.

`PuzBspMakePieces` is the same helper as §5.6. Write it in C in
`PuzBspCore.c` and both routes share it.

### 6.2 Declare it

**EDIT FILE** · `C:\irit\irit\inc_irit\ext_lib.h` · inside the existing
`extern "C"` block. The first line below is already there (line 33); **add the
lines under it**:

```c
void IritExtExampleFunction(IrtRType *R, IrtVecType V);

IritPrsrObjectStruct *IritExtPuzBsp(IritPrsrObjectStruct *PolyObj,
                                    IrtRType *RNumPieces,
                                    IrtRType *RVoxelRes,
                                    IrtRType *ROutput);
```

### 6.3 Compile it into ext_lib

- **EDIT FILE** `C:\irit\irit\windowsVC2026\ext_lib\ext_lib.vcxproj` (Visual Studio): add `..\..\ext_lib\puzbsp.c` and `..\..\ext_lib\PuzBspCore.c`
  as `ClCompile` entries in `windowsVC2026\ext_lib\ext_lib.vcxproj`, next to
  `..\..\ext_lib\explfunc.c`.
- **EDIT FILE** `C:\irit\irit\ext_lib\makefile.wnt` (nmake): extend `OBJS = explfunc.$(IRIT_OBJ_PF)` in `ext_lib\makefile.wnt`.
- **EDIT FILE** `C:\irit\irit\ext_lib\Makefile.am` (Unix builds only): add both `.c` files.

### 6.4 Give it a token

**EDIT FILE** · `C:\irit\irit\irit\inptprsl.h` · **add one line**, `IP_PUZBSP,`.
It returns an object, so it belongs in `ObjValueFuncType`: put it last, after
`IP_JOINCRVCRV,` and before `IRIT_PRSR_OBJ_VAL_LAST` (the enum closes at line
783). The other lines are shown only so you can find the spot:

```c
    IP_HOTWIRECUT,
    IP_JOINCRVCRV,
    IP_PUZBSP,

    IRIT_PRSR_OBJ_VAL_LAST
} ObjValueFuncType;
```

Capacity is not a problem: object tokens run from `IRIT_PRSR_OBJ_FUNC_OFFSET`
(200) to `IRIT_PRSR_OBJ_FUNC_END` (799), and 500 of those 600 are used.

### 6.5 Add the table row

**EDIT FILE** · `C:\irit\irit\irit\inptevl0.c` · `ObjFuncTable[]`, which starts at
line 96. Two changes: **add a comma** at the end of the existing `JOINCRVCRV` row
(currently the last row), then **add the `PUZBSP` row** under it, before `};`.
The first line below is that existing row with its new comma:

```c
    { IPNP(JOINCRVCRV), IPNP2(IritJointCrvCrv3D),     7,        { CURVE_EXPR,  CURVE_EXPR, NUMERIC_EXPR, NUMERIC_EXPR, NUMERIC_EXPR, NUMERIC_EXPR, NUMERIC_EXPR }, CURVE_EXPR },
    { IPNP(PUZBSP),     IPNP2(IritExtPuzBsp),         4,        { POLY_EXPR, NUMERIC_EXPR, NUMERIC_EXPR, NUMERIC_EXPR }, OLST_EXPR }
};
```

Reading the row: `IPNP(F)` expands to `"F", IP_F` and `IPNP2(F)` to `"F", F`
(`inptevl0.c:55-56`). So the script name is `PUZBSP`, its token `IP_PUZBSP`, and
the C function `IritExtPuzBsp` bound by name and pointer. Then the parameter
count, the parameter types, and the return type. Script names are limited to
`IP_FUNC_NAME_LEN` (31) and parameters to `IP_FUNC_MAX_PARAM` (12)
(`inptprsl.h:104-105`). `TDIVIDE` (`DivideTrivarObject`, `freefrm4.c:709`) is a
working example of a function that returns `OLST_EXPR`.

> ### ⚠️ Why "append at the end" is not a style preference
>
> The evaluator does not search the table. It **indexes** it by token:
>
> ```c
> ObjFuncTable[Root -> NodeKind - IRIT_PRSR_OBJ_FUNC_OFFSET]    /* inptevl1.c:99 */
> ```
>
> So row *n* must belong to token 200 + *n*. Insert a row in the middle, or an
> enum entry at a different position from its row, and **every function after
> it silently calls its neighbour's C code**. Nothing fails to compile. Appending
> both at the end keeps them aligned by construction.

### 6.6 Export it from `Irit64.dll`

`GuIrit64.exe` does not link ext_lib statically — it reaches ext_lib code through
`Irit64.dll` via `irit64.lib`. A function the DLL does not export therefore
**fails to link GuIrit**, even though `irit64.exe` (which links the static libs)
builds fine. That is why `IritExtExampleFunction` is already listed as an export
(`dll_defs\irit_all_dll_defs.txt:3732`).

1. **EDIT FILE** · add `IritExtPuzBsp` — and `PuzBspBuildCells` / `PuzBspMakePieces` if the
   plugin should call them through the DLL — to
   `C:\irit\irit\dll_defs\irit_all_dll_defs.txt`.
2. **RUN** · regenerate the `.def` files: `dll_defs\make_defs.cmd C:\irit\irit\dll_defs`.
   It rebuilds `irit.def`, `irit64.def`, `iritD64.def`, `iritP64.def` from that
   list and moves them up one directory. **Do not edit `irit64.def` by hand** —
   the next regeneration discards it.

### 6.7 Rebuild in dependency order

| # | Build | Produces |
|---|---|---|
| 1 | `windowsVC2026\ext_lib` | `lib64\IritExt64.lib` |
| 2 | `windowsVC2026\iritDll` | `ntbin64\Irit64.dll`, `lib64\Irit64.lib` |
| 3 | `windowsVC2026\inpt_lib` | `lib64\iritInpt64.lib` — the table |
| 4 | `windowsVC2026\irit` | `ntbin64\irit64.exe` — **test here first** (§6.8) |
| 5 | `C:\irit\extra\guirit\Src\WindowsVC2026\GuIrit.sln` → `GuIrit` | `Src\WindowsVC2026\ntbin64\GuIrit64.exe` |
| 6 | `C:\irit\GuIritSetup2026.bat` | copies `GuIrit64.exe` and extensions into `C:\irit\irit\windowsVC2026\ntbin64` |

Before step 5, confirm in the GuIrit project's *Library Directories* that it
picks up the `Irit64.lib` and `IritInpt64.lib` you have just built, not an older
copy from elsewhere.

Back up `Irit64.dll` and `GuIrit64.exe` before step 6. The setup script
overwrites them with no prompt.

### 6.8 Test in the console before touching GuIrit

`irit64.exe` rebuilds in minutes and has no GUI in the way. Set `IRIT_PATH` to
`C:\irit\irit\windowsVC2026\ntbin64`, and **run test scripts with a timeout**:
after any error IRIT drops to its prompt, and with stdin at end-of-file it
prints `Irit> ` forever.

**NEW FILE** · `C:\irit\irit\scripts\puzbsp_test.irt` · the whole file.
**RUN** · start `irit64.exe`, then at the `Irit>` prompt type
`chdir( "c:/irit/irit/scripts" ); include( "puzbsp_test.irt" );` — the same
`chdir` and `include` that GuIrit's own `Scripts\test.irt` uses.

```
# puzbsp_test.irt
PrimType = iritstate( "PrimType", 0 );          # polygonal primitives

Cube = box( vector( 0, 0, 0 ), 1, 1, 1 );
P = PUZBSP( Cube, 8, 96, 2 );

Total = 0;
for ( i = 1, 1, sizeof( P ),
    Total = Total + volume( nth( P, i ) ) );

printf( "pieces %d   sum of volumes %f   cube %f\\n",
        list( sizeof( P ), Total, volume( Cube ) ) );
```

Every construct here — `iritstate( "PrimType", 0 )`, `box( vector, dx, dy, dz )`,
the four-argument `for`, `sizeof`, `nth`, `volume`, and `printf` with a list —
is taken from `GuIritinit.irt`, the shipped scripts, or the interpreter table
(`BOX` at `inptevl0.c:111`, `VOLUME` at `:80`). The `printf` form with a
`list(...)` of several values is the one assumption; if it complains, print one
value at a time.

### 6.9 Document it

`C:\irit\irit\docs\irit.src` holds the user manual. Add `PUZBSP` to the
alphabetical function index — `TREGION` is listed at line 1157 — and write its
entry by mirroring the `TREGION` entry near line 1661. That is what IRIT's
`help` shows.

---

## 7. Route C — pure `.irt` script

For prototyping with no compiler. Every built-in used here exists in the
interpreter table; the **algorithm as a whole is an untested sketch**.

**What it gives up.** No voxels, so no prefix sum and no flood fill: material is
measured by a Boolean and `VOLUME` per candidate, and **the connectivity
guarantee is lost** — a piece can come out as two lumps. It is also slow; plan
on small piece counts.

The one primitive it needs:

**NEW FILE** · `C:\irit\irit\scripts\bsp_puzzle.irt` · the start of the file

```
# Material inside an axis-aligned box: Boolean AND, then volume.
MaterialIn = function( Model, Lo, Hi ):
    return = volume( Model * box( Lo, coord( Hi, 0 ) - coord( Lo, 0 ),
                                      coord( Hi, 1 ) - coord( Lo, 1 ),
                                      coord( Hi, 2 ) - coord( Lo, 2 ) ) );
```

`*` between two polygonal solids is Boolean intersection — `puz_vol.irt` relies
on it to cut its joints. `coord( V, i )` is used the same way in the shipped
scripts.

The loop, in outline — the same three decisions as the C core:

**REFERENCE** · an outline, not code — write the real loop below `MaterialIn` in
the same `bsp_puzzle.irt`

```
# 1. leaf  = the cell with the most material        (MaterialIn on each)
# 2. for each axis, longest first:
#        for k = 1..K candidate planes:
#            left  = MaterialIn( Model, Lo, cut )
#            right = cellMaterial - left
#            cost  = abs( left - right ) / cellMaterial
#        keep the cheapest plane leaving material on both sides
# 3. replace the leaf by its two halves; repeat until N leaves
```

IRIT lists are rebuilt rather than edited in place: produce the new leaf list
with `snoc` into a fresh list inside a `for` loop.

Traps specific to scripts (from running `.irt` files on this machine):

- **Built-in names cannot be locals.** `box`, `cone`, `circle` and `p1` in a
  function's local list is a parse error that silently discards the whole
  function; the failure surfaces much later as *undefined object*.
- **An empty Boolean may raise an error rather than return a zero-volume
  object.** A cell with no material in it is exactly that case — guard it.
- Cost per cut is 3 × K Booleans. With K = 8 and 8 pieces that is 168
  Booleans on the full model, before any trimming.

For the sub-trivariates, `TREGION( TV, Dir, T0, T1 )` takes a direction
constant. Check the constant names for trivariates in the `TREGION` manual entry
— the surface functions use `row` and `col`, and a trivariate needs a third.

---

## 8. Route D — exchange files (works today)

No code in IRIT at all. The Qt app already divides, trims and writes IRIT's
native format with one named object per piece.

**RUN** · in a terminal, in the folder that holds `QtQuickApplication1.exe`

```
QtQuickApplication1.exe --cage model.stl 8 7 --save pieces.itd
```

In GuIrit, load `pieces.itd` with the *Load* button. The file holds a list
object whose children are `piece_000`, `piece_001`, … (`PieceExport.cpp`), each
selectable in GuIrit's object tree.

- **Do not add `--spread`** for this. It lays pieces out on a print bed and
  destroys their assembled positions.
- **Do not use `--split`** unless you want one file per piece.

This is also the reference output for §10: whatever Routes A and B produce
should match it.

---

## 9. Pitfalls already paid for

Each of these cost real time in the Qt app. They carry straight over.

| # | Pitfall | Where it bites in IRIT | What to do |
|---|---|---|---|
| 1 | **IRIT's Booleans want inward-facing polygon planes.** Outward-wound polygons make AND compute a union — measured at 704% and 887% of the model volume, while reporting success. | Any polygon object **you build yourself**. | Prefer IRIT's own primitives and IRIT-loaded models, which you then never rebuild. Volume-check every Boolean regardless. |
| 2 | **Booleans modify their operands.** | Reusing one model object across pieces. | Copy the model per Boolean. |
| 3 | **Coplanar faces.** On a cube split into 8, two Booleans fail: cell faces coincide with the model's own. | Any cell face lying on the model's bounding box. | Push those faces outward by ε. It is outside the model, so `model ∩ cell` is unchanged — a cheap fix the Qt app does not yet apply. |
| 4 | **Open meshes.** The teapot is four interpenetrating open shells; parity fill hollows the joints and Booleans return garbage without erroring. | Any non-watertight input. | Reject non-closed input up front with a clear message. |
| 5 | **Voxel centre on a mesh edge.** A UV sphere splits in two. | The port. | Keep `MaterialField.cpp:72` verbatim. |
| 6 | **Circular vertex lists.** | The triangle walk. | Stop on returning to the first vertex as well as on `NULL` (§4.3). |
| 7 | **`IritGeomConvertPolysToTriangles` access-violates** on valid inputs. | Triangulating non-convex polygons. | Port the app's ear clipper. |
| 8 | **Asserts inside IRIT open a modal dialog** in a GUI build before `abort()`. | Any Boolean on an unsupported configuration, inside GuIrit. | Validate inputs before calling in; an assert cannot be caught from the plugin. |
| 9 | **Table order.** | Route B. | Append enum entry and row at the end (§6.5). |
| 10 | **Setup overwrites installed files.** | Editing `GuIritinit.irt` or replacing DLLs in `ntbin64`. | Edit under `Src\RunTime\GuIritData`; back up before running setup. |
| 11 | **Debug/Release mixing.** | Plugin deployment. | `_64.dll` ↔ `GuIrit64.exe`; `_D64.dll` ↔ `GuIritD64.exe`. |

---

## 10. Verification protocol

Run these in order. Each assumes the previous one passed.

| Step | Model | Pass condition | What a failure means |
|---|---|---|---|
| 1 | cube, 8 pieces, output 0 (cells) | 8 cells; they tile the cube | the core port is wrong |
| 2 | same, output 2 (trimmed) | Σ piece volume = cube volume (±0.1%) | winding, or coplanar faces (pitfall 3) |
| 3 | UV sphere, 6 pieces | 6 pieces, each one lump; Σ volume ≈ 100% | the voxel-centre offset was not ported |
| 4 | any model — C port vs Qt app, same N and resolution | cut positions identical to printed precision (`BSP_LOG=1` in the app prints them) | the port diverged: the material path does not use the RNG (§4.2), so there is no legitimate source of difference |
| 5 | armadillo, 6 pieces | exactly 6, none multi-part | connectivity gate or leaf choice |
| 6 | the same in GuIrit, via the panel | matches step 5 | plugin glue, not the algorithm |

Only after step 6 does a GuIrit screenshot mean anything.

---

## 11. Every file touched

### Route A — GuIrit panel

| File | Action |
|---|---|
| `C:\irit\extra\guirit\Src\GuIritDllBspPuzzle\GuIritDllBspPuzzle.cpp` | **create** — table, local-data class, callback, registration |
| `…\GuIritDllBspPuzzle\GuIritDllBspPuzzle.def` | **create** — `EXPORTS _IrtMdlrDllRegister` |
| `…\GuIritDllBspPuzzle\PuzBspCore.c` / `PuzBspCore.h` | **create** — the shared algorithm (moves to `ext_lib` on Route B) |
| `…\GuIritDllBspPuzzle\Icons\IconBspPuzzle.xpm` | **create** — copy an existing icon and redraw it |
| `…\Src\WindowsVC2026\GuIritDllBspPuzzle\GuIritDllBspPuzzle.vcxproj` | **create** — a copy of the Puzzles project, with a new GUID |
| `…\Src\WindowsVC2026\GuIrit.sln` | **edit** — add the project |
| `…\Src\RunTime\GuIritData\GuIritinit.irt` | **edit**, optional — append a `GuIrit_BspPzl` wrapper, after Route B |

### Route B — IRIT built-in

| File | Action |
|---|---|
| `C:\irit\irit\ext_lib\puzbsp.c` | **create** — the interpreter-facing function |
| `C:\irit\irit\ext_lib\PuzBspCore.c` | **move here** from the plugin folder — the shared algorithm |
| `C:\irit\irit\inc_irit\ext_lib.h` | **edit** — declare `IritExtPuzBsp` |
| `C:\irit\irit\windowsVC2026\ext_lib\ext_lib.vcxproj` | **edit** — compile the new files |
| `C:\irit\irit\ext_lib\makefile.wnt`, `Makefile.am` | **edit** — same, for nmake and Unix |
| `C:\irit\irit\irit\inptprsl.h` | **edit** — add `IP_PUZBSP`, last in `ObjValueFuncType` |
| `C:\irit\irit\irit\inptevl0.c` | **edit** — add the `PUZBSP` row, last in `ObjFuncTable` |
| `C:\irit\irit\dll_defs\irit_all_dll_defs.txt` | **edit** — export `IritExtPuzBsp`, then run `make_defs.cmd` |
| `C:\irit\irit\docs\irit.src` | **edit** — index entry and manual entry |
| *(no file edit)* | **rebuild** `IritExt64.lib`, `Irit64.dll`, `iritInpt64.lib`, `irit64.exe`, `GuIrit64.exe` |

### Reference files worth keeping open while you work

| File | Why |
|---|---|
| `C:\irit\irit\ext_lib\README.1st` | Elber's own four-step guide to adding an interpreter function |
| `C:\irit\irit\ext_lib\explfunc.c` | the smallest working ext_lib function |
| `C:\irit\irit\irit\freefrm4.c` | `RegionFromTrivarObject` and `DivideTrivarObject` — trivariate in, object or list out |
| `C:\irit\extra\guirit\Src\GuIritDllExample\GuIritDllExample.cpp` | every dialog widget type, one example each |
| `C:\irit\extra\guirit\Src\GuIritDllPuzzles\GuIritDllPuzzles.cpp` | the closest relative — polygon in, puzzle pieces out, Booleans on OK |
| `C:\irit\extra\guirit\Src\Docs\GuIritSharedExtensionsInterface.doc` | the official list of every `GuIritMdlrDll*` call, by category |
| `C:\irit\irit\scripts\puz_vol.irt` | Elber's volumetric puzzle script, to call `PUZBSP` from |
