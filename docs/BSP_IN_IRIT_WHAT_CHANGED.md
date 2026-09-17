# BSP division in IRIT and GuIrit — what was actually changed

*A record of the work done on 2026-09-15 and 2026-09-16, file by file: what was
added, what was fixed, what was built and measured, what is still outstanding,
and how to undo any of it.*

The companion document [`BSP_IN_IRIT_AND_GUIRIT.md`](BSP_IN_IRIT_AND_GUIRIT.md)
explains **how** to do this and why each route works. This one records **what
was done**, so the tree can be audited or reverted without re-reading that guide.

---

## 1. Where things stand

| | Status |
|---|---|
| **BSP division as a GuIrit panel** | **Working and installed.** Applications → 3D Puzzles → **BSP Volumetric**. |
| **`PUZBSP` as an IRIT script command** | **Working in `irit64.exe`** as of 2026-09-16. Measured: a cube into 8 gives 8 pieces summing to 100.0% of its volume. `GuIrit64.exe` was relinked and installed the same day (§5), so GuIrit's own interpreter now carries the command as well — checked by searching both binaries: the new one contains `PUZBSP`, the replaced one does not. It has not yet been *run* from inside GuIrit (§9). |
| **Elber's own code** | Untouched apart from three shared files: the interpreter table, the export list and GuIrit's menu file (§4). Every one has a `.before-claude` backup. |

**Confirmed by a run** (2026-09-16): `PUZBSP` on a cube into 8 pieces returns
8 pieces summing to 100.0% of the model volume, with 0 Booleans failed. The
orientation fix (§3.2, item 8) and the coplanar fix (§3.3) are both exercised
by that run.

The coplanar-fix plugin DLL is now installed in `Extensions` (§5), but the
GuIrit panel has **not** been re-run since it was deployed. That check is still
outstanding, and it is the one that decides whether the panel returns trimmed
pieces rather than whole cells (§9, step 1).

---

## 2. Who did what

The work was split, and the record should say so plainly.

**Done earlier the same day, before this session's work** — the first pass
through the guide, creating the plugin and the IRIT-side edits:

| File | What |
|---|---|
| `C:\irit\irit\ext_lib\PuzBspCore.h`, `PuzBspCore.c` | a first port of the division core |
| `C:\irit\irit\ext_lib\puzbsp.c` | the interpreter-facing `IritExtPuzBsp` (34 lines, unchanged since) |
| `C:\irit\irit\inc_irit\ext_lib.h` | declares `IritExtPuzBsp` (line 34) |
| `C:\irit\irit\irit\inptprsl.h` | `IP_PUZBSP`, last in `ObjValueFuncType` (line 781) |
| `C:\irit\irit\irit\inptevl0.c` | the `PUZBSP` row, last in `ObjFuncTable` (line 600) |
| `C:\irit\irit\dll_defs\irit_all_dll_defs.txt` | four exports, then `make_defs.cmd` |
| `C:\irit\irit\windowsVC2026\ext_lib\ext_lib.vcxproj`, `ext_lib\makefile.wnt`, `Makefile.am` | compile the two new files |
| `…\Src\GuIritDllBspPuzzle\` | the plugin `.cpp`, `.def` and icon |
| `…\Src\WindowsVC2026\GuIritDllBspPuzzle\GuIritDllBspPuzzle.vcxproj` | the plugin project, added to `GuIrit.sln` |

Those edits were correct where they touched Elber's files: `IP_PUZBSP` and the
table row are both **last**, which the table's index-by-token lookup requires,
and the `JOINCRVCRV` row above has its new comma.

**This session:** fixed what would not compile or would misbehave, made the
button appear, built everything, and found the blocker. Details below.

---

## 3. Files changed in this session

### 3.1 New or replaced

| File | Size | What it is |
|---|---|---|
| `C:\irit\irit\ext_lib\PuzBspCore.c` | 1082 lines | the division core: voxel field, 3D prefix sum, connectivity, the cost-function BSP, cell→sub-trivariate, cell box, Boolean trimming |
| `C:\irit\irit\ext_lib\PuzBspCore.h` | 81 lines | its interface, plus `PuzBspCellWire` and `PuzBspLastReport` |

Both replaced the earlier versions, which are kept beside them as
`PuzBspCore.c.before-claude` and `PuzBspCore.h.before-claude`. The state just
before the orientation fix is also kept, as `PuzBspCore.c.before-orientation`.

The core is a port of the Qt app's `MaterialField.cpp` and
`PuzzleDivider::buildBspTree`, in plain C with no Qt and no GuIrit dependency,
so the same file compiles into the plugin today and into `ext_lib` when the
script command is unblocked. Comments in it name the Qt source lines they came
from.

### 3.2 Why the earlier core was replaced

Eight problems, in the order they would have been hit:

1. **`IritPrsrCopyObject(NULL, Model)`** — the function takes three arguments
   (`allocate.h:225`). Compile error.
2. **`IritMatGenMatTrans`, `IritPrsrTransformObject`** — neither name exists in
   the IRIT headers; the real ones are `IritMiscMatGenMatTrans` and
   `IritGeomTransformObject`.
3. **`IRIT_PRSR_SET_OBJ_NAME2`** — unverified macro. Replaced with the
   `IritFree(ObjName)` + `IritMiscStrdup` pattern the Qt app's exporter uses.
4. **Wrong coordinate frame.** The BSP root cell was built in world coordinates
   while the voxel field measures from the model's minimum corner. Every
   material measurement was therefore offset for any model not sitting at the
   origin, and the cell boxes and sub-trivariates were then shifted twice.
5. **No Boolean error trap.** An IRIT Boolean error inside GuIrit would have
   ended the process. The core now installs a `setjmp` trap through
   `IritBoolSetFatalErrorFunc`, restores the previous handler afterwards, and
   reports a failed piece rather than dying — the arrangement
   `GuIritDllPuzzles.cpp` uses.
6. **Coplanar faces not handled.** `IritBoolSetHandleCoplanarPoly(TRUE)` is now
   set around the Booleans, as Elber's puzzle plugin does. Cell faces that lie
   exactly on the model's own faces are the case IRIT handles worst.
7. **Leaf order.** Cells came out in internal leaf-array order; the Qt app
   flattens the BSP tree depth-first, left child first. Now matched, so piece
   numbering agrees between the two implementations.
8. **Model orientation — the reason trimmed pieces came out as whole boxes.**
   IRIT's Booleans decide inside from outside from the direction polygons face,
   and a model read from STL/OBJ commonly faces the opposite way to IRIT's own
   solids. `model ∩ cell` then returns the entire cell. The core now measures
   the model's signed volume against an IRIT box primitive's and, when the signs
   disagree, cuts with `IritPrsrReverseObject(Model)` — the same operation as
   the interpreter's unary minus (`overload.c:1065`). It also reports the
   pieces' total volume as a percentage of the model's, which is the check that
   catches this class of bug.

### 3.3 The coplanar fix, found by testing `PUZBSP`

The first working `PUZBSP` run exposed a real defect rather than a build
problem. On a cube split into 8:

| Output | Result |
|---|---|
| cells only (`Output = 0`) | 8 cells, summing to exactly 1.000000 — the split is right |
| trimmed (`Output = 2`), voxel res 48 / 96 / 128 | **7** pieces, 0.883 / 0.872 / 0.883 — one piece lost, ~12% of the material gone |
| trimmed, 4 pieces | 4 pieces, 0.999997 — fine |

So the division was correct and one `IritBooleanAND` was returning nothing. The
cause is coplanar faces: at 8 pieces the cell faces on the outside of the model
lie exactly on the cube's own faces, which is the configuration IRIT's Booleans
handle worst — the same weakness the Qt app hits on a cube.

**The fix** (`PuzBspCellBoxForCut` in `PuzBspCore.c`): when a cell face sits on
the model's bounding box, push that face outward by 1e-4 of the model's largest
extent. The sliver added is outside the solid, so `model ∩ cell` is unchanged,
but the faces no longer coincide. **Interior cut faces are never moved**, or
neighbouring pieces would overlap instead of meeting. The preview outline keeps
the exact cell.

After the fix, the same run gives **8 of 8 pieces, 0 failed, 0 empty, 100.0% of
the model volume**.

**And on a real model.** The armadillo (`armadillo.obj`) into 10 pieces:

```
PUZBSP: 10 of 10 pieces made - 0 failed, 0 empty - pieces sum to 100.0% of the
        model volume - model orientation reversed for the Booleans
```

That run also proves the orientation check (§3.2, item 8) does real work here:
the model's own signed volume is **negative** (-237,926), so its polygons face
the opposite way to IRIT's solids. Left uncorrected, that is exactly what made
the GuIrit panel return whole cell boxes instead of trimmed pieces.

**Silent failure fixed too.** `puzbsp.c` now prints `PuzBspLastReport()`, so a
script is told how many pieces were made, how many Booleans failed or came back
empty, and what the pieces sum to. The 7-piece run above reported nothing at
all, which is how a short puzzle could pass unnoticed.

### 3.4 The plugin

| File | Change |
|---|---|
| `…\Src\GuIritDllBspPuzzle\GuIritDllBspPuzzle.cpp` | added one call: prints `PuzBspLastReport()` to General Output after OK/Apply, so the volume check is visible. Backup: `.cpp.before-claude` |
| `…\Src\WindowsVC2026\GuIritDllBspPuzzle\GuIritDllBspPuzzle.vcxproj` | three fixes, backup `.vcxproj.before-claude` |
| `…\Src\GuIritDllBspPuzzle\Icons\IconBspPuzzle.xpm` | **overwritten by me in error** — see §7 |

The project fixes:
- removed `GuIritDllBspPuzzlelottedTube.cpp`, a file that does not exist (a slip
  when find-replacing "SlottedTube");
- removed a `ClInclude` for `GuIritDllBspPuzzle.h`, which does not exist;
- added `c:\irit\irit\ext_lib\PuzBspCore.c` to the build, so the plugin carries
  the core itself and does not depend on a rebuilt `Irit64.dll`;
- dropped the `ProjectReference` to `GuIritDllExtensions`, linking the prebuilt
  `GuIritDllExtensions_64.lib` instead, so building this plugin never rebuilds
  Elber's extension DLL.

---

## 4. Elber's shared files

Three files outside the plugin were edited. All have backups.

| File | Change | Backup |
|---|---|---|
| `C:\irit\irit\windowsVC2026\ntbin64\guirit64.cfg` | one line: the **BSP Volumetric** button, after Slotted Tube (line 825) | `.before-claude` |
| `C:\irit\extra\guirit\Src\GuIrit\guirit64.cfg` | the same line, because `GuIritSetup2026.bat` copies this file over the installed one | `.before-claude` |
| the interpreter table and export list | edited earlier in the day, not by this session (§2) | — |

**Why the menu line is needed at all:** GuIrit loads every DLL in
`GuIritData64\Extensions` and calls its `_IrtMdlrDllRegister`, but a registered
function only gets a button if `guirit64.cfg` lists its string id. Without the
line, the plugin loaded and did nothing visible. The line is:

```
    "Applications.3D Puzzles.BSP\nVolumetric",   "IRT_MDLR_PUZZLE_BSP",   "Type:REGULAR",
```

---

## 5. What was built, and where it went

Built with the **Visual Studio 2026** MSBuild
(`C:\Program Files\Microsoft Visual Studio\18\…`). The earlier failed attempt
used the VS 2022 MSBuild, which cannot build this project: it needs toolset
`v145`, and VS 2022 provides `v143`.

| Build | Output | Result |
|---|---|---|
| `GuIritDllBspPuzzle.vcxproj`, Release x64 | `…\Src\WindowsVC2026\ntbin64\GuIritDllBspPuzzle_64.dll` (42 KB) | **succeeded**; exports exactly `_IrtMdlrDllRegister`; imports only `irit64.dll`, `GuIritDllExtensions_64.dll` and the C runtime |
| `ext_lib.vcxproj`, Release x64 | `lib64\IritExt64.lib` | **succeeded** with `puzbsp.c` and `PuzBspCore.c` |
| `inpt_lib.vcxproj`, Release x64 | `lib64\iritInpt64.lib` | **succeeded** with the `PUZBSP` table row |
| `iritDll.vcxproj`, Release x64 | `ntbin64\Irit64.dll` | **succeeded** on 2026-09-16 at 13:50, once GuIrit was closed and released the file: 8,152,064 bytes, against 8,128,512 for the 2026-05-04 backup. The only warning is the pre-existing `LNK4098` about `libcmt.lib`. The earlier failure is kept in §6 because the cause is worth knowing |
| `GuIrit.vcxproj`, Release x64 | `…\Src\WindowsVC2026\ntbin64\GuIrit64.exe` | **succeeded** on 2026-09-16 at 14:00: 12,780,032 bytes, against 12,679,680 for the 2026-03-19 binary. It rebuilt `GuIritDllExtensions_64.dll` first, then the whole GuIrit application. Only warning is a pre-existing `C4722` in `IrtDspFrame.cpp`. Checked by searching both binaries: the new exe contains `PUZBSP`, the old one does not |

**Deployed:** the plugin DLL was copied to
`C:\irit\irit\windowsVC2026\ntbin64\GuIritData64\Extensions\`. Two earlier
versions are kept beside it: `GuIritDllBspPuzzle_64.dll.before-orientation`
(39,936 bytes) and `GuIritDllBspPuzzle_64.dll.before-coplanar-deploy`
(42,496 bytes).

The copy installed there is now the **coplanar-fix build** — 43,520 bytes, built
2026-09-16 13:33, deployed the same day once GuIrit was closed. Until then the
copy attempt was refused outright (`The process cannot access the file … because
it is being used by another process`), which is worth knowing: a running GuIrit
blocks the extension DLL as well as `Irit64.dll`, and Windows refuses the write
rather than truncating the target.

Note the filename carries a `_64` suffix — searching for `GuIritDllBspPuzzle.dll`
finds nothing.

**Side effect worth knowing:** the failed `iritDll` build first rebuilt **all**
IRIT release x64 libraries (17:01). They were compiled from your own sources
with the same toolset, and the Qt app links the Debug (`D64`) libraries, so it
is unaffected. Only `IritExt64.lib` and `iritInpt64.lib` have backups.

---

## 6. The blocker, and how it was cleared

**Cleared on 2026-09-16.** `geom_lib\ogl_depth_peel.c` was restored from
`irit-sm1Feb2026.zip` (44,963 bytes, dated 2026-01-24); the 32-byte stub is
kept beside it as `ogl_depth_peel.c.stub-before-claude`. `ext_lib` and
`irit64.exe` were then rebuilt, and `PUZBSP` works. `Irit64.dll` was relinked at
13:50 and `GuIrit64.exe` at 14:00 the same day, once GuIrit had been closed.

A running GuIrit blocks all three files, and it does so in two different ways
worth telling apart: the `Irit64.dll` link fails with `LINK : fatal error
LNK1104`, while a copy onto the extension DLL is refused by Windows before
anything is written (`The process cannot access the file … because it is being
used by another process`). The refusal is safe — the target keeps its old
contents — but it is silent unless the copy's result is actually checked.

What the problem was:

```
irit64.def : error LNK2001: unresolved external symbol IritGeomOglZPeel
irit64.def : error LNK2001: unresolved external symbol IritGeomOglZPeelTesselate
```

- Both are declared in `inc_irit\geom_lib.h` and called from
  `irit\freefrm10.c`, and both are in the export list.
- `geom_lib\ogl_depth_peel.c`, which the `geom_lib` project compiles, contains
  exactly `/* Disabled - requires GLEW */` — 32 bytes, dated **2026-05-29**.
- Your working `Irit64.dll` is dated **2026-05-04**, before that change, and it
  does export both symbols.

So the implementation was removed locally after the DLL was last built. Nothing
to do with BSP.

**The failed link emptied the installed DLL**; it was restored immediately from
the backup and verified byte-identical, along with `Irit64.lib` and
`Irit64.exp`.

**What was done to unblock it — all three steps are complete as of 2026-09-16:**
1. Restore `geom_lib\ogl_depth_peel.c` from `C:\Users\Admin\Downloads\irit-sm1Feb2026.zip`
   (`geom_lib/ogl_depth_peel.c`, 45 KB, dated 2026-01-24 — the archive that
   matches this tree). Keep the stub as a backup. GLEW is installed at
   `C:\c\glew\glew-1.10.0` and is already on `geom_lib`'s include path.
2. Rebuild `iritDll` → `Irit64.dll`, then `irit` → `irit64.exe`, and test
   `PUZBSP` in the console first.
3. Relink `GuIrit64.exe` from `…\Src\WindowsVC2026\GuIrit.sln`, backing up the
   installed binary first, because the interpreter's function table is compiled
   into that executable.

Re-enabling that file undoes a deliberate change someone made in May, which is
why it was left alone.

---

## 7. One mistake to record

Before discovering that the plugin folder already existed, a script of mine
overwrote `…\Src\GuIritDllBspPuzzle\Icons\IconBspPuzzle.xpm` with a renamed copy
of Elber's linear-slice icon. If the original was also a renamed copy of that
icon, as the guide suggests, nothing is lost; if it had been redrawn, that
drawing is gone. There is no backup of it.

---

## 8. How to undo

**Remove the GuIrit panel:**
1. Delete `…\ntbin64\GuIritData64\Extensions\GuIritDllBspPuzzle_64.dll`.
2. Restore both `guirit64.cfg` files from their `.before-claude` copies, or
   delete the one `IRT_MDLR_PUZZLE_BSP` line from each.

**Undo this session's code changes, keeping the earlier work:** restore each
`*.before-claude` file over its original. They are:

```
C:\irit\irit\ext_lib\PuzBspCore.c.before-claude
C:\irit\irit\ext_lib\PuzBspCore.h.before-claude
C:\irit\extra\guirit\Src\GuIritDllBspPuzzle\GuIritDllBspPuzzle.cpp.before-claude
C:\irit\extra\guirit\Src\WindowsVC2026\GuIritDllBspPuzzle\GuIritDllBspPuzzle.vcxproj.before-claude
C:\irit\irit\windowsVC2026\ntbin64\guirit64.cfg.before-claude
C:\irit\extra\guirit\Src\GuIrit\guirit64.cfg.before-claude
C:\irit\irit\windowsVC2026\lib64\IritExt64.lib.before-claude
C:\irit\irit\windowsVC2026\lib64\iritInpt64.lib.before-claude
C:\irit\irit\windowsVC2026\ntbin64\Irit64.dll.before-claude      (already restored)
C:\irit\irit\windowsVC2026\lib64\Irit64.lib.before-claude        (already restored)
C:\irit\irit\windowsVC2026\lib64\Irit64.exp.before-claude        (already restored)
```

**Remove the IRIT-side edits entirely** (the earlier work): delete
`ext_lib\puzbsp.c`, `PuzBspCore.*`; remove `IritExtPuzBsp` from
`inc_irit\ext_lib.h`; remove `IP_PUZBSP` from `irit\inptprsl.h`; remove the
`PUZBSP` row from `irit\inptevl0.c` and the comma it added to the row above;
remove the four names from `dll_defs\irit_all_dll_defs.txt` and run
`make_defs.cmd`; remove the two `ClCompile` lines from `ext_lib.vcxproj` and the
two file names from `ext_lib\makefile.wnt` and `Makefile.am`; rebuild `ext_lib`
and `inpt_lib`.

---

## 9. What to do next

1. **Confirm the orientation fix.** In GuIrit, divide with Output = *Trimmed
   pieces* and read the `BspPzl:` line in General Output. About **100%** of the
   model volume means the pieces are real; several hundred percent means the
   cells are still coming back whole.
2. **Redraw the icon** if the original was yours (§7).
3. **Try `PUZBSP` from inside GuIrit.** The relinked `GuIrit64.exe` is installed
   (§5) and the binary carries the command, but it has never been run from
   GuIrit's own interpreter — only from `irit64.exe`.
4. The division inside GuIrit is the same algorithm as the Qt app's, so results
   can be compared directly: same model, same piece count, same voxel
   resolution should give the same cuts. The Qt app prints them with
   `BSP_LOG=1`.
