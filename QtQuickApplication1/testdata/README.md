# testdata

Small fixtures for the Increment 2 loader. The four failure cases matter as
much as the three good ones: a GUI that dies on a bad file is worse than one
that refuses it.

| file              | exercises                                              |
|-------------------|--------------------------------------------------------|
| `cube_ascii.stl`  | ASCII STL; vertex welding (36 raw verts → 8)           |
| `cube_binary.stl` | binary STL whose 80-byte header *starts with* `solid`, so the text sniff would get it wrong |
| `cube_quads.obj`  | OBJ with quad faces → ear clipping                     |
| `garbage.stl`     | valid extension, junk content                          |
| `empty.obj`       | zero bytes                                             |
| `part.stp`        | STEP — recognised, refused with a reason               |
| `mystery.xyz`     | unknown extension                                      |

All three good files are the same 10 × 10 × 5 box, so `--probe` output should
read `v=8 t=12 e=18` for each.

## Running

```sh
cd x64/Debug
./QtQuickApplication1.exe --probe ../../testdata/*
./QtQuickApplication1.exe --probe C:/irit/irit/data/*.itd     # 63 of 66 load
./QtQuickApplication1.exe --render C:/irit/irit/data/ih_tpot.itd out.png
```

`--probe` exits with the number of files that failed, so it drops straight into
a build script.

The three IRIT samples that do not load are correct refusals, not bugs:
`ir_l3ort.itd` holds only points, and `sphere8/16.itd` hold scalar (E1)
trivariates — volumetric functions with no boundary surface to draw.

## Larger fixture

The 160k-triangle torus used for timing is not checked in (8 MB). To regenerate:

```python
import struct, math
R, r, N, M = 30.0, 10.0, 400, 200
P = lambda i, j: ((R + r*math.cos(2*math.pi*j/M)) * math.cos(2*math.pi*i/N),
                  (R + r*math.cos(2*math.pi*j/M)) * math.sin(2*math.pi*i/N),
                  r * math.sin(2*math.pi*j/M))
tris = []
for i in range(N):
    for j in range(M):
        a, b, c, d = P(i,j), P(i+1,j), P(i+1,j+1), P(i,j+1)
        tris += [(a,b,c), (a,c,d)]
with open("torus_big.stl", "wb") as f:
    f.write(b"\0"*80 + struct.pack("<I", len(tris)))
    for t in tris:
        f.write(struct.pack("<3f", 0, 0, 0))
        for p in t:
            f.write(struct.pack("<3f", *p))
        f.write(struct.pack("<H", 0))
```


## Division modes (headless)

Two dividers, because they answer different questions.

**`--divide` — Elber's method.** Divides a *trivariate's* parameter domain and
region-extracts each cell, so every piece is a genuine solid sub-trivariate.
Needs a V-rep; an STL has none.

```sh
./QtQuickApplication1.exe --divide Torus uniform 4 4 1
./QtQuickApplication1.exe --divide Torus fit 0.8 0.8 0.8 16      # non-uniform
./QtQuickApplication1.exe --divide Sphere jitter 2 2 2 30 7
./QtQuickApplication1.exe --divide C:/irit/irit/data/fltrtest.itd uniform 2 2 2
./QtQuickApplication1.exe --divide cage:model.stl uniform 3 3 2  # box, see below
```

**`--meshdivide` — clips the model itself.** Works on any loaded mesh and the
pieces keep its real shape. This is what to use on an STL of a torus or a
lattice.

```sh
./QtQuickApplication1.exe --meshdivide model.stl uniform  3 3 2
./QtQuickApplication1.exe --meshdivide model.stl jitter   3 3 2 30 7
./QtQuickApplication1.exe --meshdivide model.stl balanced 3 3 2
./QtQuickApplication1.exe --meshdivide model.stl fit 60 60 60 16
```

Add `--png out.png` to either for an exploded render.

`VOL` in the `--meshdivide` output compares the summed piece volume against the
model's. It only proves anything when every piece is closed — the divergence
integral over an open shell is not a volume, so a run with open pieces can hit
100% by accident and the line says so.

## Known limits

- `cage:` / "Bounding cage" builds a **box** over the model's extent. It has the
  size but not the shape, so a torus comes out cuboid. It is a placeholder for
  the Increment 3 trivariate fit, not a way to divide a model.
- `--meshdivide` caps a cut face when its outline is a single simple loop.
  Where the outline is not simple — a hollow wall, a lattice strut with an
  interior void — the face is left open and the run says how many pieces are
  still shells. Nothing is ever filled speculatively.
- A cell buried entirely inside solid material contains no surface triangles;
  it is detected by ray casting and emitted as a full box.
