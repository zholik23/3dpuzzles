# IRIT to GCode - 3D Puzzle & Iso-Slicer

This project is an advanced 3D processing application that combines two major capabilities:
1. **3D Puzzle Divider**: Divides a 3D solid into printable puzzle pieces, analyzes their assemblability, and adds pin/hole joints.
2. **Iso-Parametric GCode Generator**: Generates conformal GCode for 3D printing by slicing along the parametric iso-curves of the model surfaces, rather than traditional planar layers.

## 📂 File Architecture ("What files for what")

### Core Application & UI
- **`main.cpp`**: Main entry point. Handles CLI flags (`--divide`, `--meshdivide`, `--joints`, `--plan`, `--probe`) and boots the QML engine.
- **`AppController.h` / `.cpp`**: The primary C++ controller exposed to QML. Manages the loaded mesh, triggers division logic, and holds application state.
- **`main.qml`**: The Qt Quick UI layout.
- **`MeshView.h` / `.cpp`**: Custom QQuickItem that provides 3D OpenGL visualization of the models and separated puzzle pieces.

### Geometry Loading & Representation
- **`CadLoader.h` / `.cpp`**: Handles file importing (STL, OBJ, ITD, IGS). Dispatches to IRIT parser and extracts geometry.
- **`MeshData.h` / `.cpp`**: A lightweight, flat data structure representing meshes (triangles, vertices, bounding boxes). Used as the standard currency across the app instead of IRIT-specific structs.
- **`IritMesh.h` / `.cpp`**: Converts IRIT geometry (freeform surfaces, trivariates) into `MeshData` through tessellation and ear-clipping.
- **`Trivariate.h` / `.cpp`**: Represents IRIT trivariate volumetric splines. 

### Division & Puzzle Logic
- **`MeshDivider.h` / `.cpp`**: Divides polygon meshes using physical clipping planes. Supports BSP recursive splitting, uniform slicing, and jittered cuts.
- **`PuzzleDivider.h` / `.cpp`**: Divides IRIT trivariates. Instead of physical planes, this splits the *parameter domain* (U, V, W) and extracts sub-regions, creating true solid sub-trivariates.
- **`CutWarp.h` / `.cpp`**: Allows cuts to be warped (wave/grid shapes) instead of flat planes.

### Assembly Planning & Joints
- **`AssemblyPlanner.h` / `Planner*.cpp`**: The algorithms for checking assemblability. Builds a contact graph of the pieces, tests for "translational blocking" (if a piece is physically trapped), and finds a valid assembly order.
- **`IritJoint.h` / `.cpp`**: Places Elber-style interlocking pin-and-hole joints on the shared faces of adjacent puzzle pieces using IRIT boolean operations.
- **`PlannerJoints.cpp`**: Assigns joints specifically along the validated assembly/disassembly path so the puzzle doesn't get locked permanently.

### GCode & Slicing (`demo test files/` & `IsoGcodeGenerator.cpp`)
- **`IsoGcodeGenerator.cpp`**: The Iso-parametric slicer. Instead of slicing horizontally, it tracks the UV coordinates of a surface and generates toolpaths that conform to the shape. It features:
  - **Hausdorff-validated Kåsa–Späth Arc Fitting**: Fits G2/G3 arcs to 3D curves to reduce GCode size and improve machine motion.
  - **Multi-wall Iso-shells**: Expands single tracks into multiple parallel tracks at specified parameter offsets.
  - **Clipper2 Integration**: Used for handling infill and planar support structures where required.
- **`micro_support.c`**: Implements advanced 3D-printing support generation using microstructures (authored by Gershon Elber).
- **`wall_thicknesses.c`**: Computes local wall thicknesses of the model to ensure physical viability (authored by Youngjin Park).

---

## 🧠 How the Code Works

### 1. The Pipeline
1. **Load**: A model is loaded (`CadLoader`), stored as a `Trivariate` (if parametric) or `MeshData` (if mesh).
2. **Divide**: The user chooses a subdivision strategy (e.g., Uniform, BSP, Jitter). `PuzzleDivider` or `MeshDivider` cuts the model into an array of `PuzzlePiece` objects.
3. **Plan**: `AssemblyPlanner` analyzes the bounding boxes and face adjacencies to simulate removal of pieces, proving that the puzzle can actually be taken apart.
4. **Joint**: `IritJoint` uses the contact graph to carve complementary pins and holes into the matching faces of the pieces using precise geometric booleans.
5. **Slice/Export**: The resulting geometry can either be exported, visualized in `MeshView`, or fed into the `IsoGcodeGenerator` for 3D printing.

### 2. How the Division Works (BSP, Jitter, etc.)

The core challenge of the puzzle maker is dividing a solid model into smaller, interlocking blocks. The app handles this via two separate classes depending on the model's format:
*   **`PuzzleDivider` (Parametric Domain Slicing):** Used when the model is a mathematical "Trivariate" (a true solid volume). Instead of cutting with physical 3D planes, it slices the mathematical parameter space ($U, V, W$). The sub-regions are then extracted as true mathematical solids.
*   **`MeshDivider` (Physical Plane Clipping):** Used when the model is a standard polygon mesh (like an STL). It physically clips the triangles against 3D bounding boxes. This keeps the model's real outer shape intact.

#### Division Strategies:
1. **Uniform Grid:**
   The simplest method. The model's bounding box is divided into equal-sized cubes (e.g., a $2 \times 2 \times 2$ grid). Every piece is cut by straight, globally shared planes.
2. **Jittered (Elber's Difficulty Randomization):**
   Starts with a uniform grid, but pushes the cut positions off the grid by a random "curve percentage." This guarantees no two pieces are exactly the same size. The "Jitter" breaks symmetry, making the puzzle harder to solve. Furthermore, a single "seed" drives the randomness, meaning the same seed always reproduces the exact same puzzle geometry.
3. **BSP (Binary Space Partitioning):**
   Unlike a global grid (where a cut plane goes all the way through the model), BSP is a **recursive** algorithm. 
   - It takes the whole bounding box and splits it into two smaller boxes (cells) with a single plane. 
   - Then, it picks one of those smaller cells and splits *it* in two, and so on.
   - **How it chooses:** The algorithm specifically targets cells in proportion to their **volume** and always cuts along their **longest axis** near the middle. This prevents the creation of long, thin "sliver" pieces, ensuring the pieces remain roughly cube-like and physically viable for 3D printing.
4. **Curved Cuts (`CutWarp`):**
   Instead of flat planes, the cuts themselves can be bent into smooth, irregular wavy surfaces. To do this mathematically safely, the model is warped into a wavy space, sliced with a straight plane, and then un-warped. This ensures the cut perfectly mates the two pieces without intersecting.

#### Handling Mesh Clipping Edge-Cases:
When clipping polygon meshes, if a clipping cell is completely buried *inside* the solid material, it won't contain any surface triangles. The `MeshDivider` uses ray casting to detect these entirely internal blocks so they aren't accidentally discarded as "empty" space. Additionally, open faces created by the cuts are automatically capped/closed using the mathematical winding (signed volume) of the mesh.

### 3. Isoparametric Slicing Explained
Traditional slicers intersect the model with horizontal planes, resulting in "stair-stepping" on shallow slopes. 
This project's `IsoGcodeGenerator` directly follows the math of the parametric surfaces (NURBS/Trivariates). 
- It sweeps across the U or V domain of a surface.
- For each path, it evaluates exact 3D points.
- It applies the Ramer-Douglas-Peucker (RDP) algorithm to simplify the 3D curves.
- It identifies sections that are planar and circular, fitting G-code arcs (G2/G3) via the **Kåsa–Späth** algebraic method.
- The fitted arcs are then strictly validated using a bidirectional **Hausdorff Distance** check against the original curve to ensure no deviation exceeds the tolerance.

### 3. Stability & IRIT Integration
The project relies heavily on the IRIT solid modeling kernel. Because IRIT is a C library that uses `exit()` on fatal errors, `IritGuard` wraps calls using `setjmp/longjmp`. This allows the application to gracefully catch parsing errors or boolean failures without crashing the GUI. All geometry is promptly converted from IRIT types to `MeshData` at boundaries so the app doesn't leak memory or mix paradigms.
