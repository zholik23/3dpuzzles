/******************************************************************************
* PuzBspCore.c - material-aware BSP division of a polygonal solid.           *
*******************************************************************************
* Ported from the Puzzle Divider Qt app. Comments name the source they were  *
* ported from - MaterialField.cpp and PuzzleDivider.cpp - so a divergence    *
* between the two implementations can be traced to the line it came from.   *
******************************************************************************/

#include <math.h>
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "inc_irit/irit_sm.h"
#include "inc_irit/iritprsr.h"
#include "inc_irit/allocate.h"
#include "inc_irit/misc_lib.h"
#include "inc_irit/geom_lib.h"
#include "inc_irit/bool_lib.h"
#include "inc_irit/triv_lib.h"

#include "PuzBspCore.h"

#define PUZ_BSP_CANDIDATES	24     /* Planes scored per axis per cut.   */
#define PUZ_BSP_MAX_PIECES	4096
#define PUZ_BSP_REPORT_LEN	512

static char PuzBspReport[PUZ_BSP_REPORT_LEN] = "";

/* Stage times in seconds.  Kept here because PuzBspMakePieces overwrites the
   report buffer that PuzBspBuildCells filled, so the earlier timings would be
   gone before anyone reads them.                                             */
static IrtRType
    PuzBspSecVoxel = 0.0,
    PuzBspSecCuts  = 0.0,
    PuzBspSecTrim  = 0.0;

const char *PuzBspLastReport(void)
{
    return PuzBspReport;
}

static int PuzBspClampI(int v, int Lo, int Hi)
{
    return v < Lo ? Lo : (v > Hi ? Hi : v);
}

/* ------------------------------------------------------------- polygons -- */

typedef void (*PuzBspTriFuncType)(const IrtPtType A,
				  const IrtPtType B,
				  const IrtPtType C,
				  void *Data);

/* Fan-triangulates every polygon of a polygonal object.  IRIT vertex lists
   may be circular, so the walk stops on returning to the first vertex as
   well as on NULL.  A fan is exact for convex polygons only.                */
static void PuzBspForEachTriangle(const IritPrsrObjectStruct *PObj,
				  PuzBspTriFuncType Tri,
				  void *Data)
{
    const IritPrsrPolygonStruct *Pl;

    for (Pl = PObj -> U.Pl; Pl != NULL; Pl = Pl -> Pnext) {
	const IritPrsrVertexStruct
	    *V0 = Pl -> PVertex,
	    *V;

	if (V0 == NULL || V0 -> Pnext == NULL)
	    continue;

	for (V = V0 -> Pnext;
	     V -> Pnext != NULL && V -> Pnext != V0;
	     V = V -> Pnext)
	    Tri(V0 -> Coord, V -> Coord, V -> Pnext -> Coord, Data);
    }
}

static int PuzBspBBox(const IritPrsrObjectStruct *PObj,
		      IrtRType Min[3],
		      IrtRType Max[3])
{
    const IritPrsrPolygonStruct *Pl;
    int a,
	Any = FALSE;

    for (Pl = PObj -> U.Pl; Pl != NULL; Pl = Pl -> Pnext) {
	const IritPrsrVertexStruct
	    *V0 = Pl -> PVertex,
	    *V = V0;

	if (V0 == NULL)
	    continue;
	do {
	    for (a = 0; a < 3; a++) {
		if (!Any)
		    Min[a] = Max[a] = V -> Coord[a];
		else if (V -> Coord[a] < Min[a])
		    Min[a] = V -> Coord[a];
		else if (V -> Coord[a] > Max[a])
		    Max[a] = V -> Coord[a];
	    }
	    Any = TRUE;
	    V = V -> Pnext;
	}
	while (V != NULL && V != V0);
    }
    return Any;
}

static void PuzBspVolTri(const IrtPtType A,
			 const IrtPtType B,
			 const IrtPtType C,
			 void *Data)
{
    *((IrtRType *) Data) += (A[0] * (B[1] * C[2] - B[2] * C[1]) -
			     A[1] * (B[0] * C[2] - B[2] * C[0]) +
			     A[2] * (B[0] * C[1] - B[1] * C[0])) / 6.0;
}

/* Signed volume by the divergence theorem.  Its SIGN says which way the     */
/* polygons face; only meaningful for a closed solid.                        */
static IrtRType PuzBspSignedVolume(const IritPrsrObjectStruct *PObj)
{
    IrtRType
	V = 0.0;

    if (PObj != NULL && IRIT_PRSR_IS_POLY_OBJ(PObj))
	PuzBspForEachTriangle(PObj, PuzBspVolTri, &V);
    return V;
}

/* ---------------------------------------------------------- voxel field -- */
/* MaterialField.h / .cpp.  Coordinates are LOCAL: (0, 0, 0) is the model's  */
/* minimum corner, the frame the BSP runs in.                                */

typedef struct PuzBspFieldStruct {
    int N[3];
    IrtRType Cell[3];
    unsigned char *Occ;			    /* N0 * N1 * N2 occupancy.      */
    IrtRType *Sum;		/* (N0+1)(N1+1)(N2+1) inclusive prefix sums. */
} PuzBspFieldStruct;

#define PUZ_OCC_IDX(F, i, j, k) \
    (((i) * (F) -> N[1] + (j)) * (F) -> N[2] + (k))
#define PUZ_SUM_IDX(F, i, j, k) \
    (((i) * ((F) -> N[1] + 1) + (j)) * ((F) -> N[2] + 1) + (k))

typedef struct PuzBspCrossCtxStruct {
    const PuzBspFieldStruct *F;
    IrtRType Org[3];
    int Fill;				  /* FALSE = count, TRUE = store.   */
    int *Count;				  /* Crossings per (x, y) column.   */
    int *Cursor;			  /* Write position per column.     */
    float *Z;
} PuzBspCrossCtxStruct;

/* One triangle's crossings of the voxel columns - MaterialField.cpp:42-89. */
static void PuzBspCrossTri(const IrtPtType A,
			   const IrtPtType B,
			   const IrtPtType C,
			   void *Data)
{
    PuzBspCrossCtxStruct
	*Ctx = (PuzBspCrossCtxStruct *) Data;
    const PuzBspFieldStruct
	*F = Ctx -> F;
    const int
	nx = F -> N[0],
	ny = F -> N[1];
    const double
	ax = A[0] - Ctx -> Org[0], ay = A[1] - Ctx -> Org[1],
	bx = B[0] - Ctx -> Org[0], by = B[1] - Ctx -> Org[1],
	cx = C[0] - Ctx -> Org[0], cy = C[1] - Ctx -> Org[1],
	det = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    double loX, hiX, loY, hiY;
    int i, j, i0, i1, j0, j1;

    /* Seen edge-on: crosses no column interior. */
    if (fabs(det) < 1e-12)
	return;

    loX = IRIT_MIN(ax, IRIT_MIN(bx, cx));
    hiX = IRIT_MAX(ax, IRIT_MAX(bx, cx));
    loY = IRIT_MIN(ay, IRIT_MIN(by, cy));
    hiY = IRIT_MAX(ay, IRIT_MAX(by, cy));

    i0 = IRIT_MAX(0,      (int) floor(loX / F -> Cell[0] - 0.5));
    i1 = IRIT_MIN(nx - 1, (int) ceil (hiX / F -> Cell[0] - 0.5));
    j0 = IRIT_MAX(0,      (int) floor(loY / F -> Cell[1] - 0.5));
    j1 = IRIT_MIN(ny - 1, (int) ceil (hiY / F -> Cell[1] - 0.5));

    /* MaterialField.cpp:72 - sample off the voxel centre by an irrational-  */
    /* ish fraction, or a centre on a shared triangle edge is counted twice  */
    /* or not at all, and a UV sphere's meridians split the model in two.    */
    {
	const double
	    jx = 0.5 + 1.0 / 512.0,
	    jy = 0.5 + 1.0 / 337.0;

	for (i = i0; i <= i1; i++) {
	    const double
		px = (i + jx) * F -> Cell[0];

	    for (j = j0; j <= j1; j++) {
		const double
		    py = (j + jy) * F -> Cell[1],
		    w0 = ((bx - px) * (cy - py) - (by - py) * (cx - px)) / det,
		    w1 = ((cx - px) * (ay - py) - (cy - py) * (ax - px)) / det,
		    w2 = 1.0 - w0 - w1;
		const int
		    Col = i * ny + j;

		if (w0 < 0.0 || w1 < 0.0 || w2 < 0.0)
		    continue;

		if (!Ctx -> Fill)
		    Ctx -> Count[Col]++;
		else
		    Ctx -> Z[Ctx -> Cursor[Col]++] =
			(float) (w0 * (A[2] - Ctx -> Org[2]) +
				 w1 * (B[2] - Ctx -> Org[2]) +
				 w2 * (C[2] - Ctx -> Org[2]));
	    }
	}
    }
}

static int PuzBspCmpFloat(const void *P1, const void *P2)
{
    const float
	a = *(const float *) P1,
	b = *(const float *) P2;

    return a < b ? -1 : (a > b ? 1 : 0);
}

static void PuzBspFieldFree(PuzBspFieldStruct *F)
{
    if (F -> Occ != NULL)
	IritFree(F -> Occ);
    if (F -> Sum != NULL)
	IritFree(F -> Sum);
}

/* MaterialField::build - MaterialField.cpp:9-134. */
static int PuzBspFieldBuild(const IritPrsrObjectStruct *PObj,
			    int MaxRes,
			    const IrtRType Min[3],
			    const IrtRType Ext[3],
			    PuzBspFieldStruct *F)
{
    PuzBspCrossCtxStruct Ctx;
    IrtRType Longest = 0.0, Vox;
    int a, i, j, k, c, nx, ny, nz, Cols, Total,
	*Start;

    memset(F, 0, sizeof(PuzBspFieldStruct));

    for (a = 0; a < 3; a++)
	Longest = IRIT_MAX(Longest, Ext[a]);
    if (!(Longest > 0.0))
	return FALSE;

    /* Resolution fixed on the longest axis; the others keep voxels cubic. */
    MaxRes = PuzBspClampI(MaxRes, 8, 256);
    for (a = 0; a < 3; a++) {
	F -> N[a] = PuzBspClampI((int) lround(MaxRes * Ext[a] / Longest),
				 4, MaxRes);
	F -> Cell[a] = Ext[a] / F -> N[a];
	if (!(F -> Cell[a] > 0.0))
	    return FALSE;
    }
    nx = F -> N[0];
    ny = F -> N[1];
    nz = F -> N[2];
    Cols = nx * ny;

    /* Column parity, two passes: count the crossings, then store them. */
    Ctx.F = F;
    for (a = 0; a < 3; a++)
	Ctx.Org[a] = Min[a];
    Ctx.Count = (int *) IritMalloc(sizeof(int) * Cols);
    memset(Ctx.Count, 0, sizeof(int) * Cols);
    Ctx.Cursor = NULL;
    Ctx.Z = NULL;
    Ctx.Fill = FALSE;
    PuzBspForEachTriangle(PObj, PuzBspCrossTri, &Ctx);

    Start = (int *) IritMalloc(sizeof(int) * (Cols + 1));
    Start[0] = 0;
    for (c = 0; c < Cols; c++)
	Start[c + 1] = Start[c] + Ctx.Count[c];
    Total = Start[Cols];

    Ctx.Cursor = (int *) IritMalloc(sizeof(int) * Cols);
    memcpy(Ctx.Cursor, Start, sizeof(int) * Cols);
    Ctx.Z = (float *) IritMalloc(sizeof(float) * IRIT_MAX(Total, 1));
    Ctx.Fill = TRUE;
    PuzBspForEachTriangle(PObj, PuzBspCrossTri, &Ctx);

    /* Fill between consecutive pairs - MaterialField.cpp:92-111. */
    F -> Occ = (unsigned char *) IritMalloc(nx * ny * nz);
    memset(F -> Occ, 0, nx * ny * nz);

    for (i = 0; i < nx; i++) {
	for (j = 0; j < ny; j++) {
	    const int
		Col = i * ny + j,
		m = Ctx.Count[Col];
	    float
		*zs = &Ctx.Z[Start[Col]];
	    int p, Pairs;

	    if (m < 2)
		continue;
	    qsort(zs, m, sizeof(float), PuzBspCmpFloat);

	    /* An odd count means the surface is not closed along this       */
	    /* column: drop the orphan instead of flooding the column.       */
	    Pairs = m & ~1;
	    for (p = 0; p + 1 < Pairs; p += 2) {
		const int
		    k0 = IRIT_MAX(0,      (int) ceil (zs[p]     / F -> Cell[2] - 0.5)),
		    k1 = IRIT_MIN(nz - 1, (int) floor(zs[p + 1] / F -> Cell[2] - 0.5));

		for (k = k0; k <= k1; k++)
		    F -> Occ[PUZ_OCC_IDX(F, i, j, k)] = 1;
	    }
	}
    }

    IritFree(Ctx.Count);
    IritFree(Ctx.Cursor);
    IritFree(Ctx.Z);
    IritFree(Start);

    /* Inclusive 3D prefix sum, offset by one - MaterialField.cpp:115-133. */
    Vox = F -> Cell[0] * F -> Cell[1] * F -> Cell[2];
    F -> Sum = (IrtRType *) IritMalloc(sizeof(IrtRType) *
				       (nx + 1) * (ny + 1) * (nz + 1));
    memset(F -> Sum, 0, sizeof(IrtRType) * (nx + 1) * (ny + 1) * (nz + 1));

    for (i = 1; i <= nx; i++) {
	for (j = 1; j <= ny; j++) {
	    for (k = 1; k <= nz; k++) {
		const IrtRType
		    Here = F -> Occ[PUZ_OCC_IDX(F, i - 1, j - 1, k - 1)] ? Vox : 0.0;

		F -> Sum[PUZ_SUM_IDX(F, i, j, k)] =
		      Here
		    + F -> Sum[PUZ_SUM_IDX(F, i - 1, j,     k    )]
		    + F -> Sum[PUZ_SUM_IDX(F, i,     j - 1, k    )]
		    + F -> Sum[PUZ_SUM_IDX(F, i,     j,     k - 1)]
		    - F -> Sum[PUZ_SUM_IDX(F, i - 1, j - 1, k    )]
		    - F -> Sum[PUZ_SUM_IDX(F, i - 1, j,     k - 1)]
		    - F -> Sum[PUZ_SUM_IDX(F, i,     j - 1, k - 1)]
		    + F -> Sum[PUZ_SUM_IDX(F, i - 1, j - 1, k - 1)];
	    }
	}
    }
    return TRUE;
}

static IrtRType PuzBspFieldTotal(const PuzBspFieldStruct *F)
{
    return F -> Sum[PUZ_SUM_IDX(F, F -> N[0], F -> N[1], F -> N[2])];
}

/* Voxel index range [a, b) whose centres lie inside [Lo, Hi] -           */
/* MaterialField.cpp:143.                                                  */
static int PuzBspRange(const PuzBspFieldStruct *F,
		       const IrtRType Lo[3],
		       const IrtRType Hi[3],
		       int a[3],
		       int b[3])
{
    int d;

    for (d = 0; d < 3; d++) {
	a[d] = PuzBspClampI((int) ceil(Lo[d] / F -> Cell[d] - 0.5), 0, F -> N[d]);
	b[d] = PuzBspClampI((int) floor(Hi[d] / F -> Cell[d] - 0.5) + 1, 0,
			    F -> N[d]);
	if (b[d] <= a[d])
	    return FALSE;
    }
    return TRUE;
}

/* Material in a box: eight prefix-sum lookups - MaterialField.cpp:275. */
static IrtRType PuzBspFieldVolumeIn(const PuzBspFieldStruct *F,
				    const IrtRType Lo[3],
				    const IrtRType Hi[3])
{
    int a[3], b[3];

    if (!PuzBspRange(F, Lo, Hi, a, b))
	return 0.0;

    return F -> Sum[PUZ_SUM_IDX(F, b[0], b[1], b[2])]
	 - F -> Sum[PUZ_SUM_IDX(F, a[0], b[1], b[2])]
	 - F -> Sum[PUZ_SUM_IDX(F, b[0], a[1], b[2])]
	 - F -> Sum[PUZ_SUM_IDX(F, b[0], b[1], a[2])]
	 + F -> Sum[PUZ_SUM_IDX(F, a[0], a[1], b[2])]
	 + F -> Sum[PUZ_SUM_IDX(F, a[0], b[1], a[2])]
	 + F -> Sum[PUZ_SUM_IDX(F, b[0], a[1], a[2])]
	 - F -> Sum[PUZ_SUM_IDX(F, a[0], a[1], a[2])];
}

/* Is the material in the box one 6-connected lump - MaterialField.cpp:219. */
static int PuzBspFieldIsConnected(const PuzBspFieldStruct *F,
				  const IrtRType Lo[3],
				  const IrtRType Hi[3])
{
    static const int Step[6][3] = { { 1, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 },
				    { 0, -1, 0 }, { 0, 0, 1 }, { 0, 0, -1 } };
    int a[3], b[3], nx, ny, nz, i, j, k, d,
	Total = 0,
	Seed = -1,
	Reached = 0,
	Top = 0,
	*Stack;
    unsigned char *Seen;

    if (!PuzBspRange(F, Lo, Hi, a, b))
	return TRUE;			      /* Nothing here, nothing to sever. */

    nx = b[0] - a[0];
    ny = b[1] - a[1];
    nz = b[2] - a[2];

    for (i = a[0]; i < b[0]; i++)
	for (j = a[1]; j < b[1]; j++)
	    for (k = a[2]; k < b[2]; k++)
		if (F -> Occ[PUZ_OCC_IDX(F, i, j, k)]) {
		    ++Total;
		    if (Seed < 0)
			Seed = ((i - a[0]) * ny + (j - a[1])) * nz + (k - a[2]);
		}
    if (Total <= 1)
	return TRUE;

    Seen = (unsigned char *) IritMalloc(nx * ny * nz);
    memset(Seen, 0, nx * ny * nz);
    Stack = (int *) IritMalloc(sizeof(int) * Total);

    Stack[Top++] = Seed;
    Seen[Seed] = 1;

    while (Top > 0) {
	const int
	    Idx = Stack[--Top],
	    li = Idx / (ny * nz),
	    lj = (Idx / nz) % ny,
	    lk = Idx % nz;

	++Reached;
	for (d = 0; d < 6; d++) {
	    const int
		ni = li + Step[d][0],
		nj = lj + Step[d][1],
		nk = lk + Step[d][2];
	    int NIdx;

	    if (ni < 0 || nj < 0 || nk < 0 || ni >= nx || nj >= ny || nk >= nz)
		continue;
	    NIdx = (ni * ny + nj) * nz + nk;
	    if (Seen[NIdx] ||
		!F -> Occ[PUZ_OCC_IDX(F, a[0] + ni, a[1] + nj, a[2] + nk)])
		continue;
	    Seen[NIdx] = 1;
	    Stack[Top++] = NIdx;	  /* Each voxel pushed once: <= Total. */
	}
    }

    IritFree(Seen);
    IritFree(Stack);
    return Reached == Total;
}

/* ------------------------------------------------------------------ BSP -- */
/* PuzzleDivider::buildBspTree - PuzzleDivider.cpp:630-850, material path.  */

typedef struct PuzBspNodeStruct {
    PuzBspCellStruct Box;
    int Child[2];				       /* Both -1 for a leaf. */
} PuzBspNodeStruct;

typedef struct PuzBspCandStruct {
    IrtRType F;		   /* Cut position as a fraction of the extent.     */
    IrtRType Cost;
    IrtRType Imbalance;
    int Index;			     /* Generation order, to break ties.    */
} PuzBspCandStruct;

/* The cost is minimised.  Only the balance term is live - the Qt app's      */
/* thinness and disconnection terms are stubs weighted 0, and are omitted.   */
static int PuzBspCmpCand(const void *P1, const void *P2)
{
    const PuzBspCandStruct
	*A = (const PuzBspCandStruct *) P1,
	*B = (const PuzBspCandStruct *) P2;

    if (A -> Cost < B -> Cost)
	return -1;
    if (A -> Cost > B -> Cost)
	return 1;
    return A -> Index - B -> Index;
}

int PuzBspBuildCells(const IritPrsrObjectStruct *PolyModel,
		     int NumPieces,
		     int MaxRes,
		     PuzBspCellStruct **Cells,
		     IrtRType ModelMin[3],
		     IrtRType ModelExt[3])
{
    const char
	*LogEnv = getenv("BSP_LOG");
    const int
	Verbose = LogEnv != NULL && atoi(LogEnv) != 0;
    PuzBspFieldStruct F;
    PuzBspNodeStruct *Tree;
    IrtRType Max[3], RootVol, MinSide, Degenerate;
    IrtRType TVoxel, TCuts;
    int a, i, n, Target, NumNodes, NumLeaves, Top, Filled,
	*Leaves, *Exhausted, *Stack;

    *Cells = NULL;
    PuzBspReport[0] = 0;

    /* Read-only: TRUE would reset the clock IRIT's TIME() uses. */
    TVoxel = TCuts = IritMiscCPUTime(FALSE);

    if (PolyModel == NULL || !IRIT_PRSR_IS_POLY_OBJ(PolyModel) ||
	!PuzBspBBox(PolyModel, ModelMin, Max)) {
	snprintf(PuzBspReport, PUZ_BSP_REPORT_LEN,
		 "the object has no polygons");
	return 0;
    }
    for (a = 0; a < 3; a++)
	ModelExt[a] = Max[a] - ModelMin[a];

    PuzBspSecVoxel = 0.0;
    PuzBspSecCuts  = 0.0;
    PuzBspSecTrim  = 0.0;

    if (!PuzBspFieldBuild(PolyModel, MaxRes, ModelMin, ModelExt, &F) ||
	!(PuzBspFieldTotal(&F) > 0.0)) {
	PuzBspFieldFree(&F);
	snprintf(PuzBspReport, PUZ_BSP_REPORT_LEN,
		 "could not voxelise - the object must be a closed solid "
		 "with volume");
	return 0;
    }

    PuzBspSecVoxel = IritMiscCPUTime(FALSE) - TVoxel;
    TCuts = IritMiscCPUTime(FALSE);
    Target = PuzBspClampI(NumPieces, 1, PUZ_BSP_MAX_PIECES);
    RootVol = ModelExt[0] * ModelExt[1] * ModelExt[2];

    /* Automatic floor: a fraction of an average piece's side. */
    MinSide = 0.45 * cbrt(RootVol / Target);

    /* "Nearly nothing", not a ratio - the only material threshold. */
    Degenerate = 4.0 * F.Cell[0] * F.Cell[1] * F.Cell[2];

    Tree = (PuzBspNodeStruct *) IritMalloc(sizeof(PuzBspNodeStruct) *
					   2 * Target);
    Leaves = (int *) IritMalloc(sizeof(int) * Target);
    Exhausted = (int *) IritMalloc(sizeof(int) * Target);

    for (a = 0; a < 3; a++) {
	Tree[0].Box.Lo[a] = 0.0;
	Tree[0].Box.Hi[a] = ModelExt[a];
    }
    Tree[0].Child[0] = Tree[0].Child[1] = -1;
    NumNodes = 1;
    Leaves[0] = 0;
    Exhausted[0] = FALSE;
    NumLeaves = 1;

    while (NumLeaves < Target && RootVol > 0.0) {
	IrtRType Best = 0.0;
	int Slot = -1,
	    DidSplit = FALSE;

	/* Always split the leaf holding the MOST material. */
	for (i = 0; i < NumLeaves; i++) {
	    IrtRType W;

	    if (Exhausted[i])
		continue;
	    W = PuzBspFieldVolumeIn(&F, Tree[Leaves[i]].Box.Lo,
				    Tree[Leaves[i]].Box.Hi);
	    if (W > Best) {
		Best = W;
		Slot = i;
	    }
	}
	if (Slot < 0)
	    break;				  /* Nothing left that can be cut. */

	{
	    const int
		NodeIdx = Leaves[Slot];
	    const PuzBspCellStruct
		Cell = Tree[NodeIdx].Box;
	    int Order[3] = { 0, 1, 2 },
		Pass, t, j;

	    /* Longest axis first keeps pieces blocky. */
	    for (i = 0; i < 3; i++)
		for (j = i + 1; j < 3; j++)
		    if (Cell.Hi[Order[j]] - Cell.Lo[Order[j]] >
			Cell.Hi[Order[i]] - Cell.Lo[Order[i]]) {
			const int
			    Tmp = Order[i];

			Order[i] = Order[j];
			Order[j] = Tmp;
		    }

	    /* Pass 0 demands both halves stay connected; pass 1 relaxes it, */
	    /* so the piece count stays honest.                              */
	    for (Pass = 0; Pass < 2 && !DidSplit; Pass++) {
		for (t = 0; t < 3 && !DidSplit; t++) {
		    const int
			Axis = Order[t];
		    const IrtRType
			Extent = Cell.Hi[Axis] - Cell.Lo[Axis];
		    PuzBspCandStruct Cands[PUZ_BSP_CANDIDATES];
		    IrtRType Edge, Lo, Hi, CellMat, Cut[3], RLo[3];
		    int c,
			NumCands = 0,
			Picked = -1;

		    if (Extent < 2.0 * MinSide)
			continue;
		    Edge = MinSide / Extent;
		    Lo = Edge;
		    Hi = 1.0 - Edge;
		    if (Hi <= Lo)
			continue;

		    CellMat = PuzBspFieldVolumeIn(&F, Cell.Lo, Cell.Hi);
		    if (CellMat < 2.0 * Degenerate)
			continue;

		    for (c = 0; c < PUZ_BSP_CANDIDATES; c++) {
			const IrtRType
			    Cf = Lo + (Hi - Lo) * (c + 0.5) / PUZ_BSP_CANDIDATES;
			IrtRType Left, Right;

			for (a = 0; a < 3; a++)
			    Cut[a] = Cell.Hi[a];
			Cut[Axis] = Cell.Lo[Axis] + Extent * Cf;

			Left = PuzBspFieldVolumeIn(&F, Cell.Lo, Cut);
			Right = CellMat - Left;
			if (Left <= Degenerate || Right <= Degenerate)
			    continue;	    /* Would make an empty or sliver cell. */

			Cands[NumCands].F = Cf;
			Cands[NumCands].Imbalance = fabs(Left - Right) / CellMat;
			Cands[NumCands].Cost = 1.0 * Cands[NumCands].Imbalance;
			Cands[NumCands].Index = c;
			NumCands++;
		    }
		    if (NumCands == 0) {
			if (Verbose)
			    fprintf(stderr, "SPLIT cell %d - axis %c - no candidate "
				    "leaves material on both sides\n",
				    NodeIdx, "XYZ"[Axis]);
			continue;
		    }

		    qsort(Cands, NumCands, sizeof(PuzBspCandStruct), PuzBspCmpCand);

		    if (Verbose) {
			fprintf(stderr, "SPLIT cell %d - axis %c - %d candidates\n",
				NodeIdx, "XYZ"[Axis], NumCands);
			for (c = 0; c < NumCands; c++)
			    fprintf(stderr, "        f=%.3f imb=%.3f%s",
				    Cands[c].F, Cands[c].Imbalance,
				    (c % 5 == 4 || c + 1 == NumCands) ? "\n" : "");
		    }

		    /* The cheapest candidate that keeps both halves connected. */
		    for (c = 0; c < NumCands && Picked < 0; c++) {
			if (Pass != 0) {
			    Picked = c;
			    break;
			}
			for (a = 0; a < 3; a++) {
			    Cut[a] = Cell.Hi[a];
			    RLo[a] = Cell.Lo[a];
			}
			Cut[Axis] = Cell.Lo[Axis] + Extent * Cands[c].F;
			RLo[Axis] = Cut[Axis];
			if (PuzBspFieldIsConnected(&F, Cell.Lo, Cut) &&
			    PuzBspFieldIsConnected(&F, RLo, Cell.Hi))
			    Picked = c;
		    }
		    if (Picked < 0) {
			if (Verbose)
			    fprintf(stderr, "    -> none of %d kept both halves "
				    "connected; trying another axis\n", NumCands);
			continue;
		    }

		    if (Verbose)
			fprintf(stderr, "    -> chose f=%.3f  imbalance %.3f  "
				"(rank %d of %d)\n", Cands[Picked].F,
				Cands[Picked].Imbalance, Picked + 1, NumCands);

		    {
			const IrtRType
			    At = Cell.Lo[Axis] + Extent * Cands[Picked].F;
			const int
			    Li = NumNodes;

			Tree[Li].Box = Cell;
			Tree[Li].Box.Hi[Axis] = At;
			Tree[Li].Child[0] = Tree[Li].Child[1] = -1;
			Tree[Li + 1].Box = Cell;
			Tree[Li + 1].Box.Lo[Axis] = At;
			Tree[Li + 1].Child[0] = Tree[Li + 1].Child[1] = -1;
			Tree[NodeIdx].Child[0] = Li;
			Tree[NodeIdx].Child[1] = Li + 1;
			NumNodes += 2;

			Leaves[Slot] = Li;	 /* The parent stops being a leaf. */
			Exhausted[Slot] = FALSE;
			Leaves[NumLeaves] = Li + 1;
			Exhausted[NumLeaves] = FALSE;
			NumLeaves++;
			DidSplit = TRUE;
		    }
		}
	    }

	    if (!DidSplit)
		Exhausted[Slot] = TRUE;		 /* Too small on every axis. */
	}
    }

    /* Leaves in traversal order, left child first - PuzzleDivider::leavesOf. */
    *Cells = (PuzBspCellStruct *) IritMalloc(sizeof(PuzBspCellStruct) *
					     NumLeaves);
    Stack = (int *) IritMalloc(sizeof(int) * NumNodes);
    Top = 0;
    n = 0;
    Stack[Top++] = 0;
    while (Top > 0) {
	const int
	    Nd = Stack[--Top];

	if (Tree[Nd].Child[0] < 0)
	    (*Cells)[n++] = Tree[Nd].Box;
	else {
	    Stack[Top++] = Tree[Nd].Child[1];
	    Stack[Top++] = Tree[Nd].Child[0];
	}
    }

    PuzBspSecCuts = IritMiscCPUTime(FALSE) - TCuts;

    Filled = 0;
    for (i = 0; i < F.N[0] * F.N[1] * F.N[2]; i++)
	Filled += F.Occ[i] ? 1 : 0;

    snprintf(PuzBspReport, PUZ_BSP_REPORT_LEN,
	     "%d of %d cells - voxels %d x %d x %d (%d filled) - material %.6g"
	     " - min side %.4g",
	     n, Target, F.N[0], F.N[1], F.N[2], Filled,
	     PuzBspFieldTotal(&F), MinSide);

    IritFree(Stack);
    IritFree(Tree);
    IritFree(Leaves);
    IritFree(Exhausted);
    PuzBspFieldFree(&F);
    return n;
}

/* ------------------------------------------------------- IRIT geometry -- */

TrivTVStruct *PuzBspCellToTV(const TrivTVStruct *Cage,
			     const PuzBspCellStruct *Cell,
			     const IrtRType ModelExt[3])
{
    static const TrivTVDirType
	Dir[3] = { TRIV_CONST_U_DIR, TRIV_CONST_V_DIR, TRIV_CONST_W_DIR };
    CagdRType D[6];
    const TrivTVStruct
	*Cur = Cage;
    TrivTVStruct
	*Owned = NULL;
    int a;

    if (Cage == NULL || Cell == NULL)
	return NULL;

    IritTrivTVDomain(Cage, &D[0], &D[1], &D[2], &D[3], &D[4], &D[5]);

    /* Map the local world cell into the cage's parameter domain, then cut */
    /* one direction at a time - doRegion() in PuzzleDivider.cpp.          */
    for (a = 0; a < 3; a++) {
	const IrtRType
	    Span = D[2 * a + 1] - D[2 * a],
	    Ext = ModelExt[a] > 0.0 ? ModelExt[a] : 1.0;
	const CagdRType
	    T0 = D[2 * a] + Span * (Cell -> Lo[a] / Ext),
	    T1 = D[2 * a] + Span * (Cell -> Hi[a] / Ext);
	TrivTVStruct
	    *Next = IritTrivTVRegionFromTV(Cur, T0, T1, Dir[a]);

	if (Owned != NULL)
	    IritTrivTVFree(Owned);
	if (Next == NULL)
	    return NULL;
	Owned = Next;
	Cur = Next;
    }
    return Owned;
}

/* A cell face that sits on the model's bounding box is coplanar with the
   model's own outer face, and coplanar faces are what IRIT's Booleans handle
   worst - measured on a cube split into 8, one AND returned nothing and that
   piece was lost.  Growing only those outer faces cures it: the sliver added
   lies outside the solid, so model AND cell is unchanged.

   Interior cut faces are left exactly where they are, or neighbouring pieces
   would overlap instead of meeting.                                          */
static void PuzBspGrowOuterFaces(const PuzBspCellStruct *Cell,
				 const IrtRType ModelExt[3],
				 IrtRType Lo[3],
				 IrtRType Hi[3])
{
    IrtRType
	Eps = 0.0;
    int a;

    for (a = 0; a < 3; a++)
	Eps = IRIT_MAX(Eps, ModelExt[a]);
    Eps *= 1e-4;			     /* Far below any printable size. */

    for (a = 0; a < 3; a++) {
	Lo[a] = Cell -> Lo[a] - (Cell -> Lo[a] <= 0.0 ? Eps : 0.0);
	Hi[a] = Cell -> Hi[a] + (Cell -> Hi[a] >= ModelExt[a] ? Eps : 0.0);
    }
}

static IritPrsrObjectStruct *PuzBspBoxPrim(const PuzBspCellStruct *Cell,
					   const IrtRType ModelMin[3],
					   const IrtRType ModelExt[3],
					   int Wire)
{
    IritPrsrObjectStruct *Box;
    IrtRType Lo[3], Hi[3];
    IrtVecType Pt;
    int a, OldType;

    if (Cell == NULL)
	return NULL;

    /* ModelExt NULL means "exactly the cell" - the preview wants that. */
    if (ModelExt != NULL)
	PuzBspGrowOuterFaces(Cell, ModelExt, Lo, Hi);
    else {
	for (a = 0; a < 3; a++) {
	    Lo[a] = Cell -> Lo[a];
	    Hi[a] = Cell -> Hi[a];
	}
    }

    for (a = 0; a < 3; a++)
	Pt[a] = ModelMin[a] + Lo[a];

    /* 0 = polygonal primitive (geom_lib/primitv1.c); restored afterwards. */
    OldType = IritGeomPrimSetGeneratePrimType(0);
    Box = Wire ? IritGeomPrimGenBOXWIREObject(Pt, Hi[0] - Lo[0],
						  Hi[1] - Lo[1],
						  Hi[2] - Lo[2])
	       : IritGeomPrimGenBOXObject(Pt, Hi[0] - Lo[0],
					      Hi[1] - Lo[1],
					      Hi[2] - Lo[2]);
    IritGeomPrimSetGeneratePrimType(OldType);

    if (Box != NULL && !Wire && !IRIT_PRSR_IS_POLY_OBJ(Box)) {
	IritPrsrFreeObject(Box);
	return NULL;
    }
    return Box;
}

IritPrsrObjectStruct *PuzBspCellBox(const PuzBspCellStruct *Cell,
				    const IrtRType ModelMin[3])
{
    return PuzBspBoxPrim(Cell, ModelMin, NULL, FALSE);
}

/* The Boolean operand: outer faces grown off the model's own faces. */
IritPrsrObjectStruct *PuzBspCellBoxForCut(const PuzBspCellStruct *Cell,
					  const IrtRType ModelMin[3],
					  const IrtRType ModelExt[3])
{
    return PuzBspBoxPrim(Cell, ModelMin, ModelExt, FALSE);
}

IritPrsrObjectStruct *PuzBspCellWire(const PuzBspCellStruct *Cell,
				     const IrtRType ModelMin[3])
{
    return PuzBspBoxPrim(Cell, ModelMin, NULL, TRUE);
}

/* ------------------------------------------------------------- Booleans -- */

static jmp_buf PuzBspBoolJmp;
static const char
    *PuzBspBoolMsg = NULL;

/* Bool_lib fatal errors come back here instead of ending the program, the  */
/* same arrangement as IrtMdlrPuzBoolFatalError in GuIritDllPuzzles.cpp.    */
static void PuzBspBoolFatal(BoolFatalErrorType ErrID)
{
    PuzBspBoolMsg = IritBoolDescribeError(ErrID);
    longjmp(PuzBspBoolJmp, 1);
}

/* A AND B with Bool_lib errors trapped.  *Failed is TRUE when the Boolean */
/* raised an error; the result is then NULL.                                */
static IritPrsrObjectStruct *PuzBspAnd(const IritPrsrObjectStruct *A,
				       const IritPrsrObjectStruct *B,
				       int *Failed)
{
    IritPrsrObjectStruct
	* volatile Result = NULL;

    *Failed = FALSE;
    if (setjmp(PuzBspBoolJmp) == 0)
	Result = IritBooleanAND(A, B);
    else {
	Result = NULL;
	*Failed = TRUE;
    }
    return Result;
}

IritPrsrObjectStruct *PuzBspMakePieces(const IritPrsrObjectStruct *Model,
				       const PuzBspCellStruct *Cells,
				       int n,
				       const IrtRType ModelMin[3],
				       const IrtRType ModelExt[3],
				       int Output,
				       IrtRType Explode,
				       const char *Name)
{
    IritPrsrObjectStruct
	*List = NULL;
    TrivTVStruct
	*Cage = NULL;
    BoolFatalErrorFuncType
	OldBoolErr = NULL;
    const char
	*FirstError = NULL;
    IritPrsrObjectStruct
	*Operand = NULL;		  /* Model reversed to IRIT's winding. */
    IrtRType
	ModelVol = 0.0,
	PiecesVol = 0.0;
    IrtRType TTrim = IritMiscCPUTime(FALSE);
    int Reversed = FALSE;
    IrtRType Centre[3];
    char PieceName[32];
    int i, a,
	OldCoplanar = FALSE,
	Made = 0,
	Failed = 0,
	Empty = 0;

    if (Model == NULL || Cells == NULL || n <= 0) {
	snprintf(PuzBspReport, PUZ_BSP_REPORT_LEN, "nothing to make");
	return NULL;
    }

    for (a = 0; a < 3; a++)
	Centre[a] = ModelMin[a] + 0.5 * ModelExt[a];

    if (Output == PUZ_BSP_OUTPUT_TRIVARS) {
	Cage = IritTrivNSPrimBox(ModelMin[0], ModelMin[1], ModelMin[2],
				 ModelMin[0] + ModelExt[0],
				 ModelMin[1] + ModelExt[1],
				 ModelMin[2] + ModelExt[2]);
	if (Cage == NULL) {
	    snprintf(PuzBspReport, PUZ_BSP_REPORT_LEN,
		     "could not build the bounding cage");
	    return NULL;
	}
    }
    else if (Output == PUZ_BSP_OUTPUT_TRIMMED) {
	/* As GuIritDllPuzzles does: coplanar handling on, errors trapped.  */
	OldCoplanar = IritBoolSetHandleCoplanarPoly(TRUE);
	OldBoolErr = IritBoolSetFatalErrorFunc(PuzBspBoolFatal);

	/* IRIT's Booleans tell inside from outside by which way polygons    */
	/* face.  A model read from STL/OBJ is often wound opposite to IRIT's */
	/* own solids, and AND then returns the whole cell box.  Compare with */
	/* an IRIT box primitive and reverse the model when they disagree -   */
	/* IritPrsrReverseObject is what the interpreter's unary minus does.  */
	{
	    IritPrsrObjectStruct
		*Probe = PuzBspCellBoxForCut(&Cells[0], ModelMin, ModelExt);
	    const IrtRType
		BoxVol = PuzBspSignedVolume(Probe);

	    ModelVol = PuzBspSignedVolume(Model);
	    if (Probe != NULL)
		IritPrsrFreeObject(Probe);

	    if (BoxVol != 0.0 && ModelVol != 0.0 &&
		(BoxVol > 0.0) != (ModelVol > 0.0)) {
		Operand = IritPrsrReverseObject(Model);
		Reversed = Operand != NULL;
	    }
	}
    }

    for (i = 0; i < n; i++) {
	IritPrsrObjectStruct
	    *Piece = NULL;

	if (Output == PUZ_BSP_OUTPUT_TRIVARS) {
	    TrivTVStruct
		*TV = PuzBspCellToTV(Cage, &Cells[i], ModelExt);

	    if (TV != NULL)
		Piece = IritPrsrGenTRIVARObject(TV);
	    else
		++Failed;
	}
	else {
	    IritPrsrObjectStruct
		*Box = Output == PUZ_BSP_OUTPUT_TRIMMED
		    ? PuzBspCellBoxForCut(&Cells[i], ModelMin, ModelExt)
		    : PuzBspCellBox(&Cells[i], ModelMin);

	    if (Box == NULL) {
		++Failed;
		continue;
	    }

	    if (Output == PUZ_BSP_OUTPUT_CELLS)
		Piece = Box;
	    else {
		/* A fresh copy of the model per piece: the Qt app measured  */
		/* that IRIT's Booleans modify their operands.               */
		IritPrsrObjectStruct
		    *ModelCopy = IritPrsrCopyObject(NULL,
						 Operand != NULL ? Operand : Model,
						 FALSE);
		int BoolFailed;

		Piece = PuzBspAnd(ModelCopy, Box, &BoolFailed);
		if (BoolFailed) {
		    /* After a longjmp the operands' state is unknown: leak  */
		    /* them rather than risk a double free.                  */
		    ++Failed;
		    if (FirstError == NULL)
			FirstError = PuzBspBoolMsg;
		    Piece = NULL;
		}
		else {
		    IritPrsrFreeObject(ModelCopy);
		    IritPrsrFreeObject(Box);
		    if (Piece == NULL)
			++Failed;
		    else if (IRIT_PRSR_IS_POLY_OBJ(Piece) && Piece -> U.Pl == NULL) {
			/* Succeeded with nothing in it.  Every material-aware */
			/* cell holds material, so this is worth reporting.    */
			IritPrsrFreeObject(Piece);
			Piece = NULL;
			++Empty;
		    }
		}
	    }
	}

	if (Piece == NULL)
	    continue;

	if (Output == PUZ_BSP_OUTPUT_TRIMMED)
	    PiecesVol += fabs(PuzBspSignedVolume(Piece));

	if (Explode != 0.0) {
	    IrtHmgnMatType Mat;
	    IritPrsrObjectStruct *Moved;
	    IrtRType T[3];

	    for (a = 0; a < 3; a++)
		T[a] = Explode * (ModelMin[a] +
				  0.5 * (Cells[i].Lo[a] + Cells[i].Hi[a]) -
				  Centre[a]);
	    IritMiscMatGenMatTrans(T[0], T[1], T[2], Mat);
	    Moved = IritGeomTransformObject(Piece, Mat);
	    if (Moved != NULL) {
		IritPrsrFreeObject(Piece);
		Piece = Moved;
	    }
	}

	snprintf(PieceName, sizeof(PieceName), "piece_%03d", i);
	if (Piece -> ObjName != NULL)
	    IritFree(Piece -> ObjName);
	Piece -> ObjName = IritMiscStrdup(PieceName);

	if (List == NULL)
	    List = IritPrsrGenListObject(Name != NULL ? Name : "PuzBsp",
					 Piece, NULL);
	else
	    IritPrsrListObjectAppend(List, Piece);
	++Made;
    }

    if (Output == PUZ_BSP_OUTPUT_TRIMMED) {
	IritBoolSetFatalErrorFunc(OldBoolErr);
	IritBoolSetHandleCoplanarPoly(OldCoplanar);
    }
    if (Operand != NULL)
        IritPrsrFreeObject(Operand);
    if (Cage != NULL)
	IritTrivTVFree(Cage);

    if (Output == PUZ_BSP_OUTPUT_TRIMMED && ModelVol != 0.0)
	snprintf(PuzBspReport, PUZ_BSP_REPORT_LEN,
		 "%d of %d pieces made - %d failed, %d empty - pieces sum to "
		 "%.1f%% of the model volume%s%s%s",
		 Made, n, Failed, Empty, 100.0 * PiecesVol / fabs(ModelVol),
		 Reversed ? " - model orientation reversed for the Booleans" : "",
		 FirstError != NULL ? " - first error: " : "",
		 FirstError != NULL ? FirstError : "");
    else
	snprintf(PuzBspReport, PUZ_BSP_REPORT_LEN,
		 "%d of %d pieces made - %d failed, %d empty%s%s",
		 Made, n, Failed, Empty,
		 FirstError != NULL ? " - first error: " : "",
		 FirstError != NULL ? FirstError : "");
    PuzBspSecTrim = IritMiscCPUTime(FALSE) - TTrim;
    {
	const int
	    L = (int) strlen(PuzBspReport);

	/* Appended rather than woven into the two format strings above: it
	   cannot corrupt them, and a long first-error message keeps its text. */
	snprintf(PuzBspReport + L, PUZ_BSP_REPORT_LEN - L,
		 " - %.2fs total (voxelise %.2f, cuts %.2f, trim %.2f)",
		 PuzBspSecVoxel + PuzBspSecCuts + PuzBspSecTrim,
		 PuzBspSecVoxel, PuzBspSecCuts, PuzBspSecTrim);
    }

    return List;
}
