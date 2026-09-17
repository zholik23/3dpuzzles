/******************************************************************************
* PuzBspCore.h - material-aware BSP division of a polygonal solid.           *
*******************************************************************************
* Ported from the Puzzle Divider Qt app: MaterialField (voxel field, 3D      *
* prefix sum, connectivity) and PuzzleDivider::buildBspTree (cost-function   *
* cut selection).  Plain C with no GuIrit or Qt dependency, so the same file *
* can be compiled into a GuIrit plugin or moved into IRIT's ext_lib.         *
*                                                                            *
* Deterministic: on this path the cut positions do not depend on any random  *
* numbers, so for the same model, count and resolution the cells match the   *
* Qt app's (see docs/BSP_IN_IRIT_AND_GUIRIT.md, section 10).                 *
******************************************************************************/

#ifndef PUZ_BSP_CORE_H
#define PUZ_BSP_CORE_H

#include "inc_irit/irit_sm.h"
#include "inc_irit/iritprsr.h"
#include "inc_irit/triv_lib.h"

#if defined(__cplusplus) || defined(c_plusplus)
extern "C" {
#endif

#define PUZ_BSP_OUTPUT_CELLS	0   /* Cell boxes as polygons.               */
#define PUZ_BSP_OUTPUT_TRIVARS	1   /* Sub-trivariates of a bounding cage.   */
#define PUZ_BSP_OUTPUT_TRIMMED	2   /* Pieces trimmed to the model (Elber 5).*/

typedef struct PuzBspCellStruct {
    IrtRType Lo[3], Hi[3];     /* Local box; (0, 0, 0) is the model's min.  */
} PuzBspCellStruct;

/* Splits a closed polygonal solid into exactly NumPieces axis-aligned cells.
   MaxRes is the voxel resolution of the longest axis (96 in the Qt app).
   Returns the cell count and sets *Cells (IritMalloc - the caller frees with
   IritFree), or 0 on failure.  ModelMin/ModelExt receive the model's bounding
   box, which turns local cell coordinates back into world coordinates.     */
int PuzBspBuildCells(const IritPrsrObjectStruct *PolyModel,
		     int NumPieces,
		     int MaxRes,
		     PuzBspCellStruct **Cells,
		     IrtRType ModelMin[3],
		     IrtRType ModelExt[3]);

/* The sub-trivariate of Cage covering one cell: the cell mapped into Cage's
   parameter domain, then one IritTrivTVRegionFromTV call per direction.     */
TrivTVStruct *PuzBspCellToTV(const TrivTVStruct *Cage,
			     const PuzBspCellStruct *Cell,
			     const IrtRType ModelExt[3]);

/* One cell as a closed polygonal box in world coordinates - the Boolean
   operand when trimming.                                                    */
IritPrsrObjectStruct *PuzBspCellBox(const PuzBspCellStruct *Cell,
				    const IrtRType ModelMin[3]);

/* The same box, but with any face lying on the model's bounding box pushed
   outward by a whisker.  Such a face is coplanar with the model's own outer
   face, which is what IRIT's Booleans handle worst - on a cube split into 8,
   one intersection returned nothing and that piece was lost.  The sliver added
   is outside the solid, so the trimmed piece is unchanged.  Interior cut faces
   are never moved, or neighbouring pieces would overlap.                     */
IritPrsrObjectStruct *PuzBspCellBoxForCut(const PuzBspCellStruct *Cell,
					  const IrtRType ModelMin[3],
					  const IrtRType ModelExt[3]);

/* One cell as a wireframe box in world coordinates - for previews.         */
IritPrsrObjectStruct *PuzBspCellWire(const PuzBspCellStruct *Cell,
				     const IrtRType ModelMin[3]);

/* All pieces as one list object named Name, or NULL if none was produced.
   Output is one of PUZ_BSP_OUTPUT_*.  Explode moves each piece away from the
   model's centre by that fraction of its offset; 0 keeps them assembled.
   A piece whose Boolean fails is skipped and counted, never fatal.          */
IritPrsrObjectStruct *PuzBspMakePieces(const IritPrsrObjectStruct *Model,
				       const PuzBspCellStruct *Cells,
				       int n,
				       const IrtRType ModelMin[3],
				       const IrtRType ModelExt[3],
				       int Output,
				       IrtRType Explode,
				       const char *Name);

/* A one-line summary of the last PuzBspBuildCells / PuzBspMakePieces call:
   grid size, material volume, pieces made, and how many failed and why.    */
const char *PuzBspLastReport(void);

#if defined(__cplusplus) || defined(c_plusplus)
}
#endif

#endif /* PUZ_BSP_CORE_H */
