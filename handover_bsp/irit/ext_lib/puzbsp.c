#include <stdio.h>
#include "inc_irit/irit_sm.h"
#include "inc_irit/iritprsr.h"
#include "inc_irit/allocate.h"
#include "inc_irit/ext_lib.h"
#include "PuzBspCore.h"

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
        fprintf(stderr, "PUZBSP: %s\n", PuzBspLastReport());
        fprintf(stderr, "PUZBSP: could not voxelise - is it a closed solid?\n");
        return NULL;
    }

    Result = PuzBspMakePieces(PolyObj, Cells, n, ModelMin, ModelExt,
                              (int) *ROutput, 0.0, "PuzBsp");
    /* Says how many pieces were made, how many Booleans failed or came
       back empty, and what the pieces sum to against the model.  A short
       puzzle must never be silent.                                      */
    fprintf(stderr, "PUZBSP: %s\n", PuzBspLastReport());
    IritFree(Cells);
    return Result;
}
