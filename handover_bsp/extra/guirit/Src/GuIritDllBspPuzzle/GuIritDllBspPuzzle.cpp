#include "IrtDspBasicDefs.h"
#include "IrtMdlr.h"
#include "IrtMdlrFunc.h"
#include "IrtMdlrDll.h"
#include "GuIritDllExtensions.h"
#include "ext_lib/PuzBspCore.h"

#include "Icons/IconBspPuzzle.xpm"

static void IrtMdlrBspPuzzle(IrtMdlrFuncInfoClass *FI);

IRT_DSP_STATIC_DATA IrtMdlrFuncTableStruct IrtMdlrBspPuzzleFuncTable[] =
{
    { 0,
      "BspPzl",
      IconBspPuzzle,
      "IRT_MDLR_PUZZLE_BSP",
      "BspPzl",
      "BSP Volumetric Puzzle",
      "Divides a watertight polygonal solid into exactly N pieces.\n"
      "Cuts are chosen by a cost function against the material inside\n"
      "each cell, and a cut is only accepted if both halves stay one lump.",
      IrtMdlrBspPuzzle, NULL,
      IRT_MDLR_PARAM_HIDE_GEOM_PARAM_DFLT_ON |
          IRT_MDLR_PARAM_INTERMEDIATE_UPDATE_DFLT_ON,
      IRT_MDLR_OLST_EXPR,
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
            ParamVals[1] = (void *) &Object;
            ParamVals[2] = (void *) &NumPieces;
            ParamVals[3] = (void *) &VoxelRes;
            ParamVals[4] = (void *) &Output;
            ParamVals[5] = (void *) &Spacing;
        }

        IrtMdlrObjectExprClass Object;
        unsigned int NumPieces;
        unsigned int VoxelRes;
        IrtMdlrSelectExprClass Output;
        IrtRType Spacing;
};

static void IrtMdlrBspPuzzle(IrtMdlrFuncInfoClass *FI)
{
    IRT_MDLR_DLL_LCL_DATA_INIT(FI, IrtMdlrBspPuzzleLclClass, TRUE);

    if (FI -> CnstrctState == IRT_MDLR_CNSTRCT_STATE_CANCEL)
        return;

    if (FI -> InvocationNumber == 0) {
        GuIritMdlrDllSetIntInputDomain(FI, 1, 512, 2);
        GuIritMdlrDllSetIntInputDomain(FI, 8, 256, 3);
        GuIritMdlrDllSetRealInputDomain(FI, 0.0, IRIT_INFNTY, 5);
    }

    const IritPrsrObjectStruct
        *Model = LclData -> Object.GetIPObj();

    if (Model == NULL)
        return;

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

        /* Pieces made, failures, and the volume check. */
        GuIritMdlrDllPrintf(FI, IRT_DSP_LOG_INFO, "BspPzl: %s",
                            PuzBspLastReport());
    }
    else {
        for (int i = 0; i < n; i++)
            GuIritMdlrDllAddTempDisplayObject(FI,
                                              PuzBspCellBox(&Cells[i], ModelMin),
                                              FALSE, 3,
                                              IRT_DSP_GEOM_HIGHLIGHT3, FALSE);
    }

    IritFree(Cells);
}

extern "C" bool _IrtMdlrDllRegister(void)
{
    GuIritMdlrDllRegister(IrtMdlrBspPuzzleFuncTable,
                          sizeof(IrtMdlrBspPuzzleFuncTable) /
                              sizeof(IrtMdlrFuncTableStruct),
                          "BSP Puzzles",
                          IconBspPuzzle);
    return true;
}
