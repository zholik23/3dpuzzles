/******************************************************************************
* wall_thicknesses.c - Computes wall thicknesses of a given model.	      *
*******************************************************************************
* (C) Gershon Elber, Technion, Israel Institute	of Technology		      *
*******************************************************************************
* Written by Youngjin Park, June 2025.					      *
******************************************************************************/

#include "inc_irit/allocate.h"
#include "inc_irit/attribut.h"
#include "inc_irit/irit_sm.h"
#include "inc_irit/rndr_lib.h"
#include "inc_irit/user_lib.h"

#define USER_WALL_THICKNESSES_WEDGE_THRESHOLD_ANGLE 135

typedef enum {
    IRIT_USER_WT_MIN = 0,
    IRIT_USER_WT_AVG,
    IRIT_USER_WT_MAX
} IritUserWTAggregationMode;

#ifdef IRIT_NOT_USED
static void IritUserWTCalcViewMatrix(const IrtPtType CameraPos,
				     const IrtVecType ViewVec,
				     const IrtVecType UpVec,
				     IrtRType ViewMat[4][4]);
static void IritUserWTCnvrtPixelToWorld(IrtRType CamX,
					IrtRType CamY,
					IrtRType CamZ,
					IrtRType ViewMat[4][4],
					IrtPtType WorldPt);
static void IritUserWTCalcNrmlWithAdjPolysArea(
					     const IritPrsrObjectStruct *PObj,
					     const IritPrsrVertexStruct *V,
					     IrtVecType NormDir);
static void IritUserWTCalcNrmlWithAdjPolys(const IritPrsrObjectStruct *PObj,
					   const IritPrsrVertexStruct *V,
					   IrtVecType NormDir);
#endif /* IRIT_NOT_USED */

static int IritUserWTUpdateVertexThickness(
				const IritPrsrVertexStruct *Vertex,
				const IrtVecType ViewVec,
				IritPrsrObjectStruct *PixelInfo,
				const IritUserWallThicknessParamStruct *Params,
				IrtRType *Thickness);
static IritPrsrObjectStruct *IritUserWTGetHitsFromDepthFrame(
					    const IritPrsrObjectStruct *PlObj,
					    const IrtPtType CameraPos,
					    const IrtVecType ViewVec,
					    const IrtVecType UpVec);
static IrtBType IritUserWTGetHitsFromBVH(
			       const IritPrsrObjectStruct *PlObj,
			       const struct IritGeomPolyBVHStruct *PolyBVH,
			       const IritPrsrVertexStruct *V,
			       const int VIdx,
			       const int RayIdx,
			       const IrtVecType RayDir,
			       IrtRType *Thickness,
			       IritPrsrObjectStruct *ShootRays,
			       const IritUserWallThicknessParamStruct *Params);
static void IritUserWTCalcPtFromHit(const IritPrsrVertexStruct *Vertex,
				    const IrtRType HitDepth,
				    const IrtVecType ViewVec,
				    IrtPtType *RetPt);
static void IritUserWTGetColor(const IrtRType Value,
			       const IrtRType MinValue, 
			       const IrtRType MaxValue,
			       int *r, 
			       int *g, 
			       int *b);
static int IritUserWTSortDist(const VoidPtr D1,
			      const VoidPtr D2);
static IrtBType IritUserWTGetHits(
			       const IritPrsrObjectStruct *PObj,
			       const struct IritGeomPolyBVHStruct *PolyBVH,
			       const IritPrsrVertexStruct *V,
			       const IrtVecType ViewDir,
			       const int VrtxIdx,
			       const int RayIdx,
			       const IrtVecType ToModelCtrDir,
			       IrtRType *LocalThickness,
    			       IritPrsrObjectStruct *ShootRays,
			       const IritUserWallThicknessParamStruct *Params);
static IrtBType IritUserWTUpdateThickness(const int ThicknessMode,
					  const IrtRType LocalThickness,
					  int *HitCnt,
					  IrtRType *ThickSum);
static void IritUserWTGenDirFromNormalPyramid(const IrtVecType NormDir,
					      const int ViewIdx,
					      const int ViewVecLen,
					      const IrtRType ViewAngleTol,
					      IrtVecType ViewDir);
static int IritUserWTGetNeighborFaceCentersAndNormals(
				     const IritPrsrPolyVrtxArrayStruct *PVIdx,
				     int VIdx,
				     CagdVecStruct *FaceCenters,
				     CagdVecStruct *FaceNormals,
				     int *NeighborFaceNum);
static IrtBType IritUserWTGetTriCornerAngleDegAtV(
					   const IritPrsrPolygonStruct *CurPl,
					   const IritPrsrVertexStruct *V,
					   IrtRType *AngleDeg);
static void IritUserWTCalcNrmlWithAdjPolysDegree(
					     const IritPrsrObjectStruct *PObj,
					     const IritPrsrVertexStruct *V,
					     IrtVecType NormDir);
static IritPrsrObjectStruct *IritUserWTGenRayObj(
			       IrtPtType RayPt,
			       const IrtVecType RayDir,
			       const IrtRType LocalThickness,
			       const IritUserWallThicknessParamStruct *Params);
static IrtRType IritUserGetMaxFaceNormalAngle(CagdVecStruct *FaceNrmls, 
					      int FaceNum);
static void IritUserWTSetAttrColorWithThickness(
			       IritMiscAttributeStruct **Attr,
			       const IrtRType ThicknessVal,
			       const IrtBType ReverseColormap,
			       const IritUserWallThicknessParamStruct *Params);

IRIT_STATIC_DATA IritUserComputeWallThicknessesClrCBFuncType
     GlblColorCallBack = NULL;
IRIT_STATIC_DATA void
     *GlblColorCBData = NULL;

/*****************************************************************************
* DESCRIPTION:                                                               M
*   Sets th call back function assign a color to a value between zero and    M
* one.                                                                       M
*                                                                            *
* PARAMETERS:                                                                M
*   CallBackFunc:    New call back function.				     M
*                                                                            *
* RETURN VALUE:                                                              M
*   IritUserComputeWallThicknessesClrCBFuncType: The old call back function. M
*                                                                            *
* SEE ALSO:                                                                  M
*   IritUserComputeWallThicknesses                                           M
*                                                                            *
* KEYWORDS:                                                                  M
*   IritUserComputeWallThicknessesSetClrCB                                   M
*****************************************************************************/
IritUserComputeWallThicknessesClrCBFuncType
	   IritUserComputeWallThicknessesSetClrCB(
		     IritUserComputeWallThicknessesClrCBFuncType CallBackFunc,
		     void *CBData)
{
    IritUserComputeWallThicknessesClrCBFuncType
        OldVal = GlblColorCallBack;

    GlblColorCallBack = CallBackFunc;
    GlblColorCBData = CBData;

    return OldVal;      
}

/*****************************************************************************
* DESCRIPTION:								     M
*   Analyzes the input polygonal model to evaluate local wall thickness by   M
* casting rays into the interior from multiple directions at each vertex and M
* measuring ray--polygon intersection depths. ThicknessMode controls both    M
* the aggregation of thickness values (minimum, average, or maximum) and     M
* their mapping to colors. In binary mapping, values below ThicknessMinTol   M
* are colored red, and all others are colored green. In continuous mapping,  M 
* values below ThicknessMinTol are colored red, values above ThicknessMaxTol M
* are colored green, and intermediate values are linearly interpolated.	     M
*									     *
* PARAMETERS:								     M
*   PlObj:   Input polygonal object to be analyzed.			     M
*   Params:  The parameters for wall thickness, accumulated depth, and gap   M
*	     detection analysis.					     M
*									     *
* RETURN VALUE:								     M
*   IritPrsrObjectStruct *: The input model, where each vertex is assigned   M
*			    an RGB color based on its estimated thickness.   M
*			    If IsAccumulateRay is TRUE, the returned object  M
*			    also has a list of rays shot from each vertex,   M
*			    but the directions are inverted to outward.      M
*									     *
* SEE ALSO:								     M
*   IritUserComputeWallThicknessesSetClrCB                                   M
*									     *
* KEYWORDS:								     M
*   IritUserComputeWallThicknesses					     M
*****************************************************************************/
IritPrsrObjectStruct *IritUserComputeWallThicknesses(
			       const IritPrsrObjectStruct *PlObj,
			       const IritUserWallThicknessParamStruct* Params)
{
    int VrtxIdx, ViewVecLen,
	FaceCnt = 0,
	Processed = 0;
    IrtRType 
	MinThickness = IRIT_INFNTY,
	MaxThickness = 0,
	CosAngleTol = sin(IRIT_DEG2RAD(Params -> ViewAngleTol));
    IrtPtType ModelCtr;
    GMBBBboxStruct ModelBBox;
    struct IritGeomPolyBVHStruct *PolyBVH;
    IritPrsrPolygonStruct *Poly;
    IritPrsrObjectStruct **RaysObjArr,
	*ShootRays = NULL,
	*AccumulatedRays = IritPrsrGenLISTObject(NULL),
	*PObj = IritPrsrCopyObject(NULL, PlObj, FALSE);
    IritPrsrPolyVrtxArrayStruct
	*VrtxArr = IritPrsrCnvrtIritPolyToPolyVrtxArray(PObj, TRUE, 0);
    IritPrsrVertexStruct 
	**Vertices = VrtxArr -> Vertices;
    IritMiscProgressReportStruct PRInfo;

#ifdef DEBUG_RAY_SHOOTING
    ShootRays = IritPrsrGenLISTObject(PObj);
#endif /* DEBUG_RAY_SHOOTING */

    assert(IRIT_PRSR_IS_POLY_OBJ(PObj));

    IritPrsrSetPolyListCirc(TRUE);  /* Make sure all polygons are circular. */

    /* 0. Print information of the model. */
    
    for (Poly = PObj -> U.Pl; Poly != NULL; Poly = Poly -> Pnext)
	FaceCnt++;

    fprintf(stderr, "Model Info: #Vertices = %d, #Faces = %d\n",
	    VrtxArr -> NumVrtcs, FaceCnt);

    /* 1. Initialization with options. */
    switch (Params -> AnalysisMode) {
	case IRIT_USER_WT_WALL_THICKNESS:
	case IRIT_USER_WT_ACCUMULATED_RAY:
	case IRIT_USER_WT_GAP_THICKNESS:
	    break;
	default:
	    IRIT_WARNING_MSG("Unknown analysis mode. Aborted.\n");
	    return NULL;
    }

    /* Select the number of ray directions to be generated within the       */
    /* normal pyramid, according to the specified RayDirDensity level.      */
    if (Params -> ViewDirDensity == 0) {
	ViewVecLen = 9;
    }
    else if (Params -> ViewDirDensity == 1) {
	ViewVecLen = 25;
    }
    else if (Params -> ViewDirDensity == 2) {
	ViewVecLen = 49;
    }
    else if (Params -> ViewDirDensity == 3) {
	ViewVecLen = 100;
    }
    else {
	IRIT_WARNING_MSG("Unknown ray density. Aborted.\n");
	return NULL;
    }

    /* Validate thickness computation and coloring mode. */
    if (!((Params -> ThicknessMode >= 0 && Params -> ThicknessMode <= 1) ||
	  (Params -> ThicknessMode >= 10 && Params -> ThicknessMode <= 11) ||
	  (Params -> ThicknessMode >= 20 && Params -> ThicknessMode <= 21))) {
	IRIT_WARNING_MSG("Unknown thickness measure method. Aborted.\n");
	return NULL;
    }

    /* Build a BVH for accelerating spatial intersection queries. */
    PolyBVH = IritGeomPolyBVHCreate(PObj -> U.Pl);

    if (!Params -> UseBVH) {
	IritMiscConfigInitState();
	IritRndrInitOptions();
	
	IritRndrOptions.ShadeModel = IRNDR_SHADING_FLAT;
	IritRndrOptions.Transp = TRUE;
	IritRndrOptions.ZNear = IritRndrOptions.ZFar = -IRIT_INFNTY;
	IritRndrOptions.BackFace = FALSE;
	IritRndrOptions.DrawPoints = TRUE;
	IritRndrOptions.PointDfltRadius = 0.01;
	IritRndrOptions.FilterName = FALSE;
	IritRndrOptions.FileType = "PPM3";
	IritRndrOptions.XSize = 256;
	IritRndrOptions.YSize = 256;

	IritGeomBBComputeBboxObject(PObj, &ModelBBox, FALSE);
	IRIT_PT_BLEND(ModelCtr, ModelBBox.Min, ModelBBox.Max, 0.5);
    }

    RaysObjArr = (IritPrsrObjectStruct **)IritMalloc(
		    sizeof(IritPrsrObjectStruct *) * (VrtxArr -> NumVrtcs));
    IRIT_ZAP_MEM(RaysObjArr, sizeof(IritPrsrObjectStruct *) * 
						       (VrtxArr -> NumVrtcs));


    /* Display the processing progress. */
    IRIT_ZAP_MEM(&PRInfo, sizeof(IritMiscProgressReportStruct));
    if (IritMiscIsProgressReportSet()) {
        PRInfo.InitMsg = "Wall thickness estimation progress: ";
               IritMiscProgressReport(IRIT_MISC_PROGRESS_REPORT_INIT, &PRInfo);
    }

    /* 2. For each vertex, shoot the ray and update the thickness. */
    #pragma omp parallel for private(VrtxIdx) if (_IritParallelExec)
    for (VrtxIdx = 0; VrtxIdx < VrtxArr -> NumVrtcs; VrtxIdx++) {
	int AdjFaceCnt = 0,
	    HitCnt = 0;
	IrtRType ThickSum, ThicknessVal;
	IrtVecType ToModelCtrDir, ViewDir, NormDir, ValidRayViewDir;
	CagdVecStruct
    	    *AdjFaceCtr = IritCagdVecNew(), 
    	    *AdjFaceNrml = IritCagdVecNew();
	IritPrsrVertexStruct 
	    *V = Vertices[VrtxIdx];
	
	IRIT_VEC_RESET(ValidRayViewDir);
	IRIT_VEC_RESET(ToModelCtrDir);

	/* Display the processing progress. */
	#pragma omp atomic 
	Processed++;

	if ((Processed % IRIT_MAX(1, VrtxArr -> NumVrtcs / 100)) == 0 || 
	     Processed == VrtxArr -> NumVrtcs) {
	    #pragma omp critical
	    {
	        if (IritMiscIsProgressReportSet()) {
		    PRInfo.Progress = (int) (100.0 * (IrtRType) Processed /
					      (IrtRType) VrtxArr -> NumVrtcs);
		    IritMiscProgressReport(IRIT_MISC_PROGRESS_REPORT_UPDATE,
					   &PRInfo);
		}
	    }
	}

	if (!Params -> UseBVH) {
	    IRIT_VEC_SUB(ToModelCtrDir, ModelCtr, V -> Coord);
	    IRIT_VEC_NORMALIZE(ToModelCtrDir);
	}

	if (Params -> ThicknessMode / 10 == IRIT_USER_WT_MIN)
	    ThickSum = IRIT_INFNTY;
	else if (Params -> ThicknessMode / 10 == IRIT_USER_WT_AVG)
	    ThickSum = 0.0;
	else						/* IRIT_USER_WT_MAX */
	    ThickSum = -IRIT_INFNTY;

	IritUserWTGetNeighborFaceCentersAndNormals(VrtxArr, VrtxIdx,
						   AdjFaceCtr, AdjFaceNrml,
						   &AdjFaceCnt);

	/* Compute the vertex normal with angle-weighted 1-ring face nrmls. */
	IritUserWTCalcNrmlWithAdjPolysDegree(PObj, V, NormDir);

	/* Check whether the 1-ring face nrmls are sufficiently consistent  */
	if (IritUserGetMaxFaceNormalAngle(AdjFaceNrml, AdjFaceCnt) /* (not  */
	    < USER_WALL_THICKNESSES_WEDGE_THRESHOLD_ANGLE) {	 /* wedge). */
	    int ViewIdx;
	    IrtPtType MovedPt;

		/* Shoot ViewVecLen + 15 sampled rays. +1 is the normal, +14 */
	    for (ViewIdx = 0; ViewIdx < ViewVecLen + 15; ViewIdx++) {  /* is */
		IrtBType			    /* 6 axis + 8 diagonals. */
	    	    HasHit = FALSE;

		if (ViewIdx < ViewVecLen) /* Shoot rays with normal pyramid. */
		    IritUserWTGenDirFromNormalPyramid(NormDir, ViewIdx,
						      ViewVecLen,
						      Params -> ViewAngleTol,
						      ViewDir);
		else if (ViewIdx == ViewVecLen)	   /* Shoot a ray along with */
		    IRIT_VEC_COPY(ViewDir, NormDir); /* angle-weighted nrml. */
		else {
		    int 
			ExtraIdx = ViewIdx - (ViewVecLen + 1);
		    IrtVecType Dir;

		    if (ExtraIdx < 6) {  /* Shoot rays along with the 6 axis */
			IRIT_VEC_SET(Dir, 0.0, 0.0, 0.0);     /* directions. */
			Dir[ExtraIdx >> 1] = (ExtraIdx & 1) ? 1.0 : -1.0;
		    }
		    else {   /* Shoot rays along with the 8 diag directions. */
			IrtRType 
		    	    Sx = ((ExtraIdx - 6) & 1) ? 1.0 : -1.0,
			    Sy = ((ExtraIdx - 6) & 2) ? 1.0 : -1.0,
		    	    Sz = ((ExtraIdx - 6) & 4) ? 1.0 : -1.0;

			IRIT_VEC_SET(Dir, Sx, Sy, Sz);
			IRIT_VEC_NORMALIZE(Dir);
		    }

		    if (IRIT_DOT_PROD(Dir, NormDir) < CosAngleTol)     /* If */
			continue;		      /* outside cone, skip. */

		    IRIT_VEC_COPY(ViewDir, Dir);
		}

		if (Params -> AnalysisMode == IRIT_USER_WT_GAP_THICKNESS)
		    IRIT_VEC_SCALE(ViewDir, -1.0);

		/* If the ray is placed outside the model, do not test. */
		IRIT_PT_SCALE_AND_ADD(MovedPt, V -> Coord, ViewDir, 1e-4);
		if (!IritGeomPolyBVHPointInsidePolys(MovedPt, PolyBVH) &&
		    Params -> AnalysisMode != IRIT_USER_WT_GAP_THICKNESS)
		    continue;
		/* Shoot a ray. */
		HasHit = IritUserWTGetHits(PObj, PolyBVH, V, ViewDir, VrtxIdx,
					   ViewIdx, ToModelCtrDir,
					   &ThicknessVal, ShootRays, Params);


		    
		/* If there is a hit, check the previously known thickness  */
		if (HasHit) {			  /* can be updated or not. */
		    if (IritUserWTUpdateThickness(Params -> ThicknessMode, 
						  ThicknessVal, &HitCnt, 
						  &ThickSum))
			IRIT_VEC_COPY(ValidRayViewDir, ViewDir);
		}
		else { /* No hit should not be happened w/ watertight model. */
		    fprintf(stderr, "\nError: No hit for vrtx %d, ray %d.\n", 
			    VrtxIdx, ViewIdx);
		    IritMiscAttrIDSetRealAttrib(&V -> Attr,
						IRIT_ATTR_CREATE_ID(Thickness),
						ThicknessVal);
		}
	    }

	    if (HitCnt == 0) {			  /* Prevent zero division. */
		/* Free memory. */
		IritCagdVecFreeList(AdjFaceCtr);
		IritCagdVecFreeList(AdjFaceNrml);
		continue;

	    }

	    /* Update thickness if at least one valid ray hit is found. */
	    IritMiscAttrIDSetRealAttrib(&V -> Attr,
					IRIT_ATTR_CREATE_ID(Thickness),
					ThickSum / (IrtRType) HitCnt);

	    /* Visualize rays only for accumulated ray analysis mode. */
	    if (Params -> AnalysisMode == IRIT_USER_WT_ACCUMULATED_RAY) {
		IrtRType
		    AccumDepth = ThickSum / (IrtRType) HitCnt;

		/* If the aggregation is avg, we set a representative ray to */
		if (Params -> ThicknessMode / 10 == IRIT_USER_WT_AVG) /* its */
		    IRIT_VEC_COPY(ValidRayViewDir, NormDir);	  /* normal. */

		IRIT_VEC_SCALE(ValidRayViewDir, -1.0)  /* Make ray outward. */

		if (AccumDepth > Params -> AccumDepthTol)
		    RaysObjArr[VrtxIdx] = IritUserWTGenRayObj(V -> Coord,
							      ValidRayViewDir,
							      AccumDepth,
							      Params);
	    }
	}
	else {			/* Degenerate wedge: set thickness to +EPS. */
	    IritMiscAttrIDSetRealAttrib(&V -> Attr, 
					IRIT_ATTR_CREATE_ID(Thickness), 
					IRIT_EPS);
	}

	/* Free memory. */
	IritCagdVecFreeList(AdjFaceCtr);
	IritCagdVecFreeList(AdjFaceNrml);
    }

    if (IritMiscIsProgressReportSet())
        IritMiscProgressReport(IRIT_MISC_PROGRESS_REPORT_DONE, &PRInfo);

#ifdef DEBUG_RAY_SHOOTING
    {
	char
    	    *FileName = "ShootRays.itd";
	IritPrsrObjectStruct
	    *AllPtsObj = IritPrsrGenListObject("Vertices", NULL, NULL);

	IritPrsrPutObjectToFile3(FileName, ShootRays, 0);

	for (VrtxIdx = 0; VrtxIdx < VrtxArr -> NumVrtcs; VrtxIdx++) {
	    char VertIdxName[20];
	    IritPrsrVertexStruct
		*V = Vertices[VrtxIdx];
	    IrtRType
		ThicknessVal = IritMiscAttrIDGetRealAttrib(V -> Attr,
					      IRIT_ATTR_CREATE_ID(Thickness));
	    IritPrsrObjectStruct *PtObj;

	    if (ThicknessVal > 1E+5)
		ThicknessVal = 1E+5;

	    sprintf(VertIdxName, "%d_%.3f\n", VrtxIdx, ThicknessVal);

	    PtObj = IritPrsrGenPtObject(VertIdxName, &V -> Coord[0], 
					&V -> Coord[1], &V -> Coord[2], NULL);
	    IritPrsrListObjectAppend(AllPtsObj, PtObj);
	}

	IritPrsrPutObjectToFile3("Vertices.itd", AllPtsObj, 0);
    }
#endif /* DEBUG_RAY_SHOOTING */

    /* Iterate all vertices and get minimum and maximum thicknesses. */
    for (VrtxIdx = 0; VrtxIdx < VrtxArr -> NumVrtcs; VrtxIdx++) {
	IritPrsrVertexStruct 
	    *V = Vertices[VrtxIdx];
	IrtRType 
	    ThicknessVal = IritMiscAttrIDGetRealAttrib(V -> Attr, 
					      IRIT_ATTR_CREATE_ID(Thickness));
	
	if (ThicknessVal < 0)
	    continue;

    	MinThickness = IRIT_MIN(MinThickness, ThicknessVal);
	MaxThickness = IRIT_MAX(MaxThickness, ThicknessVal);
    }

    IritMiscAttrIDSetRealAttrib(&PObj -> Attr, /* Set min and max thickness.*/
				IRIT_ATTR_CREATE_ID(ThicknessMin), 
				MinThickness); 
    IritMiscAttrIDSetRealAttrib(&PObj -> Attr, 
				IRIT_ATTR_CREATE_ID(ThicknessMax), 
				MaxThickness);
    fprintf(stderr, "MinThickness %f MaxThickness %f.\n", MinThickness,  
							  MaxThickness);

    /* 3. Set color for each vertex of polys. */
    for (Poly = PObj -> U.Pl; Poly != NULL; Poly = Poly -> Pnext) {
	IritPrsrVertexStruct *PolyVrtx;

	for (PolyVrtx = Poly -> PVertex; PolyVrtx != NULL;
	     PolyVrtx = PolyVrtx -> Pnext) {
	    int 
		VIndex = IritMiscAttrIDGetIntAttrib(PolyVrtx -> Attr,
						  IRIT_ATTR_CREATE_ID(_VIdx));
	    IrtRType ThicknessVal;
	    IritPrsrVertexStruct *V;

	    VIndex = IRIT_ABS(VIndex) - 1;
	    V = Vertices[VIndex];

	    /* Propagate computed per-vertex attributes to the polygon. */
	    ThicknessVal = IritMiscAttrIDGetRealAttrib(V -> Attr, 
					      IRIT_ATTR_CREATE_ID(Thickness));

	    IritMiscAttrIDSetRealAttrib(&PolyVrtx -> Attr, 
					IRIT_ATTR_CREATE_ID(Thickness),
					ThicknessVal);

	    if (ThicknessVal > 0)
		IritUserWTSetAttrColorWithThickness(&PolyVrtx -> Attr,
						    ThicknessVal, FALSE, 
						    Params);
	    else     /* Set black to vertex if there is no thickness value. */
		IritMiscAttrIDSetRGBColor(&PolyVrtx -> Attr, 0, 0, 0);

	    if (PolyVrtx -> Pnext == Poly -> PVertex)
		break;
	}
    }

    /* 4. Generate rays from vertex with its thickness for visualizing. */
    if (Params -> AnalysisMode == IRIT_USER_WT_ACCUMULATED_RAY) {
	char ObjName[50];

	PObj = IritPrsrGenLISTObject(PObj);

	sprintf(ObjName, "AccumulatedDepthRays");
	IRIT_PRSR_SET_OBJ_NAME2(AccumulatedRays, ObjName);

	for (VrtxIdx = 0; VrtxIdx < VrtxArr -> NumVrtcs; VrtxIdx++)
	    if (RaysObjArr[VrtxIdx] != NULL)
		IritPrsrListObjectAppend(AccumulatedRays, RaysObjArr[VrtxIdx]);

	IritPrsrListObjectAppend(PObj, AccumulatedRays);
    }
    else {
	IritPrsrFreeObject(AccumulatedRays);
    }

    /* Free memory. */
    IritFree(RaysObjArr);
    IritPrsrPolyVrtxArrayFree(VrtxArr);
    IritGeomPolyBVHFree(PolyBVH);

    return PObj;
}

/*****************************************************************************
* DESCRIPTION:								     *
*   Computes a 4x4 view matrix (camera transformation matrix) based on the   *
* given camera position, viewing direction, and up vector. This matrix	     *
* transforms world-space coordinates into view-space coordinates using a     *
* right-handed coordinate system.					     *
*									     *
* PARAMETERS:								     *
*   CameraPos:  IN, camera position in world coordinates.		     *
*   ViewVec:    IN, forward viewing direction vector.			     *
*   UpVec:      IN, up direction vector of the camera.			     *
*   ViewMat:    OUT, resulting 4x4 view matrix.				     *
*									     *
* RETURN VALUE:								     *
*   void								     *
*****************************************************************************/
static void IritUserWTCalcViewMatrix(const IrtPtType CameraPos, 
				     const IrtVecType ViewVec,
				     const IrtVecType UpVec,
				     IrtRType ViewMat[4][4])
{
    IrtVecType ViewPos, f, s, u, Up;

    IRIT_VEC_COPY_AND_NORMALIZE(Up, UpVec);

    IRIT_VEC_ADD(ViewPos, CameraPos, ViewVec);

    IRIT_VEC_SUB(f, ViewPos, CameraPos);
    IRIT_VEC_NORMALIZE(f);

    IRIT_CROSS_PROD(s, f, Up);
    IRIT_VEC_NORMALIZE(s);

    IRIT_CROSS_PROD(u, s, f);

    ViewMat[0][0] = s[0];
    ViewMat[0][1] = u[0];
    ViewMat[0][2] = -f[0];
    ViewMat[0][3] = 0.0;

    ViewMat[1][0] = s[1];
    ViewMat[1][1] = u[1];
    ViewMat[1][2] = -f[1];
    ViewMat[1][3] = 0.0;

    ViewMat[2][0] = s[2];
    ViewMat[2][1] = u[2];
    ViewMat[2][2] = -f[2];
    ViewMat[2][3] = 0.0;

    ViewMat[3][0] = -IRIT_DOT_PROD(s, CameraPos);
    ViewMat[3][1] = -IRIT_DOT_PROD(u, CameraPos);
    ViewMat[3][2] = IRIT_DOT_PROD(f, CameraPos);
    ViewMat[3][3] = 1.0;
}

#ifdef IRIT_NOT_USED

/*****************************************************************************
* DESCRIPTION:								     *
*   Converts a given pixel position and depth value in view space into a 3D  *
* world-space coordinate, using the inverse of the provided view matrix.     *
* The input pixel (CamX, CamY) is normalized based on the render resolution, *
* and the result is stored in WorldPt.					     *
*									     *
* PARAMETERS:								     *
*   CamX, CamY: IN, Pixel coordinates in image space.			     *
*   CamZ:       IN, Depth value at the pixel, in view space.		     *
*   ViewMat:    IN, 4x4 view matrix used to transform from world to view     *
*		space.							     *
*   WorldPt:    OUT, 3D point in world space.				     *
*									     *
* RETURN VALUE:								     *
*   void								     *
*****************************************************************************/
static void IritUserWTCnvrtPixelToWorld(IrtRType CamX,
					IrtRType CamY,
					IrtRType CamZ,
					IrtRType ViewMat[4][4],
					IrtPtType WorldPt)
{
    IrtRType ViewPt[4];
    IrtRType WPt[4];
    IrtHmgnMatType InvMat;

    ViewPt[0] = -1. + (CamX + 0.5) * 2. / IritRndrOptions.XSize;
    ViewPt[1] = -1. + (CamY + 0.5) * 2. / IritRndrOptions.YSize;
    ViewPt[2] = CamZ;
    ViewPt[3] = 1.0;

    IritMiscMatInverseMatrix(ViewMat, InvMat);

    IritMiscMatMultPtby4by4(WPt, ViewPt, InvMat);

    WorldPt[0] = WPt[0];
    WorldPt[1] = WPt[1];
    WorldPt[2] = WPt[2];
}

/*****************************************************************************
* DESCRIPTION:                                                               *
*   Computes a vertex normal by averaging the normals of all polygons        *
* adjacent to the given vertex, using each polygon's area as a weight. The   *
* resulting normal is normalized and returned in NormDir.                    *
*                                                                            *
* PARAMETERS:                                                                *
*   PObj:     IN,  input polygonal object.                                   *
*   V:        IN,  vertex whose normal is to be computed.                    *
*   NormDir:  OUT, Output vector to store the computed (normalized) normal.  *
*                                                                            *
* RETURN VALUE:                                                              *
*   void                                                                     *
*****************************************************************************/
static void IritUserWTCalcNrmlWithAdjPolysArea(
					     const IritPrsrObjectStruct *PObj,
					     const IritPrsrVertexStruct *V,
					     IrtVecType NormDir)
{
    IrtRType
	SumArea = 0;
    IPPolygonStruct *CurPl;

    IRIT_VEC_SET(NormDir, 0, 0, 0);

    IrtRType
	D0 = 0;
    for (CurPl = PObj -> U.Pl; CurPl != NULL; CurPl = CurPl -> Pnext) {
	IritPrsrVertexStruct
	    *Vrtx = CurPl -> PVertex;

	do {
	    if (IRIT_PT_APX_EQ(Vrtx -> Coord, V -> Coord)) {
		IrtVecType AB, AC, ABC;

		Vrtx = CurPl -> PVertex;
		IRIT_VEC_SUB(AB, Vrtx -> Pnext -> Coord, Vrtx -> Coord)
		IRIT_VEC_SUB(AC, Vrtx -> Pnext -> Pnext -> Coord, 
			     Vrtx -> Coord)

		IRIT_CROSS_PROD(ABC, AB, AC)
		D0 = IRIT_VEC_LENGTH(ABC) * 0.5;
		SumArea += D0;

		break;
	    }

	    Vrtx = Vrtx -> Pnext;
	}
    	while (Vrtx != NULL && Vrtx != CurPl -> PVertex);
    }

    for (CurPl = PObj -> U.Pl; CurPl != NULL; CurPl = CurPl -> Pnext) {
	IritPrsrVertexStruct
	    *Vrtx = CurPl -> PVertex;

	do {
	    if (IRIT_PT_APX_EQ(Vrtx -> Coord, V -> Coord)) {
		IrtRType Area, t;
		IrtVecType AB, AC, ABC;

		Vrtx = CurPl -> PVertex;
		IRIT_VEC_SUB(AB, Vrtx -> Pnext -> Coord, Vrtx -> Coord)
		IRIT_VEC_SUB(AC, Vrtx -> Pnext -> Pnext -> Coord, 
			     Vrtx -> Coord)

		IRIT_CROSS_PROD(ABC, AB, AC)
		Area = IRIT_VEC_LENGTH(ABC) * 0.5;

		t = Area / SumArea;
		IRIT_PT_SCALE_AND_ADD(NormDir, NormDir, CurPl -> Plane, t);

		break;
	    }

	    Vrtx = Vrtx -> Pnext;
	}
    	while (Vrtx != NULL && Vrtx != CurPl -> PVertex);
    }

    IRIT_VEC_NORMALIZE(NormDir);
}

/*****************************************************************************
* DESCRIPTION:                                                               *
*   Computes a vertex normal by averaging the normals of all polygons        *
* adjacent to the given vertex. The resulting normal is normalized and       *
* returned in NormDir.                                                       *
*                                                                            *
* PARAMETERS:                                                                *
*   PObj:     IN,  input polygonal object.                                   *
*   V:        IN,  vertex whose normal is to be computed.                    *
*   NormDir:  OUT, Output vector to store the computed (normalized) normal.  *
*                                                                            *
* RETURN VALUE:                                                              *
*   void                                                                     *
*****************************************************************************/
static void IritUserWTCalcNrmlWithAdjPolys(const IritPrsrObjectStruct *PObj,
					   const IritPrsrVertexStruct *V,
					   IrtVecType NormDir)
{
    int
	AdjPolyCnt = 0;
    IPPolygonStruct *CurPl;

    IRIT_VEC_SET(NormDir, 0, 0, 0)

	for (CurPl = PObj -> U.Pl; CurPl != NULL; CurPl = CurPl -> Pnext) {
	    IritPrsrVertexStruct
		*Vrtx = CurPl -> PVertex;

	    do {
		if (IRIT_PT_APX_EQ(Vrtx -> Coord, V -> Coord)) {
		    IRIT_VEC_ADD(NormDir, NormDir, CurPl -> Plane);
		    AdjPolyCnt++;
		    break;
		}

		Vrtx = Vrtx -> Pnext;
	    }
	    while (Vrtx != NULL && Vrtx != CurPl -> PVertex);
	}

    IRIT_VEC_SCALE(NormDir, 1.0 / (IrtRType) AdjPolyCnt);
    IRIT_VEC_NORMALIZE(NormDir);
}
#endif /* IRIT_NOT_USED */

/*****************************************************************************
* DESCRIPTION:								     *
*   Computes the (accumulated) wall thickness or gap at a given vertex by    *
* analyzing a list of depth hits (PixelInfo) along a viewing ray. It	     *
* calculates the distance between each pair of hit points. If a valid	     *
* thickness is found, the output variable is updated.			     *
*									     *
* PARAMETERS:								     *
*   Vertex:     IN, a vertex being analyzed.				     *
*   ViewVec:    IN, direction of the ray used to collect depth hits.	     *
*   PixelInfo:  IN, list of depth hits along the ray at this pixel.	     *
*   Params:     IN, the parameters for wall thickness, accumulated depth,    *
*		    and gap detection analysis.				     *
*   Thickness:  OUT, computed thickness at this vertex.			     *
*									     *
* SEE ALSO:								     *
*   IritUserWTGetHitsFromBVH						     *
*									     *
* RETURN VALUE:								     *
*   int:	   TRUE if a valid thickness was found and written.	     *
*****************************************************************************/
static int IritUserWTUpdateVertexThickness(
				const IritPrsrVertexStruct *Vertex,
				const IrtVecType ViewVec,
				IritPrsrObjectStruct *PixelInfo,
				const IritUserWallThicknessParamStruct *Params,
				IrtRType *Thickness)
{
    IrtBType FindThickness = FALSE;
    int i, ReadIdx,
	WriteIdx = 0,
	HitListLength = IritPrsrListObjectLength(PixelInfo);
    IrtRType *DistList, LocalThickness;
    IrtPtType *SortedPts;
	

    if (HitListLength < 2)
	return FALSE;				  /* No hits, no thickness. */

    DistList = (IrtRType *)IritMalloc(sizeof(IrtRType) * (HitListLength));
    SortedPts = (IrtPtType *)IritMalloc(sizeof(IrtPtType) * HitListLength);

    for (i = 0; i < HitListLength; i++)
	DistList[i] = -IritPrsrListObjectGet(PixelInfo, i) -> U.R;

    qsort(DistList, HitListLength, sizeof(IrtRType), IritUserWTSortDist);

    for (ReadIdx = 0; ReadIdx < HitListLength; ReadIdx++) {	   /* Remove */
	IrtRType					      /* duplicates. */
	    x = DistList[ReadIdx];

	if (WriteIdx == 0 || !IRIT_APX_EQ(x, DistList[WriteIdx - 1]))
	    DistList[WriteIdx++] = x;
    }

    HitListLength = WriteIdx;

    for (i = 0; i < HitListLength; i++)
	IritUserWTCalcPtFromHit(Vertex, -DistList[i], ViewVec, 
				&SortedPts[i]);

    if (Params -> AnalysisMode == IRIT_USER_WT_ACCUMULATED_RAY)	   /* Set 0 */
	LocalThickness = 0;		 /* for accumulating all intervals. */
    else						   /* Set infinity  */
	LocalThickness = IRIT_INFNTY;	/* for taking the minimum interval. */

    for (i = Params -> AnalysisMode == IRIT_USER_WT_GAP_THICKNESS ? 1 : 0;
	 i + 1 < HitListLength;		   /* Even-odd pairs for thickness, */
	 i += 2) {			     /* and odd-even pairs for gap. */
	IrtRType 
    	    Interval = IRIT_PT_PT_DIST(SortedPts[i], SortedPts[i + 1]);

	/* Skip if the ray interval is too close. */
	if (IRIT_APX_EQ_EPS(Interval, 0, 1e-6))
	    continue;

	FindThickness = TRUE;

	if (Params -> AnalysisMode == IRIT_USER_WT_ACCUMULATED_RAY)
	    LocalThickness += Interval;
	else {		   /* For wall/gap thickness, we only consider local */
	    LocalThickness = Interval;			    /* neighborhood. */
	    break;
	}
    }

    if (FindThickness)
	*Thickness = LocalThickness;

    IritFree(DistList);
    IritFree(SortedPts);

    return FindThickness;
}

/*****************************************************************************
* DESCRIPTION:								     *
*   Renders the input polygonal model from a given viewpoint and returns the *
* list of depth hits along the center pixel's ray.			     *
*									     *
* PARAMETERS:								     *
*   PlObj:	Polygonal object to be rendered.			     *
*   CameraPos:  Camera position.					     *
*   ViewVec:    Camera viewing direction.				     *
*   UpVec:	Camera up vector.					     *
*									     *
* RETURN VALUE:								     *
*   IritPrsrObjectStruct *: list of depth-intersection points along ray.     *
*****************************************************************************/
static IritPrsrObjectStruct *IritUserWTGetHitsFromDepthFrame(
					    const IritPrsrObjectStruct *PlObj,
					    const IrtPtType CameraPos,
					    const IrtVecType ViewVec,
					    const IrtVecType UpVec)
{
    IrtBType
	DoClipping = FALSE;
    int y, x;
    IrtHmgnMatType ViewMat;
    IRndrPtrType Rend;
    
    IritPrsrObjectStruct *Obj, *RetObj, *VisMapTriangles, ***ZDepthFrame;

    Rend = IritRndrInitialize(IritRndrOptions.XSize,
			      IritRndrOptions.YSize,
			      IritRndrOptions.FilterName ? 3 : 1,
			      IritRndrOptions.NPRClrQuant,
			      IritRndrOptions.Transp,
			      IritRndrOptions.BackFace,
			      IritRndrOptions.BackGround,
			      IritRndrOptions.Ambient,
			      IritRndrOptions.VisMap);

    IritRndrSetShadeModel(Rend,
			  (IRndrShadingType)IritRndrOptions.ShadeModel);
    
    IritUserWTCalcViewMatrix(CameraPos, ViewVec, UpVec, ViewMat);
    IritPrsrWasViewMat = TRUE;
    IritRndrSetViewPrsp(Rend, ViewMat, NULL, NULL);

    Obj = IritPrsrCopyObject(NULL, PlObj, FALSE);
    IritRndrScanObjects(Rend, Obj, DoClipping, ViewMat,	&VisMapTriangles);

    /* Get per-pixel depth hit lists from the renderer. */
    ZDepthFrame = IritRndrZBufferGetPixelHitWrapper(Rend);
    RetObj = IritPrsrCopyObject(
	    NULL,
	    ZDepthFrame[IritRndrOptions.YSize / 2][IritRndrOptions.XSize / 2],
	    FALSE);

    /* Free memory. */
    for (y = 0; y < IritRndrOptions.YSize; y++) {
	for (x = 0; x < IritRndrOptions.XSize; x++)
	    IritPrsrFreeObjectList(ZDepthFrame[y][x]);
	IritFree(ZDepthFrame[y]);
    }
    IritFree(ZDepthFrame);

#ifdef DEBUG_PRINT_PPM
    char 
	*BaseDirectory = "";
    char FileName[100];
    
    sprintf(FileName,
	    "Pos%.3f_%.3f_%.3f_View%.3f_%.3f_%.3f_UP%.3f_%.3f_%.3f.ppm",
	    CameraPos[0], CameraPos[1], CameraPos[2],
	    ViewVec[0], ViewVec[1], ViewVec[2],
	    UpVec[0], UpVec[0], UpVec[0]);

    IritRndrSaveFile(Rend, BaseDirectory, FileName, "PPM3");
#endif /* DEBUG_PRINT_PPM */

    IritRndrDestroy(Rend);

    return RetObj;
}

/*****************************************************************************
* DESCRIPTION:								     *
*   Maps a scalar Value to an RGB color by quadratic Bezier interpolation    *
* between red (MinValue), yellow , and green (MaxValue). Values below MinVal *
* are clamped to red, above MaxValue to green.				     *
*									     *
* PARAMETERS:								     *
*   Value:     IN,  the scalar Value to map to a color.			     *
*   MinValue:  IN,  minimum threshold for normalization.		     *
*   MaxValue:  IN,  maximum threshold for normalization.		     *
*   r:	       OUT, resulting red component [0, 255].			     *
*   g:	       OUT, resulting green component [0, 255].			     *
*   b:	       OUT, resulting blue component, always 0.			     *
*									     *
* RETURN VALUE:								     *
*	void								     *
*****************************************************************************/
static void IritUserWTGetColor(const IrtRType Value,
			       const IrtRType MinValue, 
			       const IrtRType MaxValue,
			       int *r,
			       int *g,
			       int *b) 
{
    IrtRType 
	t = IRIT_BOUND((Value - MinValue) / (MaxValue - MinValue), 0.0, 1.0);

    if (GlblColorCallBack != NULL) {
        GlblColorCallBack(GlblColorCBData, t, r, g, b);
	return;
    }

    *r = (int)((IRIT_SQR(1.0 - t) + 2 * (1.0 - t) * t) * 255);
    *g = (int)((2 * (1.0 - t) * t + IRIT_SQR(t)) * 255);
    *b = 0;
}

/*****************************************************************************
* DESCRIPTION:  							     *
*   Computes the 3D hit point from a given vertex along a specified view     *
* direction and a hit depth.						     *
*									     *
* PARAMETERS:								     *
*   Vertex:    IN,  pointer to the vertex from which the ray originates.     *
*   HitDepth:  IN,  scalar depth value representing the distance from the    *
*		    vertex along the negative view direction.		     *
*   ViewVec:   IN,  view direction vector (assumed to be normalized).	     *
*   RetPt:     OUT, computed 3D point of intersection (hit point).	     *
*									     *
* RETURN VALUE:								     *
*   void								     *
*****************************************************************************/
static void IritUserWTCalcPtFromHit(const IritPrsrVertexStruct *Vertex,
				    const IrtRType HitDepth,
				    const IrtVecType ViewVec,
				    IrtPtType *RetPt)
{
    IrtVecType HitDir;

    IRIT_PT_COPY(*RetPt, Vertex -> Coord);
    IRIT_VEC_COPY(HitDir, ViewVec);
    IRIT_VEC_SCALE(HitDir, -HitDepth);
    IRIT_PT_ADD(*RetPt, *RetPt, HitDir);
}

/*****************************************************************************
* DESCRIPTION:                                                               *
*   Casts a single ray from the given vertex through a polygonal BVH and     *
* collects intersection depths in ascending order  (v = p0).		     *
*									     *
* Even-odd pairs (p_2i, p_2i+1) define interior thickness segments:	     *
*    - WallThickness:  minimum interior segment.			     *
*    - AccumDepth:     sum of all interior segments.			     *
*									     *
* Odd-even pairs (p_2i+1, p_2i+2) define gaps:				     *
*    - Gap:	       minimum exterior segment (cavity).		     *
*                                                                            *
* PARAMETERS:                                                                *
*   PlObj:      IN,  polygonal object to be tested.                          *
*   PolyBVH:    IN,  BVH built from PlObj for ray queries.                   *
*   V:          IN,  vertex providing the ray origin.                        *
*   VIdx:       IN,  index of V (for bookkeeping/diagnostics).               *
*   RayIdx:     IN,  index of the current ray for this vertex.               *
*   RayDir:     IN,  unit ray direction.                                     *
*   Thickness:  OUT, on success, measured scalar value.			     *
*   ShootRays:  OUT, optional container for visualization of rays for        *
*                    debugging. Can be NULL if not needed.                   *
*   Params:     IN,  the parameters for wall thickness, accumulated depth,   *
*		     and gap detection analysis.			     *
* SEE ALSO:								     *
*   IritUserWTUpdateVertexThickness					     *
*                                                                            *
* RETURN VALUE:                                                              *
*   IrtBType: TRUE if a valid thickness was found, FALSE otherwise.          *
*****************************************************************************/
static IrtBType IritUserWTGetHitsFromBVH(
				const IritPrsrObjectStruct *PlObj,
				const struct IritGeomPolyBVHStruct *PolyBVH,
				const IritPrsrVertexStruct *V,
				const int VIdx,
				const int RayIdx,
				const IrtVecType RayDir,
				IrtRType *Thickness,
				IritPrsrObjectStruct *ShootRays,
				const IritUserWallThicknessParamStruct *Params)
{
    IrtBType FindThickness = FALSE;
    int i, NumInters, ReadIdx,
	WriteIdx = 0,
	PlLen = IritPrsrPolyListLen(PlObj -> U.Pl);
    IrtRType LocalThickness, *DistList;
    IrtPtType RayPt,
	*AllIntersPts = (IrtPtType *) IritMalloc(sizeof(IrtPtType) * PlLen),
	*SortedInterPts = (IrtPtType *) IritMalloc(sizeof(IrtPtType) * PlLen);
    const IritPrsrPolygonStruct
	**AllInterPls = (const IritPrsrPolygonStruct **) IritMalloc
				    (sizeof(IritPrsrPolygonStruct *) * PlLen);
#ifdef DEBUG_RAY_SHOOTING
    IrtBType 
	DebugOut = FALSE;
    IritPrsrObjectStruct *HitListObj;

    if (VIdx == 0 || VIdx == 3 || VIdx == 1000)
	DebugOut = TRUE;
#endif 

    /* 1. Move the ray origin slightly forward to ray direction, shoot rays */
    IRIT_PT_COPY(RayPt, V -> Coord);		  /* and get intersections. */
    IRIT_PT_SCALE_AND_ADD(RayPt, RayPt, RayDir, 1e-4);

    NumInters = IritGeomPolyBVHGetRayBVHIntersectionPt(RayPt,
						       RayDir,
						       PolyBVH,
						       AllIntersPts,
						       AllInterPls);

    if (NumInters == 0) {     /* If there is no intersection, return FALSE. */
	IritFree(AllIntersPts);
	IritFree(SortedInterPts);
	IritFree((IritPrsrPolygonStruct *) AllInterPls);

	return FALSE;
    }

    /* 2. Sort the intersection points by distance from the ray origin. */
    DistList = (IrtRType *)IritMalloc(sizeof(IrtRType) * (NumInters + 1));

    for (i = 0; i < NumInters; i++)
	DistList[i] = IRIT_PT_PT_DIST(AllIntersPts[i], V -> Coord);

    DistList[NumInters] = 0;		      /* Always include the origin. */
    qsort(DistList, NumInters + 1, sizeof(IrtRType), IritUserWTSortDist);

    for (ReadIdx = 0; ReadIdx < NumInters + 1; ReadIdx++) {	   /* Remove */
	IrtRType					      /* duplicates. */
    	    x = DistList[ReadIdx];

	if (WriteIdx == 0 || !IRIT_APX_EQ(x, DistList[WriteIdx - 1]))
	    DistList[WriteIdx++] = x;
    }

    NumInters = WriteIdx;

#ifdef DEBUG_RAY_SHOOTING
    if (DebugOut) {
	char GroupName[20];

	sprintf(GroupName, "Vert%d-Ray%d", VIdx, RayIdx);
	HitListObj = IritPrsrGenListObject(GroupName, NULL, NULL);
    }
#endif /* DEBUG_RAY_SHOOTING */

    /* 3. Compute the thickness with the sorted hit points. */
    /* 3-a. Overwrite the hit points from the sorted distances. */
    for (i = 0; i < NumInters; i++)
	IritUserWTCalcPtFromHit(V, -DistList[i], RayDir, &SortedInterPts[i]);

    /* 3-b. Compute the thickness. */
    if (Params -> AnalysisMode == IRIT_USER_WT_ACCUMULATED_RAY)	   /* Set 0 */
	LocalThickness = 0;		 /* for accumulating all intervals. */
    else						   /* Set infinity  */
	LocalThickness = IRIT_INFNTY;	/* for taking the minimum interval. */

    for (i = 0;/*Params -> AnalysisMode == IRIT_USER_WT_GAP_THICKNESS ? 1 : 0;*/
	 i + 1 < NumInters;		   /* Even-odd pairs for thickness, */ 
	 i += 2) {			     /* and odd-even pairs for gap. */
	IrtRType 
    	    Interval = IRIT_PT_PT_DIST(SortedInterPts[i], 
				       SortedInterPts[i + 1]);

	if (IRIT_APX_EQ(Interval, 0))	     /* Skip if the ray interval is */ 
	    continue;					      /* too close. */

#ifdef DEBUG_RAY_SHOOTING
	if (DebugOut) {
	    char Pt0Name[20], Pt1Name[20], RayName[20];
	    IrtPtType CurPt, NextPt;
	    CagdPolylnStruct *CagdPoly;
	    CagdPolylineStruct *CagdPolyline;
	    IritPrsrPolygonStruct *RayPoly;
	    IritPrsrObjectStruct *Pt0Obj, *Pt1Obj, *RayObj;

	    IRIT_PT_COPY(CurPt, SortedInterPts[i]);
	    IRIT_PT_COPY(NextPt, SortedInterPts[i + 1]);

	    sprintf(Pt0Name, "HitPt%d-%f", i, DistList[i]);
	    sprintf(Pt1Name, "HitPt%d-%f", i + 1, DistList[i + 1]);

	    Pt0Obj = IritPrsrGenPtObject(Pt0Name, 
					 &CurPt[0], &CurPt[1], &CurPt[2],
					 NULL);
	    Pt1Obj = IritPrsrGenPtObject(Pt1Name, 
					 &NextPt[0], &NextPt[1], &NextPt[2],
					 NULL);

	    CagdPolyline = IritCagdPolylineNew(2, 3);
	    CagdPoly = CagdPolyline -> Polyline;
	    IRIT_PT_COPY(CagdPoly -> Pt, CurPt);
	    CagdPoly++;
	    IRIT_PT_COPY(CagdPoly -> Pt, NextPt);
	    RayPoly = IritUserCnvrtCagdPolyline2IritPolyline(CagdPolyline);
	    sprintf(RayName, "Ray_%d", i);
	    RayObj = IritPrsrGenPolylineObject(RayName, RayPoly, NULL);
	    IritMiscAttrIDSetRGBColor(&RayObj -> Attr, 0, 255, 0);

	    IritPrsrListObjectAppend(HitListObj, Pt0Obj);
	    IritPrsrListObjectAppend(HitListObj, RayObj);
	    IritPrsrListObjectAppend(HitListObj, Pt1Obj);

	    IritCagdPolylineFree(CagdPolyline);
	}
#endif /* DEBUG_RAY_SHOOTING */

	FindThickness = TRUE;

	if (Params -> AnalysisMode == IRIT_USER_WT_ACCUMULATED_RAY)
	    LocalThickness += Interval;
	else {		   /* For wall/gap thickness, we only consider local */
	    LocalThickness = Interval;			    /* neighborhood. */
	    break;
	}
    }

    if (FindThickness)
	*Thickness = LocalThickness;

#ifdef DEBUG_RAY_SHOOTING
    if (DebugOut)
	IritPrsrListObjectAppend(ShootRays, HitListObj);
#endif /* DEBUG_RAY_SHOOTING */

    /* Free memory. */
    IritFree(DistList);
    IritFree(AllIntersPts);
    IritFree(SortedInterPts);
    IritFree((IritPrsrPolygonStruct *)AllInterPls);

    return FindThickness;
}

/*****************************************************************************
* DESCRIPTION:								     *
*   Routine to compare two reals for sorting purposes.			     *
*									     *
* PARAMETERS:								     *
*   VPt1, VPt2:	 Two pointers to the real values.			     *
*									     *
* RETURN VALUE:								     *
*   int:  1, 0, or -1 as the relation between the two real values.	     *
*****************************************************************************/
static int IritUserWTSortDist(const VoidPtr D1, const VoidPtr D2) 
{	
    if (IRIT_APX_EQ(*(const IrtRType *) D1, *(const IrtRType *) D2))
	return 0;
    else if (*(const IrtRType *) D1 > *(const IrtRType *) D2)
	return 1;

    return -1;
}

/*****************************************************************************
* DESCRIPTION:                                                               *
*   For a single vertex and a single ray direction, attempts to measure      *
* local wall thickness. If IsUseBVH is TRUE, intersection depths are queried *
* via a polygonal BVH; otherwise a z-depth frame is rendered and parsed to   *
* recover hit depths. When a valid thickness is found, it is written to      *
* LocalThickness and the function returns TRUE.                              *
*                                                                            *
* PARAMETERS:                                                                *
*   PObj:            IN,  input polygonal object.                            *
*   PolyBVH:         IN,  BVH built from PObj (used only if IsUseBVH).       *
*   V:               IN,  vertex being tested.                               *
*   ViewDir:         IN,  unit ray direction to cast from V.                 *
*   VrtxIdx:         IN,  index of V in VrtxArr.                             *
*   RayIdx:          IN,  index of the current ray for this vertex.          *
*   ToModelCtrDir:   IN,  direction from vertex to model center (for up      *
*                         vector construction in depth frame path).          *
*   LocalThickness:  OUT, on success, the measured local thickness.          *
*   ShootRays:       OUT, optional container for visualization of rays for   *
*                         debugging. Can be NULL if not needed.              *
*   Params:          IN,  the parameters for wall thickness, accumulated     *
*			  depth and gap detection analysis.		     *
*                                                                            *
* RETURN VALUE:                                                              *
*   IrtBType: TRUE if a valid thickness was found, FALSE otherwise.          *
*****************************************************************************/
static IrtBType IritUserWTGetHits(
				const IritPrsrObjectStruct *PObj,
				const struct IritGeomPolyBVHStruct *PolyBVH,
				const IritPrsrVertexStruct *V,
				const IrtVecType ViewDir,
				const int VrtxIdx,
				const int RayIdx,
				const IrtVecType ToModelCtrDir,
				IrtRType *LocalThickness,
    				IritPrsrObjectStruct *ShootRays,
				const IritUserWallThicknessParamStruct *Params)
{
    IrtBType 
	FindThickness = FALSE;
    IrtRType CurThickness;

    if (Params -> UseBVH) {
	FindThickness = IritUserWTGetHitsFromBVH(PObj, PolyBVH, V, VrtxIdx, 
						 RayIdx, ViewDir,
						 &CurThickness, 
						 ShootRays, Params);
    }
    else {
	IrtPtType Pos;
	IrtVecType UpDir, TmpDir, TmpAxis;
	IritPrsrObjectStruct *ZDepthFrame;

	IRIT_VEC_COPY(TmpDir, ToModelCtrDir);	/* Initialize Up direction. */
	IRIT_CROSS_PROD(UpDir, TmpDir, ViewDir);

	if (IRIT_VEC_LENGTH(UpDir) < IRIT_EPS) { /* Handle degenerate case. */
	    if (IRIT_FABS(ViewDir[2]) < 0.9) {
		IRIT_VEC_SET(TmpAxis, 0.0, 0.0, 1.0);
	    }
	    else {
		IRIT_VEC_SET(TmpAxis, 0.0, 1.0, 0.0);
	    }

	    IRIT_CROSS_PROD(UpDir, TmpAxis, ViewDir);

	    if (IRIT_VEC_LENGTH(UpDir) < IRIT_EPS) {
		IRIT_VEC_SET(TmpAxis, 1.0, 0.0, 0.0);
		IRIT_CROSS_PROD(UpDir, TmpAxis, ViewDir);
	    }
	}

	IRIT_VEC_NORMALIZE(UpDir);
	IRIT_VEC_COPY(Pos, V -> Coord);

	ZDepthFrame = IritUserWTGetHitsFromDepthFrame(PObj, Pos, ViewDir, 
						      UpDir);

	/* Update vertex thickness based on the hit lists. */
	if (ZDepthFrame != NULL &&
	    IritUserWTUpdateVertexThickness(V, ViewDir, ZDepthFrame, Params,
					    &CurThickness)) {
	    FindThickness = TRUE;
	}

	/* Free memory. */
	IritPrsrFreeObject(ZDepthFrame);
    }

    if (FindThickness)
	*LocalThickness = CurThickness;
    else if (Params -> AnalysisMode == IRIT_USER_WT_GAP_THICKNESS) {   /* No */
	*LocalThickness = Params -> ThicknessMaxTol; /* gap found, therefore */
	FindThickness = TRUE;			    /* mark as safe (green). */
    }
    else
	*LocalThickness = -1.0;
	

    return FindThickness;
}

/*****************************************************************************
* DESCRIPTION:                                                               *
*   Updates the accumulated wall thickness statistics according to the       *
* specified computation mode. Depending on the ThicknessMode, the function   *
* behaves as follows:                                                        *
*   IRIT_USER_WT_MIN:  Keeps the minimum thickness among all hits.           *
*   IRIT_USER_WT_AVG:  Accumulates the sum and increases the hit count for   *
*                      later averaging.                                      *
*   IRIT_USER_WT_MAX:  Keeps the maximum thickness among all hits.           *
*                                                                            *
* PARAMETERS:                                                                *
*   ThicknessMode:  IN,  options for computing thickness.                    *
*   LocalThickness: IN,  thickness value from the current ray intersection.  *
*   HitCnt:         IN/OUT, number of thickness samples accumulated so far.  *
*   ThickSum:       IN/OUT, accumulated thickness value.                     *
*                                                                            *
* RETURN VALUE:                                                              *
*   IrtBType: TRUE if the thickness is updated, FALSE otherwise.             *
*****************************************************************************/
static IrtBType IritUserWTUpdateThickness(const int ThicknessMode,
					  const IrtRType LocalThickness,
					  int *HitCnt,
					  IrtRType *ThickSum)
{
    IrtRType
	TmpThickSum = *ThickSum;

#ifdef DEBUG
    if (LocalThickness > 1E+5)
	fprintf(stderr, "Warning: Local thickness %.3f is too large!\n",
		LocalThickness);
#endif /* DEBUG */

    switch (ThicknessMode / 10) {
    case IRIT_USER_WT_MIN:
	*HitCnt = 1;
	*ThickSum = IRIT_MIN(*ThickSum, LocalThickness);
	return *ThickSum < TmpThickSum;
    case IRIT_USER_WT_AVG:
	(*HitCnt)++;
	*ThickSum += LocalThickness;
	return TRUE;
    case IRIT_USER_WT_MAX:
	*HitCnt = 1;
	*ThickSum = IRIT_MAX(*ThickSum, LocalThickness);
	return *ThickSum > TmpThickSum;
    default:
	break;
    }
    return FALSE;
}

/*****************************************************************************
* DESCRIPTION:                                                               *
*   Generates a direction vector (ViewDir) inside a square pyramid centered  *
* on the given normal direction(NormDir). A 2D grid on the image plane is    *
* mapped to directions, providing uniform set of rays within the specified   *
* angular tolerance.                                                         *
*                                                                            *
*                  *                                                         *
*               *     *                                                      *
*             *         *                                                    *
*             \    ^ NormDir                                                 *
*              \   |   /                                                     *
*               \  |  /                                                      *
*                \ | /                                                       *
*   ViewAngleTol  \|/  ViewAngleTol                                          *
*            ------*-------                                                  *
*                                                                            *
* PARAMETERS:                                                                *
*   NormDir:      IN,  the normalized cone axis direction.                   *
*   ViewIdx:      IN,  index of the sample direction.                        *
*   ViewVecLen:   IN,  total number of view directions.                      *
*   ViewAngleTol: IN,  half-angle (in degrees) of the angular region         *
*                      excluded from ray emission around the reference       *
*                      halfspace defined by the local surface normal.        *
*   ViewDir:      OUT, resulting unit direction vector inside the cone.      *
*                                                                            *
* RETURN VALUE:                                                              *
*   void                                                                     *
*****************************************************************************/
static void IritUserWTGenDirFromNormalPyramid(const IrtVecType NormDir,
					      const int ViewIdx,
					      const int ViewVecLen,
					      const IrtRType ViewAngleTol,
					      IrtVecType ViewDir)
{
    int NSide, UIdx, VIdx;
    IrtRType HalfSize, XVal, YVal;
    IrtVecType u, v, HelperAxis;

    /* Build orthonormal basis (u, v, NormDir) with HelperAxis. */
    if (IRIT_ABS(NormDir[0]) > 0.95) {
	    IRIT_VEC_SET(HelperAxis, 0, 1, 0);
    }
    else {
	IRIT_VEC_SET(HelperAxis, 1, 0, 0);
    }
	
    IRIT_CROSS_PROD(u, HelperAxis, NormDir);
    IRIT_VEC_NORMALIZE(u);
    IRIT_CROSS_PROD(v, NormDir, u);

    /* Decide grid resolution on the square and get HalfSize with angle tol.*/
    NSide = (int) ceil(sqrt(ViewVecLen));
    UIdx = ViewIdx % NSide;
    VIdx = ViewIdx / NSide;

    HalfSize = tan(IRIT_DEG2RAD(90.0 - ViewAngleTol)) / sqrt(2.0);


    /* Map (ix, iy) to [-halfSize, halfSize]^2. */
    XVal = -HalfSize + 
	(2.0 * HalfSize) * ((IrtRType) UIdx + 0.5) / (IrtRType) NSide;
    YVal = -HalfSize + 
	(2.0 * HalfSize) * ((IrtRType) VIdx + 0.5) / (IrtRType) NSide;

    /* Local direction = NormDir + XVal * u + YVal * v, then normalize. */
    IRIT_VEC_SCALE(u, XVal);
    IRIT_VEC_SCALE(v, YVal);
    IRIT_VEC_ADD(ViewDir, NormDir, u)
    IRIT_VEC_ADD(ViewDir, ViewDir, v)
    IRIT_VEC_NORMALIZE(ViewDir)
}

/*****************************************************************************
* DESCRIPTION:                                                               *
*   Iterates the 1-ring polygons that share the given vertex index (VIdx)    *
* and collects each polygon's centroid and normal.                           *
*                                                                            *
* PARAMETERS:                                                                *
*   PVIdx:            IN, polygonal mesh index structure.                    *
*   VIdx:             IN, index of the source vertex.                        *
*   FaceCenters:      OUT, head of a list for 1-ring neighbor's centroids.   * 
*   FaceNormals:      OUT, head of a list for 1-ring neighbor's normals.     *
*   NeighborFaceNum:  OUT, number of 1-ring neighbor faces collected.        *
*                                                                            *
* RETURN VALUE:                                                              *
*   int:           TRUE if at least one neighbor face was collected,         *
*                  FALSE otherwise.                                          *
*****************************************************************************/
static int IritUserWTGetNeighborFaceCentersAndNormals(
				     const IritPrsrPolyVrtxArrayStruct *PVIdx,
				     int VIdx,
				     CagdVecStruct *FaceCenters,
				     CagdVecStruct *FaceNormals, 
				     int *NeighborFaceNum)
{
    int Count = 0;
    const IritPrsrPolyRefListStruct *Pls;
    CagdVecStruct 
	*CurCntr = FaceCenters,
	*CurNrml = FaceNormals;

    for (Pls = PVIdx -> PPolys[VIdx]; Pls != NULL; Pls = Pls -> Pnext) {
	IritGeomPolyCentroid(Pls -> Poly, CurCntr -> Vec);
	IRIT_VEC_COPY(CurNrml -> Vec, Pls-> Poly -> Plane);

	CurCntr -> Pnext = IritCagdVecNew();
	CurCntr = CurCntr -> Pnext;
	CurNrml -> Pnext = IritCagdVecNew();
	CurNrml = CurNrml -> Pnext;

	Count++;
    }

    *NeighborFaceNum = Count;
    return Count > 0;
}

/*****************************************************************************
* DESCRIPTION:                                                               *
*   Computes the interior corner angle (in degrees) at a given vertex within *
* a triangular polygon. The vertex is identified by approximate coordinate   *
* comparison. If the vertex is found in the polygon, the angle between the   *
* two incident edges at that vertex is computed and returned.                *
*                                                                            *
* PARAMETERS:                                                                *
*   CurPl:    IN,  triangular polygon to be queried.                         *
*   V:        IN,  vertex whose corner angle is to be computed.              *
*   AngleDeg: OUT, corner angle at V in degrees (valid only on success).     *
*                                                                            *
* RETURN VALUE:                                                              *
*   IrtBType: TRUE if the vertex V is found in CurPl and the angle is        *
*             successfully computed, FALSE otherwise.                        *
*****************************************************************************/
static IrtBType IritUserWTGetTriCornerAngleDegAtV(
					   const IritPrsrPolygonStruct *CurPl,
					   const IritPrsrVertexStruct *V,
					   IrtRType *AngleDeg)
{
    const IritPrsrVertexStruct *Start, *Vrtx;
    int i;
    IrtVecType VArr[3], e0, e1;
    IrtRType Rad;

    if (CurPl == NULL || CurPl -> PVertex == NULL || V == NULL || 
	AngleDeg == NULL)
	return FALSE;

    Start = CurPl -> PVertex;
    Vrtx = Start;

    do {
	if (IRIT_PT_APX_EQ(Vrtx -> Coord, V -> Coord)) {
	    for (i = 0; i < 3; i++) {
		IRIT_VEC_COPY(VArr[i], Vrtx -> Coord);

		Vrtx = Vrtx -> Pnext;
		if (Vrtx == NULL)
		    Vrtx = Start;
	    }

	    IRIT_VEC_SUB(e0, VArr[1], VArr[0]);
	    IRIT_VEC_SUB(e1, VArr[2], VArr[0]);

	    Rad = IritGeomVecVecAngle(e0, e1, TRUE);
	    *AngleDeg = IRIT_ABS(IRIT_RAD2DEG(Rad));
	    return TRUE;
	}

	Vrtx = Vrtx -> Pnext;
    }
    while (Vrtx != NULL && Vrtx != Start);

    return FALSE;
}

/*****************************************************************************
* DESCRIPTION:                                                               *
*   Computes a vertex normal by averaging the normals of all polygons        *
* adjacent to the given vertex, weighted by their angles on the vertex. The  *
* resulting angle-weighted normal is normalized and returned in NormDir.     *
*                                                                            *
* PARAMETERS:                                                                *
*   PObj:     IN,  input polygonal object.                                   *
*   V:        IN,  vertex whose normal is to be computed.                    *
*   NormDir:  OUT, output vector to store the computed (normalized) normal.  *
*                                                                            *
* RETURN VALUE:                                                              *
*   void                                                                     *
*****************************************************************************/
static void IritUserWTCalcNrmlWithAdjPolysDegree(
					     const IritPrsrObjectStruct *PObj,
					     const IritPrsrVertexStruct *V,
					     IrtVecType NormDir)
{
    IritPrsrPolygonStruct *CurPl;
    IrtRType SumDegree = 0.0;

    IRIT_VEC_SET(NormDir, 0, 0, 0);

    if (PObj == NULL || V == NULL)
	return;

    /* Accumulate total corner angle around V. */
    for (CurPl = PObj -> U.Pl; CurPl != NULL; CurPl = CurPl -> Pnext) {
	IrtRType AngleDeg;

	if (IritUserWTGetTriCornerAngleDegAtV(CurPl, V, &AngleDeg)) {
	    SumDegree += AngleDeg;
	}
    }

    
    if (SumDegree <= IRIT_EPS)		  /* Guard against degenerate cases */ 
	return;			      /*(no incident polys or zero angles). */

    /* Accumulate weighted polygon normals. */
    for (CurPl = PObj -> U.Pl; CurPl != NULL; CurPl = CurPl -> Pnext) {
	IrtRType AngleDeg, t;

	if (IritUserWTGetTriCornerAngleDegAtV(CurPl, V, &AngleDeg)) {
	    t = AngleDeg / SumDegree;
	    IRIT_PT_SCALE_AND_ADD(NormDir, NormDir, CurPl -> Plane, t);
	}
    }

    IRIT_VEC_NORMALIZE(NormDir);
}



/*****************************************************************************
* DESCRIPTION:                                                               *
*   Generates a polyline object representing a wall thickness for            *
* visualization. The line length corresponds to the given thickness value,   *
* and is color-coded according to the thickness mode and the given           *
* tolerances.                                                                *
*                                                                            *
* PARAMETERS:                                                                *
*   RayPt:           IN, ray origin.                                         *
*   RayDir:          IN, ray direction.                                      *
*   LocalThickness:  IN, thickness value used to scale the line length.      *
*   Params:          IN, the parameters for wall thickness, accumulated      *
*			 depth and gap detection analysis.		     *
*                                                                            *
* RETURN VALUE:                                                              *
*   IritPrsrObjectStruct *  A newly allocated polyline object representing   *
*                           the thickness.                                   * 
*****************************************************************************/
static IritPrsrObjectStruct *IritUserWTGenRayObj(
				IrtPtType RayPt,
				const IrtVecType RayDir,
				const IrtRType LocalThickness,
				const IritUserWallThicknessParamStruct *Params)
{
    IrtPtType EndPt;
    IrtVecType TmpRayDir;
    IritPrsrObjectStruct *RayObj;

    /* Calculate the ray's endpoint with the thickness value. */
    IRIT_VEC_COPY_AND_NORMALIZE(TmpRayDir, RayDir)
    IRIT_VEC_SCALE(TmpRayDir, Params -> AccumDepthRayScl);
    IRIT_VEC_SCALE(TmpRayDir, LocalThickness);
    IRIT_PT_ADD(EndPt, RayPt, TmpRayDir);

    /* Generate polyline object and set colors with the thickness. */
    RayObj = IritPrsrGenPOLYLINEObject(IritGeomGenPolyline2Vrtx(RayPt, EndPt, 
								NULL));

    IritUserWTSetAttrColorWithThickness(&RayObj -> Attr, LocalThickness, TRUE,
					Params);

    return RayObj;
}

/*****************************************************************************
* DESCRIPTION:                                                               *
*   Computes the maximum angular difference between face normals among all   *
* polygons in the 1-ring neighborhood. The function iterates over all pairs  *
* of incident face normals and returns the largest angle (in degrees)        *
* between any two normals.                                                   *
*                                                                            *
* PARAMETERS:                                                                *
*   FaceNrmls:  IN,  list of face normal vectors in the 1-ring neighborhood. *
*   FaceNum:    IN,  number of incident faces.                               *
*                                                                            *
* RETURN VALUE:                                                              *
*   IrtRType:  maximum angle (in degrees) between face normal vectors.       *
*****************************************************************************/
static IrtRType IritUserGetMaxFaceNormalAngle(CagdVecStruct *FaceNrmls, 
					      int FaceNum)
{
    int i;
    IrtRType 
	MaxAngle = 0.0;
    CagdVecStruct
	*CurNrml = FaceNrmls;

    for (i = 0; i < FaceNum - 1; i++) {
	int j;
	CagdVecStruct
	    *NextNrml = CurNrml -> Pnext;

	for (j = i + 1; j < FaceNum; j++) {
	    IrtRType
		CurAngle = IritGeomVecVecAngle(CurNrml -> Vec, 
					       NextNrml -> Vec, TRUE);

	    MaxAngle = IRIT_MAX(MaxAngle, IRIT_RAD2DEG(CurAngle));
	    NextNrml = NextNrml -> Pnext;
	}

	CurNrml = CurNrml -> Pnext;
    }

    return MaxAngle;
}

/*****************************************************************************
* DESCRIPTION:                                                               *
*   Assigns a color to a vertex based on its computed wall thickness value.  *
* Depending on the ThicknessMode, the vertex is colored either using a       *
* simple binary scheme (red/green) or an interpolated color map over a       *
* specified thickness range.                                                 *
*                                                                            *
* PARAMETERS:                                                                *
*   Attr:            Pointer to the vertex attribute pointer, to which the   *
*                    RGB color attribute is written.                         *
*   ThicknessVal:    Computed wall thickness value at the vertex.            *
*   ReverseColormap: Flag indicating whether to use reverse colormap.        *
*   Params:          the parameters for wall thickness, accumulated depth    *
*		     and gap detection analysis.			     *
*                                                                            *
* RETURN VALUE:                                                              *
*   void                                                                     *
*****************************************************************************/
static void IritUserWTSetAttrColorWithThickness(
				IritMiscAttributeStruct **Attr,
				const IrtRType ThicknessVal,
				const IrtBType ReverseColormap,
				const IritUserWallThicknessParamStruct *Params)
{
    if (Params -> ThicknessMode % 10) {	       /* Use simple red and green. */
	if (ReverseColormap) {
	    if (ThicknessVal > Params -> ThicknessMaxTol)
		IritMiscAttrIDSetRGBColor(Attr, 255, 0, 0);
	    else
		IritMiscAttrIDSetRGBColor(Attr, 0, 255, 0);
	}
	else {				  
	    if (ThicknessVal < Params -> ThicknessMinTol)
		IritMiscAttrIDSetRGBColor(Attr, 255, 0, 0);
	    else
		IritMiscAttrIDSetRGBColor(Attr, 0, 255, 0);
	}
    }
    else {					 /* Use interpolated color. */
	int r, g, b;
	if (ReverseColormap)
	    IritUserWTGetColor(ThicknessVal, Params -> ThicknessMaxTol,
			       Params -> ThicknessMinTol, &r, &g, &b);
	else
	    IritUserWTGetColor(ThicknessVal, Params -> ThicknessMinTol,
			       Params -> ThicknessMaxTol, &r, &g, &b);

	IritMiscAttrIDSetRGBColor(Attr, r, g, b);
    }
}
