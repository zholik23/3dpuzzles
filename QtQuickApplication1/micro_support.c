/******************************************************************************
* micro_support.c - Implementation of AM support using microstructures.	      *
*******************************************************************************
* (C) Gershon Elber, Technion, Israel Institute	of Technology		      *
*******************************************************************************
* Written by Gershon Elber, Sep. 2023.					      *
******************************************************************************/

#include "inc_irit/irit_sm.h"
#include "inc_irit/iritprsr.h"
#include "inc_irit/allocate.h"
#include "inc_irit/attribut.h"
#include "inc_irit/ip_cnvrt.h"
#include "inc_irit/misc_lib.h"
#include "user_loc.h"

#define USER_MICRO_SUPPORT_CNTCT_PERT_AMNT  0.10301060
#define USER_MICRO_SUPPORT_CNTCT_PERT_AMNT_TINY  0.00010301060

#define USER_MICRO_SUPPORT_NO_UP_RAY(TileTopo) \
    (IRIT_PT_APX_EQ((TileTopo).Tile4RayUpPos[0], VecNone) && \
     IRIT_PT_APX_EQ((TileTopo).Tile4RayUpPos[1], VecNone) && \
     IRIT_PT_APX_EQ((TileTopo).Tile4RayUpPos[2], VecNone) && \
     IRIT_PT_APX_EQ((TileTopo).Tile4RayUpPos[3], VecNone))
#define USER_MICRO_SUPPORT_4UP_RAY(TileTopo) \
    !(IRIT_PT_APX_EQ((TileTopo).Tile4RayUpPos[0], VecNone) || \
      IRIT_PT_APX_EQ((TileTopo).Tile4RayUpPos[1], VecNone) || \
      IRIT_PT_APX_EQ((TileTopo).Tile4RayUpPos[2], VecNone) || \
      IRIT_PT_APX_EQ((TileTopo).Tile4RayUpPos[3], VecNone))
#define USER_MICRO_SUPPORT_NO_DWN_RAY(TileTopo) \
    (IRIT_PT_APX_EQ((TileTopo).Tile4RayDwnPos[0], VecNone) && \
     IRIT_PT_APX_EQ((TileTopo).Tile4RayDwnPos[1], VecNone) && \
     IRIT_PT_APX_EQ((TileTopo).Tile4RayDwnPos[2], VecNone) && \
     IRIT_PT_APX_EQ((TileTopo).Tile4RayDwnPos[3], VecNone))
#define USER_MICRO_SUPPORT_4DWN_RAY(TileTopo) \
    !(IRIT_PT_APX_EQ((TileTopo).Tile4RayDwnPos[0], VecNone) || \
      IRIT_PT_APX_EQ((TileTopo).Tile4RayDwnPos[1], VecNone) || \
      IRIT_PT_APX_EQ((TileTopo).Tile4RayDwnPos[2], VecNone) || \
      IRIT_PT_APX_EQ((TileTopo).Tile4RayDwnPos[3], VecNone))

IRIT_STATIC_DATA const IrtPtType
    VecNone = { IRIT_INFNTY, IRIT_INFNTY, IRIT_INFNTY };

static void UserMicroFreeAMSprtTopology(UserMicroSupportTopoInfoStruct
					                           ***MSTopo,
					int SizeX,
					int SizeY);
static UserMicroSupportTopoInfoStruct ***UserMicroGenAMSprtTopology(
			   IritPrsrObjectStruct *MS,
			   int MSTopoSizes[3],
			   const IritUserMicroGenAMSupportParamStruct *Params);
static int UserMicroGenPrmThicknessInTopology(
			  const int *MSTopoSizes,
			  UserMicroSupportTopoInfoStruct ***MSTopology,
			  const IritUserMicroGenAMSupportParamStruct *Params);
static void UserMicroShootUpDownClosestInter(
				      const IrtPtType RayOrigin,
				      IrtPtType ClosestInterPos,
				      IrtVecType ClosestInterNrml,
				      const IrtPtType *AllIntersPts,
				      const IritPrsrPolygonStruct **AllIntersPls,
				      int IntersNum);

static int UserMicroShootRay(
			   const IritUserMicroGenAMSupportParamStruct *Params,
			   const IrtPtType RayPt, 
			   const IrtVecType RayDir, 
			   const struct IritGeomPolyBVHStruct *PolyBVH,
			   const IritPrsrObjectStruct *PlObj,
			   IrtPtType ClosestInterPos,
			   IrtVecType ClosestInterNrml);
static int UserMicroTestRay(const IritUserMicroGenAMSupportParamStruct *Params,
			    const UserMicroSupportTopoInfoStruct *TlTopo,
			    const struct IritGeomPolyBVHStruct *PolyBVH,
			    const IrtVecType RayPt,
			    IrtPtType InterPt,
			    IrtVecType InterNrml,
			    int IsUpRay,
			    int RayIndex,
			    IrtPtType ClosestInterPos,
			    IrtVecType ClosestInterNrml,
			    int IntersectNum);
static int UserMicroShootRays(
			    const IritUserMicroGenAMSupportParamStruct *Params,
			    UserMicroSupportTopoInfoStruct *TlTopo,
			    int RayIndex,
			    const IrtVecType RayPt, 
			    const IrtVecType RayDir, 
			    const struct IritGeomPolyBVHStruct *PolyBVH,
			    const IritPrsrObjectStruct *PlObj,
			    IrtPtType ClosestInterPos,
			    IrtVecType ClosestInterNrml);
static void UserMicroShootUpDownRays(
			   const IritUserMicroGenAMSupportParamStruct *Params,
			   const IritPrsrObjectStruct *PlObj,
			   struct IritGeomPolyBVHStruct *PolyBVH,
			   const IritPrsrObjectStruct *MS,
			   UserMicroSupportTopoInfoStruct ***MSTopology,
			   const int MSTopoSizes[3]);
static void UserMicroPurgeInactiveArms(IritPrsrObjectStruct *Tile,
				       const int PurgeArms[8]);
static void UserMicroFilterCheckNeighborArmsAux(
				const int *MSTopoSizes,
				UserMicroSupportTopoInfoStruct ***MSTopology);
static int UserMicroFilterCheckNeighborArms(
				IritPrsrObjectStruct *Tile,
				int PurgeArms[8],
				UserMicroSupportTopoInfoStruct ***MSTopology);
static IritPrsrObjectStruct *UserMicroFilterInactiveTilesAndArms(
			       IritPrsrObjectStruct *MS,
			       const int *MSTopoSizes,
			       UserMicroSupportTopoInfoStruct ***MSTopology);
static TrivTVStruct *UserMicroCnvrtRayToTV(
			   const IrtPtType RayPt,
			   const IrtPtType InterPt,
			   const IrtVecType InterNrml,
			   IrtRType BaseDim[2],
			   IrtRType TipScale,
			   IrtRType NormalTipLen,
			   IrtRType SclSlopeDist,
			   IrtBType IsUpRay,
			   const IritUserMicroGenAMSupportParamStruct *Params);
static TrivTVStruct *UserMicroCnvrtRayToTV2(
			   const UserMicroSupportTopoInfoStruct *TileTopo,
			   const IritUserMicroGenAMSupportParamStruct *Params,
			   IrtPtType InterPt,
			   IrtPtType InterNrml,
			   IrtBType IsUpRay,
			   int RayIdx);
static IritPrsrObjectStruct *UserMicroGenRaySpikes(
			   IritPrsrObjectStruct *MS,
			   const int *MSTopoSizes,
			   UserMicroSupportTopoInfoStruct ***MSTopology,
			   const IritUserMicroGenAMSupportParamStruct *Params);

/* Functions for merging tiles to generate multi-resolution tiles. */
static int UserMicroMultiResGetCornerArms(
				 UserMicroSupportTopoInfoStruct ***MSTopology,
				 int XIdx, 
				 int YIdx, 
				 int ZIdx,
				 int Stride,
				 int PurgeArms[8]);
static int UserMicroMultiResCheckNeighborsBeforeMerge(
				 const int *MSTopoSizes,
				 UserMicroSupportTopoInfoStruct ***MSTopology,
				 int XIdx, 
				 int YIdx, 
				 int ZIdx, 
				 int Stride);
static int UserMicroCountMergingTiles(
				 const int *MSTopoSizes,
				 UserMicroSupportTopoInfoStruct ***MSTopology,
				 int Stride);
IritPrsrObjectStruct *UserMicroGenOneMergedTile(
				 const int *MSTopoSizes,
				 UserMicroSupportTopoInfoStruct ***MSTopology,
				 int XIdx,
				 int YIdx,
				 int ZIdx,
				 int Stride,
				 CagdRType TileCntrThickness,
				 const CagdRType TileEndEdgeThickness[10],
				 CagdRType TileBaseScale);
static IritPrsrObjectStruct *UserMicroGenMergedTiles(
			   const int *MSTopoSizes,
			   UserMicroSupportTopoInfoStruct ***MSTopology,
			   int Stride,
			   const TrivTVStruct *BBoxTV,
			   const IritUserMicroGenAMSupportParamStruct *Params);
static IritPrsrObjectStruct *UserMicroGenMergedPrmTiles(
			  const int *MSTopoSizes,
			  UserMicroSupportTopoInfoStruct ***MSTopology,
			  int Stride,
			  const TrivTVStruct *BBoxTV,
			  const IritUserMicroGenAMSupportParamStruct *Params);
static void UserMicroMultiResFilterActiveArms(
				 const int *MSTopoSizes,
				 UserMicroSupportTopoInfoStruct ***MSTopology,
				 int XIdx,
				 int YIdx,
				 int ZIdx);
static int UserMicroMultiResFilterDanglingArms(
				 const int *MSTopoSizes,
				 UserMicroSupportTopoInfoStruct ***MSTopology,
				 int XIdx, 
				 int YIdx, 
				 int ZIdx);
static int UserMicroMultiResCheckRaysOnTile(
				 UserMicroSupportTopoInfoStruct ***MSTopology,
				 int XIdx, 
				 int YIdx, 
				 int ZIdx, 
				 int UpDwnRay,
				 int RaysOnArm[8]);
static void UserMicroMultiResFilterInactiveTilesAndArms(
			        const int *MSTopoSizes,
			        UserMicroSupportTopoInfoStruct ***MSTopology);
static void UserMicroMultiResMoveActiveArmsInfo(
				 const int *MSTopoSizes, 
				 UserMicroSupportTopoInfoStruct ***MSTopology);
static IritPrsrObjectStruct *UserMicroRemoveInvalidTiles(
						    IritPrsrObjectStruct *MS);
static IritPrsrObjectStruct *UserMicroMultiResolutionTiling(
			  IritPrsrObjectStruct *MS,
			  const int *MSTopoSizes,
			  UserMicroSupportTopoInfoStruct ***MSTopology,
			  const TrivTVStruct *BBoxTV,
			  const IritUserMicroGenAMSupportParamStruct *Params);
static IritPrsrObjectStruct *UserMicroThickenBaseTileArms(
			  IritPrsrObjectStruct *MS,
			  const int *MSTopoSizes,
			  UserMicroSupportTopoInfoStruct ***MSTopology,
			  const TrivTVStruct *BBoxTV,
			  const IritUserMicroGenAMSupportParamStruct *Params);
static IritPrsrObjectStruct *UserMicroThickenBaseTilePrmArms(
			  IritPrsrObjectStruct *MS,
			  const int *MSTopoSizes,
			  UserMicroSupportTopoInfoStruct ***MSTopology,
			  const TrivTVStruct *BBoxTV,
			  const IritUserMicroGenAMSupportParamStruct *Params);
static int IritUserMicroMarkSupportMSInfo(IritPrsrObjectStruct *MS,
					  int TileType,
					  int TileLvl);
static IritPrsrObjectStruct *UserMicroGenClosestRaySpikes(
			   UserMicroSupportTopoInfoStruct *TlTopo,
			   const struct IritGeomPolyBVHStruct *PolyBVH,
			   const IritUserMicroGenAMSupportParamStruct *Params);
static IritPrsrObjectStruct *UserMicroFilterSpikesWithExistedTilesAndSpikes(
				IritPrsrObjectStruct *MS,
				UserMicroSupportTopoInfoStruct ***MSTopology,
				IritPrsrObjectStruct *ExtSpikes,
				IritPrsrObjectStruct *NewSpikes);
static IritPrsrObjectStruct *UserMicroReplaceSpikesToClosestSpikes(
			   IritPrsrObjectStruct *MS,
			   const int *MSTopoSizes,
			   UserMicroSupportTopoInfoStruct ***MSTopology,
			   const struct IritGeomPolyBVHStruct *PolyBVH,
			   const IritUserMicroGenAMSupportParamStruct *Params,
			   IritPrsrObjectStruct *Spikes);

static IritPrsrObjectStruct *UserMicroPrmSupportTilePreProcess(
				     IritPrsrObjectStruct *Tile,
				     UserMicroPreProcessTileCBStruct *CBData);

/*****************************************************************************
* DESCRIPTION:								     *
*   Free the 3D	grid of	the MS topology.				     *
*									     *
* PARAMETERS:								     *
*   MSTopo:	    The	3D grid	of topology to free.			     *
*   SizeX, SizeY:   The	XY size	of the grid to free.			     *
*									     *
* RETURN VALUE:								     *
*   void								     *
*****************************************************************************/
static void UserMicroFreeAMSprtTopology(UserMicroSupportTopoInfoStruct
					***MSTopo,
					int SizeX,
					int SizeY)
{
    int i, j;

    for (i = 0; i < SizeX; i++) {
	for (j = 0; j < SizeY; j++)
	    IritFree(MSTopo[i][j]);
	IritFree(MSTopo[i]);
    }
    IritFree(MSTopo);
}

/*****************************************************************************
* DESCRIPTION:								     *
*   Figure out what tile exists	and what not, in this 3D grid of tiles.	     *
*   Returned 3D	grid will have slots with 0 if tile does not exists and	1 if *
* exist.								     *
*									     *
* PARAMETERS:								     *
*   MS:		The microstructure to figure out its neighborhood topology.  *
*   MSTopoSizes:  Will be updated with the size	of the created 3D grid.	     *
*   Params:  The support parameters.					     *
*									     *
* RETURN VALUE:								     *
*   int	***:	A 3D grid of TRUE/FALSE	existence values.		     *
*		Allocated dynamically.					     *
*****************************************************************************/
static UserMicroSupportTopoInfoStruct ***UserMicroGenAMSprtTopology(
			   IritPrsrObjectStruct *MS,
			   int	MSTopoSizes[3],
			   const IritUserMicroGenAMSupportParamStruct *Params)
{
    int i, j, k, l, x, y, z, SzX, SzY, SzZ, SizeX, SizeY, SizeZ,
	UsePrmThickness = Params -> PrmThicknessTV != NULL;
    UserMicroSupportTopoInfoStruct ***MSTopo;
    IritPrsrObjectStruct *Tile;
    const char
        *MSIndex = IritMiscAttrIDGetObjectStrAttrib(
					        IritPrsrListObjectGet(MS, 0),
					        IRIT_ATTR_CREATE_ID(MSIndex));

    if (MSIndex == NULL ||
	sscanf(MSIndex, "%d,%d,%d::%d,%d,%d",
	       &x, &y, &z, &SizeX, &SizeY, &SizeZ) != 6) {
        IRIT_WARNING_MSG("Failed to fetch topology information from tiles. Aborted.\n");
	return NULL;
    }

    /* Allocate a 3D grid of size (Size x SizeY x SizeZ): */
    MSTopo = (UserMicroSupportTopoInfoStruct ***)
                IritMalloc(sizeof(UserMicroSupportTopoInfoStruct **) * SizeX);
    for (i = 0; i < SizeX; i++) {
        MSTopo[i] = (UserMicroSupportTopoInfoStruct **)
	         IritMalloc(sizeof(UserMicroSupportTopoInfoStruct *) * SizeY);
	for (j = 0; j < SizeY; j++) {
	    MSTopo[i][j] = (UserMicroSupportTopoInfoStruct *)
	           IritMalloc(sizeof(UserMicroSupportTopoInfoStruct) * SizeZ);
	    IRIT_ZAP_MEM(MSTopo[i][j],
			 sizeof(UserMicroSupportTopoInfoStruct) * SizeZ);
	}
    }

    /* UPdate the tiles in MS in the topology: */
    for (i = 0; (Tile = IritPrsrListObjectGet(MS, i)) != NULL; i++) {
        if ((MSIndex = IritMiscAttrIDGetObjectStrAttrib(Tile,
						IRIT_ATTR_CREATE_ID(MSIndex)))
								    == NULL ||
	    sscanf(MSIndex, "%d,%d,%d::%d,%d,%d",
		   &x, &y, &z, &SzX, &SzY, &SzZ) != 6 ||
	    (SzX != SizeX || SzY != SizeY || SzZ != SizeZ)) {
	    IRIT_WARNING_MSG("Failed to fetch topology information from tiles. Aborted.\n");
	    UserMicroFreeAMSprtTopology(MSTopo, SizeX, SizeY);
	    return NULL;
	}

	MSTopo[x][y][z].Tile = Tile;
    }

    MSTopoSizes[0] = SizeX;
    MSTopoSizes[1] = SizeY;
    MSTopoSizes[2] = SizeZ;

    /* Precompute thickness of each tile at the center and 8 corners. */
    if (UsePrmThickness) {
	if (UserMicroGenPrmThicknessInTopology(MSTopoSizes, MSTopo, Params)
								   == FALSE) {
	    IRIT_WARNING_MSG("Failed to compute parametric arm thickness.\n");
	    UsePrmThickness = FALSE;
	}
    }

    for (i = 0; i < MSTopoSizes[0]; i++) {
	for (j = 0; j < MSTopoSizes[1]; j++) {
	    for (k = 0; k < MSTopoSizes[2]; k++) {
	        UserMicroSupportTopoInfoStruct
		    *TlTopo = &MSTopo[i][j][k];	        
		Tile = TlTopo -> Tile;

		for (l = 0; l < 4; l++) {         /* Init all vecs to None. */
		    IRIT_PT_COPY(TlTopo -> Tile4TopPos[l], VecNone);
		    IRIT_PT_COPY(TlTopo -> Tile4BotPos[l], VecNone);
		    IRIT_PT_COPY(TlTopo -> Tile4RayUpPos[l], VecNone);
		    IRIT_PT_COPY(TlTopo -> Tile4RayDwnPos[l], VecNone);
		    IRIT_PT_COPY(TlTopo -> Tile4RayUpNrml[l], VecNone);
		    IRIT_PT_COPY(TlTopo -> Tile4RayDwnNrml[l], VecNone);
		}

		if (Tile != NULL) { /* Mark top/bottom-tiles - tiles with no */
		    GMBBBboxStruct *BBox;      /* other tile above/below it. */

		    IritGeomBBComputeBboxObject(TlTopo -> Tile,
						&TlTopo -> BBox,
						TRUE);
		    BBox = &TlTopo -> BBox;

		    if ((k < MSTopoSizes[2] - 1 &&
			 MSTopo[i][j][k + 1].Tile == NULL) ||
			k == MSTopoSizes[2] - 1)
		        TlTopo -> TopTile = TRUE;
		    if ((k > 0 && MSTopo[i][j][k - 1].Tile == NULL) || k == 0)
		        TlTopo -> BotTile = TRUE;

		    if (TlTopo -> TopTile || TlTopo -> BotTile) {
			IrtRType DSizeX, DSizeY, BoxSize[2], JX, JY;			    

			IRIT_PT2D_SUB(BoxSize, BBox -> Max, BBox -> Min);

			if (!UsePrmThickness) {
			    DSizeX = Params -> TileEndEdgeThickness * 0.5 *
								   BoxSize[0];
			    DSizeY = Params -> TileEndEdgeThickness * 0.5 *
								   BoxSize[1];
			    
			    /* Compute locations of top and bot joints of   */
			    /* tile.					    */
			    for (l = 0; l < 4; l++) {
				TlTopo -> Tile4TopPos[l][0] =
				    TlTopo -> Tile4BotPos[l][0] =
					((l & 0x01) ? BBox -> Max[0] - DSizeX
						    : BBox -> Min[0] + DSizeX);
				TlTopo -> Tile4TopPos[l][1] =
				    TlTopo -> Tile4BotPos[l][1] =
					((l >= 2) ? BBox -> Max[1] - DSizeY
						  : BBox -> Min[1] + DSizeY);
				TlTopo -> Tile4BotPos[l][2] = BBox -> Min[2];
				TlTopo -> Tile4TopPos[l][2] = BBox -> Max[2];
			    }
			}
			else {
			    /* Compute locations of top and bot joints of   */
			    /* parametric tile.				    */
			    for (l = 0; l < 8; l++) {
				DSizeX = TlTopo -> PrmCrnrThickness[l] * 0.5 *
								   BoxSize[0];
				DSizeY = TlTopo -> PrmCrnrThickness[l] * 0.5 *
								   BoxSize[1];					 
				JX = (l & 0x01) ? BBox -> Max[0] - DSizeX
						: BBox -> Min[0] + DSizeX;
				JY = (l % 4 >= 2) ? BBox -> Max[1] - DSizeY
						  : BBox -> Min[1] + DSizeY;
				if (l < 4) {		/* For bottom arms. */
				    IRIT_PT_SET(TlTopo -> Tile4BotPos[l],
						JX, JY, BBox -> Min[2]);
				}
				else {			   /* For top arms. */
				    IRIT_PT_SET(TlTopo -> Tile4TopPos[l - 4],
						JX, JY, BBox -> Max[2]);
				}
			    }
			}
		    }

		    /* Mark the stride of each tile as 1. */
		    MSTopo[i][j][k].Stride = 1;
		    
		    /* Mark the level for tiles of the finest resolution. */
		    IritUserMicroMarkSupportMSInfo(Tile, 
						   USER_MICRO_SUPPORT_NONBASE, 
						   0);
		}
		else {
		    MSTopo[i][j][k].Stride = 0;
		}
	    }
	}
    }

#ifdef DEBUG_DUMP_MS_TOPO
    /* print the 3D grid - topology, for valid and top/bottom tiles. */
    for (k = 0; k < MSTopoSizes[2]; k++) {
	for (j = 0; j < MSTopoSizes[1]; j++) {
	    for (i = 0; i < MSTopoSizes[0]; i++) {
		if (MSTopo[i][j][k].Tile != NULL) {
		    if (MSTopo[i][j][k].TopTile) {
			fprintf(stderr, "%1d", 2);
			IritMiscAttrSetObjectRGBColor(MSTopo[i][j][k].Tile,
						      200, 200, 0);
		    }
		    else if (MSTopo[i][j][k].BotTile) {
			fprintf(stderr, "%1d", 3);
			IritMiscAttrSetObjectRGBColor(MSTopo[i][j][k].Tile,
						      0, 200, 200);
		    }
		    else
			fprintf(stderr, "%1d", 1);
		}
		else
		    fprintf(stderr, "%1d", 0);
	    }
	    fprintf(stderr, "\n");
	}
	fprintf(stderr, "\n\n");
    }
#endif /* DEBUG_DUMP_MS_TOPO */

    return MSTopo;
}

/*****************************************************************************
* DESCRIPTION:								     *
*   Precompute arm thickness of tiles and store them in the 3D tile grid.    *
* Arm thickness is evaluated on the scalar trivariate defined in the same    *
* domain of box trivariate bounding the support microstructure, which        *
* usually has the domain of [0,1]^3. For each tile, thickness is sampled at  *
* the center and 8 corners of the tile domain and saved in the 3D tile grid  *
* respectively.								     *
*									     *
* PARAMETERS:								     *
*   MSTopoSizes:  The sizes in XYZ of MSTopology.			     *
*   MSTopology:	  A 3D grid hold the topology of this grid.  Will be updated *
*		  with center and corner thickness.			     *
*   Params:  The support parameters.					     *
*									     *
* RETURN VALUE:								     *
*   int:	TRUE if thickness is computed, and FALSE otherwise.	     *
*****************************************************************************/
static int UserMicroGenPrmThicknessInTopology(
			   const int *MSTopoSizes,
			   UserMicroSupportTopoInfoStruct ***MSTopology,
			   const IritUserMicroGenAMSupportParamStruct *Params)
{
    int i, NumXYTiles, NumAllTiles;
    CagdRType UVWInt[3];
    CagdBBoxStruct PrmTVDmn;
    const TrivTVStruct
	*PrmTV = Params -> PrmThicknessTV;

    if (PrmTV == NULL)
	return FALSE;

    IritTrivTVDomain(PrmTV, &PrmTVDmn.Min[0], &PrmTVDmn.Max[0], 
			    &PrmTVDmn.Min[1], &PrmTVDmn.Max[1], 
			    &PrmTVDmn.Min[2], &PrmTVDmn.Max[2]);

    for (i = 0; i < 3; i++)
	UVWInt[i] = (PrmTVDmn.Max[i] - PrmTVDmn.Min[i]) 
						  / (CagdRType)MSTopoSizes[i];

    NumXYTiles = MSTopoSizes[0] * MSTopoSizes[1];
    NumAllTiles = NumXYTiles * MSTopoSizes[2];

#   pragma omp parallel for private(i) if (_IritParallelExec)
    for (i = 0; i < NumAllTiles; i++) {
	int j, XId, YId, XYId, ZId, UVId;
	CagdRType Pt[3];
	CagdBBoxStruct UVWDmn;
	UserMicroSupportTopoInfoStruct *TlTopo;

	ZId = i / NumXYTiles;
	XYId = i % NumXYTiles;
	YId = XYId / MSTopoSizes[0];
	XId = XYId % MSTopoSizes[0];
	TlTopo = &MSTopology[XId][YId][ZId];

	if (TlTopo -> Tile == NULL)
	    continue;

	CAGD_BBOX_INIT_3D(UVWDmn);
	IRIT_PT_SET(UVWDmn.Min, UVWInt[0] * (CagdRType) XId,
				UVWInt[1] * (CagdRType) YId,
				UVWInt[2] * (CagdRType) ZId);
	IRIT_PT_ADD(UVWDmn.Max, UVWDmn.Min, UVWInt);
	
	/* Compute corner thickness. */
	for (j = 0; j < 8; j++) {
	    UVId = j % 4;
	    Pt[0] = (UVId % 2 == 0) ? UVWDmn.Min[0] : UVWDmn.Max[0];
	    Pt[1] = (UVId / 2 == 0) ? UVWDmn.Min[1] : UVWDmn.Max[1];
	    Pt[2] = (j / 4 == 0) ? UVWDmn.Min[2] : UVWDmn.Max[2];
	    TRIV_TV_EVAL_SCALAR(PrmTV, Pt[0], Pt[1], Pt[2], 
				&TlTopo -> PrmCrnrThickness[j]);
	    if (TlTopo -> PrmCrnrThickness[j] > 0.5 - IRIT_EPS)
		TlTopo -> PrmCrnrThickness[j] = 0.5 - IRIT_EPS;
	}

	/* Compute center thickness. */
	IRIT_PT_BLEND(Pt, UVWDmn.Min, UVWDmn.Max, 0.5);
	TRIV_TV_EVAL_SCALAR(PrmTV, Pt[0], Pt[1], Pt[2], 
			    &TlTopo -> PrmCntrThickness);
	if (TlTopo -> PrmCntrThickness > 0.5 - IRIT_EPS)
	    TlTopo -> PrmCntrThickness = 0.5 - IRIT_EPS;
    }

    return TRUE;
}

/*****************************************************************************
* DESCRIPTION:								     *
*   Fetches the	closest	intersection detected from Ray Origin.		     *
*									     *
* PARAMETERS:								     *
*   RayOrigin:	     The origin	of the shot ray.			     *
*   ClosestInterPos: Will be updated with the closest intersection found.    *
*   AllIntersPts:    A vector of all detected intersection of size IntersNum.*
*   AllIntersPls:    A vector of all detected intersecting polygons, of	size *
*		     IntersNum.						     *
*   IntersNum:	     The size of vectors AllIntersPts and AllIntersPls.	     *
*									     *
* RETURN VALUE:								     *
*   void								     *
*****************************************************************************/
static void UserMicroShootUpDownClosestInter(
				 const IrtPtType RayOrigin,
				 IrtPtType ClosestInterPos,
				 IrtVecType ClosestInterNrml,
				 const IrtPtType *AllIntersPts,
				 const IritPrsrPolygonStruct **AllIntersPls,
				 int IntersNum)
{
    int i;
    IrtRType MinDistSqr, MinSqr;

    IRIT_PT_COPY(ClosestInterPos, AllIntersPts[0]);
    assert(IRIT_PRSR_HAS_PLANE_POLY(AllIntersPls[0]));
    IRIT_PT_COPY(ClosestInterNrml, AllIntersPls[0] -> Plane);
    MinDistSqr = IRIT_PT_PT_DIST_SQR(RayOrigin, AllIntersPts[0]); 

    for (i = 1; i < IntersNum; i++) {
        MinSqr = IRIT_PT_PT_DIST_SQR(RayOrigin, AllIntersPts[i]);
	if (MinDistSqr > MinSqr) {
	    IRIT_PT_COPY(ClosestInterPos, AllIntersPts[i]);
	    assert(IRIT_PRSR_HAS_PLANE_POLY(AllIntersPls[i]));
	    IRIT_PT_COPY(ClosestInterNrml, AllIntersPls[i] -> Plane);
	    MinDistSqr = MinSqr;
	}
    }
}

/*****************************************************************************
* DESCRIPTION:								     *
*   Compute the	closest	intersection point between the input model and the   *
* ray (RayPt, RayDir).							     *
*									     *
* PARAMETERS:								     *
*   Params:	 The support parameters.				     *
*   RayPt:	 A point whether the ray is being shot from.		     *
*   RayDir:	 The direction of the ray.				     *
*   PolyBVH:	 The BVH of the	mesh.					     *
*   PlObj:	 The original input polygonal model.			     *
*   AllInters:	 Array of the ray-PolyMesh intersection	points.	 Allocated   *
*		 by the	caller to be large enough to hold all inters.	     *
*  AllIntersPolys:  Array of the ray-PolyMesh intersection polygons.	     *
*		 Allocated by the caller to be large enough to hold all	pls. *
*   ClosestInterPts:  Will be updated with closest intersection	location,    *
*		 if any.						     *
*   ClosestInterPol:  Will be updated with closest intersection	mesh normal, *
*		 if any.						     *
*									     *
* RETURN VALUE:								     *
*   int:      The number of intersection points.			     *
*****************************************************************************/
static int UserMicroShootRay(
			   const IritUserMicroGenAMSupportParamStruct *Params,
			   const IrtPtType RayPt, 
			   const IrtVecType RayDir, 
			   const struct IritGeomPolyBVHStruct *PolyBVH,
			   const IritPrsrObjectStruct *PlObj,
			   IrtPtType ClosestInterPos,
			   IrtVecType ClosestInterNrml)
{
    IRIT_STATIC_DATA const IrtPtType
	VecNone = { IRIT_INFNTY, IRIT_INFNTY, IRIT_INFNTY };
    IrtBType
        HasInter = FALSE;
    int NumInters,
        PlLen = IritPrsrPolyListLen(PlObj -> U.Pl);
    IrtPtType
	*AllIntersPts = (IrtPtType *) IritMalloc(sizeof(IrtPtType) * PlLen);
    const IritPrsrPolygonStruct
	**AllIntersPls = (const IritPrsrPolygonStruct **)
       			IritMalloc(sizeof(IritPrsrPolygonStruct *) * PlLen);

    /* Ray initial position better be outside the model - verify it. */
    if (IritGeomPolyBVHPointInsidePolys(RayPt, PolyBVH)) {
#	ifdef DEBUG
        IRIT_WARNING_MSG_PRINTF(
	     "Ray pos (%.10lg %.10lg %.10lg) inside model - aborted.\n",
	     RayPt[0], RayPt[1], RayPt[2]);
#	endif /* DEBUG */
	return 0;	
    }

    NumInters = IritGeomPolyBVHGetRayBVHIntersectionPt(
						    RayPt, RayDir, PolyBVH,
						    AllIntersPts, AllIntersPls);

#   ifdef DEBUG_MS_TOPO_BVH_TEST
    {
	/* Compare inters against exhaustive test. */
        int NumInters2 = IritGeomRayCnvxPolygonListInter2(RayPt, RayDir,
							  PlObj -> U.Pl,
							  AllIntersPts,
							  AllIntersPls);

        if (NumInters != NumInters2) {
	    static int
	        Idx = 0;

	    fprintf(stderr, "\tPts[%d][0] = %.15lg;", Idx, RayPt[0]);
	    fprintf(stderr, " Pts[%d][1] = %.15lg;", Idx, RayPt[1]);
	    fprintf(stderr, " Pts[%d][2] = %.15lg;\n", Idx, RayPt[2]);
	    fprintf(stderr, "\tDirs[%d][0] = %.15lg;", Idx, RayDir[0]);
	    fprintf(stderr, " Dirs[%d][1] = %.15lg;", Idx, RayDir[1]);
	    fprintf(stderr, " Dirs[%d][2] = %.15lg;\n", Idx, RayDir[2]);
	    Idx++;

	    NumInters = NumInters2;
	}
    }
#   endif /* DEBUG_MS_TOPO_BVH_TEST */

    if (NumInters > 0) {
        IrtRType Slope;

	HasInter = TRUE;
        UserMicroShootUpDownClosestInter(RayPt,
					 ClosestInterPos, ClosestInterNrml,
					 (const IrtPtType *) AllIntersPts,
					 AllIntersPls, NumInters);
	/* Verify model slope requires this support. */
	Slope = asin(IRIT_ABS(ClosestInterNrml[2]));

	if (IRIT_RAD2DEG(Slope) < Params -> ModelMinSlope)
	    HasInter = FALSE;
    }
    else
	HasInter = FALSE;

    IritFree(AllIntersPts);
    IritFree((IritPrsrPolygonStruct *) AllIntersPls);

    if (!HasInter) {
    	IRIT_PT_COPY(ClosestInterPos, VecNone);
    	IRIT_VEC_COPY(ClosestInterNrml, VecNone);
	return 0;
    }

    return NumInters;
}

/*****************************************************************************
* DESCRIPTION:								     *
*   Use	the ray	and intersection location to determine if the generated	tip  *
* is in	an exception situation.	The following exception	are used:	     *
* 1. The tip is	too short or too long.					     *
* 2. Has negative Jacobian.						     *
* 3. Has intersection with the model.					     *
*									     *
* PARAMETERS:								     *
*   Params:	       The support parameters.				     *
*   TlTopo:	       The topology of the tile.			     *
*   PolyBVH:	       The BVH of the mesh.				     *
*   RayPt:	       Origin of ray.					     *
*   InterPt:	       End/intersecting	location of ray	from RayPt.	     *
*   InterNrml:	       End/intersection	mesh normal at InterPt.		     *
*   IsUpRay:	       TRUE for	up ray,	FALSE for down ray.		     *
*   RayIndex:	       Number between 0	to 3, for the specific arm of the    *
*		       tile to shoot ray from.				     *
*   ClosestInterPts:   Will be updated with closest intersection location,   *
*		       if any.						     *
*   ClosestInterNrml:  Will be updated with closest intersection mesh normal,*
*		       if any.						     *
*   IntersectNum:      The number of intersection points.		     *
*									     *
* RETURN VALUE:								     *
*   int:      The number of intersection points.			     *
*****************************************************************************/
static int UserMicroTestRay(const IritUserMicroGenAMSupportParamStruct *Params,
			    const UserMicroSupportTopoInfoStruct *TlTopo,
			    const struct IritGeomPolyBVHStruct *PolyBVH,
			    const IrtVecType RayPt,
			    IrtPtType InterPt,
			    IrtVecType InterNrml,
			    int	IsUpRay,
			    int	RayIndex,
			    IrtPtType ClosestInterPos,
			    IrtVecType ClosestInterNrml,
			    int	IntersectNum)
{
    IrtRType
	Dst = IRIT_PT_PT_DIST(RayPt, InterPt),
	RayXYLen = sqrt(IRIT_SQR(RayPt[0] - InterPt[0]) +
			IRIT_SQR(RayPt[1] - InterPt[1])),
	Slope = atan2(IRIT_ABS(RayPt[2] - InterPt[2]), RayXYLen);
    TrivTVStruct *SpikeTV, *SpikeCutTV;
    IritPrsrPolygonStruct *SpikePolys;
    CagdSrf2PlsInfoStrct TessInfo;
    IritUserMicroGenAMSupportParamStruct LclParams;

    IritPrsrTSrf2PlysInitTessInfo2(&TessInfo, TRUE, 20, NULL, 
				   FALSE, FALSE, 0, NULL);

    /* Purge too short tip. */
    if (Dst < Params -> TileSize * 0.1) {
	if (!Params -> _ShortTipsError) {
	    ((IritUserMicroGenAMSupportParamStruct *) Params) ->
							_ShortTipsError = TRUE;
	    IRIT_WARNING_MSG("Some tips are too short - consider adjusting TileSize.\n");
	}
	return 0;
    }

    /* Let's see the generated tip has intersection with the model. */
    LclParams = *Params;
    LclParams._BzrOnlyTVs = FALSE;
    SpikeTV = UserMicroCnvrtRayToTV2(TlTopo, &LclParams,
				     InterPt, InterNrml,
				     IsUpRay ? TRUE : FALSE,
				     RayIndex);

    /* Purge the tip with a negative Jacobian. */
    if (IritMvarTVRglrIsNegJacobian(SpikeTV)) {
	if (!Params -> _NegJacoError) {
	    ((IritUserMicroGenAMSupportParamStruct *) Params) ->
							  _NegJacoError = TRUE;
	    IRIT_WARNING_MSG("Negative Jacobian detected in tips - consider a shorter TipLength.\n");
	}
	IritTrivTVFree(SpikeTV);
	return 0;
    }

    SpikeCutTV = IsUpRay ?
		 IritTrivTVRegionFromTV(SpikeTV, 0.0, 0.8, TRIV_CONST_W_DIR) :
		 IritTrivTVRegionFromTV(SpikeTV, 0.2, 1.0, TRIV_CONST_W_DIR);
    SpikePolys = IritPrsrTrivar2Polygons(SpikeCutTV, &TessInfo);

    if (IritGeomPolyBVHPolysInter(PolyBVH, SpikePolys) == 0) {
	/* Lets see if intersection is not too far or too shallow. */
	if (90 - IRIT_ABS(IRIT_RAD2DEG(Slope)) < Params -> TipMinSlope) {
	    if (Dst < Params -> TipMaxLength) {    /* Use this solution! */
		IRIT_PT_COPY(ClosestInterPos, InterPt);
		IRIT_PT_COPY(ClosestInterNrml, InterNrml);

		IritTrivTVFree(SpikeTV);
		IritTrivTVFree(SpikeCutTV);
		IritPrsrFreePolygonList(SpikePolys);
		return IntersectNum;
	    }
	    else if (!Params -> _LongTipsError) {
		((IritUserMicroGenAMSupportParamStruct *) Params) ->
							 _LongTipsError = TRUE;
		IRIT_WARNING_MSG("Too long tips were purged - consider a longer TipMaxLength.\n");
	    }
	}
    }
    else if (!Params -> _IntersectTipsError) {
	((IritUserMicroGenAMSupportParamStruct *) Params) ->
						    _IntersectTipsError = TRUE;
	IRIT_WARNING_MSG("Some tips that were too short or intersected with the model were purged - consider adjusting TileSize.\n");
    }

    IritTrivTVFree(SpikeTV);
    IritTrivTVFree(SpikeCutTV);
    IritPrsrFreePolygonList(SpikePolys);
    return 0;
}

/*****************************************************************************
* DESCRIPTION:								     *
*   Compute the	closest	intersection point between the input model and	     *
* 1. ray (RayPtShifted,	+/-Zdir), where	RayPtShifted is	the shifted position *
*    in	XY to form uniform support in XY plane.				     *
*   Then, if no	intersection is	detected or intersection is too	far/shallow, *
*   try:								     *
* 2. ray (RayPt, RayDir).						     * 
*									     *
* PARAMETERS:								     *
*   Params:	 The support parameters.				     *
*   TlTopo:	 The topology of the tile.				     *
*   RayIndex:	 Number	between	0 to 3,	for the	specific arm of	the tile to  *
*		 shoot ray from.					     *
*   PayPt:	 Origin	of ray.						     *
*   RayDir:	 The direction of the ray.				     *
*   PolyBVH:	 The BVH of the	mesh.					     *
*   PlObj:	 The original input polygonal model.			     *
*   AllInters:	 Array of the ray-PolyMesh intersection	points.	 Allocated   *
*		 by the	caller to be large enough to hold all inters.	     *
*   AllIntersPolys:  Array of the ray-PolyMesh intersection polygons.	     *
*		 Allocated by the caller to be large enough to hold all	pls. *
*   ClosestInterPts:  Will be updated with closest intersection	location,    *
*		 if any.						     *
*   ClosestInterPol:  Will be updated with closest intersection	mesh normal, *
*		 if any.						     *
*									     *
* RETURN VALUE:								     *
*   int:      The number of intersection points.			     *
*****************************************************************************/
static int UserMicroShootRays(
			   const IritUserMicroGenAMSupportParamStruct *Params,
			   UserMicroSupportTopoInfoStruct *TlTopo,
			   int RayIndex,
			   const IrtVecType RayPt, 
			   const IrtVecType RayDir, 
			   const struct IritGeomPolyBVHStruct *PolyBVH,
			   const IritPrsrObjectStruct *PlObj,
			   IrtPtType ClosestInterPos,
			   IrtVecType ClosestInterNrml)
{
    int n,
	OldCircLin = IritPrsrSetPolyListCirc(TRUE),
	IsUpRay = RayDir[2] > 0.0;
    IrtVecType InterNrml;
    IrtPtType RayPtShifted, InterPt,
	VecUp = { USER_MICRO_SUPPORT_CNTCT_PERT_AMNT_TINY,
		  USER_MICRO_SUPPORT_CNTCT_PERT_AMNT_TINY,
		  IsUpRay ? 1.0 : -1.0 };
    CagdSrf2PlsInfoStrct TessInfo;

    IritPrsrTSrf2PlysInitTessInfo2(&TessInfo, TRUE, 20, NULL, 
				   FALSE, FALSE, 0, NULL);

    /* 1. Try the vertical ray first. */
    RayPtShifted[0] = IRIT_BLEND(TlTopo -> BBox.Min[0], TlTopo -> BBox.Max[0],
				 (RayIndex & 0x01) ? 0.25 : 0.75);
    RayPtShifted[1] = IRIT_BLEND(TlTopo -> BBox.Min[1], TlTopo -> BBox.Max[1],
			         RayIndex >= 2 ? 0.25 : 0.75);
    RayPtShifted[2] = RayPt[2];

    if ((n = UserMicroShootRay(Params, RayPtShifted, VecUp, PolyBVH, PlObj,
			       InterPt, InterNrml)) > 0) {
	n = UserMicroTestRay(Params, TlTopo, PolyBVH, RayPtShifted, InterPt,
			     InterNrml, IsUpRay, RayIndex, ClosestInterPos, 
			     ClosestInterNrml, n);
	if (n > 0) {
	    IritPrsrSetPolyListCirc(OldCircLin);
	    return n;
	}
    }

    /* 2. Vertical ray failed - try the diagonal. */
    if ((n = UserMicroShootRay(Params, RayPt, RayDir, PolyBVH, PlObj,
			       InterPt, InterNrml)) > 0) {
	n = UserMicroTestRay(Params, TlTopo, PolyBVH, RayPt, InterPt,
			     InterNrml, IsUpRay, RayIndex, ClosestInterPos,
			     ClosestInterNrml, n);
	if (n > 0) {
	    IritPrsrSetPolyListCirc(OldCircLin);
	    return n;
	}
    }

    IritPrsrSetPolyListCirc(OldCircLin);
    return 0;
}

/*****************************************************************************
* DESCRIPTION:								     *
*   Shots rays up/down from the	tiles that are top in grid (no tile above    *
* them.									     *
*   If hitting PlObj, this support tile	is valid.  Otherwise, mark to purge. *
*									     *
* PARAMETERS:								     *
*   Params:	  The support parameters.				     *
*   PlObj:	  The original input polygonal model.			     *
*   MS:		  The given support microstructure so far.		     *
*   MSTopology:	  A 3D grid hold the topology of this grid.  Will be updated *
*		  with the shot	rays.					     *
*   MSTopoSizes:  The sizes in XYZ of MSTopology.			     *
*									     *
* RETURN VALUE:								     *
*   void								     *
*****************************************************************************/
static void UserMicroShootUpDownRays(
			   const IritUserMicroGenAMSupportParamStruct *Params,
			   const IritPrsrObjectStruct *PlObj,
			   struct IritGeomPolyBVHStruct *PolyBVH,
			   const IritPrsrObjectStruct *MS,
			   UserMicroSupportTopoInfoStruct ***MSTopology,
			   const int MSTopoSizes[3])
{
    IRIT_STATIC_DATA const IrtPtType
        VecUp = { 0.0, 0.0, 1.0 },
	VecDown = { 0.0, 0.0, -1.0 };
    int i, j, k, l;

    for (i = 0; i < MSTopoSizes[0]; i++) {
	for (j = 0; j < MSTopoSizes[1]; j++) {
	    for (k = 0; k < MSTopoSizes[2]; k++) {
	        UserMicroSupportTopoInfoStruct
		    *TlTopo = &MSTopology[i][j][k];
	        IritPrsrObjectStruct
		    *Tile = TlTopo -> Tile;

		if (Tile != NULL) {
		    GMBBBboxStruct
		        *BBox = &TlTopo -> BBox;
		    IrtPtType VecXYPert, PertAmnt;

		    PertAmnt[0] = USER_MICRO_SUPPORT_CNTCT_PERT_AMNT
		      			  * (BBox -> Max[0] - BBox -> Min[0])
					  / (BBox -> Max[2] - BBox -> Min[2]); 
		    PertAmnt[1] = USER_MICRO_SUPPORT_CNTCT_PERT_AMNT
					  * (BBox -> Max[1] - BBox -> Min[1])
					  / (BBox -> Max[2] - BBox -> Min[2]);

		    if (TlTopo -> TopTile) {		   /* Shoot rays up. */
		        for (l = 0; l < 4; l++) {
			    IRIT_VEC_COPY(VecXYPert, VecUp);
			    VecXYPert[0] += (l & 0x01) ? -PertAmnt[0]
						       : PertAmnt[0];
			    VecXYPert[1] += (l < 2) ? PertAmnt[1]
						    : -PertAmnt[1];

			    UserMicroShootRays(Params, TlTopo, l,
					       TlTopo -> Tile4TopPos[l],
					       VecXYPert, PolyBVH, PlObj,
					       TlTopo -> Tile4RayUpPos[l],
					       TlTopo -> Tile4RayUpNrml[l]);
			}
		    }
		    if (TlTopo -> BotTile && k != 0) {	 /* Shoot rays down. */
		        for (l = 0; l < 4; l++) {
			    IRIT_VEC_COPY(VecXYPert, VecDown);
			    VecXYPert[0] += (l & 0x01) ? -PertAmnt[0]
						       : PertAmnt[0];
			    VecXYPert[1] += (l < 2) ? PertAmnt[1]
						    : -PertAmnt[1];

			    UserMicroShootRays(Params, TlTopo, l,
					       TlTopo -> Tile4BotPos[l],
					       VecXYPert, PolyBVH, PlObj,
					       TlTopo -> Tile4RayDwnPos[l],
					       TlTopo -> Tile4RayDwnNrml[l]);
			}
		    }
		}
	    }
	}
    }
}

/*****************************************************************************
* DESCRIPTION:								     *
*   Remove the top and bottom arms of the tile,	Tile, in place,		     *
* as specified by PurgeArms, lexicographic XY order.			     *
*   The	corner boxes and diagonal arms are assumed to be ordered in the	tile *
* in reverse lexicographic XYZ order.  See IritUserMicroDiagTile1	     *
*									     *
* PARAMETERS:								     *
*   Tile:      A diagonal tile to purge	some of	its top	arms.		     *
*   PurgeArms: The top and the bottom arms to remove.			     *
*   TileCase:  The number that indicates the tile is TopTile only,	     *
*	       BotTile only, or	both TopTile and BotTile.		     *
*									     *
* RETURN VALUE:								     *
*   void								     *
*****************************************************************************/
static void UserMicroPurgeInactiveArms(IritPrsrObjectStruct *Tile,
				       const int PurgeArms[8])
{
    int i, Idx, RodIdx,
	NumVertRods = 0,
	NumPurgeArms = 0;
    IritPrsrObjectStruct
	*RodDiag = IritPrsrListObjectGet(Tile, 1),
	*CrnrDiag = IritPrsrListObjectGet(Tile, 2);

    /* Count the number of vertical rods. */
    NumVertRods = IritCagdListLength(RodDiag -> U.Trivars) - 8;

    /* Check if all arms are deleted or not. */
    for (i = 0; i < 8; i++) {
	if (PurgeArms[i])
	    NumPurgeArms++;
    }

    if (NumVertRods == 0 && NumPurgeArms == 8) {
	IritMiscAttrIDSetObjectIntAttrib(Tile, IRIT_ATTR_CREATE_ID(Invalid),
					 TRUE);
	return;
    }

    /* Filters the bottom arms first to maintain indices. */
    for (i = 0; i < 4; i++) {
	if (PurgeArms[i + 4]) {
	    TrivTVStruct *TV;

	    /* Map i to index in the reverse lexicographic XYZ order. */
	    Idx = 7 - i;

	    /* If vertical rods exists, shift the index by the number */
	    /* of vertical rods.				      */
	    RodIdx = Idx + NumVertRods;

	    /* We now remove end to start so indices are not shifted. */
	    TV = (TrivTVStruct *)IritCagdListDelNth((VoidPtr *)
						    &RodDiag -> U.Trivars,
						    RodIdx);
	    IritTrivTVFree(TV);

	    if (CrnrDiag) {	   /* Might have no boxes at the corners... */
		TV = (TrivTVStruct *)
		    IritCagdListDelNth((VoidPtr *)
				   &CrnrDiag -> U.Trivars, Idx);
		IritTrivTVFree(TV);
	    }
	}
    }

    /* Filters the top arms. */
    for (i = 0; i < 4; i++) {
	if (PurgeArms[i]) {
	    TrivTVStruct *TV;

	    /* Map i to index in the reverse lexicographic XYZ order. */
	    Idx = 3 - i;

	    /* If vertical rods exists, shift the index by the number */
	    /* of vertical rods.				      */
	    RodIdx = Idx + NumVertRods;

	    /* We now remove end to start so indices are not shifted. */
	    TV = (TrivTVStruct *)IritCagdListDelNth((VoidPtr *)
						    &RodDiag -> U.Trivars,
						    RodIdx);
	    IritTrivTVFree(TV);

	    if (CrnrDiag) {	   /* Might have no boxes at the corners... */
		TV = (TrivTVStruct *)
		    IritCagdListDelNth((VoidPtr *)
				   &CrnrDiag -> U.Trivars, Idx);
		IritTrivTVFree(TV);
	    }
	}
    }

    /* Filters the center only if both top and bottom arms are removed, and */
    /* there are no vertical rods.					    */
    for (i = 0; i < 4; i++) {
	IritPrsrObjectStruct
	    *CtrDiag = IritPrsrListObjectGet(Tile, 0);

	if (PurgeArms[i] && PurgeArms[i + 4] && NumVertRods == 0) {
	    TrivTVStruct *TV;

	    /* Map i to index in the reverse lexicographic XYZ order. */
	    Idx = 3 - i;

	    /* We now remove end to start so indices are not shifted. */
	    TV = (TrivTVStruct *)IritCagdListDelNth((VoidPtr *)
						    &CtrDiag -> U.Trivars,
						    Idx);
	    IritTrivTVFree(TV);
	}
    }
}

/*****************************************************************************
* DESCRIPTION:								     *
*    If	the arm	of the tile can	support	the neighbor tile by face-to-face    *
*    contact with the arm of the neighbor, mark	the arm	as not being purged. *
*									     *
* PARAMETERS:								     *
*   MSTopoSizes:  The sizes in XYZ of MSTopology.			     *
*   MSTopology:	The topology information of MS.				     *
*									     *
* RETURN VALUE:								     *
*   void								     *
*****************************************************************************/
static void UserMicroFilterCheckNeighborArmsAux(
				 const int *MSTopoSizes,
				 UserMicroSupportTopoInfoStruct ***MSTopology)
{
    int i, j, k;
    IrtBType MadeChanges;

    do {
	MadeChanges = FALSE;
	for (k = 0; k < MSTopoSizes[2]; k++) {
	    for (j = 0; j < MSTopoSizes[1]; j++) {
		for (i = 0; i < MSTopoSizes[0]; i++) {
		    UserMicroSupportTopoInfoStruct
			*TlTopo = &MSTopology[i][j][k];

		    if (TlTopo -> Tile == NULL)
			continue;

		    /* Do not check the internal tile. */
		    if (!(TlTopo -> TopTile || TlTopo -> BotTile))
			continue;		     

		    MadeChanges = UserMicroFilterCheckNeighborArms(
			TlTopo -> Tile, TlTopo -> ActiveArms, MSTopology) ?
								TRUE :
								MadeChanges;
		}
	    }
	}
    }
    while (MadeChanges);
}

/*****************************************************************************
* DESCRIPTION:								     *
*    If	the arm	of the tile can	support	the neighbor tile by face-to-face    *
*    contact with the arm of the neighbor, mark	the arm	as not being purged. *
*									     *
* PARAMETERS:								     *
*   Tile:	A diagonal tile	to purge some of its top arms.		     *
*   PurgeArms:	The top	and the	bottom arms to remove.			     *
*   MSTopology:	The topology information of MS.				     *
*									     *
* RETURN VALUE:								     *
*   int:	TRUE if	PurgeArms is changed				     *
*****************************************************************************/
static int UserMicroFilterCheckNeighborArms(
				IritPrsrObjectStruct *Tile,
				int PurgeArms[8],
			        UserMicroSupportTopoInfoStruct ***MSTopology)
{
    int j, X, Y, Z, SizeX, SizeY, SizeZ, ArmIdx,
	MadeChange = FALSE;
    const char
	*MSIndex = IritMiscAttrIDGetObjectStrAttrib(Tile,
					    IRIT_ATTR_CREATE_ID(MSIndex));
    IRIT_STATIC_DATA const int
        NInfo[8][3] = {     /* The neighbor's corresponding arm information */
	    {1, 2, 4}, {0, 3, 5}, {3, 0, 6}, {2, 1, 7},   /* of each arm of */
	    {5, 6, 0}, {4, 7, 1}, {7, 4, 2}, {6, 5, 3}         /* the tile. */
	},
        SignInfo[8][3] = {     /* Sign to find neighbor tiles that arms are */
	    {-1, -1, 1}, {1, -1, 1}, {-1, 1, 1}, {1, 1, 1},/* touching each */
	    {-1, -1, -1}, {1, -1, -1}, {-1, 1, -1}, {1, 1, -1}    /* other. */
	};

    sscanf(MSIndex, "%d,%d,%d::%d,%d,%d",
	   &X, &Y, &Z, &SizeX, &SizeY, &SizeZ);

    for (ArmIdx = 0; ArmIdx < 8; ArmIdx++) {
	if (PurgeArms[ArmIdx]) {
	    /* Get three neighbor tiles that each tile's corresponding arm */
	    /* has face-to-face contact with the arm of the tile. */
	    for (j = 0; j < 3; j++) { /* Get XY neighbor tiles that current */
		UserMicroSupportTopoInfoStruct *NeigTopo; /* arm is facing. */
		int Sign = SignInfo[ArmIdx][j];
		
		if (j == 0 && X + Sign >= 0 && X + Sign < SizeX)
		    NeigTopo = &MSTopology[X + Sign][Y][Z];
		else if (j == 1 && Y + Sign >= 0 && Y + Sign < SizeY)
		    NeigTopo = &MSTopology[X][Y + Sign][Z];
		else if (j == 2 && Z + Sign >= 0 && Z + Sign < SizeZ)
		    NeigTopo = &MSTopology[X][Y][Z + Sign];
		else continue;

		if (NeigTopo -> Tile == NULL)
		    continue;

		/* If the neighbor's arm exists, do not purge current arm. */
		if (!NeigTopo -> ActiveArms[NInfo[ArmIdx][j]]) {
		    PurgeArms[ArmIdx] = FALSE;
		    MadeChange = TRUE;
		}
	    }
	}
	else { /* If the current arm is marked not to purge. */
	    IrtBType IsRayHit, IsNoAdjArm = TRUE;

	    /* Check the corresponding ray hits the model. */
	    if (ArmIdx < 4)
		IsRayHit =
		    !IRIT_PT_APX_EQ(MSTopology[X][Y][Z].Tile4RayUpPos[ArmIdx],
				    VecNone);
	    else
		IsRayHit =
		!IRIT_PT_APX_EQ(MSTopology[X][Y][Z].Tile4RayDwnPos[ArmIdx - 4],
				VecNone);
			 
	    if (IsRayHit) /* If the ray hit the model, we need to keep the */
		continue;		  /* arm, so do not further check. */

	    /* First, mark the arm to be purged. */
	    PurgeArms[ArmIdx] = TRUE;
	    MadeChange = TRUE;

	    for (j = 0; j < 3; j++) { /* Get XY neighbor tiles that current */
		UserMicroSupportTopoInfoStruct *NeigTopo; /* arm is facing. */
		int Sign = SignInfo[ArmIdx][j];

		if (j == 0 && X + Sign >= 0 && X + Sign < SizeX)
		    NeigTopo = &MSTopology[X + Sign][Y][Z];
		else if (j == 1 && Y + Sign >= 0 && Y + Sign < SizeY)
		    NeigTopo = &MSTopology[X][Y + Sign][Z];
		else if (j == 2 && Z + Sign >= 0 && Z + Sign < SizeZ)
		    NeigTopo = &MSTopology[X][Y][Z + Sign];
		else continue;

		if (NeigTopo -> Tile == NULL)
		    continue;

		IsNoAdjArm = FALSE;

		/* If the neighbor's arm exists, do not purge current arm. */
		if (!NeigTopo -> ActiveArms[NInfo[ArmIdx][j]]) {
		    PurgeArms[ArmIdx] = FALSE;
		    MadeChange = FALSE;
		}
	    }

	    /* Handle the 4 corner arms of the model. */
	    if (IsNoAdjArm) {
		PurgeArms[ArmIdx] = FALSE;
		MadeChange = FALSE;
	    }
	}
    }

    return MadeChange;
}

/*****************************************************************************
* DESCRIPTION:								     *
*   Filters out	tiles that have	no contribution	to the support:		     *
* 1. If	a tile is a top	tile and there is no valid up rays, purge it.	     *
* 2. If	a tile is a top	tile and not all its four corners are providing	     *
*    support, remove top arms that are supporting nothing. The same applies  *
*    to	non-base bottom	tiles.						     *
* 3. If	a bottom tile has no down arms,	purge it.			     *
* 4. Remove all	inactive arms of the tile.				     *
*									     *
* PARAMETERS:								     *
*   MS:		   The support microstructure.				     *
*   MSTopoSizes:   The size of the 3D grid of tiles.			     *
*   MSTopology:	   The topology	information of MS.			     *
*									     *
* RETURN VALUE:								     *
*   IritPrsrObjectStruct *:  The filtered out set of tiles that contribute   *
*			to the support microstructure.			     *
*****************************************************************************/
static IritPrsrObjectStruct *UserMicroFilterInactiveTilesAndArms(
				 IritPrsrObjectStruct *MS,
				 const int *MSTopoSizes,
				 UserMicroSupportTopoInfoStruct ***MSTopology)
{
    int i, j, k, n,
        MadeChanges = FALSE;

    /* 1. Remove top tiles that offer no support - mark those tiles. */
    do {
        MadeChanges = FALSE;
        for (k = MSTopoSizes[2] - 1; k >= 0; k--) {
	    for (j = 0; j < MSTopoSizes[1]; j++) {
	        for (i = 0; i < MSTopoSizes[0]; i++) {
		    if (MSTopology[i][j][k].Tile != NULL &&
			MSTopology[i][j][k].TopTile &&
			USER_MICRO_SUPPORT_NO_UP_RAY(MSTopology[i][j][k])) {
		        /* A top tile (nothing above it) and no up rays.   */
		        /* Remove this tile!				   */
		        MadeChanges = TRUE;
			IritMiscAttrIDSetObjectIntAttrib(
						 MSTopology[i][j][k].Tile,
						 IRIT_ATTR_CREATE_ID(Invalid),
						 TRUE);
			MSTopology[i][j][k].Tile = NULL;
			if (k > 0 && MSTopology[i][j][k - 1].Tile != NULL) {
			    /* The tile below this one is a top tile! */
			    MSTopology[i][j][k - 1].TopTile = TRUE;
			}
		    }
		}
	    }
	}
    }
    while (MadeChanges);

    /* 2a. Mark ActiveArms depending on whether tile require arms or not. */
    for (k = 0; k < MSTopoSizes[2]; k++) {
	for (j = 0; j < MSTopoSizes[1]; j++) {
	    for (i = 0; i < MSTopoSizes[0]; i++) {
		UserMicroSupportTopoInfoStruct
		    *TlTopo = &MSTopology[i][j][k];

		if (TlTopo -> Tile == NULL)
		    continue;

		IRIT_ZAP_MEM(TlTopo -> ActiveArms, sizeof(int) * 8);

		if (TlTopo -> TopTile)
		    for (n = 0; n < 4; n++)
			TlTopo -> ActiveArms[n] =
			  IRIT_PT_APX_EQ(TlTopo -> Tile4RayUpPos[n], VecNone);
	    	
		if (TlTopo -> BotTile && k != 0)
		    for (n = 0; n < 4; n++)
			TlTopo -> ActiveArms[n + 4] =
			  IRIT_PT_APX_EQ(TlTopo -> Tile4RayDwnPos[n], VecNone);
	    }
	}
    }

    /* 2b. If arms of a neighboring tile and the arms of the tile can face   */
    /*     each other, mark those arms as required. */
    UserMicroFilterCheckNeighborArmsAux(MSTopoSizes, MSTopology);

    /* 3. Remove bottom tiles that all down arms are inactive. */
    do {
	MadeChanges = FALSE;
	for (k = 0; k < MSTopoSizes[2]; k++) {
	    for (j = 0; j < MSTopoSizes[1]; j++) {
		for (i = 0; i < MSTopoSizes[0]; i++) {
		    UserMicroSupportTopoInfoStruct
			*TlTopo = &MSTopology[i][j][k];

		    if (TlTopo -> Tile == NULL)
			continue;

		    if (!TlTopo -> BotTile)  /* Check bottom tile only. */
			continue;

		    /* Check if all down arms are needed to be purged. */
		    if (TlTopo -> ActiveArms[4] && TlTopo -> ActiveArms[5] &&
			TlTopo -> ActiveArms[6] && TlTopo -> ActiveArms[7]) {
			MadeChanges = TRUE;
			IritMiscAttrIDSetObjectIntAttrib(TlTopo -> Tile,
						 IRIT_ATTR_CREATE_ID(Invalid),
						 TRUE);

			TlTopo -> Tile = NULL;

			/* The tile above this one is a bottom tile now. */
			if (k + 1 < MSTopoSizes[2] &&
			    MSTopology[i][j][k + 1].Tile != NULL) {
			    UserMicroSupportTopoInfoStruct
				*UpTlTopo = &MSTopology[i][j][k + 1];

			    UpTlTopo -> BotTile = TRUE;

			    for (n = 4; n < 8; n++)
				UpTlTopo -> ActiveArms[n] = TRUE;

			    UserMicroFilterCheckNeighborArms(
							UpTlTopo -> Tile,
							UpTlTopo -> ActiveArms,
							MSTopology);
			}
			
		    }
		}
	    }
	}
    }
    while (MadeChanges);

#ifdef DEBUG_DUMP_MS_UP_DOWN_RAYS
    {
        IritPrsrObjectStruct
	    *Rays = IritPrsrGenLISTObject(NULL);

	/* Generate up down lines in directions of the detected valid rays. */
	for (k = 0; k < MSTopoSizes[2]; k++) {
	    for (j = 0; j < MSTopoSizes[1]; j++) {
	        for (i = 0; i < MSTopoSizes[0]; i++) {
		    if (MSTopology[i][j][k].Tile != NULL) {
			int l;
			IritPrsrObjectStruct *PObj;

			if (MSTopology[i][j][k].TopTile) {
			    for (l = 0; l < 4; l++) {
			        if (!IRIT_PT_APX_EQ(VecNone,
				      MSTopology[i][j][k].Tile4RayUpPos[l])) {
				    /* Generate  and dump the up ray here. */
				    CagdCrvStruct
				        *Ln = IritCagdMergePtPt2(
					 MSTopology[i][j][k].Tile4TopPos[l],
					 MSTopology[i][j][k].Tile4RayUpPos[l]);

				    PObj = IritPrsrGenCRVObject(Ln);
				    IritMiscAttrIDSetObjectRGBColor(PObj,
								    0, 0, 100);
				    IritMiscAttrIDSetObjectRealAttrib(PObj,
					     IRIT_ATTR_CREATE_ID(DWidth), 3);
				    IritPrsrListObjectAppend(Rays, PObj);
				}
			    }
			}
			if (MSTopology[i][j][k].BotTile) {
			    for (l = 0; l < 4; l++) {
			        if (!IRIT_PT_APX_EQ(VecNone,
				     MSTopology[i][j][k].Tile4RayDwnPos[l])) {
				    /* Generate and dump the down ray here. */
				    CagdCrvStruct
				        *Ln = IritCagdMergePtPt2(
					 MSTopology[i][j][k].Tile4BotPos[l],
					 MSTopology[i][j][k].Tile4RayDwnPos[l]);

				    PObj = IritPrsrGenCRVObject(Ln);
				    IritMiscAttrIDSetObjectRGBColor(PObj,
								    0, 100, 0);
				    IritMiscAttrIDSetObjectRealAttrib(PObj,
					     IRIT_ATTR_CREATE_ID(DWidth), 3);
				    IritPrsrListObjectAppend(Rays, PObj);
				}			    
			    }
			}
		    }
		}
	    }
	}

	IritPrsrPutObjectToFile3("MSSupportRays.itd", Rays, 0);
	IritPrsrFreeObject(Rays);
    }
#endif /* DEBUG_DUMP_MS_UP_DOWN_RAYS */

    return MS;
}

/*****************************************************************************
* DESCRIPTION:								     *
*   Convert an intersecting ray	from RayPt to InterPt.			     *
*									     *
* PARAMETERS:								     *
*   RayPt:	  Starting position of ray.				     *
*   InterPt:	  End/intersecting location of ray from	Ray Pt.		     *
*   InterNrml:	  End/intersection mesh	normal at InterPt.		     *
*   BaseDim:	  The diameter of the base of the spike, centered at RayPt.  *
*   TipScale:	  The scale to apply to	the end	tip of the spike.	     *
*   NormalTipLen: The length of	the tip	in the mesh normal direction.	     *
*   SclSlopeDist: How far from end tip should we apply scale down on dim.    *
*   Params:	  The support parameters.				     *
*									     *
* RETURN VALUE:								     *
*   TrivTVStruct *:  The created spike as a trivariates			     *
*****************************************************************************/
static TrivTVStruct *UserMicroCnvrtRayToTV(
			   const IrtPtType RayPt,
			   const IrtPtType InterPt,
			   const IrtVecType InterNrml,
			   IrtRType BaseDim[2],
			   IrtRType TipScale,
			   IrtRType NormalTipLen,
			   IrtRType SclSlopeDist,
			   IrtBType IsUpRay,
			   const IritUserMicroGenAMSupportParamStruct *Params)
{
    IRIT_STATIC_DATA const IrtVecType
        XVec = { 1.0, 0.0, 0.0 },
        ZVec = { 0.0, 0.0, 1.0 };
    int i;
    IrtRType TipScales[3], TipSize,
	Dist = IRIT_PT_PT_DIST(RayPt, InterPt),
	BaseDim2[2] = { BaseDim[0] * 0.5, BaseDim[1] * 0.5 };
    IrtPtType Translate, Translate2;
    IrtVecType NrmlZAvg, InterNrmlFlip;
    CagdBBoxStruct BBox;
    IrtHmgnMatType Mat;
    CagdCrvStruct *TipRect2,
	*BaseRect = IritCagdPrimRectangleCrv(RayPt[0] - BaseDim2[0],
					     RayPt[1] - BaseDim2[1],
					     RayPt[0] + BaseDim2[0],
					     RayPt[1] + BaseDim2[1],
					     RayPt[2]),
        *TipRect = IritCagdCrvCopy(BaseRect);
    CagdSrfStruct *Srf;
    TrivTVStruct *TV;

    if (Dist < Params -> TileSize * 0.1 && !Params -> _ShortTipsError) {
	((IritUserMicroGenAMSupportParamStruct *) Params) ->
    							_ShortTipsError = TRUE;
	IRIT_WARNING_MSG("Some tips are too short - consider adjusting TileSize.\n");
    }

    IRIT_PT_SUB(Translate, InterPt, RayPt);
    IritCagdCrvTransform(TipRect, Translate, 1.0);
    TipScales[0] = TipScales[1] = TipScales[2] = TipScale;
    IritCagdCrvScaleCenter(TipRect, TipScales, NULL);
    TipRect2 = IritCagdCrvCopy(TipRect);

    for (i = 0; i < 3; i++)
	InterNrmlFlip[i] = IsUpRay ? InterNrml[i] : -InterNrml[i];

    IritGeomGenMatrixZ2Dir2(Mat, InterNrmlFlip, XVec);
    IritCagdCrvMatTransCenter(TipRect, Mat);
    IritCagdCrvBBox(TipRect, &BBox);
    TipSize = IRIT_MAX3(BBox.Max[0] - BBox.Min[0],
			BBox.Max[1] - BBox.Min[1],
			BBox.Max[2] - BBox.Min[2]);

    if (Dist > SclSlopeDist) {
        /* Apply Slope in scale only from (Dist - SclSlopeDist). */
        IrtRType
	    SclSlopeFactor = 1.0 - SclSlopeDist / Dist;
        CagdCrvStruct
	    *MidRect = IritCagdCrvCopy(BaseRect);

	for (i = 0; i < 3; i++)
	    Translate[i] *= SclSlopeFactor;
	IritCagdCrvTransform(MidRect, Translate, 1.0);

	BaseRect -> Pnext = MidRect;
	if (TipSize > NormalTipLen) {			      /* Do no tip! */
	    MidRect -> Pnext = TipRect;
	    IritCagdCrvFree(TipRect2);
	}
	else {
	    for (i = 0; i < 3; i++)
		Translate[i] = -InterNrml[i] * NormalTipLen;
	    IritCagdCrvTransform(TipRect2, Translate, 1.0);

	    IRIT_VEC_BLEND(NrmlZAvg, InterNrmlFlip, ZVec, 0.5);
	    IRIT_VEC_NORMALIZE(NrmlZAvg);
	    IritGeomGenMatrixZ2Dir2(Mat, NrmlZAvg, XVec);
	    IritCagdCrvMatTransCenter(TipRect2, Mat); 

	    MidRect -> Pnext = TipRect2;
	    TipRect2 -> Pnext = TipRect;
	}
    }
    else if (Dist > NormalTipLen && TipSize < NormalTipLen) {
        /* Create only a tip of length NormalTipLen. */
	for (i = 0; i < 3; i++)
	    Translate2[i] = -InterNrml[i] * NormalTipLen;

	/* If tip is way too small in Z - make sure the middle cross        */
	/* section curve is still in between.				    */
	if (IRIT_ABS(Translate[2]) * 0.75 < IRIT_ABS(Translate2[2])) {
	    IrtRType
		t = IRIT_ABS(Translate[2]) * 0.75 / IRIT_ABS(Translate2[2]);

	    IRIT_PT_SCALE(Translate2, t);
	}

	IritCagdCrvTransform(TipRect2, Translate2, 1.0);
        BaseRect -> Pnext = TipRect2;
        TipRect2 -> Pnext = TipRect;
    }
    else {
        BaseRect -> Pnext = TipRect;
	IritCagdCrvFree(TipRect2);
    }

    Srf = IritCagdSrfFromCrvs(BaseRect, 2, CAGD_END_COND_OPEN, NULL);
    IritCagdCrvFreeList(BaseRect);

    if (!IsUpRay) {			     /* Reverse the tip orientation. */
	CagdSrfStruct
	    *TSrf = IritCagdSrfReverseDir(Srf, CAGD_CONST_V_DIR);

	IritCagdSrfFree(Srf);
	Srf = TSrf;
    }

    TV = IritMvarTrivarBoolOne(Srf);
    IritCagdSrfFree(Srf);

#   ifdef DEBUG_UNIQUE_TV_IDS
    {
        IRIT_STATIC_DATA int
	    TVIdx = 1;

        /* Add unique indices to all TVs. */
        IritMiscAttrIDSetIntAttrib(&TV -> Attr, IRIT_ATTR_CREATE_ID(index),
				   TVIdx++);

	if (TVIdx == 2281) {
	    IritTrivDbg(TV);
	    printf("Found it");
	}
    }
#   endif /* DEBUG_UNIQUE_TV_IDS */

    /* Mark the tip as such. */
    if (Params -> _BzrOnlyTVs) {
	TrivTVStruct
	    *TVs = IritTrivTVSubdivAtAllC1Discont(TV);

	if (TVs -> Pnext != NULL) {  /* TV divided - keep original as well. */
	    IritMiscAttrIDSetObjAttrib(&TVs -> Attr, IRIT_ATTR_CREATE_ID(OrigTV),
				       IritPrsrGenTRIVARObject(TV), FALSE);
	}
	else {
	    IritTrivTVFree(TV);
	}

	TV = IritCagdListLast(TVs); /* Place Bndry attribute in the last TV. */
    	IritMiscAttrIDSetStrAttrib(&TV -> Attr, IRIT_ATTR_CREATE_ID(Bndry),
			       	   "5::Tip");
    	return TVs;
    }
    else {
    	IritMiscAttrIDSetStrAttrib(&TV -> Attr, IRIT_ATTR_CREATE_ID(Bndry),
			       	   "5::Tip");
        return TV;
    }
}

/*****************************************************************************
* DESCRIPTION:								     *
*   Convert an intersecting ray	from RayPt to InterPt.			     *
*									     *
* PARAMETERS:								     *
*   TileTopo:	 A 3D grid hold	the topology of	this grid.		     *
*   Params:	 The support parameters.				     *
*   InterPt:	 End/intersecting location of ray from Ray Pt.		     *
*   InterNrml:	 End/intersection mesh normal at InterPt.		     *
*   IsUpRay:	 TRUE if the ray is up ray.				     *
*   RayIdx:	 Number	between	0 to 3,	for the	specific arm of	the tile to  *
*		 shoot ray from.					     *
*									     *
* RETURN VALUE:								     *
*   TrivTVStruct *:  The created spike as a single trivariate.		     *
*****************************************************************************/
static TrivTVStruct *UserMicroCnvrtRayToTV2(
			   const UserMicroSupportTopoInfoStruct *TileTopo,
			   const IritUserMicroGenAMSupportParamStruct *Params,
			   IrtPtType InterPt,
			   IrtPtType InterNrml,
			   IrtBType IsUpRay,
			   int RayIdx)
{
    int i;
    IrtRType BaseDim[2];
    TrivTVStruct *TVs;
    IrtPtType TilePos;

    IsUpRay ? IRIT_PT_COPY(TilePos, TileTopo -> Tile4TopPos[RayIdx])
	    : IRIT_PT_COPY(TilePos, TileTopo -> Tile4BotPos[RayIdx]);

    if (!Params -> PrmThicknessTV) {
	for (i = 0; i < 2; i++)
	    BaseDim[i] = (TileTopo -> BBox.Max[i] - TileTopo -> BBox.Min[i]) *
			 Params -> TileEndEdgeThickness;
    }
    else {
	/* Use parametric thickness. */
	for (i = 0; i < 2; i++)
	    BaseDim[i] = (TileTopo -> BBox.Max[i] - TileTopo -> BBox.Min[i]) *
			 TileTopo -> PrmCrnrThickness[IsUpRay ? (RayIdx + 4) 
							      : RayIdx];
    }

    TVs = UserMicroCnvrtRayToTV(TilePos, InterPt, InterNrml, BaseDim, 
			        Params -> TipScale, 
			        Params -> TileSize * Params -> TipLength, 
			        Params -> TileSize * 0.5, IsUpRay,
				Params);

    return TVs;
}

/*****************************************************************************
* DESCRIPTION:								     *
*   Generate spikes for	all valid rays in the MS support structures, as	     *
* trivariates.								     *
*									     *
* PARAMETERS:								     *
*   MS:		  The give support microstructure so far.		     *
*   MSTopology:	  A 3D grid hold the topology of this grid.  Will be updated *
*		  with the shot	rays.					     *
*   MSTopoSizes:  The sizes in XYZ of MSTopology.			     *
*									     *
* RETURN VALUE:								     *
*   IritPrsrObjectStruct *:	The generated spikes as	trivars.	     *
*****************************************************************************/
static IritPrsrObjectStruct* UserMicroGenRaySpikes(
			   IritPrsrObjectStruct *MS,
			   const int *MSTopoSizes,
			   UserMicroSupportTopoInfoStruct ***MSTopology,
			   const IritUserMicroGenAMSupportParamStruct *Params)
{
    int i, j, k, NumTiles, NumSpikes, Spike0Idx, *ItstArr,
	ItstArrSize = 0;
    IritCagdBspMultComputationMethodType
	OldMultMethod = IritCagdBspMultComputationMethod(IRIT_QUERY_INT_PROP);
    CagdBBoxStruct *TileBBoxArray;
    CagdSrfStruct **TileTVSurfs;
    CagdSrf2PlsInfoStrct TessInfo;
    TrivTVStruct **SpikeCutTVs, *TV;
    IritPrsrPolygonStruct **SpikeCutPolys;
    IritPrsrObjectStruct **TileSrfObjs, *MSTile,
	*TmpSpikes = IritPrsrGenLISTObject(NULL),
	*Spikes = IritPrsrGenLISTObject(NULL),
	*Tiles = IritPrsrGenLISTObject(NULL);

    IritPrsrTSrf2PlysInitTessInfo2(&TessInfo, TRUE, 20, NULL, 
				   FALSE, FALSE, 0, NULL);

    /* Convert tile objects. */
    for (i = 0; (MSTile = IritPrsrListObjectGet(MS, i)) != NULL; i++) {
	int x, y, z, SizeX, SizeY, SizeZ;
	IritPrsrObjectStruct* MSTilePart;
	const char
	    *MSIndex = IritMiscAttrIDGetObjectStrAttrib(MSTile,
						IRIT_ATTR_CREATE_ID(MSIndex));
	
	sscanf(MSIndex, "%d,%d,%d::%d,%d,%d",
	       &x, &y, &z, &SizeX, &SizeY, &SizeZ);

	for (j = 0;
	     (MSTilePart = IritPrsrListObjectGet(MSTile, j)) != NULL;
	     j++) {
	    for (TV = MSTilePart -> U.Trivars; TV != NULL; TV = TV -> Pnext) {
		IritPrsrObjectStruct
		    *PartObj = IritPrsrGenTRIVARObject(TV);

		IritMiscAttrIDSetObjectStrAttrib(PartObj,
						 IRIT_ATTR_CREATE_ID(MSIndex),
						 MSIndex);
		IritPrsrListObjectAppend(Tiles, PartObj);
	    }
	}
    }

    /* Generate Tile BBoxes. */
    k = IritPrsrListObjectLength(Tiles);
    if (k > 0) {
	TileBBoxArray = IritCagdBBoxArrayNew(k);
	for (i = 0; (MSTile = IritPrsrListObjectGet(Tiles, i)) != NULL; i++) {
	    IritGeomBBComputeBboxObject(MSTile, &TileBBoxArray[i], FALSE);
	}
    }
    else
	TileBBoxArray = NULL;

    /* Go over all tiles and look for valid rays, for which we generate TVs. */
    for (k = 0; k < MSTopoSizes[2]; k++) {
	for (j = 0; j < MSTopoSizes[1]; j++) {
	    for (i = 0; i < MSTopoSizes[0]; i++) {
		if (MSTopology[i][j][k].Tile != NULL) {
		    int l, x, y, z, SizeX, SizeY, SizeZ;
		    char *TipIndex;
		    const char
			*MSIndex = IritMiscAttrIDGetObjectStrAttrib(
					       MSTopology[i][j][k].Tile,
					       IRIT_ATTR_CREATE_ID(MSIndex));
		    TrivTVStruct *TV;
		    IritPrsrObjectStruct *TVObj;

		    sscanf(MSIndex, "%d,%d,%d::%d,%d,%d",
			   &x, &y, &z, &SizeX, &SizeY, &SizeZ);

		    TipIndex = (char *) IritMalloc((int) (strlen(MSIndex) + 4));

		    if (MSTopology[i][j][k].TopTile) {
			for (l = 0; l < 4; l++) {
			    if (!IRIT_PT_APX_EQ(VecNone,
				MSTopology[i][j][k].Tile4RayUpPos[l])) {
				/* Generate and dump the up spike. */
				TV = UserMicroCnvrtRayToTV2(
					&MSTopology[i][j][k], Params,
					MSTopology[i][j][k].Tile4RayUpPos[l],
					MSTopology[i][j][k].Tile4RayUpNrml[l],
				        TRUE, l);
				TVObj = IritPrsrGenTRIVARObject(TV);
				sprintf(TipIndex, "%d,%d,%d::%d,%d,%d::%d",
					x, y, z, SizeX, SizeY, SizeZ, l);
				IritMiscAttrIDSetObjectStrAttrib(
						TVObj,
						IRIT_ATTR_CREATE_ID(MSIndex),
						TipIndex);
				IritPrsrListObjectAppend(TmpSpikes, TVObj);
				ItstArrSize++;
			    }
			}
		    }
		    if (MSTopology[i][j][k].BotTile) {
			for (l = 0; l < 4; l++) {
			    if (!IRIT_PT_APX_EQ(VecNone,
				MSTopology[i][j][k].Tile4RayDwnPos[l])) {
				/* Generate and dump the down spike. */
				TV = UserMicroCnvrtRayToTV2(
				        &MSTopology[i][j][k], Params,
					MSTopology[i][j][k].Tile4RayDwnPos[l],
					MSTopology[i][j][k].Tile4RayDwnNrml[l],
					FALSE, l);
				TVObj = IritPrsrGenTRIVARObject(TV);
				sprintf(TipIndex, "%d,%d,%d::%d,%d,%d::%d",
					x, y, z, SizeX, SizeY, SizeZ, l + 4);
				IritMiscAttrIDSetObjectStrAttrib(
				        	TVObj,
						IRIT_ATTR_CREATE_ID(MSIndex),
						TipIndex);
				IritPrsrListObjectAppend(TmpSpikes, TVObj);
				ItstArrSize++;
			    }
			}
		    }

		    IritFree(TipIndex);
		}
	    }
	}
    }

    NumSpikes = IritPrsrListObjectLength(TmpSpikes);
    NumTiles = IritPrsrListObjectLength(Tiles);

    if (NumTiles == 0 || NumSpikes == 0)
	return NULL;     /* Something is wrong here - no support generated. */

    /* Memory allocation and object generation for parallelization. */
    ItstArr = IritMalloc(sizeof(int) * NumSpikes);
    IRIT_ZAP_MEM(ItstArr, sizeof(int) * NumSpikes);

    TileTVSurfs = (CagdSrfStruct **)
			        IritMalloc(sizeof(CagdSrfStruct *) * NumTiles);

#   pragma omp parallel for private(i) if (_IritParallelExec)
    for (i = 0; i < NumTiles; i++) {
	TileTVSurfs[i] = IritTrivBndrySrfsFromTVs(
				 IritPrsrListObjectGet(Tiles, i) -> U.Trivars,
				 IRIT_EPS, TRUE, FALSE, FALSE);
    }

    SpikeCutTVs = (TrivTVStruct **)
			    IritMalloc(sizeof(TrivTVStruct *) * NumSpikes);

    SpikeCutPolys = (IritPrsrPolygonStruct **)
		    IritMalloc(sizeof(IritPrsrPolygonStruct *) * NumSpikes);

#   pragma omp parallel for private(i) if (_IritParallelExec)
    for (i = 0; i < NumSpikes; i++) {
	IrtRType UMin, UMax, VMin, VMax, WMin, WMax;
	IritPrsrObjectStruct
	    *SpikeObj = IritPrsrListObjectGet(TmpSpikes, i);
	
	IritTrivTVDomain(SpikeObj -> U.Trivars, &UMin, &UMax, &VMin, &VMax, 
								 &WMin, &WMax);
	SpikeCutTVs[i] = IritTrivTVRegionFromTV(SpikeObj -> U.Trivars,
						IRIT_BLEND(WMin, WMax, 0.1), 
						IRIT_BLEND(WMin, WMax, 0.9), 
						TRIV_CONST_W_DIR);
	SpikeCutPolys[i] = IritPrsrTrivar2Polygons(SpikeCutTVs[i], &TessInfo);
    }
    
    TileSrfObjs = (IritPrsrObjectStruct **)
	IritMalloc(sizeof(IritPrsrObjectStruct *) * NumTiles);

#   pragma omp parallel for private(i) if (_IritParallelExec)
    for (i = 0; i < NumTiles; i++) {
	TileSrfObjs[i] = IritPrsrGenSRFObject(TileTVSurfs[i]);
    }

    /* If spike has intersection with other spikes or tiles, mark it. */
#   pragma omp parallel for private(Spike0Idx) if (_IritParallelExec)
    for (Spike0Idx = 0; Spike0Idx < NumSpikes; Spike0Idx++) {
	int SpX, SpY, SpZ, SizeX, SizeY, SizeZ, TileIdx, Sp0ArmIdx, Spike1Idx;
	const char *MSIndex;
	struct IritGeomPolyBVHStruct *SpikePolyBVH;
	GMBBBboxStruct SpikeBBox;
	IritPrsrPolygonStruct *SpikePolys;
	IritPrsrObjectStruct
	    *SpikeObj = IritPrsrListObjectGet(TmpSpikes, Spike0Idx);

	SpikePolys = IritPrsrTrivar2Polygons(SpikeCutTVs[Spike0Idx],
					     &TessInfo);
	SpikePolyBVH = IritGeomPolyBVHCreate(SpikePolys);
	IritGeomBBComputeBboxObject(SpikeObj, &SpikeBBox, FALSE);
	MSIndex = IritMiscAttrIDGetObjectStrAttrib(SpikeObj,
					   IRIT_ATTR_CREATE_ID(MSIndex));
	sscanf(MSIndex, "%d,%d,%d::%d,%d,%d::%d",
	       &SpX, &SpY, &SpZ, &SizeX, &SizeY, &SizeZ, &Sp0ArmIdx);

	/* Check if spike has intersection with tiles. */
	for (TileIdx = 0; TileIdx < NumTiles; TileIdx++) {
	    /* Check bbox intersection test first. */
	    int TiX, TiY, TiZ;
	    GMBBBboxStruct
	        *TileBox = &TileBBoxArray[TileIdx];

	    MSIndex = IritMiscAttrIDGetObjectStrAttrib(
					IritPrsrListObjectGet(Tiles, TileIdx),
					IRIT_ATTR_CREATE_ID(MSIndex));
	    sscanf(MSIndex, "%d,%d,%d::%d,%d,%d",
		   &TiX, &TiY, &TiZ, &SizeX, &SizeY, &SizeZ);

	    /* If index of the tile and the spike are same, do not check. */
	    if (SpX == TiX && SpY == TiY && SpZ == TiZ)
		continue;

	    /* Check only if BBox are intersecting. */
	    if (!(SpikeBBox.Max[0] < TileBox -> Min[0] ||
		  SpikeBBox.Min[0] > TileBox -> Max[0] ||
		  SpikeBBox.Max[1] < TileBox -> Min[1] ||
		  SpikeBBox.Min[1] > TileBox -> Max[1] ||
		  SpikeBBox.Max[2] < TileBox -> Min[2] ||
		  SpikeBBox.Min[2] > TileBox -> Max[2])) {
		/* Check if the spike has intersection with the tile. */
		if (IritGeomPolyBVHSrfsPolyInter(TileSrfObjs[TileIdx], 
						 SpikePolyBVH) != 0) {
		    if (Sp0ArmIdx < 4)
			IRIT_PT_COPY(MSTopology[SpX][SpY][SpZ].
				     Tile4RayUpPos[Sp0ArmIdx], VecNone);
		    else
			IRIT_PT_COPY(MSTopology[SpX][SpY][SpZ].
				     Tile4RayDwnPos[Sp0ArmIdx - 4], VecNone);
		    ItstArr[Spike0Idx]++;
		    break;
		}
	    }
	}

	if (ItstArr[Spike0Idx])
	    continue;

	/* Check if spike has intersection with other spikes. */
	for (Spike1Idx = Spike0Idx + 1; Spike1Idx < NumSpikes; Spike1Idx++) {
	    if (IritGeomPolyBVHPolysInter(SpikePolyBVH,
					  SpikeCutPolys[Spike1Idx])) {
		if (Sp0ArmIdx < 4)
		    IRIT_PT_COPY(MSTopology[SpX][SpY][SpZ].
				 Tile4RayUpPos[Sp0ArmIdx], VecNone);
		else
		    IRIT_PT_COPY(MSTopology[SpX][SpY][SpZ].
				 Tile4RayDwnPos[Sp0ArmIdx - 4], VecNone);
		ItstArr[Spike0Idx]++;

	    }
	}

	IritGeomPolyBVHFree(SpikePolyBVH);
	IritPrsrFreePolygonList(SpikePolys);
    }

    for (i = 0; i < NumSpikes; i++) {
	IritPrsrObjectStruct
	    *SpikeObj = IritPrsrListObjectDelete(TmpSpikes, 0, FALSE);

	if (ItstArr[i] == 0)
	    IritPrsrListObjectAppend(Spikes, SpikeObj);
	else
	    IritPrsrFreeObject(SpikeObj);
    }

    IritPrsrFreeObject(TmpSpikes);

    IritFree(ItstArr);
    IritFree(TileTVSurfs);
    IritCagdBBoxArrayFree(TileBBoxArray, NumTiles);

    for (i = 0; i < NumTiles; i++) {
    	IritPrsrFreeObject(TileSrfObjs[i]);
    	IritPrsrListObjectGet(Tiles, i) -> U.Trivars = NULL;
    }
    IritFree(TileSrfObjs);
    IritPrsrFreeObject(Tiles);

    for (i = 0; i < NumSpikes; i++)
	IritPrsrFreePolygonList(SpikeCutPolys[i]);
    IritFree(SpikeCutPolys);

    for (i = 0; i < NumSpikes; i++)
        IritTrivTVFree(SpikeCutTVs[i]);
    IritFree(SpikeCutTVs);

    IritCagdBspMultComputationMethod(OldMultMethod);

    return Spikes;
}

/*****************************************************************************
* DESCRIPTION:								     *
*   Find 8 corner arms of a multi resolution tile of size Stride from the    *
* base 3D topology grid. For a tile of size larger than	1, the information   *
* of arms is spread out	in the different grid cells at the corner of the     *
* Stride x Stride x Stride -sized grid block.				     *
* PARAMETERS:								     *
*   MSTopology:	  A 3D grid hold the topology of this grid.		     *
*   XIdx:	  The smallest X index in the 3D topology grid to be merged. *
*   YIdx:	  The smallest Y index in the 3D topology grid to be merged. *
*   ZIdx:	  The smallest Z index in the 3D topology grid to be merged. *
*   Stride:	  The size of each tile	in terms of tile units		     *
*   PurgeArms:	  The information of 8 corner arms of a	tile of	size Stride, *
*		  spanned from (XIdx, YIdx, ZIdx).			     *
*									     *
* RETURN VALUE:								     *
*   int: The number of inactive	arms (to be purged) of the tile.	     *
*****************************************************************************/
static int UserMicroMultiResGetCornerArms(
				 UserMicroSupportTopoInfoStruct ***MSTopology,
				 int XIdx, 
				 int YIdx, 
				 int ZIdx,
				 int Stride,
				 int PurgeArms[8])
{
    int i, j, k, CnrX[2], CnrY[2], CnrZ[2], ArmId,
	NumPurgeArms = 0;

    if (Stride < 1)
	return 0;

    /* Identify the indices of corner topologies of size Stride. */
    IRIT_PT2D_SET(CnrX, XIdx, XIdx + Stride - 1);
    IRIT_PT2D_SET(CnrY, YIdx, YIdx + Stride - 1);
    IRIT_PT2D_SET(CnrZ, ZIdx, ZIdx + Stride - 1);

    for (k = 0; k < 2; k++) {
	for (j = 0; j < 2; j++) {
	    for (i = 0; i < 2; i++) {
		ArmId = 4 * (1 - k) + 2 * j + i;
		PurgeArms[ArmId]
		    = MSTopology[CnrX[i]][CnrY[j]][CnrZ[k]].ActiveArms[ArmId];
		if (PurgeArms[ArmId])
		    NumPurgeArms++;
	    }
	}
    }
    return NumPurgeArms;
}

/*****************************************************************************
* DESCRIPTION:								     *
*   Check the neighboring topology to determine	whether	to merge a 2 x 2 x 2 *
* block	of half	Stride-sized tiles.					     *
*									     *
* PARAMETERS:								     *
*   MSTopoSizes:  The sizes in XYZ of MSTopology.			     *
*   MSTopology:	  A 3D grid hold the topology of this grid.		     *
*   XIdx:	  The smallest X index in the 3D topology grid to be merged. *
*   YIdx:	  The smallest Y index in the 3D topology grid to be merged. *
*   ZIdx:	  The smallest Z index in the 3D topology grid to be merged. *
*   Stride:	  The size of a	tile after merging, in terms of	the number   *
*		  of cells in the 3D topology grid.			     *
*									     *
* RETURN VALUE:								     *
*   int:  TRUE if the tiles can	be merged, FALSE otherwise.		     *
*****************************************************************************/
static int UserMicroMultiResCheckNeighborsBeforeMerge(
				  const int *MSTopoSizes,
				  UserMicroSupportTopoInfoStruct ***MSTopology,
				  int XIdx,	
				  int YIdx,	
				  int ZIdx,	
				  int Stride)
{
    int i, j, k, CnrX[2], CnrY[2], CnrZ[2], MidX, MidY,
	HS = Stride / 2,
	MultiResTileBelow = FALSE;
    
    /* Check neighboring tiles below the given input tile. */
    if (ZIdx >= Stride &&
	MSTopology[XIdx][YIdx][ZIdx - Stride].Stride == Stride) {
	MultiResTileBelow = TRUE;
    }

    /* Check if neighboring tiles are all half Stride-sized or not. */
    if (!MultiResTileBelow && ZIdx >= HS) {
	for (i = XIdx; i < XIdx + Stride; i += HS) {
	    for (j = YIdx; j < YIdx + Stride; j += HS) {
		if (MSTopology[i][j][ZIdx - HS].Stride != HS)
		    return FALSE;
	    }
	}
    }

    /* Check neighboring tiles above the given input tile. */
    if (ZIdx + Stride < MSTopoSizes[2]) {
	for (i = XIdx; i < XIdx + Stride; i += HS) {
	    for (j = YIdx; j < YIdx + Stride; j += HS) {
		if (MSTopology[i][j][ZIdx + Stride].Stride != HS)
		    return FALSE;
	    }
	}
    }

    /* Find corner indices of this tile block. */
    IRIT_PT2D_SET(CnrX, XIdx, XIdx + Stride - 1);
    IRIT_PT2D_SET(CnrY, YIdx, YIdx + Stride - 1);
    IRIT_PT2D_SET(CnrZ, ZIdx, ZIdx + Stride - 1);
  
    /* Check if the spikes are directly attached to one of the group tiles. */
    /* Check if 01, 10 spikes exists for 00, 11 tiles. */
    for (k = 0; k < 2; k++) {
	for (j = 0; j < 2; j++) {
	    MidY = YIdx + HS - 1 + j;
	    for (i = 0; i < 2; i++) {
		MidX = XIdx + HS - 1 + i;

		if (i == j) {
		    /* Check for spikes in 10 and 01 directions. */
		    if (k == 0) {
			if (!IRIT_PT_APX_EQ(MSTopology[MidX][CnrY[j]][CnrZ[k]].
							    Tile4RayDwnPos[1],
					    VecNone) ||
			    !IRIT_PT_APX_EQ(MSTopology[CnrX[i]][MidY][CnrZ[k]].
							    Tile4RayDwnPos[2],
					    VecNone))
			    return FALSE;
		    }
		    else {
			if (!IRIT_PT_APX_EQ(MSTopology[MidX][CnrY[j]][CnrZ[k]].
							     Tile4RayUpPos[1],
					    VecNone) ||
			    !IRIT_PT_APX_EQ(MSTopology[CnrX[i]][MidY][CnrZ[k]].
							     Tile4RayUpPos[2],
					    VecNone))
			    return FALSE;
		    }		    
		}
		else {
		    /* Check for spikes in 00 and 11 directions. */
		    if (k == 0) {
			if (!IRIT_PT_APX_EQ(MSTopology[MidX][CnrY[j]][CnrZ[k]].
							    Tile4RayDwnPos[0],
					    VecNone) ||
			    !IRIT_PT_APX_EQ(MSTopology[CnrX[i]][MidY][CnrZ[k]].
							    Tile4RayDwnPos[3],
				VecNone))
			    return FALSE;
		    }
		    else {
			if (!IRIT_PT_APX_EQ(MSTopology[MidX][CnrY[j]][CnrZ[k]].
							     Tile4RayUpPos[0],
					    VecNone) ||
			    !IRIT_PT_APX_EQ(MSTopology[CnrX[i]][MidY][CnrZ[k]].
							     Tile4RayUpPos[3],
					    VecNone))
			    return FALSE;
		    }
		}
	    }
	}
    }

    /* Do not merge if spikes exist in the mid of half Stride-sized tiles. */
    if (HS > 1) {
	for (k = 0; k < 2; k++) {
	    for (j = 0; j < 2; j++) {
		for (i = 0; i < 2; i++) {
		    UserMicroSupportTopoInfoStruct *Topo;
		    MidX = XIdx + HS / 2 - 1 + i;
		    MidY = YIdx + HS / 2 - 1 + j;
		    Topo = &MSTopology[MidX][MidY][CnrZ[k]];
		    
		    if (k == 0 ) {
			if (!IRIT_PT_APX_EQ(
			        Topo -> Tile4RayDwnPos[(1 - i) * 2 + (1 - j)],
			        VecNone))
			    return FALSE;
		    }
		    else {
			if (!IRIT_PT_APX_EQ(
			         Topo -> Tile4RayUpPos[(1 - i) * 2 + (1 - j)],
			         VecNone))
			    return FALSE;
		    }
		}
	    }
	}
    }

    return TRUE;
}

/*****************************************************************************
* DESCRIPTION:								     *
*   Count the number of	tiles that can be merged. If there exists a	     *
* consecutive block of Stride x	Stride x Stride	tiles and the size of each   *
* tile is 0.5xStride, the block	of tiles can be	merged into one	single block.*
*									     *
* PARAMETERS:								     *
*   MSTopoSizes:  The sizes in XYZ of MSTopology.			     *
*   MSTopology:	  A 3D grid hold the topology of this grid.		     *
*   Stride:	  The size of each tile	in terms of tile units		     *
*									     *
* RETURN VALUE:								     *
*   int:  The number of	tile groups that can be	merged.			     *
*****************************************************************************/
static int UserMicroCountMergingTiles(
				 const int *MSTopoSizes,
				 UserMicroSupportTopoInfoStruct ***MSTopology,
				 int Stride)
{
    int i, j, k, p, q, r,
	HS = Stride / 2, /* Half Stride. */
	NumTileMerged = 0;

    if (HS < 1)
	return 0;

    for (k = 0; k < MSTopoSizes[2] - Stride + 1; k++) {
	for (j = 0; j < MSTopoSizes[1] - Stride + 1; j++) {
	    i = 0;
	    do {		
		int PurgeArms[8],
		    CanMerge = TRUE;

		/* Check a block of 8 half Stride-sized tiles are valid. */
		for (p = 0; p < 8; p++) {
		    int XIdx, YIdx, ZIdx, XY;
		    ZIdx = k + (p / 4) * HS;
		    XY = p % 4;
		    YIdx = j + (XY / 2) * HS;
		    XIdx = i + (XY % 2) * HS;
		    		    
		    if (MSTopology[XIdx][YIdx][ZIdx].Stride != HS ||
			0 != UserMicroMultiResGetCornerArms(MSTopology, 
							    XIdx, YIdx, ZIdx,
							    HS, PurgeArms)) {
			CanMerge = FALSE;
			break;
		    }
		}
		
		if (!CanMerge ||
		    !UserMicroMultiResCheckNeighborsBeforeMerge(
				  MSTopoSizes, MSTopology, i, j, k, Stride)) {
		    i++;
		    continue;
		}

		/* The eight half Stride-sized tiles are merged into the    */
		/* tile of Stride size. The cell with the smallest indices  */
		/* is marked with +Stride, whereas the remaining cells in   */
		/* the block are marked with -Stride. The tile geometries   */
		/* are stored in cells with positive Stride values.	    */
		for (p = i; p < i + Stride; p++) {
		    for (q = j; q < j + Stride; q++) {
			for (r = k; r < k + Stride; r++) {
			    MSTopology[p][q][r].Stride = -Stride;
			}
		    }
		}
		MSTopology[i][j][k].Stride = Stride;
		NumTileMerged++;

		/* Move to the next tile. */
		i += Stride;
	    } while (i < MSTopoSizes[0] - Stride + 1);
	} 
    } 
    return NumTileMerged;
}

/*****************************************************************************
* DESCRIPTION:								     *
*   Merge a 2 x	2 x 2 block of half Stride-sized tiles that lie	from	     *
* (XIdx, YIdx, ZIdx) in	the 3D topology	grid to	a signel tile. The geometry  *
* of the tile in [0,1]^3 will be generated as a	result.	Vertical bars are    *
* inserted based on whether the	top or bottom neighbors	exist.		     *
*									     *
* PARAMETERS:								     *
*   MSTopoSizes:  The sizes in XYZ of MSTopology.			     *
*   MSTopology:	  A 3D grid hold the topology of this grid.		     *
*   XIdx:	  The smallest X index in the 3D topology grid to be merged. *
*   YIdx:	  The smallest Y index in the 3D topology grid to be merged. *
*   ZIdx:	  The smallest Z index in the 3D topology grid to be merged. *
*   Stride:	  The size of each tile	in terms of tile units		     *
*   TileCntrThickness:	   Center size in the merged tile.		     *
*   TileEndEdgeThickness:  Edge	sizes in the merged tile.		     *
*   TileBaseScale:	   A scale factor to thicken the base arms.	     *
*									     *
* RETURN VALUE:								     *
*   IritPrsrObjectStruct:  The merged	tile in	[0,1]^3.		     *
*****************************************************************************/
IritPrsrObjectStruct *UserMicroGenOneMergedTile(
				 const int *MSTopoSizes,
				 UserMicroSupportTopoInfoStruct ***MSTopology,
				 int XIdx,
				 int YIdx,
				 int ZIdx,
				 int Stride,
				 CagdRType TileCntrThickness,
				 const CagdRType TileEndEdgeThickness[10],
				 CagdRType TileBaseScale)
{
    int i, j, k, 
	HS = Stride/2,
	NeighborCnt = 0;
    CagdRType TileCrnrVertSizes[10],
	CrnrVertScl[2] = { 0.5, 0.5 };
    UserMicroSupportTopoInfoStruct *NeighborTopo[4];
    IritPrsrObjectStruct *Tile;

    if (MSTopology[XIdx][YIdx][ZIdx].Stride != Stride)
	return NULL;

    /* Wipe out half-sized tiles. */
    for (i = 0; i < Stride; i++) {
	for (j = 0; j < Stride; j++) {
	    for (k = 0; k < Stride; k++) {
		UserMicroSupportTopoInfoStruct
		    *Topo = &(MSTopology[XIdx + i][YIdx + j][ZIdx + k]);

		if (Topo -> Tile) {
		    IritMiscAttrIDSetObjectIntAttrib(Topo -> Tile,
						  IRIT_ATTR_CREATE_ID(Invalid),
						  TRUE);
		    Topo -> Tile = NULL;
		}
	    }
	}
    }

    /* Set tile parameters for a new tile. */
    for (i = 0; i < 8; i++)
	TileCrnrVertSizes[i] = TileEndEdgeThickness[i];

    /* If the tile is attached to the base, then thicken the lower arms. */
    if (ZIdx == 0 && TileBaseScale > 1.0) {
	for (i = 0; i < 4; i++)
	    TileCrnrVertSizes[i] *= TileBaseScale;
    }

    TileCrnrVertSizes[8] = TileCrnrVertSizes[9] = 0.0;

    /* Create tile geometry. */
    IRIT_ZAP_MEM(NeighborTopo, sizeof(UserMicroSupportTopoInfoStruct *) * 4);
    
    /* Create vertical arms if the lower/upper neighbor tiles exist. */
    if (ZIdx >= HS) {
	/* Check with the lower neighbors. */
	NeighborTopo[0] = &(MSTopology[XIdx][YIdx][ZIdx - HS]);
	NeighborTopo[1] = &(MSTopology[XIdx + HS][YIdx][ZIdx - HS]);
	NeighborTopo[2] = &(MSTopology[XIdx][YIdx + HS][ZIdx - HS]);
	NeighborTopo[3] = &(MSTopology[XIdx + HS][YIdx + HS][ZIdx - HS]);

	/* Remove the upper arms of the lower neighbors. */
	for (i = 0, NeighborCnt = 0; i < 4; i++) {
	    if (NeighborTopo[i] -> Stride == HS) {
		NeighborCnt++;
	    }
	}
	if (NeighborCnt > 0)
	    TileCrnrVertSizes[8] = TileEndEdgeThickness[8] * -2.0;
    }

    if (ZIdx + Stride < MSTopoSizes[2]) {
	NeighborTopo[0] = &(MSTopology[XIdx][YIdx][ZIdx + Stride]);
	NeighborTopo[1] = &(MSTopology[XIdx + HS][YIdx][ZIdx + Stride]);
	NeighborTopo[2] = &(MSTopology[XIdx][YIdx + HS][ZIdx + Stride]);
	NeighborTopo[3] = &(MSTopology[XIdx + HS][YIdx + HS][ZIdx + Stride]);

	for (i = 0, NeighborCnt = 0; i < 4; i++) {
	    if (NeighborTopo[i] -> Stride == HS) {
		NeighborCnt++;		
	    }
	}
	if (NeighborCnt > 0)
	    TileCrnrVertSizes[9] = TileEndEdgeThickness[9] * -2.0;
    }

    /* Construct the merge tile. */
    Tile = IritUserMicroDiagTile1(TileCntrThickness, 1.0, TileCrnrVertSizes,
				  CrnrVertScl, &TileCrnrVertSizes[8],
				  TRUE, 0.0, NULL, NULL);
    return Tile;
}

/*****************************************************************************
* DESCRIPTION:								     *
*   Merging a 2	x 2 x 2	block of tiles of 0.5 *	Stride-sized. The merged tile*
* will occupy a	Stride x Stride	x Stride block in 3D topology grid. The	     *
* stride parameter in 3D topology grid is updated accordingly; the stride of *
* (0, 0, 0) corner cell	in the block is	set to +Stride,	whereas	the other    *
* cells	in the block is	set to -Stride.					     *
*									     *
* PARAMETERS:								     *
*   MSTopoSizes:  The sizes in XYZ of MSTopology.			     *
*   MSTopology:	  A 3D grid hold the topology of this grid.		     *
*   Stride:	  The size of each tile	in terms of tile units		     *
*   BBoxTV:	  The total dimension of the support.			     *
*   Params:	  The support parameters (for tiles of Stride =	1).	     *
*									     *
* RETURN VALUE:								     *
*   IritPrsrObjectStruct:  A list of merged tiles. Each tile is tagged with  *
*		     the index of its (0,0,0) cell.			     *
*****************************************************************************/
static IritPrsrObjectStruct *UserMicroGenMergedTiles(
			   const int *MSTopoSizes,
			   UserMicroSupportTopoInfoStruct ***MSTopology,
			   int Stride,
			   const TrivTVStruct *BBoxTV,
			   const IritUserMicroGenAMSupportParamStruct *Params)
{
    int i, j, k, TileLvl;
    CagdRType CntrThickness, EdgeThickness[10], TileInt[3];
    CagdBBoxStruct BBox;
    IrtHmgnMatType MatScl, Mat;
    IritPrsrObjectStruct 
	*NewTiles = IritPrsrGenLISTObject(NULL);
    
    IritTrivTVBBox(BBoxTV, &BBox);
    
    CntrThickness = Params -> TileCntrThickness / (CagdRType)Stride;
    if (Params -> MultiResCntrScale > 1.0) {
	for (i = Stride; i > 1; i >>= 1)
	    CntrThickness *= Params -> MultiResCntrScale;
    }
    for (i = 0; i < 10; i++)
	EdgeThickness[i] = Params -> TileEndEdgeThickness / (CagdRType)Stride;
    
    /* for tile transformations. */
    for (i = 0; i < 3; i++) 
	TileInt[i] = (BBox.Max[i] - BBox.Min[i]) / (CagdRType)MSTopoSizes[i];

    IritMiscMatGenMatScale(TileInt[0] * (CagdRType)Stride,
		   TileInt[1] * (CagdRType)Stride,
		   TileInt[2] * (CagdRType)Stride, MatScl);

    /* Find the level of tiles of the given Stride. */
    TileLvl = 0;
    for (i = 1; i < Stride; i *= 2)
	TileLvl++;

    for (k = 0; k < MSTopoSizes[2]; k++) {
	for (j = 0; j < MSTopoSizes[1]; j++) {
	    i = 0;
	    do {
		IritPrsrObjectStruct *Tile;

		if (MSTopology[i][j][k].Stride != Stride) {
		    i++;
		    continue;
		}

		Tile = UserMicroGenOneMergedTile(MSTopoSizes, MSTopology,
						 i, j, k, Stride, 
						 CntrThickness, EdgeThickness,
						 Params -> TileBaseEdgeScale);
		if (Tile) {
		    /* Transform the tile. */
		    char IdxsStr[IRIT_LINE_LEN];

		    IritMiscMatGenMatTrans(
				   BBox.Min[0] + TileInt[0] * (CagdRType) i,
				   BBox.Min[1] + TileInt[1] * (CagdRType) j,
				   BBox.Min[2] + TileInt[2] * (CagdRType) k,
				   Mat);

		    /* Scaling followed by translation.*/
		    IritMiscMatMultTwo4by4(Mat, MatScl, Mat);
		    Tile = IritGeomTransformObjectInPlace(Tile, Mat);
		    
		    IritMiscAttrSetObjectRGBColor(Tile, 200, 200, 255);

		    sprintf(&IdxsStr[0], "%d,%d,%d::%d,%d,%d", i, j, k,
			    MSTopoSizes[0], MSTopoSizes[1], MSTopoSizes[2]);
		    IritMiscAttrIDSetObjectStrAttrib(Tile, 
					     IRIT_ATTR_CREATE_ID(MSIndex), 
					     IdxsStr);
		    /* Save the support tile type and level attributes. */
		    IritUserMicroMarkSupportMSInfo(
					Tile,
					(k == 0) ? USER_MICRO_SUPPORT_BASE :
						   USER_MICRO_SUPPORT_NONBASE,
					TileLvl);

		    MSTopology[i][j][k].Tile = Tile;
		    IritPrsrListObjectAppend(NewTiles, Tile);
		}
		/* Move to the next tile. */
		i += Stride;
	    }
	    while (i < MSTopoSizes[0] - Stride + 1);
	}
    }

    return NewTiles;    
}

/*****************************************************************************
* DESCRIPTION:								     *
*   Merging a 2	x 2 x 2	block of "parameteric" tiles of 0.5 * Stride-sized.  *
* Like nonparametric tiles, the merged tile will occupy a Stride x Stride x  *
* Stride block in 3D topology grid. The stride parameter in 3D topology grid *
* is updated accordingly; the stride of (0, 0, 0) corner cell in the block   *
* is set to +Stride, whereas the other cells in the block is set to -Stride. *
*   Center and corner thickness of the merged tiles are computed accordingly;*
* the thickness at the base level (with Stride = 1) are first evaluated with *
* the scalar trivariate; and then the thickness is scaled by 1/Stride. The   *
* arm thickness of vertical rods are evaluated on the center of bottom and   *
* top faces of the tile domain (and used only when required).		     *
*									     *
* PARAMETERS:								     *
*   MSTopoSizes:  The sizes in XYZ of MSTopology.			     *
*   MSTopology:	  A 3D grid hold the topology of this grid.		     *
*   Stride:	  The size of each tile	in terms of tile units		     *
*   BBoxTV:	  The total dimension of the support.			     *
*   Params:	  The support parameters (for tiles of Stride =	1).	     *
*									     *
* RETURN VALUE:								     *
*   IritPrsrObjectStruct:  A list of merged tiles. Each tile is tagged with  *
*		   the index of its (0,0,0) cell.			     *
*****************************************************************************/
static IritPrsrObjectStruct *UserMicroGenMergedPrmTiles(
			   const int *MSTopoSizes,
			   UserMicroSupportTopoInfoStruct ***MSTopology,
			   int Stride,
			   const TrivTVStruct *BBoxTV,
			   const IritUserMicroGenAMSupportParamStruct *Params)
{
    int i, j, k, l, TileLvl;
    CagdRType TileInt[3], UVWInt[3];
    CagdBBoxStruct BBox, PrmTVDmn;
    IrtHmgnMatType MatScl, Mat;
    IritPrsrObjectStruct 
	*NewTiles = IritPrsrGenLISTObject(NULL);
    const TrivTVStruct
	*PrmTV = Params -> PrmThicknessTV;

    assert(PrmTV != NULL);

    /* for parametric thickness. */
    IritTrivTVDomain(PrmTV, &PrmTVDmn.Min[0], &PrmTVDmn.Max[0],
			    &PrmTVDmn.Min[1], &PrmTVDmn.Max[1],
			    &PrmTVDmn.Min[2], &PrmTVDmn.Max[2]);
    for (i = 0; i < 3; i++)
	UVWInt[i] = (PrmTVDmn.Max[i] - PrmTVDmn.Min[i]) 
						  / (CagdRType)MSTopoSizes[i];

    /* for tile transformations. */
    IritTrivTVBBox(BBoxTV, &BBox);
    for (i = 0; i < 3; i++) 
	TileInt[i] = (BBox.Max[i] - BBox.Min[i]) / (CagdRType)MSTopoSizes[i];

    IritMiscMatGenMatScale(TileInt[0] * (CagdRType)Stride,
		   TileInt[1] * (CagdRType)Stride,
		   TileInt[2] * (CagdRType)Stride, MatScl);

    /* Find the level of tiles of the given Stride. */
    TileLvl = 0;
    for (i = 1; i < Stride; i *= 2)
	TileLvl++;

    for (k = 0; k < MSTopoSizes[2]; k++) {
	for (j = 0; j < MSTopoSizes[1]; j++) {
	    i = 0;
	    do {
		CagdRType Pt[3], CntrThickness, CrnrThickness[10];
		CagdBBoxStruct UVWDmn;
		IritPrsrObjectStruct *Tile;

		if (MSTopology[i][j][k].Stride != Stride) {
		    i++;
		    continue;
		}

		/* Find uvw parameters to compute corner and center	    */
		/* thickness.						    */
		CAGD_BBOX_INIT_3D(UVWDmn);
		IRIT_PT_SET(UVWDmn.Min, UVWInt[0] * (CagdRType)i,
					UVWInt[1] * (CagdRType)j,
					UVWInt[2] * (CagdRType)k);
		IRIT_PT_SCALE_AND_ADD(UVWDmn.Max, UVWDmn.Min,
				      UVWInt, (CagdRType)Stride);

		/* Compute corner thickness (in the base resolution). */
		for (l = 0; l < 8; l++) {
		    int UVId = l % 4;
		    Pt[0] = (UVId % 2 == 0) ? UVWDmn.Min[0] : UVWDmn.Max[0];
		    Pt[1] = (UVId / 2 == 0) ? UVWDmn.Min[1] : UVWDmn.Max[1];
		    Pt[2] = (l / 4 == 0) ? UVWDmn.Min[2] : UVWDmn.Max[2];
		    TRIV_TV_EVAL_SCALAR(PrmTV, Pt[0], Pt[1], Pt[2],
					&CrnrThickness[l]);		    
		}

		/* Compute center thickness (in the base resolution). */
		IRIT_PT_BLEND(Pt, UVWDmn.Min, UVWDmn.Max, 0.5);
		TRIV_TV_EVAL_SCALAR(PrmTV, Pt[0], Pt[1], Pt[2], 
				    &CntrThickness);
		
		/* Thickness of vertical rods. */
		TRIV_TV_EVAL_SCALAR(PrmTV, Pt[0], Pt[1], UVWDmn.Min[2], 
				    &CrnrThickness[8]);
		TRIV_TV_EVAL_SCALAR(PrmTV, Pt[0], Pt[1], UVWDmn.Max[2],
				    &CrnrThickness[9]);

		/* Apply multi-resolution scales. */
		for (l = 0; l < 10; l++) {
		    CrnrThickness[l] /= (CagdRType)Stride;
		    if (CrnrThickness[l] > 0.5 - IRIT_EPS)
			CrnrThickness[l] = 0.5 - IRIT_EPS;
		}
		
		CntrThickness /= (CagdRType)Stride;
		if (Params -> MultiResCntrScale > 1.0) {
		    for (l = Stride; l > 1; l >>= 1)
			CntrThickness *= Params -> MultiResCntrScale;
		}
		if (CntrThickness > 0.5 - IRIT_EPS)
		    CntrThickness = 0.5 - IRIT_EPS;

		Tile = UserMicroGenOneMergedTile(MSTopoSizes, MSTopology,
						 i, j, k, Stride, 
						 CntrThickness, CrnrThickness,
						 Params -> TileBaseEdgeScale);
		if (Tile) {
		    /* Transform the tile. */
		    char IdxsStr[IRIT_LINE_LEN];

		    IritMiscMatGenMatTrans(
				     BBox.Min[0] + TileInt[0] * (CagdRType) i,
				     BBox.Min[1] + TileInt[1] * (CagdRType) j,
				     BBox.Min[2] + TileInt[2] * (CagdRType) k,
				     Mat);

		    /* Scaling followed by translation.*/
		    IritMiscMatMultTwo4by4(Mat, MatScl, Mat);
		    Tile = IritGeomTransformObjectInPlace(Tile, Mat);
		    
		    IritMiscAttrSetObjectRGBColor(Tile, 200, 200, 255);

		    sprintf(&IdxsStr[0], "%d,%d,%d::%d,%d,%d", i, j, k,
			    MSTopoSizes[0], MSTopoSizes[1], MSTopoSizes[2]);
		    IritMiscAttrIDSetObjectStrAttrib(Tile, 
					     IRIT_ATTR_CREATE_ID(MSIndex), 
					     IdxsStr);
		    /* Save the support tile type and level attributes. */
		    IritUserMicroMarkSupportMSInfo(
					Tile,
					(k == 0) ? USER_MICRO_SUPPORT_BASE :
						   USER_MICRO_SUPPORT_NONBASE,
					TileLvl);

		    MSTopology[i][j][k].Tile = Tile;
		    IritPrsrListObjectAppend(NewTiles, Tile);
		}
		/* Move to the next tile. */
		i += Stride;
	    }
	    while (i < MSTopoSizes[0] - Stride + 1);
	}
    }

    return NewTiles;    
}

/*****************************************************************************
* DESCRIPTION:								     *
*   Filter the arms of multi-resolution	tiles based on the information of    *
* active arms of neighboring tiles. When 8 half-sized tiles are	merged into  *
* a single tile	of size	= Stride, filtering is done in the following steps:  *
*   1. All arms	of all tiles in	(Stride	x Stride x Stride) block are wiped   *
*      out except 8 corner arms	of 8 corner tiles. However, if these corner  *
*      arms are	inactive before	merging, they will remain inactive after     *
*      merging as well.							     *
*   2. Check neighboring tiles along Z-direction of the	merged tile. Arms of *
*      the neighbor tiles that contact the mid point of	boundary of merged   *
*      tiles are purged. That is, if the merged	tile occupies [0, 2] x [0, 2]*
*      x [0, 2], the arms of the Z-neighboring tiles contacting	 (0,+1/-1,0),*
*      (0,+1/-1, 2), (+1/-1, 0,	2), (+1/-1, 0, 2) will be marked as inactie. *
*   3. For the arms of X-neighboring and Y-neighboring tiles, mark them	as   *
*      inactive	when they face X (or Y)	face of	the merged tile, and	     *
*      the arms	are also attached to other structures, including the arms,   *
*      ray spikes, or then base	plane. If these	mid arms are explosed to the *
*      air, they are marked as inactive.				     *
*									     *
* PARAMETERS:								     *
*   MSTopology:	  A 3D grid hold the topology of this grid.		     *
*   MSTopoSizes:  The sizes in XYZ of MSTopology.			     *
*   XIdx:	  The smallest X index of the multi-resolution tile in the   *
*		  3D topology grid.					     *
*   YIdx:	  The smallest Y index of the multi-resolution tile in the   *
*		  3D topology grid.					     *
*   ZIdx:	  The smallest Z index of the multi-resolution tile in the   *
*		  3D topology grid.					     *
*									     *
* RETURN VALUE:								     *
*   VOID								     *
*****************************************************************************/
static void UserMicroMultiResFilterActiveArms(
				 const int *MSTopoSizes,
				 UserMicroSupportTopoInfoStruct ***MSTopology,
				 int XIdx,
				 int YIdx,
				 int ZIdx)
{
    int i, j, k, l, OldActiveArms[8], CnrX[2], CnrY[2], CnrZ[2], Stride, HS;
	
    /* Size of the current tile. */
    Stride = MSTopology[XIdx][YIdx][ZIdx].Stride;
    HS = Stride / 2;			  
   
    if (HS < 1)
	return;

    /* Identify the indices of corner topologies of a new tile. */
    IRIT_PT2D_SET(CnrX, XIdx, XIdx + Stride - 1);
    IRIT_PT2D_SET(CnrY, YIdx, YIdx + Stride - 1);
    IRIT_PT2D_SET(CnrZ, ZIdx, ZIdx + Stride - 1);


    /* Remove all arms from half-sized topologies, except eight arms at	    */
    /* corner tiles. These eight arms are kept if they were previously	    */
    /* active in half - sized topologies.				    */
    /* Initially a new tile after merging has all 8 arms. */
    IRIT_ZAP_MEM(OldActiveArms, sizeof(int) * 8);

    /* Record the activeness of eight corner arms from half-sized topos.    */
    for (k = 0; k < 2; k++)
	for (j = 0; j < 2; j++)
	    for (i = 0; i < 2; i++) {
		int KeepArmId = 4 * (1 - k) + 2 * j + i;
		UserMicroSupportTopoInfoStruct
		    *Topo = &(MSTopology[CnrX[i]][CnrY[j]][CnrZ[k]]);

		OldActiveArms[4 * k + 2 * j + i] 
					      = Topo -> ActiveArms[KeepArmId];
	    }

    /* Wipe out all active arms from all topologies to be merged. */
    for (k = 0; k < Stride; k++)
	for (j = 0; j < Stride; j++)
	    for (i = 0; i < Stride; i++) {
		UserMicroSupportTopoInfoStruct
		    *Topo = &(MSTopology[XIdx + i][YIdx + j][ZIdx + k]);

		for (l = 0; l < 8; l++)
		    Topo -> ActiveArms[l] = TRUE;
	    }

    /* Mark the activeness of eight corner arms of a new tile. */
    for (k = 0; k < 2; k++)
	for (j = 0; j < 2; j++)
	    for (i = 0; i < 2; i++) {
		int ArmId = 4 * (1 - k) + 2 * j + i;

		MSTopology[CnrX[i]][CnrY[j]][CnrZ[k]].ActiveArms[ArmId]
					   = OldActiveArms[4 * k + 2 * j + i];
	    }

    /* Adjust active arms in the adjacent tiles. */
    /* Along the Z-directions : the middle active arms are purged. */
    for (k = 0; k < 2; k++) {
	int AdjZId,
	    MidX = CnrX[0] + HS - 1,
	    MidY = CnrY[0] + HS - 1;

	if (k == 0 && CnrZ[k] > 0) 
	    AdjZId = CnrZ[k] - 1;
	else if (k == 1 && CnrZ[k] < MSTopoSizes[2] - 1) 
	    AdjZId = CnrZ[k] + 1;
	else
	    continue;

	/* Remove mid-X direction arms. */
	MSTopology[MidX][CnrY[0]][AdjZId].ActiveArms[1 + k * 4] = TRUE;
	MSTopology[MidX][CnrY[1]][AdjZId].ActiveArms[3 + k * 4] = TRUE;
	MSTopology[MidX + 1][CnrY[0]][AdjZId].ActiveArms[0 + k * 4] = TRUE;
	MSTopology[MidX + 1][CnrY[1]][AdjZId].ActiveArms[2 + k * 4] = TRUE;

	/* Remove mid-Y direction arms. */
	MSTopology[CnrX[0]][MidY][AdjZId].ActiveArms[2 + k * 4] = TRUE;
	MSTopology[CnrX[1]][MidY][AdjZId].ActiveArms[3 + k * 4] = TRUE;
	MSTopology[CnrX[0]][MidY + 1][AdjZId].ActiveArms[0 + k * 4] = TRUE;
	MSTopology[CnrX[1]][MidY + 1][AdjZId].ActiveArms[1 + k * 4] = TRUE;
    }

    /* For the adjacent tiles along X-directions: keep the mid arm only if  */
    /* there exist some Z-neighboring structures (tiles or spikes).	    */
    for (i = 0; i < 2; i++) {
	int AdjXId,
	    MidY = CnrY[0] + HS - 1;

	if (i == 0 && CnrX[i] > 0) 
	    AdjXId = CnrX[i] - 1;
	else if (i == 1 && CnrX[i] < MSTopoSizes[0] - 1) 
	    AdjXId = CnrX[i] + 1;
	else
	    continue;

	/* Attempt to remove the lower arms from mid-Y tiles. */	
	for (k = CnrZ[0]; k < CnrZ[1] + 1; k += HS) {
	    for (j = 0; j < 2; j++) {
		UserMicroSupportTopoInfoStruct
		    *Topo = &(MSTopology[AdjXId][MidY + j][k]);

		for (l = 0; l < 2; l++) {
		    /* If a tile is not a base, the lower tile does not */
		    /* exist, and there is no downward spike attached, 	*/
		    /* then remove the lower arms from  mid-Y tiles.	*/
		    if (k != 0 &&
			MSTopology[AdjXId][MidY + j][k - 1].Stride == 0 &&
			IRIT_PT_APX_EQ(Topo -> Tile4RayDwnPos[3 - (2 * l + i)],
				       VecNone)) {
			Topo -> ActiveArms[7 - (2 * l + i)] = TRUE;
		    }
		}
	    }
	}
	
	/* Attempt to remove the upper from mid-Y tiles.  */
	for (k = CnrZ[0] + HS - 1; k < CnrZ[1] + 1; k += HS) {
	    for (j = 0; j < 2; j++) {
		UserMicroSupportTopoInfoStruct
		    *Topo = &(MSTopology[AdjXId][MidY + j][k]);

		for (l = 0; l < 2; l++) {
		    /* If the upper tile does not exist, and there is no */
		    /* upward spike attached, then remove the upper arms */
		    /* from mid-Xv tiles.				 */
		    if (k < MSTopoSizes[2] - 1 &&
			MSTopology[AdjXId][MidY + j][k + 1].Stride == 0 &&
			IRIT_PT_APX_EQ(Topo -> Tile4RayUpPos[3 - (2 * l + i)], 
				       VecNone)) {
			Topo -> ActiveArms[3 - (2 * l + i)] = TRUE;
		    }
		}
	    }
	}	
    }

    /* For the adjacent tiles along Y-directions: keep the mid arms only if */
    /* there exists some Z-neighboring structures (tiles or spikes).	    */
    for (j = 0; j < 2; j++) {
	int AdjYId,
	    MidX = CnrX[0] + HS - 1;

	if (j == 0 && CnrY[j] > 0)
	    AdjYId = CnrY[j] - 1;
	else if (j == 1 && CnrY[j] < MSTopoSizes[1] - 1)
	    AdjYId = CnrY[j] + 1;
	else
	    continue;

	/* Attempt to remove the lower arms from mid-X tiles. */	
	for (k = CnrZ[0]; k < CnrZ[1] + 1; k += HS) {
	    for (i = 0; i < 2; i++) {
		UserMicroSupportTopoInfoStruct
		    *Topo = &(MSTopology[MidX + i][AdjYId][k]);
		for (l = 0; l < 2; l++) {
		    /* If a tile is not a base, the lower tile does not */
		    /* exist, and there is no downward spike attached,  */
		    /* then remove the lower arms from mid-X tiles.	*/
		    if (k != 0 &&
			MSTopology[MidX + i][AdjYId][k - 1].Stride == 0 &&
			IRIT_PT_APX_EQ(Topo -> Tile4RayDwnPos[3 - (2 * l + i)],
				       VecNone)) {
			Topo -> ActiveArms[7 - (2 * l + i)] = TRUE;
		    }
		}
	    }		
	}

	/* Attempt to remove the upper arms from mid-X tiles. */
	for (k = CnrZ[0] + HS - 1; k < CnrZ[1] + 1; k += HS) {
	    for (i = 0; i < 2; i++) {
		UserMicroSupportTopoInfoStruct
		    *Topo = &(MSTopology[MidX + i][AdjYId][k]);
		for (l = 0; l < 2; l++) {
		    /* If the lower tile does not exist, and there is no  */
		    /* upward spike attached, then remove the upper arms  */
		    /* from mid-X tiles.				  */
		    if (k < MSTopoSizes[2] - 1 &&
			MSTopology[MidX + i][AdjYId][k + 1].Stride == 0 &&
			IRIT_PT_APX_EQ(Topo -> Tile4RayUpPos[3 - (2 * l + i)],
				       VecNone)) {
			Topo -> ActiveArms[3 - (2 * l + i)] = TRUE;
		    }
		}
	    }
	}
    }
}

/*****************************************************************************
* DESCRIPTION:								     *
*    If	an arm of a tile is dangling in	the air	without	contacting any other *
* arms from neighboring	tiles face-to-face, having any upward/downward ray   *
* spike	attached, or being attached to the base	plane, then mark the arm as  *
* being	purged.								     *
*									     *
* PARAMETERS:								     *
*   MSTopology:	  A 3D grid hold the topology of this grid.		     *
*   MSTopoSizes:  The sizes in XYZ of MSTopology.			     *
*   XIdx:	  The smallest X index of the multi-resolution tile in the   *
*		  3D topology grid.					     *
*   YIdx:	  The smallest Y index of the multi-resolution tile in the   *
*		  3D topology grid.					     *
*   ZIdx:	  The smallest Z index of the multi-resolution tile in the   *
*		  3D topology grid.					     *
*									     *
* RETURN VALUE:								     *
*   int:	TRUE if	new purging arms are detected.			     *
*****************************************************************************/
static int UserMicroMultiResFilterDanglingArms(
				 const int *MSTopoSizes,
				 UserMicroSupportTopoInfoStruct ***MSTopology,
				 int XIdx, 
				 int YIdx, 
				 int ZIdx)
{
    int i, j, k, CnrX[2], CnrY[2], CnrZ[2], Stride,
	MadeChange = FALSE;
    IRIT_STATIC_DATA const int
	NInfo[8][3] = {     /* The neighbor's corresponding arm information */
	    {1, 2, 4}, {0, 3, 5}, {3, 0, 6}, {2, 1, 7},   /* of each arm of */
	    {5, 6, 0}, {4, 7, 1}, {7, 4, 2}, {6, 5, 3}         /* the tile. */
	},
	SignInfo[8][3] = {     /* Sign to find neighbor tiles that arms are */
	    {-1, -1, 1}, {1, -1, 1}, {-1, 1, 1}, {1, 1, 1},/* touching each */
	    {-1, -1, -1}, {1, -1, -1}, {-1, 1, -1}, {1, 1, -1}    /* other. */
	};

    Stride = MSTopology[XIdx][YIdx][ZIdx].Stride;

    IRIT_PT2D_SET(CnrX, XIdx, XIdx + Stride - 1);
    IRIT_PT2D_SET(CnrY, YIdx, YIdx + Stride - 1);
    IRIT_PT2D_SET(CnrZ, ZIdx, ZIdx + Stride - 1);

    for (k = 0; k < 2; k++) {
	for (j = 0; j < 2; j++) {
	    for (i = 0; i < 2; i++) {
		UserMicroSupportTopoInfoStruct *CurTopo, *NeigTopo;
		int NeigId,
		    ArmId = 4 * (1 - k) + 2 * j + i,
		    NeigArm = FALSE;

		CurTopo = &MSTopology[CnrX[i]][CnrY[j]][CnrZ[k]];
		if (CurTopo -> ActiveArms[ArmId])
		    continue;

		/* If there exist at least one neighbor along X, Y, or Z    */
		/* direction that has an arm contacting with the given arm  */
		/* do not purge the arm.				    */
		NeigId = CnrX[i] + SignInfo[ArmId][0];
		if (NeigId >= 0 && NeigId < MSTopoSizes[0]) {
		    NeigTopo = &MSTopology[NeigId][CnrY[j]][CnrZ[k]];
		    if (!NeigTopo -> ActiveArms[NInfo[ArmId][0]])
			NeigArm = TRUE;
		}

		NeigId = CnrY[j] + SignInfo[ArmId][1];
		if (!NeigArm && NeigId >= 0 && NeigId < MSTopoSizes[1]) {
		    NeigTopo = &MSTopology[CnrX[i]][NeigId][CnrZ[k]];
		    if (!NeigTopo -> ActiveArms[NInfo[ArmId][1]])
			NeigArm = TRUE;
		}

		NeigId = CnrZ[k] + SignInfo[ArmId][2];
		if (!NeigArm && NeigId >= 0 && NeigId < MSTopoSizes[2]) {
		    NeigTopo = &(MSTopology[CnrX[i]][CnrY[j]][NeigId]);
		    if (!NeigTopo -> ActiveArms[NInfo[ArmId][2]])
			NeigArm = TRUE;
		}

		if (!NeigArm) {
		    if (k == 0 && 
			CnrZ[k] != 0 &&
			IRIT_PT_APX_EQ(CurTopo -> Tile4RayDwnPos[2 * j + i], 
				       VecNone)) {
			CurTopo -> ActiveArms[ArmId] = TRUE;
			MadeChange = TRUE;
		    }
		    else if (k == 1 &&
			     IRIT_PT_APX_EQ(
					  CurTopo -> Tile4RayUpPos[2 * j + i],
					  VecNone)) {
			CurTopo -> ActiveArms[ArmId] = TRUE;
			MadeChange = TRUE;
		    }		    
		}
	    }
	}
    }

    return MadeChange;
}

/*****************************************************************************
* DESCRIPTION:								     *
*    Check if up or down rays are directly shot	from the given tile of	     *
* multiresolution, having (XIdx, YIdx, ZIdx) as	its smallest indices and     *
* Stride tile size. 8 rays shot	from 8 corner cells of the multiresolution   *
* tile is tested and the results are stored on RaysOnArm[8]. For the tile    *
* having the size(stride) larger than 1, the additional	rays shot from the   *
* middle of the	multiresolution	tile are also tested.			     *
*									     *
* PARAMETERS:								     *
*   MSTopology:	  A 3D grid hold the topology of this grid.		     *
*   XIdx:	  The smallest X index of the multi-resolution tile in the   *
*		  3D topology grid.					     *
*   YIdx:	  The smallest Y index of the multi-resolution tile in the   *
*		  3D topology grid.					     *
*   ZIdx:	  The smallest Z index of the multi-resolution tile in the   *
*		  3D topology grid.					     *
*   Stride:	  The size of the multi	resolution tile.		     *
*   UpDwnRay:	  0x01 if only down rays are tested.			     *
*		  0x10 if only up rays are tested.			     *
*		  0x11 if both up and down rays	are tested.		     *
*   RaysOnArm:	  True if rays are directly shot from the individual arms.   *
*									     *
* RETURN VALUE:								     *
*   int:	TRUE if	any rays are directly shot from	the given tile.	     *
*****************************************************************************/
static int UserMicroMultiResCheckRaysOnTile(
				 UserMicroSupportTopoInfoStruct ***MSTopology,
				 int XIdx, 
				 int YIdx, 
				 int ZIdx, 
				 int UpDwnRay,
				 int RaysOnArm[8])
{
    int i, j, CnrX[2], CnrY[2],
	ST = MSTopology[XIdx][YIdx][ZIdx].Stride,
	HS = ST / 2,
	RaysOnTile = FALSE;

     /* Check for up rays. */
    if (UpDwnRay & 0x10) {
	if (ST == 1) {
	    for (i = 0; i < 4; i++) {
		if (!IRIT_PT_APX_EQ(
			MSTopology[XIdx][YIdx][ZIdx].Tile4RayUpPos[i],
			VecNone)) {
		    RaysOnArm[i] = TRUE;
		    RaysOnTile = TRUE;
		}
	    }
	}
	else {
	    /* Check the rays from four half-sized tiles. */
	    for (j = 0; j < 2; j++) {
		IRIT_PT2D_SET(CnrY, YIdx + HS * j, YIdx + HS * j + HS - 1);
		for (i = 0; i < 2; i++) {
		    IRIT_PT2D_SET(CnrX, 
				  XIdx + HS * i, XIdx + HS * i + HS - 1);

		    /* Check rays on corner arms. */
		    if (!IRIT_PT_APX_EQ(
			    MSTopology[CnrX[i]][CnrY[j]][ZIdx + ST - 1].
						     Tile4RayUpPos[2 * j + i],
			    VecNone)) {
			RaysOnArm[2 * j + i] = TRUE;
			RaysOnTile = TRUE;
		    }

		    /* When the stride > 1, check rays for arms diagonal to */
		    /* corner arms.					    */
		    if (HS > 1 &&
			!IRIT_PT_APX_EQ(
			    MSTopology[CnrX[1 - i]][CnrY[1 - j]][ZIdx + ST - 1].
					       Tile4RayUpPos[3 - (2 * j + i)],
			    VecNone)) {
			RaysOnTile = TRUE;
		    }
		}
	    }
	}
    }

    /* Check for down rays. */
    if (UpDwnRay & 0x01) {
	if (ST == 1) {
	    for (i = 0; i < 4; i++) {
		if (!IRIT_PT_APX_EQ(
			MSTopology[XIdx][YIdx][ZIdx].Tile4RayDwnPos[i],
			VecNone)) {
		    RaysOnArm[4 + i] = TRUE;
		    RaysOnTile = TRUE;
		}
	    }
	}
	else {
	    /* Check the rays from four half-sized tiles. */
	    for (j = 0; j < 2; j++) {
		IRIT_PT2D_SET(CnrY, YIdx + HS * j, YIdx + HS * j + HS - 1);
		for (i = 0; i < 2; i++) {
		    IRIT_PT2D_SET(CnrX, 
				  XIdx + HS * i, XIdx + HS * i + HS - 1);

		    /* Check rays on corner arms. */
		    if (!IRIT_PT_APX_EQ(MSTopology[CnrX[i]][CnrY[j]][ZIdx].
						    Tile4RayDwnPos[2 * j + i],
					VecNone)) {
			RaysOnArm[4 + 2 * j + i] = TRUE;
			RaysOnTile = TRUE;
		    }

		    /* When the stride > 1, check rays for arms diagonal to */
		    /* corner arms.					    */
		    if (HS > 1 &&
			!IRIT_PT_APX_EQ(
			    MSTopology[CnrX[1 - i]][CnrY[1 - j]][ZIdx].
					      Tile4RayDwnPos[3 - (2 * j + i)],
			    VecNone)) {
			RaysOnTile = TRUE;
		    }
		}
	    }
	}
    }

    return RaysOnTile;
}

/*****************************************************************************
* DESCRIPTION:								     *
*   This routine is the	multiresolution	version	of			     *
* UserMicroFilterInactiveTilesAndArms, which filters out multiresolution     *
* tiles	that have no contribution to the support as follows.		     *
* 1. If	a tile has no up rays or other tile(s) attached, it is considered a  *
*    top tile to be purged, and	purge it.				     *
* 2. If	a tile is a top	tile and not all its four corners are providing	     *
*    support, remove top arms that are supporting nothing. The same applies  *
*    to	non-base bottom	tiles.						     *
* 3. If	a tile has no down arms	or other tile(s) attached, it is considered  *
*    a bottom tile to be purged, and purge it.				     *
*									     *
* PARAMETERS:								     *
*   MSTopoSizes:   The size of the 3D grid of tiles.			     *
*   MSTopology:	   The topology	information of MS.			     *
*									     *
* RETURN VALUE:								     *
*   void								     *
*****************************************************************************/
static void UserMicroMultiResFilterInactiveTilesAndArms(
				 const int *MSTopoSizes,
				 UserMicroSupportTopoInfoStruct ***MSTopology)
{
    int i, j, k, n, x, y, z, ST, HS, RemoveTile, 
	RaysOnArms[8],
	MadeChanges = FALSE;
    UserMicroSupportTopoInfoStruct *TlTopo, *UpTopo;

    /* 1. Remove top tiles that offer no support. */
    do {
	MadeChanges = FALSE;
	for (k = MSTopoSizes[2] - 1; k >= 0; k--) {
	    for (j = 0; j < MSTopoSizes[1]; j++) {
		i = 0;
		do {
		    TlTopo = &MSTopology[i][j][k];
		    ST = TlTopo -> Stride;
		    if (ST <= 0) {
			i++;
			continue;
		    }
		    
		    /* If there are any rays shot from one of four corner   */
		    /* arms or from the middle vertical arm (if exists),    */
		    /* this tile cannot be removed.			    */
		    if (UserMicroMultiResCheckRaysOnTile(
				     MSTopology, i, j, k, 0x10, RaysOnArms)) {
			i += ST;
			continue;
		    }

		    /* If no rays shot and there are some tiles above it,   */
		    /* the tile cannot be removed as well.		    */
		    RemoveTile = TRUE;
		    HS = ST / 2;
		    if (k + ST < MSTopoSizes[2]) {
			for (y = 0; y < 2; y++) {
			    for (x = 0; x < 2; x++) {
				if (MSTopology[i + HS * x][j + HS * y][k + ST].
								Stride != 0) {
				    RemoveTile = FALSE;
				    break;
				}
			    }
			    if (!RemoveTile)
				break;
			}
		    }

		    if (RemoveTile) {
			MadeChanges = TRUE;
			IritMiscAttrIDSetObjectIntAttrib(TlTopo -> Tile,
						 IRIT_ATTR_CREATE_ID(Invalid),
						 TRUE);
			TlTopo -> Tile = NULL;

			/* Wipe-out stride informaiton in the tile. */
			for (x = i; x < i + ST; x++)
			    for (y = j; y < j + ST; y++)
				for (z = k; z < k + ST; z++) {
				    MSTopology[x][y][k].Stride = 0;
				    for (n = 0; n < 8; n++)
					MSTopology[x][y][k].ActiveArms[n]
								     = TRUE;
				}			
		    }
		    i += ST;
		}
		while (i < MSTopoSizes[0]);
	    }
	}
    }
    while (MadeChanges);

    /* 2a. Mark ActiveArms depending on whether tile require arms or not. */
    for (k = 0; k < MSTopoSizes[2]; k++) {
	for (j = 0; j < MSTopoSizes[1]; j++) {
	    i = 0;
	    do {
		TlTopo = &MSTopology[i][j][k];
		ST = TlTopo -> Stride;
		if (ST <= 0) {
		    i++;
		    continue;
		}
		
		UserMicroMultiResCheckRaysOnTile(MSTopology, i, j, k, 0x11, 
						 RaysOnArms);
		/* For top and bottom tiles, check if rays are shot from    */
		/* corner arms.						    */
		HS = ST / 2;
		for (y = 0; y < 2; y++) {
		    for (x = 0; x < 2; x++) {
			if (k + ST == MSTopoSizes[2] ||
			    MSTopology[i + HS * x][j + HS * y][k + ST].Stride
								      == 0) {
			    if (!RaysOnArms[2 * y + x])
				TlTopo -> ActiveArms[2 * y + x] = TRUE;
			}

			if (k != 0 &&
			    MSTopology[i + HS * x][j + HS * y][k - 1].Stride
								      == 0) {
			    if (!RaysOnArms[4 + 2 * y + x])
				TlTopo -> ActiveArms[4 + 2 * y + x] = TRUE;
			}
		    }
		}
		i += ST;
	    }
	    while (i < MSTopoSizes[0]);
	}
    }

    /* 2b. If arms of a neighboring tile and the arms of the tile can face  */
    /*     each other, mark those arms as required. */
    do {
	MadeChanges = FALSE;
	for (k = 0; k < MSTopoSizes[2]; k++) {
	    for (j = 0; j < MSTopoSizes[1]; j++) {
		i = 0;
		do {
		    TlTopo = &MSTopology[i][j][k];
		    ST = TlTopo -> Stride;
		    if (ST <= 0) {
			i++;
			continue;
		    }

		    MadeChanges = UserMicroMultiResFilterDanglingArms(
					  MSTopoSizes, MSTopology, i, j, k) ?
				  TRUE : MadeChanges;

		    i += ST;		    
		}
		while (i < MSTopoSizes[0]);
	    }
	}
    } 
    while (MadeChanges);

    /* 3. Remove bottom tiles that all down arms are inactive. */
    do {
	MadeChanges = FALSE;
	for (k = 0; k < MSTopoSizes[2]; k++) {
	    for (j = 0; j < MSTopoSizes[1]; j++) {
		i = 0;
		do {
		    int PurgeArms[8];
		    TlTopo = &MSTopology[i][j][k];
		    ST = TlTopo -> Stride;
		    if (ST <= 0) {
			i++;
			continue;
		    }

		    HS = ST / 2;

		    /* Get filtering information of arms. */
		    UserMicroMultiResGetCornerArms(MSTopology, i, j, k, ST, 
						   PurgeArms);

		    if (PurgeArms[4] && PurgeArms[5] && PurgeArms[6] && 
			PurgeArms[7]) {
			MadeChanges = TRUE;
			IritMiscAttrIDSetObjectIntAttrib(TlTopo -> Tile,
						 IRIT_ATTR_CREATE_ID(Invalid),
						 TRUE);
			TlTopo -> Tile = NULL;
			for (x = i; x < i + ST; x++)
			    for (y = j; y < j + ST; y++)
				for (z = k; z < k + ST; z++) {
				    MSTopology[x][y][z].Stride = 0;
				    for (n = 0; n < 8; n++)
					MSTopology[x][y][z].ActiveArms[n] 
								       = TRUE;
				}

			/* Check the tile(s) above the current tile. */
			if (k + ST < MSTopoSizes[2]) {
			    /* Arms of the upper tiles connected to the	    */
			    /* vertical arms of the current tile are always */
			    /* deleted.					    */
			    for (y = 0; y < 2; y++) {
				for (x = 0; x < 2; x++) {
				    UpTopo = &(MSTopology[i + HS * x]
				    			[j + HS * y][k + ST]);
				    if (UpTopo -> Stride == HS) {
					UpTopo -> ActiveArms[7 - (2 * y + x)] 
								       = TRUE;
				    }
				}
			    }

			    /* Arms of the upper tiles connected to the	    */
			    /* diagonal arms of the current tile are only   */
			    /* deleted when there exist no XY neighbors.    */
			    for (y = 0; y < 2; y++) {
				for (x = 0; x < 2; x++) {
				    UpTopo = &(MSTopology[i + HS * x]
				    			[j + HS * y][k + ST]);
				    if (UpTopo -> Stride > 0) {
					MadeChanges = 
					  UserMicroMultiResFilterDanglingArms(
					      MSTopoSizes, MSTopology, 
					      i + HS * x, j + HS * y, k + ST) ?
					  TRUE : MadeChanges;
				    }
				}
			    }
			}
		    }
		    i += ST;
		}
		while (i < MSTopoSizes[0]);
	    }
	}
    } 
    while (MadeChanges);
}

/*****************************************************************************
* DESCRIPTION:								     *
*   Before this	function is called, the	information of active arms of multi- *
* resolution tiles are spread out in 8 corner tiles among the tile blocks in *
* most dense grid. Gather the infomration of active arms of multi-resolution *
* tiles	into one 3D topology grid cell where this multi-resolution tile	is   *
* stored (the starting cell of tile blocks).				     *
*									     *
* PARAMETERS:								     *
*   MSTopology:	  A 3D grid hold the topology of this grid.		     *
*   MSTopoSizes:  The sizes in XYZ of MSTopology.			     *
*									     *
* RETURN VALUE:								     *
*   void								     *
*****************************************************************************/
static void UserMicroMultiResMoveActiveArmsInfo(
				 const int *MSTopoSizes,	
				 UserMicroSupportTopoInfoStruct ***MSTopology)
{
    int i, j, k, l, CX[2], CY[2], CZ[2];

    for (k = 0; k < MSTopoSizes[2]; k++) {
	for (j = 0; j < MSTopoSizes[1]; j++) {
	    for (i = 0; i < MSTopoSizes[0]; i++) {
		UserMicroSupportTopoInfoStruct 
		    *Topo = &(MSTopology[i][j][k]);

		if (Topo -> Stride <= 1)
		    continue;

		/* CX, CY, CZ store min/max indices of the multi-resolution */
		/* tiles of Stride.					    */
		IRIT_PT2D_SET(CX, i, i + Topo -> Stride - 1);
		IRIT_PT2D_SET(CY, j, j + Topo -> Stride - 1);
		IRIT_PT2D_SET(CZ, k, k + Topo -> Stride - 1);
	    
		for (l = 0; l < 8; l++) {
		    int U, V, W, UV;
		    W = l / 4;
		    UV = l % 4;
		    V = UV / 2;
		    U = UV % 2;

		    Topo -> ActiveArms[l]
			  = MSTopology[CX[U]][CY[V]][CZ[1 - W]].ActiveArms[l];
		}
	    }
	}
    }
}

/*****************************************************************************
* DESCRIPTION:								     *
*   Remove all of the invalid tiles from the given list	of tiles.	     *
*									     *
* PARAMETERS:								     *
*   MS:		  The list of tiles from which tiles are deleted.	     *
*									     *
* RETURN VALUE:								     *
*   IritPrsrObjectStruct *:	The list of the	remaining tiles	after purge. *
*****************************************************************************/
static IritPrsrObjectStruct *UserMicroRemoveInvalidTiles(
						     IritPrsrObjectStruct *MS)
{
    int i;

    /* Remove all invalid tiles from MS. */
    for (i = 0; IritPrsrListObjectLength(MS) > 0; ) {
	int Inv;
	IritPrsrObjectStruct
	    *Tile = IritPrsrListObjectGet(MS, i);

	if (Tile == NULL)
	    break;

	Inv = IritMiscAttrIDGetObjectIntAttrib(Tile,
					       IRIT_ATTR_CREATE_ID(Invalid));
	if (Inv == TRUE)		    /* Remove the tile of the list. */
	    IritPrsrListObjectDelete(MS, i, TRUE);
	else
	    i++;
    }

    return MS;
}

/*****************************************************************************
* DESCRIPTION:								     *
*   Detects a 2	x 2 x 2	block of tiles of the same size	and merge them into  *
* a single double-sized	tile. The resulting tile replaces a block of 8 old   *
* tiles	in the 3D topology grid. The new tiles are merged again	to create    *
* double-sized tiles. As a result, tiles of multi-resolution will be	     *
* generated, each of which has the size	of the power of	2.		     *
*									     *
* PARAMETERS:								     *
*   MS:		  The give support microstructure with the tile	stride = 1.  *
*   MSTopoSizes:  The sizes in XYZ of MSTopology.			     *
*   MSTopology:	  A 3D grid hold the topology of this grid.		     *
*   BBoxTV:	  The total dimension of the support.			     *
*   Params:	  The support parameters (for tiles of Stride =	1).	     *
*									     *
* RETURN VALUE:								     *
*   IritPrsrObjectStruct  *: A list of multi-resolution tiles. Each item in  *
*		       the list	is again a list	of tiles having	the same     *
*		       size (stride).					     *
*****************************************************************************/
static IritPrsrObjectStruct *UserMicroMultiResolutionTiling(
			   IritPrsrObjectStruct *MS,
			   const int *MSTopoSizes,
			   UserMicroSupportTopoInfoStruct ***MSTopology,
			   const TrivTVStruct *BBoxTV,
			   const IritUserMicroGenAMSupportParamStruct *Params)
{
    int i, j, k, l, St, NumMerged, MinTopoSize, 
	Stride = 2,
	UsePrmThickness = Params -> PrmThicknessTV != NULL;
    IritPrsrObjectStruct *NewTiles,
	*MultiResTiles = NULL;

    MinTopoSize = IRIT_MIN3(MSTopoSizes[0], MSTopoSizes[1], MSTopoSizes[2]);
    if (MinTopoSize < 2)
	return NULL;

    /* Prefilter tiles before merging. Tiles with no arms are prefiltered. */
    for (i = 0; i < MSTopoSizes[0]; i++) {
	for (j = 0; j < MSTopoSizes[1]; j++) {
	    for (k = 0; k < MSTopoSizes[2]; k++) {
		int NumPurge;
		UserMicroSupportTopoInfoStruct
		    *TlTopo = &MSTopology[i][j][k];
		
		if (TlTopo -> Tile == NULL) {
		    /* Mark arms of empty tiles as inactive. */
		    for (l = 0; l < 8; l++)
			TlTopo -> ActiveArms[l] = TRUE;
		    TlTopo -> Stride = 0;
		    continue;
		}

		NumPurge = 0;
		for (l = 0; l < 8; l++) {
		    if (TlTopo -> ActiveArms[l])
			NumPurge++;
		}

		if (NumPurge == 8) {
		    IritMiscAttrIDSetObjectIntAttrib(TlTopo -> Tile,
						  IRIT_ATTR_CREATE_ID(Invalid),
						  TRUE);
		    TlTopo -> Tile = NULL;
		    TlTopo -> Stride = 0;
		}
	    }
	}
    }

    MultiResTiles = IritPrsrGenLISTObject(NULL);

    /* Try merging one resolution by one resolution. */
    while (Stride < MinTopoSize) {
	NumMerged = UserMicroCountMergingTiles(MSTopoSizes, MSTopology, 
					       Stride);
#	ifdef DEBUG_TEST_MICRO_SUPPORT
	    fprintf(stderr, "Stride = %d, Num of mergable tiles = %d\n",
		    Stride, NumMerged);
#	endif /* DEBUG_TEST_MICRO_SUPPORT */
	if (NumMerged == 0)
	    break;
	Stride *= 2;
    }
    
    if (!UsePrmThickness) {
	for (St = 2; St < Stride; St *= 2) {
	    NewTiles = UserMicroGenMergedTiles(MSTopoSizes, MSTopology,
					       St, BBoxTV, Params);
	    IritPrsrListObjectAppend(MultiResTiles, NewTiles);
	}

    }
    else {
	for (St = 2; St < Stride; St *= 2) {
	    NewTiles = UserMicroGenMergedPrmTiles(MSTopoSizes, MSTopology,
						  St, BBoxTV, Params);
	    IritPrsrListObjectAppend(MultiResTiles, NewTiles);
	}
    }

    for (i = 0; IritPrsrListObjectLength(MultiResTiles) > 0; ) {
	NewTiles = IritPrsrListObjectGet(MultiResTiles, i);

	if (NewTiles == NULL)
	    break;

	NewTiles = UserMicroRemoveInvalidTiles(NewTiles);
	
	if (IritPrsrListObjectLength(NewTiles) == 0)
	    IritPrsrListObjectDelete(MultiResTiles, i, TRUE);
	else
	    i++;
    }

    if (IritPrsrListObjectLength(MultiResTiles) == 0) {
	IritPrsrFreeObject(MultiResTiles);
	MultiResTiles = NULL;
    }

    return MultiResTiles;
}

/*****************************************************************************
* DESCRIPTION:								     *
*   Thicken the	lower arms of the tiles	attached to the	base, or a tile	with *
* z-index = 0. Replace the old tiles by	the new	tiles with thicker arms.     *
*									     *
* PARAMETERS:								     *
*   MS:		  The give support microstructure so far.		     *
*   MSTopoSizes:  The sizes in XYZ of MSTopology.			     *
*   MSTopology:	  A 3D grid hold the topology of this grid.		     *
*   BBoxTV:	  The total dimension of the support.			     *
*   Params:	  The support parameters (for tiles of Stride =	1).	     *
*									     *
* RETURN VALUE:								     *
*   IritPrsrObjectStruct:  The set of tiles that contributes to the support  *
*		     microstructure.					     *
*****************************************************************************/
static IritPrsrObjectStruct *UserMicroThickenBaseTileArms(
			   IritPrsrObjectStruct *MS,
			   const int *MSTopoSizes,
			   UserMicroSupportTopoInfoStruct ***MSTopology,
			   const TrivTVStruct *BBoxTV,
			   const IritUserMicroGenAMSupportParamStruct *Params)
{
    int i, j;
    CagdRType TileCrnrSizes[8], TileInt[3],
	CrnrVertScl[2] = { 0.5, 0.5 };
    CagdBBoxStruct BBox;
    IrtHmgnMatType MatScl, Mat;
    IritPrsrObjectStruct *Tile, *TileNew;

    if (Params -> TileBaseEdgeScale < 1.0 + IRIT_EPS)
	return MS;

    for (i = 0; i < 8; i++)
	TileCrnrSizes[i] = Params -> TileEndEdgeThickness;

    /* Thicken the lower arms. */
    for (i = 0; i < 4; i++)
	TileCrnrSizes[i] *= Params -> TileBaseEdgeScale;

    /* Generate a tile in [0,1]^3 with thicker lower arms. */
    Tile = IritUserMicroDiagTile1(Params -> TileCntrThickness, 1.0,
				  TileCrnrSizes, CrnrVertScl, NULL,
				  TRUE, 0.0, NULL, NULL);
    if (Tile == NULL) { /* Failed to thicken. */
	IRIT_WARNING_MSG("Base tiles will not be thickened due to invalid parameter.\n");
	return MS;
    }

    /* For tile transformations. */
    IritTrivTVBBox(BBoxTV, &BBox);
    for (i = 0; i < 3; i++)
	TileInt[i] = (BBox.Max[i] - BBox.Min[i]) / (CagdRType)MSTopoSizes[i];
    IritMiscMatGenMatScale(TileInt[0], TileInt[1], TileInt[2], MatScl);

    /* Replace the old tiles with new tiles having thicker arms. */
    for (i = 0; i < MSTopoSizes[0]; i++) {
	for (j = 0; j < MSTopoSizes[1]; j++) {
	    char IdxsStr[IRIT_LINE_LEN];

	    UserMicroSupportTopoInfoStruct
		*TlTopo = &MSTopology[i][j][0];
	    
	    if (TlTopo -> Tile == NULL)
		continue;

	    /* Make the old tile invalid. */
	    IritMiscAttrIDSetObjectIntAttrib(TlTopo -> Tile,
					     IRIT_ATTR_CREATE_ID(Invalid),
					     TRUE);

	    IritMiscMatGenMatTrans(BBox.Min[0] + TileInt[0] * (CagdRType)i,
				   BBox.Min[1] + TileInt[1] * (CagdRType)j,
				   BBox.Min[2], Mat);
	    
	    /* Scaling followed by translation.*/
	    IritMiscMatMultTwo4by4(Mat, MatScl, Mat);
	    TileNew = IritGeomTransformObject(Tile, Mat);

	    sprintf(&IdxsStr[0], "%d,%d,0::%d,%d,%d", i, j, 
		    MSTopoSizes[0], MSTopoSizes[1], MSTopoSizes[2]);
	    IritMiscAttrIDSetObjectStrAttrib(TileNew, 
					     IRIT_ATTR_CREATE_ID(MSIndex), 
					     IdxsStr);
	    /* Set the support tile information */
	    IritUserMicroMarkSupportMSInfo(TileNew,
					   USER_MICRO_SUPPORT_BASE, 
					   0);
	    TlTopo -> Tile = TileNew;
	    IritPrsrListObjectAppend(MS, TileNew);
	}
    }
    IritPrsrFreeObject(Tile);

    /* Purge all invalid tiles from MS. */
    MS = UserMicroRemoveInvalidTiles(MS);

    return MS;
}

/*****************************************************************************
* DESCRIPTION:								     *
*   Thicken the	lower arms of the tiles	attached to the	base, or a tile	with *
* z-index = 0. Tile center and corner thickness are parametrized in a scalar *
* trivariate. Replace the old tiles by	the new	tiles with thicker arms.     *
*									     *
* PARAMETERS:								     *
*   MS:		  The give support microstructure so far.		     *
*   MSTopoSizes:  The sizes in XYZ of MSTopology.			     *
*   MSTopology:	  A 3D grid hold the topology of this grid.		     *
*   BBoxTV:	  The total dimension of the support.			     *
*   Params:	  The support parameters (for tiles of Stride =	1).	     *
*									     *
* RETURN VALUE:								     *
*   IritPrsrObjectStruct:  The set of tiles that contributes to the support  *
*		     microstructure.					     *
*****************************************************************************/
static IritPrsrObjectStruct *UserMicroThickenBaseTilePrmArms(
			   IritPrsrObjectStruct *MS,
			   const int *MSTopoSizes,
			   UserMicroSupportTopoInfoStruct ***MSTopology,
			   const TrivTVStruct *BBoxTV,
			   const IritUserMicroGenAMSupportParamStruct *Params)
{
    int i, NumBaseTiles;
    CagdRType TileInt[3],
	CrnrVertScl[2] = { 0.5, 0.5 };
    CagdBBoxStruct BBox;
    IrtHmgnMatType MatScl;
    IritPrsrObjectStruct **TileArray;

    if (Params -> TileBaseEdgeScale < 1.0 + IRIT_EPS)
	return MS;

    /* For tile transformations. */
    IritTrivTVBBox(BBoxTV, &BBox);
    for (i = 0; i < 3; i++)
	TileInt[i] = (BBox.Max[i] - BBox.Min[i]) / (CagdRType) MSTopoSizes[i];
    IritMiscMatGenMatScale(TileInt[0], TileInt[1], TileInt[2], MatScl);

    NumBaseTiles = MSTopoSizes[0] * MSTopoSizes[1];
    TileArray = (IritPrsrObjectStruct **)
			  IritMalloc(sizeof(IritPrsrObjectStruct *) * NumBaseTiles);
    IRIT_ZAP_MEM(TileArray, sizeof(IritPrsrObjectStruct *) * NumBaseTiles);

#   pragma omp parallel for private(i) if (_IritParallelExec)
    for (i = 0; i < NumBaseTiles; i++) {
	int j, XId, YId;
	CagdRType TileCrnrSizes[8];
	IrtHmgnMatType Mat;
	IritPrsrObjectStruct *Tile, *TileNew;
	UserMicroSupportTopoInfoStruct *TlTopo;
	char IdxsStr[IRIT_LINE_LEN];

	XId = i % MSTopoSizes[0];
	YId = i / MSTopoSizes[0];
	TlTopo = &MSTopology[XId][YId][0];

	if (TlTopo -> Tile == NULL)
	    continue;
	
	/* Create tile geometry with parametric thickness. */
	IRIT_GEN_COPY(TileCrnrSizes, TlTopo -> PrmCrnrThickness,
		      sizeof(CagdRType) * 8);

	/* Thicken the lower arms. */
	for (j = 0; j < 4; j++) {
	    TileCrnrSizes[j] *= Params -> TileBaseEdgeScale;
	    if (TileCrnrSizes[j] > 0.5 - IRIT_EPS)
		TileCrnrSizes[j] = 0.5 - IRIT_EPS;
	}
	Tile = IritUserMicroDiagTile1(TlTopo -> PrmCntrThickness, 1.0,
				      TileCrnrSizes, CrnrVertScl, NULL,
				      TRUE, 0.0, NULL, NULL);
	if (Tile == NULL)
	    continue;

	/* Compute tile transformation. */
	IritMiscMatGenMatTrans(BBox.Min[0] + TileInt[0] * (CagdRType)XId,
			       BBox.Min[1] + TileInt[1] * (CagdRType)YId,
			       BBox.Min[2], Mat);
	    
	/* Scaling followed by translation.*/
	IritMiscMatMultTwo4by4(Mat, MatScl, Mat);
	TileNew = IritGeomTransformObject(Tile, Mat);

	sprintf(&IdxsStr[0], "%d,%d,0::%d,%d,%d", XId, YId, 
		MSTopoSizes[0], MSTopoSizes[1], MSTopoSizes[2]);
	IritMiscAttrIDSetObjectStrAttrib(TileNew, 
					 IRIT_ATTR_CREATE_ID(MSIndex), 
					 IdxsStr);
	/* Set the support tile information */
	IritUserMicroMarkSupportMSInfo(TileNew,
				       USER_MICRO_SUPPORT_BASE, 
				       0);
	TileArray[i] = TileNew;
	IritPrsrFreeObject(Tile);	
    }

    /* Replace the old tiles with new tiles having thicker arms. */
    for (i = 0; i < NumBaseTiles; i++) {
	int XId, YId;
	UserMicroSupportTopoInfoStruct *TlTopo;

	XId = i % MSTopoSizes[0];
	YId = i / MSTopoSizes[0];
	TlTopo = &MSTopology[XId][YId][0];
    
        if (TileArray[i] == NULL)
	    continue;

	/* Make the old tile invalid. */
	IritMiscAttrIDSetObjectIntAttrib(TlTopo -> Tile,
					 IRIT_ATTR_CREATE_ID(Invalid),
					 TRUE);
	TlTopo -> Tile = TileArray[i];
	IritPrsrListObjectAppend(MS, TileArray[i]);
    }
    IritFree(TileArray);

    /* Purge all invalid tiles from MS. */
    MS = UserMicroRemoveInvalidTiles(MS);

    return MS;
}

/*****************************************************************************
* DESCRIPTION:                                                               *
*   Add support tile attributes to micro structure MS. For each trivariate   *
* in MS, the type of support tile and the level of multiresolution are saved *
* to "MSSupportType" and "MSSupportLvl" attributes, respectively. If MS	     *
* contains several tiles, then the same tile type and level are stored in    *
* every trivariate in MS.						     *
*                                                                            *
* PARAMETERS:                                                                *
*   MS:       A micro structure of trivariates to mark the (same) support    *
*	      tile type and the multiresolution level.			     *
*   TileType: USER_MICRO_SUPPORT_BASE (0) for base tiles,		     *
*	      USER_MICRO_SUPPORT_NONBASE (1) for the interior non-base tiles,*
*	      USER_MICRO_SUPPORT_TIP (2) for tip objects.		     *
*	      -1 not to create the "MSSupportType" attribute.		     *
*   TileLvl:  The level of multi-resolution tiling. TileLvl is set to the    *
*	      nonnegative integer n for tiles of size(stride) 2^n.	     *
*	      -1 not to create the "MSSupportLvl" attribute.		     *
*                                                                            *
* RETURN VALUE:                                                              *
*   int:     TRUE if successful, FALSE if error.                             *
*                                                                            *
* SEE ALSO:                                                                  *
*   IritUserMicroMarkSupportMSInfo                                           *
*                                                                            *
* KEYWORDS:                                                                  *
*   IritUserMicroMarkSupportMSInfo                                           *
*****************************************************************************/
static int IritUserMicroMarkSupportMSInfo(IritPrsrObjectStruct *MS,
				          int TileType,
				          int TileLvl)
{
    int i,
	RetVal = TRUE;
    TrivTVStruct *TV;
    IritPrsrObjectStruct *PObj;
    
    do {
	switch (MS -> ObjType) {
    	    case IRIT_PRSR_OBJ_LIST_OBJ:
		for (i = 0;
		     (PObj = IritPrsrListObjectGet(MS, i)) != NULL;
		     i++) {
		    RetVal = (RetVal &&
			      IritUserMicroMarkSupportMSInfo(PObj, 
							     TileType, 
							     TileLvl));
		}
		break;
	    case IRIT_PRSR_OBJ_TRIVAR:
		for (TV = MS -> U.Trivars; TV != NULL; TV = TV -> Pnext) {
		    if (TileType >= 0)
			IritMiscAttrIDSetIntAttrib(
					   &TV -> Attr,
					   IRIT_ATTR_CREATE_ID(MSSupportType),
					   TileType);
		    if (TileLvl >= 0)
			IritMiscAttrIDSetIntAttrib(
					    &TV -> Attr,
					    IRIT_ATTR_CREATE_ID(MSSupportLvl),
					    TileLvl);
		    RetVal = TRUE;
		}
		break;
	    default:
		break;
	}

	MS = MS -> Pnext;
    } 
    while (MS != NULL);

    return RetVal;
}

/*****************************************************************************
* DESCRIPTION:                                                               *
*   For the given tile (topology), generate new spike(s) with the closest    *
*   point on the model.							     *
*									     *
* PARAMETERS:                                                                *
*   TlTopo:  The topology of the tile.					     *
*   PolyBVH: The BVH of the mesh.					     *
*   Params:  The support parameters.					     *
*									     *
* RETURN VALUE:                                                              *
*   IritPrsrObjectStruct *: List of the new spikes.			     *
*									     *
* KEYWORDS:                                                                  *
*   UserMicroGenClosestRaySpikes                                             *
*****************************************************************************/
static IritPrsrObjectStruct *UserMicroGenClosestRaySpikes(
			    UserMicroSupportTopoInfoStruct *TlTopo,
			    const struct IritGeomPolyBVHStruct *PolyBVH,
			    const IritUserMicroGenAMSupportParamStruct *Params)
{
    int l, x, y, z, SizeX, SizeY, SizeZ;
    char *TipIndex;
    const char
	*MSIndex = IritMiscAttrIDGetObjectStrAttrib(
	    					TlTopo -> Tile,
	    					IRIT_ATTR_CREATE_ID(MSIndex));
    TrivTVStruct *TV;
    IritPrsrObjectStruct *TVObj,
	*NewSpikes = IritPrsrGenLISTObject(NULL);

    sscanf(MSIndex, "%d,%d,%d::%d,%d,%d",
	   &x, &y, &z, &SizeX, &SizeY, &SizeZ);
    TipIndex = (char *) IritMalloc((int)(strlen(MSIndex) + 4));

    if (TlTopo -> TopTile) {
	for (l = 0; l < 4; l++) {
	    int n, OldCircLin;
	    IrtPtType InterPt;
	    IrtVecType InterNrml;

	    if (!IRIT_PT_APX_EQ(VecNone, TlTopo -> Tile4RayUpPos[l]))
		continue;

	    if ((n = IritGeomPolyBVHGetClosestPointNrml(
				    	   TlTopo -> Tile4TopPos[l],
				    	   PolyBVH, InterPt, InterNrml)) > 0) {
		IrtRType Slope;

		Slope = asin(IRIT_ABS(InterNrml[2]));
		if (IRIT_RAD2DEG(Slope) <= Params -> ModelMinSlope)
		    continue;

		OldCircLin = IritPrsrSetPolyListCirc(TRUE);
		n = UserMicroTestRay(Params, TlTopo, PolyBVH,
				     TlTopo -> Tile4TopPos[l],
				     InterPt, InterNrml, TRUE, l,
				     TlTopo -> Tile4RayUpPos[l],
				     TlTopo -> Tile4RayUpNrml[l], n);
		IritPrsrSetPolyListCirc(OldCircLin);

		if (n == 0)
		    continue;

		TV = UserMicroCnvrtRayToTV2(TlTopo, Params,
					    TlTopo -> Tile4RayUpPos[l],
					    TlTopo -> Tile4RayUpNrml[l],
					    TRUE, l);
		TVObj = IritPrsrGenTRIVARObject(TV);

		sprintf(TipIndex, "%d,%d,%d::%d,%d,%d::%d",
			x, y, z, SizeX, SizeY, SizeZ, l);
		IritMiscAttrIDSetObjectStrAttrib(TVObj,
					         IRIT_ATTR_CREATE_ID(MSIndex),
					         TipIndex);
		IritPrsrListObjectAppend(NewSpikes, TVObj);
	    }
	}
    }
    if (TlTopo -> BotTile) {
	for (l = 0; l < 4; l++) {
	    int n, OldCircLin;
	    IrtPtType InterPt;
	    IrtVecType InterNrml;

	    if (!IRIT_PT_APX_EQ(VecNone, TlTopo -> Tile4RayDwnPos[l]))
		continue;

	    if ((n = IritGeomPolyBVHGetClosestPointNrml(
				    	  TlTopo -> Tile4BotPos[l],
				    	  PolyBVH, InterPt, InterNrml)) > 0) {
		IrtRType Slope;

		Slope = asin(IRIT_ABS(InterNrml[2]));
		if (IRIT_RAD2DEG(Slope) <= Params -> ModelMinSlope)
		    continue;

		OldCircLin = IritPrsrSetPolyListCirc(TRUE);
		n = UserMicroTestRay(Params, TlTopo, PolyBVH,
				     TlTopo -> Tile4BotPos[l],
				     InterPt, InterNrml, FALSE, l,
				     TlTopo -> Tile4RayDwnPos[l],
				     TlTopo -> Tile4RayDwnNrml[l], n);
		IritPrsrSetPolyListCirc(OldCircLin);

		if (n == 0)
		    continue;

		TV = UserMicroCnvrtRayToTV2(TlTopo, Params,
					    TlTopo -> Tile4RayDwnPos[l],
					    TlTopo -> Tile4RayDwnNrml[l],
					    FALSE, l);
		TVObj = IritPrsrGenTRIVARObject(TV);

		sprintf(TipIndex, "%d,%d,%d::%d,%d,%d::%d",
			x, y, z, SizeX, SizeY, SizeZ, l + 4);
		IritMiscAttrIDSetObjectStrAttrib(TVObj,
					 IRIT_ATTR_CREATE_ID(MSIndex),
					 TipIndex);
		IritPrsrListObjectAppend(NewSpikes, TVObj);
	    }
	}
    }

    IritFree(TipIndex);

    return NewSpikes;
}

/*****************************************************************************
* DESCRIPTION:                                                               *
*   Filter the NewSpikes that are intersect with the existed tiles or spikes.*
*									     *
* PARAMETERS:								     *
*   MS:		The given support microstructure so far.		     *
*   MSTopology: A 3D grid hold the topology of this grid.		     *
*   ExtSpikes:  The existed spikes that are already generated.		     *
*   NewSpikes:  The spikes that need to be checked for intersection with     *
*		existed tiles or spikes.				     *
*									     *
* RETURN VALUE:                                                              *
*   IritPrsrObjectStruct *: The spikes that are not intersect with the	     *
*		    existed tiles or spikes.				     *
*									     *
* KEYWORDS:                                                                  *
*   UserMicroFilterSpikesWithExistedTilesAndSpikes			     *
*****************************************************************************/
static IritPrsrObjectStruct *UserMicroFilterSpikesWithExistedTilesAndSpikes(
				IritPrsrObjectStruct *MS,
				UserMicroSupportTopoInfoStruct ***MSTopology,
				IritPrsrObjectStruct *ExtSpikes,
				IritPrsrObjectStruct *NewSpikes)
{
    int i, NumTiles, *ItstArr,
	ExtSpNum = IritPrsrListObjectLength(ExtSpikes),
	NewSpNum = IritPrsrListObjectLength(NewSpikes);
    CagdBBoxStruct *TileBBoxArray;
    CagdSrfStruct **TileTVSurfs;
    TrivTVStruct *TV, **ExtSpikeCutTVs, **NewSpikeCutTVs;
    IritPrsrPolygonStruct **ExtSpikeCutPolys, **NewSpikeCutPolys;
    IritPrsrObjectStruct **TileSrfObjs, *MSTile,
	*Tiles = IritPrsrGenLISTObject(NULL),
	*TestedNewSpikes = IritPrsrGenLISTObject(NULL);
    struct IritGeomPolyBVHStruct **NewSpikePolyBVHs;
    CagdSrf2PlsInfoStrct TessInfo;

    IritPrsrTSrf2PlysInitTessInfo2(&TessInfo, TRUE, 20, NULL, 
				   FALSE, FALSE, 0, NULL);

    /* 0. Memory allocations. */
    ExtSpikeCutTVs = (TrivTVStruct **)
			    IritMalloc(sizeof(TrivTVStruct *) * ExtSpNum);
    ExtSpikeCutPolys = (IritPrsrPolygonStruct **)
		      IritMalloc(sizeof(IritPrsrPolygonStruct *) * ExtSpNum);
    NewSpikeCutTVs = (TrivTVStruct **)
			       IritMalloc(sizeof(TrivTVStruct *) * NewSpNum);
    NewSpikeCutPolys = (IritPrsrPolygonStruct **)
		      IritMalloc(sizeof(IritPrsrPolygonStruct *) * NewSpNum);
    NewSpikePolyBVHs = (struct IritGeomPolyBVHStruct **)
		IritMalloc(sizeof(struct IritGeomPolyBVHStruct *) * NewSpNum);

    ItstArr = IritMalloc(sizeof(int) * NewSpNum);
    IRIT_ZAP_MEM(ItstArr, sizeof(int) * NewSpNum);

#   pragma omp parallel for private(i) if (_IritParallelExec)
    for (i = 0; i < ExtSpNum; i++) {
	IrtRType UMin, UMax, VMin, VMax, WMin, WMax;
	IritPrsrObjectStruct
	    *SpikeObj = IritPrsrListObjectGet(ExtSpikes, i);
	
	IritTrivTVDomain(SpikeObj -> U.Trivars, &UMin, &UMax, &VMin, &VMax, 
								 &WMin, &WMax);
	ExtSpikeCutTVs[i] = IritTrivTVRegionFromTV(SpikeObj -> U.Trivars,
						   IRIT_BLEND(WMin, WMax, 0.1), 
						   IRIT_BLEND(WMin, WMax, 0.9), 
						   TRIV_CONST_W_DIR);
	ExtSpikeCutPolys[i] = IritPrsrTrivar2Polygons(ExtSpikeCutTVs[i], 
						      &TessInfo);
    }

#   pragma omp parallel for private(i) if (_IritParallelExec)
    for (i = 0; i < NewSpNum; i++) {
	IrtRType UMin, UMax, VMin, VMax, WMin, WMax;
	IritPrsrObjectStruct
	    *SpikeObj = IritPrsrListObjectGet(NewSpikes, i);


	IritTrivTVDomain(SpikeObj -> U.Trivars, &UMin, &UMax, &VMin, &VMax, 
								 &WMin, &WMax);
	NewSpikeCutTVs[i] = IritTrivTVRegionFromTV(SpikeObj -> U.Trivars,
						   IRIT_BLEND(WMin, WMax, 0.1), 
						   IRIT_BLEND(WMin, WMax, 0.9), 
						   TRIV_CONST_W_DIR);
	NewSpikeCutPolys[i] = IritPrsrTrivar2Polygons(NewSpikeCutTVs[i],
						      &TessInfo);
	NewSpikePolyBVHs[i] = IritGeomPolyBVHCreate(NewSpikeCutPolys[i]);
    }

    /* 1. Filter spikes that intersect with tiles. */
    /* Convert tile objects. */
    for (i = 0; (MSTile = IritPrsrListObjectGet(MS, i)) != NULL; i++) {
	int j, x, y, z, SizeX, SizeY, SizeZ;
	IritPrsrObjectStruct *MSTilePart;
	const char
	    *MSIndex = IritMiscAttrIDGetObjectStrAttrib(MSTile,
						 IRIT_ATTR_CREATE_ID(MSIndex));

	sscanf(MSIndex, "%d,%d,%d::%d,%d,%d",
	       &x, &y, &z, &SizeX, &SizeY, &SizeZ);

	for (j = 0;
	     (MSTilePart = IritPrsrListObjectGet(MSTile, j)) != NULL;
	     j++) {
	    for (TV = MSTilePart -> U.Trivars; TV != NULL; TV = TV -> Pnext) {
		IritPrsrObjectStruct
		    *PartObj = IritPrsrGenTRIVARObject(TV);

		IritMiscAttrIDSetObjectStrAttrib(PartObj,
						 IRIT_ATTR_CREATE_ID(MSIndex),
						 MSIndex);
		IritPrsrListObjectAppend(Tiles, PartObj);
	    }
	}
    }

    /* Generate Tile BBoxes. */
    NumTiles = IritPrsrListObjectLength(Tiles);
    if (NumTiles > 0) {
	TileBBoxArray = IritCagdBBoxArrayNew(NumTiles);
	for (i = 0; (MSTile = IritPrsrListObjectGet(Tiles, i)) != NULL; i++) {
	    IritGeomBBComputeBboxObject(MSTile, &TileBBoxArray[i], FALSE);
	}
    }
    else
	TileBBoxArray =  NULL;

    TileTVSurfs = (CagdSrfStruct **)
			    IritMalloc(sizeof(CagdSrfStruct *) * NumTiles);
    TileSrfObjs = (IritPrsrObjectStruct **)
			IritMalloc(sizeof(IritPrsrObjectStruct *) * NumTiles);

#   pragma omp parallel for private(i) if (_IritParallelExec)
    for (i = 0; i < NumTiles; i++) {
	TileTVSurfs[i] = IritTrivBndrySrfsFromTVs(
	    IritPrsrListObjectGet(Tiles, i) -> U.Trivars,
	    IRIT_EPS, TRUE, FALSE, FALSE);
	TileSrfObjs[i] = IritPrsrGenSRFObject(TileTVSurfs[i]);
    }

#   pragma omp parallel for private(i) if (_IritParallelExec)
    for (i = 0; i < NewSpNum; i++) {
	int SpX, SpY, SpZ, SizeX, SizeY, SizeZ, SpArmIdx, TileIdx, IsUpRay;
	const char *MSIndex;
	UserMicroSupportTopoInfoStruct *TlTopo;
	GMBBBboxStruct SpikeBBox;
	IritPrsrObjectStruct
	    *SpikeObj = IritPrsrListObjectGet(NewSpikes, i);

	MSIndex = IritMiscAttrIDGetObjectStrAttrib(SpikeObj,
						IRIT_ATTR_CREATE_ID(MSIndex));
	sscanf(MSIndex, "%d,%d,%d::%d,%d,%d::%d",
	       &SpX, &SpY, &SpZ, &SizeX, &SizeY, &SizeZ, &SpArmIdx);

	IsUpRay = SpArmIdx < 4 ? TRUE : FALSE;

	IritGeomBBComputeBboxObject(SpikeObj, &SpikeBBox, FALSE);

	for (TileIdx = 0; TileIdx < NumTiles; TileIdx++) {
	    /* Check bbox intersection test first. */
	    int TiX, TiY, TiZ, SizeX, SizeY, SizeZ;
	    GMBBBboxStruct
		*TileBox = &TileBBoxArray[TileIdx];
	    const char
		*MSIndex = IritMiscAttrIDGetObjectStrAttrib(
					IritPrsrListObjectGet(Tiles, TileIdx),
					IRIT_ATTR_CREATE_ID(MSIndex));
	    sscanf(MSIndex, "%d,%d,%d::%d,%d,%d",
		   &TiX, &TiY, &TiZ, &SizeX, &SizeY, &SizeZ);
	    if (SpX == TiX && SpY == TiY && SpZ == TiZ)
		continue;

	    /* Check only if BBox are intersecting. */
	    if (!(SpikeBBox.Max[0] < TileBox -> Min[0] ||
		  SpikeBBox.Min[0] > TileBox -> Max[0] ||
		  SpikeBBox.Max[1] < TileBox -> Min[1] ||
		  SpikeBBox.Min[1] > TileBox -> Max[1] ||
		  SpikeBBox.Max[2] < TileBox -> Min[2] ||
		  SpikeBBox.Min[2] > TileBox -> Max[2])) {
		/* Check if the spike has intersection with the tile. */
		if (IritGeomPolyBVHSrfsPolyInter(TileSrfObjs[TileIdx],
						 NewSpikePolyBVHs[i]) != 0) {
		    TlTopo = &MSTopology[SpX][SpY][SpZ];
		    if (IsUpRay){
			IRIT_PT_COPY(TlTopo -> Tile4RayUpPos[SpArmIdx], 
								    VecNone);
		    }
		    else {
		    	IRIT_PT_COPY(TlTopo -> Tile4RayDwnPos[SpArmIdx - 4], 
								    VecNone);
		    }

		    ItstArr[i]++;
		    break;
		}
	    }
	}
    }

    /* 2. Filter spikes that intersect with other spikes. */
#   pragma omp parallel for private(i) if (_IritParallelExec)
    for (i = 0; i < NewSpNum; i++) {
	int j, SpX, SpY, SpZ, SizeX, SizeY, SizeZ, SpArmIdx, IsUpRay;
	const char *MSIndex;
	UserMicroSupportTopoInfoStruct *TlTopo;
	IritPrsrObjectStruct
	    *SpikeObj = IritPrsrListObjectGet(NewSpikes, i);

    	if (ItstArr[i] != 0)
	    continue;

	MSIndex = IritMiscAttrIDGetObjectStrAttrib(SpikeObj,
						IRIT_ATTR_CREATE_ID(MSIndex));
	sscanf(MSIndex, "%d,%d,%d::%d,%d,%d::%d",
	       &SpX, &SpY, &SpZ, &SizeX, &SizeY, &SizeZ, &SpArmIdx);

	IsUpRay = SpArmIdx < 4 ? TRUE : FALSE;

	/* Check if the new spikes intersect with the existed spikes. */
	for (j = 0; j < ExtSpNum; j++) {
	    if (IritGeomPolyBVHPolysInter(NewSpikePolyBVHs[i],
					  ExtSpikeCutPolys[j])) {
		TlTopo = &MSTopology[SpX][SpY][SpZ];
		if (IsUpRay) {
		    IRIT_PT_COPY(TlTopo -> Tile4RayUpPos[SpArmIdx], VecNone);
		}
		else {
		    IRIT_PT_COPY(TlTopo -> Tile4RayDwnPos[SpArmIdx - 4],
								    VecNone);
		}

		ItstArr[i]++;
		break;
	    }		
	}

	/* Check if the new spikes intersect with each other. */
	for (j = i + 1; j < NewSpNum; j++) {
	    if (IritGeomPolyBVHPolysInter(NewSpikePolyBVHs[i],
					  NewSpikeCutPolys[j])) {
		TlTopo = &MSTopology[SpX][SpY][SpZ];
		if (IsUpRay) {
		    IRIT_PT_COPY(TlTopo -> Tile4RayUpPos[SpArmIdx], VecNone);
		}
		else {
		    IRIT_PT_COPY(TlTopo -> Tile4RayDwnPos[SpArmIdx - 4],
								    VecNone);
		}

		ItstArr[i]++;
		break;
	    }
	}
    }

    /* Remove intersecting spikes. */
    for (i = 0; i < NewSpNum; i++) {
	IritPrsrObjectStruct
	    *SpikeObj = IritPrsrListObjectDelete(NewSpikes, 0, FALSE);

	/* New spikes with the closest point: pink. */
	IritMiscAttrSetObjectRGBColor(SpikeObj, 255, 105, 180);

	if (ItstArr[i] == 0)
	    IritPrsrListObjectAppend(TestedNewSpikes, SpikeObj);
	else
	    IritPrsrFreeObject(SpikeObj);
    }

    /* Free memory. */
    for (i = 0; i < NumTiles; i++) {
	IritPrsrFreeObject(TileSrfObjs[i]);
	IritPrsrListObjectGet(Tiles, i) -> U.Trivars = NULL;
    }

    IritFree(TileSrfObjs);
    IritPrsrFreeObject(Tiles);
    IritFree(TileTVSurfs);
    IritCagdBBoxArrayFree(TileBBoxArray, NumTiles);

    for (i = 0; i < ExtSpNum; i++)
	IritTrivTVFree(ExtSpikeCutTVs[i]);
    IritFree(ExtSpikeCutTVs);

    for (i = 0; i < ExtSpNum; i++)
	IritPrsrFreePolygonList(ExtSpikeCutPolys[i]);
    IritFree(ExtSpikeCutPolys);

    for (i = 0; i < NewSpNum; i++)
	IritTrivTVFree(NewSpikeCutTVs[i]);
    IritFree(NewSpikeCutTVs);

    for (i = 0; i < NewSpNum; i++)
	IritPrsrFreePolygonList(NewSpikeCutPolys[i]);
    IritFree(NewSpikeCutPolys);

    for (i = 0; i < NewSpNum; i++)
	IritGeomPolyBVHFree(NewSpikePolyBVHs[i]);
    IritFree(NewSpikePolyBVHs);

    IritFree(ItstArr);

    return TestedNewSpikes;
}

/*****************************************************************************
* DESCRIPTION:                                                               *
*   For the generated spikes with vertical or diagonal rays, try to replace  *
*   them with the new spikes generated by the closest point on the model if  *
*   there is no intersection between the existing spikes and tiles. For an   *
*   arm of the tile that could not be generated by vertical or diagonal ray, *
*   try to generate a new spike with the closest point also.		     *
*									     *
* PARAMETERS:                                                                *
*   MS:		 The given support microstructure so far.		     *
*   MSTopoSizes: The sizes in XYZ of MSTopology.			     *
*   MSTopology:  A 3D grid hold the topology of this grid.		     *
*   PolyBVH:	 The BVH of the mesh.					     *
*   Params:	 The support parameters.				     *
*   Spikes:	 The spikes that need to be replaced.			     *
*									     *
*									     *
* RETURN VALUE:                                                              *
*   IritPrsrObjectStruct *: The spikes that some spikes have been newly	     *
*		    generated or replaced by the closest spikes.	     *
*									     *
* KEYWORDS:                                                                  *
*   UserMicroReplaceSpikesToClosestSpikes				     *
*****************************************************************************/
static IritPrsrObjectStruct *UserMicroReplaceSpikesToClosestSpikes(
			    IritPrsrObjectStruct *MS,
			    const int *MSTopoSizes,
			    UserMicroSupportTopoInfoStruct ***MSTopology,
			    const struct IritGeomPolyBVHStruct *PolyBVH,
			    const IritUserMicroGenAMSupportParamStruct *Params,
			    IritPrsrObjectStruct *Spikes)
{
    int i, j, k, NumTiles, NumSpikes, NumAllSpikes, Spike0Idx, *ItstArr,
	NumRepSpikes = 0;
    IritCagdBspMultComputationMethodType
	OldMultMethod = IritCagdBspMultComputationMethod(IRIT_QUERY_INT_PROP);
    CagdBBoxStruct *TileBBoxArray;
    CagdSrfStruct **TileTVSurfs;
    CagdSrf2PlsInfoStrct TessInfo;
    TrivTVStruct **SpikeCutTVs, *TV;
    IritPrsrPolygonStruct **SpikeCutPolys;
    IritPrsrObjectStruct **TileSrfObjs, *MSTile,
	*RepSpikes = IritPrsrGenLISTObject(NULL),
	*NewSpikes = IritPrsrGenLISTObject(NULL),
	*Tiles = IritPrsrGenLISTObject(NULL),
	*AllSpikes = IritPrsrGenLISTObject(NULL);
    UserMicroSupportTopoInfoStruct *TlTopo;

    IritPrsrTSrf2PlysInitTessInfo2(&TessInfo, TRUE, 20, NULL, 
				   FALSE, FALSE, 0, NULL);

    /* Convert tile objects. */
    for (i = 0; (MSTile = IritPrsrListObjectGet(MS, i)) != NULL; i++) {
	int x, y, z, SizeX, SizeY, SizeZ;
	IritPrsrObjectStruct *MSTilePart;
	const char
	    *MSIndex = IritMiscAttrIDGetObjectStrAttrib(MSTile,
						IRIT_ATTR_CREATE_ID(MSIndex));

	sscanf(MSIndex, "%d,%d,%d::%d,%d,%d",
	       &x, &y, &z, &SizeX, &SizeY, &SizeZ);
	for (j = 0; 
	     (MSTilePart = IritPrsrListObjectGet(MSTile, j)) != NULL; 
	     j++) {
	    for (TV = MSTilePart -> U.Trivars; TV != NULL; TV = TV -> Pnext) {
		IritPrsrObjectStruct
		    *PartObj = IritPrsrGenTRIVARObject(TV);

		IritMiscAttrIDSetObjectStrAttrib(PartObj,
						 IRIT_ATTR_CREATE_ID(MSIndex),
						 MSIndex);
		IritPrsrListObjectAppend(Tiles, PartObj);
	    }
	}
    }

    /* Generate Tile BBoxes. */
    k = IritPrsrListObjectLength(Tiles);
    if (k > 0) {
	TileBBoxArray = IritCagdBBoxArrayNew(k);
	for (i = 0; (MSTile = IritPrsrListObjectGet(Tiles, i)) != NULL; i++) {
	    IritGeomBBComputeBboxObject(MSTile, &TileBBoxArray[i], FALSE);
	}
    }
    else
	return NULL;

    NumSpikes = IritPrsrListObjectLength(Spikes);
    NumTiles = IritPrsrListObjectLength(Tiles);

    /* Memory allocation and object generation for parallelization. */
    ItstArr = IritMalloc(sizeof(int) * NumSpikes);
    IRIT_ZAP_MEM(ItstArr, sizeof(int) * NumSpikes);

    TileTVSurfs = (CagdSrfStruct **)
				IritMalloc(sizeof(CagdSrfStruct *) * NumTiles);

#   pragma omp parallel for private(i) if (_IritParallelExec)
    for (i = 0; i < NumTiles; i++) {
	TileTVSurfs[i] = IritTrivBndrySrfsFromTVs(
	    			IritPrsrListObjectGet(Tiles, i) -> U.Trivars,
	    			IRIT_EPS, TRUE, FALSE, FALSE);
    }

    SpikeCutTVs = (TrivTVStruct **)
			    IritMalloc(sizeof(TrivTVStruct *) * NumSpikes);
    SpikeCutPolys = (IritPrsrPolygonStruct **)
		     IritMalloc(sizeof(IritPrsrPolygonStruct *) * NumSpikes);

#   pragma omp parallel for private(i) if (_IritParallelExec)
    for (i = 0; i < NumSpikes; i++) {
	IrtRType UMin, UMax, VMin, VMax, WMin, WMax;
	IritPrsrObjectStruct
	    *SpikeObj = IritPrsrListObjectGet(Spikes, i);
	
	IritTrivTVDomain(SpikeObj -> U.Trivars, &UMin, &UMax, &VMin, &VMax, 
								 &WMin, &WMax);
	SpikeCutTVs[i] = IritTrivTVRegionFromTV(SpikeObj -> U.Trivars,
						IRIT_BLEND(WMin, WMax, 0.1), 
						IRIT_BLEND(WMin, WMax, 0.9), 
						TRIV_CONST_W_DIR);
	SpikeCutPolys[i] = IritPrsrTrivar2Polygons(SpikeCutTVs[i], &TessInfo);
    }

    TileSrfObjs = (IritPrsrObjectStruct **)
		        IritMalloc(sizeof(IritPrsrObjectStruct *) * NumTiles);

#   pragma omp parallel for private(i) if (_IritParallelExec)
    for (i = 0; i < NumTiles; i++) {
	TileSrfObjs[i] = IritPrsrGenSRFObject(TileTVSurfs[i]);
    }


    /* 1. For the generated vertical / diagonal spikes, try to generate    */
    /*    another spike with the closest point. If the new one is not      */
    /*    intersected with any other spikes or tiles, replace the old one. */ 
    for (Spike0Idx = 0; Spike0Idx < NumSpikes; Spike0Idx++) {
	IrtBType IsUpRay,
    	    IsItst = FALSE;
	int x, y, z, SizeX, SizeY, SizeZ, l, n, TileIdx, Spike1Idx,
	    OldCircLin = IritPrsrSetPolyListCirc(TRUE);
	IrtVecType RayPt, InterNrml;
	IrtPtType InterPt, OldRayPt;
	struct IritGeomPolyBVHStruct *SpikeCutPolyBVH;
	GMBBBboxStruct SpikeBBox;
	IritPrsrPolygonStruct *SpikeCutPoly;
	TrivTVStruct *SpikeTV, *SpikeCutTV;
	IritPrsrObjectStruct *TVOrigObj,
	    *SpikeTVObj = NULL,
    	    *SpikeObj = IritPrsrListObjectGet(Spikes, Spike0Idx);
	const char
	    *MSIndex = IritMiscAttrIDGetObjectStrAttrib(SpikeObj,
						IRIT_ATTR_CREATE_ID(MSIndex));

	sscanf(MSIndex, "%d,%d,%d::%d,%d,%d::%d",
	       &x, &y, &z, &SizeX, &SizeY, &SizeZ, &l);

	TlTopo = &MSTopology[x][y][z];

	if (l < 4)
	    IsUpRay = TRUE;
	else
    	    IsUpRay = FALSE;

	IsUpRay ? IRIT_VEC_COPY(RayPt, TlTopo -> Tile4TopPos[l]) :
	          IRIT_VEC_COPY(RayPt, TlTopo -> Tile4BotPos[l - 4]);

	/* 1a. Generate a spike with the closest point. */
	if ((n = IritGeomPolyBVHGetClosestPointNrml(RayPt, PolyBVH, InterPt,
						    InterNrml)) > 0) {
	    IrtRType
		Slope = asin(IRIT_ABS(InterNrml[2]));

	    if (IRIT_RAD2DEG(Slope) <= Params -> ModelMinSlope)
		continue;

	    if (IsUpRay) {
		IRIT_PT_COPY(OldRayPt, TlTopo -> Tile4RayUpPos[l]);
		n = UserMicroTestRay(Params, TlTopo, PolyBVH, RayPt, InterPt, 
				     InterNrml, TRUE, l, 
				     TlTopo -> Tile4RayUpPos[l], 
				     TlTopo -> Tile4RayUpNrml[l], n);

		if (n == 0)
		    continue;

		SpikeTV = UserMicroCnvrtRayToTV2(TlTopo, Params,
					         TlTopo -> Tile4RayUpPos[l],
					         TlTopo -> Tile4RayUpNrml[l],
					         TRUE, l);
	    }
	    else { /* Down ray. */
		IRIT_PT_COPY(OldRayPt, TlTopo -> Tile4RayDwnPos[l - 4]);
		n = UserMicroTestRay(Params, TlTopo, PolyBVH, RayPt, InterPt, 
				     InterNrml, FALSE, l - 4, 
				     TlTopo -> Tile4RayDwnPos[l - 4], 
				     TlTopo -> Tile4RayDwnNrml[l - 4], n);

		if (n == 0)
		    continue;

		SpikeTV = UserMicroCnvrtRayToTV2(TlTopo, Params,
					      TlTopo -> Tile4RayDwnPos[l - 4],
					      TlTopo -> Tile4RayDwnNrml[l - 4],
					      FALSE, l - 4);
	    }

	    IritPrsrSetPolyListCirc(OldCircLin);
	    SpikeTVObj = IritPrsrGenTRIVARObject(SpikeTV);
	    IritMiscAttrIDSetObjectStrAttrib(SpikeTVObj,
					     IRIT_ATTR_CREATE_ID(MSIndex),
					     MSIndex);
	}
	else
	    continue;

	if ((TVOrigObj = IritMiscAttrIDGetObjAttrib(
				       SpikeTVObj -> U.Trivars -> Attr,
				       IRIT_ATTR_CREATE_ID(OrigTV))) != NULL) {
	    IrtRType UMin, UMax, VMin, VMax, WMin, WMax;

	    IritTrivTVDomain(TVOrigObj -> U.Trivars, &UMin, &UMax, &VMin, 
							  &VMax, &WMin, &WMax);
	    SpikeCutTV = IritTrivTVRegionFromTV(TVOrigObj -> U.Trivars,
						IRIT_BLEND(WMin, WMax, 0.1), 
						IRIT_BLEND(WMin, WMax, 0.9), 
						TRIV_CONST_W_DIR);
	}
	else {
	    IrtRType UMin, UMax, VMin, VMax, WMin, WMax;

	    IritTrivTVDomain(SpikeTVObj -> U.Trivars, &UMin, &UMax, &VMin, 
							  &VMax, &WMin, &WMax);
	    SpikeCutTV = IritTrivTVRegionFromTV(SpikeTVObj -> U.Trivars,
						IRIT_BLEND(WMin, WMax, 0.1), 
						IRIT_BLEND(WMin, WMax, 0.9), 
						TRIV_CONST_W_DIR);
	}
	SpikeCutPoly = IritPrsrTrivar2Polygons(SpikeCutTV, &TessInfo);
	SpikeCutPolyBVH = IritGeomPolyBVHCreate(SpikeCutPoly);
	IritGeomBBComputeBboxObject(SpikeTVObj, &SpikeBBox, FALSE);
	
	/* 1b. Check if the spike intersects with tiles. */
	for (TileIdx = 0; TileIdx < NumTiles; TileIdx++) {
	    /* Check bbox intersection test first. */
	    int TiX, TiY, TiZ;
	    GMBBBboxStruct
		*TileBox = &TileBBoxArray[TileIdx];

	    MSIndex = IritMiscAttrIDGetObjectStrAttrib(
					IritPrsrListObjectGet(Tiles, TileIdx),
					IRIT_ATTR_CREATE_ID(MSIndex));
	    sscanf(MSIndex, "%d,%d,%d::%d,%d,%d",
		   &TiX, &TiY, &TiZ, &SizeX, &SizeY, &SizeZ);

	    /* If index of the tile and the spike are same, do not check. */
	    if (x == TiX && y == TiY && z == TiZ)
		continue;

	    /* Check only if BBox are intersecting. */
	    if (!(SpikeBBox.Max[0] < TileBox -> Min[0] ||
		  SpikeBBox.Min[0] > TileBox -> Max[0] ||
		  SpikeBBox.Max[1] < TileBox -> Min[1] ||
		  SpikeBBox.Min[1] > TileBox -> Max[1] ||
		  SpikeBBox.Max[2] < TileBox -> Min[2] ||
		  SpikeBBox.Min[2] > TileBox -> Max[2])) {
		/* Check if the spike has intersection with the tile. */
		if (IritGeomPolyBVHSrfsPolyInter(TileSrfObjs[TileIdx],
						 SpikeCutPolyBVH) != 0) {
		    if (IsUpRay)
			IRIT_PT_COPY(TlTopo -> Tile4RayUpPos[l], OldRayPt);
		    else
			IRIT_PT_COPY(TlTopo -> Tile4RayDwnPos[l - 4], 
								    OldRayPt);

		    IsItst = TRUE;
		    break;
		}
	    }
	}

	if (IsItst) {
	    IritTrivTVFree(SpikeCutTV);
	    IritPrsrFreePolygonList(SpikeCutPoly);
	    IritGeomPolyBVHFree(SpikeCutPolyBVH);
	    IritPrsrFreeObject(SpikeTVObj);
	    continue;
	}

	/* 1c. Check if the spike intersects with other spikes. */
	for (Spike1Idx = 0; Spike1Idx < NumSpikes; Spike1Idx++) {
	    /* Skip if the other spike are replaced or the spike is same. */
	    if (ItstArr[Spike1Idx] || Spike1Idx == Spike0Idx) 
		continue;

	    if (IritGeomPolyBVHPolysInter(SpikeCutPolyBVH,
					  SpikeCutPolys[Spike1Idx])) {
		if (IsUpRay)
		    IRIT_PT_COPY(TlTopo -> Tile4RayUpPos[l], OldRayPt);
		else
		    IRIT_PT_COPY(TlTopo -> Tile4RayDwnPos[l - 4], OldRayPt);

		IsItst = TRUE;
		break;
	    }
	}

	if (IsItst) {
	    IritTrivTVFree(SpikeCutTV);
	    IritPrsrFreePolygonList(SpikeCutPoly);
	    IritGeomPolyBVHFree(SpikeCutPolyBVH);
	    IritPrsrFreeObject(SpikeTVObj);
	    continue;
	}

	/* 1d. Check if the spike intersects with other closest spikes. */
	for (Spike1Idx = 0; Spike1Idx < NumRepSpikes; Spike1Idx++) {
	    IrtRType UMin, UMax, VMin, VMax, WMin, WMax;
	    IritPrsrObjectStruct
		*NewSpikeObj = IritPrsrListObjectGet(RepSpikes, Spike1Idx);
	    TrivTVStruct *NewSpikeTV;
	    IritPrsrPolygonStruct *NewSpikeCutPoly;
	    
	    IritTrivTVDomain(NewSpikeObj -> U.Trivars, &UMin, &UMax, &VMin, 
							  &VMax, &WMin, &WMax);
	    NewSpikeTV = IritTrivTVRegionFromTV(NewSpikeObj -> U.Trivars,
						IRIT_BLEND(WMin, WMax, 0.1), 
						IRIT_BLEND(WMin, WMax, 0.9), 
						TRIV_CONST_W_DIR);
	    NewSpikeCutPoly = IritPrsrTrivar2Polygons(NewSpikeTV, &TessInfo);
	    IritTrivTVFree(NewSpikeTV);

	    if (IritGeomPolyBVHPolysInter(SpikeCutPolyBVH, NewSpikeCutPoly)) {
		if (IsUpRay)
		    IRIT_PT_COPY(TlTopo -> Tile4RayUpPos[l], OldRayPt);
		else
		    IRIT_PT_COPY(TlTopo -> Tile4RayDwnPos[l - 4], OldRayPt);

		IsItst = TRUE;
		IritPrsrFreePolygonList(NewSpikeCutPoly);
		break;
	    }

	    IritPrsrFreePolygonList(NewSpikeCutPoly);
	}

	if (IsItst) {
	    IritTrivTVFree(SpikeCutTV);
	    IritPrsrFreePolygonList(SpikeCutPoly);
	    IritGeomPolyBVHFree(SpikeCutPolyBVH);
	    IritPrsrFreeObject(SpikeTVObj);
	    continue;
	}

	/* Replaced spikes with the closest point: orange. */
	IritMiscAttrSetObjectRGBColor(SpikeTVObj, 234, 132, 0);

	/* The new spike with the closest point is not intersected with */
	/* any other tiles or spikes. */
	IritPrsrListObjectAppend(RepSpikes, SpikeTVObj);

	/* The old spike is replaced to the new spike, so we mark it and    */
	/* do not use it further. */
	ItstArr[Spike0Idx]++;

	NumRepSpikes++;

	IritTrivTVFree(SpikeCutTV);
	IritPrsrFreePolygonList(SpikeCutPoly);
	IritGeomPolyBVHFree(SpikeCutPolyBVH);
    }

    /* Collect the vertical/diagonal spikes that are not replaced with the */
    /* closest spikes. */
    for (i = 0; i < NumSpikes; i++) {
	IritPrsrObjectStruct
    	    *SpikeObj = IritPrsrListObjectDelete(Spikes, 0, FALSE);

	/* Vertical or diagonal spikes: cyan. */
	IritMiscAttrSetObjectRGBColor(SpikeObj, 0, 200, 200);

	if (ItstArr[i] == 0) 
	    IritPrsrListObjectAppend(AllSpikes, SpikeObj);
	else 
	    IritPrsrFreeObject(SpikeObj);
    }

    /* Collect the new spikes (with the closest point). */
    IritPrsrListObjectAppendList(AllSpikes, RepSpikes);

    /* 2. For the tile's up/down arms without the spikes, try to generate */
    /*    spike with the closest point. */
    for (k = 0; k < MSTopoSizes[2]; k++) {
	for (j = 0; j < MSTopoSizes[1]; j++) {
	    for (i = 0; i < MSTopoSizes[0]; i++) {
		TlTopo = &MSTopology[i][j][k];
	    	if (TlTopo -> Tile == NULL)
		    continue;

		IritPrsrListObjectAppendList(NewSpikes,
					     UserMicroGenClosestRaySpikes(
								      TlTopo,
								      PolyBVH,
								      Params));
	    }
	}
    }

    /* 3. Filter the new spikes that intersect with the existed tiles or */
    /*    spikes. */
    IritPrsrListObjectAppendList(AllSpikes,
		    UserMicroFilterSpikesWithExistedTilesAndSpikes(MS,
								   MSTopology,
								   AllSpikes,
								   NewSpikes));
    NumAllSpikes = IritPrsrListObjectLength(AllSpikes);

    /* 4. Turn off the ActiveArms for the new spikes so that their arms are */
    /*    not to be deleted. */
    for (i = 0; i < NumAllSpikes; i++) {
	int SpX, SpY, SpZ, SpArmIdx, SizeX, SizeY, SizeZ;
	IritPrsrObjectStruct
	    *SpikeObj = IritPrsrListObjectGet(AllSpikes, i);
	const char
	    *MSIndex = IritMiscAttrIDGetObjectStrAttrib(SpikeObj,
						IRIT_ATTR_CREATE_ID(MSIndex));

	sscanf(MSIndex, "%d,%d,%d::%d,%d,%d::%d",
	       &SpX, &SpY, &SpZ, &SizeX, &SizeY, &SizeZ, &SpArmIdx);
	MSTopology[SpX][SpY][SpZ].ActiveArms[SpArmIdx] = FALSE;
    }

    IritPrsrFreeObject(Spikes);
    IritFree(ItstArr);
    IritCagdBBoxArrayFree(TileBBoxArray, NumTiles);

    for (i = 0; i < NumTiles; i++) {
	IritPrsrFreeObject(TileSrfObjs[i]);
	IritPrsrListObjectGet(Tiles, i) -> U.Trivars = NULL;
    }

    IritFree(TileTVSurfs);
    IritFree(TileSrfObjs);
    IritPrsrFreeObject(Tiles);

    for (i = 0; i < NumSpikes; i++) {
	    IritPrsrFreePolygonList(SpikeCutPolys[i]);
	    IritTrivTVFree(SpikeCutTVs[i]);
    }

    IritFree(SpikeCutPolys);
    IritFree(SpikeCutTVs);

    IritPrsrFreeObject(NewSpikes);

    IritCagdBspMultComputationMethod(OldMultMethod);
    return AllSpikes;
}

/*****************************************************************************
* DESCRIPTION:								     *
*   Preprocess applied before parametric support microstructure composition. *
* The center and corner thickness of a tile are computed on the fly using    *
* the given parametric thickness trivariate. Center and corner thickness is  *
* parametrized in a scalar trivariate, stored in support parameter struct.   *
*									     *
* PARAMETERS:								     *
*   Tile:	  A tile to be composed.				     *
*   CBData:	  Callback function data passed	from microstructure	     *
*		  composition. CBData must include a scalar trivariate 	     *
*		  that parametrizes thickness.				     *
*									     *
* RETURN VALUE:								     *
*   IritPrsrObjectStruct *:  A tile object with parametric center and corner *
*			 thickness.					     *
*****************************************************************************/
static IritPrsrObjectStruct *UserMicroPrmSupportTilePreProcess(
				      IritPrsrObjectStruct *Tile,
				      UserMicroPreProcessTileCBStruct *CBData)
{
    int i;
    CagdRType CntrThickness, TileCrnrSizes[8],
	CrnrVertScl[2] = { 0.5, 0.5 };
    CagdRType Center[3], Pt[3];
    CagdBBoxStruct UVWDmn;
    IrtHmgnMatType Mat2Glb, MatScl;
    IritPrsrObjectStruct *TmpTile;
    
    const TrivTVStruct 
	*PrmTV = (const TrivTVStruct *) CBData -> CBFuncData;

    /* Transformation matrix to transform to global uvw coordinates. */
    IritMiscMatGenMatTrans(CBData -> DefMapDmnMin[0],
			   CBData -> DefMapDmnMin[1],
			   CBData -> DefMapDmnMin[2], Mat2Glb);
    IritMiscMatGenMatScale(CBData -> DefMapDmnMax[0] -
			   CBData -> DefMapDmnMin[0],
			   CBData -> DefMapDmnMax[1] -
			   CBData -> DefMapDmnMin[1],
			   CBData -> DefMapDmnMax[2] -
			   CBData -> DefMapDmnMin[2],
			   MatScl);
    IritMiscMatMultTwo4by4(Mat2Glb, MatScl, Mat2Glb);

    /* Fetch Size parameters from the macro deformation map. */
    CAGD_BBOX_INIT_3D(UVWDmn);
    IritMiscMatMultPtby4by4(UVWDmn.Min, CBData -> TileLclDmnMin, Mat2Glb);
    IritMiscMatMultPtby4by4(UVWDmn.Max, CBData -> TileLclDmnMax, Mat2Glb);

    /* CntrThickness is evaluated at the center point. */
    IRIT_PT_BLEND(Center, UVWDmn.Min, UVWDmn.Max, 0.5);
    TRIV_TV_EVAL_SCALAR(PrmTV, Center[0], Center[1], Center[2], 
			&CntrThickness);
    if (CntrThickness > 0.5 - IRIT_EPS)
	CntrThickness = 0.5 - IRIT_EPS;

    /* Compute the parametric thickness for support tiles. */
    for (i = 0; i < 8; i++) {
	int UVId = i % 4;
	Pt[0] = (UVId % 2 == 0) ? UVWDmn.Min[0] : UVWDmn.Max[0];
	Pt[1] = (UVId / 2 == 0) ? UVWDmn.Min[1] : UVWDmn.Max[1];
	Pt[2] = (i / 4 == 0) ? UVWDmn.Min[2] : UVWDmn.Max[2];
	TRIV_TV_EVAL_SCALAR(PrmTV, Pt[0], Pt[1], Pt[2], &TileCrnrSizes[i]);
	if (TileCrnrSizes[i] > 0.5 - IRIT_EPS)
	    TileCrnrSizes[i] = 0.5 - IRIT_EPS;
    }
    
    TmpTile = IritUserMicroDiagTile1(CntrThickness, 1.0, TileCrnrSizes,
				     CrnrVertScl, NULL, TRUE, 0.0, NULL, NULL);

    if (TmpTile) {
	Tile = IritGeomTransformObjectList(TmpTile, CBData -> Mat);
	IritPrsrFreeObject(TmpTile);
    }
    else
	Tile = NULL;
    
    return Tile;
}

/*****************************************************************************
* DESCRIPTION:								     M
*   Build support as a microstructure, to a given polygonal model.	     M
*									     *
* PARAMETERS:								     M
*   PlObj:   A polygonal object	to build support for.			     M
*   Params:  The support parameters.					     M
*									     *
* RETURN VALUE:								     M
*   IritPrsrObjectStruct *:	 The built support as a	microstructure.	     M
*									     *
* SEE ALSO:								     M
*   IritUserMicroGenAMSupportTPath					     M
*									     *
* KEYWORDS:								     M
*   IritUserMicroGenAMSupport						     M
*****************************************************************************/
IritPrsrObjectStruct *IritUserMicroGenAMSupport(
			   const IritPrsrObjectStruct *PlObj,
			   const IritUserMicroGenAMSupportParamStruct *Params)

{
    int i, j, k, NumTiles[3], MSTopoSizes[3],
	FetchBndry = IritMiscAttrIDGetObjectIntAttrib(PlObj,
					  IRIT_ATTR_CREATE_ID(Bndry)) == TRUE,
	UsePrmThickness = Params -> PrmThicknessTV != NULL;
    CagdRType TileCrnrSizes[8], ActualTileHeight, BndryVals[6],
	CrnrVertScl[2] = { 0.5, 0.5 };
    struct IritGeomPolyBVHStruct *PolyBVH;
    GMBBBboxStruct ModelBBox, TVBBox;
    TrivTVStruct *BoxTV;
    UserMicroParamStruct MSParam;
    UserMicroSupportTopoInfoStruct ***MSTopology;
    UserMicroRegularParamStruct *MSRegularParam;
    IritPrsrObjectStruct *MSTile, *MS, *Spikes, *All, *Path,
	*MultiResTiles = NULL;

    assert(IRIT_PRSR_IS_POLY_OBJ(PlObj));

    IritGeomBBComputeBboxObject(PlObj, &ModelBBox, FALSE);

    for (i = 0; i < 3; i++) {
	/* Expand domain a bit, to fit the tiles below the model etc. */
	TVBBox.Min[i] = ModelBBox.Min[i] - Params -> SupportExtent[i];
	TVBBox.Max[i] = ModelBBox.Max[i] + Params -> SupportExtent[i];
	if (i == 2) {
	    ActualTileHeight = Params -> TileSize *
			       tan(IRIT_DEG2RAD(Params -> TileDiagEdgeSlope));
	    NumTiles[2] =
	           (int) ((TVBBox.Max[2] - TVBBox.Min[2]) / ActualTileHeight);
	}
	else
	    NumTiles[i] = (int) ((TVBBox.Max[i] - TVBBox.Min[i]) /
							Params -> TileSize);
	if (NumTiles[i] <= 1) {
	    IRIT_WARNING_MSG("Too few tiles for an AM support (tile size too large!?). Aborted.\n");
	    return NULL;
	}
    }

    /* Verify that the support base is below the bottom of the model by at */
    /* least tile height.  if not - enforce it.				   */
    if (Params -> SupportExtent[2] > 0.0 &&
	ActualTileHeight > Params -> SupportExtent[2]) {
	TVBBox.Min[2] -= ActualTileHeight;
	NumTiles[2]++;
	IRIT_WARNING_MSG("Base Z extent too low (less than tile height) and was extended.\n");
    }

    /* Verify that the factor of the tip length is not too small. */
    if (Params -> TipLength < 0.05) {
	IRIT_WARNING_MSG("The factor of tip length is too small - consider adjusting TipLength.\n");
    }

    IRIT_WARNING_MSG_PRINTF("Generating support tiles: (%d x %d x %d)\n",
			    NumTiles[0], NumTiles[1], NumTiles[2]);

    /* Generate the support microstructure. */
    BoxTV = IritTrivNSPrimBox(TVBBox.Min[0], TVBBox.Min[1], TVBBox.Min[2],
			      TVBBox.Max[0], TVBBox.Max[1], TVBBox.Max[2]);

    IRIT_ZAP_MEM(&MSParam, sizeof(UserMicroParamStruct));
    MSParam.TilingType = USER_MICRO_TILE_REGULAR;
    MSParam.DeformMV = IritMvarCnvrtTVToMV(BoxTV);
    MSParam.ApproxLowOrder = 3;

    MSRegularParam = &MSParam.U.RegularParam;
    MSRegularParam -> Tile = NULL;
    MSRegularParam -> TilingStepMode = TRUE;
    MSRegularParam -> MaxPolyEdgeLen = 1.0;
    for (i = 0; i < 3; ++i) {
	MSRegularParam -> TilingSteps[i].TilesPerIntervals =
			    (CagdRType *) IritMalloc(sizeof(CagdRType) * 2);
	MSRegularParam -> TilingSteps[i].Len = 1;
    }
    MSRegularParam -> TilingSteps[0].TilesPerIntervals[0] = NumTiles[0];
    MSRegularParam -> TilingSteps[1].TilesPerIntervals[0] = NumTiles[1];
    MSRegularParam -> TilingSteps[2].TilesPerIntervals[0] = NumTiles[2];

    if (!UsePrmThickness) {		  /* With constant thickness tiles. */
	for (i = 0; i < 8; i++)
	    TileCrnrSizes[i] = Params -> TileEndEdgeThickness;
	if ((MSTile = IritUserMicroDiagTile1(Params -> TileCntrThickness, 1.0,
					     TileCrnrSizes, CrnrVertScl, NULL,
					     TRUE, 0.0, NULL, NULL)) == NULL)
	    return NULL;
	
	MSRegularParam -> Tile = IritUserMicroTileNew(MSTile);	
	/* Construct support MS. */
	MS = IritUserMicroStructComposition(&MSParam);  
	IritUserMicroTileFree(MSRegularParam -> Tile);
    }
    else {				/* With parametric thickness tiles. */
	MSRegularParam -> PreProcessCBFunc = UserMicroPrmSupportTilePreProcess;
	MSRegularParam -> CBFuncData = (void *)(Params -> PrmThicknessTV);	
	/* Construct support MS. */
	MS = IritUserMicroStructComposition(&MSParam);  
    }
    IritMvarMVFree(MSParam.DeformMV);
 
    for (i = 0; i < 3; ++i)
	IritFree(MSRegularParam -> TilingSteps[i].TilesPerIntervals);

    /* Purge tiles that are intersecting/inside the given PlObj. */
    PolyBVH = IritGeomPolyBVHCreate(PlObj -> U.Pl);
    for (k = 0; (MSTile = IritPrsrListObjectGet(MS, k)) != NULL;) {
	CagdPType Ctr;
	GMBBBboxStruct TileBBox;

	IritGeomBBComputeBboxObject(MSTile, &TileBBox, FALSE);
	IRIT_PT_BLEND(Ctr, TileBBox.Max, TileBBox.Min, 0.5);

	/* Check if the tile's BBox intersects with or insides the model. */
	if (IritGeomPolyBVHBoxPolyInter(&TileBBox, PolyBVH) ||
	    IritGeomPolyBVHPointInsidePolys(Ctr, PolyBVH))
	    IritPrsrListObjectDelete(MS, k, TRUE); /* Purge this tile. */
	else 
	    k++; /* Keep this tile. */
    }

    /* Figure out the neighborhood topology we have as a 3D grid of tiles. */
    if ((MSTopology = UserMicroGenAMSprtTopology(MS, MSTopoSizes,
						 Params)) == NULL) {
	IritGeomPolyBVHFree(PolyBVH);
	IritPrsrFreeObject(MS);
	return NULL;
    }

    /* Thicken the lower arms of the base tiles. */
    if (!UsePrmThickness)
	MS = UserMicroThickenBaseTileArms(MS, MSTopoSizes, MSTopology, BoxTV,
					  Params);
    else
	MS = UserMicroThickenBaseTilePrmArms(MS, MSTopoSizes, MSTopology, 
					     BoxTV, Params);

    /* Shoot vertical up/down rays from the tiles that are on top. */
    UserMicroShootUpDownRays(Params, PlObj, PolyBVH, MS,
			     MSTopology, MSTopoSizes);

    /* Filter out tiles and arms that contribute nothing to the support. */
    MS = UserMicroFilterInactiveTilesAndArms(MS, MSTopoSizes, MSTopology);

    /* Multi-resolution tiling. */
    if (Params -> MultiResTiling) {
	MultiResTiles = UserMicroMultiResolutionTiling(MS, MSTopoSizes,
						       MSTopology, BoxTV,
						       Params);
    }

    /* Remove invalid tiles. */
    MS = UserMicroRemoveInvalidTiles(MS);

    IritTrivTVFree(BoxTV);

    /* Create the spikes out of the intersecting ray. */
    Spikes = UserMicroGenRaySpikes(MS, MSTopoSizes, MSTopology, Params);

    /* Replace the spikes to the spikes with the closest points. */
    if (Params -> SupportClosestPt) {
	Spikes = UserMicroReplaceSpikesToClosestSpikes(MS, MSTopoSizes, 
						       MSTopology, PolyBVH, 
						       Params, Spikes);
    }

    /* Remove inactive arms. */
    if (Params -> MultiResTiling) {
	int St,
	    Stride = 2;

	for (k = 0; k < MSTopoSizes[2]; k++)
	    for (j = 0; j < MSTopoSizes[1]; j++)
		for (i = 0; i < MSTopoSizes[0]; i++)
		    Stride = IRIT_MAX(Stride, MSTopology[i][j][k].Stride);
	Stride *= 2;

    	/* Check inactive arms based on adjacency information. */
	for (St = 2; St < Stride; St *= 2) {
	    for (k = 0; k < MSTopoSizes[2]; k++) {
		for (j = 0; j < MSTopoSizes[1]; j++) {
		    i = 0;
		    do {
			if (MSTopology[i][j][k].Stride != St) {
			    i++;
			    continue;
			}

			UserMicroMultiResFilterActiveArms(MSTopoSizes,
							  MSTopology,
							  i, j, k);
			i += St; /* Move to the next tile. */
		    }
		    while (i < MSTopoSizes[0] - St + 1);
		}
	    }
	}

	/* Additonally filter out redundant arms and tiles based on	    */
	/* connectivity information.					    */
	UserMicroMultiResFilterInactiveTilesAndArms(MSTopoSizes, MSTopology);

	/* Move activity arm information of multi-resolution tiles into */
	/* one topology (to make it easy to purge). */
	UserMicroMultiResMoveActiveArmsInfo(MSTopoSizes, MSTopology);

	/* Remove inactive arms altogether. */
	for (k = 0; k < MSTopoSizes[2]; k++) {
	    for (j = 0; j < MSTopoSizes[1]; j++) {
		i = 0;
		do {
		    UserMicroSupportTopoInfoStruct
			*Topo = &MSTopology[i][j][k];
		    if (Topo -> Stride > 0) {
			int Inv;
			UserMicroPurgeInactiveArms(Topo -> Tile,
						   Topo -> ActiveArms);
			Inv = IritMiscAttrIDGetObjectIntAttrib(
			    Topo -> Tile,
			    IRIT_ATTR_CREATE_ID(Invalid));
			if (Inv == TRUE) {
			    Topo -> Tile = NULL;
			}
 			i += Topo -> Stride; /* Move to the next tile. */
		    }
		    else {
			if (Topo -> Tile != NULL) {
			    IritMiscAttrIDSetObjectIntAttrib(Topo -> Tile,
						  IRIT_ATTR_CREATE_ID(Invalid),
						  TRUE);
			    Topo -> Tile = NULL;
			}
			i++;
		    }
		}
	    	while (i < MSTopoSizes[0]);
	    }
	}

    }
    else {
	MS = UserMicroFilterInactiveTilesAndArms(MS, MSTopoSizes, MSTopology);

	for (k = 0; k < MSTopoSizes[2]; k++) {
	    for (j = 0; j < MSTopoSizes[1]; j++) {
		for (i = 0; i < MSTopoSizes[0]; i++) {
		    UserMicroSupportTopoInfoStruct
			*TlTopo = &MSTopology[i][j][k];

		    if (TlTopo -> Tile == NULL)
			continue;

		    UserMicroPurgeInactiveArms(TlTopo -> Tile,
					       TlTopo -> ActiveArms);
		}
	    }
	}
    }

    /* Remove additional invalid tiles. */
    MS = UserMicroRemoveInvalidTiles(MS);

    /* Color the tiles according to their support status. */
    for (k = 0; k < MSTopoSizes[2]; k++) {
	for (j = 0; j < MSTopoSizes[1]; j++) {
	    for (i = 0; i < MSTopoSizes[0]; i++) {
		if (MSTopology[i][j][k].Tile != NULL) {
		    /* Top: red, bottom: green, both: yellow, none: blue. */
		    if (MSTopology[i][j][k].TopTile && 
			!MSTopology[i][j][k].BotTile) {
			IritMiscAttrSetObjectRGBColor(MSTopology[i][j][k].Tile,
						      200, 0, 0);
		    }
		    else if (!MSTopology[i][j][k].TopTile &&
			      MSTopology[i][j][k].BotTile) {
			IritMiscAttrSetObjectRGBColor(MSTopology[i][j][k].Tile,
						      0, 200, 0);
		    }
		    else if (MSTopology[i][j][k].TopTile && 
			     MSTopology[i][j][k].BotTile) {
			IritMiscAttrSetObjectRGBColor(MSTopology[i][j][k].Tile,
						      200, 200, 0);
		    }
		    else {
			IritMiscAttrSetObjectRGBColor(MSTopology[i][j][k].Tile,
						      0, 0, 200);
		    }
		}
	    }
	}
    }

    All = IritPrsrGenLISTObject(MS);

    if (MultiResTiles) 
	IritPrsrListObjectAppend(All, MultiResTiles);	

    /* Mark the base tiles, in the MS. */
    IRIT_ZAP_MEM(BndryVals, 6 * sizeof(CagdRType));
    BndryVals[4] = TVBBox.Min[2];		      /* Set the WMin value. */
    IritUserMicroMarkMSOnBndry(All, TRIV_W_MIN_BNDRY, BndryVals, FALSE);

    if (Spikes != NULL) {
	IritUserMicroMarkSupportMSInfo(Spikes, USER_MICRO_SUPPORT_TIP, -1);
	IritPrsrListObjectAppend(All, Spikes);
    }

    if (FetchBndry) {
        IritPrsrObjectStruct
	    *BndrySrf = IritUserMicroFetchMarkedMSBndry(All);

	IritPrsrListObjectAppend(All, BndrySrf);
    }

    /* Generate path. */
    if (Params -> OutputType > 0) {
	Path = IritUserMicroGenAMSupportTPath(Spikes, PlObj, MSTopoSizes,
		(const struct UserMicroSupportTopoInfoStruct ***) MSTopology, 
		Params);

	if (Params -> OutputType == 1 || Params -> OutputType == 2) {
	    IritPrsrFreeObjectList(All);
	    All = Path;
	}
	else if (Params -> OutputType == 3 || Params -> OutputType == 4) {
	    IritPrsrListObjectAppend(All, Path);
	}
	else 
	    fprintf(stderr, "Invalid output type.\n");
    }

    /* Free memory. */
    UserMicroFreeAMSprtTopology(MSTopology, MSTopoSizes[0], MSTopoSizes[1]);
    IritGeomPolyBVHFree(PolyBVH);

    return All;
}

/* #define DEBUG_TEST_MICRO_SUPPORT */
#ifdef DEBUG_TEST_MICRO_SUPPORT
int main(int argc, char **argv)
{
    int Handler;

    if (argc == 2) {
	if ((Handler = IritPrsrOpenDataFile(argv[1], TRUE, TRUE)) >= 0) {
	    IritPrsrObjectStruct *Support,
		*PObj = IritPrsrGetObjects(Handler);

	    /* Done with file - close it. */
	    IritPrsrCloseStream(Handler, TRUE);

	    if (IRIT_PRSR_IS_POLY_OBJ(PObj)) {
	        UserMicroSupportParamStruct Params;

		IRIT_ZAP_MEM(&Params, sizeof(UserMicroSupportParamStruct));

		Params.TileSize = 0.15;
		Params.TileCntrThickness = 0.15;
		Params.TileEndEdgeThickness = 0.15;
		Params.TileDiagEdgeSlope = 55;
		Params.TileHaveVertBars = TRUE;
		Params.SupportExtent[0] = Params.TileSize * 0.5;
		Params.SupportExtent[1] = Params.TileSize * 0.5;
		Params.SupportExtent[2] = Params.TileSize *
			    tan(IRIT_DEG2RAD(Params.TileDiagEdgeSlope)) * 1.2;
		Params.TipMaxLength = Params.TileSize * 3.0;
		Params.TipMinSlope = 45.0;
		Params.ModelMinSlope = 20.0;

		Support = IritUserMicroGenAMSupport(PObj, &Params);

		IritPrsrPutObjectToFile3("MS_sprt.itd", Support, 0);
		IritPrsrFreeObject(Support);
	    }
	}
	else {
	    fprintf(stderr, "Failed to open file \"%s\"\n", argv[1]);
	    exit(1);
	}
    }
    else {
	fprintf(stderr, "Usage: TestMicroSupport PolyGeom.itd\n");
	exit(2);
    }

    return 0;
}
#endif /* DEBUG_TEST_MICRO_SUPPORT */
