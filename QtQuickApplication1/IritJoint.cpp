//
// IritJoint - implementation: builds the pin loft, places it on a face, and
// applies it to a piece as a boolean, all inside IritGuard.
//

#include "IritJoint.h"

#include "IritGuard.h"
#include "IritMesh.h"
#include "IritSolid.h"

extern "C" {
#include "inc_irit/cagd_lib.h"
#include "inc_irit/bool_lib.h"
}

#include <cmath>

namespace {

constexpr double kPi = 3.14159265358979323846;

using IritSolid::closeLists;
using IritSolid::signedVolume;
static IritPrsrObjectStruct *meshToIrit(const MeshData &m)
{
    return IritSolid::fromMesh(m, IritSolid::Winding::Outward);
}

struct PinSection { double z, scale; };

const PinSection kElberPin[] = {
    { -1.00, 0.000 },
    { -1.00, 1.000 },
    {  0.00, 0.750 },
    {  0.20, 0.775 },
    {  0.30, 0.800 },
    {  0.40, 0.800 },
    {  0.60, 0.600 },
    {  0.60, 0.000 }
};
const int kElberPinCount = int(sizeof(kElberPin) / sizeof(kElberPin[0]));

CagdCrvStruct *buildSections(const JointParams &p)
{
    CagdCrvStruct *head = NULL, *tail = NULL;

    const double zScale = p.height > 1e-9 ? p.height / 0.60 : 1.0;

    for (int i = 0; i < kElberPinCount; ++i) {
        const double r = p.pinRadius * kElberPin[i].scale;
        const double z = (kElberPin[i].z <= -0.99)
                             ? -p.baseSink * p.height
                             : kElberPin[i].z * zScale;

        CagdPtStruct centre;
        centre.Pt[0] = centre.Pt[1] = centre.Pt[2] = 0.0;
        CagdCrvStruct *crv =
            IritCagdBspCrvCreateCircle(&centre, r > 1e-9 ? r : p.pinRadius);
        if (crv == NULL)
            continue;

        IrtHmgnMatType mScl, mTrz, mAll;
        IritMiscMatGenMatUnifScale(r > 1e-9 ? 1.0 : 0.0, mScl);
        IritMiscMatGenMatTrans(0.0, 0.0, z, mTrz);
        IritMiscMatMultTwo4by4(mAll, mScl, mTrz);
        IritCagdCrvMatTransform2(crv, mAll);

        crv -> Pnext = NULL;
        if (tail != NULL) { tail -> Pnext = crv; tail = crv; }
        else              { head = tail = crv; }
    }
    return head;
}

IritPrsrObjectStruct *buildTool(const JointParams &p)
{
    CagdCrvStruct *sections = buildSections(p);
    if (sections == NULL)
        return NULL;

    CagdSrfStruct *srf = IritCagdSrfFromCrvs(sections, 3, CAGD_END_COND_OPEN,
                                             NULL);
    IritCagdCrvFreeList(sections);
    if (srf == NULL)
        return NULL;

    IritPrsrObjectStruct *srfObj = IritPrsrGenSRFObject(srf);
    if (srfObj == NULL) {
        IritCagdSrfFree(srf);
        return NULL;
    }

    IPFreeformConvStateStruct st = IritPrsrFFCState;
    st.Talkative           = FALSE;
    st.DumpObjsAsPolylines = FALSE;
    st.DrawFFGeom          = TRUE;
    st.DrawFFMesh          = FALSE;
    st.ComputeNrml         = TRUE;
    st.ComputeUV           = FALSE;
    st.FineNess            = p.fineNess;

    IritPrsrObjectStruct *polys =
        IritPrsrConvertFreeFormHierachy(srfObj, &st, FALSE, FALSE);

    for (IritPrsrObjectStruct *o = polys; o != NULL; o = o -> Pnext)
        if (IRIT_PRSR_IS_POLY_OBJ(o) && o -> U.Pl != NULL)
            IritPrsrOpenPolysToClosed(o -> U.Pl);

    return polys;
}

void placementMatrix(const JointPlacement &j, const JointParams &p,
                     IrtHmgnMatType out)
{
    IrtHmgnMatType mScl, mSize, mRot, mTr, mTmp;

    if (j.pin)
        IritMiscMatGenUnitMat(mScl);
    else
        IritMiscMatGenMatScale(p.clearanceXY, p.clearanceXY, p.clearanceZ, mScl);

    IritMiscMatGenMatUnifScale(j.size, mSize);

    switch (j.axis) {
        case 0:  IritMiscMatGenMatRotY1( kPi * 0.5, mRot); break;
        case 1:  IritMiscMatGenMatRotX1(-kPi * 0.5, mRot); break;
        default: IritMiscMatGenUnitMat(mRot);              break;
    }
    IritMiscMatGenMatTrans(j.at[0], j.at[1], j.at[2], mTr);

    IritMiscMatMultTwo4by4(mTmp, mScl, mSize);
    IritMiscMatMultTwo4by4(out,  mTmp, mRot);
    IritMiscMatMultTwo4by4(mTmp, out,  mTr);
    IRIT_HMGN_MAT_COPY(out, mTmp);
}

struct Ctx {
    IritPrsrObjectStruct  *piece;
    const JointParams     *p;
    const JointPlacement  *places;
    int                    nPlaces;
    int                    applied;
    int                    declined;
};

void doApply(void *v)
{
    Ctx *c = static_cast<Ctx *>(v);

    IritPrsrObjectStruct *unitTool = buildTool(*c -> p);
    if (unitTool == NULL)
        return;

    for (int i = 0; i < c -> nPlaces; ++i) {
        const JointPlacement &j = c -> places[i];

        IrtHmgnMatType mat;
        placementMatrix(j, *c -> p, mat);

        IritPrsrObjectStruct *tool = IritGeomTransformObject(unitTool, mat);
        if (tool == NULL)
            continue;

        closeLists(c -> piece);
        closeLists(tool);

        IritPrsrObjectStruct *res = j.pin
            ? IritBooleanOR (c -> piece, tool)
            : IritBooleanSUB(c -> piece, tool);
        closeLists(res);

        if (res != NULL && res != c -> piece) {
            IritPrsrFreeObject(c -> piece);
            c -> piece = res;
            ++c -> applied;
        }
        else {
            ++c -> declined;
            if (res != NULL && res != c -> piece)
                IritPrsrFreeObject(res);
        }
        IritPrsrFreeObject(tool);
    }

    IritPrsrFreeObject(unitTool);
}

struct ToolCtx {
    const JointParams    *p;
    IritPrsrObjectStruct *out;
};

void doBuildTool(void *v)
{
    ToolCtx *c = static_cast<ToolCtx *>(v);
    c -> out = buildTool(*c -> p);
}

}

double IritJoint::sizeForFace(const JointParams &p, double faceMin, double depth)
{
    const double toolR = p.pinRadius;
    if (toolR <= 1e-9 || faceMin <= 0.0)
        return 0.0;

    double size = p.faceFraction * faceMin / (2.0 * toolR);
    if (p.height > 1e-9)
        size = qMin(size, 0.8 * depth / p.height);
    if (p.maxSize > 0.0)
        size = qMin(size, p.maxSize);
    if (p.minSize > 0.0 && size < p.minSize)
        return 0.0;

    if (p.minPinThickness > 0.0 &&
        thinnestFeature(p, size) < p.minPinThickness)
        return 0.0;

    return size > 1e-9 ? size : 0.0;
}

QVector<QVector<JointPlacement> > IritJoint::planPlacementsFor(
        const QVector<PuzzlePiece> &pieces,
        const Planner::Graph &g,
        const Planner::JointSet &keep,
        const JointParams &p,
        int *skipped)
{
    QVector<QVector<JointPlacement> > out(pieces.size());
    int dropped = 0;

    for (int ci = 0; ci < g.contactCount(); ++ci) {
        if (ci >= keep.size() || !keep[ci])
            continue;

        const Planner::Contact &c = g.contacts[ci];
        if (c.lowSide < 0 || c.highSide >= pieces.size())
            continue;

        const double size = sizeForFace(p, qMin(c.ext[0], c.ext[1]), c.depth);
        if (size <= 0.0) {
            ++dropped;
            continue;
        }

        const int u = (c.axis + 1) % 3, v = (c.axis + 2) % 3;
        JointPlacement j;
        j.axis      = c.axis;
        j.size      = size;
        j.at[c.axis] = c.plane;
        j.at[u] = 0.5 * (qMax(pieces[c.lowSide].p0[u], pieces[c.highSide].p0[u])
                       + qMin(pieces[c.lowSide].p1[u], pieces[c.highSide].p1[u]));
        j.at[v] = 0.5 * (qMax(pieces[c.lowSide].p0[v], pieces[c.highSide].p0[v])
                       + qMin(pieces[c.lowSide].p1[v], pieces[c.highSide].p1[v]));

        j.pin = true;
        out[c.lowSide].append(j);
        j.pin = false;
        out[c.highSide].append(j);
    }

    if (skipped != nullptr)
        *skipped = dropped;
    return out;
}

QVector<QVector<JointPlacement> > IritJoint::planPlacements(
        const QVector<PuzzlePiece> &pieces,
        const QVector<PuzzleDivider::Neighbours> &links,
        const JointParams &p, int *skipped)
{
    QVector<QVector<JointPlacement> > out(pieces.size());
    int dropped = 0;

    const double toolR = p.pinRadius;
    if (toolR <= 1e-9) {
        if (skipped != nullptr) *skipped = links.size();
        return out;
    }

    for (const PuzzleDivider::Neighbours &l : links) {
        if (l.a < 0 || l.b < 0 || l.a >= pieces.size() || l.b >= pieces.size())
            continue;
        const int axis = l.axis;
        if (axis < 0 || axis > 2)
            continue;

        int lo = l.a, hi = l.b;
        if (pieces[lo].p0[axis] > pieces[hi].p0[axis])
            qSwap(lo, hi);

        const PuzzlePiece &A = pieces[lo], &B = pieces[hi];
        const int u = (axis + 1) % 3, v = (axis + 2) % 3;

        const double uLo = qMax(A.p0[u], B.p0[u]), uHi = qMin(A.p1[u], B.p1[u]);
        const double vLo = qMax(A.p0[v], B.p0[v]), vHi = qMin(A.p1[v], B.p1[v]);
        if (uHi - uLo <= 0.0 || vHi - vLo <= 0.0) {
            ++dropped;
            continue;
        }

        const double depth = qMin(A.p1[axis] - A.p0[axis], B.p1[axis] - B.p0[axis]);
        const double size  = sizeForFace(p, qMin(uHi - uLo, vHi - vLo), depth);
        if (size <= 0.0) {
            ++dropped;
            continue;
        }

        JointPlacement j;
        j.axis    = axis;
        j.size    = size;
        j.at[axis] = 0.5 * (A.p1[axis] + B.p0[axis]);
        j.at[u]    = 0.5 * (uLo + uHi);
        j.at[v]    = 0.5 * (vLo + vHi);

        j.pin = true;
        out[lo].append(j);
        j.pin = false;
        out[hi].append(j);
    }

    if (skipped != nullptr)
        *skipped = dropped;
    return out;
}

bool IritJoint::apply(MeshData *mesh, const QVector<JointPlacement> &places,
                      const JointParams &p, QString *error, int *applied,
                      int *declined)
{
    if (applied != nullptr)
        *applied = 0;
    if (declined != nullptr)
        *declined = 0;
    if (mesh == nullptr || places.isEmpty())
        return true;

    IritPrsrObjectStruct *piece = meshToIrit(*mesh);
    if (piece == NULL) {
        if (error != nullptr)
            *error = QStringLiteral("piece has no usable triangles");
        return false;
    }

    Ctx c;
    c.piece    = piece;
    c.p        = &p;
    c.places   = places.constData();
    c.nPlaces  = places.size();
    c.applied  = 0;
    c.declined = 0;

    if (!IritGuard::run(&c, doApply)) {
        if (error != nullptr)
            *error = IritGuard::lastError();
        return false;
    }

    if (c.applied == 0) {
        IritPrsrFreeObject(c.piece);
        if (error != nullptr)
            *error = QStringLiteral("all %1 boolean(s) declined - the piece is "
                                    "probably not closed").arg(places.size());
        return false;
    }

    MeshData out;
    QString  convErr;
    if (!IritMesh::tessellate(c.piece, &out, p.fineNess, &convErr)) {
        if (error != nullptr)
            *error = convErr;
        return false;
    }
    if (out.isEmpty()) {
        if (error != nullptr)
            *error = QStringLiteral("boolean produced no geometry");
        return false;
    }

    *mesh = out;
    if (applied != nullptr)
        *applied = c.applied;
    if (declined != nullptr)
        *declined = c.declined;
    return true;
}

double IritJoint::thinnestFeature(const JointParams &p, double size)
{
    double thinnest = 1e30;
    for (int i = 0; i < kElberPinCount; ++i)
        if (kElberPin[i].scale > 1e-6)
            thinnest = qMin(thinnest, kElberPin[i].scale);
    return 2.0 * p.pinRadius * thinnest * size;
}

bool IritJoint::preview(MeshData *out, const JointParams &p, QString *error)
{
    ToolCtx c;
    c.p   = &p;
    c.out = NULL;

    if (!IritGuard::run(&c, doBuildTool)) {
        if (error != nullptr)
            *error = IritGuard::lastError();
        return false;
    }
    if (c.out == NULL) {
        if (error != nullptr)
            *error = QStringLiteral("could not loft the joint");
        return false;
    }
    return IritMesh::tessellate(c.out, out, p.fineNess, error);
}
