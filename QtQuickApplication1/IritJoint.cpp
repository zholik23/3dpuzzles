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

// --------------------------------------------------------------- mesh in ---
//
// MeshData is a flat triangle soup; IRIT wants a chain of polygons, each with
// its plane equation set. Booleans need that plane, so IritPrsrUpdatePolyPlane
// is not optional - a polygon without one is silently skipped.

// The mesh-to-IRIT conversion and the list closing live in IritSolid now: the
// cage-vs-model intersection needs exactly the same two fixes, and one copy is
// better than two that can drift apart.
using IritSolid::closeLists;
using IritSolid::signedVolume;
static IritPrsrObjectStruct *meshToIrit(const MeshData &m)
{
    return IritSolid::fromMesh(m, IritSolid::Winding::Outward);
}

// ------------------------------------------------------------- the joint ---
//
// Elber's pin, section for section out of PuzTile() in puz_vol.irt. Built once
// at unit size about +Z, then transformed into place per joint.
//
//   Crc = pcircle( vector( 0, 0, 0 ), 0.2 )
//   sFromCrvs( list( Crc          * tz( -0.02 ),
//                    Crc * sc( 0.75  ),
//                    Crc * sc( 0.775 ) * tz( 0.2 ),
//                    Crc * sc( 0.8   ) * tz( 0.3 ),
//                    Crc * sc( 0.8   ) * tz( 0.4 ),
//                    Crc * sc( 0.6   ) * tz( 0.6 ),
//                    Crc * sc( 0.0   ) * tz( 0.6 ) ), 3, kv_open )

struct PinSection { double z, scale; };

const PinSection kElberPin[] = {
    { -1.00, 0.000 },     // cap closing the sunk base: without it the loft is a
                          // tube open at the root, and the UNION that makes the
                          // pin loses material there. Elber never needs this -
                          // his open root is closed by the tile it grows from -
                          // but ours is a free-standing tool used twice.
    { -1.00, 1.000 },     // the sunk base ring - depth from p.baseSink
    {  0.00, 0.750 },
    {  0.20, 0.775 },
    {  0.30, 0.800 },     // the barb swells here ...
    {  0.40, 0.800 },
    {  0.60, 0.600 },     // ... and necks in before the tip, which is what holds
    {  0.60, 0.000 }      // the cap: without it the loft is not a solid
};
const int kElberPinCount = int(sizeof(kElberPin) / sizeof(kElberPin[0]));

CagdCrvStruct *buildSections(const JointParams &p)
{
    CagdCrvStruct *head = NULL, *tail = NULL;

    // His table is written against a 0.6-tall pin; scaling z keeps the profile
    // proportional if the height is changed.
    const double zScale = p.height > 1e-9 ? p.height / 0.60 : 1.0;

    for (int i = 0; i < kElberPinCount; ++i) {
        const double r = p.pinRadius * kElberPin[i].scale;
        // The sentinel -1 marks the sunk base section; everything else is
        // Elber's table scaled to the requested height.
        const double z = (kElberPin[i].z <= -0.99)
                             ? -p.baseSink * p.height
                             : kElberPin[i].z * zScale;

        CagdPtStruct centre;
        centre.Pt[0] = centre.Pt[1] = centre.Pt[2] = 0.0;
        // A zero-radius circle cannot be constructed, so the cap is built full
        // size and then collapsed by a zero scale - Elber's `Crc * sc( 0.0 )`.
        // Building it at a tiny radius instead is NOT the same thing: it leaves
        // a real ring of control points, the loft stays open there, and the tip
        // came out with 16 unmatched edges.
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

// The lofted surface, tessellated into a closed polygonal solid.
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

    // Consumes srfObj and returns a chain; the loft is one surface, so the
    // chain is one polygonal object.
    IritPrsrObjectStruct *polys =
        IritPrsrConvertFreeFormHierachy(srfObj, &st, FALSE, FALSE);

    // The tessellator hands back OPEN vertex lists, which the booleans refuse.
    for (IritPrsrObjectStruct *o = polys; o != NULL; o = o -> Pnext)
        if (IRIT_PRSR_IS_POLY_OBJ(o) && o -> U.Pl != NULL)
            IritPrsrOpenPolysToClosed(o -> U.Pl);

    return polys;
}

// Places the unit tool: clearance scale, then size, then onto the face normal,
// then out to the site. IRIT is row-vector, so the multiplication order reads
// the same as the `Obj * m1 * m2` chain in the .irt.
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
        case 0:  IritMiscMatGenMatRotY1( kPi * 0.5, mRot); break;  // +Z -> +X
        case 1:  IritMiscMatGenMatRotX1(-kPi * 0.5, mRot); break;  // +Z -> +Y
        default: IritMiscMatGenUnitMat(mRot);              break;
    }
    IritMiscMatGenMatTrans(j.at[0], j.at[1], j.at[2], mTr);

    IritMiscMatMultTwo4by4(mTmp, mScl, mSize);
    IritMiscMatMultTwo4by4(out,  mTmp, mRot);
    IritMiscMatMultTwo4by4(mTmp, out,  mTr);
    IRIT_HMGN_MAT_COPY(out, mTmp);
}

// ------------------------------------------------------------- the guard ---
//
// IRIT's fatal errors longjmp, so everything that can raise one runs inside a
// POD context with no C++ destructors in scope.

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
            // IRIT hands back the first operand when the two do not intersect.
            // That is a real result - the piece is unchanged - not a crash.
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

} // namespace

// ------------------------------------------------------------------ api ----

double IritJoint::sizeForFace(const JointParams &p, double faceMin, double depth)
{
    // Widest point of the loft, in joint-local units: the base collar that sits
    // just inside the face.
    const double toolR = p.pinRadius;
    if (toolR <= 1e-9 || faceMin <= 0.0)
        return 0.0;

    // Span `faceFraction` of the narrower side, then hold the pin short of the
    // depth of both pieces it lives between.
    double size = p.faceFraction * faceMin / (2.0 * toolR);
    if (p.height > 1e-9)
        size = qMin(size, 0.8 * depth / p.height);
    if (p.maxSize > 0.0)
        size = qMin(size, p.maxSize);
    if (p.minSize > 0.0 && size < p.minSize)
        return 0.0;

    // Printability floor: a joint thinner than the printer can lay down is not
    // a joint. Leaving the face bare is the honest outcome.
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
            continue;                       // the planner left this face bare

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

    // Widest point of the loft, in joint-local units: the base collar that sits
    // just inside the face.
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

        // Which one is on the low side of the shared plane.
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
        // The longjmp left IRIT's allocations unreachable. Leaking them beats
        // freeing a half-consumed boolean result.
        if (error != nullptr)
            *error = IritGuard::lastError();
        return false;
    }

    if (c.applied == 0) {
        // Every boolean declined: the tool never met the piece. Almost always
        // an open (non-watertight) piece, or a joint placed off its surface.
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
    // The neck: the narrowest section that still carries material.
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
