//
// IritJoint - implementation: builds the pin loft, places it on a face, and
// applies it to a piece as a boolean, all inside IritGuard.
//

#include "IritJoint.h"

#include "IritGuard.h"
#include "IritMesh.h"
#include "IritSolid.h"

#include <QDebug>

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
    // INWARD, not outward. IRIT decides inside from outside by winding, and the
    // sign is the opposite of the right-hand rule a divergence volume uses.
    // Every boolean in CageBoolean.cpp converts both operands Inward and works
    // (the cube regression sums to 100.0% of the model volume); this call site
    // was the only boolean in the codebase asking for Outward, so the piece and
    // the tool - which is built natively by IRIT, with IRIT's own convention -
    // did not even agree with each other.
    // Weld and orient BEFORE converting, exactly as CageBoolean does at its
    // lines 166 and 176. fromMesh picks the winding from signedVolume(m) < 0,
    // and that sign only means anything for a closed, consistently wound mesh.
    // A trimmed piece arrives with unwelded seams and mixed triangle
    // orientation, so for some pieces the sign comes out wrong and that piece
    // alone converts inverted - which is how a hole GREW piece 5 by 0.245 while
    // the other eleven booleans behaved. orientConsistently welds first.
    MeshData fixed = m;
    IritSolid::orientConsistently(&fixed);

    return IritSolid::fromMesh(fixed, IritSolid::Winding::Inward);
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

// ------------------------------------------------------------ dovetail ----
//
// A trapezoidal prism, built straight into world coordinates. No unit tool and
// no orientation matrix: a basis for six (normal, slide) combinations is easy
// to get subtly wrong and impossible to see afterwards.
//
// The cross-section lies in (normal, cross) - narrow where it crosses the
// contact plane, wide at full depth. That undercut is the lock: the socket
// cannot lift off along the normal. The section is constant along the slide
// axis, so the socket piece slides on from the edge of the face.
//
// The tail runs only part of the face (dtRunFrac) while the socket is cut the
// full length and overshoots both ends - otherwise there is no open end to
// slide in from and the joint could never be assembled.
//
// Winding does not have to be perfect here: meshToIrit welds and orients before
// converting, which is the fix that stopped booleans inverting.
MeshData dovetailSolid(const JointPlacement &j, const JointParams &p,
                       bool socket)
{
    MeshData m;

    const int n = j.axis;
    const int s = j.slide;
    if (s < 0 || s == n)
        return m;
    const int t = 3 - n - s;                   // the remaining axis

    // lo/hi are stored for (n+1)%3 first, then (n+2)%3.
    const int i1 = (n + 1) % 3;
    const int is = (s == i1) ? 0 : 1;
    const int it = (t == i1) ? 0 : 1;

    const double spanS = j.hi[is] - j.lo[is];
    const double spanT = j.hi[it] - j.lo[it];
    if (spanS <= 0.0 || spanT <= 0.0)
        return m;

    // Sized by the room the face actually has, not by its bounding box: j.room
    // is the half-width of a square solid on both pieces. Falling back to the
    // span keeps the old friction pin working.
    const double faceMin = j.room > 0.0 ? qMin(qMin(spanS, spanT), 2.0 * j.room)
                                        : qMin(spanS, spanT);
    const double clr     = socket ? p.clearanceXY : 1.0;

    // Never deeper than a third of the thinner piece, or the tooth punches
    // straight through and takes a chunk with it.
    double depth = p.dtDepth * faceMin * (socket ? p.clearanceZ : 1.0);
    if (j.wall > 0.0)
        depth = qMin(depth, 0.33 * j.wall);
    const double w0    = 0.5 * p.dtNarrow * faceMin * clr;
    const double w1    = 0.5 * p.dtWide   * faceMin * clr;
    const double back  = 0.08 * faceMin;       // root inside the tail's piece

    double half = 0.5 * p.dtRunFrac * spanS;
    if (j.room > 0.0)
        half = qMin(half, j.room);

    double s0, s1;
    if (socket) {
        // Slide out to the NEAREST edge only. Sweeping the full span cuts through
        // other parts of concave pieces (like U-shapes), severing them entirely.
        if (j.at[s] - j.lo[is] < j.hi[is] - j.at[s]) {
            s0 = j.lo[is] - 0.10 * spanS;
            s1 = j.at[s] + half;
        } else {
            s0 = j.at[s] - half;
            s1 = j.hi[is] + 0.10 * spanS;
        }
    }
    else {
        // Keep the tail INSIDE the material.
        //
        // This is what was deforming the model. Running the tail the full width
        // of the shared face pushes both of its ends out through the model's
        // surface, so the joint shows up as a lump on the silhouette. The tooth
        // was never the problem - its length was. j.room is the half-width of a
        // square that is solid on both pieces, so stay within it.
        s0 = j.at[s] - half;
        s1 = j.at[s] + half;
    }

    // The low-side piece carries the tail, so "into the socket" is +normal.
    const double n0 = j.at[n] - back;
    const double n1 = j.at[n] + depth;
    const double tc = j.at[t];

    const double cn[4] = { n0,      n0,      n1,      n1      };
    const double ct[4] = { tc - w0, tc + w0, tc + w1, tc - w1 };

    for (int e = 0; e < 2; ++e) {
        const double sv = (e == 0) ? s0 : s1;
        for (int k = 0; k < 4; ++k) {
            double v[3];
            v[n] = cn[k];
            v[t] = ct[k];
            v[s] = sv;
            m.addVertex(v[0], v[1], v[2]);
        }
    }

    // 0..3 at s0, 4..7 at s1.
    static const int quad[6][4] = {
        { 0, 1, 2, 3 },        // end at s0
        { 7, 6, 5, 4 },        // end at s1
        { 0, 4, 5, 1 },        // the n0 face, inside the tail's own piece
        { 3, 2, 6, 7 },        // the n1 face, deep in the socket
        { 1, 5, 6, 2 },        // +cross flank
        { 0, 3, 7, 4 }         // -cross flank
    };
    for (int f = 0; f < 6; ++f) {
        const int *q = quad[f];
        m.tris << uint32_t(q[0]) << uint32_t(q[1]) << uint32_t(q[2]);
        m.tris << uint32_t(q[0]) << uint32_t(q[2]) << uint32_t(q[3]);
    }

    m.finalize();
    return m;
}

struct Ctx {
    IritPrsrObjectStruct    *piece;
    const JointParams       *p;
    const JointPlacement    *places;
    int                      nPlaces;
    int                      applied;
    int                      declined;
    IritPrsrObjectStruct    *clip;        // the model, converted once
};

void doApply(void *v)
{
    Ctx *c = static_cast<Ctx *>(v);

    // A dovetail does not use the lofted pin, so a failure to build it is only
    // fatal for placements that actually need it.
    IritPrsrObjectStruct *rawTool = buildTool(*c -> p);
    IritPrsrObjectStruct *unitTool = NULL;

    if (rawTool != NULL) {
        MeshData toolMesh;
        QString err;
        if (IritMesh::tessellate(rawTool, &toolMesh, c->p->fineNess, &err)) {
            unitTool = meshToIrit(toolMesh);
        }
        IritPrsrFreeObject(rawTool);
    }

    for (int i = 0; i < c -> nPlaces; ++i) {
        const JointPlacement &j = c -> places[i];

        IritPrsrObjectStruct *tool = NULL;
        if (j.slide >= 0) {
            const MeshData d = dovetailSolid(j, *c -> p, !j.pin);
            if (!d.isEmpty())
                tool = meshToIrit(d);
        }
        else if (unitTool != NULL) {
            IrtHmgnMatType mat;
            placementMatrix(j, *c -> p, mat);
            tool = IritGeomTransformObject(unitTool, mat);
        }
        if (tool == NULL)
            continue;

        // Clip the TAIL to the model before unioning it on.
        //
        // Without this the union adds a lump wherever the tooth pokes past the
        // model's surface, which is what deformed the silhouette. Elber's
        // linear slice avoids the problem by making the CUT itself toothed, so
        // nothing is ever added; clipping to the model gets the same outcome on
        // this pipeline, and a tooth sitting over empty space intersects to
        // nothing and is quietly skipped instead of welding on a floating lump.
        //
        // The model, NOT the neighbouring piece. The tail's root reaches back
        // into its own piece on purpose, to give the union solid overlap to
        // bite on; clipping to the neighbour shears that root off and leaves an
        // operand exactly coplanar with the piece's own cut face, which is the
        // case these booleans get wrong. Clipping to the model keeps the root,
        // because the root is inside the piece and the piece is inside the
        // model.
        if (j.pin && j.slide >= 0 && c -> clip != NULL) {
            closeLists(tool);

            IritPrsrObjectStruct
                *clipped = IritBooleanAND(tool, c -> clip);

            IritPrsrFreeObject(tool);

            if (clipped == NULL) {
                ++c -> declined;   // no material under the tooth at all
                continue;
            }
            tool = clipped;
        }

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

    if (unitTool != NULL)
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

// A cut face as the trimmed mesh actually has it.
//
// The TRIANGLES lying on the contact plane, not their vertices. A flat cut face
// has vertices only around its boundary and along tessellation seams, so binning
// vertices tells you where the outline is and nothing about where the material
// is - and material coverage is the whole question here.
namespace {

const double kFaceEpsFrac = 1e-3;   // "on the plane", relative to piece size

struct FaceFootprint {
    double centre[2] = { 0, 0 };
    double lo[2]     = { 0, 0 };
    double hi[2]     = { 0, 0 };
    int    n         = 0;
    QVector<double> tu, tv;         // on-plane triangles, 3 entries each
};

bool faceFootprint(const MeshData &m, int axis, double plane, double eps,
                   FaceFootprint *f)
{
    const int u = (axis + 1) % 3, v = (axis + 2) % 3;
    double su = 0.0, sv = 0.0;
    double lo[2] = {  1e300,  1e300 };
    double hi[2] = { -1e300, -1e300 };
    int n = 0;

    f -> tu.clear();
    f -> tv.clear();

    const int nt = m.triangleCount();
    for (int t = 0; t < nt; ++t) {
        const uint32_t idx[3] = { m.tris[t * 3 + 0],
                                  m.tris[t * 3 + 1],
                                  m.tris[t * 3 + 2] };

        bool onPlane = true;
        for (int k = 0; k < 3 && onPlane; ++k)
            onPlane = std::fabs(double(m.pos[idx[k] * 3 + axis]) - plane) <= eps;
        if (!onPlane)
            continue;

        for (int k = 0; k < 3; ++k) {
            const double cu = double(m.pos[idx[k] * 3 + u]),
                         cv = double(m.pos[idx[k] * 3 + v]);
            f -> tu.append(cu);
            f -> tv.append(cv);
            su += cu;
            sv += cv;
            ++n;
            lo[0] = qMin(lo[0], cu);  hi[0] = qMax(hi[0], cu);
            lo[1] = qMin(lo[1], cv);  hi[1] = qMax(hi[1], cv);
        }
    }

    if (n < 3)
        return false;

    f -> n         = n;
    f -> centre[0] = su / n;
    f -> centre[1] = sv / n;
    f -> lo[0] = lo[0];  f -> hi[0] = hi[0];
    f -> lo[1] = lo[1];  f -> hi[1] = hi[1];
    return true;
}

// Mark every grid cell whose centre falls inside an on-plane triangle.
void rasterise(const FaceFootprint &f, double lo0, double lo1,
               double du, double dv, int G, QVector<quint8> *g)
{
    for (int t = 0; t + 2 < f.tu.size(); t += 3) {
        const double ax = f.tu[t],     ay = f.tv[t],
                     bx = f.tu[t + 1], by = f.tv[t + 1],
                     cx = f.tu[t + 2], cy = f.tv[t + 2];

        const double den = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy);
        if (std::fabs(den) < 1e-30)
            continue;                       // degenerate sliver

        int u0 = int(std::floor((qMin(ax, qMin(bx, cx)) - lo0) / du));
        int u1 = int(std::floor((qMax(ax, qMax(bx, cx)) - lo0) / du));
        int v0 = int(std::floor((qMin(ay, qMin(by, cy)) - lo1) / dv));
        int v1 = int(std::floor((qMax(ay, qMax(by, cy)) - lo1) / dv));
        u0 = qMax(0, u0);  u1 = qMin(G - 1, u1);
        v0 = qMax(0, v0);  v1 = qMin(G - 1, v1);

        for (int gv = v0; gv <= v1; ++gv) {
            for (int gu = u0; gu <= u1; ++gu) {
                const double px = lo0 + (gu + 0.5) * du,
                             py = lo1 + (gv + 0.5) * dv;
                const double l1 = ((by - cy) * (px - cx) + (cx - bx) * (py - cy)) / den,
                             l2 = ((cy - ay) * (px - cx) + (ax - cx) * (py - cy)) / den,
                             l3 = 1.0 - l1 - l2;
                if (l1 >= -1e-9 && l2 >= -1e-9 && l3 >= -1e-9)
                    (*g)[gv * G + gu] = 1;
            }
        }
    }
}

// The solid cell with the largest fully-solid square around it. That square is
// the room a tooth has; if the best is a single cell the face is too ragged to
// carry a joint at all.
bool bestSolidSpot(const QVector<quint8> &solid, int G,
                   int *outU, int *outV, int *outR)
{
    int bestU = -1, bestV = -1, bestR = -1;

    for (int gv = 0; gv < G; ++gv) {
        for (int gu = 0; gu < G; ++gu) {
            if (!solid[gv * G + gu])
                continue;

            int r = 0;
            for (;;) {
                const int nr = r + 1;
                if (gu - nr < 0 || gu + nr >= G || gv - nr < 0 || gv + nr >= G)
                    break;
                bool ok = true;
                for (int y = gv - nr; y <= gv + nr && ok; ++y)
                    for (int x = gu - nr; x <= gu + nr && ok; ++x)
                        ok = solid[y * G + x] != 0;
                if (!ok)
                    break;
                r = nr;
            }

            if (r > bestR) { bestR = r; bestU = gu; bestV = gv; }
        }
    }

    if (bestR < 0)
        return false;
    *outU = bestU;  *outV = bestV;  *outR = bestR;
    return true;
}

}

namespace {

// One tooth, two booleans, run inside the guard together so a Bool_lib error
// cannot leave one side cut and the other not.
struct CutCtx {
    IritPrsrObjectStruct *tail;
    IritPrsrObjectStruct *socket;
    const MeshData       *tooth;      // converted fresh for each boolean
    bool                  okTail;
    bool                  okSocket;
};

// How many polygons an object carries, or -1 if it is not a polygon object.
//
// This is the honest test for "did that boolean actually do anything". A
// pointer comparison is not: when the operands do not intersect, IRIT prints
// "failed to intersect, first object returned" and hands back a COPY - a new
// pointer with identical contents. Comparing volumes is not either; that was
// tried and measured nonsense, because the before is a raw cell box and the
// after is a re-tessellated boolean result. Polygon count is structural and
// survives both problems.
int polyCount(const IritPrsrObjectStruct *o)
{
    if (o == NULL || !IRIT_PRSR_IS_POLY_OBJ(o))
        return -1;

    int n = 0;
    for (const IritPrsrPolygonStruct *pl = o -> U.Pl; pl != NULL; pl = pl -> Pnext)
        ++n;
    return n;
}

void doCut(void *v)
{
    CutCtx *c = static_cast<CutCtx *>(v);

    // A FRESH tool per boolean. Handing the same object to both operations and
    // then freeing it drops the reference count twice - IRIT says "Free an
    // object with ref. count that is negative!" - because a boolean may take
    // over parts of its operands rather than copying them. doApply above builds
    // its tool per placement for exactly this reason.
    IritPrsrObjectStruct
        *t1 = meshToIrit(*c -> tooth);
    if (t1 != NULL) {
        closeLists(c -> tail);
        closeLists(t1);

        const int beforeTail = polyCount(c -> tail);

        IritPrsrObjectStruct
            *t = IritBooleanOR(c -> tail, t1);
        if (t != NULL && t != c -> tail && polyCount(t) != beforeTail) {
            closeLists(t);
            IritPrsrFreeObject(c -> tail);
            c -> tail   = t;
            c -> okTail = true;
        }

        // KNOWN: this run prints exactly two "Free an object with ref. count
        // that is negative!" per cut - one per boolean - and they are ours;
        // logs from before cutDovetail existed show none. Measured, so it is
        // not guesswork: leaving the superseded tail/socket unfreed changed the
        // count by zero, which rules those out. That leaves this free of the
        // TOOL as the suspect - the boolean appears to take the tool's lists
        // over rather than copy them. Not chased further because the geometry
        // is correct either way (both models cut and trim with 0 failures) and
        // IRIT raises this defensively rather than corrupting the heap.
        IritPrsrFreeObject(t1);
    }

    IritPrsrObjectStruct
        *t2 = meshToIrit(*c -> tooth);
    if (t2 != NULL) {
        closeLists(c -> socket);
        closeLists(t2);

        // The one that matters. On spot.obj this subtraction silently does
        // nothing for one contact - IRIT returns a copy of the socket - while
        // the union still gives the tail its tooth. The tooth then exists on
        // both sides, and the spare copy has nothing to attach to, so the trim
        // leaves it as an extra tooth-shaped piece. Requiring the polygon count
        // to change refuses that cut outright, and cutDovetail then writes back
        // neither side.
        const int beforeSocket = polyCount(c -> socket);

        IritPrsrObjectStruct
            *s = IritBooleanSUB(c -> socket, t2);
        if (s != NULL && s != c -> socket && polyCount(s) != beforeSocket) {
            closeLists(s);
            IritPrsrFreeObject(c -> socket);
            c -> socket   = s;
            c -> okSocket = true;
        }
        IritPrsrFreeObject(t2);   // same suspect as t1 above
    }
}

}

bool IritJoint::cutDovetail(MeshData *tail, MeshData *socket,
                            const JointPlacement &j, const JointParams &p,
                            QString *error)
{
    if (tail == nullptr || socket == nullptr)
        return false;

    // The SAME tooth for both sides - socket == false, so no clearance is
    // applied anywhere. Exact complements.
    const MeshData tooth = dovetailSolid(j, p, false);
    if (tooth.isEmpty()) {
        if (error != nullptr)
            *error = QStringLiteral("dovetail solid came out empty");
        return false;
    }

    CutCtx c;
    c.tail     = meshToIrit(*tail);
    c.socket   = meshToIrit(*socket);
    c.tooth    = &tooth;              // converted per boolean inside doCut
    c.okTail   = false;
    c.okSocket = false;

    if (c.tail == NULL || c.socket == NULL) {
        if (c.tail   != NULL) IritPrsrFreeObject(c.tail);
        if (c.socket != NULL) IritPrsrFreeObject(c.socket);
        if (error != nullptr)
            *error = QStringLiteral("could not convert a piece for the cut");
        return false;
    }

    // Coplanar handling ON for the cut, restored the moment it returns.
    //
    // This cut guarantees coplanar faces: the tooth's root sits exactly on the
    // contact plane, which is exactly the face of the piece it is unioned onto
    // and of the piece it is subtracted from. Without the flag IRIT says so out
    // loud - "Boolean: coplanar polygons detected. Enable COPLANAR state." on
    // armadillo - and the result is unreliable.
    //
    // Set and restored OUT HERE, not inside doCut: a fatal Bool_lib error
    // longjmps out of the guarded call, which would skip a restore placed
    // inside it and leave the flag set for every later boolean in the run.
    // CageBoolean brackets its own AND the same way.
    const int oldCoplanar = IritBoolSetHandleCoplanarPoly(TRUE);
    const bool ran = IritGuard::run(&c, doCut);
    IritBoolSetHandleCoplanarPoly(oldCoplanar);

    bool ok = ran && c.okTail && c.okSocket;

    if (ok) {
        MeshData outTail, outSocket;
        QString  e1, e2;
        ok = IritMesh::tessellate(c.tail,   &outTail,   p.fineNess, &e1) &&
             IritMesh::tessellate(c.socket, &outSocket, p.fineNess, &e2) &&
             !outTail.isEmpty() && !outSocket.isEmpty();

        if (ok) {
            IritSolid::orientConsistently(&outTail);
            IritSolid::orientConsistently(&outSocket);

            // Both sides must move, or neither is kept.
            //
            // A dovetail MOVES material: the tail gains exactly what the socket
            // loses. If the subtraction quietly did nothing, the tail still
            // gained the tooth and the tooth now exists twice - which is how an
            // extra, tooth-shaped piece appeared in spot.obj (5 pieces with
            // joints on, 4 with them off). Refusing the whole cut leaves both
            // pieces untouched and the face simply stays flat.
            const double vTailAfter   = std::fabs(IritSolid::signedVolume(outTail));
            const double vSocketAfter = std::fabs(IritSolid::signedVolume(outSocket));

            // NO volume test here, deliberately.
            //
            // One was tried: require the tail to gain and the socket to lose,
            // on the theory that IRIT returning a copy of the first object
            // ("failed to intersect") lets a tooth be added without being
            // removed. Measured, the test is unusable - on spot a tail went
            // +169% and its socket +200%, one socket +1222%; on cheburashka
            // BOTH sides shrank 45-75%. A tooth is a fraction of a percent, so
            // these numbers are not measuring the cut at all.
            //
            // The reason is that signedVolume only means anything for a closed,
            // consistently wound solid, and the two values are not comparable:
            // the "before" is the raw cell box, the "after" is a re-tessellated
            // boolean result. Any threshold over that is guesswork, and it
            // rejected every cut on three models with contradictory reasons.
            //
            // The duplicated-tooth artifact on spot.obj is therefore still
            // open, and still needs a mechanism that is actually observed
            // rather than inferred.
            *tail   = outTail;
            *socket = outSocket;
        }
        else if (error != nullptr) {
            *error = e1.isEmpty() ? (e2.isEmpty()
                                         ? QStringLiteral("cut produced no geometry")
                                         : e2)
                                  : e1;
        }
    }
    else if (error != nullptr) {
        *error = ran ? QStringLiteral("a dovetail boolean declined")
                     : IritGuard::lastError();
    }

    IritPrsrFreeObject(c.tail);
    IritPrsrFreeObject(c.socket);
    return ok;
}

// A dovetail's thinnest solid dimension is its narrow width at the contact
// plane, or its depth if that is smaller - both scale with the face.
double IritJoint::dovetailThinnest(const JointParams &p, double faceMin)
{
    return qMin(p.dtNarrow, p.dtDepth) * faceMin;
}

double IritJoint::sizeForFace(const JointParams &p, double faceMin, double depth,
                              bool dovetail)
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

    // Gate on the thinness of the joint that will actually be cut. Judging a
    // dovetail by the pin formula drops faces that a dovetail would have fitted
    // on, and passes ones it would not.
    if (p.minPinThickness > 0.0) {
        const double thin = dovetail ? dovetailThinnest(p, faceMin)
                                     : thinnestFeature(p, size);
        if (thin < p.minPinThickness)
            return 0.0;
    }

    return size > 1e-9 ? size : 0.0;
}

QVector<QVector<JointPlacement> > IritJoint::planPlacementsFor(
        const QVector<PuzzlePiece> &pieces,
        const Planner::Graph &g,
        const Planner::JointSet &keep,
        const JointParams &p,
        int *skipped,
        const QVector<int> *slideAxis)
{
    QVector<QVector<JointPlacement> > out(pieces.size());
    int dropped = 0;

    for (int ci = 0; ci < g.contactCount(); ++ci) {
        if (ci >= keep.size() || !keep[ci])
            continue;

        const Planner::Contact &c = g.contacts[ci];
        if (c.lowSide < 0 || c.highSide >= pieces.size())
            continue;

        // Place the joint from the TRIMMED meshes, not from the cell boxes.
        //
        // Everything upstream - adjacencyOfBoxes, the contact graph, c.ext and
        // c.depth - is computed on the original cell boxes, and CageBoolean
        // never writes p0/p1 back after trimming a piece to the model. On a
        // box-shaped model the two agree. On anything organic they do not: the
        // centre of the box face can be thin material or no material at all,
        // which is how a pin ends up standing proud on a curved surface with
        // nothing to mate into.
        const MeshData &MA = pieces[c.lowSide].mesh;
        const MeshData &MB = pieces[c.highSide].mesh;

        // The contact plane in WORLD coordinates, read off the meshes.
        //
        // c.plane cannot be used here. It is derived from p0/p1, and on the
        // V-rep path those are the cell box in the TRIVARIATE'S PARAMETER
        // DOMAIN: the bounding cage is IritTrivNSPrimBox -> IritCagdPrimPlaneSrf
        // -> IritCagdBilinearSrf extruded, all bilinear/Bezier, so the domain
        // is 0..1 while the geometry spans the model. Using a 0..1 coordinate
        // as a world position is what put every pin in one cluster near the
        // origin corner. The mesh path stores world boxes, so the two paths
        // disagree; the trimmed meshes are world space in BOTH cases.
        //
        // A ends where B begins, so the shared plane is between them.
        const double plane = 0.5 * (double(MA.bmax[c.axis]) +
                                    double(MB.bmin[c.axis]));

        FaceFootprint fa, fb;
        const double eps = kFaceEpsFrac *
            qMax(1e-9, double(qMax(MA.diagonal(), MB.diagonal())));

        if (!faceFootprint(MA, c.axis, plane, eps, &fa) ||
            !faceFootprint(MB, c.axis, plane, eps, &fb)) {
            ++dropped;                    // no real face left after trimming
            continue;
        }

        // The rectangle the two pieces genuinely share on that plane.
        const double lo0 = qMax(fa.lo[0], fb.lo[0]),
                     hi0 = qMin(fa.hi[0], fb.hi[0]),
                     lo1 = qMax(fa.lo[1], fb.lo[1]),
                     hi1 = qMin(fa.hi[1], fb.hi[1]);
        if (hi0 - lo0 <= 0.0 || hi1 - lo1 <= 0.0) {
            ++dropped;
            continue;
        }

        // Depth from the trimmed meshes only. c.depth is off limits for the
        // same reason as c.plane: on the V-rep path it is a parameter-domain
        // span (order 0.1), which would shrink every pin to nothing.
        const double depth =
            qMin(double(MA.bmax[c.axis] - MA.bmin[c.axis]),
                 double(MB.bmax[c.axis] - MB.bmin[c.axis]));

        const bool dove = (slideAxis != nullptr && ci < slideAxis->size() &&
                           slideAxis->at(ci) >= 0);

        // Where on this face is there material on BOTH sides?
        //
        // The centroid is not good enough. On a concave face - an arm, a leg -
        // it lands in a hole, and a tooth cut there joins nothing: it slices a
        // lump off instead, which is what "piece 7 left 5 disconnected parts"
        // was. So rasterise both faces and find the largest square that is
        // solid on both.
        const int G = 24;
        const double du = (hi0 - lo0) / G, dv = (hi1 - lo1) / G;
        if (du <= 0.0 || dv <= 0.0) {
            ++dropped;
            continue;
        }

        QVector<quint8> ga(G * G, 0), gb(G * G, 0);
        rasterise(fa, lo0, lo1, du, dv, G, &ga);
        rasterise(fb, lo0, lo1, du, dv, G, &gb);
        for (int q = 0; q < G * G; ++q)
            ga[q] = (ga[q] && gb[q]) ? 1 : 0;

        int bu = 0, bv = 0, br = 0;
        if (!bestSolidSpot(ga, G, &bu, &bv, &br) || br < 1) {
            ++dropped;                 // nowhere solid enough to hold a joint
            continue;
        }

        const double room    = br * qMin(du, dv);
        const double faceMin = qMin(qMin(hi0 - lo0, hi1 - lo1), 2.0 * room);

        const double size = sizeForFace(p, faceMin, depth, dove);
        if (size <= 0.0) {
            ++dropped;
            continue;
        }

        const int u = (c.axis + 1) % 3, v = (c.axis + 2) % 3;

        // The centre of the solid square, not the centroid of the outline.
        const double cu = lo0 + (bu + 0.5) * du,
                     cv = lo1 + (bv + 0.5) * dv;

        // A ROW of teeth along the cut, the way Elber's linear slice repeats
        // its trapeze - one tooth reads as a block stuck on the model, not as
        // a joint. They march along the CROSS axis (the in-plane axis that is
        // not the slide axis), inside the solid square so none of them reaches
        // the model's surface.
        //
        // Each tooth is its own placement and so its own boolean: several
        // disjoint shells in one operand is a case these booleans handle badly.
        const int sAx = dove ? slideAxis->at(ci) : -1;
        const int tAx = (sAx < 0) ? -1 : ((u == sAx) ? v : u);

        int teeth = 1;
        double toothFace = faceMin;
        if (dove && tAx >= 0 && p.dtTeeth > 1) {
            // Share the solid square between the teeth, and back off until each
            // one still clears the printability floor.
            for (int n = qMax(1, p.dtTeeth); n >= 1; --n) {
                const double share = qMin(faceMin, 2.0 * room / n);
                if (n == 1 || p.minPinThickness <= 0.0 ||
                    dovetailThinnest(p, share) >= p.minPinThickness) {
                    teeth     = n;
                    toothFace = share;
                    break;
                }
            }
        }

        const double pitch = (teeth > 1) ? (2.0 * room / teeth) : 0.0;

        for (int n = 0; n < teeth; ++n) {
            JointPlacement j;
            j.axis       = c.axis;
            j.size       = size;
            j.at[c.axis] = plane;
            j.at[u]      = cu;
            j.at[v]      = cv;

            // Spread along the cross axis, centred on the solid square.
            if (teeth > 1 && tAx >= 0) {
                const double off = (n - 0.5 * (teeth - 1)) * pitch;
                j.at[tAx] += off;
            }

            // Each tooth gets only its own share of the face, so the trapeze
            // is sized to the tooth and not to the whole contact.
            j.room = (teeth > 1) ? 0.5 * toothFace : room;
            j.wall = depth;

            j.lo[0] = lo0;  j.hi[0] = hi0;
            j.lo[1] = lo1;  j.hi[1] = hi1;

            if (dove)
                j.slide = sAx;

            j.thinnest = dove ? dovetailThinnest(p, toothFace)
                              : thinnestFeature(p, size);

            j.pin  = true;
            j.mate = c.highSide;
            out[c.lowSide].append(j);

            j.pin  = false;
            j.mate = c.lowSide;
            out[c.highSide].append(j);
        }
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
                      int *declined, const MeshData *clipTo)
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

    // Converted once for the whole piece, not per joint: it is the model, and
    // it is the largest operand in the room.
    IritPrsrObjectStruct *clipObj = NULL;
    if (clipTo != nullptr && !clipTo->isEmpty())
        clipObj = meshToIrit(*clipTo);

    Ctx c;
    c.piece    = piece;
    c.p        = &p;
    c.places   = places.constData();
    c.nPlaces  = places.size();
    c.applied  = 0;
    c.declined = 0;
    c.clip     = clipObj;

    const bool ran = IritGuard::run(&c, doApply);

    if (clipObj != NULL)
        IritPrsrFreeObject(clipObj);

    if (!ran) {
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

    // And orient the result, as CageBoolean does after its tessellate (line
    // 215). The piece goes back into m_pieces and is read again by the next
    // boolean, the volume check and the next division; handing any of them a
    // mesh with mixed orientation restarts the problem above.
    IritSolid::orientConsistently(&out);

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
