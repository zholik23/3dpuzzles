//
// Trivariate - implementation: domain and evaluation, tessellation, and the three
// sources (file, primitive, bounding cage), each run under IritGuard.
//

#include "Trivariate.h"
#include "IritGuard.h"
#include "IritMesh.h"

#include <QFile>
#include <QFileInfo>

#include <algorithm>

extern "C" {
#include "inc_irit/irit_sm.h"
#include "inc_irit/misc_lib.h"
#include "inc_irit/iritprsr.h"
#include "inc_irit/cagd_lib.h"
#include "inc_irit/trim_lib.h"
#include "inc_irit/triv_lib.h"
#include "inc_irit/mvar_lib.h"
#include "inc_irit/allocate.h"
#include "inc_irit/attribut.h"
}

Trivariate::~Trivariate()
{
    if (m_tv != nullptr) {
        TrivTVStruct *tv = static_cast<TrivTVStruct *>(m_tv);
        IritTrivTVFree(tv);
        m_tv = nullptr;
    }
}

Trivariate::Trivariate(Trivariate &&other) noexcept
    : m_tv(other.m_tv), m_label(std::move(other.m_label))
{
    other.m_tv = nullptr;
}

Trivariate &Trivariate::operator=(Trivariate &&other) noexcept
{
    if (this != &other) {
        if (m_tv != nullptr) {
            TrivTVStruct *tv = static_cast<TrivTVStruct *>(m_tv);
            IritTrivTVFree(tv);
        }
        m_tv       = other.m_tv;
        m_label    = std::move(other.m_label);
        other.m_tv = nullptr;
    }
    return *this;
}

Trivariate Trivariate::adopt(void *tv, const QString &label)
{
    Trivariate t;
    t.m_tv    = tv;
    t.m_label = label;
    return t;
}

void Trivariate::domain(double d[6]) const
{
    d[0] = d[2] = d[4] = 0.0;
    d[1] = d[3] = d[5] = 1.0;
    if (m_tv == nullptr)
        return;
    IritTrivTVDomain(static_cast<const TrivTVStruct *>(m_tv),
                     &d[0], &d[1], &d[2], &d[3], &d[4], &d[5]);
}

void Trivariate::orders(int o[3]) const
{
    o[0] = o[1] = o[2] = 0;
    if (m_tv == nullptr)
        return;
    const TrivTVStruct *tv = static_cast<const TrivTVStruct *>(m_tv);
    o[0] = tv -> UOrder;
    o[1] = tv -> VOrder;
    o[2] = tv -> WOrder;
}

Trivariate Trivariate::clone() const
{
    if (m_tv == nullptr)
        return Trivariate();
    return adopt(IritTrivTVCopy(static_cast<const TrivTVStruct *>(m_tv)), m_label);
}

void Trivariate::lengths(int l[3]) const
{
    l[0] = l[1] = l[2] = 0;
    if (m_tv == nullptr)
        return;
    const TrivTVStruct *tv = static_cast<const TrivTVStruct *>(m_tv);
    l[0] = tv -> ULength;
    l[1] = tv -> VLength;
    l[2] = tv -> WLength;
}

bool Trivariate::evaluate(double u, double v, double w, double p[3]) const
{
    p[0] = p[1] = p[2] = 0.0;
    if (m_tv == nullptr)
        return false;

    const TrivTVStruct *tv = static_cast<const TrivTVStruct *>(m_tv);
    if (CAGD_NUM_OF_PT_COORD(tv -> PType) < 3)
        return false;

    CagdRType r[CAGD_MAX_PT_SIZE], *pr = r, e3[3];
    IritTrivTVEvalToData(tv, u, v, w, r);
    IritCagdCoerceToE3(e3, &pr, -1, tv -> PType);
    p[0] = e3[0]; p[1] = e3[1]; p[2] = e3[2];
    return true;
}

bool Trivariate::tessellate(MeshData *out, double fineNess, QString *error) const
{
    *out = MeshData();
    if (m_tv == nullptr) {
        if (error) *error = QStringLiteral("No trivariate.");
        return false;
    }

    TrivTVStruct *copy = IritTrivTVCopy(static_cast<const TrivTVStruct *>(m_tv));
    if (copy == NULL) {
        if (error) *error = QStringLiteral("Could not copy the trivariate.");
        return false;
    }

    IritPrsrObjectStruct *obj = IritPrsrGenTRIVARObject(copy);
    if (obj == NULL) {
        IritTrivTVFree(copy);
        if (error) *error = QStringLiteral("Could not wrap the trivariate.");
        return false;
    }

    out -> sourceKind = QStringLiteral("Trivar");
    return IritMesh::tessellate(obj, out, fineNess, error);
}

QStringList Trivariate::primitiveKinds()
{
    return { QStringLiteral("Sphere"),   QStringLiteral("Torus"),
             QStringLiteral("Cylinder"), QStringLiteral("Cone"),
             QStringLiteral("Box") };
}

namespace {

struct PrimCtx {
    int           kind;
    TrivTVStruct *result;
};

void doPrimitive(void *v)
{
    PrimCtx *c = static_cast<PrimCtx *>(v);
    CagdVType centre = { 0.0, 0.0, 0.0 };

    switch (c -> kind) {
        case 0: c -> result = IritTrivPrimSphere(centre, 1.0, FALSE);         break;
        case 1: c -> result = IritTrivPrimTorus(centre, 1.0, 0.35, FALSE);    break;
        case 2: c -> result = IritTrivPrimCylinder(centre, 0.7, 2.0, FALSE);  break;
        case 3: c -> result = IritTrivPrimCone(centre, 0.9, 2.0, FALSE);      break;
        default:
            c -> result = IritTrivNSPrimBox(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0);
            break;
    }
}

struct CageCtx {
    double        lo[3], hi[3];
    TrivTVStruct *result;
};

void doCage(void *v)
{
    CageCtx *c = static_cast<CageCtx *>(v);
    c -> result = IritTrivNSPrimBox(c -> lo[0], c -> lo[1], c -> lo[2],
                                    c -> hi[0], c -> hi[1], c -> hi[2]);
}

struct FileCtx {
    const char           *path;
    IritPrsrObjectStruct *objs;
};

void doRead(void *v)
{
    FileCtx *c = static_cast<FileCtx *>(v);
    c -> objs = IritPrsrGetObjects2(c -> path);
}

// Depth-limited search for the first trivariate carrying real (3D or more)
// geometry.
const TrivTVStruct *findGeometricTV(const IritPrsrObjectStruct *o,
                                    int *scalarsSeen,
                                    int  depth = 0)
{
    if (depth > 64)
        return NULL;

    for (; o != NULL; o = o -> Pnext) {
        if (IRIT_PRSR_IS_OLST_OBJ(o)) {
            for (int i = 0; ; ++i) {
                const IritPrsrObjectStruct *child =
                    IritPrsrListObjectGet(
                        const_cast<IritPrsrObjectStruct *>(o), i);
                if (child == NULL)
                    break;
                IritPrsrObjectStruct tmp = *child;
                tmp.Pnext = NULL;
                const TrivTVStruct *hit = findGeometricTV(&tmp, scalarsSeen,
                                                          depth + 1);
                if (hit != NULL)
                    return hit;
            }
            continue;
        }
        if (IRIT_PRSR_IS_TRIVAR_OBJ(o) && o -> U.Trivars != NULL) {
            for (const TrivTVStruct *tv = o -> U.Trivars; tv != NULL; tv = tv -> Pnext) {
                if (CAGD_NUM_OF_PT_COORD(tv -> PType) >= 3)
                    return tv;
                ++(*scalarsSeen);
            }
        }
    }
    return NULL;
}

}

Trivariate Trivariate::primitive(const QString &kind, QString *error)
{
    const int idx = primitiveKinds().indexOf(kind);
    PrimCtx c;
    c.kind   = idx < 0 ? 0 : idx;
    c.result = NULL;

    if (!IritGuard::run(&c, doPrimitive) || c.result == NULL) {
        if (error)
            *error = QStringLiteral("Could not build the %1 trivariate%2")
                         .arg(kind,
                              IritGuard::lastError().isEmpty()
                                  ? QStringLiteral(".")
                                  : QStringLiteral(" - ") + IritGuard::lastError());
        return Trivariate();
    }
    return adopt(c.result, kind + QStringLiteral(" (primitive trivariate)"));
}

// A trilinear box over the mesh. A zero-thickness axis would make the trivariate
// degenerate, so such an axis is given a sliver of the diagonal.
Trivariate Trivariate::boundingCage(const MeshData &mesh, QString *error)
{
    if (mesh.isEmpty()) {
        if (error) *error = QStringLiteral("No model loaded to build a cage around.");
        return Trivariate();
    }

    CageCtx c;
    for (int k = 0; k < 3; ++k) {
        c.lo[k] = mesh.bmin[k];
        c.hi[k] = mesh.bmax[k];
        if (c.hi[k] - c.lo[k] < 1e-9) {
            const double pad = 0.005 * double(mesh.diagonal());
            c.lo[k] -= pad;
            c.hi[k] += pad;
        }
    }
    c.result = NULL;

    if (!IritGuard::run(&c, doCage) || c.result == NULL) {
        if (error) *error = QStringLiteral("Could not build the bounding cage - %1")
                                .arg(IritGuard::lastError());
        return Trivariate();
    }
    return adopt(c.result, QStringLiteral("Bounding cage (Increment 3 placeholder)"));
}

Trivariate Trivariate::fromFile(const QString &path, QString *error)
{
    const QFileInfo fi(path);
    if (!fi.exists() || !fi.isFile()) {
        if (error) *error = QStringLiteral("File not found: %1").arg(path);
        return Trivariate();
    }

    const QByteArray path8 = QFile::encodeName(path);
    FileCtx fc;
    fc.path = path8.constData();
    fc.objs = NULL;

    if (!IritGuard::run(&fc, doRead)) {
        if (error) *error = QStringLiteral("Could not read %1 - %2")
                                .arg(fi.fileName(), IritGuard::lastError());
        return Trivariate();
    }
    if (fc.objs == NULL) {
        if (error) *error = QStringLiteral("%1 held no IRIT objects.").arg(fi.fileName());
        return Trivariate();
    }

    int scalars = 0;
    const TrivTVStruct *found = findGeometricTV(fc.objs, &scalars);

    TrivTVStruct *copy = (found != NULL) ? IritTrivTVCopy(found) : NULL;
    IritPrsrFreeObjectList(fc.objs);

    if (copy == NULL) {
        if (error) {
            *error = scalars > 0
                ? QStringLiteral("%1 holds %2 scalar (E1) trivariate(s) - volumetric "
                                 "functions with no geometry to divide. A puzzle needs "
                                 "a trivariate whose control points are 3D positions.")
                      .arg(fi.fileName()).arg(scalars)
                : QStringLiteral("%1 holds no trivariate. Load it as a model instead, "
                                 "or pick a primitive.").arg(fi.fileName());
        }
        return Trivariate();
    }

    return adopt(copy, fi.fileName());
}
bool Trivariate::saveToFile(const QString& path, QString* error) const
{
    if (!m_tv) {
        if (error) *error = QStringLiteral("Trivariate is invalid.");
        return false;
    }

    int handler = IritPrsrOpenDataFile(path.toUtf8().constData(), FALSE, FALSE);
    if (handler < 0) {
        if (error) *error = QStringLiteral("Could not open file for writing: %1").arg(path);
        return false;
    }

    IritPrsrObjectStruct* pObj = IritPrsrGenTRIVARObject(IritTrivTVCopy(static_cast<TrivTVStruct*>(m_tv)));
    if (pObj == NULL) {
        IritTrivTVFree(static_cast<TrivTVStruct*>(m_tv));
        if (error) *error = QStringLiteral("Could not wrap the trivariate.");
        return false;
    }

    IritPrsrPutObjectToHandler(handler, pObj);
    IritPrsrFreeObject(pObj);
    IritPrsrCloseStream(handler, TRUE);

    return true;
}

namespace {

struct SubRegionCtx {
    const TrivTVStruct *src;
    double              lo[3], hi[3];
    TrivTVStruct       *result;
};

// Inside IritGuard: IritTrivTVRegionFromTV can raise a fatal error.
void doSubRegion(void *v)
{
    SubRegionCtx *c = static_cast<SubRegionCtx *>(v);
    static const TrivTVDirType kDir[3] = { TRIV_CONST_U_DIR, TRIV_CONST_V_DIR, TRIV_CONST_W_DIR };
    c -> result = NULL;
    const TrivTVStruct *cur = c -> src;
    TrivTVStruct *owned = NULL;
    for (int a = 0; a < 3; ++a) {
        TrivTVStruct *next = IritTrivTVRegionFromTV(cur, c -> lo[a], c -> hi[a], kDir[a]);
        if (owned != NULL)
            IritTrivTVFree(owned);
        if (next == NULL)
            return;
        owned = next;
        cur   = next;
    }
    c -> result = owned;
}

}

// The exact sub-trivariate over a box of D: a V-rep piece, not a mesh.
Trivariate Trivariate::subRegion(double u0, double u1, double v0, double v1,
                                 double w0, double w1) const
{
    if (m_tv == nullptr || u1 <= u0 || v1 <= v0 || w1 <= w0)
        return Trivariate();
    SubRegionCtx c;
    c.src = static_cast<const TrivTVStruct *>(m_tv);
    c.lo[0] = u0; c.hi[0] = u1;
    c.lo[1] = v0; c.hi[1] = v1;
    c.lo[2] = w0; c.hi[2] = w1;
    c.result = NULL;
    if (!IritGuard::run(&c, doSubRegion) || c.result == NULL)
        return Trivariate();
    return adopt(c.result, m_label + QStringLiteral(" region"));
}

// Several trivariates in one .itd, each a named object (piece_0, piece_1, ...).
bool Trivariate::saveAll(const QVector<const Trivariate *> &tvs, const QStringList &names,
                         const QString &path, QString *error)
{
    int handler = IritPrsrOpenDataFile(path.toUtf8().constData(), FALSE, FALSE);
    if (handler < 0) {
        if (error) *error = QStringLiteral("Could not open file for writing: %1").arg(path);
        return false;
    }
    for (int i = 0; i < tvs.size(); ++i) {
        if (tvs[i] == nullptr || !tvs[i]->isValid())
            continue;
        IritPrsrObjectStruct *obj =
            IritPrsrGenTRIVARObject(IritTrivTVCopy(static_cast<TrivTVStruct *>(tvs[i]->raw())));
        if (obj == NULL)
            continue;
        const QByteArray name = (i < names.size() ? names[i] : QStringLiteral("tv_%1").arg(i)).toUtf8();
        IRIT_PRSR_SET_OBJ_NAME2(obj, name.constData());
        IritPrsrPutObjectToHandler(handler, obj);
        IritPrsrFreeObject(obj);
    }
    IritPrsrCloseStream(handler, TRUE);
    return true;
}

namespace {

// A surface in D as an IRIT B-spline surface (E3: u, v, w).
CagdSrfStruct *dSurfaceToSrf(const Trivariate::DSurface &d)
{
    if (d.n[0] < d.order[0] || d.n[1] < d.order[1] || d.knots[0].size() != d.n[0] + d.order[0] ||
        d.knots[1].size() != d.n[1] + d.order[1] || d.ctrl.size() != 3 * d.n[0] * d.n[1])
        return NULL;
    CagdSrfStruct *s = IritCagdBspSrfNew(d.n[0], d.n[1], d.order[0], d.order[1], CAGD_PT_E3_TYPE);
    if (s == NULL) return NULL;
    for (int k = 0; k < d.knots[0].size(); ++k) s -> UKnotVector[k] = d.knots[0][k];
    for (int k = 0; k < d.knots[1].size(); ++k) s -> VKnotVector[k] = d.knots[1][k];
    for (int m = 0; m < d.n[0] * d.n[1]; ++m)
        for (int a = 0; a < 3; ++a) s -> Points[a + 1][m] = d.ctrl[3 * m + a];
    return s;
}

struct ComposeCtx {
    const TrivTVStruct         *tv;
    const Trivariate::DSurface *d;
    IritPrsrObjectStruct       *dList;   // gets the surface(s) in D
    IritPrsrObjectStruct       *mList;   // gets M composed with them
};

// Inside IritGuard (IRIT's composition can raise a fatal error).
void doCompose(void *v)
{
    ComposeCtx *c = static_cast<ComposeCtx *>(v);
    static const TrivTVDirType kDir[3] = { TRIV_CONST_U_DIR, TRIV_CONST_V_DIR, TRIV_CONST_W_DIR };
    CagdSrfStruct *parts = dSurfaceToSrf(*c -> d);
    if (parts == NULL) return;
    // The closed v: split where v reaches 1 and bring the far part back by 1.
    if (c -> d -> vWrapAt > 0.0) {
        CagdRType s0, s1, t0, t1;
        IritCagdSrfDomain(parts, &s0, &s1, &t0, &t1);
        if (c -> d -> vWrapAt > s0 && c -> d -> vWrapAt < s1) {
            CagdSrfStruct *two = IritCagdSrfSubdivAtParam(parts, c -> d -> vWrapAt, CAGD_CONST_U_DIR);
            IritCagdSrfFree(parts);
            parts = two;
            if (parts != NULL && parts -> Pnext != NULL) {
                CagdSrfStruct *far = parts -> Pnext;
                for (int m = 0; m < far -> ULength * far -> VLength; ++m) far -> Points[2][m] -= 1.0;
            }
        }
    }
    CagdRType TD[6];
    IritTrivTVDomain(c -> tv, &TD[0], &TD[1], &TD[2], &TD[3], &TD[4], &TD[5]);
    for (CagdSrfStruct *s = parts; s != NULL; s = s -> Pnext) {
        const int np = s -> ULength * s -> VLength;
        double lo[3] = { 1e300, 1e300, 1e300 }, hi[3] = { -1e300, -1e300, -1e300 };
        for (int m = 0; m < np; ++m)
            for (int a = 0; a < 3; ++a) {
                // Round-off at the domain's faces (the wrap, w = 0 / 1) snapped in.
                CagdRType &x = s -> Points[a + 1][m];
                if (x < TD[2 * a] && x > TD[2 * a] - 1e-7) x = TD[2 * a];
                if (x > TD[2 * a + 1] && x < TD[2 * a + 1] + 1e-7) x = TD[2 * a + 1];
                lo[a] = std::min(lo[a], double(x));
                hi[a] = std::max(hi[a], double(x));
            }
        CagdSrfStruct *one = IritCagdSrfCopy(s);
        one -> Pnext = NULL;
        IritPrsrListObjectAppend(c -> dList, IritPrsrGenSRFObject(one));

        const int iso = c -> d -> iso;
        if (iso >= 0 && iso < 3) {
            // On an iso-plane of D: M(S) is M's own iso-surface over the
            // surface's range in the other two directions - exact, and plain.
            CagdSrfStruct *is = IritTrivSrfFromTV(c -> tv, 0.5 * (lo[iso] + hi[iso]), kDir[iso], FALSE);
            int k = 0;
            for (int a = 0; a < 3 && is != NULL; ++a) {
                if (a == iso) continue;
                const double r0 = std::max(lo[a], double(TD[2 * a])), r1 = std::min(hi[a], double(TD[2 * a + 1]));
                if (r1 > r0) {
                    CagdSrfStruct *r = IritCagdSrfRegionFromSrf(is, r0, r1, k == 0 ? CAGD_CONST_U_DIR : CAGD_CONST_V_DIR);
                    IritCagdSrfFree(is);
                    is = r;
                }
                ++k;
            }
            if (is != NULL) IritPrsrListObjectAppend(c -> mList, IritPrsrGenSRFObject(is));
            continue;
        }
        // Otherwise composed through the part of M it reaches (fewer knot lines
        // to cross); IRIT splits it where it crosses M's knot lines.
        double r0[3], r1[3];
        for (int a = 0; a < 3; ++a) {
            r0[a] = std::max(double(TD[2 * a]), lo[a] - 1e-6);
            r1[a] = std::min(double(TD[2 * a + 1]), hi[a] + 1e-6);
            if (r1[a] - r0[a] < 1e-4) {                 // flat in this direction: some room round it
                const double m = 0.5 * (r0[a] + r1[a]);
                r0[a] = std::max(double(TD[2 * a]), m - 1e-4);
                r1[a] = std::min(double(TD[2 * a + 1]), m + 1e-4);
            }
        }
        TrivTVStruct *sub = NULL;
        const TrivTVStruct *cur = c -> tv;
        for (int a = 0; a < 3; ++a) {
            TrivTVStruct *next = IritTrivTVRegionFromTV(cur, r0[a], r1[a], kDir[a]);
            if (sub != NULL) IritTrivTVFree(sub);
            sub = next;
            cur = next;
            if (next == NULL) break;
        }
        if (sub == NULL) continue;
        MvarMVStruct *mv = IritMvarCnvrtTVToMV(sub);
        MvarMVStruct *smv = IritMvarCnvrtSrfToMV(s);
        IritPrsrObjectStruct *res = (mv != NULL && smv != NULL) ? IritMvarMVCompose2(mv, smv, TRUE) : NULL;
        if (res != NULL) IritPrsrListObjectAppend(c -> mList, res);
        if (mv != NULL) IritMvarMVFree(mv);
        if (smv != NULL) IritMvarMVFree(smv);
        IritTrivTVFree(sub);
    }
    IritCagdSrfFreeList(parts);
}

}

bool Trivariate::saveTrimmed(const QVector<TrimmedPiece> &pieces, const QVector<ExactCut> &cuts,
                             const QString &path, QString *error, QStringList *notes)
{
    int handler = IritPrsrOpenDataFile(path.toUtf8().constData(), FALSE, FALSE);
    if (handler < 0) {
        if (error) *error = QStringLiteral("Could not open file for writing: %1").arg(path);
        return false;
    }
    // The cuts, once each: in D, and composed through their part's M.
    for (int k = 0; k < cuts.size(); ++k) {
        const ExactCut &ec = cuts[k];
        IritPrsrObjectStruct *list = IritPrsrGenLISTObject(NULL);
        IritPrsrObjectStruct *dList = IritPrsrGenLISTObject(NULL);
        IritPrsrObjectStruct *mList = IritPrsrGenLISTObject(NULL);
        int failed = 0;
        for (const DSurface &d : ec.surfaces) {
            if (ec.block == nullptr || !ec.block -> isValid()) { ++failed; continue; }
            ComposeCtx c;
            c.tv = static_cast<const TrivTVStruct *>(ec.block -> raw());
            c.d = &d;
            c.dList = dList;
            c.mList = mList;
            if (!IritGuard::run(&c, doCompose)) ++failed;
        }
        if (failed && notes)
            *notes << QStringLiteral("cut_%1 (%2): %3 of %4 surface(s) not composed through M (%5) - "
                                     "its surface in D is written")
                          .arg(k).arg(ec.name).arg(failed).arg(ec.surfaces.size()).arg(IritGuard::lastError());
        IRIT_PRSR_SET_OBJ_NAME2(dList, "d");
        IRIT_PRSR_SET_OBJ_NAME2(mList, "m");
        IritPrsrListObjectAppend(list, dList);
        IritPrsrListObjectAppend(list, mList);
        const QByteArray what = ec.name.toUtf8();
        IritMiscAttrSetObjectStrAttrib(list, "what", what.constData());
        const QByteArray name = QStringLiteral("cut_%1").arg(k).toUtf8();
        IRIT_PRSR_SET_OBJ_NAME2(list, name.constData());
        IritPrsrPutObjectToHandler(handler, list);
        IritPrsrFreeObject(list);
    }
    for (const TrimmedPiece &tp : pieces) {
        IritPrsrObjectStruct *list = IritPrsrGenLISTObject(NULL);
        for (const Trivariate *t : tp.cells) {
            if (t == nullptr || !t->isValid()) continue;
            IritPrsrObjectStruct *o = IritPrsrGenTRIVARObject(IritTrivTVCopy(static_cast<TrivTVStruct *>(t->raw())));
            if (o != NULL) {
                IRIT_PRSR_SET_OBJ_NAME2(o, "cell");
                IritPrsrListObjectAppend(list, o);
            }
        }
        IritPrsrObjectStruct *trim = IritPrsrGenLISTObject(NULL);
        for (int q = 0; q + 8 < tp.trim.size(); q += 9) {
            CagdSrfStruct *srf = IritCagdBzrSrfNew(2, 2, CAGD_PT_E3_TYPE);
            if (srf == NULL) continue;
            // (0,0) = p0, (1,0) = p1, (0,1) = p2, (1,1) = p2: the triangle p0 p1 p2.
            const int idx[4] = { 0, 3, 6, 6 };
            for (int m = 0; m < 4; ++m)
                for (int a = 0; a < 3; ++a) srf->Points[a + 1][m] = tp.trim[q + idx[m] + a];
            IritPrsrObjectStruct *o = IritPrsrGenSRFObject(srf);
            if (o != NULL) IritPrsrListObjectAppend(trim, o);
        }
        IRIT_PRSR_SET_OBJ_NAME2(trim, "trim");
        IritPrsrListObjectAppend(list, trim);
        if (!tp.cuts.isEmpty()) {
            QStringList refs;
            for (const auto &cr : tp.cuts)
                refs << QStringLiteral("%1:%2").arg(cr.first).arg(cr.second > 0 ? QStringLiteral("+1") : QStringLiteral("-1"));
            const QByteArray a = refs.join(QLatin1Char(' ')).toUtf8();
            IritMiscAttrSetObjectStrAttrib(list, "cuts", a.constData());
        }
        const QByteArray name = tp.name.toUtf8();
        IRIT_PRSR_SET_OBJ_NAME2(list, name.constData());
        IritPrsrPutObjectToHandler(handler, list);
        IritPrsrFreeObject(list);
    }
    IritPrsrCloseStream(handler, TRUE);
    return true;
}
