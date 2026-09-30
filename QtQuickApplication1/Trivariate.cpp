//
// Trivariate - implementation: domain and evaluation, tessellation, and the three
// sources (file, primitive, bounding cage), each run under IritGuard.
//

#include "Trivariate.h"
#include "IritGuard.h"
#include "IritMesh.h"

#include <QFile>
#include <QFileInfo>

extern "C" {
#include "inc_irit/triv_lib.h"
#include "inc_irit/cagd_lib.h"
#include "inc_irit/allocate.h"
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
