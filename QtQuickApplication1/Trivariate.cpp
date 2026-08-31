#include "Trivariate.h"
#include "IritGuard.h"
#include "IritMesh.h"          // brings in the IRIT C headers

#include <QFile>
#include <QFileInfo>

extern "C" {
#include "inc_irit/triv_lib.h"
#include "inc_irit/cagd_lib.h"
}

// --------------------------------------------------------------- lifetime --

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

// ---------------------------------------------------------------- queries --

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

bool Trivariate::evaluate(double u, double v, double w, double p[3]) const
{
    p[0] = p[1] = p[2] = 0.0;
    if (m_tv == nullptr)
        return false;

    const TrivTVStruct *tv = static_cast<const TrivTVStruct *>(m_tv);
    if (CAGD_NUM_OF_PT_COORD(tv -> PType) < 3)
        return false;                       // scalar field: no position to read

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

    // IritPrsrGenTRIVARObject takes ownership of what it is handed, and
    // IritMesh::tessellate then consumes the object - so hand over a copy and
    // keep ours.
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

// ---------------------------------------------------------------- sources --

QStringList Trivariate::primitiveKinds()
{
    return { QStringLiteral("Sphere"),   QStringLiteral("Torus"),
             QStringLiteral("Cylinder"), QStringLiteral("Cone"),
             QStringLiteral("Box") };
}

namespace {

struct PrimCtx {
    int           kind;             // index into primitiveKinds()
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

// Depth-limited search for the first trivariate with real (>= 3D) geometry.
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

} // namespace

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
        // A zero-thickness axis (a flat plate, a planar sketch) would make a
        // degenerate trivariate, so give it a sliver of the diagonal.
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
