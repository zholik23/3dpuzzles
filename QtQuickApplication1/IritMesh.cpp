#include "IritMesh.h"
#include "IritGuard.h"

#include <QHash>
#include <QtGlobal>

// ------------------------------------------------------- guarded IRIT work --
//
// Conversion can trip an IRIT fatal error, so it runs inside IritGuard::run,
// which longjmps out on failure. Its callback therefore touches only POD.

struct ConvCtx {
    IritPrsrObjectStruct     *in;
    IritPrsrObjectStruct     *out;
    IPFreeformConvStateStruct state;
};

// Tessellates freeform geometry and triangulates every polygon over the whole
// forest. IritPrsrConvertFreeFormHierachy CONSUMES the object it is handed and
// returns a (possibly longer) chain, so the caller frees only `out`.
static void doConvert(void *v)
{
    ConvCtx *c = static_cast<ConvCtx *>(v);
    IritPrsrObjectStruct *out = NULL, *tail = NULL, *o = c -> in;

    while (o != NULL) {
        IritPrsrObjectStruct *next = o -> Pnext, *conv;

        o -> Pnext = NULL;
        // TriangleOnly is deliberately FALSE: it routes through
        // IritGeomConvertPolysToTriangles, which access-violates on some
        // valid inputs. Harvester::triangulate does the job instead.
        conv = IritPrsrConvertFreeFormHierachy(o, &c -> state,
                                               FALSE,     /* TriangleOnly */
                                               FALSE);    /* Regularize   */
        while (conv != NULL) {
            IritPrsrObjectStruct *cNext = conv -> Pnext;

            conv -> Pnext = NULL;
            if (tail != NULL) {
                tail -> Pnext = conv;
                tail = conv;
            }
            else {
                out = tail = conv;
            }
            conv = cNext;
        }
        o = next;
    }
    c -> out = out;
}

namespace {

struct VKey {
    float x, y, z;
    bool operator==(const VKey &o) const { return x == o.x && y == o.y && z == o.z; }
};

inline size_t qHash(const VKey &k, size_t seed = 0)
{
    return qHashMulti(seed, k.x, k.y, k.z);
}

// Walks the raw parse result and names what is in it. Runs BEFORE conversion,
// because conversion consumes the objects - and when nothing displayable comes
// out the far end, this inventory is the only thing that can tell the user why.
QStringList inventoryImpl(const IritPrsrObjectStruct *o, int depth = 0)
{
    QStringList out;
    if (depth > 64)
        return out;

    for (; o != NULL; o = o -> Pnext) {
        if (IRIT_PRSR_IS_OLST_OBJ(o)) {
            for (int i = 0; ; ++i) {
                const IritPrsrObjectStruct *child =
                    IritPrsrListObjectGet(
                        const_cast<IritPrsrObjectStruct *>(o), i);
                if (child == NULL)
                    break;
                // A list element still points at its siblings through Pnext,
                // so it is described on its own rather than walked as a chain.
                IritPrsrObjectStruct tmp = *child;
                tmp.Pnext = NULL;
                for (const QString &n : inventoryImpl(&tmp, depth + 1))
                    if (!out.contains(n))
                        out << n;
            }
            continue;
        }

        QString name = QString::fromLatin1(IritPrsrGetObjectTypeAsString(o));

        // A scalar (E1) trivariate is a volumetric *function*, not a solid: it
        // has no boundary geometry to tessellate. Worth calling out by name -
        // it is a shape this project cares about but cannot simply display.
        if (IRIT_PRSR_IS_TRIVAR_OBJ(o) && o -> U.Trivars != NULL) {
            const int nCoord = CAGD_NUM_OF_PT_COORD(o -> U.Trivars -> PType);
            if (nCoord < 3)
                name += QStringLiteral(" (scalar E%1 field, no boundary surface)")
                            .arg(nCoord);
        }
        if (!out.contains(name))
            out << name;
    }
    return out;
}

class Harvester {
public:
    explicit Harvester(MeshData *m) : m_mesh(m) {}

    void walk(const IritPrsrObjectStruct *o, int depth = 0)
    {
        if (depth > 64)                 // malformed file with a cyclic tree
            return;

        for (; o != NULL; o = o -> Pnext) {
            if (IRIT_PRSR_IS_OLST_OBJ(o)) {
                for (int i = 0; ; ++i) {
                    const IritPrsrObjectStruct *child =
                        IritPrsrListObjectGet(
                            const_cast<IritPrsrObjectStruct *>(o), i);
                    if (child == NULL)
                        break;
                    walkOne(child, depth + 1);
                }
                ++m_mesh -> objectCount;
            }
            else if (IRIT_PRSR_IS_POLY_OBJ(o)) {
                if (IRIT_PRSR_IS_POLYLINE_OBJ(o))
                    harvestPolylines(o -> U.Pl);
                else
                    harvestPolygons(o -> U.Pl);
                ++m_mesh -> objectCount;
            }
            else if (IRIT_PRSR_IS_FFGEOM_OBJ(o)) {
                // Freeform that survived conversion (an unsupported subtype).
                // Counted so the UI can say it was not displayed.
                ++m_mesh -> freeformCount;
                ++m_mesh -> objectCount;
            }
            // Numeric / matrix / string objects are ignored on purpose.
        }
    }

private:
    // A list element carries its own Pnext into the parent's list; following it
    // here would re-walk siblings, so children are visited one at a time.
    void walkOne(const IritPrsrObjectStruct *o, int depth)
    {
        if (o == NULL || depth > 64)
            return;
        if (IRIT_PRSR_IS_OLST_OBJ(o)) {
            for (int i = 0; ; ++i) {
                const IritPrsrObjectStruct *child =
                    IritPrsrListObjectGet(
                        const_cast<IritPrsrObjectStruct *>(o), i);
                if (child == NULL)
                    break;
                walkOne(child, depth + 1);
            }
            ++m_mesh -> objectCount;
        }
        else if (IRIT_PRSR_IS_POLY_OBJ(o)) {
            if (IRIT_PRSR_IS_POLYLINE_OBJ(o))
                harvestPolylines(o -> U.Pl);
            else
                harvestPolygons(o -> U.Pl);
            ++m_mesh -> objectCount;
        }
        else if (IRIT_PRSR_IS_FFGEOM_OBJ(o)) {
            ++m_mesh -> freeformCount;
            ++m_mesh -> objectCount;
        }
    }

    // Vertices are welded on exact coordinates. STL in particular repeats every
    // vertex once per facet, so welding is what turns a triangle soup into a
    // mesh with a meaningful edge list - and later, a cell-adjacency graph.
    uint32_t vertexFor(const IrtPtType p)
    {
        // -0.0f and 0.0f compare equal but hash differently, which would put
        // two "equal" keys in the hash and silently unweld a seam. Normalise.
        const auto z = [](double v) { return float(v) + 0.0f; };
        const VKey k { z(p[0]), z(p[1]), z(p[2]) };
        auto it = m_index.constFind(k);
        if (it != m_index.constEnd())
            return *it;
        const uint32_t idx = m_mesh -> addVertex(p[0], p[1], p[2]);
        m_index.insert(k, idx);
        return idx;
    }

    void harvestPolygons(const IritPrsrPolygonStruct *pl)
    {
        for (; pl != NULL; pl = pl -> Pnext) {
            m_idx.clear();
            for (const IritPrsrVertexStruct *v = pl -> PVertex; v != NULL; v = v -> Pnext) {
                m_idx.push_back(vertexFor(v -> Coord));
                if (m_idx.size() > 4096)            // runaway list
                    break;
                if (v -> Pnext == pl -> PVertex)    // IRIT may close the loop
                    break;
            }
            if (m_idx.size() < 3)
                continue;
            ++m_mesh -> polygonCount;
            triangulate(m_idx);
        }
    }

    void emitTri(uint32_t a, uint32_t b, uint32_t c)
    {
        if (a == b || b == c || a == c)             // degenerate after welding
            return;
        m_mesh -> tris.push_back(a);
        m_mesh -> tris.push_back(b);
        m_mesh -> tris.push_back(c);
    }

    // Ear clipping, done here rather than by IRIT.
    //
    // ConvertFreeFormHierachy's TriangleOnly flag routes through
    // IritGeomConvertPolysToTriangles -> IritGeomConvexPolyObjectN, which
    // access-violates on some perfectly valid inputs (data/pl_cncyl.itd and
    // data/pl_sold3.itd both kill the process). So polygons arrive here
    // untriangulated. Ear clipping is also correct for non-convex faces,
    // which a naive triangle fan is not.
    void triangulate(const QVector<uint32_t> &loop)
    {
        const int n = loop.size();
        if (n < 3)
            return;
        if (n == 3) {
            emitTri(loop[0], loop[1], loop[2]);
            return;
        }

        // Newell normal, then drop the dominant axis to get a 2D projection
        // that cannot collapse the polygon to a line.
        double nx = 0.0, ny = 0.0, nz = 0.0;
        for (int i = 0; i < n; ++i) {
            const float *a = &m_mesh -> pos[loop[i] * 3];
            const float *b = &m_mesh -> pos[loop[(i + 1) % n] * 3];
            nx += double(a[1] - b[1]) * double(a[2] + b[2]);
            ny += double(a[2] - b[2]) * double(a[0] + b[0]);
            nz += double(a[0] - b[0]) * double(a[1] + b[1]);
        }
        const double ax = qAbs(nx), ay = qAbs(ny), az = qAbs(nz);
        int u = 0, v = 1;
        if (ax >= ay && ax >= az)       { u = 1; v = 2; }
        else if (ay >= ax && ay >= az)  { u = 2; v = 0; }

        m_px.resize(n);
        m_py.resize(n);
        for (int i = 0; i < n; ++i) {
            const float *p = &m_mesh -> pos[loop[i] * 3];
            m_px[i] = double(p[u]);
            m_py[i] = double(p[v]);
        }

        // Work on a mutable ring of positions into `loop`, wound CCW in 2D.
        double area2 = 0.0;
        for (int i = 0, j = n - 1; i < n; j = i++)
            area2 += m_px[j] * m_py[i] - m_px[i] * m_py[j];

        m_ring.resize(n);
        for (int i = 0; i < n; ++i)
            m_ring[i] = (area2 >= 0.0) ? i : (n - 1 - i);

        int remaining = n;
        int guard     = 2 * n;              // no ear found in a full pass

        while (remaining > 3 && guard-- > 0) {
            bool clipped = false;

            for (int i = 0; i < remaining; ++i) {
                const int ia = m_ring[(i + remaining - 1) % remaining];
                const int ib = m_ring[i];
                const int ic = m_ring[(i + 1) % remaining];

                if (!isEar(ia, ib, ic, remaining))
                    continue;

                emitTri(loop[ia], loop[ib], loop[ic]);
                m_ring.remove(i);
                --remaining;
                clipped = true;
                guard = 2 * remaining;
                break;
            }

            if (!clipped)
                break;      // self-intersecting or degenerate: fan the rest
        }

        for (int i = 1; i + 1 < remaining; ++i)
            emitTri(loop[m_ring[0]], loop[m_ring[i]], loop[m_ring[i + 1]]);
    }

    double cross2(int a, int b, int c) const
    {
        return (m_px[b] - m_px[a]) * (m_py[c] - m_py[a])
             - (m_py[b] - m_py[a]) * (m_px[c] - m_px[a]);
    }

    bool isEar(int ia, int ib, int ic, int remaining) const
    {
        const double area = cross2(ia, ib, ic);
        if (area <= 0.0)
            return false;                           // reflex or collinear

        for (int k = 0; k < remaining; ++k) {
            const int ip = m_ring[k];
            if (ip == ia || ip == ib || ip == ic)
                continue;
            // Inside test via the three edge signs of the candidate ear.
            const double d0 = (m_px[ib] - m_px[ia]) * (m_py[ip] - m_py[ia])
                            - (m_py[ib] - m_py[ia]) * (m_px[ip] - m_px[ia]);
            const double d1 = (m_px[ic] - m_px[ib]) * (m_py[ip] - m_py[ib])
                            - (m_py[ic] - m_py[ib]) * (m_px[ip] - m_px[ib]);
            const double d2 = (m_px[ia] - m_px[ic]) * (m_py[ip] - m_py[ic])
                            - (m_py[ia] - m_py[ic]) * (m_px[ip] - m_px[ic]);
            if (d0 >= 0.0 && d1 >= 0.0 && d2 >= 0.0)
                return false;
        }
        return true;
    }

    void harvestPolylines(const IritPrsrPolygonStruct *pl)
    {
        for (; pl != NULL; pl = pl -> Pnext) {
            uint32_t prev = 0;
            bool     have = false;
            int      n    = 0;
            for (const IritPrsrVertexStruct *v = pl -> PVertex; v != NULL; v = v -> Pnext) {
                const uint32_t cur = vertexFor(v -> Coord);
                if (have && cur != prev) {
                    m_mesh -> polylineEdges.push_back(prev);
                    m_mesh -> polylineEdges.push_back(cur);
                }
                prev = cur;
                have = true;
                if (++n > 65536)
                    break;
                if (v -> Pnext == pl -> PVertex)
                    break;
            }
        }
    }

    MeshData            *m_mesh;
    QHash<VKey, quint32>  m_index;
    QVector<uint32_t>     m_idx;      // one polygon's welded vertex indices
    QVector<double>       m_px, m_py; // that polygon projected to 2D
    QVector<int>          m_ring;     // ear-clipping working ring
};

} // namespace
// --------------------------------------------------------------- interface --

bool IritMesh::tessellate(IritPrsrObjectStruct *objs,
                          MeshData             *out,
                          double                fineNess,
                          QString              *error)
{
    if (objs == NULL)
        return true;                            // nothing to do, not a failure

    ConvCtx cc;
    cc.in    = objs;
    cc.out   = NULL;
    cc.state = IritPrsrFFCState;             // IRIT's defaults, then our tweaks
    cc.state.Talkative           = FALSE;
    cc.state.DumpObjsAsPolylines = FALSE;    // we want polygons, not isolines
    cc.state.DrawFFGeom          = TRUE;
    cc.state.DrawFFMesh          = FALSE;
    cc.state.ComputeNrml         = TRUE;
    cc.state.ComputeUV           = FALSE;
    cc.state.FineNess            = fineNess;

    if (!IritGuard::run(&cc, doConvert)) {
        // doConvert already took ownership of whatever it consumed, and the
        // partial chain is unreachable after the longjmp. Leaking a failed
        // parse beats freeing a half-consumed forest.
        if (error != nullptr)
            *error = IritGuard::lastError();
        return false;
    }

    {
        Harvester h(out);
        h.walk(cc.out);
    }

    IritPrsrFreeObjectList(cc.out);
    out -> finalize();
    return true;
}

QStringList IritMesh::inventory(const IritPrsrObjectStruct *objs)
{
    return inventoryImpl(objs);
}
