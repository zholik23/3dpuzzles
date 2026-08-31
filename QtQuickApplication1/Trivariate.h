#pragma once
//
// Trivariate - owns one TrivTVStruct, the V-rep M that gets divided.
//
// Elber's construction divides the *parameter domain* of a trivariate and
// composes each sub-domain back through M. So a trivariate, not a mesh, is
// what the divider needs: a mesh has no interior to cut into solid pieces.
//
// Three sources exist today:
//   fromFile      - an E3/P3 trivariate stored in a native .itd
//   primitive     - sphere / torus / cylinder / cone / box, for developing
//                   and testing the divider against known shapes
//   boundingCage  - a trilinear box over a loaded mesh. This is the placeholder
//                   for Increment 3: it has the model's extent but not its
//                   shape, so dividing it shows the cell structure only. The
//                   real fit (IritTrivFitTV2PolyMesh, which needs a medial axis
//                   and a tube-topology mesh) replaces it.
//
// The IRIT type stays behind a void* so callers need no IRIT headers.
//
#include "MeshData.h"

#include <QString>
#include <QStringList>

class Trivariate {
public:
    Trivariate() = default;
    ~Trivariate();

    Trivariate(Trivariate &&other) noexcept;
    Trivariate &operator=(Trivariate &&other) noexcept;
    Trivariate(const Trivariate &)            = delete;
    Trivariate &operator=(const Trivariate &) = delete;

    // --- sources ---------------------------------------------------------
    static Trivariate fromFile(const QString &path, QString *error);
    static Trivariate primitive(const QString &kind, QString *error);
    static Trivariate boundingCage(const MeshData &mesh, QString *error);

    static QStringList primitiveKinds();

    // Wraps a raw TrivTVStruct*, taking ownership. Used by PuzzleDivider for
    // the sub-trivariates it extracts.
    static Trivariate adopt(void *tv, const QString &label);

    // --- queries ---------------------------------------------------------
    bool    isValid() const { return m_tv != nullptr; }
    QString label()   const { return m_label; }

    // Parameter domain as { uMin, uMax, vMin, vMax, wMin, wMax }.
    void domain(double d[6]) const;

    // Orders (degree + 1) along u, v, w - a piece cannot be finer than these.
    void orders(int o[3]) const;

    // Tessellates the boundary into `out`. `fineNess` follows IRIT's
    // convention (20 is its default; higher means more polygons).
    bool tessellate(MeshData *out, double fineNess, QString *error) const;

    // Physical position of one parameter point. Returns false if the trivariate
    // is scalar (E1), which has no position to evaluate.
    bool evaluate(double u, double v, double w, double p[3]) const;

    void *raw() const { return m_tv; }              // TrivTVStruct *

private:
    void   *m_tv = nullptr;
    QString m_label;
};
