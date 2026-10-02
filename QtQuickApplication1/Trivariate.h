#pragma once
//
// Trivariate - owns one TrivTVStruct, the V-rep M that gets divided.
//
// Elber's construction divides a parameter domain and composes each sub-domain
// back through M, so a trivariate is what the divider needs: a mesh has no
// interior to cut into solid pieces.
//
// Three sources exist - fromFile, primitive (for testing the divider against
// known shapes), and boundingCage, a trilinear box carrying the model's extent
// but not its shape, which is the placeholder until the real fit lands. The IRIT
// type stays behind a void*, so callers need no IRIT headers.
//
#include "MeshData.h"

#include <QPair>
#include <QString>
#include <QStringList>
#include <QVector>

class Trivariate {
public:
    Trivariate() = default;
    ~Trivariate();

    Trivariate(Trivariate &&other) noexcept;
    Trivariate &operator=(Trivariate &&other) noexcept;
    Trivariate(const Trivariate &)            = delete;
    Trivariate &operator=(const Trivariate &) = delete;

    static Trivariate fromFile(const QString &path, QString *error);
    static Trivariate primitive(const QString &kind, QString *error);
    static Trivariate boundingCage(const MeshData &mesh, QString *error);

    static QStringList primitiveKinds();

    static Trivariate adopt(void *tv, const QString &label);

    // A deep copy (the IRIT trivariate is copied).
    Trivariate clone() const;

    bool    isValid() const { return m_tv != nullptr; }
    QString label()   const { return m_label; }

    void domain(double d[6]) const;

    void orders(int o[3]) const;

    // Control points per direction (0 if invalid).
    void lengths(int l[3]) const;

    bool tessellate(MeshData *out, double fineNess, QString *error) const;

    bool evaluate(double u, double v, double w, double p[3]) const;

    void *raw() const { return m_tv; }
    bool saveToFile(const QString& path, QString* error) const;

    Trivariate subRegion(double u0, double u1, double v0, double v1, double w0, double w1) const;

    // A surface in the parameter domain D = [0,1]^3 of a trivariate M: a
    // B-spline surface (open knots) whose control points are (u, v, w). M
    // composed with it is a surface in the model, exactly (no fitting).
    struct DSurface {
        int             order[2] = { 2, 2 };
        int             n[2] = { 2, 2 };       // control points per direction
        QVector<double> knots[2];              // n + order each
        QVector<double> ctrl;                  // (u, v, w) per point, index i + n[0] * j
        int             iso = -1;              // 0 / 1 / 2: it lies on u / v / w = const -
                                               // then M(S) is an iso-surface of M
        double          vWrapAt = -1.0;        // a parameter in direction 0 where v reaches 1:
                                               // split there, and v - 1 beyond (the closed v)
    };
    // A cut of a part, exact: its surface(s) in the D of its part's trivariate.
    struct ExactCut {
        QString            name;
        const Trivariate  *block = nullptr;
        QVector<DSurface>  surfaces;
    };
    // A trimmed V-rep piece: its trivariate cell(s), its trimming triangles
    // (x0 y0 z0 x1 y1 z1 x2 y2 z2 each; the model's skin and the joint caps)
    // and the exact cuts bounding it: (index into the cuts, its side: +1 = the
    // side dS/ds x dS/dt of the cut's D surface points to, -1 the other).
    struct TrimmedPiece {
        QString                     name;
        QVector<const Trivariate *> cells;
        QVector<double>             trim;
        QVector<QPair<int, int>>    cuts;
    };
    // Writes trimmed V-rep pieces into one .itd:
    //   - per cut a list "cut_k": a list "d" of its surface(s) in D and a list
    //     "m" of M composed with them (IRIT's exact composition; where a cut
    //     crosses M's knot lines the pieces are trimmed surfaces);
    //   - per piece a list "piece_N": its trivariate cell(s), a list "trim" of
    //     its trimming triangles (each a flat bilinear Bezier patch with two
    //     corners merged, which is the triangle exactly) and the attribute
    //     "cuts" = "k:+1 m:-1 ..." - the piece is the part of its cells on those
    //     sides of those cuts, inside its trimming surface.
    // `notes` gets a line per cut that could not be composed (its D surface is
    // still written).
    static bool saveTrimmed(const QVector<TrimmedPiece> &pieces, const QVector<ExactCut> &cuts,
                            const QString &path, QString *error, QStringList *notes = nullptr);

    // Writes several trivariates into one .itd file as named objects.
    static bool saveAll(const QVector<const Trivariate *> &tvs, const QStringList &names,
                        const QString &path, QString *error);
private:
    void   *m_tv = nullptr;
    QString m_label;
};
