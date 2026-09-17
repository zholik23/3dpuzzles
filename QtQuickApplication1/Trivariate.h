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

    static Trivariate fromFile(const QString &path, QString *error);
    static Trivariate primitive(const QString &kind, QString *error);
    static Trivariate boundingCage(const MeshData &mesh, QString *error);

    static QStringList primitiveKinds();

    static Trivariate adopt(void *tv, const QString &label);

    bool    isValid() const { return m_tv != nullptr; }
    QString label()   const { return m_label; }

    void domain(double d[6]) const;

    void orders(int o[3]) const;

    bool tessellate(MeshData *out, double fineNess, QString *error) const;

    bool evaluate(double u, double v, double w, double p[3]) const;

    void *raw() const { return m_tv; }
    bool saveToFile(const QString& path, QString* error) const;

    Trivariate subRegion(double u0, double u1, double v0, double v1, double w0, double w1) const;
private:
    void   *m_tv = nullptr;
    QString m_label;
};
