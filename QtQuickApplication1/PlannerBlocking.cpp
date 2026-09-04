#include "PlannerBlocking.h"

namespace Planner {

BlockingModel::~BlockingModel() = default;

QString maskToString(DirMask m)
{
    QStringList on;
    for (int d = 0; d < DirectionCount; ++d)
        if (isBlocked(m, d))
            on << QString::fromLatin1(dirName(d));
    return on.isEmpty() ? QStringLiteral("none") : on.join(QStringLiteral(" "));
}

// ------------------------------------------------------- translational ------

QString TranslationalBlocking::name() const
{
    return QStringLiteral("translational DBG (placeholder)");
}

QString TranslationalBlocking::caveat() const
{
    return QStringLiteral("straight-line withdrawal only; no rotation, no swept "
                          "volume, no non-contacting obstacles - real collision "
                          "check pending");
}

DirMask TranslationalBlocking::blocked(const Graph &g, int piece,
                                       const QVector<bool> &present) const
{
    DirMask m = 0;
    if (piece < 0 || piece >= g.incident.size())
        return m;

    for (int ci : g.incident[piece]) {
        const int other = g.otherSide(ci, piece);
        if (other < 0 || other >= present.size() || !present[other])
            continue;                   // already removed: not in the way
        // A neighbour sitting on this face stops the piece leaving through it.
        m = withBlocked(m, g.directionFrom(ci, piece));
    }
    return m;
}

QVector<int> TranslationalBlocking::blockers(const Graph &g, int piece, int dir,
                                             const QVector<bool> &present) const
{
    QVector<int> out;
    if (piece < 0 || piece >= g.incident.size())
        return out;

    for (int ci : g.incident[piece]) {
        const int other = g.otherSide(ci, piece);
        if (other < 0 || other >= present.size() || !present[other])
            continue;
        if (g.directionFrom(ci, piece) == dir)
            out.append(other);
    }
    return out;
}

// ------------------------------------------------------------ test only -----

QString AlwaysBlocking::name() const
{
    return QStringLiteral("fully blocking (test model)");
}

QString AlwaysBlocking::caveat() const
{
    return QStringLiteral("not a geometric model - exists only to exercise the "
                          "non-assemblable branch");
}

DirMask AlwaysBlocking::blocked(const Graph &g, int piece,
                                const QVector<bool> &present) const
{
    for (int ci : g.incident.value(piece)) {
        const int other = g.otherSide(ci, piece);
        if (other >= 0 && other < present.size() && present[other])
            return 0x3Fu;               // every direction
    }
    return 0;                           // no neighbours left: free
}

QVector<int> AlwaysBlocking::blockers(const Graph &g, int piece, int dir,
                                      const QVector<bool> &present) const
{
    Q_UNUSED(dir);
    QVector<int> out;
    for (int ci : g.incident.value(piece)) {
        const int other = g.otherSide(ci, piece);
        if (other >= 0 && other < present.size() && present[other])
            out.append(other);
    }
    return out;
}

// ----------------------------------------------------------------- log ------

QStringList describeBlocking(const Graph &g, const BlockingModel &model,
                             int maxPieces)
{
    QStringList out;
    out << QStringLiteral("STAGE 2  %1").arg(model.name());
    out << QStringLiteral("         %1").arg(model.caveat());

    const QVector<bool> present(g.pieceCount(), true);
    const int n = (maxPieces < 0) ? g.pieceCount()
                                  : qMin(maxPieces, g.pieceCount());

    for (int i = 0; i < n; ++i) {
        const DirMask m = model.blocked(g, i, present);

        QStringList why;
        for (int d = 0; d < DirectionCount; ++d) {
            if (!isBlocked(m, d))
                continue;
            QStringList ids;
            for (int b : model.blockers(g, i, d, present))
                ids << QString::number(b);
            why << QStringLiteral("%1 by %2")
                       .arg(QString::fromLatin1(dirName(d)))
                       .arg(ids.join(QStringLiteral("/")));
        }

        out << QStringLiteral("  piece %1: blocked %2%3")
                   .arg(i, 3)
                   .arg(maskToString(m))
                   .arg(why.isEmpty() ? QString()
                                      : QStringLiteral("   [%1]")
                                            .arg(why.join(QStringLiteral(", "))));
    }
    if (n < g.pieceCount())
        out << QStringLiteral("  ... and %1 more").arg(g.pieceCount() - n);
    return out;
}

} // namespace Planner
