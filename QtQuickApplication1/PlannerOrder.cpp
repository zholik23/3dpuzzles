#include "PlannerOrder.h"

namespace Planner {

// A note on what the search does and does not settle.
//
// The loop is greedy and monotone: at each step it takes the first piece that
// has a free direction, and never reconsiders. Two consequences worth being
// straight about.
//
//   * Success is a genuine witness. The order it returns is a real removal
//     sequence under the model that was passed in - each step was checked
//     against the pieces still present at that moment.
//   * Failure is NOT a proof of non-assemblability in general. It says this
//     greedy descent got stuck; a different choice earlier might not have. It
//     also only covers MONOTONE sequences, where a piece goes straight out and
//     never moves again, and only ONE piece at a time - a puzzle needing two
//     sub-assemblies mated together is outside the model.
//
// So the log reports "no piece is removable" as a fact about this run, not as a
// theorem about the puzzle.
//
// Separately, and worth knowing before reading any result: for AXIS-ALIGNED
// boxes the translational model can never reach that failure. The piece with
// the largest hi[x] cannot have a neighbour on its +X face, because such a
// neighbour would need a larger hi[x] still. So at every step at least one
// piece is free, and the greedy loop always completes. A non-assemblable answer
// from stages 1-3 as they stand would mean a bug, not a hard puzzle. The
// failure branch is exercised by AlwaysBlocking, and will start doing real work
// when the swept-volume model replaces the placeholder.

Plan extract(const Graph &g, const BlockingModel &model)
{
    Plan plan;
    plan.modelName   = model.name();
    plan.modelCaveat = model.caveat();

    const int n = g.pieceCount();
    if (n == 0) {
        plan.problems << QStringLiteral("no pieces to plan");
        return plan;
    }

    QVector<bool> present(n, true);
    int remaining = n;

    while (remaining > 0) {
        int pick = -1, pickDir = -1;

        for (int i = 0; i < n && pick < 0; ++i) {
            if (!present[i])
                continue;
            const DirMask m = model.blocked(g, i, present);
            for (int d = 0; d < DirectionCount; ++d) {
                if (!isBlocked(m, d)) {
                    pick    = i;
                    pickDir = d;
                    break;
                }
            }
        }

        if (pick < 0) {
            for (int i = 0; i < n; ++i)
                if (present[i])
                    plan.stuck.append(i);
            plan.problems << QStringLiteral(
                "no piece is removable with %1 left: every remaining piece is "
                "blocked in all six directions").arg(remaining);
            return plan;
        }

        Step s;
        s.piece = pick;
        s.dir   = pickDir;
        plan.removal.append(s);
        present[pick] = false;
        --remaining;
    }

    // Assembly is the removal sequence played backwards, each piece going in
    // the way it came out.
    plan.assembly.reserve(plan.removal.size());
    for (int i = plan.removal.size() - 1; i >= 0; --i) {
        Step s;
        s.piece = plan.removal[i].piece;
        s.dir   = dirOpposite(plan.removal[i].dir);
        plan.assembly.append(s);
    }
    plan.complete = true;
    return plan;
}

int replay(const Graph &g, const BlockingModel &model, const Plan &plan,
           QString *why)
{
    QVector<bool> present(g.pieceCount(), true);

    for (int i = 0; i < plan.removal.size(); ++i) {
        const Step &s = plan.removal[i];
        if (s.piece < 0 || s.piece >= present.size())
            continue;

        const DirMask m = model.blocked(g, s.piece, present);
        if (isBlocked(m, s.dir)) {
            if (why != nullptr) {
                QStringList ids;
                for (int b : model.blockers(g, s.piece, s.dir, present))
                    ids << QString::number(b);
                *why = QStringLiteral("step %1: piece %2 cannot leave along %3 "
                                      "(blocked by %4); it is free only in %5")
                           .arg(i + 1).arg(s.piece)
                           .arg(QString::fromLatin1(dirName(s.dir)))
                           .arg(ids.isEmpty() ? QStringLiteral("?")
                                              : ids.join(QStringLiteral("/")))
                           .arg(maskToString((~m) & 0x3Fu));
            }
            return i;
        }
        present[s.piece] = false;
    }
    return -1;
}

QStringList Plan::describe(int maxSteps) const
{
    QStringList out;
    out << QStringLiteral("STAGE 3  assembly order under: %1").arg(modelName);

    if (!complete) {
        out << QStringLiteral("         NON-ASSEMBLABLE under this blocking model");
        for (const QString &p : problems)
            out << QStringLiteral("         %1").arg(p);
        QStringList ids;
        for (int i : stuck)
            ids << QString::number(i);
        out << QStringLiteral("         stuck pieces (%1): %2")
                   .arg(stuck.size()).arg(ids.join(QStringLiteral(" ")));
        out << QStringLiteral("         greedy, no backtracking - this is a "
                              "failure of this search, not a proof that no "
                              "order exists");
        return out;
    }

    out << QStringLiteral("         valid under translational DBG; real "
                          "collision check pending");
    out << QStringLiteral("         model caveat: %1").arg(modelCaveat);
    out << QStringLiteral("         %1 pieces ordered").arg(assembly.size());

    const int n = (maxSteps < 0) ? assembly.size()
                                 : qMin(maxSteps, int(assembly.size()));
    for (int i = 0; i < n; ++i)
        out << QStringLiteral("  %1. place piece %2, inserting along %3")
                   .arg(i + 1, 3)
                   .arg(assembly[i].piece, 3)
                   .arg(QString::fromLatin1(dirName(assembly[i].dir)));
    if (n < assembly.size())
        out << QStringLiteral("  ... and %1 more").arg(assembly.size() - n);
    return out;
}

} // namespace Planner
