//
// PuzzleAnalyzer - implementation.
//
#include "PuzzleAnalyzer.h"

#include <QCollator>
#include <QDir>
#include <QElapsedTimer>
#include <QFileInfo>
#include <QVariantMap>
#include <QtConcurrent/QtConcurrent>

#include <algorithm>
#include <cmath>

namespace {

// Piece colours with a meaning. Everything else keeps its palette colour.
constexpr quint32 kMoving = 0xffe8842au;   // the group being slid out
constexpr quint32 kFolded = 0xffd9534fu;   // det J < 0 inside the piece
constexpr quint32 kThin   = 0xffe6c84au;   // thinner than the limit

// How far a pulled piece travels, in model sizes, at full slider.
constexpr double kPullReach = 1.0;

QString groupLabel(const QVector<int> &g)
{
    QStringList s;
    for (int p : g) s << QString::number(p);
    return QStringLiteral("{%1}").arg(s.join(QStringLiteral(", ")));
}

QString dirLabel(const double d[3])
{
    return QStringLiteral("(%1, %2, %3)")
        .arg(d[0], 0, 'f', 2).arg(d[1], 0, 'f', 2).arg(d[2], 0, 'f', 2);
}

// A clearance for display: how far the pull may tilt. Under 1 deg the piece
// is free only within manufacturing tolerance, so it is flagged.
QString clearLabel(double deg)
{
    if (deg < -1.5)
        return QStringLiteral("free (sampled)");     // too many pieces for the exact test
    if (deg < 0.0)
        return QStringLiteral("blocked");
    return deg < 1.0 ? QStringLiteral("%1° (marginal)").arg(deg, 0, 'f', 2)
                     : QStringLiteral("%1°").arg(deg, 0, 'f', 1);
}

QVariantMap row(std::initializer_list<std::pair<const char *, QVariant>> kv)
{
    QVariantMap m;
    for (const auto &e : kv) m.insert(QString::fromLatin1(e.first), e.second);
    return m;
}

// det J this far below zero, relative to the piece's typical det J, is a fold.
// Smaller values are the round-off of a degenerate edge (the poles of a
// surface of revolution), where det J is exactly zero in theory.
bool folds(const DbgPieceQuality &pq, double cellParamVolume)
{
    const double typical = (cellParamVolume > 0.0) ? pq.volume / cellParamVolume : 0.0;
    return pq.minDetJ < -1e-6 * std::max(typical, 1e-300);
}

}  // namespace

PuzzleAnalyzer::PuzzleAnalyzer(QObject *parent)
    : QObject(parent)
{
    connect(&m_watcher, &QFutureWatcher<Result>::finished, this, &PuzzleAnalyzer::finishRun);
}

PuzzleAnalyzer::~PuzzleAnalyzer()
{
    // The worker reads m_tv; it must be done before m_tv goes away.
    m_watcher.waitForFinished();
}

void PuzzleAnalyzer::setStatus(const QString &s)
{
    m_status = s;
    emit statusChanged();
}

void PuzzleAnalyzer::setBusy(bool b)
{
    if (m_busy == b) return;
    m_busy = b;
    emit busyChanged();
}

// ---------------------------------------------------------------------------
// Loading
// ---------------------------------------------------------------------------
void PuzzleAnalyzer::open(const QUrl &url)
{
    openPath(url.isLocalFile() ? url.toLocalFile() : url.toString());
}

void PuzzleAnalyzer::openSibling(const QString &name)
{
    if (!m_folder.isEmpty() && name != m_fileName)
        openPath(QDir(m_folder).filePath(name));
}

void PuzzleAnalyzer::openPath(const QString &path)
{
    if (m_busy) return;

    QString err;
    Trivariate tv = Trivariate::fromFile(path, &err);
    if (!tv.isValid()) {
        setStatus(QStringLiteral("Could not open %1: %2")
                      .arg(QFileInfo(path).fileName(), err));
        return;
    }

    m_tv       = std::move(tv);
    m_path     = path;
    m_fileName = QFileInfo(path).fileName();
    m_folder   = QFileInfo(path).absolutePath();

    // The other trivariates in the same folder, so the R sweep can be stepped
    // through without the file dialog.
    QStringList names = QDir(m_folder).entryList(
        { QStringLiteral("tvs*_*.itd"), QStringLiteral("tv_*.itd") }, QDir::Files);
    if (!names.contains(m_fileName)) names << m_fileName;
    QCollator coll;
    coll.setNumericMode(true);
    std::sort(names.begin(), names.end(),
              [&](const QString &a, const QString &b) { return coll.compare(a, b) < 0; });
    m_siblings = names;

    m_pieces.clear();
    m_hasResult = false;
    m_selected  = -1;
    m_pull      = 0.0;
    m_playhead  = 0.0;
    m_sweepText.clear();
    emit fileChanged();
    emit resultChanged();
    emit sweepChanged();
    emit piecesChanged();
    setStatus(QStringLiteral("%1 loaded - choose a division and press Analyse").arg(m_fileName));
}

// ---------------------------------------------------------------------------
// Analysis
// ---------------------------------------------------------------------------
void PuzzleAnalyzer::run(int nu, int nv, int nw, int directions)
{
    if (m_busy) return;
    if (!m_tv.isValid()) {
        setStatus(QStringLiteral("Open a trivariate first (a tvs_*.itd file)."));
        return;
    }

    m_opt = DbgOptions();
    m_opt.cells[0] = qBound(1, nu, 8);
    m_opt.cells[1] = qBound(1, nv, 8);
    m_opt.cells[2] = qBound(1, nw, 8);
    m_opt.directions = qBound(500, directions, 20000);
    const int count = m_opt.cells[0] * m_opt.cells[1] * m_opt.cells[2];
    if (count > 32) {
        // Groups are bitmasks of 32 pieces, and past about a dozen pieces the
        // group search is capped anyway.
        setStatus(QStringLiteral("%1 pieces is too many for this analysis - keep it at 32 or "
                                 "fewer (a full proof needs 18 or fewer).").arg(count));
        return;
    }

    // The pieces to look at: the same cells the DBG will analyse, extracted
    // by the app's own divider. On this thread, because it runs IRIT booleans
    // under IritGuard.
    double dom[6];
    m_tv.domain(dom);
    DivisionSpec spec;
    for (int a = 0; a < 3; ++a)
        for (int c = 1; c < m_opt.cells[a]; ++c)
            spec.splits[a].append(dom[a * 2] + (dom[a * 2 + 1] - dom[a * 2]) * c / m_opt.cells[a]);

    QVector<PuzzlePiece> cut;
    QString err;
    if (!PuzzleDivider::divide(m_tv, spec, 12.0, &cut, &err)) {
        setStatus(QStringLiteral("Could not cut the pieces: %1").arg(err));
        return;
    }
    m_pieces = QVector<PuzzlePiece>(count);
    for (const PuzzlePiece &p : cut) {
        const int id = pieceId(p);
        if (id >= 0 && id < count) m_pieces[id] = p;
    }
    for (int k = 0; k < 3; ++k) { m_lo[k] = 1e30f; m_hi[k] = -1e30f; }
    for (const PuzzlePiece &p : m_pieces)
        if (!p.mesh.isEmpty())
            for (int k = 0; k < 3; ++k) {
                m_lo[k] = std::min(m_lo[k], p.mesh.bmin[k]);
                m_hi[k] = std::max(m_hi[k], p.mesh.bmax[k]);
            }

    m_hasResult = false;
    m_selected  = -1;
    m_pull      = 0.0;
    m_playhead  = 0.0;
    m_sweepText.clear();
    emit resultChanged();
    emit sweepChanged();
    emit piecesChanged();

    setBusy(true);
    setStatus(QStringLiteral("Analysing %1 as %2x%3x%4 = %5 pieces ...")
                  .arg(m_fileName).arg(m_opt.cells[0]).arg(m_opt.cells[1])
                  .arg(m_opt.cells[2]).arg(count));

    const Trivariate *tv = &m_tv;
    const DbgOptions opt = m_opt;
    m_watcher.setFuture(QtConcurrent::run([tv, opt]() {
        Result r;
        r.dbg     = DbgAnalysis::run(*tv, opt);
        r.quality = DbgAnalysis::quality(*tv, opt);
        return r;
    }));
}

void PuzzleAnalyzer::finishRun()
{
    m_res = m_watcher.result();
    setBusy(false);
    if (!m_res.dbg.valid) {
        setStatus(QStringLiteral("Analysis failed: %1").arg(m_res.dbg.error));
        return;
    }
    m_hasResult = true;
    buildTexts();
    buildCriteria();

    // Start on the first way it opens, so there is something to slide.
    m_selected = m_res.dbg.openings.isEmpty() ? -1 : 0;
    emit resultChanged();
    emit displayChanged();
    setStatus(QStringLiteral("%1 - %2x%3x%4 analysed")
                  .arg(m_fileName).arg(m_opt.cells[0]).arg(m_opt.cells[1]).arg(m_opt.cells[2]));
}

int PuzzleAnalyzer::pieceId(const PuzzlePiece &p) const
{
    return (p.i * m_opt.cells[1] + p.j) * m_opt.cells[2] + p.k;
}

// ---------------------------------------------------------------------------
// Turning the numbers into things to read
// ---------------------------------------------------------------------------
void PuzzleAnalyzer::buildTexts()
{
    const DbgReport &r = m_res.dbg;

    m_mapText = QStringLiteral("min det J inside the domain: %1   ·   including the "
                               "boundary: %2 at (u,v,w) = (%3, %4, %5)")
                    .arg(r.minDetInterior, 0, 'g', 4)
                    .arg(r.minDetJ, 0, 'g', 4)
                    .arg(r.minDetAt[0], 0, 'f', 2).arg(r.minDetAt[1], 0, 'f', 2)
                    .arg(r.minDetAt[2], 0, 'f', 2);

    // The one-paragraph answer to "is it interlocking?".
    if (!r.subsetSearchFull)
        m_keyText = QStringLiteral("Too many pieces to test every group (more than %1): "
                                   "only groups up to %2 pieces were tried, so nothing "
                                   "here is a proof.")
                        .arg(m_opt.fullSubsetsUpTo).arg(r.subsetSizeTested);
    else if (r.interlocked)
        m_keyText = QStringLiteral("Nothing can move: no piece and no group slides out in any "
                                   "direction. Locked solid - which means it cannot be "
                                   "assembled by translations either.");
    else if (r.strictSingleKey)
        m_keyText = QStringLiteral("Single key: piece %1 is the only thing that can move. "
                                   "Level %2%3.")
                        .arg(r.keySequence.value(0)).arg(r.keyLevel)
                        .arg(r.fullyRecursive ? QStringLiteral(" - recursive all the way down")
                                              : QStringLiteral(" - then: ") + r.levelStop);
    else
        m_keyText = QStringLiteral("No single key. The puzzle opens %1 different ways; the "
                                   "smallest group that comes out is %2.")
                        .arg(r.openings.size()).arg(groupLabel(r.smallestMobileSet));

    m_openingRows.clear();
    for (int i = 0; i < r.openings.size(); ++i) {
        const DbgReport::Group &g = r.openings[i];
        m_openingRows << row({ { "label", groupLabel(g.pieces) },
                               { "size", g.pieces.size() },
                               { "clear", clearLabel(g.clearanceDeg) },
                               { "dir", dirLabel(g.dir) } });
    }

    m_stepRows.clear();
    for (int i = 0; i < r.disassembly.size(); ++i) {
        const DbgReport::Group &g = r.disassembly[i];
        m_stepRows << row({ { "step", i + 1 },
                            { "label", groupLabel(g.pieces) },
                            { "clear", clearLabel(g.clearanceDeg) },
                            { "dir", dirLabel(g.dir) } });
    }

    m_pairRows.clear();
    for (const DbgPair &p : r.pairs)
        m_pairRows << row({ { "pair", QStringLiteral("%1 – %2").arg(p.a).arg(p.b) },
                            { "axis", QStringLiteral("uvw").mid(p.axis, 1) },
                            { "spread", QString::number(p.spreadDeg, 'f', 1) },
                            { "clear", p.blocked ? QStringLiteral("blocked") : clearLabel(p.clearanceDeg) },
                            { "blocked", p.blocked } });
}

void PuzzleAnalyzer::buildCriteria()
{
    const DbgReport  &r = m_res.dbg;
    const DbgQuality &q = m_res.quality;

    double dom[6];
    m_tv.domain(dom);
    const double cellParam = (dom[1] - dom[0]) * (dom[3] - dom[2]) * (dom[5] - dom[4]) /
                             double(std::max(1, r.pieces));
    const double limit = m_minThickness * q.modelSize;

    // Per-piece rows.
    m_pieceRows.clear();
    QVector<int> folded, thin;
    int thinnest = -1;
    for (int i = 0; i < q.pieces.size(); ++i) {
        const DbgPieceQuality &pq = q.pieces[i];
        const bool f = folds(pq, cellParam);
        const bool t = pq.thickness >= 0.0 && pq.thickness < limit;
        if (f) folded << i;
        if (t) thin << i;
        if (pq.thickness >= 0.0 &&
            (thinnest < 0 || pq.thickness < q.pieces[thinnest].thickness))
            thinnest = i;
        m_pieceRows << row({ { "id", i },
                             { "volume", QString::number(pq.volume, 'g', 3) },
                             { "rel", q.volMean > 0 ? QString::number(pq.volume / q.volMean, 'f', 2)
                                                    : QStringLiteral("-") },
                             { "thick", pq.thickness < 0 ? QStringLiteral("-")
                                                         : QString::number(pq.thickness, 'g', 3) },
                             { "thickPct", pq.thickness < 0 || q.modelSize <= 0
                                               ? QStringLiteral("-")
                                               : QString::number(100.0 * pq.thickness / q.modelSize, 'f', 1) + QLatin1Char('%') },
                             { "detJ", QString::number(pq.minDetJ, 'g', 3) },
                             { "folds", f },
                             { "thin", t },
                             { "clear", clearLabel(r.pieceClearanceDeg.value(i, -1.0)) } });
    }

    m_criteria.clear();
    const auto add = [&](const QString &name, const QString &state, const QString &detail) {
        m_criteria << row({ { "name", name }, { "state", state }, { "detail", detail } });
    };

    // 1. The map.
    if (r.minDetInterior <= 0.0)
        add(QStringLiteral("Positive Jacobian"), QStringLiteral("fail"),
            QStringLiteral("The map folds (det J = %1 inside the domain)%2. Every other answer "
                           "below is about a shape that passes through itself.")
                .arg(r.minDetInterior, 0, 'g', 3)
                .arg(folded.isEmpty() ? QString()
                                      : QStringLiteral(" - in pieces %1").arg(groupLabel(folded))));
    else if (r.minDetJ <= 0.0)
        add(QStringLiteral("Positive Jacobian"), QStringLiteral("warn"),
            QStringLiteral("Positive inside (%1). Zero only on the boundary at (u,v,w) = "
                           "(%2, %3, %4), where the map degenerates - a pole or a face that collapses to a "
                           "curve - not a fold.")
                .arg(r.minDetInterior, 0, 'g', 3)
                .arg(r.minDetAt[0], 0, 'f', 2).arg(r.minDetAt[1], 0, 'f', 2)
                .arg(r.minDetAt[2], 0, 'f', 2));
    else
        add(QStringLiteral("Positive Jacobian"), QStringLiteral("pass"),
            QStringLiteral("det J > 0 everywhere sampled (min %1).").arg(r.minDetJ, 0, 'g', 3));

    // 2. Single key.
    if (!r.subsetSearchFull)
        add(QStringLiteral("Interlocking in R³: single key"), QStringLiteral("na"),
            QStringLiteral("Too many pieces for a full check (limit %1).").arg(m_opt.fullSubsetsUpTo));
    else if (r.strictSingleKey)
        add(QStringLiteral("Interlocking in R³: single key"), QStringLiteral("pass"),
            QStringLiteral("Only piece %1 can move; every other piece and every group is held.")
                .arg(r.keySequence.value(0)));
    else if (r.interlocked)
        add(QStringLiteral("Interlocking in R³: single key"), QStringLiteral("fail"),
            QStringLiteral("Nothing can move at all - no key, and not assemblable by translation."));
    else
        add(QStringLiteral("Interlocking in R³: single key"), QStringLiteral("fail"),
            QStringLiteral("%1 ways to open. Smallest: %2 (%3 pieces free alone).")
                .arg(r.openings.size()).arg(groupLabel(r.smallestMobileSet))
                .arg(r.mobilePieces));

    // 3. Level k.
    if (r.subsetSearchFull) {
        QStringList keys;
        for (int k : r.keySequence) keys << QString::number(k);
        add(QStringLiteral("Interlocking level k"),
            r.keyLevel == 0 ? QStringLiteral("fail")
                            : (r.fullyRecursive ? QStringLiteral("pass") : QStringLiteral("warn")),
            QStringLiteral("k = %1%2. Stopped: %3. (Song 2012 recursive level; Chen 2022's "
                           "move-count level needs motion search - not computed.)")
                .arg(r.keyLevel)
                .arg(keys.isEmpty() ? QString()
                                    : QStringLiteral(", keys in order %1").arg(keys.join(QStringLiteral(" → "))))
                .arg(r.levelStop.isEmpty() ? QStringLiteral("-") : r.levelStop));
    }

    // 4. Thickness.
    if (thinnest < 0)
        add(QStringLiteral("Minimum thickness"), QStringLiteral("na"),
            QStringLiteral("No ray could be measured."));
    else
        add(QStringLiteral("Minimum thickness"),
            thin.isEmpty() ? QStringLiteral("pass") : QStringLiteral("fail"),
            QStringLiteral("Thinnest: piece %1, %2 (%3% of the model) near (%4, %5, %6). "
                           "Limit %7 = %8% of the model%9.")
                .arg(thinnest)
                .arg(q.pieces[thinnest].thickness, 0, 'g', 3)
                .arg(100.0 * q.pieces[thinnest].thickness / q.modelSize, 0, 'f', 1)
                .arg(q.pieces[thinnest].thickAt[0], 0, 'f', 2)
                .arg(q.pieces[thinnest].thickAt[1], 0, 'f', 2)
                .arg(q.pieces[thinnest].thickAt[2], 0, 'f', 2)
                .arg(limit, 0, 'g', 3)
                .arg(100.0 * m_minThickness, 0, 'f', 1)
                .arg(thin.isEmpty() ? QString()
                                    : QStringLiteral(" - too thin: %1").arg(groupLabel(thin))));

    // 5. One component.
    add(QStringLiteral("One connected component per piece"),
        folded.isEmpty() ? QStringLiteral("pass") : QStringLiteral("fail"),
        folded.isEmpty()
            ? QStringLiteral("Each piece is the image of one box under a continuous map, so it is "
                             "one piece by construction, and det J > 0 in every piece, so none "
                             "passes through itself.")
            : QStringLiteral("Pieces %1 fold (det J < 0): the piece passes through itself, so it "
                             "is not a valid solid.").arg(groupLabel(folded)));

    // 6. Sizes.
    const double ratio = q.volMin > 0 ? q.volMax / q.volMin : 0.0;
    add(QStringLiteral("Roughly uniform piece sizes"),
        ratio <= 0 ? QStringLiteral("na")
                   : (ratio <= 2.0 ? QStringLiteral("pass")
                                   : (ratio <= 3.0 ? QStringLiteral("warn") : QStringLiteral("fail"))),
        QStringLiteral("Largest / smallest volume = %1 (pass ≤ 2, warn ≤ 3); spread %2% "
                       "of the mean.")
            .arg(ratio, 0, 'f', 2).arg(100.0 * q.volCv, 0, 'f', 0));

    // 7. Joints in cavities.
    add(QStringLiteral("No joints exposed in cavities"), QStringLiteral("na"),
        QStringLiteral("No joints in this puzzle (pins removed), and a trivariate cell has no "
                       "inner cavity. Applies once joints are added."));
}

// ---------------------------------------------------------------------------
// Independent check
// ---------------------------------------------------------------------------
void PuzzleAnalyzer::verifySelected()
{
    if (m_busy || !m_hasResult) return;
    const QVector<DbgReport::Group> &op = m_res.dbg.openings;
    if (m_selected < 0 || m_selected >= op.size()) {
        m_sweepText = QStringLiteral("Select an opening first.");
        emit sweepChanged();
        return;
    }

    const DbgReport::Group g = op[m_selected];
    QVector<int> freeAlone;
    for (double c : m_res.dbg.pieceClearanceDeg)
        freeAlone.append(c >= 0.0 || c < -1.5 ? 1 : 0);   // -1 = blocked
    const double travel = 0.4 * m_res.quality.modelSize;
    const Trivariate *tv = &m_tv;
    const DbgOptions opt = m_opt;

    setBusy(true);
    setStatus(QStringLiteral("Sliding %1 for real ...").arg(groupLabel(g.pieces)));

    auto *w = new QFutureWatcher<QString>(this);
    connect(w, &QFutureWatcher<QString>::finished, this, [this, w] {
        m_sweepText = w->result();
        w->deleteLater();
        setBusy(false);
        setStatus(QStringLiteral("Slide check done"));
        emit sweepChanged();
    });
    w->setFuture(QtConcurrent::run([tv, opt, g, freeAlone, travel]() {
        QStringList out;
        const auto line = [&](const QString &what, const QVector<int> &grp, const double d[3],
                              const QString &expect) {
            const DbgSweep s = DbgAnalysis::sweep(*tv, opt, grp, d, travel);
            const QString got = s.collided > 0 ? QStringLiteral("collides") : QStringLiteral("clear");
            const bool agree = (expect == QLatin1String("either")) || (expect == got);
            out << QStringLiteral("%1 %2 - DBG expects %3, sliding found %4 (%5 of %6 points hit%7)")
                       .arg(agree ? QStringLiteral("✓") : QStringLiteral("✗"))
                       .arg(what, expect, got)
                       .arg(s.collided).arg(s.samples)
                       .arg(s.firstHit >= 0 ? QStringLiteral(", first after %1").arg(s.firstHit, 0, 'g', 3)
                                            : QString());
        };
        const double plus[3]  = {  g.dir[0],  g.dir[1],  g.dir[2] };
        const double minus[3] = { -g.dir[0], -g.dir[1], -g.dir[2] };
        line(QStringLiteral("%1 along +d").arg(groupLabel(g.pieces)), g.pieces, plus,
             QStringLiteral("clear"));
        line(QStringLiteral("%1 along -d").arg(groupLabel(g.pieces)), g.pieces, minus,
             QStringLiteral("collides"));
        if (g.pieces.size() > 1)
            for (int p : g.pieces)
                line(QStringLiteral("piece %1 alone along +d").arg(p), QVector<int> { p }, plus,
                     freeAlone.value(p) > 0 ? QStringLiteral("either") : QStringLiteral("collides"));
        out << QStringLiteral("Travel %1. Translation only: assemblable under translational "
                              "blocking; rotational/swept check pending.").arg(travel, 0, 'g', 3);
        return out.join(QLatin1Char('\n'));
    }));
}

// ---------------------------------------------------------------------------
// What the 3D view shows
// ---------------------------------------------------------------------------
void PuzzleAnalyzer::setSelected(int i)
{
    if (i == m_selected) return;
    m_selected = i;
    m_sweepText.clear();
    emit sweepChanged();
    emit displayChanged();
}

void PuzzleAnalyzer::setPull(double v)
{
    v = qBound(0.0, v, 1.0);
    if (qFuzzyCompare(v + 1.0, m_pull + 1.0)) return;
    m_pull = v;
    emit displayChanged();
}

void PuzzleAnalyzer::setPlayhead(double v)
{
    v = qBound(0.0, v, double(m_res.dbg.disassembly.size()));
    if (qFuzzyCompare(v + 1.0, m_playhead + 1.0)) return;
    m_playhead = v;
    emit displayChanged();
}

void PuzzleAnalyzer::setMinThickness(double v)
{
    v = qBound(0.0, v, 1.0);
    if (qFuzzyCompare(v + 1.0, m_minThickness + 1.0)) return;
    m_minThickness = v;
    if (m_hasResult) buildCriteria();
    emit resultChanged();
    emit displayChanged();
}

QVector<PuzzlePiece> PuzzleAnalyzer::displayPieces() const
{
    QVector<PuzzlePiece> out = m_pieces;
    if (!m_hasResult)
        return out;

    const DbgReport  &r = m_res.dbg;
    const DbgQuality &q = m_res.quality;
    const double reach = kPullReach * q.modelSize;

    double dom[6];
    m_tv.domain(dom);
    const double cellParam = (dom[1] - dom[0]) * (dom[3] - dom[2]) * (dom[5] - dom[4]) /
                             double(std::max(1, r.pieces));

    QVector<double> off(out.size() * 3, 0.0);
    QVector<bool>   moving(out.size(), false);

    // The selected opening, slid out by the Pull slider.
    if (m_selected >= 0 && m_selected < r.openings.size()) {
        const DbgReport::Group &g = r.openings[m_selected];
        for (int p : g.pieces) {
            if (p < 0 || p >= out.size()) continue;
            moving[p] = true;
            for (int k = 0; k < 3; ++k) off[p * 3 + k] += g.dir[k] * m_pull * reach;
        }
    }

    // The disassembly, played to the playhead: step s runs from s to s + 1.
    for (int s = 0; s < r.disassembly.size(); ++s) {
        const double t = qBound(0.0, m_playhead - s, 1.0);
        if (t <= 0.0) break;
        const DbgReport::Group &g = r.disassembly[s];
        for (int p : g.pieces) {
            if (p < 0 || p >= out.size()) continue;
            if (t < 1.0) moving[p] = true;
            for (int k = 0; k < 3; ++k) off[p * 3 + k] += g.dir[k] * t * 1.2 * reach;
        }
    }

    for (int i = 0; i < out.size(); ++i) {
        PuzzlePiece &pc = out[i];
        if (i < q.pieces.size()) {
            const DbgPieceQuality &pq = q.pieces[i];
            if (pq.thickness >= 0.0 && pq.thickness < m_minThickness * q.modelSize)
                pc.tint = kThin;
            if (folds(pq, cellParam))
                pc.tint = kFolded;
        }
        if (moving[i]) pc.tint = kMoving;

        const float d[3] = { float(off[i * 3]), float(off[i * 3 + 1]), float(off[i * 3 + 2]) };
        if (d[0] == 0.0f && d[1] == 0.0f && d[2] == 0.0f) continue;
        for (int v = 0; v + 2 < pc.mesh.pos.size(); v += 3)
            for (int k = 0; k < 3; ++k) pc.mesh.pos[v + k] += d[k];
        for (int k = 0; k < 3; ++k) {
            pc.mesh.bmin[k] += d[k];
            pc.mesh.bmax[k] += d[k];
            pc.centre[k]    += d[k];
        }
    }
    return out;
}

bool PuzzleAnalyzer::viewBounds(float lo[3], float hi[3]) const
{
    if (m_pieces.isEmpty() || m_lo[0] > m_hi[0])
        return false;
    // Room for a piece pulled a full reach in any direction.
    float size = 0.0f;
    for (int k = 0; k < 3; ++k) size = std::max(size, m_hi[k] - m_lo[k]);
    const float pad = 0.6f * size;
    for (int k = 0; k < 3; ++k) { lo[k] = m_lo[k] - pad; hi[k] = m_hi[k] + pad; }
    return true;
}
