//
// PlannerFigure - implementation: renders the pieces once, then paints the four
// figures over that render with QPainter.
//

#include "PlannerFigure.h"

#include "AssemblyOrder.h"
#include "MeshView.h"

#include <QDir>
#include <QFileInfo>
#include <QFont>
#include <QFontMetrics>
#include <QImage>
#include <QPainter>
#include <QPolygonF>
#include <QStandardPaths>

#include <algorithm>
#include <cmath>

namespace {

const QColor kInk      (0x14, 0x1a, 0x20);
const QColor kInkSoft  (0x5c, 0x68, 0x74);
const QColor kRule     (0xc6, 0xd0, 0xd9);
const QColor kAccent   (0xd9, 0x48, 0x0f);
const QColor kStuck    (0xb4, 0x23, 0x18);
const QColor kAxis[3] = { QColor(0xc0, 0x39, 0x2b),
                          QColor(0x2e, 0x8b, 0x57),
                          QColor(0x2f, 0x6f, 0x99) };
const QColor kBlockedFill(0xf6, 0xdc, 0xd8);
const QColor kBlockedText(0x9b, 0x2c, 0x1f);
const QColor kFreeFill   (0xdd, 0xef, 0xe3);
const QColor kFreeText   (0x1e, 0x6b, 0x3a);

const QChar kDash (0x2014);
const QChar kArrow(0x2192);
const QChar kDot  (0x00b7);

constexpr int    kCanvasW = 1600;
constexpr int    kHeaderH = 150;
constexpr int    kRenderH = 1000;
constexpr int    kFooterH = 110;
constexpr double kExplode = 0.45;

QFont font(int px, bool bold = false)
{
    QFont f(QStringLiteral("Segoe UI"));
    f.setPixelSize(px);
    f.setBold(bold);
    return f;
}

QString dirText(int d)
{
    QString s = QString::fromLatin1(Planner::dirName(d));
    if (s.startsWith(QLatin1Char('-')))
        s[0] = QChar(0x2212);
    return s;
}

QString idList(const QVector<int> &ids)
{
    QStringList s;
    for (int i : ids)
        s << QString::number(i);
    return s.join(QStringLiteral(", "));
}

void beginPainter(QPainter &p)
{
    p.setRenderHint(QPainter::Antialiasing, true);
    p.setRenderHint(QPainter::TextAntialiasing, true);
}

// Title and a wrapped line of explanation. Returns where content starts.
int drawHeader(QPainter &p, int width, const QString &title,
               const QString &subtitle)
{
    p.setPen(kInk);
    p.setFont(font(34, true));
    p.drawText(QRect(48, 24, width - 96, 50),
               Qt::AlignLeft | Qt::AlignVCenter, title);
    p.setPen(kInkSoft);
    p.setFont(font(20));
    p.drawText(QRect(48, 80, width - 96, 64),
               Qt::AlignLeft | Qt::AlignTop | Qt::TextWordWrap, subtitle);
    return kHeaderH;
}

// Lines stacked upward from the bottom edge.
void drawFooter(QPainter &p, int width, int height, const QStringList &lines)
{
    p.setPen(kInkSoft);
    p.setFont(font(17));
    int y = height - 24 - 26 * int(lines.size());
    for (const QString &l : lines) {
        p.drawText(QRect(48, y, width - 96, 26),
                   Qt::AlignLeft | Qt::AlignVCenter, l);
        y += 26;
    }
}

void drawNode(QPainter &p, const QPointF &at, qreal r, const QColor &fill,
              int id, const QPen &outline, bool label = true)
{
    p.setPen(outline);
    p.setBrush(fill);
    p.drawEllipse(at, r, r);
    if (!label)
        return;
    p.setPen(kInk);
    p.setFont(font(qMax(10, int(r * 1.05)), true));
    p.drawText(QRectF(at.x() - r, at.y() - r, 2.0 * r, 2.0 * r),
               Qt::AlignCenter, QString::number(id));
}

void drawArrow(QPainter &p, const QPointF &from, const QPointF &dir,
               qreal length, const QColor &colour, qreal width)
{
    const qreal len = std::hypot(dir.x(), dir.y());
    if (len < 1e-9)
        return;
    const QPointF u(dir.x() / len, dir.y() / len);
    const QPointF n(-u.y(), u.x());
    const QPointF tip  = from + u * length;
    const qreal   head = width * 3.0;

    p.setPen(QPen(colour, width, Qt::SolidLine, Qt::RoundCap));
    p.drawLine(from, tip - u * (head * 0.5));

    QPolygonF tri;
    tri << tip + u * (head * 0.6)
        << tip - u * (head * 0.9) + n * (head * 0.7)
        << tip - u * (head * 0.9) - n * (head * 0.7);
    p.setPen(Qt::NoPen);
    p.setBrush(colour);
    p.drawPolygon(tri);
}

// Screen direction of a planner direction, taken from the view's own camera, so
// an arrow points the way the drawn piece would actually move.
QPointF screenDir(const MeshView &view, int dir)
{
    return view.axisOnScreen(Planner::dirAxis(dir)) *
           qreal(Planner::dirSign(dir));
}

// The exploded view on a white ground: MeshView paints its own dark ground, and
// every pixel still exactly that colour is lifted to white so the image sits on
// a slide.
QImage renderPieces(MeshView &view, const QVector<PuzzlePiece> &pieces,
                    int w, int h)
{
    view.setWidth(w);
    view.setHeight(h);
    view.setPieces(pieces);
    view.setExplode(kExplode);

    QImage img(w, h, QImage::Format_RGB32);
    img.fill(Qt::white);
    {
        QPainter p(&img);
        view.paint(&p);
    }

    const QRgb ground = MeshView::background() & 0x00ffffffu;
    for (int y = 0; y < h; ++y) {
        QRgb *line = reinterpret_cast<QRgb *>(img.scanLine(y));
        for (int x = 0; x < w; ++x)
            if ((line[x] & 0x00ffffffu) == ground)
                line[x] = qRgb(0xff, 0xff, 0xff);
    }
    return img;
}

bool shown(const QVector<bool> &has, int i)
{
    return i >= 0 && i < has.size() && has[i];
}

QImage figurePieces(const QImage &render, const QVector<PuzzlePiece> &pieces,
                    const QVector<QPointF> &at, const QVector<bool> &has,
                    const QString &title)
{
    const int H = kHeaderH + kRenderH + kFooterH;
    QImage img(kCanvasW, H, QImage::Format_ARGB32_Premultiplied);
    img.fill(Qt::white);
    QPainter p(&img);
    beginPainter(p);

    const int top = drawHeader(p, kCanvasW,
        QStringLiteral("%1 %2 the pieces").arg(title).arg(kDash),
        QStringLiteral("Divided into %1 pieces, shown "
                       "pulled apart. The number on each piece is the id the "
                       "planner uses in the next three pictures.")
            .arg(pieces.size()));

    p.drawImage(QPoint(0, top), render);

    const QPointF off(0, top);
    for (int i = 0; i < pieces.size(); ++i)
        if (shown(has, i))
            drawNode(p, at[i] + off, 17, Qt::white, i, QPen(kInk, 2.0));

    drawFooter(p, kCanvasW, H, {
        QStringLiteral("Each colour is one piece. They are pushed outward from "
                       "the centre only so every piece is visible; assembled, "
                       "they fit together with no gaps.") });
    p.end();
    return img;
}

QImage figureGraph(const QImage &render, const QVector<PuzzlePiece> &pieces,
                   const Planner::Graph &g, const QVector<QPointF> &at,
                   const QVector<bool> &has)
{
    const int H = kHeaderH + kRenderH + kFooterH;
    QImage img(kCanvasW, H, QImage::Format_ARGB32_Premultiplied);
    img.fill(Qt::white);
    QPainter p(&img);
    beginPainter(p);

    const int top = drawHeader(p, kCanvasW,
        QStringLiteral("Step 1 %1 who touches whom").arg(kDash),
        QStringLiteral("Every line joins two pieces that share a face. Its "
                       "colour is the axis that face is perpendicular to, which "
                       "is the direction those two pieces press on each other."));

    p.setOpacity(0.28);
    p.drawImage(QPoint(0, top), render);
    p.setOpacity(1.0);

    const QPointF off(0, top);
    for (const Planner::Contact &c : g.contacts) {
        if (!shown(has, c.lowSide) || !shown(has, c.highSide) ||
            c.axis < 0 || c.axis > 2)
            continue;
        p.setPen(QPen(kAxis[c.axis], 4.0, Qt::SolidLine, Qt::RoundCap));
        p.drawLine(at[c.lowSide] + off, at[c.highSide] + off);
    }
    for (int i = 0; i < pieces.size(); ++i)
        if (shown(has, i))
            drawNode(p, at[i] + off, 18, QColor(MeshView::tintFor(pieces[i])),
                     i, QPen(kInk, 2.0));

    p.setFont(font(19));
    for (int a = 0; a < 3; ++a) {
        const int y = top + 24 + a * 32;
        p.setPen(QPen(kAxis[a], 5.0, Qt::SolidLine, Qt::RoundCap));
        p.drawLine(QPointF(kCanvasW - 300, y), QPointF(kCanvasW - 250, y));
        p.setPen(kInk);
        p.drawText(QRect(kCanvasW - 236, y - 14, 200, 28),
                   Qt::AlignLeft | Qt::AlignVCenter,
                   QStringLiteral("face across %1").arg(QChar(int('X') + a)));
    }

    int fewest = 0, most = 0;
    for (int i = 0; i < g.incident.size(); ++i) {
        const int d = int(g.incident[i].size());
        fewest = (i == 0) ? d : qMin(fewest, d);
        most   = qMax(most, d);
    }

    drawFooter(p, kCanvasW, H, {
        QStringLiteral("%1 pieces %2 %3 shared faces %2 each piece touches "
                       "between %4 and %5 others")
            .arg(pieces.size()).arg(kDot).arg(g.contactCount())
            .arg(fewest).arg(most),
        QStringLiteral("Only real faces count: pieces that meet at just an edge "
                       "or a corner are not joined.") });
    p.end();
    return img;
}

QImage figureBlocking(const QVector<PuzzlePiece> &pieces,
                      const Planner::Graph &g,
                      const Planner::BlockingModel &model)
{
    const int n = g.pieceCount();
    constexpr int kMaxRows = 60;
    const int rows = qMin(n, kMaxRows);

    constexpr int left = 48, pieceW = 130, dirW = 200, rowH = 36, headRowH = 44;
    const int tableW = pieceW + 6 * dirW;
    const int W = qMax(kCanvasW, 2 * left + tableW);
    const int H = kHeaderH + headRowH + rows * rowH + (n > rows ? 40 : 0)
                + 40 + kFooterH;

    QImage img(W, H, QImage::Format_ARGB32_Premultiplied);
    img.fill(Qt::white);
    QPainter p(&img);
    beginPainter(p);

    int y = drawHeader(p, W,
        QStringLiteral("Step 2 %1 which ways can each piece move?").arg(kDash),
        QStringLiteral("With every piece in place, try sliding each one straight "
                       "out in all six directions. A direction is blocked when a "
                       "neighbour sits on that face."));

    p.setFont(font(20, true));
    p.setPen(kInk);
    p.drawText(QRect(left, y, pieceW, headRowH),
               Qt::AlignLeft | Qt::AlignVCenter, QStringLiteral("piece"));
    for (int d = 0; d < Planner::DirectionCount; ++d) {
        p.setPen(kAxis[Planner::dirAxis(d)]);
        p.drawText(QRect(left + pieceW + d * dirW, y, dirW, headRowH),
                   Qt::AlignCenter, dirText(d));
    }
    y += headRowH;

    const QVector<bool> all(n, true);
    for (int i = 0; i < rows; ++i) {
        const int ry = y + i * rowH;
        const Planner::DirMask m = model.blocked(g, i, all);

        drawNode(p, QPointF(left + 12, ry + rowH / 2.0), 10,
                 i < pieces.size() ? QColor(MeshView::tintFor(pieces[i])) : kRule,
                 i, QPen(kInk, 1.2), false);
        p.setPen(kInk);
        p.setFont(font(19, true));
        p.drawText(QRect(left + 32, ry, pieceW - 32, rowH),
                   Qt::AlignLeft | Qt::AlignVCenter, QString::number(i));

        p.setFont(font(15));
        for (int d = 0; d < Planner::DirectionCount; ++d) {
            const QRect cell(left + pieceW + d * dirW + 3, ry + 3,
                             dirW - 6, rowH - 6);
            const bool blocked = Planner::isBlocked(m, d);
            p.setPen(Qt::NoPen);
            p.setBrush(blocked ? kBlockedFill : kFreeFill);
            p.drawRoundedRect(cell, 5, 5);
            p.setPen(blocked ? kBlockedText : kFreeText);
            p.drawText(cell, Qt::AlignCenter,
                       blocked ? QStringLiteral("blocked by %1")
                                     .arg(idList(model.blockers(g, i, d, all)))
                               : QStringLiteral("free"));
        }
    }
    y += rows * rowH;

    if (n > rows) {
        p.setPen(kInkSoft);
        p.setFont(font(18));
        p.drawText(QRect(left, y + 6, tableW, 30),
                   Qt::AlignLeft | Qt::AlignVCenter,
                   QStringLiteral("%1 and %2 more pieces")
                       .arg(QChar(0x2026)).arg(n - rows));
    }

    drawFooter(p, W, H, {
        QStringLiteral("Straight-line moves only %1 no turning, and no check of "
                       "the space a piece sweeps through on its way out.")
            .arg(kDash),
        AssemblyOrder::caveat() });
    p.end();
    return img;
}

QImage figureOrder(const QVector<PuzzlePiece> &pieces, const Planner::Graph &g,
                   const Planner::BlockingModel &model,
                   const Planner::Plan &plan, const MeshView &view,
                   const QVector<QPointF> &at, const QVector<bool> &has)
{
    const int  n          = g.pieceCount();
    const int  steps      = int(plan.removal.size());
    const bool stuckPanel = !plan.complete;
    constexpr int kMaxPanels = 48;
    const int wanted = steps + (stuckPanel ? 1 : 0);
    const int panels = qMin(wanted, kMaxPanels);

    constexpr int left = 48, pw = 300, ph = 290, gap = 18;
    const int cols  = qBound(3, int(std::ceil(std::sqrt(qMax(1, panels) * 1.5))), 8);
    const int rowsN = qMax(1, (panels + cols - 1) / cols);
    const int W     = qMax(kCanvasW, 2 * left + cols * pw + (cols - 1) * gap);

    QStringList order;
    for (const Planner::Step &s : plan.assembly)
        order << QStringLiteral("%1 (%2)").arg(s.piece).arg(dirText(s.dir));
    const QString assembleText = plan.complete
        ? QStringLiteral("Assembly order %1 the same steps backwards, each piece "
                         "going in the way it came out:   %2")
              .arg(kDash)
              .arg(order.join(QStringLiteral("  %1  ").arg(kArrow)))
        : QStringLiteral("No complete order found: %1 piece(s) left with no free "
                         "direction (%2). The search is greedy, so this is a "
                         "failure of this search, not proof that no order exists.")
              .arg(plan.stuck.size()).arg(idList(plan.stuck));
    const QFont footFont = font(18);
    const QRect assembleBox = QFontMetrics(footFont).boundingRect(
        QRect(0, 0, W - 2 * left, 100000), Qt::TextWordWrap, assembleText);
    const int moreH   = (panels < wanted) ? 34 : 0;
    const int footerH = moreH + assembleBox.height() + 130;

    const int H = kHeaderH + rowsN * (ph + gap) + footerH;
    QImage img(W, H, QImage::Format_ARGB32_Premultiplied);
    img.fill(Qt::white);
    QPainter p(&img);
    beginPainter(p);

    const int top = drawHeader(p, W,
        QStringLiteral("Step 3 %1 take the puzzle apart, then reverse it")
            .arg(kDash),
        QStringLiteral("Repeatedly pick a piece that has at least one free "
                       "direction and slide it out. Orange marks the piece "
                       "leaving at each step; grey rings are pieces already gone."));

    qreal minX = 0, minY = 0, maxX = 0, maxY = 0;
    bool any = false;
    for (int i = 0; i < at.size(); ++i) {
        if (!shown(has, i))
            continue;
        if (!any) {
            minX = maxX = at[i].x();
            minY = maxY = at[i].y();
            any = true;
        }
        minX = qMin(minX, at[i].x());  maxX = qMax(maxX, at[i].x());
        minY = qMin(minY, at[i].y());  maxY = qMax(maxY, at[i].y());
    }

    const qreal pad = 36, captionH = 60, nodeRoom = 22;
    const qreal spanX = qMax<qreal>(maxX - minX, 1.0);
    const qreal spanY = qMax<qreal>(maxY - minY, 1.0);
    const qreal innerH = ph - pad - captionH - nodeRoom;
    const qreal scale = qMin((pw - 2 * pad) / spanX, innerH / spanY);
    const auto place = [&](int piece, const QPointF &origin) {
        const qreal cx = (pw - scale * spanX) / 2.0;
        const qreal cy = pad + (innerH - scale * spanY) / 2.0;
        return origin + QPointF(cx + (at[piece].x() - minX) * scale,
                                cy + (at[piece].y() - minY) * scale);
    };

    QVector<bool> present(n, true);
    for (int k = 0; k < panels && any; ++k) {
        const QPointF origin(left + (k % cols) * (pw + gap),
                             top  + (k / cols) * (ph + gap));

        const bool isStuck = stuckPanel && k == steps;
        const int  mover   = isStuck ? -1 : plan.removal[k].piece;
        const int  dir     = isStuck ? -1 : plan.removal[k].dir;

        p.setPen(QPen(kRule, 1.5));
        p.setBrush(Qt::white);
        p.drawRoundedRect(QRectF(origin, QSizeF(pw, ph)), 10, 10);

        p.setPen(QPen(kRule, 2.0));
        for (const Planner::Contact &c : g.contacts) {
            const int a = c.lowSide, b = c.highSide;
            if (a < 0 || b < 0 || a >= n || b >= n || !present[a] ||
                !present[b] || !shown(has, a) || !shown(has, b))
                continue;
            p.drawLine(place(a, origin), place(b, origin));
        }

        for (int i = 0; i < n; ++i) {
            if (!shown(has, i) || i == mover)
                continue;
            const QPointF c = place(i, origin);
            if (!present[i]) {
                p.setPen(QPen(kRule, 1.6));
                p.setBrush(Qt::NoBrush);
                p.drawEllipse(c, 5.0, 5.0);
            }
            else {
                drawNode(p, c, 11, QColor(MeshView::tintFor(pieces[i])), i,
                         QPen(isStuck ? kStuck : kInk, isStuck ? 2.6 : 1.2));
            }
        }

        const QRectF caption(origin.x() + 14, origin.y() + ph - captionH,
                             pw - 28, 30);
        if (isStuck) {
            p.setPen(kStuck);
            p.setFont(font(17, true));
            p.drawText(caption.adjusted(0, 0, 0, 28),
                       Qt::AlignLeft | Qt::AlignVCenter | Qt::TextWordWrap,
                       QStringLiteral("stuck: no remaining piece has a free "
                                      "direction"));
            continue;
        }

        const Planner::DirMask m = model.blocked(g, mover, present);
        QStringList free;
        for (int d = 0; d < Planner::DirectionCount; ++d)
            if (!Planner::isBlocked(m, d))
                free << dirText(d);

        if (shown(has, mover)) {
            const QPointF c = place(mover, origin);
            drawArrow(p, c, screenDir(view, dir), 54, kAccent, 4.5);
            drawNode(p, c, 15, QColor(MeshView::tintFor(pieces[mover])), mover,
                     QPen(kAccent, 3.5));
        }

        p.setPen(kInk);
        p.setFont(font(18, true));
        p.drawText(caption, Qt::AlignLeft | Qt::AlignVCenter,
                   QStringLiteral("%1.  piece %2 out along %3")
                       .arg(k + 1).arg(mover).arg(dirText(dir)));
        p.setPen(kInkSoft);
        p.setFont(font(15));
        p.drawText(caption.translated(0, 28), Qt::AlignLeft | Qt::AlignVCenter,
                   QStringLiteral("free at this moment: %1")
                       .arg(free.join(QLatin1Char(' '))));

        if (mover >= 0 && mover < n)
            present[mover] = false;
    }

    int fy = top + rowsN * (ph + gap) + 10;
    if (panels < wanted) {
        p.setPen(kInkSoft);
        p.setFont(font(18));
        p.drawText(QRect(left, fy, W - 2 * left, 30),
                   Qt::AlignLeft | Qt::AlignVCenter,
                   QStringLiteral("%1 and %2 more steps, not drawn")
                       .arg(QChar(0x2026)).arg(wanted - panels));
        fy += moreH;
    }

    p.setPen(kInk);
    p.setFont(footFont);
    p.drawText(QRect(left, fy, W - 2 * left, assembleBox.height() + 6),
               Qt::AlignLeft | Qt::AlignTop | Qt::TextWordWrap, assembleText);

    drawFooter(p, W, H, {
        QStringLiteral("A division into boxes always has some piece free to "
                       "leave, so this step can only fail once joints restrict "
                       "how pieces may move."),
        AssemblyOrder::caveat() });
    p.end();
    return img;
}

}

QString PlannerFigure::folderFor(const QString &modelPath)
{
    if (modelPath.isEmpty())
        return QString();
    const QFileInfo fi(modelPath);
    return fi.absolutePath() + QLatin1Char('/') + fi.completeBaseName()
         + QStringLiteral("_planner");
}

PlannerFigure::Result PlannerFigure::write(const QVector<PuzzlePiece> &pieces,
                                           const Planner::Graph &graph,
                                           const Planner::BlockingModel &model,
                                           const Planner::Plan &plan,
                                           const QString &folder,
                                           const QString &title)
{
    Result r;
    const QString name = title.isEmpty() ? QStringLiteral("model") : title;

    QString dir = folder;
    if (dir.isEmpty() || !QDir().mkpath(dir)) {
        const QString fallback =
            QStandardPaths::writableLocation(QStandardPaths::PicturesLocation)
            + QStringLiteral("/PuzzleDivider/") + name
            + QStringLiteral("_planner");
        if (!dir.isEmpty())
            r.problems << QStringLiteral("could not create %1 - wrote to %2 "
                                         "instead")
                              .arg(QDir::toNativeSeparators(dir),
                                   QDir::toNativeSeparators(fallback));
        dir = fallback;
        if (!QDir().mkpath(dir)) {
            r.problems << QStringLiteral("could not create %1")
                              .arg(QDir::toNativeSeparators(dir));
            return r;
        }
    }
    r.folder = dir;

    if (pieces.isEmpty() || graph.pieceCount() != pieces.size()) {
        r.problems << QStringLiteral("the planner graph does not match the "
                                     "pieces (%1 nodes, %2 pieces) - no figures "
                                     "drawn")
                          .arg(graph.pieceCount()).arg(pieces.size());
        return r;
    }

    MeshView view;
    const QImage render = renderPieces(view, pieces, kCanvasW, kRenderH);
    QVector<bool> has;
    const QVector<QPointF> at = view.pieceAnchors(kCanvasW, kRenderH, &has);

    const auto save = [&](const QImage &img, const QString &file) {
        const QString path = dir + QLatin1Char('/') + file;
        if (img.save(path, "PNG"))
            r.written << path;
        else
            r.problems << QStringLiteral("could not write %1")
                              .arg(QDir::toNativeSeparators(path));
    };

    save(figurePieces(render, pieces, at, has, name),
         QStringLiteral("1_pieces.png"));
    save(figureGraph(render, pieces, graph, at, has),
         QStringLiteral("2_adjacency_graph.png"));
    save(figureBlocking(pieces, graph, model),
         QStringLiteral("3_blocking.png"));
    save(figureOrder(pieces, graph, model, plan, view, at, has),
         QStringLiteral("4_removal_order.png"));
    return r;
}
