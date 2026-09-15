#include "MeshView.h"

#include <QMouseEvent>
#include <QPainter>
#include <QWheelEvent>
#include <cmath>
#include <limits>

namespace {

constexpr float kDegToRad     = 3.14159265358979323846f / 180.0f;
constexpr int   kMaxDrawEdges = 400000;      // beyond this, edges are subsampled

const QRgb kBackground = qRgb(0x20, 0x24, 0x2b);
const QRgb kSinglePart = qRgb(0xc9, 0xa0, 0x6a);

// Distinct hues for adjacent pieces. Grid neighbours land on different entries
// because consecutive cells differ in k, so a face between two pieces is always
// a colour change - which is the whole point of looking at an exploded view.
const QRgb kPalette[] = {
    qRgb(0xd8, 0x9c, 0x62), qRgb(0x7a, 0xb0, 0xc8), qRgb(0x9c, 0xc0, 0x7e),
    qRgb(0xcf, 0x8b, 0x8b), qRgb(0xa9, 0x92, 0xc9), qRgb(0xd6, 0xc0, 0x70),
    qRgb(0x74, 0xbd, 0xa8), qRgb(0xc4, 0x8f, 0xb4)
};
constexpr int kPaletteSize = int(sizeof(kPalette) / sizeof(kPalette[0]));

} // namespace

MeshView::MeshView(QQuickItem *parent)
    : QQuickPaintedItem(parent)
{
    setAcceptedMouseButtons(Qt::LeftButton | Qt::RightButton);
    setAcceptHoverEvents(false);
    setFlag(ItemHasContents, true);
}

// ------------------------------------------------------------- geometry in --

void MeshView::setMesh(const MeshData &mesh)
{
    m_parts.clear();
    if (!mesh.isEmpty()) {
        Part p;
        p.mesh = mesh;
        p.tint = kSinglePart;
        m_parts.append(std::move(p));
    }
    rebuildParts();
    emit meshChanged();
    update();
}

void MeshView::setPieces(const QVector<PuzzlePiece> &pieces)
{
    m_parts.clear();
    m_parts.reserve(pieces.size());
    for (int n = 0; n < pieces.size(); ++n) {
        if (pieces[n].mesh.isEmpty())
            continue;
        Part p;
        p.mesh = pieces[n].mesh;
        p.tint   = tintFor(pieces[n]);
        p.source = n;
        m_parts.append(std::move(p));
    }
    rebuildParts();
    emit meshChanged();
    update();
}

void MeshView::clearMesh()
{
    m_parts.clear();
    rebuildParts();
    emit meshChanged();
    update();
}

QRgb MeshView::tintFor(const PuzzlePiece &piece)
{
    // Colour by cell index rather than list position, so a piece keeps its
    // colour when the grid changes size.
    const int h = piece.i * 5 + piece.j * 3 + piece.k;
    return kPalette[((h % kPaletteSize) + kPaletteSize) % kPaletteSize];
}

QRgb MeshView::background()
{
    return kBackground;
}

// Assigns each part its slice of m_proj and the direction it moves when the
// assembly is exploded: straight out from the centre of the whole set.
void MeshView::rebuildParts()
{
    m_proj.clear();

    for (int k = 0; k < 3; ++k) {
        m_bmin[k] =  std::numeric_limits<float>::max();
        m_bmax[k] = -std::numeric_limits<float>::max();
    }
    if (m_parts.isEmpty()) {
        for (int k = 0; k < 3; ++k)
            m_bmin[k] = m_bmax[k] = 0.0f;
        return;
    }

    int base = 0;
    for (Part &p : m_parts) {
        p.base = base;
        base  += p.mesh.vertexCount();
        for (int k = 0; k < 3; ++k) {
            m_bmin[k] = qMin(m_bmin[k], p.mesh.bmin[k]);
            m_bmax[k] = qMax(m_bmax[k], p.mesh.bmax[k]);
        }
    }
    m_proj.resize(base);

    float centre[3];
    for (int k = 0; k < 3; ++k)
        centre[k] = 0.5f * (m_bmin[k] + m_bmax[k]);

    const float dx = m_bmax[0] - m_bmin[0];
    const float dy = m_bmax[1] - m_bmin[1];
    const float dz = m_bmax[2] - m_bmin[2];
    const float radius = 0.5f * std::sqrt(dx * dx + dy * dy + dz * dz);

    for (int n = 0; n < m_parts.size(); ++n) {
        Part &p = m_parts[n];
        float off[3];
        for (int k = 0; k < 3; ++k)
            off[k] = 0.5f * (p.mesh.bmin[k] + p.mesh.bmax[k]) - centre[k];

        const float len = std::sqrt(off[0] * off[0] + off[1] * off[1] + off[2] * off[2]);

        // Offset alone leaves pieces near the middle almost stationary - the
        // inner ring of a cylinder, the core of a sphere. Adding a fixed step
        // along the same direction guarantees every piece clears its
        // neighbours while keeping the layout recognisably the original shape.
        if (len > 1e-6f) {
            for (int k = 0; k < 3; ++k)
                p.push[k] = off[k] + (off[k] / len) * 0.45f * radius;
        }
        else {
            // Dead centre: no direction to push along, so fan it out using the
            // part's position in the list purely to break the tie.
            const float a = 2.399963f * float(n);        // golden angle
            p.push[0] = 0.45f * radius * std::cos(a);
            p.push[1] = 0.45f * radius * std::sin(a);
            p.push[2] = 0.0f;
        }
    }
}

void MeshView::setController(AppController *c)
{
    if (m_controller == c)
        return;
    if (m_controller != nullptr)
        disconnect(m_controller, nullptr, this, nullptr);

    m_controller = c;

    if (m_controller != nullptr) {
        connect(m_controller, &AppController::meshChanged, this, [this] {
            setMesh(m_controller->mesh());
            setExplode(0.0);
            resetView();
        });
        connect(m_controller, &AppController::piecesChanged, this, [this] {
            if (m_controller->pieces().isEmpty()) {
                setMesh(m_controller->mesh());
            } else {
                setPieces(m_controller->pieces());
            }
            resetView();
        });
        setMesh(m_controller->mesh());
    }
    else {
        clearMesh();
    }
    emit controllerChanged();
}

// ---------------------------------------------------------------- view set --

void MeshView::setYaw(qreal v)
{
    if (qFuzzyCompare(m_yaw, v)) return;
    m_yaw = v; emit viewChanged(); update();
}

void MeshView::setPitch(qreal v)
{
    // Clamped so the model cannot be rolled past vertical and end up mirrored.
    v = qBound(-89.9, v, 89.9);
    if (qFuzzyCompare(m_pitch, v)) return;
    m_pitch = v; emit viewChanged(); update();
}

void MeshView::setZoom(qreal v)
{
    v = qBound(0.05, v, 40.0);
    if (qFuzzyCompare(m_zoom, v)) return;
    m_zoom = v; emit viewChanged(); update();
}

void MeshView::setExplode(qreal v)
{
    v = qBound(0.0, v, 3.0);
    if (qFuzzyCompare(m_explode, v)) return;
    m_explode = v; emit viewChanged(); update();
}

void MeshView::setShaded(bool v)
{
    if (m_shaded == v) return;
    m_shaded = v; emit viewChanged(); update();
}

void MeshView::setShowEdges(bool v)
{
    if (m_showEdges == v) return;
    m_showEdges = v; emit viewChanged(); update();
}

void MeshView::resetView()
{
    m_yaw = -35.0; m_pitch = -25.0; m_zoom = 1.0;
    emit viewChanged();
    update();
}

// -------------------------------------------------------------- projection --

void MeshView::setFixedBounds(const float bmin[3], const float bmax[3])
{
    for (int k = 0; k < 3; ++k) {
        m_fixMin[k] = bmin[k];
        m_fixMax[k] = bmax[k];
    }
    m_fixedBounds = true;
    update();
}

void MeshView::clearFixedBounds()
{
    m_fixedBounds = false;
    update();
}

bool MeshView::camera(int w, int h, Camera *c) const
{
    if (m_parts.isEmpty())
        return false;

    const float *lo = m_fixedBounds ? m_fixMin : m_bmin;
    const float *hi = m_fixedBounds ? m_fixMax : m_bmax;

    for (int k = 0; k < 3; ++k)
        c->centre[k] = 0.5f * (lo[k] + hi[k]);

    const float dx = hi[0] - lo[0];
    const float dy = hi[1] - lo[1];
    const float dz = hi[2] - lo[2];
    float radius = 0.5f * std::sqrt(dx * dx + dy * dy + dz * dz);
    if (!(radius > 1e-12f))
        radius = 1.0f;

    // Exploding pushes pieces outward, so the set needs more room on screen.
    const float fit = radius * (1.0f + float(m_explode));
    c->scale = (float(qMin(w, h)) * 0.45f * float(m_zoom)) / fit;
    c->ox    = 0.5f * float(w);
    c->oy    = 0.5f * float(h);
    orient(c);
    return true;
}

void MeshView::orient(Camera *c) const
{
    c->cy = std::cos(float(m_yaw)   * kDegToRad);
    c->sy = std::sin(float(m_yaw)   * kDegToRad);
    c->cp = std::cos(float(m_pitch) * kDegToRad);
    c->sp = std::sin(float(m_pitch) * kDegToRad);
}

// (x, y, z) is already centred and exploded.
void MeshView::project(const Camera &c, float x, float y, float z, SV *s) const
{
    // Yaw about Y, then pitch about X.
    const float x1 =  c.cy * x + c.sy * z;
    const float z1 = -c.sy * x + c.cy * z;
    const float y2 =  c.cp * y - c.sp * z1;
    const float z2 =  c.sp * y + c.cp * z1;

    s->x = c.ox + x1 * c.scale;
    s->y = c.oy - y2 * c.scale;          // screen Y grows downward
    s->z = z2 * c.scale;                 // larger z = nearer the viewer
}

void MeshView::projectVertices(int w, int h)
{
    Camera c;
    if (!camera(w, h, &c))
        return;

    const float e = float(m_explode);
    for (const Part &p : m_parts) {
        const int n = p.mesh.vertexCount();
        for (int i = 0; i < n; ++i)
            project(c,
                    p.mesh.pos[i * 3 + 0] - c.centre[0] + p.push[0] * e,
                    p.mesh.pos[i * 3 + 1] - c.centre[1] + p.push[1] * e,
                    p.mesh.pos[i * 3 + 2] - c.centre[2] + p.push[2] * e,
                    &m_proj[p.base + i]);
    }
}

QVector<QPointF> MeshView::pieceAnchors(int w, int h, QVector<bool> *valid) const
{
    int n = 0;
    for (const Part &p : m_parts)
        n = qMax(n, p.source + 1);

    QVector<QPointF> out(n);
    if (valid != nullptr)
        valid->fill(false, n);

    Camera c;
    if (!camera(w, h, &c))
        return out;

    const float e = float(m_explode);
    for (const Part &p : m_parts) {
        if (p.source < 0)
            continue;
        SV s;
        project(c,
                0.5f * (p.mesh.bmin[0] + p.mesh.bmax[0]) - c.centre[0] + p.push[0] * e,
                0.5f * (p.mesh.bmin[1] + p.mesh.bmax[1]) - c.centre[1] + p.push[1] * e,
                0.5f * (p.mesh.bmin[2] + p.mesh.bmax[2]) - c.centre[2] + p.push[2] * e,
                &s);
        out[p.source] = QPointF(qreal(s.x), qreal(s.y));
        if (valid != nullptr)
            (*valid)[p.source] = true;
    }
    return out;
}

QPointF MeshView::axisOnScreen(int axis) const
{
    Camera c;
    c.scale = 1.0f;
    c.ox = c.oy = 0.0f;
    orient(&c);

    SV s;
    project(c, axis == 0 ? 1.0f : 0.0f,
               axis == 1 ? 1.0f : 0.0f,
               axis == 2 ? 1.0f : 0.0f, &s);
    return QPointF(qreal(s.x), qreal(s.y));
}

// ------------------------------------------------------------- rasterising --

void MeshView::rasterise(int w, int h)
{
    if (m_frame.width() != w || m_frame.height() != h)
        m_frame = QImage(w, h, QImage::Format_RGB32);
    m_frame.fill(kBackground);

    // Sized and cleared before the early-out: a curve-only file has no
    // triangles, but the edge pass still depth-tests against this buffer.
    m_zbuf.resize(w * h);
    m_zbuf.fill(-std::numeric_limits<float>::max());

    // Rotate the face normal the same way the vertices were rotated, then use
    // its view-space Z as a headlight term. abs() gives two-sided shading, so
    // open surfaces (a lone trimmed patch, the cut face of a puzzle piece)
    // stay lit from either side instead of going black.
    const float cy = std::cos(float(m_yaw)   * kDegToRad);
    const float sy = std::sin(float(m_yaw)   * kDegToRad);
    const float cp = std::cos(float(m_pitch) * kDegToRad);
    const float sp = std::sin(float(m_pitch) * kDegToRad);

    for (const Part &part : m_parts) {
        const MeshData &m = part.mesh;
        const int nTri = m.triangleCount();

        const float tintR = float(qRed(part.tint));
        const float tintG = float(qGreen(part.tint));
        const float tintB = float(qBlue(part.tint));

        for (int t = 0; t < nTri; ++t) {
            const SV &a = m_proj[part.base + m.tris[t * 3 + 0]];
            const SV &b = m_proj[part.base + m.tris[t * 3 + 1]];
            const SV &c = m_proj[part.base + m.tris[t * 3 + 2]];

            const float area = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
            if (!(std::fabs(area) >= 1e-7f))
                continue;     // degenerate on screen - and NaN fails this too,
                              // which keeps a corrupt file out of the int casts
                              // below, where it would be undefined behaviour.

            int minX = int(std::floor(qMin(a.x, qMin(b.x, c.x))));
            int maxX = int(std::ceil (qMax(a.x, qMax(b.x, c.x))));
            int minY = int(std::floor(qMin(a.y, qMin(b.y, c.y))));
            int maxY = int(std::ceil (qMax(a.y, qMax(b.y, c.y))));
            minX = qMax(minX, 0); maxX = qMin(maxX, w - 1);
            minY = qMax(minY, 0); maxY = qMin(maxY, h - 1);
            if (minX > maxX || minY > maxY)
                continue;

            const float nx = m.triNrm[t * 3 + 0];
            const float ny = m.triNrm[t * 3 + 1];
            const float nz = m.triNrm[t * 3 + 2];
            const float rz1   = -sy * nx + cy * nz;
            const float viewZ =  sp * ny + cp * rz1;

            const float lambert = 0.30f + 0.70f * std::fabs(viewZ);
            const QRgb colour = qRgb(int(qBound(0.0f, lambert * tintR, 255.0f)),
                                     int(qBound(0.0f, lambert * tintG, 255.0f)),
                                     int(qBound(0.0f, lambert * tintB, 255.0f)));

            const float invArea = 1.0f / area;

            for (int y = minY; y <= maxY; ++y) {
                QRgb  *row = reinterpret_cast<QRgb *>(m_frame.scanLine(y));
                float *zr  = m_zbuf.data() + y * w;
                const float py = float(y) + 0.5f;

                for (int x = minX; x <= maxX; ++x) {
                    const float px = float(x) + 0.5f;

                    // Each weight comes from the edge opposite its vertex.
                    // Dividing by the *signed* area normalises the winding away,
                    // so "all three >= 0" means inside for front- and back-facing
                    // triangles alike - open surfaces stay solid, no culling.
                    const float wa = ((c.x - b.x) * (py - b.y) - (c.y - b.y) * (px - b.x)) * invArea;
                    const float wb = ((a.x - c.x) * (py - c.y) - (a.y - c.y) * (px - c.x)) * invArea;
                    const float wc = ((b.x - a.x) * (py - a.y) - (b.y - a.y) * (px - a.x)) * invArea;
                    if (wa < 0.0f || wb < 0.0f || wc < 0.0f)
                        continue;

                    const float z = wa * a.z + wb * b.z + wc * c.z;
                    if (z <= zr[x])
                        continue;
                    zr[x]  = z;
                    row[x] = colour;
                }
            }
        }
    }
}

// Draws the edge lists into the shaded frame with a depth test against the same
// z-buffer, so edges on hidden faces stay hidden. Without this the wireframe
// shows straight through the model - which makes an exploded assembly
// unreadable.
void MeshView::drawEdgesDepthTested(int w, int h)
{
    int nEdgeTotal = 0;
    for (const Part &p : m_parts)
        nEdgeTotal += p.mesh.edgeCount();
    if (nEdgeTotal == 0)
        return;

    // Once there are more edges than roughly one per 8 pixels, every triangle
    // is a couple of pixels across and the wireframe stops describing the
    // surface - it just darkens it. Drop the overlay rather than render mush;
    // the shading still shows the form, and unshaded mode still draws them all.
    if (nEdgeTotal > (w * h) / 8)
        return;

    // Edges sit exactly on the triangles they bound, so they need a bias to
    // win the depth comparison against their own faces.
    const float bias = 0.75f;

    // Two colours, picked per pixel: a dark line reads as a crease on top of a
    // lit surface, but would be invisible against the background. Anything
    // still at the z-buffer's clear value has no surface behind it - a
    // silhouette edge, or a model made only of curves - and gets the light pen.
    const float clear = -std::numeric_limits<float>::max();
    const int   onSurfR = 0x1c, onSurfG = 0x20, onSurfB = 0x26;
    const int   onBackR = 0x9f, onBackG = 0xc4, onBackB = 0xdc;
    const float alpha = 0.55f;

    const int step = nEdgeTotal > kMaxDrawEdges ? (nEdgeTotal / kMaxDrawEdges) + 1 : 1;

    for (const Part &part : m_parts) {
        const MeshData &m = part.mesh;
        const int nEdge = m.edgeCount();

        for (int e = 0; e < nEdge; e += step) {
            const SV &p = m_proj[part.base + m.edges[e * 2 + 0]];
            const SV &q = m_proj[part.base + m.edges[e * 2 + 1]];

            const float dx = q.x - p.x, dy = q.y - p.y;
            const int   n  = int(std::ceil(qMax(std::fabs(dx), std::fabs(dy))));
            if (n <= 0 || n > 8192)         // nothing, or absurdly long
                continue;

            const float sx = dx / float(n), sy = dy / float(n);
            const float sz = (q.z - p.z) / float(n);

            float x = p.x, y = p.y, z = p.z;
            for (int i = 0; i <= n; ++i, x += sx, y += sy, z += sz) {
                const int xi = int(x), yi = int(y);
                if (xi < 0 || xi >= w || yi < 0 || yi >= h)
                    continue;
                const float depth = m_zbuf[yi * w + xi];
                if (z + bias < depth)
                    continue;                    // behind a nearer surface

                const bool bare = (depth == clear);
                const int  r = bare ? onBackR : onSurfR;
                const int  g = bare ? onBackG : onSurfG;
                const int  b = bare ? onBackB : onSurfB;

                QRgb *px = reinterpret_cast<QRgb *>(m_frame.scanLine(yi)) + xi;
                const QRgb d = *px;
                *px = qRgb(int(qRed(d)   * (1.0f - alpha) + r * alpha),
                           int(qGreen(d) * (1.0f - alpha) + g * alpha),
                           int(qBlue(d)  * (1.0f - alpha) + b * alpha));
            }
        }
    }
}

// ------------------------------------------------------------------- paint --

void MeshView::paint(QPainter *painter)
{
    const int w = int(width());
    const int h = int(height());
    if (w <= 0 || h <= 0)
        return;

    if (m_parts.isEmpty()) {
        painter->fillRect(0, 0, w, h, QColor(kBackground));
        painter->setPen(QColor(0x77, 0x7f, 0x8a));
        painter->drawText(QRectF(0, 0, w, h), Qt::AlignCenter,
                          QStringLiteral("No model loaded.\nUse “Open model…” below."));
        return;
    }

    projectVertices(w, h);

    if (m_shaded) {
        rasterise(w, h);
        // Edges go into the same image so they can be depth-tested against the
        // surface that was just drawn.
        if (m_showEdges)
            drawEdgesDepthTested(w, h);
        painter->drawImage(0, 0, m_frame);
        return;
    }

    painter->fillRect(0, 0, w, h, QColor(kBackground));
    if (!m_showEdges)
        return;

    // Unshaded: a see-through wireframe with every edge visible. There is no
    // depth buffer to test against here, and seeing the far side is the point.
    painter->setRenderHint(QPainter::Antialiasing, true);

    int nEdgeTotal = 0;
    for (const Part &p : m_parts)
        nEdgeTotal += p.mesh.edgeCount();
    const int step = nEdgeTotal > kMaxDrawEdges ? (nEdgeTotal / kMaxDrawEdges) + 1 : 1;

    for (const Part &part : m_parts) {
        const MeshData &m = part.mesh;
        painter->setPen(QPen(m_parts.size() > 1 ? QColor(part.tint)
                                                : QColor(0x9f, 0xc4, 0xdc), 1.0));
        QVector<QLineF> lines;
        const int nEdge = m.edgeCount();
        lines.reserve(nEdge / step + 1);
        for (int e = 0; e < nEdge; e += step) {
            const SV &p = m_proj[part.base + m.edges[e * 2 + 0]];
            const SV &q = m_proj[part.base + m.edges[e * 2 + 1]];
            lines.append(QLineF(qreal(p.x), qreal(p.y), qreal(q.x), qreal(q.y)));
        }
        painter->drawLines(lines);
    }
}

// ------------------------------------------------------------ interaction --

void MeshView::mousePressEvent(QMouseEvent *e)
{
    m_lastMouse = e->position();
    e->accept();
}

void MeshView::mouseMoveEvent(QMouseEvent *e)
{
    const QPointF d = e->position() - m_lastMouse;
    m_lastMouse = e->position();
    setYaw(m_yaw + d.x() * 0.4);
    setPitch(m_pitch + d.y() * 0.4);
    e->accept();
}

void MeshView::wheelEvent(QWheelEvent *e)
{
    const int notches = e->angleDelta().y();
    if (notches != 0)
        setZoom(m_zoom * std::pow(1.0015, double(notches)));
    e->accept();
}
