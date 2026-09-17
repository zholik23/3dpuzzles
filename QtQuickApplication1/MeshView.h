#pragma once
//
// MeshView - draws MeshData parts with correct occlusion using nothing but
// QtQuick. Qt Quick 3D is not installed here, and a painter's-algorithm wireframe
// cannot survive an exploded assembly, so triangles are rasterised into a QImage
// with a real z-buffer.
//
// Everything is a list of parts, so a loaded model and a divided puzzle take the
// same path. Projection is orthographic on purpose: no near plane means no
// clipping code and no way for a model to vanish at an awkward scale.
//
#include "AppController.h"
#include "MeshData.h"
#include "PuzzleDivider.h"

#include <QColor>
#include <QImage>
#include <QPointF>
#include <QQuickPaintedItem>
#include <QVector>

class MeshView : public QQuickPaintedItem {
    Q_OBJECT
    Q_PROPERTY(qreal yaw       READ yaw       WRITE setYaw       NOTIFY viewChanged)
    Q_PROPERTY(qreal pitch     READ pitch     WRITE setPitch     NOTIFY viewChanged)
    Q_PROPERTY(qreal zoom      READ zoom      WRITE setZoom      NOTIFY viewChanged)
    Q_PROPERTY(qreal explode   READ explode   WRITE setExplode   NOTIFY viewChanged)
    Q_PROPERTY(bool  shaded    READ shaded    WRITE setShaded    NOTIFY viewChanged)
    Q_PROPERTY(bool  showEdges READ showEdges WRITE setShowEdges NOTIFY viewChanged)
    Q_PROPERTY(bool  hasMesh   READ hasMesh                      NOTIFY meshChanged)
    Q_PROPERTY(int   partCount READ partCount                    NOTIFY meshChanged)
    Q_PROPERTY(AppController *controller READ controller WRITE setController
                                                     NOTIFY controllerChanged)

public:
    explicit MeshView(QQuickItem *parent = nullptr);

    void setFixedBounds(const float bmin[3], const float bmax[3]);
    void clearFixedBounds();

    void setMesh(const MeshData &mesh);
    void setPieces(const QVector<PuzzlePiece> &pieces);
    void clearMesh();

    static QRgb tintFor(const PuzzlePiece &piece);
    static QRgb background();

    QVector<QPointF> pieceAnchors(int w, int h, QVector<bool> *valid) const;

    QPointF axisOnScreen(int axis) const;

    AppController *controller() const { return m_controller; }
    void setController(AppController *c);

    qreal yaw()       const { return m_yaw; }
    qreal pitch()     const { return m_pitch; }
    qreal zoom()      const { return m_zoom; }
    qreal explode()   const { return m_explode; }
    bool  shaded()    const { return m_shaded; }
    bool  showEdges() const { return m_showEdges; }
    bool  hasMesh()   const { return !m_parts.isEmpty(); }
    int   partCount() const { return m_parts.size(); }

    void setYaw(qreal v);
    void setPitch(qreal v);
    void setZoom(qreal v);
    void setExplode(qreal v);
    void setShaded(bool v);
    void setShowEdges(bool v);

    Q_INVOKABLE void resetView();

    void paint(QPainter *painter) override;

signals:
    void viewChanged();
    void meshChanged();
    void controllerChanged();

protected:
    void mousePressEvent(QMouseEvent *e) override;
    void mouseMoveEvent(QMouseEvent *e) override;
    void wheelEvent(QWheelEvent *e) override;

private:
    struct SV { float x, y, z; };

    struct Camera {
        float centre[3];
        float scale, ox, oy;
        float cy, sy, cp, sp;
    };

    struct Part {
        MeshData mesh;
        float    push[3] = { 0, 0, 0 };
        QRgb     tint    = 0;
        int      base    = 0;
        int      source  = -1;
    };

    void rebuildParts();
    void projectVertices(int w, int h);
    bool camera(int w, int h, Camera *c) const;
    void orient(Camera *c) const;
    void project(const Camera &c, float x, float y, float z, SV *s) const;
    void rasterise(int w, int h);
    void drawEdgesDepthTested(int w, int h);

    QVector<Part>   m_parts;
    QVector<SV>     m_proj;
    QVector<float>  m_zbuf;
    QImage          m_frame;

    float m_bmin[3] = { 0, 0, 0 };
    float m_bmax[3] = { 0, 0, 0 };

    qreal m_yaw     = -35.0;
    qreal m_pitch   = -25.0;
    qreal m_zoom    = 1.0;
    qreal m_explode = 0.0;
    bool   m_fixedBounds = false;
    float  m_fixMin[3] = { 0, 0, 0 };
    float  m_fixMax[3] = { 0, 0, 0 };
    bool  m_shaded    = true;
    bool  m_showEdges = false;

    QPointF        m_lastMouse;
    AppController *m_controller = nullptr;
};
