#pragma once
//
// AppController - the app-level model: the loaded mesh, the trivariate that
// stands in for it, and the pieces the divider produced. Mesh and trivariate are
// kept side by side because neither replaces the other.
//
#include "CageBoolean.h"
#include "DivisionReport.h"
#include "CutWarp.h"
#include "IritJoint.h"
#include "MeshData.h"
#include "MeshDivider.h"
#include "PieceExport.h"
#include "PuzzleDivider.h"
#include "Trivariate.h"

#include <QObject>
#include <QString>
#include <QStringList>
#include <QUrl>
#include <QVector>

class AppController : public QObject {
    Q_OBJECT
    Q_PROPERTY(QString status      READ status      NOTIFY statusChanged)
    Q_PROPERTY(QString detail      READ detail      NOTIFY statusChanged)
    Q_PROPERTY(bool    hasError    READ hasError    NOTIFY statusChanged)
    Q_PROPERTY(bool    hasMesh     READ hasMesh     NOTIFY meshChanged)
    Q_PROPERTY(QString fileName    READ fileName    NOTIFY meshChanged)
    Q_PROPERTY(QStringList nameFilters     READ nameFilters     CONSTANT)
    Q_PROPERTY(QStringList primitiveKinds  READ primitiveKinds  CONSTANT)

    Q_PROPERTY(bool    dividesMesh   READ dividesMesh   NOTIFY trivariateChanged)
    Q_PROPERTY(bool    hasTrivariate READ hasTrivariate NOTIFY trivariateChanged)
    Q_PROPERTY(QString trivariateName READ trivariateName NOTIFY trivariateChanged)
    Q_PROPERTY(QString trivariateInfo READ trivariateInfo NOTIFY trivariateChanged)

    Q_PROPERTY(QString cutShapeInfo READ cutShapeInfo NOTIFY cutShapeChanged)

    Q_PROPERTY(bool    addJoints  READ addJoints  WRITE setAddJoints NOTIFY jointsChanged)
    Q_PROPERTY(QString jointNote  READ jointNote  NOTIFY piecesChanged)

    Q_PROPERTY(int     pieceCount  READ pieceCount  NOTIFY piecesChanged)
    Q_PROPERTY(bool    canSave     READ canSave     NOTIFY piecesChanged)
    Q_PROPERTY(QStringList saveFilters READ saveFilters CONSTANT)
    Q_PROPERTY(QString divisionInfo READ divisionInfo NOTIFY piecesChanged)

    Q_PROPERTY(QStringList planFigures   READ planFigures   NOTIFY piecesChanged)
    Q_PROPERTY(QString     planFolderUrl READ planFolderUrl NOTIFY piecesChanged)
    Q_PROPERTY(QString     reportUrl     READ reportUrl     NOTIFY piecesChanged)

public:
    explicit AppController(QObject *parent = nullptr);

    Q_INVOKABLE void loadFile(const QUrl &url);
    Q_INVOKABLE void loadPath(const QString &path);

    Q_INVOKABLE void useTrivariateFromFile();
    Q_INVOKABLE void usePrimitive(const QString &kind);
    Q_INVOKABLE void useBoundingCage();
    Q_INVOKABLE void showWholeModel();

    Q_INVOKABLE void savePieces(const QUrl &url, bool separateFiles, bool spread);

    Q_INVOKABLE void divideUniform(int nu, int nv, int nw);

    Q_INVOKABLE void divideRandom(int pieces);

    Q_INVOKABLE void newLayout();

    Q_INVOKABLE void divideBySize(double maxSizeMM, int maxPerAxis);

    void setAddJoints(bool on);

    const MeshData &mesh()   const { return m_mesh; }
    const QVector<PuzzlePiece> &pieces() const { return m_pieces; }

    QString status()   const { return m_status; }
    QString detail()   const { return m_detail; }
    bool    hasError() const { return m_hasError; }
    bool    hasMesh()  const { return !m_mesh.isEmpty(); }
    QString fileName() const { return m_fileName; }

    bool    dividesMesh()   const { return !m_triv.isValid(); }
    bool    hasTrivariate() const { return m_triv.isValid(); }
    QString trivariateName() const { return m_triv.label(); }
    QString trivariateInfo() const { return m_trivInfo; }

    QString cutShapeInfo() const { return m_warp.describe(); }
    int     pieceCount()   const { return m_pieces.size(); }
    QString divisionInfo() const { return m_divisionInfo; }
    QStringList planFigures()   const { return m_planFigures; }
    QString     planFolderUrl() const { return m_planFolderUrl; }
    QString     reportUrl()     const { return m_reportUrl; }
    bool    canSave()      const { return !m_pieces.isEmpty(); }
    QStringList saveFilters() const { return PieceExport::nameFilters(); }

    bool    addJoints()  const { return m_addJoints; }
    QString jointNote()  const { return m_jointNote; }

    QStringList nameFilters() const;
    QStringList primitiveKinds() const;

signals:
    void statusChanged();
    void meshChanged();
    void trivariateChanged();
    void piecesChanged();
    void cutShapeChanged();
    void jointsChanged();

private:
    void setError(const QString &msg);
    void adoptTrivariate(Trivariate tv, const QString &sourceDesc);
    void runDivision(const DivisionSpec &spec);
    MeshData workingMesh() const;
    void runMeshDivision(const MeshDivisionSpec &spec, const MeshData &work);
    void runCellDivision(const QVector<CellBox> &cells, const MeshData &work,
                         const QString &note);
    void runTrivCellDivision(const QVector<CellBox> &cells, const QString &note);
    void describePieces(const QString &note, int gridCells, const QString &warning);
    void applyJoints();
    void cutCellDovetails();
    void trimPiecesToModel();
    void repeatLastDivision();
    void planAndDrawFigures();
    void writeDivisionReport();

    void logDivision(const QString &what, const double domain[6],
                     const QVector<double> cuts[3]) const;
    void logCells(const QString &what, const QVector<CellBox> &cells) const;
    void logPieceSizes() const;

    MeshData             m_mesh;
    MeshData             m_sourceMesh;
    CutWarp              m_warp;
    quint32              m_layoutSeed = 7;
    Trivariate           m_triv;
    QVector<PuzzlePiece> m_pieces;
    QString              m_loadedPath;

    QString m_fileName;
    QString m_status   = QStringLiteral("No model loaded");
    QString m_detail;
    QString m_trivInfo;
    QString m_divisionInfo;
    QString m_booleanNote;
    QString m_figureNote;
    QStringList m_planFigures;
    QString m_planFolderUrl;
    QString m_reportUrl;
    DivisionStats m_stats;
    int     m_figureVersion = 0;
    bool    m_hasError = false;

    JointParams m_joint;
    QString     m_jointNote;
    QString     m_cellJointNote;   // set when the CUT itself carried the joint
    bool        m_addJoints = false;

    // The cells before any dovetail was cut, and whether any was: a dovetailed
    // cut can cost a trim later with nothing at the cut stage to predict it, so
    // trimPiecesToModel() can put these back and trim flat instead.
    QVector<PuzzlePiece> m_preJointPieces;
    bool                 m_cellsDovetailed = false;

    QVector<CellBox> m_worldCells;   // TEMPORARY - Stage 0 comparison

    // The last division, so toggling Joints can repeat it: joints are cut by a
    // boolean during a division, so the pieces on screen cannot be patched in
    // place. The layout seed is untouched, so the same cells return.
    enum class LastDivision { None, Uniform, Random, BySize };
    LastDivision m_lastKind      = LastDivision::None;
    int          m_lastCounts[3] = { 3, 3, 2 };
    int          m_lastPieces    = 24;
    double       m_lastMaxSize   = 60.0;
    int          m_lastMaxAxis   = 16;
    bool         m_reDividing    = false;

    static constexpr double kModelFineNess = 20.0;
    static constexpr double kPieceFineNess = 12.0;
};
