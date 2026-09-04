#pragma once
//
// AppController - the app-level model. Owns the loaded mesh, the trivariate
// that stands in for it, and the pieces the divider produced.
//
// The mesh and the trivariate are kept side by side on purpose: the mesh is the
// reference the trivariate fit will eventually be checked against, so neither
// replaces the other.
//
#include "CutWarp.h"
#include "IritJoint.h"
#include "MeshData.h"
#include "MeshDivider.h"
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

    // True when Divide will clip the mesh rather than region-extract a
    // trivariate - i.e. no V-rep has been chosen. Lets the UI say which.
    Q_PROPERTY(bool    dividesMesh   READ dividesMesh   NOTIFY trivariateChanged)
    Q_PROPERTY(bool    hasTrivariate READ hasTrivariate NOTIFY trivariateChanged)
    Q_PROPERTY(QString trivariateName READ trivariateName NOTIFY trivariateChanged)
    Q_PROPERTY(QString trivariateInfo READ trivariateInfo NOTIFY trivariateChanged)

    // How the cuts are shaped. Flat is the default; wave and grid bend them.
    Q_PROPERTY(QString cutShapeInfo READ cutShapeInfo NOTIFY cutShapeChanged)

    // Joints. When on, every shared face gets Elber's pin/hole pair cut into
    // the two pieces by an IRIT boolean, straight after the division runs.
    Q_PROPERTY(bool    addJoints  READ addJoints  WRITE setAddJoints NOTIFY jointsChanged)
    Q_PROPERTY(QString jointNote  READ jointNote  NOTIFY piecesChanged)

    Q_PROPERTY(int     pieceCount  READ pieceCount  NOTIFY piecesChanged)
    Q_PROPERTY(QString divisionInfo READ divisionInfo NOTIFY piecesChanged)

public:
    explicit AppController(QObject *parent = nullptr);

    // --- loading ---------------------------------------------------------
    Q_INVOKABLE void loadFile(const QUrl &url);
    Q_INVOKABLE void loadPath(const QString &path);

    // --- choosing what to divide -----------------------------------------
    // Any trivariate stored in the file that is currently loaded.
    Q_INVOKABLE void useTrivariateFromFile();
    // A known shape, for developing the divider without a fit.
    Q_INVOKABLE void usePrimitive(const QString &kind);
    // A box over the loaded mesh's extent. Stands in for Increment 3: it has
    // the model's size but not its shape.
    Q_INVOKABLE void useBoundingCage();
    Q_INVOKABLE void showWholeModel();          // back to the undivided mesh

    // --- dividing --------------------------------------------------------
    //
    // These divide whatever is currently the subject: a trivariate if one has
    // been chosen (Elber's method, real solid pieces), otherwise the loaded
    // mesh (clipped pieces that keep the model's true shape). Which one ran is
    // reported in divisionInfo.
    // Every piece the same size and shape, cut by straight planes.
    Q_INVOKABLE void divideUniform(int nu, int nv, int nw);

    // Irregular: cut positions pushed off the grid so no two pieces are the
    // same size, and - when `curvePercent` is above zero - the cuts themselves
    // bent into smooth irregular surfaces rather than planes. One seed drives
    // both, so the same seed always reproduces the same puzzle.
    // `pieces` is a target count, not a grid: the domain is split recursively,
    // one cell at a time, so the sizes are independent of each other and the
    // adjacency comes out irregular. Global cut planes cannot do that.
    Q_INVOKABLE void divideRandom(int pieces, double curvePercent, int seed);

    // Non-uniform by physical size: cuts land where the accumulated real-world
    // extent reaches `maxSizeMM`, so pieces come out roughly equal in actual
    // size rather than in parameter space. `maxPerAxis` is a safety stop - a
    // small limit on a big model can otherwise ask for thousands of pieces.
    Q_INVOKABLE void divideBySize(double maxSizeMM, int maxPerAxis);

    // --- joints ----------------------------------------------------------
    void setAddJoints(bool on);

    // --- read side -------------------------------------------------------
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
    // The mesh the cuts are actually applied to: the model, warped if the cut
    // shape asks for it. Division planes have to be measured on this, not on
    // the original, because the warp moves the bounds.
    MeshData workingMesh() const;
    void runMeshDivision(const MeshDivisionSpec &spec, const MeshData &work);
    void runCellDivision(const QVector<CellBox> &cells, const MeshData &work,
                         const QString &note);
    void describePieces(const QString &note, int gridCells, const QString &warning);
    // Cuts the pin/hole pairs into m_pieces. Runs after a division, before the
    // pieces are described, so the reported counts are of the jointed result.
    void applyJoints();

    // Dumps the chosen cut positions and the resulting piece sizes to the debug
    // output, so an even division can be told from an uneven one by reading.
    void logDivision(const QString &what, const double domain[6],
                     const QVector<double> cuts[3]) const;
    void logCells(const QString &what, const QVector<CellBox> &cells) const;
    void logPieceSizes() const;

    MeshData             m_mesh;
    CutWarp              m_warp;
    Trivariate           m_triv;
    QVector<PuzzlePiece> m_pieces;
    QString              m_loadedPath;

    QString m_fileName;
    QString m_status   = QStringLiteral("No model loaded");
    QString m_detail;
    QString m_trivInfo;
    QString m_divisionInfo;
    bool    m_hasError = false;

    JointParams m_joint;
    QString     m_jointNote;
    // On by default: dividing a model is meant to produce jointed pieces.
    bool        m_addJoints = true;

    // IRIT's tessellation fineness (higher = more polygons). Pieces get a
    // lower setting than the whole model: each covers a fraction of the domain
    // but is tessellated to the same density, so N pieces at the model's
    // setting would cost roughly N times the model's polygon count.
    static constexpr double kModelFineNess = 20.0;
    static constexpr double kPieceFineNess = 12.0;
};
