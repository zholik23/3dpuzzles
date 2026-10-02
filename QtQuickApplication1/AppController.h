#pragma once
//
// AppController - the app-level model: the loaded mesh, the trivariate that
// stands in for it, and the pieces the divider produced. Mesh and trivariate are
// kept side by side because neither replaces the other.
//
#include "CageBoolean.h"
#include "DivisionReport.h"
#include "HarmonicFit.h"
#include "CutInD.h"
#include "CutWarp.h"
#include "IritJoint.h"
#include "MeshData.h"
#include "MeshDivider.h"
#include "PieceExport.h"
#include "PuzzleDivider.h"
#include "Trivariate.h"

#include <QFutureWatcher>
#include <QObject>
#include <QString>
#include <QStringList>
#include <QUrl>
#include <QVector>

#include <vector>

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
    Q_PROPERTY(bool    fitting        READ fitting        NOTIFY fittingChanged)
    // The interlocking check of Cut-in-D pieces (runs on a worker thread).
    Q_PROPERTY(bool    checking       READ checking       NOTIFY checkingChanged)
    // The limb split runs on a worker thread (search + trial fits).
    Q_PROPERTY(bool    splitting      READ splitting      NOTIFY fittingChanged)
    Q_PROPERTY(bool    canCheckInterlocking READ canCheckInterlocking NOTIFY piecesChanged)

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

    // Fits a trivariate B-spline to the loaded mesh (harmonic volumetric
    // parameterization, Martin, Cohen & Kirby 2009) on a worker thread, saves it
    // as tv_<model>.itd next to the model and adopts it, so Random divides the
    // model in its parameter domain D. detail 0 = normal (24 x 24 control
    // points; tests: bimba det J > 0 everywhere, spot 5 folded samples), 1 = fine
    // (48 x 64, about half the distance to the model, 12-21 folded samples).
    Q_INVOKABLE void fitTrivariate(int detail = 0);

    // Morse split of the loaded mesh into tube-like parts (body, legs, head,
    // ...), shown as pieces. Step 1 of fitting a branching model: each part
    // gets its own trivariate next.
    Q_INVOKABLE void splitLimbs();

    // Interlocking of the Cut-in-D pieces: every exact cut is sampled in D, the
    // two pieces meeting at each sample are found, the contact normal is
    // n' ~ J^-T n_D, and the flat joint caps between limbs add theirs; then the
    // translational directional blocking graph - which pieces and groups can
    // slide out, single key, level k, one way to take it apart. Writes
    // <model>_interlocking.md next to the model.
    Q_INVOKABLE void checkInterlocking();

    // Experimental: fit every part so its block encloses the part and trim
    // pieces by it (Elber section 5). Off by default - see fitTrivariate.
    void setEncloseTrim(bool on) { m_encloseTrim = on; }

    // After Split limbs, Fit trivariate fits every part (its u = 0 face on its
    // largest cap) and Random divides all the blocks.
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
    bool    fitting()        const { return m_fitting; }
    bool    checking()       const { return m_checking; }
    bool    splitting()      const { return m_splitting; }
    bool    canCheckInterlocking() const
    {
        return m_blocksEnclose && m_pieces.size() >= 2 && !m_vrepCuts.isEmpty();
    }

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
    void fittingChanged();
    void checkingChanged();

private:
    void setError(const QString &msg);
    void adoptTrivariate(Trivariate tv, const QString &sourceDesc);
    void runDivision(const DivisionSpec &spec);
    MeshData workingMesh() const;
    void runMeshDivision(const MeshDivisionSpec &spec, const MeshData &work);
    void runCellDivision(const QVector<CellBox> &cells, const MeshData &work,
                         const QString &note);
    void runTrivCellDivision(const QVector<CellBox> &cells, const QString &note);
    void runBspInD(int pieces);
    void finishFit();
    void finishInterlocking();
    void finishSplit();
    // Shows a split's parts as pieces and makes them the parts to fit.
    void adoptSplit(const QVector<HarmonicFit::Part> &parts, const QStringList &notes, const QString &rank);
    MeshData jointCapTriangles() const;   // the joint caps of the limb split, as triangles
    void finishPartsFit(const QVector<HarmonicFit::Result> &all);
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

    // True when m_triv came from a file (a real B-spline trivariate such as
    // Elber's tvs_*.itd), false for the bounding cage or a primitive. Only a
    // real trivariate gets the BSP in D: on the box cage every cut in D would
    // stay flat in R^3.
    bool    m_trivReal = false;

    // The harmonic fit, running on a worker thread; m_fitPath is the model it
    // was started for, so a result that arrives after another load is dropped.
    QFutureWatcher<QVector<HarmonicFit::Result>> m_fitWatcher;
    QFutureWatcher<QStringList> m_lockWatcher;   // status line, detail, report path
    // The limb split: the search's shortlist, each split re-ranked by a trial
    // fit of its parts; kept in that order, so a full fit that folds can fall
    // back to the next one.
    struct SplitRun {
        QVector<QVector<HarmonicFit::Part>> alts;
        QVector<HarmonicFit::SplitHealth>   health;
        QVector<QStringList>                notes;
        QString                             error;
        qint64                              ms = 0;
    };
    QFutureWatcher<SplitRun>            m_splitWatcher;
    bool                                m_splitting = false;
    bool                                m_fitAfterSplit = false;
    int                                 m_fitDetail = 0;
    QVector<QVector<HarmonicFit::Part>> m_splitAlts;      // best first
    QVector<QStringList>                m_splitNotes;
    QStringList                         m_splitRank;      // one line per split
    int                                 m_splitAlt = 0;   // the one in m_parts
    bool                        m_checking = false;
    bool    m_fitting = false;
    QString m_fitPath;

    // The BSP-in-D cells behind m_pieces when they are exact sub-trivariates of
    // m_triv (V-rep pieces); saving to .itd then also writes them as
    // trivariates. Cleared by anything that replaces the pieces another way.
    QVector<CellBox> m_vrepCells;
    QVector<int>     m_vrepBlockOf;     // per cell: which of m_blocks it is in (empty: m_triv)
    QVector<int>     m_vrepPieceOf;     // per cell: the piece it belongs to (a glued piece has several)
    // Cut in D: every cut, exact (its surface in its block's D), the block it
    // is in, and per piece the cuts bounding it with its side of each.
    QVector<CutInD::Cut>              m_vrepCuts;
    QVector<int>                      m_vrepCutBlock;
    QVector<QVector<QPair<int, int>>> m_vrepPieceCuts;

    // The limb split of the loaded mesh (Split limbs), and after Fit
    // trivariate one trivariate per part: a multi-block V-rep. m_triv then
    // holds a copy of the largest block, for everything that expects one.
    QVector<HarmonicFit::Part> m_parts;
    std::vector<Trivariate>    m_blocks;
    QStringList                m_blockNames;
    QVector<double>            m_blockVol;
    QVector<int>               m_blockPart;     // block -> index into m_parts (its trimming surface)
    QVector<HarmonicFit::Result> m_blockFits;   // block -> its control net (for cutting its part in D)
    bool                       m_blocksEnclose = false;   // blocks enclose their parts: trim pieces by them
    bool                       m_encloseTrim = false;
    // Curved cuts in D for the fitted path. Fixed for now; to be chosen
    // automatically (simulated annealing on the blocking analysis), not by hand.
    static constexpr double    kCutBend = 0.25;
    static constexpr int       kCutWaves = 1;
    bool                       m_curvedCuts = false;
    // Divide cuts the whole model (CutInD::cutWhole): pieces cross the limb
    // split's joints, which only supplies the coordinates. Off: each part is
    // cut on its own and small parts are glued on (the split shows).
    static constexpr bool      kWholeCut = true;
    bool                       m_wholeCutUsed = false;  // the pieces came from cutWhole
    bool                       m_fitParts = false;

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

    // Tessellation fineness for a trivariate: the fixed constants are enough
    // for Elber's small control meshes, but a fitted trivariate (24 x 24 and
    // more control points) drawn at 20 shows big flat facets - it looked less
    // like the model than it is. Scales with the control mesh.
    double fineNessFor(double base, double perControlPoint = 3.0) const;
};
