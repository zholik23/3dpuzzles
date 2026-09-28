#pragma once
//
// PuzzleAnalyzer - the "Analyse puzzle" window: open a saved trivariate (the
// tvs_*.itd files from puz_vol - Copy.irt), divide its parameter domain, and
// answer the experiment's questions on screen instead of on the command line:
//
//   - is the map valid (det J > 0, no fold)?
//   - interlocking in R^3: is there a single key, and to what level k?
//   - how does it come apart - every opening, and one full disassembly
//   - the shape criteria: minimum thickness, one component, uniform sizes
//
// Each answer comes with pieces to look at: the group that escapes is coloured
// and can be slid out along its own direction, and the whole disassembly can be
// played step by step. "Verify by sliding" re-checks a DBG verdict by actually
// moving the pieces (DbgAnalysis::sweep), which uses no normals at all.
//
// The DBG and the criteria run on a worker thread, so the window stays live.
// While one runs, nothing else in the analyser touches IRIT.
//
#include "DbgAnalysis.h"
#include "PuzzleDivider.h"
#include "Trivariate.h"

#include <QFutureWatcher>
#include <QObject>
#include <QString>
#include <QStringList>
#include <QUrl>
#include <QVariantList>
#include <QVector>

class PuzzleAnalyzer : public QObject {
    Q_OBJECT
    Q_PROPERTY(QString     fileName  READ fileName  NOTIFY fileChanged)
    Q_PROPERTY(QString     folder    READ folder    NOTIFY fileChanged)
    Q_PROPERTY(QStringList siblings  READ siblings  NOTIFY fileChanged)
    Q_PROPERTY(bool        busy      READ busy      NOTIFY busyChanged)
    Q_PROPERTY(QString     status    READ status    NOTIFY statusChanged)
    Q_PROPERTY(bool        hasResult READ hasResult NOTIFY resultChanged)
    // false = too many pieces to test every group: the answers are not a proof
    Q_PROPERTY(bool        searchFull READ searchFull NOTIFY resultChanged)

    // The answers, ready for the page.
    Q_PROPERTY(QVariantList criteria    READ criteria    NOTIFY resultChanged)
    Q_PROPERTY(QString      mapText     READ mapText     NOTIFY resultChanged)
    Q_PROPERTY(QString      keyText     READ keyText     NOTIFY resultChanged)
    Q_PROPERTY(QVariantList openings    READ openings    NOTIFY resultChanged)
    Q_PROPERTY(QVariantList steps       READ steps       NOTIFY resultChanged)
    Q_PROPERTY(QVariantList pairs       READ pairs       NOTIFY resultChanged)
    Q_PROPERTY(QVariantList pieceRows   READ pieceRows   NOTIFY resultChanged)
    Q_PROPERTY(QString      sweepText   READ sweepText   NOTIFY sweepChanged)

    // What the 3D view shows.
    Q_PROPERTY(int    selected   READ selected   WRITE setSelected   NOTIFY displayChanged)
    Q_PROPERTY(double pull       READ pull       WRITE setPull       NOTIFY displayChanged)
    Q_PROPERTY(double playhead   READ playhead   WRITE setPlayhead   NOTIFY displayChanged)
    Q_PROPERTY(double minThickness READ minThickness WRITE setMinThickness NOTIFY resultChanged)

public:
    explicit PuzzleAnalyzer(QObject *parent = nullptr);
    ~PuzzleAnalyzer() override;

    Q_INVOKABLE void open(const QUrl &url);
    Q_INVOKABLE void openPath(const QString &path);
    Q_INVOKABLE void openSibling(const QString &name);

    // Divide into nu x nv x nw cells and analyse.
    Q_INVOKABLE void run(int nu, int nv, int nw, int directions);

    // Slide the selected opening along +d and -d, and each member alone.
    Q_INVOKABLE void verifySelected();

    QString     fileName()  const { return m_fileName; }
    QString     folder()    const { return m_folder; }
    QStringList siblings()  const { return m_siblings; }
    bool        busy()      const { return m_busy; }
    QString     status()    const { return m_status; }
    bool        hasResult() const { return m_hasResult; }
    bool        searchFull() const { return m_hasResult && m_res.dbg.subsetSearchFull; }

    QVariantList criteria()  const { return m_criteria; }
    QString      mapText()   const { return m_mapText; }
    QString      keyText()   const { return m_keyText; }
    QVariantList openings()  const { return m_openingRows; }
    QVariantList steps()     const { return m_stepRows; }
    QVariantList pairs()     const { return m_pairRows; }
    QVariantList pieceRows() const { return m_pieceRows; }
    QString      sweepText() const { return m_sweepText; }

    int    selected() const { return m_selected; }
    double pull()     const { return m_pull; }
    double playhead() const { return m_playhead; }
    double minThickness() const { return m_minThickness; }
    void setSelected(int i);
    void setPull(double v);
    void setPlayhead(double v);
    void setMinThickness(double v);

    // For MeshView.
    QVector<PuzzlePiece> displayPieces() const;
    bool viewBounds(float lo[3], float hi[3]) const;

signals:
    void fileChanged();
    void busyChanged();
    void statusChanged();
    void resultChanged();
    void sweepChanged();
    void piecesChanged();    // new geometry: refit the view
    void displayChanged();   // same geometry, moved or recoloured

private:
    struct Result {
        DbgReport  dbg;
        DbgQuality quality;
    };

    void setStatus(const QString &s);
    void setBusy(bool b);
    void finishRun();
    void buildTexts();
    void buildCriteria();
    int  pieceId(const PuzzlePiece &p) const;

    Trivariate           m_tv;
    QString              m_path;
    QString              m_fileName;
    QString              m_folder;
    QStringList          m_siblings;

    DbgOptions           m_opt;
    QVector<PuzzlePiece> m_pieces;       // assembled, one per cell, by piece id
    float                m_lo[3] = { 0, 0, 0 }, m_hi[3] = { 0, 0, 0 };

    QFutureWatcher<Result> m_watcher;
    Result               m_res;
    bool                 m_hasResult = false;
    bool                 m_busy = false;
    QString              m_status;

    QVariantList m_criteria, m_openingRows, m_stepRows, m_pairRows, m_pieceRows;
    QString      m_mapText, m_keyText, m_sweepText;

    int    m_selected = -1;
    double m_pull = 0.0;
    double m_playhead = 0.0;
    double m_minThickness = 0.03;   // as a fraction of the model's size
};
