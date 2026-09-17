#include "DivisionReport.h"

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QHash>
#include <QLocale>
#include <QTextStream>

#include <cmath>

namespace {

// ---------------------------------------------------------------- measuring --

// Signed volume by the divergence theorem. Positive for an outward-wound
// closed mesh; the sign is kept out of the report and only the size is shown,
// because the app's own pieces come back from IRIT wound either way.
double meshVolume(const MeshData &m)
{
    double v = 0.0;
    for (int t = 0; t + 2 < m.tris.size(); t += 3) {
        const float *A = &m.pos[m.tris[t + 0] * 3];
        const float *B = &m.pos[m.tris[t + 1] * 3];
        const float *C = &m.pos[m.tris[t + 2] * 3];

        v += double(A[0]) * (double(B[1]) * double(C[2]) - double(B[2]) * double(C[1]))
           - double(A[1]) * (double(B[0]) * double(C[2]) - double(B[2]) * double(C[0]))
           + double(A[2]) * (double(B[0]) * double(C[1]) - double(B[1]) * double(C[0]));
    }
    return std::fabs(v) / 6.0;
}

// Every directed edge needs its opposite twin for the mesh to be closed. An
// open mesh makes the volume above meaningless, so the report says which it is
// rather than printing a number that cannot be trusted.
bool isClosed(const MeshData &m)
{
    QHash<quint64, int> edges;
    const auto key = [](quint32 a, quint32 b) {
        return (quint64(a) << 32) | quint64(b);
    };

    for (int t = 0; t + 2 < m.tris.size(); t += 3) {
        const quint32 v[3] = { m.tris[t + 0], m.tris[t + 1], m.tris[t + 2] };
        for (int e = 0; e < 3; ++e)
            ++edges[key(v[e], v[(e + 1) % 3])];
    }
    for (auto it = edges.constBegin(); it != edges.constEnd(); ++it) {
        const quint32 a = quint32(it.key() >> 32), b = quint32(it.key() & 0xffffffffu);
        if (edges.value(key(b, a), 0) != it.value())
            return false;
    }
    return true;
}

// ----------------------------------------------------------------- writing --

QString esc(const QString &s)
{
    QString out = s;
    out.replace(QLatin1Char('&'), QStringLiteral("&amp;"));
    out.replace(QLatin1Char('<'), QStringLiteral("&lt;"));
    out.replace(QLatin1Char('>'), QStringLiteral("&gt;"));
    return out;
}

QString num(double v, int prec = 4)
{
    return QString::number(v, 'g', prec);
}

// "-" for a stage that did not run, so a blank is never read as "instant".
QString ms(qint64 v)
{
    return v < 0 ? QStringLiteral("&mdash;")
                 : QLocale().toString(qlonglong(v)) + QStringLiteral(" ms");
}

QString row(const QString &name, const QString &value)
{
    return QStringLiteral("<tr><th>%1</th><td>%2</td></tr>\n").arg(name, value);
}

const char *kStyle =
    "body{margin:0;background:#fff;color:#141a20;"
    "font:15px/1.55 'Segoe UI',system-ui,sans-serif}"
    "main{max-width:1000px;margin:0 auto;padding:40px 28px 72px}"
    "h1{font-size:30px;margin:0 0 4px}"
    "h2{font-size:20px;margin:38px 0 10px;padding-bottom:6px;"
    "border-bottom:1px solid #c6d0d9}"
    ".sub{color:#5c6874;margin:0 0 8px}"
    "table{border-collapse:collapse;width:100%;margin:10px 0}"
    "th,td{text-align:left;padding:6px 10px;border-bottom:1px solid #e4e9ee;"
    "vertical-align:top}"
    "th{width:34%;font-weight:600;color:#3d4a56}"
    "table.grid th{width:auto}"
    "table.grid td,table.grid th{font-variant-numeric:tabular-nums}"
    "td.num{text-align:right;font-variant-numeric:tabular-nums}"
    ".good{color:#1e6b3a;font-weight:600}"
    ".warn{color:#9b2c1f;font-weight:600}"
    ".note{background:#f4f7f9;border-left:3px solid #2f6f99;padding:10px 14px;"
    "margin:14px 0;color:#3d4a56}"
    "figure{margin:22px 0}"
    "figure img{width:100%;border:1px solid #c6d0d9}"
    "figcaption{color:#5c6874;font-size:13px;margin-top:6px}"
    "code{background:#f4f7f9;padding:1px 5px;border-radius:3px}";

} // namespace

QString DivisionReport::write(const QString &folder,
                              const QString &title,
                              const MeshData &model,
                              const QVector<PuzzlePiece> &pieces,
                              const DivisionStats &stats,
                              QString *error)
{
    if (folder.isEmpty()) {
        if (error) *error = QStringLiteral("no folder to write the report into");
        return QString();
    }
    if (!QDir().mkpath(folder)) {
        if (error) *error = QStringLiteral("could not create %1").arg(folder);
        return QString();
    }

    const QString path = folder + QLatin1Char('/') + QStringLiteral("report.html");
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Text)) {
        if (error) *error = QStringLiteral("could not write %1").arg(path);
        return QString();
    }

    const QLocale loc;
    const double modelVol = meshVolume(model);
    const bool   closed   = isClosed(model);

    // The check that matters: the pieces must add up to the model. It only
    // means anything when both are closed solids, so it is labelled when not.
    double pieceVol = 0.0;
    int    openPieces = 0, pieceTris = 0;
    float  biggest = 0.0f, smallest = 1e30f;
    for (const PuzzlePiece &p : pieces) {
        pieceVol  += meshVolume(p.mesh);
        pieceTris += p.mesh.triangleCount();
        if (!isClosed(p.mesh))
            ++openPieces;
        biggest  = qMax(biggest,  p.largestSide());
        smallest = qMin(smallest, p.largestSide());
    }
    if (pieces.isEmpty())
        smallest = 0.0f;

    QTextStream o(&f);
    o << "<!doctype html>\n<meta charset=\"utf-8\">\n";
    o << "<title>" << esc(title) << " &mdash; division report</title>\n";
    o << "<style>" << kStyle << "</style>\n<main>\n";

    o << "<h1>" << esc(title) << "</h1>\n";
    o << "<p class=\"sub\">Division report &middot; "
      << esc(QDateTime::currentDateTime().toString(QStringLiteral("yyyy-MM-dd HH:mm")))
      << "</p>\n";

    // ---------------------------------------------------------------- model
    o << "<h2>The model</h2>\n<table>\n";
    o << row(QStringLiteral("Vertices"), loc.toString(model.vertexCount()));
    o << row(QStringLiteral("Triangles"), loc.toString(model.triangleCount()));
    o << row(QStringLiteral("Bounding box"),
             QStringLiteral("%1 &times; %2 &times; %3")
                 .arg(num(double(model.bmax[0]) - model.bmin[0]))
                 .arg(num(double(model.bmax[1]) - model.bmin[1]))
                 .arg(num(double(model.bmax[2]) - model.bmin[2])));
    o << row(QStringLiteral("Watertight"),
             closed ? QStringLiteral("<span class=\"good\">yes</span>")
                    : QStringLiteral("<span class=\"warn\">no</span> &mdash; "
                                     "volumes below are not trustworthy, and "
                                     "IRIT's booleans give garbage rather than "
                                     "an error on an open shell"));
    o << row(QStringLiteral("Volume (divergence)"), num(modelVol, 6));
    if (stats.hasField) {
        o << row(QStringLiteral("Volume (voxel fill)"),
                 QStringLiteral("%1 &middot; %2% of the divergence volume")
                     .arg(num(stats.materialVolume, 6))
                     .arg(num(100.0 * stats.materialVolume /
                              qMax(1e-9, modelVol), 3)));
    }
    o << "</table>\n";
    if (stats.hasField) {
        o << "<div class=\"note\">Two independent measures. The divergence "
             "integral counts every shell it is given, so a mesh with interior "
             "geometry reads high; the voxel fill uses parity down each column "
             "and reports what is actually solid. Them disagreeing is a fact "
             "about the model, not about the division.</div>\n";
    }

    // ------------------------------------------------------------ the split
    o << "<h2>The division</h2>\n<table>\n";
    o << row(QStringLiteral("Mode"), esc(stats.mode));
    o << row(QStringLiteral("Divided"), esc(stats.subject));
    if (stats.requested > 0) {
        const bool exact = stats.cells == stats.requested;
        o << row(QStringLiteral("Pieces asked for"),
                 QStringLiteral("%1").arg(stats.requested));
        o << row(QStringLiteral("Cells produced"),
                 exact ? QStringLiteral("<span class=\"good\">%1</span>").arg(stats.cells)
                       : QStringLiteral("<span class=\"warn\">%1</span> &mdash; "
                                        "below target: the rest would have been "
                                        "under the minimum piece size")
                             .arg(stats.cells));
    }
    else {
        o << row(QStringLiteral("Cells produced"), QString::number(stats.cells));
    }
    o << row(QStringLiteral("Pieces kept"), QString::number(pieces.size()));
    o << row(QStringLiteral("Seed"), QString::number(stats.seed));
    if (stats.hasField) {
        o << row(QStringLiteral("Voxel grid"),
                 QStringLiteral("%1 &times; %2 &times; %3 = %4 voxels, "
                                "%5 filled (%6%), side %7")
                     .arg(stats.voxels[0]).arg(stats.voxels[1]).arg(stats.voxels[2])
                     .arg(loc.toString(stats.voxels[0] * stats.voxels[1] *
                                       stats.voxels[2]))
                     .arg(loc.toString(stats.voxelsFilled))
                     .arg(num(100.0 * stats.voxelsFilled /
                              qMax(1, stats.voxels[0] * stats.voxels[1] *
                                      stats.voxels[2]), 3))
                     .arg(num(stats.voxelSide)));
    }
    if (!stats.booleanNote.isEmpty())
        o << row(QStringLiteral("Trim (Elber &sect;5)"), esc(stats.booleanNote));
    if (!stats.jointNote.isEmpty())
        o << row(QStringLiteral("Joints"), esc(stats.jointNote));
    if (!stats.warning.isEmpty())
        o << row(QStringLiteral("Warning"),
                 QStringLiteral("<span class=\"warn\">%1</span>").arg(esc(stats.warning)));
    o << "</table>\n";

    // --------------------------------------------------------------- timing
    o << "<h2>Time</h2>\n<table>\n";
    o << row(QStringLiteral("Voxelise the model"), ms(stats.msVoxelise));
    o << row(QStringLiteral("Choose the cuts"), ms(stats.msSplit));
    o << row(QStringLiteral("Extract the pieces"), ms(stats.msExtract));
    o << row(QStringLiteral("Trim to the model"), ms(stats.msTrim));
    o << row(QStringLiteral("Cut the joints"), ms(stats.msJoints));
    o << row(QStringLiteral("Draw the planner pictures"), ms(stats.msFigures));
    o << row(QStringLiteral("<b>Total</b>"), QStringLiteral("<b>%1</b>").arg(ms(stats.msTotal)));
    o << "</table>\n";

    // --------------------------------------------------------------- pieces
    o << "<h2>The pieces</h2>\n<table>\n";
    o << row(QStringLiteral("Count"), QString::number(pieces.size()));
    o << row(QStringLiteral("Triangles, all pieces"), loc.toString(pieceTris));
    o << row(QStringLiteral("Largest / smallest side"),
             QStringLiteral("%1 / %2 &middot; spread %3&times;")
                 .arg(num(double(biggest))).arg(num(double(smallest)))
                 .arg(num(double(biggest) / qMax(1e-9, double(smallest)), 3)));
    {
        const double pct = 100.0 * pieceVol / qMax(1e-9, modelVol);
        const bool sound = closed && openPieces == 0;
        QString v = QStringLiteral("%1 &middot; <span class=\"%2\">%3%</span> of the model")
                        .arg(num(pieceVol, 6),
                             (sound && std::fabs(pct - 100.0) < 1.0)
                                 ? QStringLiteral("good") : QStringLiteral("warn"),
                             num(pct, 4));
        if (!sound)
            v += QStringLiteral(" &mdash; not a meaningful check here: "
                                "%1 piece(s) are open shells, and the divergence "
                                "integral over an open shell is not a volume")
                     .arg(openPieces);
        o << row(QStringLiteral("Volume, all pieces"), v);
    }
    o << "</table>\n";

    if (!pieces.isEmpty()) {
        o << "<table class=\"grid\">\n<tr><th>#</th><th>size X</th><th>size Y</th>"
             "<th>size Z</th><th>triangles</th><th>volume</th>"
             "<th>% of model</th><th>closed</th></tr>\n";
        for (int i = 0; i < pieces.size(); ++i) {
            const PuzzlePiece &p = pieces[i];
            const double v = meshVolume(p.mesh);
            o << QStringLiteral("<tr><td>%1</td><td class=\"num\">%2</td>"
                                "<td class=\"num\">%3</td><td class=\"num\">%4</td>"
                                "<td class=\"num\">%5</td><td class=\"num\">%6</td>"
                                "<td class=\"num\">%7</td><td>%8</td></tr>\n")
                     .arg(i)
                     .arg(num(double(p.size[0])), num(double(p.size[1])),
                          num(double(p.size[2])))
                     .arg(loc.toString(p.mesh.triangleCount()))
                     .arg(num(v, 5))
                     .arg(num(100.0 * v / qMax(1e-9, modelVol), 3))
                     .arg(isClosed(p.mesh) ? QStringLiteral("yes")
                                           : QStringLiteral("<span class=\"warn\">no</span>"));
        }
        o << "</table>\n";
    }

    // -------------------------------------------------------------- planner
    if (stats.hasPlan) {
        o << "<h2>The assembly planner</h2>\n<table>\n";
        o << row(QStringLiteral("Shared faces"), QString::number(stats.contacts));
        o << row(QStringLiteral("Neighbours per piece"),
                 QStringLiteral("%1 to %2").arg(stats.minNeighbours).arg(stats.maxNeighbours));
        o << row(QStringLiteral("Order found"),
                 stats.planComplete
                     ? QStringLiteral("<span class=\"good\">yes</span>")
                     : QStringLiteral("<span class=\"warn\">no</span> &mdash; the "
                                      "greedy search got stuck; that is a failure "
                                      "of this search, not a proof that no order "
                                      "exists"));
        if (stats.planComplete && !stats.assemblyOrder.isEmpty()) {
            o << row(QStringLiteral("Assembly order"),
                     esc(stats.assemblyOrder.join(QStringLiteral(" → "))));
        }
        if (!stats.stuck.isEmpty()) {
            QStringList ids;
            for (int s : stats.stuck)
                ids << QString::number(s);
            o << row(QStringLiteral("Stuck pieces"),
                     QStringLiteral("<span class=\"warn\">%1</span>")
                         .arg(ids.join(QStringLiteral(", "))));
        }
        o << "</table>\n";
        o << "<div class=\"note\"><b>" << esc(stats.caveat) << "</b><br>"
             "A division into boxes always has some piece free to leave, so a "
             "successful order here is expected and carries little information. "
             "The test starts to bite once joints restrict how pieces may move, "
             "or once a swept-volume check replaces straight-line blocking.</div>\n";
    }

    // -------------------------------------------------------------- figures
    if (!stats.figureFiles.isEmpty()) {
        static const char *kCaptions[4] = {
            "The pieces, pulled apart, numbered with the ids the planner uses.",
            "Step 1 &mdash; every line joins two pieces that share a face, "
            "coloured by the axis that face is perpendicular to.",
            "Step 2 &mdash; for each piece, which of the six straight moves are "
            "blocked, and by which neighbour.",
            "Step 3 &mdash; the pieces taken off one at a time; reversed, this is "
            "the assembly order."
        };
        o << "<h2>The planner, drawn</h2>\n";
        for (int i = 0; i < stats.figureFiles.size(); ++i) {
            o << "<figure><img src=\"" << esc(stats.figureFiles[i]) << "\" alt=\""
              << esc(stats.figureFiles[i]) << "\">\n<figcaption>"
              << (i < 4 ? kCaptions[i] : "") << "</figcaption></figure>\n";
        }
    }

    o << "</main>\n";
    o.flush();
    f.close();

    if (f.error() != QFileDevice::NoError) {
        if (error) *error = QStringLiteral("could not finish writing %1").arg(path);
        return QString();
    }
    return path;
}
