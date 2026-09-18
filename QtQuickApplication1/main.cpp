//
// main - the GUI entry point: install the IRIT error handlers, register the
// QML types, load main.qml, and run.
//
#include "AppController.h"
#include "IritGuard.h"
#include "MeshView.h"

#include <QDebug>
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QQmlError>
#include <QStringList>
#include <QUrl>
#include <qqml.h>

int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);
    app.setApplicationName(QStringLiteral("Puzzle Divider"));

    // Before any other IRIT call: stop a malformed file from taking the
    // process down through IRIT's default exit()-on-error handlers.
    IritGuard::installHandlers();

    qmlRegisterType<MeshView>("PuzzleDivider", 1, 0, "MeshView");
    qmlRegisterUncreatableType<AppController>(
        "PuzzleDivider", 1, 0, "AppController",
        QStringLiteral("AppController is provided as the 'app' context property."));

    QQmlApplicationEngine engine;

    // Without this, a QML error means the window silently never appears and
    // the process exits with -1 - the failure mode is invisible under the
    // Windows subsystem.
    QObject::connect(&engine, &QQmlApplicationEngine::warnings,
                     [](const QList<QQmlError> &warnings) {
                         for (const QQmlError &w : warnings)
                             qCritical().noquote() << w.toString();
                     });

    AppController controller;
    engine.rootContext()->setContextProperty(QStringLiteral("app"), &controller);

    engine.load(QUrl(QStringLiteral("qrc:/qt/qml/qtquickapplication1/main.qml")));
    if (engine.rootObjects().isEmpty()) {
        qCritical() << "Failed to load main.qml - see the errors above.";
        return -1;
    }

    // Allow "app.exe model.stl" so a file can be loaded without the dialog.
    const QStringList args = QGuiApplication::arguments();
    if (args.size() > 1)
        controller.loadPath(args.at(1));

    return app.exec();
}
