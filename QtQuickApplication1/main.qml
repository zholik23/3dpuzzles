import QtQuick
import QtQuick.Controls
import QtQuick.Dialogs
import QtQuick.Layouts
import PuzzleDivider

ApplicationWindow {
    id: win
    visible: true
    width: 1180
    height: 820
    minimumWidth: 640
    minimumHeight: 520
    color: "#20242b"
    title: app.addJoints ? "Puzzle Divider — division with joints"
                         : "Puzzle Divider — division without joints"

    // ---------- 3D VIEW ----------
    // MeshView is our own QQuickPaintedItem: it rasterises with a z-buffer, so
    // no Qt Quick 3D module is needed.
    MeshView {
        id: view
        controller: app
        anchors {
            left: parent.left; right: parent.right; top: parent.top
            bottom: bottomBar.top
        }
    }

    Label {
        anchors { right: view.right; bottom: view.bottom; margins: 10 }
        visible: viewTabs.currentIndex === 0
        text: "drag = rotate · wheel = zoom" +
              (view.partCount > 1 ? " · " + view.partCount + " pieces" : "")
        color: "#6c7480"
        font.pixelSize: 11
    }

    // ---------- PLANNER PICTURES ----------
    // Every division writes four pictures of the planner beside the model
    // (PlannerFigure). These tabs show the last division's pictures here, so
    // they never have to be found in a folder. "3D view" is the model itself.
    Rectangle {
        id: figurePane
        anchors.fill: view
        visible: viewTabs.currentIndex > 0
        color: "white"

        // Swallow drags so the hidden 3D view does not rotate underneath.
        MouseArea { anchors.fill: parent }

        Image {
            anchors {
                fill: parent
                margins: 8
                topMargin: viewTabs.height + 16
            }
            fillMode: Image.PreserveAspectFit
            asynchronous: true
            cache: false
            smooth: true
            mipmap: true
            source: (viewTabs.currentIndex > 0 &&
                     viewTabs.currentIndex <= app.planFigures.length)
                    ? app.planFigures[viewTabs.currentIndex - 1] : ""
        }
    }

    TabBar {
        id: viewTabs
        anchors { left: view.left; top: view.top; margins: 8 }
        visible: app.planFigures.length > 0
        TabButton { text: "3D view";  width: implicitWidth }
        TabButton { text: "Pieces";   width: implicitWidth }
        TabButton { text: "Graph";    width: implicitWidth }
        TabButton { text: "Blocking"; width: implicitWidth }
        TabButton { text: "Order";    width: implicitWidth }
    }

    Row {
        anchors { right: view.right; top: view.top; margins: 8 }
        spacing: 8

        Button {
            visible: app.reportUrl.length > 0
            text: "Open report"
            onClicked: Qt.openUrlExternally(app.reportUrl)
            ToolTip.visible: hovered
            ToolTip.text: "Written analysis of this division: model facts, " +
                          "timings, every piece, and what the planner concluded"
        }
        Button {
            visible: app.planFigures.length > 0
            text: "Open folder"
            onClicked: Qt.openUrlExternally(app.planFolderUrl)
            ToolTip.visible: hovered
            ToolTip.text: "The planner pictures of this division, full size"
        }
    }

    // A division without pictures, or a newly loaded model, goes back to 3D.
    Connections {
        target: app
        function onPiecesChanged() {
            if (app.planFigures.length === 0)
                viewTabs.currentIndex = 0
        }
    }

    // ---------- CONTROLS ----------
    // Height follows the content: the Flow rows wrap when the window is narrow,
    // so the bar has to grow rather than clip its own controls off-screen.
    Rectangle {
        id: bottomBar
        anchors { left: parent.left; right: parent.right; bottom: parent.bottom }
        height: controls.implicitHeight + 24
        color: "#2b2f37"

        ColumnLayout {
            id: controls
            anchors { left: parent.left; right: parent.right; top: parent.top; margins: 12 }
            spacing: 8

            // --- file + view ---------------------------------------------
            Flow {
                Layout.fillWidth: true
                spacing: 8

                Button {
                    text: "Open model…"
                    onClicked: fileDialog.open()
                }
                Button {
                    text: "Save pieces…"
                    // Nothing to write until a division has run.
                    enabled: app.canSave
                    onClicked: saveDialog.open()
                }
                CheckBox {
                    id: addJoints
                    text: "Joints"
                    // Off by default: cutting a pin and a hole per shared face is
                    // a boolean per piece, so it roughly doubles the time a
                    // division takes.
                    checked: app.addJoints
                    onToggled: app.addJoints = checked
                    ToolTip.visible: hovered
                    ToolTip.text: "Cut a pin and matching hole on the faces the " +
                                  "planner chooses, then test whether each piece " +
                                  "can still turn far enough to seat. Toggling " +
                                  "this re-runs the last division with the same " +
                                  "layout."
                    contentItem: Text {
                        text: parent.text; color: "#ccc"
                        leftPadding: parent.indicator.width + 4
                        verticalAlignment: Text.AlignVCenter
                    }
                }
                CheckBox {
                    id: spreadOut
                    text: "Spread apart (for slicing)"
                    // On by default: a slicer merges touching shells into one
                    // object, so a puzzle saved in model coordinates arrives in
                    // Bambu Studio as a single lump.
                    checked: true
                    ToolTip.visible: hovered
                    ToolTip.text: "Lays the pieces out side by side on the bed. " +
                                  "Turn off to keep them in their assembled positions."
                    contentItem: Text {
                        text: parent.text; color: "#ccc"
                        leftPadding: parent.indicator.width + 4
                        verticalAlignment: Text.AlignVCenter
                    }
                }
                CheckBox {
                    text: "Shaded"
                    checked: view.shaded
                    onToggled: view.shaded = checked
                    contentItem: Text {
                        text: parent.text; color: "#ccc"
                        leftPadding: parent.indicator.width + 4
                        verticalAlignment: Text.AlignVCenter
                    }
                }
                Button {
                    text: "Reset view"
                    enabled: view.hasMesh
                    onClicked: view.resetView()
                }
                Button {
                    // A box over the model's extent: right size, wrong shape.
                    // Kept only as the Increment 3 placeholder.
                    text: "Bounding cage"
                    enabled: app.hasMesh
                    onClicked: app.useBoundingCage()
                    ToolTip.visible: hovered
                    ToolTip.text: "A box over the model's extent. It has the size " +
                                  "but not the shape — that arrives with the " +
                                  "Increment 3 trivariate fit."
                }
                Button {
                    text: "Back to model"
                    enabled: !app.dividesMesh
                    onClicked: app.showWholeModel()
                }
                Button {
                    // Its own window: opens a saved trivariate (tvs_*.itd) and
                    // checks the experiment's criteria on it, independent of
                    // whatever this window has loaded.
                    text: "Analyse puzzle…"
                    onClicked: {
                        analyzerWindow.show()
                        analyzerWindow.raise()
                    }
                    ToolTip.visible: hovered
                    ToolTip.text: "Directional blocking graph, single key and level k, " +
                                  "thickness, sizes - on a saved trivariate"
                }
            }

            Label {
                text: app.status
                color: app.hasError ? "#e88" : "#f0f0f0"
                font.bold: true
                elide: Text.ElideMiddle
                Layout.fillWidth: true
            }

            // --- how it gets divided -------------------------------------
            Flow {
                Layout.fillWidth: true
                spacing: 8

                Label {
                    text: "Mode"; color: "#ccc"; height: modeBox.height
                    verticalAlignment: Text.AlignVCenter
                }
                ComboBox {
                    id: modeBox
                    width: 190
                    model: [ "Uniform", "Random", "Max piece size" ]
                    // Opens on Random, because that is the piece-count-driven
                    // split the assemblable divider is built on. Uniform is a
                    // grid by definition, so it is the only mode that still
                    // wants three per-axis counts.
                    currentIndex: 1
                }

                // Random splits recursively, so it has a target COUNT rather
                // than a per-axis grid.
                Row {
                    spacing: 4
                    visible: modeBox.currentIndex === 1
                    Label {
                        text: "Pieces"; color: "#ccc"; height: nRandom.height
                        verticalAlignment: Text.AlignVCenter; rightPadding: 4
                    }
                    SpinBox { id: nRandom; from: 2; to: 512; value: 24; width: 110; editable: true }
                }

                Row {
                    spacing: 4
                    visible: modeBox.currentIndex === 0
                    Label {
                        text: "Grid"; color: "#ccc"; height: nx.height
                        verticalAlignment: Text.AlignVCenter; rightPadding: 4
                    }
                    SpinBox { id: nx; from: 1; to: 32; value: 3; width: 96; editable: true }
                    SpinBox { id: ny; from: 1; to: 32; value: 3; width: 96; editable: true }
                    SpinBox { id: nz; from: 1; to: 32; value: 2; width: 96; editable: true }
                }

                // No curve and no seed control. Curving the cuts warps the
                // model and deforms its outer shape on a coarse mesh, and a
                // visible seed is a number to manage rather than a decision to
                // make - the button below just rolls a new one.
                Button {
                    visible: modeBox.currentIndex === 1
                    text: "New layout"
                    enabled: app.hasMesh
                    onClicked: app.newLayout()
                    ToolTip.visible: hovered
                    ToolTip.text: "Reshuffle where the cuts fall, same piece count"
                }

                Row {
                    spacing: 4
                    visible: modeBox.currentIndex === 2
                    Label {
                        text: "Max piece mm"; color: "#ccc"; height: maxSize.height
                        verticalAlignment: Text.AlignVCenter; rightPadding: 4
                    }
                    TextField {
                        id: maxSize; text: "60"; width: 74
                        validator: DoubleValidator { bottom: 0.001 }
                    }
                    Label {
                        text: "max/axis"; color: "#ccc"; height: maxAxis.height
                        verticalAlignment: Text.AlignVCenter
                        leftPadding: 6; rightPadding: 4
                    }
                    SpinBox { id: maxAxis; from: 1; to: 64; value: 16; width: 100; editable: true }
                }

                Button {
                    text: "Divide"
                    enabled: app.hasMesh
                    onClicked: {
                        switch (modeBox.currentIndex) {
                        case 0:
                            app.divideUniform(nx.value, ny.value, nz.value);
                            break;
                        case 1:
                            app.divideRandom(nRandom.value);
                            break;
                        case 2:
                            app.divideBySize(parseFloat(maxSize.text), maxAxis.value);
                            break;
                        }
                    }
                }
            }

            // What the division actually produced, joints included. The
            // controller reports it here rather than in `detail`, which is the
            // error channel.
            Label {
                Layout.fillWidth: true
                visible: app.pieceCount > 0
                text: app.divisionInfo
                color: app.jointNote.indexOf("FAILED") >= 0 ? "#e88" : "#9cc07e"
                font.pixelSize: 11
                wrapMode: Text.WordWrap
            }

            // --- explode --------------------------------------------------
            RowLayout {
                Layout.fillWidth: true
                spacing: 8
                Label { text: "Explode"; color: "#ccc" }
                Slider {
                    Layout.fillWidth: true
                    from: 0.0; to: 2.0
                    value: view.explode
                    onMoved: view.explode = value
                }
            }

            // --- detail ---------------------------------------------------
            Label {
                text: app.detail
                color: app.hasError ? "#e88" : "#9aa4b0"
                font.pixelSize: 12
                wrapMode: Text.WordWrap
                maximumLineCount: 3
                elide: Text.ElideRight
                Layout.fillWidth: true
            }
        }
    }

    AnalyzerWindow { id: analyzerWindow; visible: false }

    FileDialog {
        id: saveDialog
        title: "Save the divided model"
        fileMode: FileDialog.SaveFile
        defaultSuffix: "itd"
        nameFilters: app.saveFilters
        // one file per piece is off while the control is hidden; PieceExport
        // still supports it, so restoring the checkbox is the only change
        // needed to bring it back.
        onAccepted: app.savePieces(selectedFile, false, spreadOut.checked)
    }

    FileDialog {
        id: fileDialog
        title: "Open a CAD model"
        // Filters come from CadLoader so the dialog and the loader can never
        // disagree about what is supported.
        nameFilters: app.nameFilters
        onAccepted: app.loadFile(selectedFile)
    }
}
