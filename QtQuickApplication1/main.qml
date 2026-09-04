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
    title: "Puzzle Divider — division without joints"

    readonly property var modeHints: [
        "Every piece the same size and shape, cut by straight planes.",
        "Split recursively, one cell at a time, so piece sizes are independent of each other and a big piece can border several small ones — a grid of cut planes can never do that. Curve % bends the cuts as well. Same seed, same puzzle.",
        "Caps how big any piece may get; the count follows from the limit. On a MESH the cuts come out evenly spaced — in world space equal slabs already satisfy a size limit, so there is nothing to be uneven about. On a TRIVARIATE the same limit gives genuinely unequal cuts, because equal parameter steps are not equal physical sizes. For unequal pieces on a mesh, use Random."
    ]

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
        text: "drag = rotate · wheel = zoom" +
              (view.partCount > 1 ? " · " + view.partCount + " pieces" : "")
        color: "#6c7480"
        font.pixelSize: 11
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
                CheckBox {
                    text: "Edges"
                    checked: view.showEdges
                    onToggled: view.showEdges = checked
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
                    text: "Use trivariate in file"
                    enabled: app.hasMesh
                    onClicked: app.useTrivariateFromFile()
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
            }

            Label {
                text: app.status
                color: app.hasError ? "#e88" : "#f0f0f0"
                font.bold: true
                elide: Text.ElideMiddle
                Layout.fillWidth: true
            }

            // --- what gets divided ---------------------------------------
            Label {
                Layout.fillWidth: true
                color: app.dividesMesh ? "#9cc07e" : "#7ab0c8"
                font.pixelSize: 12
                wrapMode: Text.WordWrap
                text: app.dividesMesh
                    ? "Dividing the loaded model — pieces keep its real shape."
                    : "Dividing the trivariate “" + app.trivariateName +
                      "” — Elber's method, every piece a solid sub-trivariate."
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
                    SpinBox { id: nRandom; from: 2; to: 512; value: 24; width: 110 }
                }

                Row {
                    spacing: 4
                    visible: modeBox.currentIndex === 0
                    Label {
                        text: "Grid"; color: "#ccc"; height: nx.height
                        verticalAlignment: Text.AlignVCenter; rightPadding: 4
                    }
                    SpinBox { id: nx; from: 1; to: 32; value: 3; width: 96 }
                    SpinBox { id: ny; from: 1; to: 32; value: 3; width: 96 }
                    SpinBox { id: nz; from: 1; to: 32; value: 2; width: 96 }
                }

                Row {
                    spacing: 4
                    visible: modeBox.currentIndex === 1
                    Label {
                        text: "Curve %"; color: "#ccc"; height: curve.height
                        verticalAlignment: Text.AlignVCenter; rightPadding: 4
                    }
                    // Off by default. Curving the cuts warps the model, cuts
                    // it flat and un-warps the pieces; on a coarse mesh the
                    // clip points are interpolated along long straight edges,
                    // so un-warping does not put them back on the original
                    // surface and the OUTER shape visibly bulges. Randomness
                    // in the cut POSITIONS is what "Random" means - this is a
                    // separate, opt-in effect.
                    SpinBox {
                        id: curve; from: 0; to: 50; value: 0; width: 96
                        ToolTip.visible: hovered
                        ToolTip.text: "Bends the cut surfaces. Leave at 0 for " +
                                      "straight planar cuts. Above 0 it deforms " +
                                      "the outer shape of a coarse model."
                    }
                    Label {
                        text: "Seed"; color: "#ccc"; height: seed.height
                        verticalAlignment: Text.AlignVCenter
                        leftPadding: 6; rightPadding: 4
                    }
                    SpinBox { id: seed; from: 1; to: 9999; value: 7; width: 110 }
                    Button {
                        text: "↻"
                        onClicked: seed.value = 1 + Math.floor(Math.random() * 9999)
                        ToolTip.visible: hovered
                        ToolTip.text: "New random puzzle"
                    }
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
                    SpinBox { id: maxAxis; from: 1; to: 64; value: 16; width: 100 }
                }

                CheckBox {
                    id: jointsBox
                    text: "Joints"
                    // Not bound to app.addJoints: clicking a CheckBox breaks a
                    // binding on `checked`, which would leave the two out of
                    // step. The checkbox owns the state and pushes it down.
                    checked: true
                    onCheckedChanged: app.addJoints = checked
                    contentItem: Text {
                        text: parent.text; color: "#ccc"
                        leftPadding: parent.indicator.width + 4
                        verticalAlignment: Text.AlignVCenter
                    }
                    ToolTip.visible: hovered
                    ToolTip.text: "Cut Elber's pin/hole pair into every shared " +
                                  "face with an IRIT boolean, using the pin " +
                                  "from his puz_vol.irt."
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
                            app.divideRandom(nRandom.value, curve.value, seed.value);
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

            Label {
                Layout.fillWidth: true
                text: win.modeHints[modeBox.currentIndex]
                color: "#7d8894"
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

    FileDialog {
        id: fileDialog
        title: "Open a CAD model"
        // Filters come from CadLoader so the dialog and the loader can never
        // disagree about what is supported.
        nameFilters: app.nameFilters
        onAccepted: app.loadFile(selectedFile)
    }
}
