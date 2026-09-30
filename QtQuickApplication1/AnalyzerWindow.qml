import QtQuick
import QtQuick.Controls
import QtQuick.Dialogs
import QtQuick.Layouts
import PuzzleDivider

// The "Analyse puzzle" window. Open a saved trivariate (tvs_*.itd), divide its
// parameter domain, and read off the experiment's criteria - with the pieces
// on screen, so every verdict can be looked at, not just believed.
ApplicationWindow {
    id: win
    width: 1320
    height: 860
    minimumWidth: 900
    minimumHeight: 600
    color: "#20242b"
    title: pa.fileName.length > 0 ? "Analyse puzzle — " + pa.fileName
                                        : "Analyse puzzle"

    PuzzleAnalyzer { id: pa }

    // One look for every section heading and every body line.
    component Heading: Label {
        color: "#f0f0f0"; font.bold: true; font.pixelSize: 14
        topPadding: 10
    }
    component Body: Label {
        color: "#aab3bf"; font.pixelSize: 12
        wrapMode: Text.WordWrap
        Layout.fillWidth: true
    }
    component Cell: Label {
        color: "#cfd6de"; font.pixelSize: 12
        font.family: "Consolas"
        elide: Text.ElideRight
    }

    // PASS / FAIL / WARN / N/A as a coloured chip, so state reads at a glance.
    component StatePill: Rectangle {
        property string verdict: "na"
        implicitWidth: 46; implicitHeight: 20; radius: 3
        color: verdict === "pass" ? "#2f6b3a" : verdict === "fail" ? "#8a2f2f"
             : verdict === "warn" ? "#7a6420" : "#3b414b"
        Text {
            anchors.centerIn: parent
            text: verdict === "pass" ? "PASS" : verdict === "fail" ? "FAIL"
                : verdict === "warn" ? "WARN" : "N/A"
            color: "white"; font.pixelSize: 10; font.bold: true
        }
    }

    RowLayout {
        anchors.fill: parent
        spacing: 0

        // ================= LEFT: controls and answers =================
        Rectangle {
            Layout.preferredWidth: 520
            Layout.fillHeight: true
            color: "#2b2f37"

            ScrollView {
                id: scroll
                anchors.fill: parent
                anchors.margins: 12
                contentWidth: availableWidth
                clip: true

                ColumnLayout {
                    width: scroll.availableWidth
                    spacing: 6

                    // --- file ------------------------------------------------
                    RowLayout {
                        Layout.fillWidth: true
                        Button {
                            text: "Open trivariate…"
                            enabled: !pa.busy
                            onClicked: openDialog.open()
                        }
                        ComboBox {
                            id: siblingBox
                            Layout.fillWidth: true
                            enabled: !pa.busy && pa.siblings.length > 0
                            model: pa.siblings
                            currentIndex: pa.siblings.indexOf(pa.fileName)
                            onActivated: pa.openSibling(currentText)
                            ToolTip.visible: hovered
                            ToolTip.text: "Other trivariates in the same folder - step through the R sweep"
                        }
                    }

                    // What M is: a real trivariate that bends, or a box.
                    Body {
                        visible: pa.trivariateText.length > 0
                        text: pa.trivariateText
                        color: pa.trivariateFlat ? "#e88" : "#9cc07e"
                        font.pixelSize: 11
                    }

                    // --- division --------------------------------------------
                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 4
                        Label { text: "Division"; color: "#ccc" }
                        ComboBox {
                            id: divMode
                            Layout.fillWidth: true
                            model: [ "BSP in D", "Grid u·v·w in D" ]
                            currentIndex: 0
                        }
                    }
                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 4
                        visible: divMode.currentIndex === 1
                        Label { text: "Cells u·v·w"; color: "#ccc" }
                        SpinBox { id: nu; from: 1; to: 8; value: 2; editable: true; Layout.preferredWidth: 86 }
                        SpinBox { id: nv; from: 1; to: 8; value: 2; editable: true; Layout.preferredWidth: 86 }
                        SpinBox { id: nw; from: 1; to: 8; value: 2; editable: true; Layout.preferredWidth: 86 }
                    }
                    GridLayout {
                        Layout.fillWidth: true
                        visible: divMode.currentIndex === 0
                        columns: 4
                        columnSpacing: 8
                        Label { text: "Pieces"; color: "#ccc" }
                        SpinBox { id: bspPieces; from: 2; to: 12; value: 8; editable: true; Layout.preferredWidth: 96 }
                        Label { text: "Layout"; color: "#ccc" }
                        SpinBox {
                            id: bspSeed; from: 1; to: 9999; value: 1; editable: true; Layout.preferredWidth: 110
                            ToolTip.visible: hovered
                            ToolTip.text: "Which random BSP layout - same number, same cuts"
                        }

                    }
                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 4
                        Label { text: "Directions"; color: "#ccc" }
                        SpinBox {
                            id: dirs; from: 500; to: 20000; stepSize: 500; value: 4000
                            editable: true; Layout.preferredWidth: 120
                            ToolTip.visible: hovered
                            ToolTip.text: "How many pull directions are tried on the sphere of directions"
                        }
                        Item { Layout.fillWidth: true }
                        BusyIndicator { running: pa.busy; Layout.preferredHeight: 32; Layout.preferredWidth: 32 }
                        Button {
                            text: "Analyse"
                            highlighted: true
                            enabled: !pa.busy && pa.fileName.length > 0
                            onClicked: divMode.currentIndex === 0
                                       // Flat cuts in D for now; the bend will be chosen
                                       // automatically, not set by hand.
                                       ? pa.runBsp(bspPieces.value, 0.0, 1, bspSeed.value, dirs.value)
                                       : pa.run(nu.value, nv.value, nw.value, dirs.value)
                        }
                    }
                    Body {
                        text: pa.status
                        color: "#f0f0f0"
                        visible: text.length > 0
                    }

                    // --- criteria --------------------------------------------
                    Heading { text: "Criteria"; visible: pa.hasResult }
                    Repeater {
                        model: pa.hasResult ? pa.criteria : []
                        delegate: RowLayout {
                            required property var modelData
                            Layout.fillWidth: true
                            spacing: 8
                            StatePill { verdict: modelData.state; Layout.alignment: Qt.AlignTop }
                            ColumnLayout {
                                Layout.fillWidth: true
                                spacing: 1
                                Label { text: modelData.name; color: "#f0f0f0"; font.pixelSize: 12; font.bold: true }
                                Body { text: modelData.detail }
                            }
                        }
                    }
                    RowLayout {
                        Layout.fillWidth: true
                        visible: pa.hasResult
                        Label { text: "Thickness limit"; color: "#ccc" }
                        Slider {
                            id: thickSlider
                            Layout.fillWidth: true
                            from: 0.0; to: 0.2; stepSize: 0.005
                            value: pa.minThickness
                            onMoved: pa.minThickness = value
                        }
                        Label {
                            text: (pa.minThickness * 100).toFixed(1) + "% of model"
                            color: "#ccc"; font.pixelSize: 12
                        }
                    }
                    Body { text: pa.mapText; visible: pa.hasResult; font.pixelSize: 11 }

                    // --- ways it opens ---------------------------------------
                    Heading { text: "Ways it opens"; visible: pa.hasResult }
                    Body {
                        visible: pa.hasResult
                        text: !pa.searchFull
                              ? "Not computed: too many pieces to test every group."
                              : pa.openings.length === 0
                              ? "None: no piece and no group can slide out in any direction."
                              : "Each row is a group that slides out along d while the rest stays; " +
                                "the rest could equally slide along −d. Click one, then drag Pull."
                    }
                    Repeater {
                        model: pa.hasResult ? pa.openings : []
                        delegate: Rectangle {
                            required property var modelData
                            required property int index
                            Layout.fillWidth: true
                            Layout.preferredHeight: 26
                            radius: 3
                            color: pa.selected === index ? "#4a3a26"
                                 : openMouse.containsMouse ? "#353a44" : "transparent"
                            RowLayout {
                                anchors.fill: parent
                                anchors.leftMargin: 6; anchors.rightMargin: 6
                                Cell { text: modelData.label; Layout.preferredWidth: 150; color: "#f0c080" }
                                Cell { text: modelData.size + (modelData.size === 1 ? " piece" : " pieces"); Layout.preferredWidth: 70 }
                                Cell { text: modelData.clear; Layout.preferredWidth: 150 }
                                Cell { text: "d = " + modelData.dir; Layout.fillWidth: true }
                            }
                            MouseArea {
                                id: openMouse
                                anchors.fill: parent
                                hoverEnabled: true
                                onClicked: pa.selected = index
                            }
                        }
                    }
                    RowLayout {
                        Layout.fillWidth: true
                        visible: pa.hasResult && pa.openings.length > 0
                        Button {
                            text: "Verify by sliding"
                            enabled: !pa.busy && pa.selected >= 0
                            onClicked: pa.verifySelected()
                            ToolTip.visible: hovered
                            ToolTip.text: "Moves the selected group for real, with no normals, " +
                                          "and checks it against the DBG's verdict"
                        }
                    }
                    Body {
                        visible: pa.sweepText.length > 0
                        text: pa.sweepText
                        font.family: "Consolas"; font.pixelSize: 11
                        color: "#cfd6de"
                    }

                    // --- disassembly -----------------------------------------
                    Heading { text: "One way to take it apart"; visible: pa.hasResult }
                    Body {
                        visible: pa.hasResult
                        text: !pa.searchFull
                              ? "Not computed: too many pieces to test every group."
                              : pa.steps.length === 0
                              ? "It does not come apart at all."
                              : "Smallest movable group first, each time. Play it with the " +
                                "Disassembly slider under the view."
                    }
                    Repeater {
                        model: pa.hasResult ? pa.steps : []
                        delegate: RowLayout {
                            required property var modelData
                            Layout.fillWidth: true
                            Cell { text: "step " + modelData.step; Layout.preferredWidth: 60 }
                            Cell { text: modelData.label; Layout.preferredWidth: 150; color: "#f0c080" }
                            Cell { text: modelData.clear; Layout.preferredWidth: 150 }
                            Cell { text: "d = " + modelData.dir; Layout.fillWidth: true }
                        }
                    }

                    // --- pieces ----------------------------------------------
                    Heading { text: "Pieces"; visible: pa.hasResult }
                    RowLayout {
                        visible: pa.hasResult
                        Layout.fillWidth: true
                        spacing: 12
                        Repeater {
                            model: [["piece", 40], ["volume", 60], ["× mean", 50],
                                    ["thickness", 110], ["min det J", 70], ["clearance", 110]]
                            Label {
                                required property var modelData
                                text: modelData[0]; color: "#8d97a3"; font.pixelSize: 11
                                Layout.preferredWidth: modelData[1]
                            }
                        }
                    }
                    Repeater {
                        model: pa.hasResult ? pa.pieceRows : []
                        delegate: RowLayout {
                            required property var modelData
                            Layout.fillWidth: true
                            spacing: 12
                            Cell { text: modelData.id; Layout.preferredWidth: 40 }
                            Cell { text: modelData.volume; Layout.preferredWidth: 60 }
                            Cell { text: modelData.rel; Layout.preferredWidth: 50 }
                            Cell {
                                text: modelData.thick + " (" + modelData.thickPct + ")"
                                color: modelData.thin ? "#e6c84a" : "#cfd6de"
                                Layout.preferredWidth: 110
                            }
                            Cell {
                                text: modelData.detJ
                                color: modelData.folds ? "#ff7b72" : "#cfd6de"
                                Layout.preferredWidth: 70
                            }
                            Cell { text: modelData.clear; Layout.preferredWidth: 110 }
                        }
                    }

                    // --- interfaces ------------------------------------------
                    Heading { text: "Interfaces"; visible: pa.hasResult }
                    Body {
                        visible: pa.hasResult
                        text: "Spread = widest angle between the face's normals and their mean. " +
                              "Clearance = how far the pull may tilt and still separate the pair: " +
                              "90° for a flat face, 0 = blocked, under 1° = marginal."
                    }
                    Repeater {
                        model: pa.hasResult ? pa.pairs : []
                        delegate: RowLayout {
                            required property var modelData
                            Layout.fillWidth: true
                            Cell { text: modelData.pair; Layout.preferredWidth: 60 }
                            Cell { text: "cut " + modelData.axis; Layout.preferredWidth: 50 }
                            Cell { text: modelData.spread + "°"; Layout.preferredWidth: 60 }
                            Cell { text: modelData.clear; Layout.preferredWidth: 150 }
                            Cell {
                                text: modelData.blocked ? "BLOCKED" : ""
                                color: "#f0c080"; Layout.fillWidth: true
                            }
                        }
                    }
                    Item { Layout.preferredHeight: 12 }
                }
            }
        }

        // ================= RIGHT: the pieces =================
        ColumnLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 0

            MeshView {
                id: view
                analyzer: pa
                Layout.fillWidth: true
                Layout.fillHeight: true

                Label {
                    anchors { left: parent.left; top: parent.top; margins: 10 }
                    visible: pa.hasResult
                    textFormat: Text.RichText
                    font.pixelSize: 12
                    color: "#aab3bf"
                    text: "<span style='color:#e8842a'>■</span> moving&nbsp;&nbsp;" +
                          "<span style='color:#d9534f'>■</span> folds (det J &lt; 0)&nbsp;&nbsp;" +
                          "<span style='color:#e6c84a'>■</span> thinner than the limit"
                }
                Label {
                    anchors { left: parent.left; right: parent.right; bottom: parent.bottom; margins: 10 }
                    visible: pa.hasResult
                    wrapMode: Text.WordWrap
                    text: pa.keyText
                    color: "#f0f0f0"; font.pixelSize: 14; font.bold: true
                }
                Label {
                    anchors.centerIn: parent
                    visible: !view.hasMesh
                    text: pa.fileName.length === 0
                          ? "Open a trivariate - a tvs_*.itd from puz_vol - Copy.irt"
                          : "Press Analyse to divide it into pieces"
                    color: "#6c7480"
                }
            }

            Rectangle {
                Layout.fillWidth: true
                Layout.preferredHeight: sliders.implicitHeight + 20
                color: "#2b2f37"
                GridLayout {
                    id: sliders
                    anchors { left: parent.left; right: parent.right; top: parent.top; margins: 10 }
                    columns: 3
                    Label { text: "Pull"; color: "#ccc" }
                    Slider {
                        Layout.fillWidth: true
                        enabled: pa.selected >= 0
                        from: 0; to: 1
                        value: pa.pull
                        onMoved: pa.pull = value
                    }
                    Label { text: "selected opening, along its d"; color: "#8d97a3"; font.pixelSize: 11 }

                    Label { text: "Disassembly"; color: "#ccc" }
                    Slider {
                        Layout.fillWidth: true
                        enabled: pa.steps.length > 0
                        from: 0; to: Math.max(1, pa.steps.length)
                        value: pa.playhead
                        onMoved: pa.playhead = value
                    }
                    Label {
                        text: pa.steps.length > 0
                              ? "step " + Math.min(pa.steps.length, Math.floor(pa.playhead) + 1)
                                + " of " + pa.steps.length
                              : "-"
                        color: "#8d97a3"; font.pixelSize: 11
                    }

                    Label { text: "View"; color: "#ccc" }
                    RowLayout {
                        Button { text: "Reset view"; onClicked: view.resetView() }
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
                    }
                    Label { text: "drag = rotate · wheel = zoom"; color: "#6c7480"; font.pixelSize: 11 }
                }
            }
        }
    }

    FileDialog {
        id: openDialog
        title: "Open a trivariate"
        nameFilters: ["IRIT trivariate (*.itd)", "All files (*)"]
        onAccepted: pa.open(selectedFile)
    }
}
