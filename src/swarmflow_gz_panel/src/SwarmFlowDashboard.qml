// SwarmFlow robot dashboard (T022): one row per robot. Data: the SwarmFlowDashboard C++ object.
import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

Rectangle {
  id: board
  Layout.minimumWidth: 380
  Layout.minimumHeight: 260
  anchors.fill: parent
  color: "#20242b"

  readonly property var levelColors: ["#2fb350", "#f2b31a", "#e2341c"]

  ColumnLayout {
    anchors.fill: parent
    anchors.margins: 8
    spacing: 6

    Text {
      text: SwarmFlowDashboard.header
      color: "#a8b0bd"
      font.pixelSize: 12
    }

    ListView {
      id: list
      Layout.fillWidth: true
      Layout.fillHeight: true
      clip: true
      spacing: 6
      model: SwarmFlowDashboard.robots

      delegate: Rectangle {
        width: list.width
        height: col.implicitHeight + 12
        radius: 4
        color: "#2b3038"
        border.color: board.levelColors[Math.max(0, Math.min(2, modelData.level))]
        border.width: 2

        Rectangle {               // robot colour bar
          width: 6
          anchors.top: parent.top
          anchors.bottom: parent.bottom
          anchors.left: parent.left
          radius: 2
          color: modelData.color
        }

        ColumnLayout {
          id: col
          anchors.left: parent.left
          anchors.right: parent.right
          anchors.top: parent.top
          anchors.leftMargin: 14
          anchors.rightMargin: 8
          anchors.topMargin: 6
          spacing: 2

          RowLayout {
            spacing: 8
            Rectangle {
              width: 10; height: 10; radius: 5
              color: board.levelColors[Math.max(0, Math.min(2, modelData.level))]
            }
            Text { text: modelData.name; color: "white"; font.bold: true; font.pixelSize: 14 }
            Text { text: modelData.mode; color: "#d6dbe3"; font.pixelSize: 13 }
            Item { Layout.fillWidth: true }
            Text {
              text: Number(modelData.speed).toFixed(2) + " m/s"
              color: "#a8b0bd"; font.pixelSize: 12
            }
          }
          Text {
            visible: modelData.goal !== ""
            text: "→ " + modelData.goal
            color: "#d6dbe3"; font.pixelSize: 12
            elide: Text.ElideRight; Layout.fillWidth: true
          }
          Text {
            text: (modelData.order !== "" ? "order " + modelData.order + (modelData.order_state !== "" ? " (" + modelData.order_state + ")" : "") : "no order")
                  + (modelData.loaded ? "  ·  loaded" : "")
                  + "  ·  updated " + Number(modelData.age).toFixed(1) + " s ago"
            color: "#a8b0bd"; font.pixelSize: 11
            elide: Text.ElideRight; Layout.fillWidth: true
          }
          Text {
            visible: modelData.fault !== ""
            text: modelData.fault
            color: "#ff7a66"; font.pixelSize: 12; font.bold: true
          }
        }
      }
    }
  }
}
