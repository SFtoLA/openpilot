import QtQuick 2.6
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4
import QtQuick.Layouts 1.0
import QtLocation 5.6
import QtPositioning 5.5

Row {
    property var route_plugin
    property alias startCoordinate: tempGeocodeModel.startCoordinate
    property alias endCoordinate: tempGeocodeModel.endCoordinate

    signal startNavigation();

    layoutDirection: Qt.LeftToRight
    anchors.top: parent.top
    anchors.bottom: parent.bottom
    anchors.right: parent.right

    Button {
        id: sliderToggler
        width: 32
        height: 96
        checkable: true
        checked: true
        anchors.verticalCenter: parent.verticalCenter

        style: ButtonStyle {
            background: Rectangle {
                color: "transparent"
            }
        }

        property real shear: 0.333
        property real buttonOpacity: 0.5

        // an arrow from two sheared rectangles
        Rectangle {
            width: 16
            height: 48
            color: "seagreen"
            antialiasing: true
            opacity: sliderToggler.buttonOpacity
            anchors.top: parent.top
            anchors.left: sliderToggler.checked ? parent.left : parent.horizontalCenter
            transform: Matrix4x4 {
                property real d: sliderToggler.checked ? 1.0 : -1.0
                matrix: Qt.matrix4x4(1.0, d * sliderToggler.shear, 0.0,
                                     0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                                     0.0, 0.0, 0.0, 0.0, 1.0)
            }
        }
        Rectangle {
            width: 16
            height: 48
            color: "seagreen"
            antialiasing: true
            opacity: sliderToggler.buttonOpacity
            anchors.top: parent.verticalCenter
            anchors.right: sliderToggler.checked ? parent.right : parent.horizontalCenter
            transform: Matrix4x4 {
                property real d: sliderToggler.checked ? -1.0 : 1.0
                matrix: Qt.matrix4x4(1.0, d * sliderToggler.shear, 0.0,
                                     0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                                     0.0, 0.0, 0.0, 0.0, 1.0)
            }
        }
    }

    Rectangle {
        id: sliderContainer
        height: parent.height
        width: 300
        visible: sliderToggler.checked
        color: Qt.rgba(1, 1, 1, 0.75)

        RouteAddressForm {
            id: routeAddressForm

            anchors.fill: parent

            GeocodeModel {
                id: tempGeocodeModel

                property int success: 0
                property variant startCoordinate
                property variant endCoordinate

                plugin: route_plugin

                onCountChanged: {
                    if (success == 1 && count == 1) {
                        query = toAddress
                        update()
                    }
                }

                onStatusChanged: {
                    if ((status == GeocodeModel.Ready) && (count == 1)) {
                        success++
                        if (success == 1) {
                            startCoordinate.latitude = get(
                                        0).coordinate.latitude
                            startCoordinate.longitude = get(
                                        0).coordinate.longitude
                        }
                        if (success == 2) {
                            endCoordinate.latitude = get(0).coordinate.latitude
                            endCoordinate.longitude = get(
                                        0).coordinate.longitude
                            success = 0
                            if (startCoordinate.isValid
                                    && endCoordinate.isValid)
                                startNavigation()
                            else
                                goButton.enabled = true
                        }
                    } else if ((status == GeocodeModel.Ready)
                               || (status == GeocodeModel.Error)) {
                        var st = (success == 0) ? "start" : "end"
                        success = 0
                        if ((status == GeocodeModel.Ready) && (count == 0)) {
                            goButton.enabled = true
                        } else if (status == GeocodeModel.Error) {
                            goButton.enabled = true
                        } else if ((status == GeocodeModel.Ready)
                                   && (count > 1)) {
                            goButton.enabled = true
                        }
                    }
                }
            }

            goButton.onClicked: {
                tempGeocodeModel.reset()
                fromAddress.state = fromState.text
                fromAddress.street = fromStreet.text
                fromAddress.city = fromCity.text
                toAddress.state = toState.text
                toAddress.street = toStreet.text
                toAddress.city = toCity.text
                tempGeocodeModel.startCoordinate = QtPositioning.coordinate()
                tempGeocodeModel.endCoordinate = QtPositioning.coordinate()
                tempGeocodeModel.query = fromAddress
                tempGeocodeModel.update()
                sliderToggler.checked = false
            }

            clearButton.onClicked: {
                fromStreet.text = ""
                fromCity.text = ""
                fromState.text = ""
                toStreet.text = ""
                toCity.text = ""
                toState.text = ""
            }

            Component.onCompleted: {
                fromStreet.text = fromAddress.street
                fromCity.text = fromAddress.city
                fromState.text = fromAddress.state
                toStreet.text = toAddress.street
                toCity.text = toAddress.city
                toState.text = toAddress.state
            }
        }
    }

    Address {
        id: fromAddress
        street: "76 2nd St"
        city: "San Francisco"
        state: "CA"
    }

    Address {
        id: toAddress
        street: "180 Woz Way"
        city: "San Jose"
        state: "CA"
    }
}
