import QtQuick 2.9
import QtLocation 5.9
import QtPositioning 5.9
import "cheap-ruler.js" as CheapRuler

Item {
  id: mapWindow
  width: 640
  height: 640

  property variant carPosition: QtPositioning.coordinate()
  property real carBearing: 0
  property bool nightMode: true
  property bool satelliteMode: false
  property bool traffic: true
  property bool navigating: true
  property variant ruler: CheapRuler.cheapRuler(raleigh.coordinate.latitude)


  function coordinateToPoint(coordinate) {
    return [coordinate.longitude, coordinate.latitude]
  }

  // animation durations for continuously updated values shouldn't be much greater than updateInterval
  // ...otherwise, animations are always trying to catch up as their target values change
  property real updateInterval: 100

  Map {
    id: map
    anchors.fill: parent

    plugin: Plugin {
        name: "mapboxgl"

        PluginParameter {
            name: "mapboxgl.mapping.items.insert_before"
            value: "road-label-small"
        }

        PluginParameter {
            name: "mapboxgl.mapping.additional_style_urls"
            value: "mapbox://styles/mapbox/navigation-guidance-day-v2,mapbox://styles/mapbox/navigation-guidance-night-v2,mapbox://styles/mapbox/navigation-preview-day-v2,mapbox://styles/mapbox/navigation-preview-night-v2"
        }
    }

    activeMapType: {
        var style

        if (mapWindow.navigating) {
            style = nightMode ? supportedMapTypes[1] : supportedMapTypes[0]
        } else {
            style = nightMode ? supportedMapTypes[3] : supportedMapTypes[2]
        }

        return style
    }

    center: mapWindow.navigating ? carPosition : map.center
    bearing: mapWindow.navigating ? carBearing : 0
    tilt: mapWindow.navigating ? 60 : 0
    zoomLevel: mapWindow.navigating ? 20 : 12.25
    minimumZoomLevel: 0
    maximumZoomLevel: 20
    copyrightsVisible: false

    // Smooth center and bearing changes
    // TODO commonize with car indicator
    Behavior on center {
      CoordinateAnimation {
        easing.type: Easing.Linear;
        duration: updateInterval;
      }
    }
    Behavior on bearing {
      RotationAnimation {
        direction: RotationAnimation.Shortest
        easing.type: Easing.InOutQuad
        duration: updateInterval
      }
    }

    gesture.enabled: true
    gesture.onPanStarted: {
      mapWindow.navigating = false
    }
    gesture.onPinchStarted: {
      mapWindow.navigating = false
    }
    MouseArea {
        anchors.fill: parent

        onWheel: {
            mapWindow.navigating = false
            wheel.accepted = false
        }
    }
    
    function updateRoute() {
        routeQuery.clearWaypoints();
        routeQuery.addWaypoint(startMarker.coordinate)
        routeQuery.addWaypoint(endMarker.coordinate)
    }

    // startMarker
    MapQuickItem {
        id: startMarker

        sourceItem: Image {
            id: greenMarker
            source: "marker-red.png"
        }

        coordinate: routeAddress.startCoordinate
        anchorPoint.x: greenMarker.width / 2
        anchorPoint.y: greenMarker.height / 2

        MouseArea {
            drag.target: parent
            anchors.fill: parent

            onReleased: {
                map.updateRoute()
            }
        }
    }

    // endMarker
    MapQuickItem {
        id: endMarker

        sourceItem: Image {
            id: redMarker
            source: "marker-red.png"
        }

        coordinate: routeAddress.endCoordinate
        anchorPoint.x: redMarker.width / 2
        anchorPoint.y: redMarker.height / 2

        MouseArea {
            drag.target: parent
            anchors.fill: parent

            onReleased: {
                map.updateRoute()
            }
        }
    }

    // routeModel
    MapItemView {
      id: route
      model: routeModel

      delegate: MapRoute {
        route: routeData
        line.color: "#ec0f73"
        line.width: map.zoomLevel - 5
        opacity: (index == 0) ? 0.8 : 0.3

        onRouteChanged: {
          // console.log("route changed:", JSON.stringify(routeData.segments, null, 2))
        }
      }
    }

    // carMarkerItem
    MapQuickItem {
      id: carMarkerItem
      visible: carPosition.isValid //&& map.zoomLevel > 10
      anchorPoint.x: carMarker.width / 2
      anchorPoint.y: carMarker.height / 2

      opacity: 0.8
      coordinate: carPosition
      rotation: carBearing - map.bearing

      Behavior on coordinate {
        CoordinateAnimation {
          easing.type: Easing.Linear;
          duration: updateInterval;
        }
      }

      sourceItem: Image {
        id: carMarker
        source: "arrow-" + (mapWindow.nightMode ? "night" : "day") + ".svg"
        width: 60 / map.scale
        height: 60 / map.scale
      }
    }

    // -------------------------------------------------------------------------------------

  }

  RouteModel {
    id: routeModel

    autoUpdate: true
    query: routeQuery

    plugin: Plugin {
        name: "mapbox"

        // Development access token, do not use in production.
        PluginParameter {
            name: "mapbox.access_token"
            value: "pk.eyJ1IjoiaGFvd3U4MHMiLCJhIjoiY2tscHZiMTRjMHJoMzJ3b2d0ZjJkankzayJ9.YC_4vDwYAP-IahGKUpSMvg"
        }
    }

    Component.onCompleted: {
        if (map) {
            map.updateRoute()
        }
    }
  }

  RouteQuery {
    id: routeQuery
  }

  RouteAddress {
    id: routeAddress
    z: map.z + 3
    route_plugin: routeModel.plugin
    onStartNavigation: {
        map.updateRoute()
        mapWindow.navigating = true
    }
  }

  Column {
    id: buttons
    anchors.left: parent.left
    anchors.bottom: parent.bottom

    MouseArea {
      id: location
      width: 125
      height: 113
      onClicked: {
        if (carPosition.isValid) {
          navigating = !navigating
        }
      }
      // Rectangle { anchors.fill: parent; color: 'transparent'; border.color: 'yellow'; border.width: 1; } // DEBUG
      Image {
        source: navigating && carPosition.isValid ? "location-active.png" : "location.png"
        opacity: navigating && carPosition.isValid ? 0.5 : 1.0
        width: 63
        height: 63
        anchors.centerIn: parent
        anchors.verticalCenterOffset: -5
        scale: location.pressed ? 0.85 : 1.0
        Behavior on scale { NumberAnimation { duration: 100 } }
      }
    }
  }

  Text {
    id: instructions
    anchors.top: parent.top
    anchors.horizontalCenter: parent.horizontalCenter
    anchors.margins: 20

    font.family: "Inter"
    font.pixelSize: 40
    color: nightMode ? "white" : "black"

    text: "Directions might go here"
  }
}
