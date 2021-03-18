#!/usr/bin/env python3
#   import os
from collections import deque

import requests

import cereal.messaging as messaging
from common.basedir import PERSIST
from selfdrive.config import Conversions as CV
from selfdrive.swaglog import cloudlog

import math
import overpy
import threading
import time
from collections import defaultdict
from common.transformations.coordinates import geodetic2ecef
import numpy as np
from scipy import spatial


class MessagedGPSThread(threading.Thread):
  def __init__(self, threadID, name, sharedParams={}):
    # invoke parent constructor
    threading.Thread.__init__(self)
    self.sharedParams = sharedParams
    self.sm = messaging.SubMaster(['gpsLocationExternal'])
  def run(self):
    print("run message gps thread")
    gps = None
    start = time.time()
    while True:
        if time.time() - start > 0.2:
            print("Mapd MessagedGPSThread lagging by: %s" % str(time.time() - start - 0.1))
        if time.time() - start < 0.1:
            time.sleep(0.01)
            continue
        else:
            start = time.time()
        self.sm.update(0)
        gps = messaging.new_message('gpsLocationExternal').gpsLocationExternal

        # if self.sm.updated['gpsLocationExternal']:
        #     gps = self.sm['gpsLocationExternal']

        query_lock = self.sharedParams.get('query_lock', None)
        # fake gps
        #TODO: remove
        gps.latitude = -122.4782
        gps.longitude = 37.6715
        gps.altitude = 20
        gps.accuracy = 2.0
        gps.flags = 1
        query_lock.acquire()
        self.sharedParams['last_gps'] = gps
        query_lock.release()


class QueryThread(threading.Thread):
  def __init__(self, threadID, name, sharedParams={}):
    threading.Thread.__init__(self)
    self.threadID = threadID
    self.name = name
    self.sharedParams = sharedParams
    self.frequency = 1.0
    self.query_radius = 1000 # meters
    self.prev_ecef = None
    self.distance_to_edge = 500
    self.OVERPASS_API_URL = "https://z.overpass-api.de/api/interpreter"
    self.mapbox_access_token = "pk.eyJ1IjoiYWxiZXJ0bHVkd2lnIiwiYSI6ImNra3Q4bW5ydDB1dHYydXJ0eHYyNGI5cmEifQ.BqCUvH3GvayncqLhqaA9lA"
 
  def build_way_query(self, lat, lon, bearing, radius=50):
    # start: 37.671104353440676, -122.46608250509264
    # end: 37.73095554576629, -122.47186631934458
    a = 111132.954*math.cos(float(lat)/180*3.141592)
    b = 111132.954 - 559.822 * math.cos( 2 * float(lat)/180*3.141592) + 1.175 * math.cos( 4 * float(lat)/180*3.141592)
    heading = math.radians(-bearing + 90)
    lat = lat+math.sin(heading)*radius/2/b
    lon = lon+math.cos(heading)*radius/2/a
    pos = "  (around:%f,%f,%f)" % (radius, lat, lon)
    lat_lon = "(%f,%f)" % (lat, lon)
    q = """(
    way
    """ + pos + """
    [highway][highway!~"^(footway|path|bridleway|steps|cycleway|construction|bus_guideway|escape)$"];
    >;);out;""" + """is_in""" + lat_lon + """;area._[admin_level~"[24]"];
    convert area ::id = id(), admin_level = t['admin_level'],
    name = t['name'], "ISO3166-1:alpha2" = t['ISO3166-1:alpha2'];out;
    """
    # self.logger.debug("build_way_query : %s" % str(q))
    return q, lat, lon
    """Builds a query to find all highways within a given radius around a point"""
    # latest_gps = gps_entries[-1]
    lat = latest_gps.latitude
    lon = latest_gps.longitude
    bearing = latest_gps.bearing
    a = 111132.954*math.cos(float(lat)/180*3.141592)
    b = 111132.954 - 559.822 * math.cos( 2 * float(lat)/180*3.141592) + 1.175 * math.cos( 4 * float(lat)/180*3.141592)
    heading = math.radians(-bearing + 90)
    lat = lat+math.sin(heading)*radius/2/b
    lon = lon+math.cos(heading)*radius/2/a
    pos = "  (around:%f,%f,%f)" % (radius, lat, lon)
    lat_lon = "(%f,%f)" % (lat, lon)
    q = """(
    way
    """ + pos + """
    [highway][highway!~"^(footway|path|bridleway|steps|cycleway|construction|bus_guideway|escape)$"];
    >;);out;""" + """is_in""" + lat_lon + """;area._[admin_level~"[24]"];
    convert area ::id = id(), admin_level = t['admin_level'],
    name = t['name'], "ISO3166-1:alpha2" = t['ISO3166-1:alpha2'];out;
    """
    # self.logger.debug("build_way_query : %s" % str(q))
    return q, lat, lon

  def query_mapbox(self, lat, lon, radius=50):
    # start (lat, long): 37.67108699534605, -122.46597571343007
    # end (lat, long): 37.73095554576629, -122.47186631934458
    # start(lat, long) 37.67111047963081, -122.46604225845272
    # end (lat, long): 37.6730376062111, -122.47695356766843
    cur_lat = 37.67111047963081
    cur_lon = -122.46604225845272
    target_lat = 37.6730376062111
    target_lon = -122.47695356766843
    data = {
        'coordinates': (f"{cur_lon}, {cur_lat};{target_lon}, {target_lat}"), # mapbox uses (long, lat) convention
        'overview': 'full',
        'geometries': 'geojson',
        'steps': 'true'
    }
    # print("printing nav data")
    # print(data)
    route_response = requests.post('https://api.mapbox.com/directions/v5/mapbox/driving?access_token=' + self.mapbox_access_token, data=data, timeout=10)
    route_json = route_response.json()
    # print(route_json)
    return route_json

  def run(self):
    period = 1.0 / self.frequency
    start = time.time()
    while True:
      if time.time() - start > period + 2.0:
        print("Mapd QueryThread lagging by: %s" % str(time.time() - start - period))
      if time.time() - start < period: # runs every 10 sec
        time.sleep(0.1)
        continue
      else:
        print("query thread runs")
        start = time.time()
      last_gps = self.sharedParams.get('last_gps', None)
      if last_gps is not None:
        # last_gps.latitude = -122.478232
        # last_gps.longitude = 37.669286
        # last_gps.altitude = 20
        # last_gps.accuracy = 2.0
        print(f'query thread gps: {last_gps}')
        # fix_ok = last_gps.flags & 1
        # if not fix_ok:
        #     continue
      else:
        continue
      #lat, long = self.build_mapbox_query(37.3176, 121.9477, self.query_radius)

      last_query_pos = self.sharedParams.get('last_query_pos', None)
      if last_query_pos is not None:
        cur_ecef = geodetic2ecef((last_gps.latitude, last_gps.longitude, last_gps.altitude))
        if self.prev_ecef is None:
            self.prev_ecef = geodetic2ecef((last_query_pos.latitude, last_query_pos.longitude, last_query_pos.altitude))
        dist_to_last_query = np.linalg.norm(cur_ecef - self.prev_ecef)
        if dist_to_last_query < self.query_radius - self.distance_to_edge: #updated when we are close to the edge of the downloaded circle
            continue
        if dist_to_last_query > self.query_radius:
          query_lock = self.sharedParams.get('query_lock', None)
          if query_lock is not None:
              query_lock.acquire()
              self.sharedParams['cache_valid'] = False
              query_lock.release()
          else:
              print("There is no query_lock")
      # gps_valid = True
      # if gps_valid:
      if last_gps is not None and last_gps.accuracy < 5.0:
        # q, lat, long = self.build_way_query(37.3176, 121.9477, 100, self.query_radius)
        # api = overpy.Overpass(url=OVERPASS_API_URL)
        # api = overpy.Overpass()
        try:
          query_start_time = time.time()
          # new_result = api.query(q)
          new_result = self.query_mapbox(37.3176, 121.9477, self.query_radius)
          query_end_time = time.time()
          print(f"query takes {query_end_time - query_start_time} secs")
          # print(str(new_result))
          route = new_result['routes'][0]
          # print(f"route distance is: {route['distance']}, duration is {route['duration']} ")
          coords = np.asarray(route['geometry']['coordinates'])
          altitudes = np.zeros((coords.shape[0], 1))
          coords = np.hstack((coords, altitudes))
          coords = geodetic2ecef(coords)
          tree = spatial.KDTree(coords)
          print(f'query thread, {str(coords)}, {str(tree)}')
          # for coord in coords:
          #   lat_coord = coord[0]
          #   lon_coord = coord[1]
            # print(f'coordinate is: {lat_coord, lon_coord}')
          # for leg in route['legs']:
          #   print(f"leg duration is {leg['duration']}, distance is {leg['distance']}, summary is: {leg['summary']}")
          #   # print(leg['steps'][0])
          #   for step in leg['steps']:
          #     # print(step)
          #     # print(step['maneuver'])
          #     maneuver = step['maneuver']
          #     bearing_before = maneuver['bearing_before']
          #     bearing_after = maneuver['bearing_after']
          #     phi = abs(bearing_after - bearing_before) % 360
          #     bearing_diff = phi if phi < 180 else (360 - phi) # degree
              # print(f"step maneuver type: {step['maneuver']['type']}, with bearing difference {bearing_diff}")
              # print("===================================================")
            # print(route)

          # write result
          query_lock = self.sharedParams.get('query_lock', None)
          if query_lock is not None:
            query_lock.acquire()
            self.sharedParams['last_query_result'] = new_result, tree, coords
            self.sharedParams['last_query_pos'] = last_gps
            self.prev_ecef = geodetic2ecef((last_gps.latitude, last_gps.longitude, last_gps.altitude))
            self.sharedParams['cache_valid'] = True
            query_lock.release()
          else:
            print("failed to get query_lock")
        except Exception as e:
          print(str(e))
          query_lock = self.sharedParams.get('query_lock', None)
          query_lock.acquire()
          self.sharedParams['last_query_result'] = None
          query_lock.release()
      else:
        query_lock = self.sharedParams.get('query_lock', None)
        query_lock.acquire()
        self.sharedParams['last_query_result'] = None
        query_lock.release()

class MapsdThread(threading.Thread):
  def __init__(self, threadID, name, sharedParams={}):
    threading.Thread.__init__(self)
    self.threadID = threadID
    self.name = name
    self.sharedParams = sharedParams
    self.pm = messaging.PubMaster(['liveMapData'])
    self.period = 0.5 #sec
    self.max_lookahead_dist = 300 # meters. TODO: use a function of speed and time
  
  # def points_in_car_frame(self, lat, lon, heading, flip):
  #   lc = LocalCoord.from_geodetic([lat, lon, 0.])

  #   # Build rotation matrix
  #   heading = math.radians(-heading + 90)
  #   c, s = np.cos(heading), np.sin(heading)
  #   rot = np.array([[c, s, 0.], [-s, c, 0.], [0., 0., 1.]])

  #   # Convert to local coordinates
  #   points_carframe = lc.geodetic2ned(self.points).T

  #   # Rotate with heading of car
  #   points_carframe = np.dot(rot, points_carframe[(1, 0, 2), :]).T

  #   if points_carframe[-1,0] < points_carframe[0,0] and flip:
  #     points_carframe = np.flipud(points_carframe)

  #   return points_carframe
  
  def run(self):
    cur_way = None
    curvature_valid = False
    curvature = None
    upcoming_curvature = 0.
    dist_to_turn = 0.
    road_points = None
    max_speed = None
    max_speed_ahead = None
    max_speed_ahead_dist = None
    max_speed_prev = 0
    had_good_gps = False 
    start = time.time()
    while True:
      if time.time() - start > self.period + 0.5:
          print("Mapd MapsdThread lagging by: %s" % str(time.time() - start - self.period))
      if time.time() - start < self.period:
          time.sleep(0.1)
          continue
      else:
          start = time.time()
      query_lock = self.sharedParams.get('query_lock', None)
      query_lock.acquire()
      gps = self.sharedParams['last_gps']
      print(f'mapd thread gps: {gps}')
      query_lock.release()
      if gps is None:
        continue
      fix_ok = gps.flags & 1
      if self.sharedParams['last_query_result'] and fix_ok and self.sharedParams['cache_valid']:
        map_valid = True
        lat = gps.latitude
        lon = gps.longitude
        heading = gps.bearingDeg
        speed = gps.speed
        cur_pos = geodetic2ecef((lat, lon, 0))
        print(f'mapd thread current pos: {cur_pos}')

        # query_lock.acquire()
        # find the current way that car is on.
        # cur_way = Way.closest(self.sharedParams['last_query_result'], lat, lon, heading, cur_way)
        query_result, tree, coords = self.sharedParams['last_query_result']
        print(f'mapd thread tree: {tree}')
        closest_dist, closest_index = tree.query(cur_pos, 1)
        print(f'mapd thread closest_dist: {closest_dist}')
        print(f'mapd thread closest_index: {closest_index}')
        print(f'mapd thread closet piont is: {tree.data[closest_index]}, query coord is: {cur_pos}')
        print(f'KD tree data shape: {tree.data.shape}')
        accum_dist = 0
        # prev_coord = None
        # coord_x = None
        # coord_y = None
        # for coord in coords[closest_index:]:
        #   if prev_coord is None:
        #     prev_coord = coord
        #     continue
        #   accum_dist += np.linalg.norm(prev_coord - coord)
        #   prev_coord = coord
        #   print(f'dist ahead is: {accum_dist}')
        #   if accum_dist > self.max_lookahead_dist:
        #     break
        last_index = closest_index
        for i in range(closest_index, tree.data.shape[0]):
          if i == closest_index:
            continue
          accum_dist += np.linalg.norm(coords[i] - coords[i-1])
          last_index = i
          print(f'dist ahead is: {accum_dist}')
          if accum_dist > self.max_lookahead_dist:
            break
        # print(f'start index is: {closest_index} last index is: {last_index}')
        # print(f'x corrds: {coords[closest_index:last_index, 0]} ')
        # print(f'y corrds: {coords[closest_index:last_index, 1]} ')
        coeff = np.polyfit(coords[closest_index:last_index, 0], coords[closest_index:last_index, 1], 3)
        # print(f'polyline coeff is: {coeff}')
        # Curvature of polynomial https://en.wikipedia.org/wiki/Curvature#Curvature_of_the_graph_of_a_function
        # y = a x^3 + b x^2 + c x + d, y' = 3 a x^2 + 2 b x + c, y'' = 6 a x + 2 b
        # k = y'' / (1 + y'^2)^1.5
        # TODO: compute max speed without using a list of points and without numpy
        path_x = np.arange(accum_dist)
        y_p = 3 * coeff[0] * path_x**2 + 2 * coeff[1] * path_x + coeff[2]
        y_pp = 6 * coeff[0] * path_x + 2 * coeff[1]
        curv = y_pp / (1. + y_p**2)**1.5
        curv = np.clip(np.abs(curv), 1e-6, None)
        max_curv = np.max(curv)
        print(f'------------- max curvature is: {max_curv}')

        # query_lock.release()

      else:
        curvature = None
        max_speed_ahead = None
        max_speed_ahead_dist = None
        curvature_valid = False
        upcoming_curvature = 0.
        dist_to_turn = 0.
        map_valid = False

def main(sm=None, pm=None):
  print("in main")
  # setup shared parameters
  last_gps = None
  query_lock = threading.Lock()
  last_query_result = None
  cache_valid = False

  sharedParams = {'last_gps' : last_gps, \
                  'query_lock' : query_lock, \
                  'last_query_result' : last_query_result, \
                  'cache_valid' : cache_valid}
  qt = QueryThread(1, "QueryThread", sharedParams=sharedParams)
  mggps = MessagedGPSThread(2, "MessagedGPSThread", sharedParams=sharedParams)
  mt = MapsdThread(2, "MapsdThread", sharedParams=sharedParams)
  qt.start()
  mggps.start()
  mt.start()

if __name__ == "__main__":
  main()