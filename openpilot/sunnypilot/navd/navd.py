#!/usr/bin/env python3
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

comma's navd (openpilot/selfdrive/navigation/navd.py, kept as in upstream) plus the waypoints of the NavDestination
param (see waypoints.py). Without waypoints it behaves exactly like upstream: the methods hand over to RouteEngine.

With waypoints:
- the Mapbox request goes ego -> waypoints -> destination, stops as waypoints and via points as silent via points
- the steps of every leg are followed in order, without the "arrive" step that ends each intermediate leg
- a waypoint closer than PASSED_DISTANCE is passed: it and those before it are dropped, also from the param,
  so neither a reroute nor a restarted navd leads back to it

With a Mapbox token sent from the phone (MapboxToken param, see mapbox.py) every route request goes straight to
Mapbox with it; without one navd keeps upstream's choice: the MAPBOX_TOKEN variable, else comma's proxy.
"""
import numpy as np
import requests

import openpilot.cereal.messaging as messaging
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.navigation.navd import RECOMPUTE_HOLD, RouteEngine
from openpilot.selfdrive.navigation.route import local_meters
from openpilot.sunnypilot.navd.mapbox import MAPBOX_HOST, MAPBOX_TOKEN_PARAM
from openpilot.sunnypilot.navd.waypoints import DESTINATION_PARAM, param_with_waypoints, waypoints_from_param

PASSED_DISTANCE = 50  # meters; at 130 km/h and 1 Hz the ego is sampled every 36 m


def leg_steps(legs) -> list:
  """The steps of all legs in driving order, dropping the "arrive" step that ends each leg but the last."""
  steps = []
  for index, leg in enumerate(legs):
    leg_steps = list(leg['steps'])
    is_last_leg = index == len(legs) - 1
    if not is_last_leg and leg_steps and leg_steps[-1].get('maneuver', {}).get('type') == 'arrive':
      leg_steps = leg_steps[:-1]
    steps.extend(leg_steps)
  return steps


class RouteEngineSP(RouteEngine):
  def __init__(self, sm, pm, params=None):
    super().__init__(sm, pm, params)
    self.nav_waypoints = ()  # the waypoints of the current route, (longitude, latitude, via)
    # upstream's choice, used while no token is saved from the phone
    self.upstream_mapbox_token = self.mapbox_token
    self.upstream_mapbox_host = self.mapbox_host

  def select_mapbox_token(self):
    """Before each route request: the token saved from the phone, which can change while navd runs."""
    saved_token = self.params.get(MAPBOX_TOKEN_PARAM)
    if saved_token:
      self.mapbox_token = saved_token
      self.mapbox_host = MAPBOX_HOST
    else:
      self.mapbox_token = self.upstream_mapbox_token
      self.mapbox_host = self.upstream_mapbox_host

  def recompute_route(self, gps_ok: bool):
    if self.last_position is None:
      return

    destination = self.params.get(DESTINATION_PARAM)
    waypoints = self.drop_passed_waypoints(destination, waypoints_from_param(destination))
    if not waypoints and not self.nav_waypoints:
      super().recompute_route(gps_ok)
      return
    if destination is None:
      super().recompute_route(gps_ok)  # clears the route
      return
    new_destination = (destination['longitude'], destination['latitude'])

    # Same rules as upstream: no recompute while the GPS drifts in tunnels, and at most one every RECOMPUTE_HOLD
    if not gps_ok and self.step_idx is not None:
      return

    self.recompute_hold -= 1
    changed = new_destination != self.nav_destination or waypoints != self.nav_waypoints
    if (self.should_recompute() or changed) and self.recompute_hold <= 0:
      self.recompute_hold = RECOMPUTE_HOLD
      self.calculate_route(new_destination, waypoints)
      self.reroute_counter = 0

  def drop_passed_waypoints(self, destination, waypoints):
    """The waypoints still ahead: up to the last one within PASSED_DISTANCE they are dropped, here and in the param."""
    if not waypoints or self.last_position is None:
      return waypoints

    passed = 0
    for index, (longitude, latitude, _) in enumerate(waypoints):
      if np.linalg.norm(local_meters((longitude, latitude), self.last_position)) < PASSED_DISTANCE:
        passed = index + 1
    if passed == 0:
      return waypoints

    remaining = waypoints[passed:]
    cloudlog.warning(f"Passed {passed} waypoint(s), {len(remaining)} left")
    # the route already goes past them: dropping them must not trigger a new route request
    if self.nav_waypoints == waypoints:
      self.nav_waypoints = remaining
    self.params.put(DESTINATION_PARAM, param_with_waypoints(destination, remaining))
    return remaining

  def calculate_route(self, destination, waypoints=()):
    self.select_mapbox_token()
    if not waypoints:
      self.nav_waypoints = ()
      super().calculate_route(destination)
      return

    # upstream calculate_route with the waypoints added to the request
    assert self.last_position is not None
    cloudlog.warning(f"Calculating route {self.last_position} -> {len(waypoints)} waypoint(s) -> {destination}")
    self.nav_destination = destination
    self.nav_waypoints = tuple(waypoints)

    lang = self.params.get('LanguageSetting')
    if lang is not None:
      lang = lang.replace('main_', '')

    token = self.mapbox_token
    if token is None:
      token = self.api.get_token()

    coordinates = [self.last_position] + [(longitude, latitude) for longitude, latitude, _ in waypoints] + [destination]
    # Indices of the coordinates that start or end a leg: the ego, every stop and the destination
    leg_indices = [0] + [index + 1 for index, (_, _, via) in enumerate(waypoints) if not via] + [len(coordinates) - 1]

    params = {
      'access_token': token,
      'geometries': 'geojson',
      'overview': 'full',
      'steps': 'true',
      'banner_instructions': 'true',
      'alternatives': 'false',
      'language': lang,
      'waypoints': ';'.join(str(index) for index in leg_indices),
    }
    if self.last_bearing is not None:
      # one entry per coordinate: the ego's bearing, then no constraint
      params['bearings'] = f"{(self.last_bearing + 360) % 360:.0f},90" + ';' * (len(coordinates) - 1)

    coords_str = ';'.join(f'{lon},{lat}' for lon, lat in coordinates)
    url = self.mapbox_host + '/directions/v5/mapbox/driving-traffic/' + coords_str
    try:
      resp = requests.get(url, params=params, timeout=10)
      if resp.status_code != 200:
        cloudlog.event("API request failed", status_code=resp.status_code, text=resp.text, error=True)
      resp.raise_for_status()

      r = resp.json()
      if len(r['routes']):
        self.route = leg_steps(r['routes'][0]['legs'])
        self.route_geometry = [np.array(step['geometry']['coordinates'], dtype=np.float64).reshape((-1, 2)) for step in self.route]
        self.step_idx = 0
      else:
        cloudlog.warning("Got empty route response")
        self.clear_route()

    except requests.exceptions.RequestException:
      cloudlog.exception("failed to get route")
      self.clear_route()

  def clear_route(self):
    super().clear_route()
    self.nav_waypoints = ()


def main():
  pm = messaging.PubMaster(['navInstruction', 'navRoute'])
  sm = messaging.SubMaster(['gpsLocation', 'gpsLocationExternal'])

  rk = Ratekeeper(1.0)
  route_engine = RouteEngineSP(sm, pm)
  while True:
    route_engine.update()
    rk.keep_time()


if __name__ == "__main__":
  main()
