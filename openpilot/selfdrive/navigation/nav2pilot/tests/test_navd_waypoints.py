"""
Waypoints on top of comma's navd: setNavRoute, the Mapbox request, multi-leg routes and passed waypoints.
"""
import time
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.navigation.navd import RECOMPUTE_HOLD
from openpilot.selfdrive.navigation.nav2pilot.navd import Nav2PilotRouteEngine, leg_steps
from openpilot.selfdrive.navigation.nav2pilot.waypoints import MAX_WAYPOINTS, param_with_waypoints, parse_waypoints, waypoints_from_param

DISTANCE = 111.195  # Mapbox's length of 0.001 degrees at the equator


def step(start, end, maneuver='turn'):
  return {'geometry': {'coordinates': [start, end]}, 'distance': DISTANCE, 'duration': 10, 'maneuver': {'type': maneuver},
          'bannerInstructions': [{'distanceAlongGeometry': 100, 'primary': {'text': 'Go', 'type': maneuver, 'modifier': 'straight'}}]}


def arrive(point):
  return {'geometry': {'coordinates': [point, point]}, 'distance': 0, 'duration': 0, 'maneuver': {'type': 'arrive'}, 'bannerInstructions': []}


def mapbox_response(legs):
  response = MagicMock(status_code=200)
  response.json.return_value = {'routes': [{'legs': [{'steps': steps} for steps in legs]}]}
  return response


# ego (0, 0) -> stop (0.002, 0) -> destination (0.004, 0), along the equator
TWO_LEGS = [
  [step([0, 0], [0.001, 0], 'depart'), step([0.001, 0], [0.002, 0]), arrive([0.002, 0])],
  [step([0.002, 0], [0.003, 0], 'depart'), step([0.003, 0], [0.004, 0]), arrive([0.004, 0])],
]
ONE_LEG = [[step([0, 0], [0.002, 0], 'depart'), step([0.002, 0], [0.004, 0]), arrive([0.004, 0])]]


def route_engine(values):
  params = MagicMock()
  params.get.side_effect = values.get
  params.remove.side_effect = lambda key: values.pop(key, None)
  params.put.side_effect = values.__setitem__
  sm = MagicMock()
  sm.valid = dict.fromkeys(('gpsLocation', 'gpsLocationExternal'), True)
  sm.logMonoTime = dict.fromkeys(sm.valid, time.monotonic_ns())
  sm.recv_time = dict.fromkeys(sm.valid, time.monotonic())
  sm.__getitem__.return_value = SimpleNamespace(hasFix=True, latitude=0, longitude=0, bearingDeg=90)
  with patch('openpilot.selfdrive.navigation.navd.Api') as api:
    api.return_value.get_token.return_value = 'test-token'
    engine = Nav2PilotRouteEngine(sm, MagicMock(), params=params)
  engine.last_position, engine.last_bearing = (0, 0), 90
  return engine


class TestWaypointParam(unittest.TestCase):
  def test_parse(self):
    self.assertEqual(parse_waypoints(None), [])
    self.assertEqual(parse_waypoints([{'latitude': 1, 'longitude': 2}]), [{'latitude': 1.0, 'longitude': 2.0, 'via': False}])
    self.assertEqual(parse_waypoints([{'latitude': 1, 'longitude': 2, 'via': True}])[0]['via'], True)
    for invalid in ('x', [1], [{'latitude': 91, 'longitude': 0}], [{'latitude': True, 'longitude': 0}],
                    [{'latitude': float('nan'), 'longitude': 0}], [{'latitude': 0, 'longitude': 0, 'via': 'yes'}],
                    [{'latitude': 0, 'longitude': 0}] * (MAX_WAYPOINTS + 1)):
      self.assertIsNone(parse_waypoints(invalid), invalid)

  def test_round_trip(self):
    destination = {'latitude': 0, 'longitude': 0.004, 'waypoints': [{'latitude': 0, 'longitude': 0.002, 'via': True}]}
    waypoints = waypoints_from_param(destination)
    self.assertEqual(waypoints, ((0.002, 0.0, True),))
    self.assertEqual(param_with_waypoints(destination, waypoints), destination)
    self.assertEqual(param_with_waypoints(destination, ()), {'latitude': 0, 'longitude': 0.004})
    self.assertEqual(waypoints_from_param({'latitude': 0, 'longitude': 0}), ())
    self.assertEqual(waypoints_from_param(None), ())

  def test_leg_steps_drop_intermediate_arrivals(self):
    steps = leg_steps([{'steps': legs} for legs in TWO_LEGS])
    self.assertEqual([s['maneuver']['type'] for s in steps], ['depart', 'turn', 'depart', 'turn', 'arrive'])


class TestRouteEngineWaypoints(unittest.TestCase):
  def test_without_waypoints_the_request_is_upstreams(self):
    values = {'NavDestination': {'latitude': 0, 'longitude': 0.004}}
    engine = route_engine(values)
    with patch('requests.get', return_value=mapbox_response(ONE_LEG)) as get:
      engine.recompute_route(gps_ok=True)
    self.assertTrue(get.call_args.args[0].endswith('/driving-traffic/0,0;0.004,0'))
    self.assertNotIn('waypoints', get.call_args.kwargs['params'])
    self.assertEqual(get.call_args.kwargs['params']['bearings'], '90,90;')

  def test_request_with_a_stop_and_a_via_point(self):
    values = {'NavDestination': {'latitude': 0, 'longitude': 0.004, 'waypoints': [
      {'latitude': 0, 'longitude': 0.001, 'via': True}, {'latitude': 0, 'longitude': 0.002, 'via': False}]}}
    engine = route_engine(values)
    with patch('requests.get', return_value=mapbox_response(TWO_LEGS)) as get:
      engine.recompute_route(gps_ok=True)

    self.assertTrue(get.call_args.args[0].endswith('/driving-traffic/0,0;0.001,0.0;0.002,0.0;0.004,0'))
    options = get.call_args.kwargs['params']
    self.assertEqual(options['waypoints'], '0;2;3')  # the via point (index 1) is not a leg boundary
    self.assertEqual(options['bearings'], '90,90;;;')  # one entry per coordinate
    self.assertEqual(len(engine.route), 5)

  def test_follows_both_legs_through_the_stop(self):
    values = {'NavDestination': {'latitude': 0, 'longitude': 0.004, 'waypoints': [{'latitude': 0, 'longitude': 0.002}]}}
    engine = route_engine(values)
    with patch('requests.get', return_value=mapbox_response(TWO_LEGS)):
      engine.recompute_route(gps_ok=True)

    for position, expected_step in [((0.0012, 0), 1), ((0.0022, 0), 2), ((0.0032, 0), 3)]:
      engine.last_position = position
      engine.send_instruction()
      self.assertEqual(engine.step_idx, expected_step, position)
    instruction = engine.pm.send.call_args.args[1].navInstruction
    self.assertAlmostEqual(instruction.distanceRemaining, DISTANCE * 0.8, delta=1)

  def test_passed_waypoint_is_dropped_without_a_new_request(self):
    values = {'NavDestination': {'latitude': 0, 'longitude': 0.004, 'waypoints': [{'latitude': 0, 'longitude': 0.002}]}}
    engine = route_engine(values)
    with patch('requests.get', return_value=mapbox_response(TWO_LEGS)):
      engine.recompute_route(gps_ok=True)

    engine.last_position = (0.0021, 0)  # 11 m past the stop
    engine.recompute_hold = 0
    with patch('requests.get') as get:
      engine.recompute_route(gps_ok=True)
      get.assert_not_called()
    self.assertEqual(values['NavDestination'], {'latitude': 0, 'longitude': 0.004})
    self.assertEqual(engine.nav_waypoints, ())

  def test_reroute_skips_passed_waypoints(self):
    values = {'NavDestination': {'latitude': 0, 'longitude': 0.004, 'waypoints': [
      {'latitude': 0, 'longitude': 0.001, 'via': True}, {'latitude': 0, 'longitude': 0.003, 'via': True}]}}
    engine = route_engine(values)
    with patch('requests.get', return_value=mapbox_response(ONE_LEG)):
      engine.recompute_route(gps_ok=True)

    engine.last_position = (0.001, 0)  # at the first via point: passed
    engine.recompute_route(gps_ok=True)
    engine.last_position = (0.0015, 0.001)  # then 111 m off route: reroute through the second via point only
    engine.recompute_hold = 0
    engine.reroute_counter = 10
    with patch('requests.get', return_value=mapbox_response(ONE_LEG)) as get:
      engine.recompute_route(gps_ok=True)
    self.assertTrue(get.call_args.args[0].endswith('/driving-traffic/0.0015,0.001;0.003,0.0;0.004,0'))

  def test_new_waypoints_for_the_same_destination_recompute(self):
    values = {'NavDestination': {'latitude': 0, 'longitude': 0.004}}
    engine = route_engine(values)
    with patch('requests.get', return_value=mapbox_response(ONE_LEG)):
      engine.recompute_route(gps_ok=True)

    values['NavDestination'] = {'latitude': 0, 'longitude': 0.004, 'waypoints': [{'latitude': 0.001, 'longitude': 0.002}]}
    engine.recompute_hold = 0
    with patch('requests.get', return_value=mapbox_response(TWO_LEGS)) as get:
      engine.recompute_route(gps_ok=True)
    get.assert_called_once()
    self.assertEqual(engine.recompute_hold, RECOMPUTE_HOLD)

    values['NavDestination'] = {'latitude': 0, 'longitude': 0.004}  # and back without them (comma's setNavDestination)
    engine.recompute_hold = 0
    with patch('requests.get', return_value=mapbox_response(ONE_LEG)) as get:
      engine.recompute_route(gps_ok=True)
    get.assert_called_once()
    self.assertEqual(engine.nav_waypoints, ())

  def test_cleared_destination_clears_waypoints(self):
    values = {'NavDestination': {'latitude': 0, 'longitude': 0.004, 'waypoints': [{'latitude': 0.001, 'longitude': 0.002}]}}
    engine = route_engine(values)
    with patch('requests.get', return_value=mapbox_response(TWO_LEGS)):
      engine.recompute_route(gps_ok=True)
    values.pop('NavDestination')
    engine.recompute_route(gps_ok=True)
    self.assertIsNone(engine.route)
    self.assertEqual(engine.nav_waypoints, ())


class TestSetNavRoute(OpenpilotTestCase):
  def test_registered_on_athena(self):
    from openpilot.system.athena import athenad
    assert "setNavRoute" in athenad.dispatcher

  def test_stores_destination_and_waypoints(self):
    from openpilot.system.athena.rpc import dispatcher
    result = dispatcher["setNavRoute"](45.47, 9.19, [{"latitude": 45.4, "longitude": 9.1, "via": True}])
    assert result == {"success": 1}
    assert Params().get("NavDestination") == {"latitude": 45.47, "longitude": 9.19,
                                              "waypoints": [{"latitude": 45.4, "longitude": 9.1, "via": True}]}

    assert dispatcher["setNavRoute"](45.47, 9.19) == {"success": 1}
    assert Params().get("NavDestination") == {"latitude": 45.47, "longitude": 9.19}

  def test_rejects_invalid_input(self):
    from openpilot.system.athena.rpc import dispatcher
    Params().remove("NavDestination")
    assert dispatcher["setNavRoute"](95, 9.19)["success"] == 0
    assert dispatcher["setNavRoute"](45.47, 9.19, [{"latitude": 45.4}])["success"] == 0
    assert dispatcher["setNavRoute"](45.47, 9.19, "not a list")["success"] == 0
    assert Params().get("NavDestination") is None
