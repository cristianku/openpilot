"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The Mapbox token sent from the phone: setMapboxToken and where navd sends the route request with and without it.
"""
import unittest
from unittest.mock import patch

from openpilot.common.api import API_HOST
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.navd.mapbox import MAPBOX_TOKEN_PARAM, valid_mapbox_token
from openpilot.sunnypilot.navd.tests.test_navd_waypoints import ONE_LEG, TWO_LEGS, mapbox_response, route_engine

TOKEN = "pk.eyJ1IjoidGVzdCIsImEiOiJ0ZXN0In0.abc_DEF-123"
DIRECTIONS = "/directions/v5/mapbox/driving-traffic/"


class TestMapboxToken(unittest.TestCase):
  def test_valid_mapbox_token(self):
    self.assertTrue(valid_mapbox_token(TOKEN))
    for invalid in ("", "pk.only-one-part", "sk.eyJ1Ijoi.abc", "tk.eyJ1Ijoi.abc", "pk.a.b c", "pk.a.b\n", None, 42,
                    "pk.a." + "b" * 600):
      self.assertFalse(valid_mapbox_token(invalid), invalid)


class TestRouteEngineMapboxToken(unittest.TestCase):
  def test_without_token_the_request_goes_to_comma(self):
    engine = route_engine({'NavDestination': {'latitude': 0, 'longitude': 0.004}})
    with patch('requests.get', return_value=mapbox_response(ONE_LEG)) as get:
      engine.recompute_route(gps_ok=True)
    self.assertTrue(get.call_args.args[0].startswith(API_HOST.rstrip('/') + '/v1/nav' + DIRECTIONS))
    self.assertEqual(get.call_args.kwargs['params']['access_token'], 'test-token')

  def test_saved_token_goes_straight_to_mapbox(self):
    engine = route_engine({'NavDestination': {'latitude': 0, 'longitude': 0.004}, MAPBOX_TOKEN_PARAM: TOKEN})
    with patch('requests.get', return_value=mapbox_response(ONE_LEG)) as get:
      engine.recompute_route(gps_ok=True)
    self.assertEqual(get.call_args.args[0], 'https://api.mapbox.com' + DIRECTIONS + '0,0;0.004,0')
    self.assertEqual(get.call_args.kwargs['params']['access_token'], TOKEN)
    self.assertEqual(engine.step_idx, 0)

  def test_saved_token_with_waypoints(self):
    values = {'NavDestination': {'latitude': 0, 'longitude': 0.004, 'waypoints': [{'latitude': 0, 'longitude': 0.002, 'via': False}]},
              MAPBOX_TOKEN_PARAM: TOKEN}
    engine = route_engine(values)
    with patch('requests.get', return_value=mapbox_response(TWO_LEGS)) as get:
      engine.recompute_route(gps_ok=True)
    self.assertTrue(get.call_args.args[0].startswith('https://api.mapbox.com' + DIRECTIONS))
    self.assertEqual(get.call_args.kwargs['params']['access_token'], TOKEN)

  def test_token_read_again_for_every_request(self):
    values = {'NavDestination': {'latitude': 0, 'longitude': 0.004}, MAPBOX_TOKEN_PARAM: TOKEN}
    engine = route_engine(values)
    with patch('requests.get', return_value=mapbox_response(ONE_LEG)) as get:
      engine.calculate_route((0.004, 0))
      self.assertTrue(get.call_args.args[0].startswith('https://api.mapbox.com'))

      # removed from the phone while navd runs: back to comma's proxy without a restart
      del values[MAPBOX_TOKEN_PARAM]
      engine.calculate_route((0.004, 0))
      self.assertTrue(get.call_args.args[0].startswith(API_HOST.rstrip('/') + '/v1/nav'))
      self.assertEqual(get.call_args.kwargs['params']['access_token'], 'test-token')


class TestSetMapboxToken(OpenpilotTestCase):
  def test_registered_on_athena_and_sunnylink(self):
    from openpilot.sunnypilot.sunnylink.athena import sunnylinkd
    from openpilot.system.athena import athenad
    assert athenad.dispatcher is sunnylinkd.dispatcher
    assert "setMapboxToken" in athenad.dispatcher

  def test_stores_and_removes_the_token(self):
    from openpilot.system.athena.rpc import dispatcher
    assert dispatcher["setMapboxToken"](TOKEN) == {"success": 1}
    assert Params().get(MAPBOX_TOKEN_PARAM) == TOKEN

    assert dispatcher["setMapboxToken"]("") == {"success": 1}
    assert Params().get(MAPBOX_TOKEN_PARAM) is None

  def test_same_token_is_not_written_again(self):
    from openpilot.system.athena.rpc import dispatcher
    assert dispatcher["setMapboxToken"](TOKEN) == {"success": 1}
    with patch.object(Params, "put") as put:
      assert dispatcher["setMapboxToken"](TOKEN) == {"success": 1}
    put.assert_not_called()

  def test_rejects_secret_and_malformed_tokens(self):
    from openpilot.system.athena.rpc import dispatcher
    Params().put(MAPBOX_TOKEN_PARAM, TOKEN)
    for invalid in ("sk.eyJ1Ijoi.abc", "not a token", None, 12):
      assert dispatcher["setMapboxToken"](invalid)["success"] == 0, invalid
    # a refused token leaves the stored one in place
    assert Params().get(MAPBOX_TOKEN_PARAM) == TOKEN


if __name__ == "__main__":
  unittest.main()
