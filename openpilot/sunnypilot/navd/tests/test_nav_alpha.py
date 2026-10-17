"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

sunnypilot integration checks for comma's Nav Alpha (openpilot/selfdrive/navigation). The upstream tests mock the
comma API, so these use sunnypilot's own Api wrapper, process list and sunnylink dispatcher.
"""
from unittest.mock import MagicMock, patch

import requests

from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.navd.destination import format_destination, parse_destination, set_destination_from_text


class TestApiCompat(OpenpilotTestCase):
  def test_api_exposes_upstream_interface(self):
    from openpilot.common.api import API_HOST, Api
    from openpilot.common.api.comma_connect import API_HOST as COMMA_API_HOST
    assert API_HOST == COMMA_API_HOST
    assert Api("0123456789abcdef").dongle_id == "0123456789abcdef"

  def test_navigation_daemons_import(self):
    from openpilot.selfdrive.navigation import mapsd, navd
    assert callable(navd.main)
    assert callable(mapsd.main)

  def test_remote_tiles_with_sunnypilot_api(self):
    from openpilot.selfdrive.navigation.tiles import RemoteTiles
    Params().put("DongleId", "0123456789abcdef", block=True)
    requested = []

    def fake_api_get(self, endpoint, method='GET', timeout=None, access_token=None, session=None, json=None, **params):
      requested.append((endpoint, access_token))
      raise requests.ConnectionError("offline")

    with patch("openpilot.common.api.base.BaseApi.get_token", return_value="device-token"), \
         patch("openpilot.common.api.base.BaseApi.api_get", fake_api_get):
      # no connectivity is an OSError for mapsd to retry, not an AttributeError from the Api wrapper
      with self.assertRaises(OSError):
        RemoteTiles()
    assert requested == [("v1/0123456789abcdef/nav/tiles", "device-token")]

  def test_unregistered_device_does_not_crash_daemons(self):
    # Without a device key sunnypilot's BaseApi asserts, where comma's Api raises RuntimeError
    from openpilot.selfdrive.navigation.mapsd import RETRY_SECONDS, MapsD
    from openpilot.selfdrive.navigation.navd import RouteEngine
    from openpilot.selfdrive.navigation.renderer import SIZE
    params = Params()
    params.put("DongleId", "0123456789abcdef", block=True)
    params.put("NavDestination", {"latitude": 45.47, "longitude": 9.19}, block=True)

    gps = MagicMock(longitude=9.18, latitude=45.46, bearingDeg=0.0)
    sm = MagicMock(frame=0)
    sm.__getitem__.side_effect = lambda service: gps
    pm = MagicMock()
    with patch("openpilot.common.api.base.BaseApi.get_key_pair", return_value=(None, None, None)), \
         patch("openpilot.selfdrive.navigation.navd.latest_gps_source", return_value="gpsLocation"):
      engine = RouteEngine(sm, pm, params=params)
      engine.update()  # the failed token is logged, not raised
      engine.update()  # held off from retrying, it reports no route
    sent = {call.args[0]: call.args[1] for call in pm.send.call_args_list}
    assert not sent["navInstruction"].valid
    assert len(sent["navRoute"].navRoute.coordinates) == 0

    route = MagicMock(coordinates=[MagicMock(longitude=9.18, latitude=45.46), MagicMock(longitude=9.19, latitude=45.47)])
    sm = MagicMock()
    sm.updated = {"navRoute": True}
    sm.__getitem__.side_effect = lambda service: route
    mapsd = MapsD()
    with patch("openpilot.common.api.base.BaseApi.get_key_pair", return_value=(None, None, None)), \
         patch("openpilot.selfdrive.navigation.mapsd.latest_gps_source", return_value=None):
      image, timestamp = mapsd.update(sm)  # the failed tile fetch backs off instead of crashing
    assert timestamp == 0 and image.shape == (SIZE, SIZE) and not image.any()
    assert mapsd.renderer is None and mapsd.retry_at > 0
    assert RETRY_SECONDS > 0


class TestUiSubscriptions(OpenpilotTestCase):
  def test_ui_services_unique(self):
    # sunnypilot's UIStateSP already subscribes gpsLocation. Subscribing it twice delivers every message twice per
    # update, and SubMaster's frequency tracker then divides by zero, crashing the UI whenever GPS publishes.
    from openpilot.selfdrive.ui.ui_state import ui_state
    services = ui_state.sm.services
    duplicates = sorted({s for s in services if services.count(s) > 1})
    assert duplicates == [], duplicates
    assert "navInstruction" in services
    assert "gpsLocation" in services


class TestProcessConfig(OpenpilotTestCase):
  def test_navigation_processes_follow_toggle(self):
    from openpilot.system.manager.process_config import managed_processes
    params = Params()
    for name in ("navd", "mapsd"):
      process = managed_processes[name]
      params.put_bool("NavigationEnabled", False, block=True)
      assert not process.should_run(True, params, None)
      params.put_bool("NavigationEnabled", True, block=True)
      assert process.should_run(True, params, None)
      assert not process.should_run(False, params, None)

  def test_navigation_disabled_by_default(self):
    assert not Params().get_bool("NavigationEnabled")

  def test_sunnypilot_processes_unchanged(self):
    from openpilot.system.manager.process_config import managed_processes
    for name in ("mapd", "mapd_manager", "modeld", "modeld_tinygrad", "locationd_llk", "manage_sunnylinkd"):
      assert name in managed_processes, name


class TestSunnylinkDestination(OpenpilotTestCase):
  def test_sunnylink_dispatcher_has_nav_methods(self):
    # sunnylinkd serves athenad's dispatcher, so a sunnylink JSON-RPC call can set the destination too
    from openpilot.sunnypilot.sunnylink.athena import sunnylinkd
    from openpilot.system.athena.rpc import dispatcher
    assert sunnylinkd.dispatcher is dispatcher
    assert dispatcher["setNavDestination"](45.4642, 9.19) == {"success": 1}
    assert Params().get("NavDestination") == {"latitude": 45.4642, "longitude": 9.19}
    assert dispatcher["clearNavDestination"]() == {"success": 1}
    assert Params().get("NavDestination") is None


class TestDestinationEntry(OpenpilotTestCase):
  def test_parse(self):
    assert parse_destination("45.4642, 9.1900") == {"latitude": 45.4642, "longitude": 9.19}
    assert parse_destination(" -33.8688  151.2093 ") == {"latitude": -33.8688, "longitude": 151.2093}
    assert parse_destination("45.4642,9.19") == {"latitude": 45.4642, "longitude": 9.19}
    for text in ("", "45.4642", "1, 2, 3", "abc, 9", "91, 0", "0, -181", "nan, 0", "0, inf", "45,46,9"):
      assert parse_destination(text) is None, text

  def test_format(self):
    assert format_destination({"latitude": 45.4642, "longitude": 9.19}) == "45.464200, 9.190000"
    assert format_destination(None) == ""
    assert format_destination({"latitude": "a", "longitude": 1}) == ""
    assert format_destination("45, 9") == ""

  def test_set_and_clear(self):
    params = Params()
    assert set_destination_from_text("45.4642, 9.19", params)
    assert params.get("NavDestination") == {"latitude": 45.4642, "longitude": 9.19}

    # invalid text leaves the destination alone
    assert not set_destination_from_text("not a place", params)
    assert params.get("NavDestination") == {"latitude": 45.4642, "longitude": 9.19}

    assert set_destination_from_text("  ", params)
    assert params.get("NavDestination") is None
