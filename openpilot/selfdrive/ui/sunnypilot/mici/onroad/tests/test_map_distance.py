"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pytest

from openpilot.selfdrive.ui.sunnypilot.mici.onroad.map_distance import MapAheadDistance, distance_text


@pytest.mark.parametrize("meters, metric, expected", [
  (0., True, ""), (4.9, True, ""), (5.1, True, "10 m"), (142., True, "140 m"), (412., True, "410 m"),
  (994., True, "990 m"), (995., True, "1.0 km"), (999., True, "1.0 km"), (1234., True, "1.2 km"),
  (9940., True, "9.9 km"), (9960., True, "10 km"), (12480., True, "12 km"), (123456., True, "123 km"),
  (0., False, ""), (7., False, ""), (8., False, "50 ft"), (142., False, "450 ft"), (250., False, "800 ft"),
  (299., False, "1000 ft"), (300., False, "0.2 mi"), (412., False, "0.3 mi"), (3400., False, "2.1 mi"),
  (15000., False, "9.3 mi"), (100000., False, "62 mi"),
])
def test_distance_text(meters, metric, expected):
  assert distance_text(meters, metric) == expected


def test_counted_down_like_the_resolver():
  # 410 m sent 2 s ago at 20 m/s: 370 m now.
  assert MapAheadDistance().update(50, 80, 410., 20., 2., True) == "370 m"
  # Never below 0, however old the message.
  assert MapAheadDistance().update(50, 80, 410., 20., 60., True) == ""


@pytest.mark.parametrize("ahead, current", [(0, 80), (80, 80), (None, 0)])
def test_hidden_without_a_different_limit_ahead(ahead, current):
  assert MapAheadDistance().update(ahead, current, 410., 0., 0., True) == ""


def test_hidden_once_reached_until_the_limit_ahead_changes():
  distance = MapAheadDistance()
  assert distance.update(50, 80, 30., 0., 0., True) == "30 m"
  assert distance.update(50, 80, 2., 0., 0., True) == ""
  # Past the change point mapd's straight-line distance grows again: still hidden for the same limit ahead.
  assert distance.update(50, 80, 60., 0., 0., True) == ""
  # A new limit ahead shows again, and so does the same one after a message without it.
  assert distance.update(30, 50, 600., 0., 0., True) == "600 m"
  assert distance.update(30, 50, 0., 0., 0., True) == ""
  assert distance.update(0, 50, 0., 0., 0., True) == ""
  assert distance.update(30, 50, 600., 0., 0., True) == "600 m"


def test_hold_kept_while_the_map_data_cannot_be_used():
  distance = MapAheadDistance()
  assert distance.update(50, 80, 2., 0., 0., True) == ""
  # A GPS dropout just past the change point (no usable map data) neither shows the distance nor clears the hold,
  # so the straight-line distance growing again behind the car stays hidden once the fix is back.
  assert distance.update(None, 0, 60., 0., 0., True) == ""
  assert distance.update(50, 80, 60., 0., 0., True) == ""
