"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.common.constants import CV

# No pyray, ui_state or messaging here: the final limit's square (final_speed_limit.py) draws what this returns, and
# the rules can be tested without a screen.


def distance_text(meters: float, metric: bool) -> str:
  """A distance as the navigation panel on the same screen writes it (navigation.format_distance, copied rather than
  imported because navigation.py pulls in the map stream's VisionIPC and ui_state): m in steps of 10 under 1 km, then
  km with one decimal; ft in steps of 50 under 300 m, then mi with one decimal. Two changes for the 72 px square: km
  starts once the 10 m rounding reaches 1000 ("1.0 km", not "1000 m"), and from 10 km or 10 mi the decimal is dropped
  ("12 km"), so that even "999 km" stays inside the border. Returns "" when the distance rounds to 0, so the square
  shows its source label again."""
  if metric:
    shown = round(meters / 10) * 10
    if shown < 1000:
      return f"{shown} m" if shown > 0 else ""
    return _with_unit(meters / 1000, "km")
  if meters < 300:
    shown = round(meters * CV.METER_TO_FOOT / 50) * 50
    return f"{shown} ft" if shown > 0 else ""
  return _with_unit(meters * CV.METER_TO_MILE, "mi")


def _with_unit(value: float, unit: str) -> str:
  # One decimal below 10 and none from 10 up: "12.5 km" already fills the inside of the square, and "123.5 km"
  # would overflow its border.
  text = f"{value:.1f}"
  return f"{text} {unit}" if float(text) < 10 else f"{value:.0f} {unit}"


class MapAheadDistance:
  """The distance to where the map's next limit starts, as the final limit's square shows it, updated every frame.

  mapd sends it about once a second (liveMapDataSP.speedLimitAheadDistance), so in between it is counted down by the
  metres driven since then, with the speed limit resolver's formula max(0, distance - v_ego * age). mapd measures it in
  a straight line to a fixed point, so once the car has passed that point it grows again until mapd moves on to the
  next limit. That is why, once the distance shown has reached 0, it stays hidden until the limit ahead changes."""
  def __init__(self) -> None:
    self._reached_ahead: int = 0

  def update(self, ahead: int | None, current: int, distance: float, v_ego: float, age: float, metric: bool) -> str:
    """ahead and current: the map's next and current limits in the screen's units, 0 when mapd's message has none.
    ahead is None while the map data cannot be used (mapd silent, or without a GPS fix): nothing is shown and the hold
    is kept, so that a GPS dropout just past the change point does not bring back a distance that grows again.
    distance: mapd's distance to the next limit, sent age seconds ago. Called every frame, also while the text is not
    drawn, so that a new limit ahead always clears the hold."""
    if ahead is None:
      return ""
    if ahead != self._reached_ahead:
      self._reached_ahead = 0
    if ahead <= 0 or ahead == current or ahead == self._reached_ahead:
      return ""
    text = distance_text(max(0., distance - v_ego * age), metric)
    if not text:
      self._reached_ahead = ahead
    return text
