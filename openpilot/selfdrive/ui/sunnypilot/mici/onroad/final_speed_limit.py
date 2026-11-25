"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import time

import pyray as rl

from openpilot.common.constants import CV
from openpilot.selfdrive.ui.sunnypilot.mici.onroad.map_distance import MapAheadDistance
from openpilot.selfdrive.ui.sunnypilot.mici.onroad.vision_speed_limit import SIZE, MARGIN, ROUNDNESS, SEGMENTS, \
  BORDER, EYE_HEIGHT, EYE_TOP, FONT_SIZE, FONT_SIZE_3_DIGITS, SOURCE_FONT_SIZE, SOURCE_TOP, GREY
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

SOURCES = {"map": "MAP", "car": "CAR", "vision": "CAM"}
FINAL = rl.Color(0, 0, 0, 140)  # see-through black, the white limit and its border in front


def final_box(rect: rl.Rectangle) -> rl.Rectangle:
  """The top right corner: the final limit, or the vision square without it."""
  return rl.Rectangle(rect.x + rect.width - MARGIN - SIZE, rect.y + MARGIN, SIZE, SIZE)


class FinalSpeedLimit(Widget):
  """The resolver's limit with its offset (speedLimitFinalLast), in the top right corner, with Information, Warning
  and Assist: with Smartspeed only the vision square shows. The
  resolver weighs the map, the car and the network as the Speed Limit Source says; the source it took is named on top
  (MAP, CAR or CAM), and with Speed Limit Assist the car drives to this limit. Grey while only the last limit holds.
  While it is the map's and the map has a different limit ahead (liveMapDataSP.speedLimitAhead), the distance to where
  that one starts replaces MAP, as the sign's distance replaces the eye in the vision square. This square is the only
  one that reads the map: the vision square and speedvisiond never do."""
  def __init__(self):
    super().__init__()
    self._font: rl.Font = gui_app.font(FontWeight.BOLD)
    self._final: float = 0.
    self._final_valid: bool = False
    self._final_last_valid: bool = False
    self._source: str = ""
    self._ahead_distance = MapAheadDistance()
    self._map_distance: str = ""

  def _update_state(self) -> None:
    sm = ui_state.sm
    speed_conv = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
    resolver = sm['longitudinalPlanSP'].speedLimit.resolver
    alive = sm.alive['longitudinalPlanSP']
    self._final = max(0., resolver.speedLimitFinalLast) * speed_conv
    self._final_valid = alive and resolver.speedLimitValid
    self._final_last_valid = alive and resolver.speedLimitLastValid
    source = str(resolver.source)
    self._source = SOURCES.get(source, "")

    # The map's next limit and its distance, counted down every frame with the resolver's formula,
    # max(0, distance - v_ego * age), on the time.monotonic() clock of SubMaster.recv_time, but with carState.vEgo
    # where the resolver takes the planner's v_ego. Updated also while not drawn, so that a new limit ahead clears the
    # hold in MapAheadDistance. The map data is usable only while mapd is alive and valid (mapd sets valid from
    # gpsOK): without a GPS fix its position, and so the distance, is stale, and the hold is kept as it is.
    map_data = sm['liveMapDataSP']
    ahead: int | None = None
    current = 0
    if sm.alive['liveMapDataSP'] and sm.valid['liveMapDataSP']:
      ahead = round(map_data.speedLimitAhead * speed_conv) if map_data.speedLimitAheadValid else 0
      current = round(map_data.speedLimit * speed_conv) if map_data.speedLimitValid else 0
    distance = self._ahead_distance.update(ahead, current, map_data.speedLimitAheadDistance, sm['carState'].vEgo,
                                           time.monotonic() - sm.recv_time['liveMapDataSP'], ui_state.is_metric)
    self._map_distance = distance if alive and source == "map" else ""

  def _render(self, rect: rl.Rectangle) -> None:
    if not ui_state.speed_limit_final_display or ui_state.sm.recv_frame["carState"] < ui_state.started_frame:
      return

    box = final_box(rect)
    has_final = (self._final_valid or self._final_last_valid) and round(self._final) > 0
    color = rl.WHITE if has_final and self._final_valid else GREY
    rl.draw_rectangle_rounded(box, ROUNDNESS, SEGMENTS, FINAL)
    rl.draw_rectangle_rounded_lines_ex(box, ROUNDNESS, SEGMENTS, BORDER, color)

    # The map's next limit's distance in place of MAP, as small as the source.
    label = (self._map_distance or self._source) if has_final else ""
    if label:
      size = measure_text_cached(self._font, label, SOURCE_FONT_SIZE)
      rl.draw_text_ex(self._font, label, rl.Vector2(box.x + (SIZE - size.x) / 2, box.y + SOURCE_TOP), SOURCE_FONT_SIZE,
                      0, color)

    # The number sits where it does in the vision square, under the eye's place.
    text = str(round(self._final)) if has_final else "-"
    font_size = FONT_SIZE if len(text) < 3 else FONT_SIZE_3_DIGITS
    size = measure_text_cached(self._font, text, font_size)
    text_top = box.y + EYE_TOP + EYE_HEIGHT
    text_y = text_top + (box.y + SIZE - text_top - size.y) / 2
    rl.draw_text_ex(self._font, text, rl.Vector2(box.x + (SIZE - size.x) / 2, text_y), font_size, 0, color)
