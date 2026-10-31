"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math

import pyray as rl

from openpilot.common.constants import CV
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

SIZE = 72
MARGIN = 12
ROUNDNESS = 0.3
SEGMENTS = 10
BORDER = 3
EYE_WIDTH = 24
EYE_HEIGHT = 17
EYE_TOP = 9
FONT_SIZE = 34
FONT_SIZE_3_DIGITS = 28
# How far up the square moves while the steering wheel shows in the bottom left corner: above it.
LIFT = 62
# A sign read (signRead, the same limit again too) makes the square pulse: it grows by PULSE_GROW of its side and back
# in PULSE_SECONDS, its border white, so the driver sees the network has read it.
PULSE_SECONDS = 0.6
PULSE_GROW = 0.18
# In the bottom right corner, the resolver's limit with its offset (speedLimitFinalLast): the one Speed Limit Assist
# drives to, from the map, the car or the network, the source named on top. Not right of the square: the torque bar
# runs along the bottom from a third of the width, and the left holds the wheel, the lead car and the distance bars.
SOURCE_FONT_SIZE = 18
SOURCE_TOP = 7
SOURCES = {"map": "MAP", "car": "CAR", "vision": "CAM"}
# Above it, small and fuchsia, what the road-type network sees (a car park, a town...).
ROAD_FONT_SIZE = 20
ROAD_GAP = 4

# Fuchsia, not the red and white of a road sign: the limit is read by the network, not a sign from the map or the car.
FUCHSIA = rl.Color(176, 38, 212, 115)  # see-through (45 %): the road shows under it
FUCHSIA_LIGHT = rl.Color(226, 120, 248, 210)
DARK_GREY = rl.Color(40, 40, 40, 115)
GREY = rl.Color(145, 155, 149, 255)
FINAL = rl.Color(0, 0, 0, 140)  # see-through black, the white limit and its border in front


class VisionSpeedLimit(Widget):
  """The speed limit read by the speed-limits neural network (speedVisionSP), drawn in the bottom left corner when
  Show Vision Speed Limit is on or Speed Limit Source is Map Data + Vision: a fuchsia square with an eye and the limit
  in white, always visible (alerts are drawn at the top). The drawing does not change the speed. A dark square with a
  dash means no reading: the network is not sure, or speedvisiond is not running. It pulses each time a sign is read,
  even one that gives the limit already shown. In the bottom right corner the final limit, the network's weighed with
  the map and the car by the resolver, grey while only its last one holds; above it the kind of road the network
  sees."""
  def __init__(self):
    super().__init__()
    self._font: rl.Font = gui_app.font(FontWeight.BOLD)
    self._eye: rl.Texture = gui_app.texture("icons_mici/onroad/eye_fill.png", EYE_WIDTH, EYE_HEIGHT)
    self._speed_limit: float = 0.
    self._lift: float = 0.
    self._pulse_at: float = -math.inf
    self._final: float = 0.
    self._final_valid: bool = False
    self._final_last_valid: bool = False
    self._source: str = ""
    self._road_type: str = ""
    self._road_lift: float = 0.

  def set_road_lift(self, lift: float) -> None:
    """Pixels the road type moves up, over the green AUTO square when it sits above the final limit."""
    self._road_lift = lift

  def set_lift(self, lift: float) -> None:
    """0 in the corner, 1 above the steering wheel (while it shows); in between while it fades."""
    self._lift = min(1., max(0., lift))

  def _update_state(self) -> None:
    sm = ui_state.sm
    speed_limit = sm['speedVisionSP'].speedLimit if sm.alive['speedVisionSP'] else 0.
    if sm.alive['speedVisionSP'] and sm.updated['speedVisionSP'] and sm['speedVisionSP'].signRead:
      self._pulse_at = rl.get_time()
    speed_conv = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
    self._speed_limit = max(0., speed_limit) * speed_conv
    self._road_type = sm['speedVisionSP'].roadType if sm.alive['speedVisionSP'] else ""

    resolver = sm['longitudinalPlanSP'].speedLimit.resolver
    alive = sm.alive['longitudinalPlanSP']
    self._final = max(0., resolver.speedLimitFinalLast) * speed_conv
    self._final_valid = alive and resolver.speedLimitValid
    self._final_last_valid = alive and resolver.speedLimitLastValid
    self._source = SOURCES.get(str(resolver.source), "")

  def _render(self, rect: rl.Rectangle) -> None:
    if not ui_state.speed_limit_vision_display or ui_state.sm.recv_frame["carState"] < ui_state.started_frame:
      return

    box = rl.Rectangle(rect.x + MARGIN, rect.y + rect.height - MARGIN - SIZE - LIFT * self._lift, SIZE, SIZE)
    has_limit = round(self._speed_limit) > 0
    since = rl.get_time() - self._pulse_at
    pulse = math.sin(math.pi * since / PULSE_SECONDS) if 0 <= since < PULSE_SECONDS else 0.
    if pulse > 0:
      grow = SIZE * PULSE_GROW * pulse
      outer = rl.Rectangle(box.x - grow / 2, box.y - grow / 2, SIZE + grow, SIZE + grow)
      rl.draw_rectangle_rounded(outer, ROUNDNESS, SEGMENTS, rl.Color(226, 120, 248, int(90 * pulse)))
      rl.draw_rectangle_rounded_lines_ex(outer, ROUNDNESS, SEGMENTS, BORDER, rl.Color(255, 255, 255, int(255 * pulse)))
    rl.draw_rectangle_rounded(box, ROUNDNESS, SEGMENTS, FUCHSIA if has_limit else DARK_GREY)
    rl.draw_rectangle_rounded_lines_ex(box, ROUNDNESS, SEGMENTS, BORDER, FUCHSIA_LIGHT if has_limit else FUCHSIA)

    eye_x = box.x + (SIZE - self._eye.width) / 2
    rl.draw_texture_v(self._eye, rl.Vector2(eye_x, box.y + EYE_TOP), rl.WHITE if has_limit else GREY)

    text = str(round(self._speed_limit)) if has_limit else "-"
    font_size = FONT_SIZE if len(text) < 3 else FONT_SIZE_3_DIGITS
    size = measure_text_cached(self._font, text, font_size)
    text_top = box.y + EYE_TOP + self._eye.height
    text_y = text_top + (box.y + SIZE - text_top - size.y) / 2
    rl.draw_text_ex(self._font, text, rl.Vector2(box.x + (SIZE - size.x) / 2, text_y), font_size, 0,
                    rl.WHITE if has_limit else GREY)

    final = rl.Rectangle(rect.x + rect.width - MARGIN - SIZE, rect.y + rect.height - MARGIN - SIZE, SIZE, SIZE)
    if self._road_type:
      size = measure_text_cached(self._font, self._road_type, ROAD_FONT_SIZE)
      rl.draw_text_ex(self._font, self._road_type, rl.Vector2(final.x + (SIZE - size.x) / 2, final.y - ROAD_GAP - size.y - self._road_lift),
                      ROAD_FONT_SIZE, 0, FUCHSIA_LIGHT)
    self._render_final(final)

  def _render_final(self, box: rl.Rectangle) -> None:
    has_final = (self._final_valid or self._final_last_valid) and round(self._final) > 0
    color = rl.WHITE if has_final and self._final_valid else GREY
    rl.draw_rectangle_rounded(box, ROUNDNESS, SEGMENTS, FINAL)
    rl.draw_rectangle_rounded_lines_ex(box, ROUNDNESS, SEGMENTS, BORDER, color)

    source = self._source if has_final else ""
    if source:
      size = measure_text_cached(self._font, source, SOURCE_FONT_SIZE)
      rl.draw_text_ex(self._font, source, rl.Vector2(box.x + (SIZE - size.x) / 2, box.y + SOURCE_TOP), SOURCE_FONT_SIZE,
                      0, color)

    text = str(round(self._final)) if has_final else "-"
    font_size = FONT_SIZE if len(text) < 3 else FONT_SIZE_3_DIGITS
    size = measure_text_cached(self._font, text, font_size)
    text_top = box.y + EYE_TOP + self._eye.height
    text_y = text_top + (box.y + SIZE - text_top - size.y) / 2
    rl.draw_text_ex(self._font, text, rl.Vector2(box.x + (SIZE - size.x) / 2, text_y), font_size, 0, color)
