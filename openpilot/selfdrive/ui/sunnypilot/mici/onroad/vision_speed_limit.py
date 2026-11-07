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
# Between the squares, and over the road type: at its biggest the vision square's pulse (PULSE_GROW * SIZE / 2, 6.5 px
# a side, its border 2 px more) only slides under what is drawn after it, the road type.
GAP = 8
ROUNDNESS = 0.3
SEGMENTS = 10
BORDER = 3
EYE_WIDTH = 24
EYE_HEIGHT = 17
EYE_TOP = 9
FONT_SIZE = 34
FONT_SIZE_3_DIGITS = 28
# A sign read (signRead, the same limit again too) makes the square pulse: it grows by PULSE_GROW of its side and back
# in PULSE_SECONDS, its border white, so the driver sees the network has read it.
PULSE_SECONDS = 0.6
PULSE_GROW = 0.18
# The small text on top of a square, where the eye would be: the sign's distance here, the final limit's source.
SOURCE_FONT_SIZE = 18
SOURCE_TOP = 7
# Under the vision square, small and fuchsia, what the road-type network sees (a car park, a town...), kept inside the
# screen ("expressway" is wider than the square), clear of the square's pulse.
ROAD_FONT_SIZE = 20
ROAD_GAP = GAP

# Fuchsia, not the red and white of a road sign: the limit is read by the network, not a sign from the map or the car.
FUCHSIA = rl.Color(176, 38, 212, 115)  # see-through (45 %): the road shows under it
FUCHSIA_LIGHT = rl.Color(226, 120, 248, 210)
DARK_GREY = rl.Color(40, 40, 40, 115)
GREY = rl.Color(145, 155, 149, 255)


def vision_box(rect: rl.Rectangle, beside_final: bool = False) -> rl.Rectangle:
  """On the right, out of the steering wheel's way (the left holds the wheel, the lead car and the distance bars), and
  at the top, as the bottom right corner holds the GPU icon of the big model: in the top right corner, or right beside
  the final limit when that takes the corner (final_speed_limit.final_box). The word SMART goes left of this square."""
  beside = SIZE + GAP if beside_final else 0
  return rl.Rectangle(rect.x + rect.width - MARGIN - SIZE - beside, rect.y + MARGIN, SIZE, SIZE)


class VisionSpeedLimit(Widget):
  """The speed limit read by the speed-limits neural network (speedVisionSP), drawn on the right of the screen, when Show Vision Speed Limit is on or Speed Limit Source is Map Data +
  Vision or Vision Only: a fuchsia square with an eye and the limit in white, always visible, over an alert too (the
  Dashcam Mode alert stays for the whole drive). The drawing does not change the speed. A dark square with a dash means
  no reading: the network is not sure, or speedvisiond is not running. It pulses each time a sign is read, even one
  that gives the limit already shown. Under it the kind of road the network sees. While the network sees a round sign
  or a motorway sign or its end ahead, the distance to it replaces the eye. Only the camera's reading is drawn here,
  as speedvisiond works from the camera and the car alone."""
  def __init__(self):
    super().__init__()
    self._font: rl.Font = gui_app.font(FontWeight.BOLD)
    self._eye: rl.Texture = gui_app.texture("icons_mici/onroad/eye_fill.png", EYE_WIDTH, EYE_HEIGHT)
    self._speed_limit: float = 0.
    self._distance: str = ""
    self._pulse_at: float = -math.inf
    self._road_type: str = ""

  def _update_state(self) -> None:
    sm = ui_state.sm
    # The number just read at once, also while it is the limit ahead: this square shows what the camera reads
    speed_limit = (sm['speedVisionSP'].speedLimitAhead or sm['speedVisionSP'].speedLimit) if sm.alive['speedVisionSP'] else 0.
    if sm.alive['speedVisionSP'] and sm.updated['speedVisionSP'] and sm['speedVisionSP'].signRead:
      self._pulse_at = rl.get_time()
    speed_conv = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
    self._speed_limit = max(0., speed_limit) * speed_conv
    self._road_type = sm['speedVisionSP'].roadType if sm.alive['speedVisionSP'] else ""
    distance = sm['speedVisionSP'].signDistance if sm.alive['speedVisionSP'] else 0.
    shown, unit = (round(distance), "m") if ui_state.is_metric else (round(distance * CV.METER_TO_FOOT), "ft")
    self._distance = f"{shown} {unit}" if shown > 0 else ""

  def _render(self, rect: rl.Rectangle) -> None:
    if not ui_state.speed_limit_vision_display or ui_state.sm.recv_frame["carState"] < ui_state.started_frame:
      return

    box = vision_box(rect, ui_state.speed_limit_final_display)
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

    # While a sign is ahead, the distance to it (speedVisionSP.signDistance, from its size on the camera) takes
    # the eye's place, as small as the source, for the driver to check: m, ft without metric units.
    if self._distance:
      size = measure_text_cached(self._font, self._distance, SOURCE_FONT_SIZE)
      rl.draw_text_ex(self._font, self._distance, rl.Vector2(box.x + (SIZE - size.x) / 2, box.y + SOURCE_TOP),
                      SOURCE_FONT_SIZE, 0, rl.WHITE)
    else:
      eye_x = box.x + (SIZE - self._eye.width) / 2
      rl.draw_texture_v(self._eye, rl.Vector2(eye_x, box.y + EYE_TOP), rl.WHITE if has_limit else GREY)

    text = str(round(self._speed_limit)) if has_limit else "-"
    font_size = FONT_SIZE if len(text) < 3 else FONT_SIZE_3_DIGITS
    size = measure_text_cached(self._font, text, font_size)
    text_top = box.y + EYE_TOP + self._eye.height
    text_y = text_top + (box.y + SIZE - text_top - size.y) / 2
    rl.draw_text_ex(self._font, text, rl.Vector2(box.x + (SIZE - size.x) / 2, text_y), font_size, 0,
                    rl.WHITE if has_limit else GREY)

    if self._road_type:
      size = measure_text_cached(self._font, self._road_type, ROAD_FONT_SIZE)
      road_x = min(box.x + (SIZE - size.x) / 2, rect.x + rect.width - MARGIN - size.x)
      rl.draw_text_ex(self._font, self._road_type, rl.Vector2(road_x, box.y + SIZE + ROAD_GAP), ROAD_FONT_SIZE, 0,
                      FUCHSIA_LIGHT)
