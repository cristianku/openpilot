"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl

from openpilot.common.constants import CV
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

SIZE = 72
MARGIN = 12
# Between the squares, and beside the road type: at its biggest the reading square's pulse (sign_reading.PULSE_GROW *
# READING_SIZE / 2, 6.5 px a side, its border 2 px more) only slides under what is drawn after it, the vision square
# over it and the road type beside it.
GAP = 8
# The sign reading's square (sign_reading.py), under the vision square and as big, for its three lines: the road type
# goes left of it, as under it there is no room above the GPU icon.
READING_SIZE = SIZE
ROUNDNESS = 0.3
SEGMENTS = 10
BORDER = 3
EYE_WIDTH = 24
EYE_HEIGHT = 17
EYE_TOP = 9
FONT_SIZE = 34
FONT_SIZE_3_DIGITS = 28
# The small text on top of a square, where the eye would be: the final limit's source (final_speed_limit.py).
SOURCE_FONT_SIZE = 18
SOURCE_TOP = 7
# Left of the reading square, small and fuchsia, what the road-type network sees (a car park, a town...), kept inside
# the screen ("expressway" is wider than the square).
ROAD_FONT_SIZE = 20

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


def reading_box(rect: rl.Rectangle, beside_final: bool = False) -> rl.Rectangle:
  """The sign reading's square, centred under the vision square: with the road type under it, still above the GPU
  icon."""
  box = vision_box(rect, beside_final)
  return rl.Rectangle(box.x + (SIZE - READING_SIZE) / 2, box.y + SIZE + GAP, READING_SIZE, READING_SIZE)


def road_type_place(rect: rl.Rectangle, size: rl.Vector2, beside_final: bool = False) -> rl.Vector2:
  """Where the road type's text of that size goes: left of the reading square and level with its middle, inside the
  screen."""
  reading = reading_box(rect, beside_final)
  return rl.Vector2(max(rect.x + MARGIN, reading.x - GAP - size.x), reading.y + (READING_SIZE - size.y) / 2)


class VisionSpeedLimit(Widget):
  """The speed the speed-limits neural network says to hold (speedVisionSP.speedLimit: the brain's, or the memory's
  without one), drawn on the right of the screen, when Show Vision Speed Limit is on or Speed Limit Source is
  Smartspeed: a fuchsia square with an eye and the speed in white, always visible, over an alert too (the Dashcam Mode
  alert stays for the whole drive), and nothing else: not the next limit, not the distance to a sign, no pulse. The
  drawing does not change the speed. A dark square with a dash means no output: the network is not sure, or
  speedvisiond is not running. Under it the sign reader's square (sign_reading.py), with the distance to the sign and
  the pulse, and left of that the kind of road the network sees. Only the camera's reading is drawn here, as
  speedvisiond works from the camera and the car alone."""
  def __init__(self):
    super().__init__()
    self._font: rl.Font = gui_app.font(FontWeight.BOLD)
    self._eye: rl.Texture = gui_app.texture("icons_mici/onroad/eye_fill.png", EYE_WIDTH, EYE_HEIGHT)
    self._speed_limit: float = 0.
    self._road_type: str = ""

  def _update_state(self) -> None:
    sm = ui_state.sm
    # The speed to hold only, never the limit ahead: the number read shows in the reading square at once
    speed_limit = sm['speedVisionSP'].speedLimit if sm.alive['speedVisionSP'] else 0.
    speed_conv = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
    self._speed_limit = max(0., speed_limit) * speed_conv
    self._road_type = sm['speedVisionSP'].roadType if sm.alive['speedVisionSP'] else ""

  def _render(self, rect: rl.Rectangle) -> None:
    if not ui_state.speed_limit_vision_display or ui_state.sm.recv_frame["carState"] < ui_state.started_frame:
      return

    box = vision_box(rect, ui_state.speed_limit_final_display)
    has_limit = round(self._speed_limit) > 0
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

    if self._road_type:
      size = measure_text_cached(self._font, self._road_type, ROAD_FONT_SIZE)
      rl.draw_text_ex(self._font, self._road_type, road_type_place(rect, size, ui_state.speed_limit_final_display),
                      ROAD_FONT_SIZE, 0, FUCHSIA_LIGHT)
