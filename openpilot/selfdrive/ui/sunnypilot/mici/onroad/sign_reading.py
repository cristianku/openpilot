"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math

import pyray as rl

from openpilot.selfdrive.ui.sunnypilot.mici.onroad.vision_speed_limit import READING_SIZE, ROUNDNESS, SEGMENTS, BORDER, \
  DARK_GREY, GREY, reading_box
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

CANCELLATION = "cancellation"
# The signs of the kind of road a reading can be (speed_limits/vision.py: ROAD_SIGNS), by their short name, crossed out
# at their end as on the sign.
ROAD_SIGNS = {
  "other_sign:motorway_start": ("MWY", False),
  "other_sign:motorway_end": ("MWY", True),
  "other_sign:expressway_start": ("EXP", False),
  "other_sign:expressway_end": ("EXP", True),
  "other_sign:town_entry": ("TOWN", False),
  "other_sign:town_exit": ("TOWN", True),
}
# The text's size by its length: a number of two digits, of three, a word; smaller still where it is wider than
# TEXT_WIDTH (MWY, TOWN).
FONT_SIZES = {1: 28, 2: 28, 3: 26, 4: 22}
TEXT_WIDTH = READING_SIZE - 2 * (BORDER + 4)
# Two lines, by where the middle of their capitals and digits goes from the top of the square: the confidence and the
# sign read. No metres: the distance to a sign is not shown (12 Nov 2026).
SMALL_FONT_SIZE = 16
CONFIDENCE_MIDDLE = 17
TEXT_MIDDLE = 44
CAP_MIDDLE = 0.63  # of the line height, Inter's: the middle of its capitals and digits
CROSS = 3  # the line through a road sign's end
# While a sign is ahead the square blinks: it grows by PULSE_GROW of its side and back every PULSE_SECONDS, its border
# white, from the moment the network sees the sign to the moment the car passes it (the zero), for debugging: the driver
# sees when the network picked the sign up and when it let it go. soundd still beeps when a sign is read (signRead).
PULSE_SECONDS = 0.6
PULSE_GROW = 0.18
# Red, the ring of a round sign, while the sign is in view.
RED = rl.Color(235, 60, 60, 235)
# A road sign read (MWY, EXP, TOWN) never takes the square's place: it goes in a small box under it, as wide, until
# the next sign is read, while the square keeps the last limit read.
ROAD_GAP = 4
ROAD_HEIGHT = 24
ROAD_FONT_SIZE = 18


def reading_text(sign_class: str, value: float) -> tuple[str, bool]:
  """What the square says of a raw reading (speedVisionSP.rawSignClass and rawSignValue): its text and whether it is
  crossed out. The number as written on the sign, in its own unit; END for the end of a limit; a road sign's short
  name; "" before the first sign."""
  if not sign_class:
    return "", False
  if sign_class == CANCELLATION:
    return "END", False
  if sign_class in ROAD_SIGNS:
    return ROAD_SIGNS[sign_class]
  return (str(round(value)), False) if value > 0 else ("?", False)


def square_and_road(sign_class: str, value: float, confidence: float,
                    square: tuple[str, bool, str]) -> tuple[tuple[str, bool, str], tuple[str, bool]]:
  """What the square and the box under it show after a raw reading: the square (its text, crossed out, the confidence)
  the last limit read, END or a number, kept as it was (square) when the reading is a road sign, which goes under it
  instead; ("", False) under it after any other reading."""
  text, crossed = reading_text(sign_class, value)
  if sign_class in ROAD_SIGNS:
    return square, (text, crossed)
  return (text, crossed, f"{round(confidence * 100)}%" if text else ""), ("", False)


def sign_ahead(seen: bool, metres: float) -> bool:
  """Whether a sign is ahead: in view now (speedVisionSP.rawSignSeen), or seen and not passed yet (signDistance over 0:
  speedvisiond counts it down to the sign with the metres driven, and it is 0 once the car is at the sign). The metres
  only end the blinking; they are not shown."""
  return bool(seen) or metres > 0


def blink(since: float | None, now: float) -> float:
  """How far the square is into a pulse, 0 to 1: 0 without a sign ahead (since None), else a pulse every PULSE_SECONDS
  from the moment the sign was first ahead."""
  if since is None or now < since:
    return 0.
  return math.sin(math.pi * ((now - since) % PULSE_SECONDS) / PULSE_SECONDS)


class SignReading(Widget):
  """The sign reader's square, under the vision square and as big, for the driver to see the sign reader work apart
  from the speed the vision square shows: the last limit the classifier read (speedVisionSP.rawSign*) as it was read
  (its number as written, END), with the classifier's confidence on top,
  sure enough or not, whatever the memory or the brain made of it. Its border is red while that sign is in view
  (rawSignSeen) and grey once it is passed, as the reading stays until another sign is read. A road sign read (its short
  name, crossed out at its end) goes in a small box under the square, red too while in view, until the next sign is
  read, and leaves the limit in the square (square_and_road). It blinks from the moment the network sees a sign until
  the car passes it (sign_ahead, blink), then stands still; soundd beeps on every sign read. No metres are shown. A
  dash before the first sign or without speedvisiond. All of it is for debugging and is to go. Drawn under the vision square, so the pulse slides under it; the drawing changes nothing."""
  def __init__(self):
    super().__init__()
    self._font: rl.Font = gui_app.font(FontWeight.BOLD)
    self._text: str = ""
    self._crossed: bool = False
    self._confidence: str = ""
    self._road: str = ""
    self._road_crossed: bool = False
    self._seen: bool = False
    self._ahead_since: float | None = None

  def _update_state(self) -> None:
    sm = ui_state.sm
    alive = sm.alive['speedVisionSP']
    vision = sm['speedVisionSP']
    if alive:
      (self._text, self._crossed, self._confidence), (self._road, self._road_crossed) = square_and_road(
        vision.rawSignClass, vision.rawSignValue, vision.rawSignConfidence, (self._text, self._crossed, self._confidence))
    else:
      self._text, self._crossed, self._confidence, self._road, self._road_crossed = "", False, "", "", False
    self._seen = alive and vision.rawSignSeen
    if not (alive and sign_ahead(vision.rawSignSeen, vision.signDistance)):
      self._ahead_since = None
    elif self._ahead_since is None:
      self._ahead_since = rl.get_time()

  def _draw_line(self, box: rl.Rectangle, text: str, font_size: float, middle: float, color: rl.Color,
                 crossed: bool = False) -> None:
    """A line, centred, its capitals' middle at middle, shrunk to TEXT_WIDTH, crossed out when asked."""
    width = measure_text_cached(self._font, text, font_size).x
    font_size *= min(1., TEXT_WIDTH / width) if width > 0 else 1.
    size = measure_text_cached(self._font, text, font_size)
    position = rl.Vector2(box.x + (READING_SIZE - size.x) / 2, box.y + middle - CAP_MIDDLE * size.y)
    rl.draw_text_ex(self._font, text, position, font_size, 0, color)
    if crossed:
      centre = position.y + CAP_MIDDLE * size.y
      rl.draw_line_ex(rl.Vector2(position.x - 2, centre + size.y / 3), rl.Vector2(position.x + size.x + 2, centre - size.y / 3),
                      CROSS, color)

  def _render(self, rect: rl.Rectangle) -> None:
    if not ui_state.speed_limit_vision_display or ui_state.sm.recv_frame["carState"] < ui_state.started_frame:
      return

    box = reading_box(rect, ui_state.speed_limit_final_display)
    pulse = blink(self._ahead_since, rl.get_time())
    if pulse > 0:
      grow = READING_SIZE * PULSE_GROW * pulse
      outer = rl.Rectangle(box.x - grow / 2, box.y - grow / 2, READING_SIZE + grow, READING_SIZE + grow)
      rl.draw_rectangle_rounded(outer, ROUNDNESS, SEGMENTS, rl.Color(255, 255, 255, int(70 * pulse)))
      rl.draw_rectangle_rounded_lines_ex(outer, ROUNDNESS, SEGMENTS, BORDER, rl.Color(255, 255, 255, int(255 * pulse)))
    rl.draw_rectangle_rounded(box, ROUNDNESS, SEGMENTS, DARK_GREY)
    rl.draw_rectangle_rounded_lines_ex(box, ROUNDNESS, SEGMENTS, BORDER, RED if self._seen and not self._road else GREY)
    if self._road:
      under = rl.Rectangle(box.x, box.y + READING_SIZE + ROAD_GAP, READING_SIZE, ROAD_HEIGHT)
      rl.draw_rectangle_rounded(under, ROUNDNESS, SEGMENTS, DARK_GREY)
      rl.draw_rectangle_rounded_lines_ex(under, ROUNDNESS, SEGMENTS, BORDER, RED if self._seen else GREY)
      self._draw_line(under, self._road, ROAD_FONT_SIZE, ROAD_HEIGHT / 2, rl.WHITE, self._road_crossed)

    color = rl.WHITE if self._text else GREY
    if self._confidence:
      self._draw_line(box, self._confidence, SMALL_FONT_SIZE, CONFIDENCE_MIDDLE, color)

    text = self._text or "-"
    self._draw_line(box, text, FONT_SIZES.get(len(text), FONT_SIZES[4]), TEXT_MIDDLE, color, self._crossed)
