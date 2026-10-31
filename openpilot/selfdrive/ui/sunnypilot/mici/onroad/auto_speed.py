"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl

from openpilot.selfdrive.ui.sunnypilot.mici.onroad.vision_speed_limit import SIZE, MARGIN, ROUNDNESS, SEGMENTS, BORDER, \
  EYE_WIDTH, EYE_HEIGHT, EYE_TOP
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

GAP = 8
FONT_SIZE = 22
V_CRUISE_UNSET = 255.

GREEN = rl.Color(23, 160, 80, 115)  # see-through (45 %), like the vision square
GREEN_LIGHT = rl.Color(120, 230, 160, 230)
DARK_GREY = rl.Color(40, 40, 40, 115)
GREY = rl.Color(145, 155, 149, 255)


class AutoSpeed(Widget):
  """Set Speed Only Engages (Alpha): a green square with an eye and AUTO, in the bottom right corner, above the final
  limit when that shows, only with the big driving model (the external GPU). The car drives to the limit, the set
  speed only engages, so the set speed is not shown.
  Green while Speed Limit Assist drives to a limit; dark with a green border while the set speed holds (not engaged,
  or no limit known yet)."""
  def __init__(self):
    super().__init__()
    self._font: rl.Font = gui_app.font(FontWeight.BOLD)
    self._eye: rl.Texture = gui_app.texture("icons_mici/onroad/eye_fill.png", EYE_WIDTH, EYE_HEIGHT)
    self._driving: bool = False
    self._above_final: bool = False

  @property
  def height_above_final(self) -> float:
    return SIZE + GAP

  def set_above_final(self, above: bool) -> None:
    self._above_final = above

  def _update_state(self) -> None:
    sm = ui_state.sm
    assist = sm['longitudinalPlanSP'].speedLimit.assist
    self._driving = (sm.alive['longitudinalPlanSP'] and ui_state.engaged and assist.enabled and
                     assist.vTarget < V_CRUISE_UNSET)

  def _render(self, rect: rl.Rectangle) -> None:
    if not ui_state.speed_limit_auto_speed or ui_state.sm.recv_frame["carState"] < ui_state.started_frame:
      return

    lift = self.height_above_final if self._above_final else 0
    box = rl.Rectangle(rect.x + rect.width - MARGIN - SIZE, rect.y + rect.height - MARGIN - SIZE - lift, SIZE, SIZE)
    rl.draw_rectangle_rounded(box, ROUNDNESS, SEGMENTS, GREEN if self._driving else DARK_GREY)
    rl.draw_rectangle_rounded_lines_ex(box, ROUNDNESS, SEGMENTS, BORDER, GREEN_LIGHT if self._driving else GREEN)

    color = rl.WHITE if self._driving else GREY
    rl.draw_texture_v(self._eye, rl.Vector2(box.x + (SIZE - self._eye.width) / 2, box.y + EYE_TOP), color)

    text = "AUTO"
    size = measure_text_cached(self._font, text, FONT_SIZE)
    text_top = box.y + EYE_TOP + self._eye.height
    text_y = text_top + (box.y + SIZE - text_top - size.y) / 2
    rl.draw_text_ex(self._font, text, rl.Vector2(box.x + (SIZE - size.x) / 2, text_y), FONT_SIZE, 0, color)
