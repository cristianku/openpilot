"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl

from opendbc.car.structs import car
from openpilot.selfdrive.ui.sunnypilot.mici.onroad.vision_speed_limit import GAP, vision_box
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

FONT_SIZE = 26
TEXT = "SMART"
V_CRUISE_UNSET = 255.
BLINK_TIME = 1.  # secs. The word blinks this long after a + or -, to say they do nothing
BLINK_RATE = 6  # half blinks a second

ButtonType = car.CarState.ButtonEvent.Type
CRUISE_BUTTONS = (ButtonType.accelCruise, ButtonType.decelCruise, ButtonType.resumeCruise, ButtonType.setCruise)

CAP_TOP = 0.2  # of the line height, Inter's: where the capitals start
GREEN = rl.Color(125, 205, 50, 190)  # pea green, see-through
GREY = rl.Color(145, 155, 149, 190)


class AutoSpeed(Widget):
  """Smartspeed (the mode Assist): the word SMART, left of the vision square in the top right corner, level with the
  square's top edge. The car drives to the limit, the set speed only engages, so the set speed is not shown. Always
  drawn, over an alert too: it is a state that holds for the drive. Pea green while Speed Limit Assist drives to a
  limit; grey while no limit is in force (not engaged, or no limit known). A + or - on the car does nothing, so the word blinks for a second, only
  while already engaged: the press that engages does not count."""
  def __init__(self):
    super().__init__()
    self._font: rl.Font = gui_app.font(FontWeight.BOLD)
    self._driving: bool = False
    self._release_toggle: int | None = None
    self._engaged_since: float | None = None
    self._blink_until: float = 0.

  def _update_state(self) -> None:
    sm = ui_state.sm
    assist = sm['longitudinalPlanSP'].speedLimit.assist
    self._driving = (sm.alive['longitudinalPlanSP'] and ui_state.engaged and assist.enabled and
                     assist.vTarget < V_CRUISE_UNSET)
    self._update_blink(sm['selfdriveStateSP'].buttonsReleaseToggle, ui_state.engaged, rl.get_time())

  def _update_blink(self, release_toggle: int, engaged: bool, now: float) -> None:
    released = 0 if self._release_toggle is None else self._release_toggle ^ release_toggle
    self._release_toggle = release_toggle
    if not engaged:
      self._engaged_since = None
    elif self._engaged_since is None:
      self._engaged_since = now
    if self._engaged_since is not None and now - self._engaged_since > BLINK_TIME and \
       any((released >> b) & 1 for b in CRUISE_BUTTONS):
      self._blink_until = now + BLINK_TIME

  def _hidden_by_blink(self, now: float) -> bool:
    return now < self._blink_until and int((self._blink_until - now) * BLINK_RATE) % 2 == 1

  def _render(self, rect: rl.Rectangle) -> None:
    if not ui_state.speed_limit_auto_speed or ui_state.sm.recv_frame["carState"] < ui_state.started_frame:
      return
    if self._hidden_by_blink(rl.get_time()):
      return

    size = measure_text_cached(self._font, TEXT, FONT_SIZE)
    box = vision_box(rect)
    position = rl.Vector2(box.x - GAP - size.x, box.y - size.y * CAP_TOP)
    rl.draw_text_ex(self._font, TEXT, position, FONT_SIZE, 0, GREEN if self._driving else GREY)
