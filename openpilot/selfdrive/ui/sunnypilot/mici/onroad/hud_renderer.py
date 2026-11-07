"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl

from openpilot.selfdrive.ui.mici.onroad.hud_renderer import HudRenderer
from openpilot.selfdrive.ui.sunnypilot.onroad.blind_spot_indicators import BlindSpotIndicators
from openpilot.selfdrive.ui.sunnypilot.mici.onroad.auto_speed import AutoSpeed
from openpilot.selfdrive.ui.sunnypilot.mici.onroad.final_speed_limit import FinalSpeedLimit
from openpilot.selfdrive.ui.sunnypilot.mici.onroad.vision_speed_limit import VisionSpeedLimit
from openpilot.selfdrive.ui.ui_state import ui_state


class HudRendererSP(HudRenderer):
  def __init__(self):
    super().__init__()
    self.blind_spot_indicators = BlindSpotIndicators()
    self.vision_speed_limit = VisionSpeedLimit()
    self.final_speed_limit = FinalSpeedLimit()
    self.auto_speed = AutoSpeed()
    self._sp_engaged: bool = False
    self._auto_speed: bool = False

  def _update_state(self) -> None:
    super()._update_state()
    self.blind_spot_indicators.update()

    sp_engaged = ui_state.engaged
    if sp_engaged != self._sp_engaged:
      self._chestnut_fade_time = rl.get_time() if sp_engaged else 0
    self._sp_engaged = sp_engaged

    self._auto_speed = ui_state.speed_limit_auto_speed

  def _render(self, rect: rl.Rectangle) -> None:
    super()._render(rect)
    # At the top right, clear of the steering wheel and of the GPU icon in the bottom right corner, all under the blind
    # spot warning. Smartspeed: the vision square with the road type under it, SMART left of it, in the corner.
    # Information, Warning and Assist: the final limit in the corner, the vision square left of it when it is on.
    self.vision_speed_limit.render(rect)
    self.final_speed_limit.render(rect)
    if self._auto_speed:
      self.auto_speed.render(rect)
    self.blind_spot_indicators.render(rect)

  def _draw_set_speed(self, rect: rl.Rectangle) -> None:
    # Smartspeed: the set speed is not the speed, the word SMART says so instead. Its fade still
    # runs down to 0: drawing_top_icons() reads it, and a value frozen above 0 would keep DMoji and the long indicator
    # hidden.
    if self._auto_speed:
      self._set_speed_alpha_filter.update(0)
      return
    super()._draw_set_speed(rect)

  def _has_blind_spot_detected(self) -> bool:

    return self.blind_spot_indicators.detected
