"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl

from openpilot.selfdrive.ui.mici.onroad.hud_renderer import HudRenderer
from openpilot.selfdrive.ui.sunnypilot.onroad.blind_spot_indicators import BlindSpotIndicators
from openpilot.selfdrive.ui.sunnypilot.mici.onroad.auto_speed import AutoSpeed
from openpilot.selfdrive.ui.sunnypilot.mici.onroad.vision_speed_limit import VisionSpeedLimit
from openpilot.selfdrive.ui.ui_state import ui_state


class HudRendererSP(HudRenderer):
  def __init__(self):
    super().__init__()
    self.blind_spot_indicators = BlindSpotIndicators()
    self.vision_speed_limit = VisionSpeedLimit()
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

    # as the planner: only with the big driving model (the external GPU), else the set speed is the maximum again
    sm = ui_state.sm
    self._auto_speed = ui_state.speed_limit_auto_speed and sm.alive['modelV2'] and sm['modelV2'].big

  def _render(self, rect: rl.Rectangle) -> None:
    super()._render(rect)
    # In the bottom left corner, above the steering wheel while it shows; under the blind spot warning.
    self.vision_speed_limit.set_lift(self._wheel_alpha_filter.x / (255 * 0.9))
    self.vision_speed_limit.set_road_lift(self.auto_speed.height_above_final if self._auto_speed else 0)
    self.vision_speed_limit.render(rect)
    if self._auto_speed:
      self.auto_speed.set_above_final(ui_state.speed_limit_vision_display)
      self.auto_speed.render(rect)
    self.blind_spot_indicators.render(rect)

  def _draw_set_speed(self, rect: rl.Rectangle) -> None:
    # Set Speed Only Engages: the set speed is not the speed, the green AUTO square says so instead
    if self._auto_speed:
      return
    super()._draw_set_speed(rect)

  def _has_blind_spot_detected(self) -> bool:

    return self.blind_spot_indicators.detected
