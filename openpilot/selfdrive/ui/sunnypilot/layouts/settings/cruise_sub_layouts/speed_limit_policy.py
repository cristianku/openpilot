"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from collections.abc import Callable

import pyray as rl
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.sunnypilot.widgets.list_view import multiple_button_item_sp
from openpilot.system.ui.widgets.network import NavButton
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.system.ui.widgets import Widget

# Map Data + Vision (5) is no longer offered: the buttons give these values, and a stored 5 shows as Smartspeed (6)
SPEED_LIMIT_POLICY_VALUES = [0, 1, 2, 3, 4, 6]

SPEED_LIMIT_POLICY_BUTTONS = [tr("Car Only"), tr("Map Only"), tr("Car First"), tr("Map First"), tr("Combined"),
                              tr("Smartspeed")]

SPEED_LIMIT_POLICY_DESCRIPTIONS = [
  tr("Car Only: Use Speed Limit data only from Car"),
  tr("Map Only: Use Speed Limit data only from OpenStreetMaps"),
  tr("Car First: Use Speed Limit data from Car if available, else use from OpenStreetMaps"),
  tr("Map First: Use Speed Limit data from OpenStreetMaps if available, else use from Car"),
  tr("Combined: Use combined Speed Limit data from Car & OpenStreetMaps"),
  tr("Smartspeed: Use only the speed limit of the speed-limits neural network from the road camera, no map"),
]


def policy_button(value: int) -> int:
  value = 6 if value == 5 else value
  return SPEED_LIMIT_POLICY_VALUES.index(value) if value in SPEED_LIMIT_POLICY_VALUES else 0


class SpeedLimitPolicyLayout(Widget):
  def __init__(self, back_btn_callback: Callable):
    super().__init__()
    self._back_button = NavButton(tr("Back"))
    self._back_button.set_click_callback(back_btn_callback)

    items = self._initialize_items()
    self._scroller = Scroller(items, line_separator=False, spacing=0)

  def _initialize_items(self):
    self._speed_limit_policy = multiple_button_item_sp(
      title=lambda: tr("Speed Limit Source"),
      description=self._get_policy_description,
      buttons=SPEED_LIMIT_POLICY_BUTTONS,
      callback=self._set_policy,
      button_width=220,
    )

    items = [
      self._speed_limit_policy
    ]
    return items

  @staticmethod
  def _set_policy(button: int):
    ui_state.params.put("SpeedLimitPolicy", SPEED_LIMIT_POLICY_VALUES[button])

  @staticmethod
  def _get_policy_description():
    selected = policy_button(int(ui_state.params.get("SpeedLimitPolicy", return_default=True)))
    return "<br>".join(f"<b>{desc}</b>" if i == selected else desc for i, desc in enumerate(SPEED_LIMIT_POLICY_DESCRIPTIONS))

  def _render(self, rect):
    self._back_button.set_position(self._rect.x, self._rect.y + 20)
    self._back_button.render()

    content_rect = rl.Rectangle(rect.x, rect.y + self._back_button.rect.height + 40, rect.width, rect.height - self._back_button.rect.height - 40)
    self._scroller.render(content_rect)

  def show_event(self):
    self._speed_limit_policy.action_item.selected_button = policy_button(int(ui_state.params.get("SpeedLimitPolicy", return_default=True)))
    self._scroller.show_event()
    self._speed_limit_policy.show_description(True)
