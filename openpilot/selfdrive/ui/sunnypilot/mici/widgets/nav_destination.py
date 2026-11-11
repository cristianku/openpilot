"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl

from openpilot.selfdrive.ui.mici.widgets.button import BigButton
from openpilot.selfdrive.ui.mici.widgets.dialog import BigInputDialog
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.sunnypilot.navd.destination import DESTINATION_PARAM, format_destination, parse_destination, set_destination_from_text
from openpilot.system.ui.lib.application import gui_app

REFRESH_SECONDS = 1.0  # athena, sunnylink and navd (on arrival) change the destination too


def destination_text_valid(text: str) -> bool:
  if not text.strip():
    return True  # empty clears the destination
  return parse_destination(text) is not None


class NavDestinationButton(BigButton):
  """On-device Nav Alpha destination entry, as latitude, longitude. Shown while navigation is enabled."""

  def __init__(self):
    super().__init__("nav destination", "Not set", icon=gui_app.texture("../../sunnypilot/selfdrive/assets/offroad/icon_map.png", 64, 64),
                     description="Set the navigation destination on the device as latitude, longitude. Leave empty to clear it.")
    self._last_refresh = 0.0
    self.set_click_callback(self._open_dialog)
    self.set_visible(lambda: not ui_state.is_release and ui_state.navigation_enabled)

  def _update_state(self):
    super()._update_state()
    if rl.get_time() - self._last_refresh >= REFRESH_SECONDS:
      self.refresh()

  def refresh(self):
    self._last_refresh = rl.get_time()
    destination = format_destination(ui_state.params.get(DESTINATION_PARAM))
    if destination:
      self.set_value(destination)
    else:
      self.set_value("Not set")

  def _open_dialog(self):
    current = format_destination(ui_state.params.get(DESTINATION_PARAM))
    gui_app.push_widget(BigInputDialog("latitude, longitude...", current, minimum_length=0, confirm_callback=self._on_confirm,
                                       text_validator=destination_text_valid))

  def _on_confirm(self, text: str):
    set_destination_from_text(text, ui_state.params)
    self.refresh()
