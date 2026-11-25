import pytest

from openpilot.selfdrive.ui.sunnypilot.mici.onroad.auto_speed import AutoSpeed, BLINK_TIME, ButtonType


def _widget() -> AutoSpeed:
  widget = AutoSpeed.__new__(AutoSpeed)  # the blink needs no font or window
  widget._release_toggle = None
  widget._engaged_since = None
  widget._blink_until = 0.
  return widget


def _blinks(widget: AutoSpeed, now: float) -> bool:
  return any(widget._hidden_by_blink(now + i * .01) for i in range(int(BLINK_TIME * 100)))


@pytest.mark.parametrize('button', [ButtonType.accelCruise, ButtonType.decelCruise])
def test_plus_and_minus_blink_for_a_second_while_engaged(button):
  widget = _widget()
  widget._update_blink(0, True, 10.)
  widget._update_blink(0, True, 20.)
  widget._update_blink(1 << button, True, 20.)
  assert _blinks(widget, 20.)
  assert not widget._hidden_by_blink(20. + BLINK_TIME)


def test_the_press_that_engages_does_not_blink():
  widget = _widget()
  widget._update_blink(0, False, 10.)
  widget._update_blink(1 << ButtonType.setCruise, True, 10.1)
  assert not _blinks(widget, 10.1)


def test_a_button_press_seen_at_start_does_not_blink():
  widget = _widget()
  widget._update_blink(1 << ButtonType.accelCruise, True, 10.)
  widget._update_blink(1 << ButtonType.accelCruise, True, 20.)
  assert not _blinks(widget, 20.)


def test_another_button_does_not_blink():
  widget = _widget()
  widget._update_blink(0, True, 10.)
  widget._update_blink(0, True, 20.)
  widget._update_blink(1 << ButtonType.altButton2, True, 20.)
  assert not _blinks(widget, 20.)
