import threading
import time
from types import SimpleNamespace

import numpy as np

from openpilot.common.test import OpenpilotTestCase
from openpilot.cereal import log, messaging, custom
from openpilot.cereal.messaging import SubMaster, PubMaster
from openpilot.selfdrive.ui.soundd import SELFDRIVE_STATE_TIMEOUT, SIGN_BEEP_GAP, Soundd, check_selfdrive_timeout_alert

AudibleAlert = log.SelfdriveState.AudibleAlert
AudibleAlertSP = custom.SelfdriveStateSP.AudibleAlert


class VisionSM:
  def __init__(self, sign_read: bool):
    self.updated = {'speedVisionSP': True}
    self._msg = SimpleNamespace(signRead=sign_read)

  def __getitem__(self, name):
    return self._msg


class TestSoundd(OpenpilotTestCase):
  def test_check_selfdrive_timeout_alert(self, mocker):
    sm = SubMaster(['selfdriveState', 'selfdriveStateSP'])
    pm = PubMaster(['selfdriveState', 'selfdriveStateSP'])

    cs = messaging.new_message('selfdriveState')
    cs.selfdriveState.enabled = True
    threading.Timer(0.01, pm.send, args=("selfdriveState", cs)).start()
    sm.update(100)
    assert sm.updated['selfdriveState']

    sm.recv_time['selfdriveState'] = 0
    clock = mocker.patch("openpilot.selfdrive.ui.soundd.time.monotonic", return_value=SELFDRIVE_STATE_TIMEOUT)
    assert not check_selfdrive_timeout_alert(sm)

    clock.return_value = SELFDRIVE_STATE_TIMEOUT + 0.1
    assert check_selfdrive_timeout_alert(sm)

    clock.return_value = SELFDRIVE_STATE_TIMEOUT + 10
    assert not check_selfdrive_timeout_alert(sm)

  def test_check_selfdrive_timeout_alert_mads_lateral_only(self):
    sm = SubMaster(['selfdriveState', 'selfdriveStateSP'])
    pm = PubMaster(['selfdriveState', 'selfdriveStateSP'])

    for _ in range(100):
      cs = messaging.new_message('selfdriveState')
      cs.selfdriveState.enabled = False

      ss_sp = messaging.new_message('selfdriveStateSP')
      ss_sp.selfdriveStateSP.mads.enabled = True

      pm.send("selfdriveState", cs)
      pm.send("selfdriveStateSP", ss_sp)

      time.sleep(0.01)

      sm.update(0)

      assert not check_selfdrive_timeout_alert(sm)

    for _ in range(SELFDRIVE_STATE_TIMEOUT * 110):
      sm.update(0)
      time.sleep(0.01)

    assert check_selfdrive_timeout_alert(sm)

  def test_a_sign_read_beeps_twice_once_per_sign(self, mocker):
    mocker.patch.object(Soundd, "load_sounds")
    s = Soundd()
    s.loaded_sounds = {AudibleAlertSP.promptDoubleHigh: np.zeros(10, np.float32)}
    clock = mocker.patch("openpilot.selfdrive.ui.soundd.time.monotonic", return_value=100.)

    s.sign_beep(VisionSM(False))
    assert s.current_alert == AudibleAlert.none
    s.sign_beep(VisionSM(True))
    assert s.current_alert == AudibleAlertSP.promptDoubleHigh

    # The same sign read again while in view: no new beep, and the gap counts from its last read.
    s.current_alert = AudibleAlert.none
    for t in (100.2, 100.4, 101.):
      clock.return_value = t
      s.sign_beep(VisionSM(True))
      assert s.current_alert == AudibleAlert.none
    clock.return_value = 101. + SIGN_BEEP_GAP + 0.1
    s.sign_beep(VisionSM(True))
    assert s.current_alert == AudibleAlertSP.promptDoubleHigh

  def test_a_sign_read_never_cuts_an_alert(self, mocker):
    mocker.patch.object(Soundd, "load_sounds")
    s = Soundd()
    s.loaded_sounds = {AudibleAlert.warningSoft: np.zeros(10, np.float32)}
    mocker.patch("openpilot.selfdrive.ui.soundd.time.monotonic", return_value=100.)
    s.update_alert(AudibleAlert.warningSoft)
    s.sign_beep(VisionSM(True))
    assert s.current_alert == AudibleAlert.warningSoft

  # TODO: add test with micd for checking that soundd actually outputs sounds
