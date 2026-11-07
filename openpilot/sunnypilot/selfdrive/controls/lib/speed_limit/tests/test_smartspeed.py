"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from openpilot.common.parameterized import parameterized

from openpilot.cereal import custom
from opendbc.car.car_helpers import interfaces
from opendbc.car.rivian.values import CAR as RIVIAN
from opendbc.car.tesla.values import CAR as TESLA
from opendbc.car.toyota.values import CAR as TOYOTA
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.car import interfaces as sunnypilot_interfaces
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit import PCM_LONG_REQUIRED_MAX_SET_SPEED
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Mode, Policy
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_assist import SpeedLimitAssist, ACTIVE_STATES
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.common.test import OpenpilotTestCase

SpeedLimitAssistState = custom.LongitudinalPlanSP.SpeedLimit.AssistState

ALL_STATES = tuple(SpeedLimitAssistState.schema.enumerants.values())

SPEED_LIMITS = {
  'residential': 25 * CV.MPH_TO_MS,  # 25 mph
  'city': 35 * CV.MPH_TO_MS,         # 35 mph
  'highway': 65 * CV.MPH_TO_MS,      # 65 mph
  'freeway': 80 * CV.MPH_TO_MS,      # 80 mph
}

DEFAULT_CAR = TOYOTA.TOYOTA_RAV4_TSS2


class TestSmartspeed(OpenpilotTestCase):
  car_name = DEFAULT_CAR

  def setup_method(self):
    self.params = Params()
    self.reset_custom_params()
    self.events_sp = EventsSP()
    CI = self._setup_platform(self.car_name)
    self.sla = SpeedLimitAssist(CI.CP, CI.CP_SP)
    self.pcm_long_max_set_speed = PCM_LONG_REQUIRED_MAX_SET_SPEED[self.sla.is_metric][1]  # use 80 MPH for now
    self.speed_conv = CV.MS_TO_KPH if self.sla.is_metric else CV.MS_TO_MPH

  def teardown_method(self):
    self.reset_state()

  def _setup_platform(self, car_name):
    CarInterface = interfaces[car_name]
    CP = CarInterface.get_non_essential_params(car_name)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, car_name)
    CI = CarInterface(CP, CP_SP)
    CI.CP.openpilotLongitudinalControl = True  # always assume it's openpilot longitudinal
    sunnypilot_interfaces.setup_interfaces(CI, self.params)
    return CI

  def reset_custom_params(self):
    self.params.put("IsReleaseSpBranch", True, block=True)
    self.params.put("SpeedLimitMode", int(Mode.smartspeed), block=True)
    self.params.put_bool("IsMetric", False, block=True)
    self.params.put("SpeedLimitOffsetType", 0, block=True)
    self.params.put("SpeedLimitValueOffset", 0, block=True)

  def reset_state(self):
    self.sla.state = SpeedLimitAssistState.disabled
    self.sla.frame = -1
    self.sla.last_op_engaged_frame = 0
    self.sla.op_engaged = False
    self.sla.op_engaged_prev = False
    self.sla._speed_limit = 0.
    self.sla.speed_limit_prev = 0.
    self.sla.last_valid_speed_limit_offsetted = 0.
    self.sla._distance = 0.
    self.events_sp.clear()

  def initialize_active_state(self, initialize_v_cruise):
    self.sla.state = SpeedLimitAssistState.active
    self.sla.v_cruise_cluster = initialize_v_cruise

  def test_initial_state(self):
    assert self.sla.state == SpeedLimitAssistState.disabled
    assert not self.sla.is_enabled
    assert not self.sla.is_active
    assert V_CRUISE_UNSET == self.sla.get_v_target_from_control()

  @parameterized.expand([RIVIAN.RIVIAN_R1, TESLA.TESLA_MODEL_Y], names=["car_name"])
  def test_disallowed_brands(self, car_name):
    """
      Speed Limit Assist is disabled for the following brands and conditions:
      - All Tesla and is a release branch;
      - All Rivian
    """
    self.car_name = car_name
    self.openpilot_setup_method()  # rebuild the platform for this brand

    assert not self.sla.enabled

    # stay disallowed even when the param may have changed from somewhere else
    self.params.put("SpeedLimitMode", int(Mode.smartspeed), block=True)
    for _ in range(int(PARAMS_UPDATE_PERIOD / DT_MDL)):
      self.sla.update(True, False, SPEED_LIMITS['city'], 0, SPEED_LIMITS['highway'], SPEED_LIMITS['city'],
                      SPEED_LIMITS['city'], True, 0, self.events_sp)
    assert not self.sla.enabled

  def test_disabled(self):
    self.params.put("SpeedLimitMode", int(Mode.off), block=True)
    for _ in range(int(10. / DT_MDL)):
      self.sla.update(True, False, SPEED_LIMITS['city'], 0, SPEED_LIMITS['highway'], SPEED_LIMITS['city'], SPEED_LIMITS['city'], True, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.disabled

  def test_transition_disabled_to_active_with_no_confirmation(self):
    for _ in range(int(3. / DT_MDL)):
      self.sla.update(True, False, SPEED_LIMITS['city'], 0, SPEED_LIMITS['highway'], SPEED_LIMITS['city'], SPEED_LIMITS['city'], True, 0, self.events_sp)
    assert self.sla.state in ACTIVE_STATES
    assert self.sla.output_v_target == SPEED_LIMITS['city']

  def test_set_speed_change_does_nothing(self):
    self.initialize_active_state(SPEED_LIMITS['highway'])
    self.sla.update(True, False, SPEED_LIMITS['city'], 0, SPEED_LIMITS['highway'] + 5, SPEED_LIMITS['city'], SPEED_LIMITS['city'], True, 0, self.events_sp)
    assert self.sla.state in ACTIVE_STATES
    assert self.sla.output_v_target == SPEED_LIMITS['city']

  def test_limit_is_a_dashboard_speed(self):
    # PSA: the dashboard shows the set speed + 3 km/h, so a limit of 50 is held at 47, as a set speed of 50 is
    offset = 3. * CV.KPH_TO_MS
    limit = 50. * CV.KPH_TO_MS
    for _ in range(int(3. / DT_MDL)):
      self.sla.update(True, False, limit, 0, SPEED_LIMITS['highway'], limit, limit, True, 0, self.events_sp, offset)
    assert self.sla.state in ACTIVE_STATES
    assert abs(self.sla.output_v_target - 47. * CV.KPH_TO_MS) < 1e-6

  def test_smartspeed_always_uses_the_network(self):
    self.params.put("SpeedLimitPolicy", int(Policy.map_data_priority), block=True)
    self.sla.frame = 0
    self.sla.update_params()
    assert self.params.get("SpeedLimitPolicy", return_default=True) == Policy.map_data_vision

  def test_transition_disabled_to_pending_no_speed_limit_not_max_initial_set_speed(self):
    for _ in range(int(3. / DT_MDL)):
      self.sla.update(True, False, SPEED_LIMITS['highway'], 0, SPEED_LIMITS['city'], 0, 0, False, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.pending
    assert self.sla.is_enabled and not self.sla.is_active

  def test_pending_to_active_when_speed_limit_available(self):
    self.sla.state = SpeedLimitAssistState.pending

    self.sla.update(True, False, SPEED_LIMITS['highway'], 0, self.pcm_long_max_set_speed,
                    SPEED_LIMITS['highway'], SPEED_LIMITS['highway'], True, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.active

  def test_pending_to_adapting_when_below_speed_limit(self):
    self.sla.state = SpeedLimitAssistState.pending

    self.sla.update(True, False, SPEED_LIMITS['highway'] + 5, 0, self.pcm_long_max_set_speed,
                    SPEED_LIMITS['highway'], SPEED_LIMITS['highway'], True, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.adapting
    assert self.sla.is_enabled and self.sla.is_active

  def test_active_to_adapting_transition(self):
    self.initialize_active_state(self.pcm_long_max_set_speed)

    self.sla.update(True, False, SPEED_LIMITS['highway'] + 2, 0, self.pcm_long_max_set_speed, SPEED_LIMITS['highway'],
                    SPEED_LIMITS['highway'], True, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.adapting

  def test_adapting_to_active_transition(self):
    self.sla.state = SpeedLimitAssistState.adapting

    self.sla.update(True, False, SPEED_LIMITS['city'], 0, self.pcm_long_max_set_speed, SPEED_LIMITS['highway'],
                    SPEED_LIMITS['highway'], True, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.active

  # TODO-SP: test lower CST cases
  def test_rapid_speed_limit_changes(self):
    self.initialize_active_state(self.pcm_long_max_set_speed)
    speed_limits = [SPEED_LIMITS['highway'], SPEED_LIMITS['freeway']]

    for _, speed_limit in enumerate(speed_limits):
      self.sla.update(True, False, speed_limit, 0, self.pcm_long_max_set_speed, speed_limit, speed_limit, True, 0, self.events_sp)
    assert self.sla.state in ACTIVE_STATES

  def test_invalid_speed_limits_handling(self):
    self.initialize_active_state(self.pcm_long_max_set_speed)

    invalid_limits = [-10, 0, 200 * CV.MPH_TO_MS]

    for invalid_limit in invalid_limits:
      self.sla.update(True, False, SPEED_LIMITS['city'], 0, self.pcm_long_max_set_speed, invalid_limit, SPEED_LIMITS['city'], True, 0, self.events_sp)
      assert isinstance(self.sla.output_v_target, (int, float))
      assert self.sla.output_v_target == V_CRUISE_UNSET or self.sla.output_v_target > 0

  def test_stale_data_handling(self):
    self.initialize_active_state(self.pcm_long_max_set_speed)
    old_speed_limit = SPEED_LIMITS['city']

    self.sla.update(True, False, SPEED_LIMITS['city'], 0, self.pcm_long_max_set_speed, 0, old_speed_limit, True, 0, self.events_sp)
    assert self.sla.state in ACTIVE_STATES
    assert self.sla.output_v_target == old_speed_limit

  def test_distance_based_adapting(self):
    self.sla.state = SpeedLimitAssistState.adapting

    distance = 100.0
    current_speed = SPEED_LIMITS['freeway']
    target_speed = SPEED_LIMITS['highway']

    self.sla.update(True, False, current_speed, 0, self.pcm_long_max_set_speed, target_speed, target_speed, True, distance, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.adapting
    assert self.sla.output_v_target == target_speed  # TODO-SP: assert expected accel, need to enable self.acceleration_solutions

  def test_long_disengaged_to_disabled(self):
    self.initialize_active_state(self.pcm_long_max_set_speed)

    self.sla.update(False, False, SPEED_LIMITS['city'], 0, self.pcm_long_max_set_speed, SPEED_LIMITS['city'],
                    SPEED_LIMITS['city'], True, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.disabled
    assert self.sla.output_v_target == V_CRUISE_UNSET

  def test_maintain_states_with_no_changes(self):
    """Test that states are maintained when no significant changes occur"""
    test_states = [
      SpeedLimitAssistState.pending,
      SpeedLimitAssistState.active,
      SpeedLimitAssistState.adapting
    ]

    for state in test_states:
      self.sla.state = state
      self.sla.op_engaged = True

      initial_state = state

      self.sla.update(True, False, SPEED_LIMITS['city'], 0, self.pcm_long_max_set_speed, SPEED_LIMITS['city'], SPEED_LIMITS['city'], True, 0, self.events_sp)

      assert self.sla.state in ALL_STATES  # Sanity check

      if initial_state in ACTIVE_STATES:
        assert self.sla.state in ACTIVE_STATES
