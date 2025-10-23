from opendbc.can.packer import CANPacker
from opendbc.car import Bus
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.lateral import apply_meas_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.psa.psacan import create_lka_steering, create_steering_hold, create_driver_torque
from opendbc.car.psa.values import CarControllerParams
import math

# def torque_to_factor(torque: int, steer_max: int) -> int:
#     """Converte torque (0–steer_max) in torque factor (0–100) con curva logaritmica."""
#     if steer_max <= 0:
#         return 0
#     t = max(0, min(torque, steer_max))
#     return int(round(100 * math.log(t + 1) / math.log(steer_max + 1)))

# def smooth_torque_factor(curr_torque: int, prev_torque: int, steer_max: int, alpha: float = 0.1) -> int:
#     """Applica smoothing esponenziale sul torque factor calcolato logaritmicamente."""
#     curr_factor = torque_to_factor(curr_torque, steer_max)
#     prev_factor = torque_to_factor(prev_torque, steer_max)
#     return int(round(alpha * curr_factor + (1 - alpha) * prev_factor))


# def status_cycle(status:int) -> int:
#   new_status = status + 1
#   if new_status == 4:
#     new_status = 2

#   return new_status


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.main])
    self.apply_torque_last = 0
    self.torque_factor_smoothed = 0  # Initialize smoothed value as int
    self.smoothing_alpha = 0.1  # Smoothing factor (0 < alpha < 1, smaller = stronger smoothing)

    # States
    self.READY  = 2
    self.AUTH   = 3
    self.ACTIVE = 4

  def update(self, CC, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators

    # lateral control
    apply_torque = 0
    if CC.latActive:
      # scaled_eps_torque = CS.out.steeringTorqueEps * 100
      new_torque = int(round(CC.actuators.torque * CarControllerParams.STEER_MAX))

      # apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last,
      #                                                 CS.out.steeringTorque, CarControllerParams)

      apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last,
                                                      0, CarControllerParams)

    #  emulate driver torque message at 1 Hz
      if self.frame % 100 == 0:
        can_sends.append(create_driver_torque(self.packer, CS.steering))

    # EPS disengages on steering override, activation sequence 2->3->4 to re-engage
    # STATUS  -  0: UNAVAILABLE, 1: UNSELECTED, 2: READY, 3: AUTHORIZED, 4: ACTIVE
    # if not CC.latActive:
    #   self.status = 2
    # elif not CS.eps_active and not CS.out.steeringPressed:
    #   self.status = 2 if self.status == 4 else self.status + 1
    # else:
    #   self.status = 4

    if not CC.latActive:
      self.status = self.READY
    elif not CS.eps_active and not CS.out.steeringPressed:
      self.status = self.READY if self.status == self.ACTIVE  else self.status + 1
    else:
      self.status = self.ACTIVE

    # if CC.latActive:
    #   if CS.eps_active:
    #     # EPS is already active no need to do anything
    #     self.status = self.ACTIVE
    #   else:
    #     # Why should not activate the  EPS if the driver is actively "pressing"
    #     # on the steering wheel ?

    #     # if not CS.out.steeringPressed:
    #     # EPS not active and driver not pressing → progress activation sequence (2→3→4)
    #       self.status = status_cycle(self.status)
    # else:
    #   # Lateral control inactive → reset to READY state
    #   self.status = self.READY


    # # Calcolo del TORQUE_FACTOR target in base all'angolo del volante
    # steering_angle_abs = abs(CS.out.steeringAngleDeg)
    # if not CC.latActive:
    #     target_torque_factor = 0
    # else:
    #     # Minimum 60, scales linearly to 100 at 30 degrees
    #     target_torque_factor = min(100, max(60, 60 + (100 - 60) * steering_angle_abs / 15))

    # Applicazione dello smoothing esponenziale
    # self.torque_factor_smoothed = int(round(
    #     self.smoothing_alpha * target_torque_factor +
    #     (1 - self.smoothing_alpha) * self.torque_factor_smoothed
    # ))

    # can_sends.append(create_lka_steering(self.packer, CC.latActive, apply_torque, self.status))
    can_sends.append(create_lka_steering(
        self.packer,
        CC.latActive,
        apply_torque,
        # smooth_torque_factor(apply_torque, self.apply_torque_last, CarControllerParams.STEER_MAX),
        100,
        self.status
    ))

    if self.frame % 10 == 0:
      # send steering wheel hold message at 10 Hz to keep EPS engaged
      can_sends.append(create_steering_hold(self.packer, CC.latActive, CS.is_dat_dira))

    self.apply_torque_last = apply_torque

    new_actuators = actuators.as_builder()
    new_actuators.torque = apply_torque / CarControllerParams.STEER_MAX
    new_actuators.torqueOutputCan = apply_torque

    # self.frame += 1
    self.frame = (self.frame + 1) % 10000

    return new_actuators, can_sends
