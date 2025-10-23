import copy
from opendbc.car import structs, Bus
from opendbc.can.parser import CANParser
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.psa.values import CAR, DBC, CarControllerParams
from opendbc.car.interfaces import CarStateBase

GearShifter = structs.CarState.GearShifter
TransmissionType = structs.CarParams.TransmissionType


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.is_dat_dira = None
    self.steering = None
    # These parameters are for Peugeot 3008 driver torque jittery sensor
    self.prev_steering_torque = 0.0  # Initialize the previous torque value to 0
    self.alpha = 0.1                # Smoothing factor (adjust between 0 and 1)

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.main]
    cp_adas = can_parsers[Bus.adas]
    cp_cam = can_parsers[Bus.cam]
    ret = structs.CarState()

    # car speed
    self.parse_wheel_speeds(ret,
      cp.vl['Dyn4_FRE']['P263_VehV_VPsvValWhlFrtL'],
      cp.vl['Dyn4_FRE']['P264_VehV_VPsvValWhlFrtR'],
      cp.vl['Dyn4_FRE']['P265_VehV_VPsvValWhlBckL'],
      cp.vl['Dyn4_FRE']['P266_VehV_VPsvValWhlBckR'],
    )
    ret.yawRate = cp_adas.vl['HS2_DYN_UCF_MDD_32D']['VITESSE_LACET_BRUTE'] * CV.DEG_TO_RAD
    ret.standstill = cp.vl['Dyn4_FRE']['P263_VehV_VPsvValWhlFrtL'] < 0.1

    # gas
    if self.CP.carFingerprint == CAR.PSA_PEUGEOT_3008:
      ret.gasPressed = cp.vl['Dyn5_CMM']['P334_ACCPed_Position'] > 0
    else:
      ret.gasPressed = cp_cam.vl['DRIVER']['GAS_PEDAL'] > 0

    # brake pressed
    ret.brakePressed = bool(cp_cam.vl['Dat_BSI']['P013_MainBrake'])

    # brake pressure
    if self.CP.carFingerprint == CAR.PSA_PEUGEOT_3008:
      raw = cp.vl["Dyn2_FRE"]["BRAKE_PRESSURE"]
      ret.brake = max(0.0, float(raw) - 550.0)  # clamp a 0

    # parking brake
    ret.parkingBrake = cp.vl['Dyn_EasyMove']['P337_Com_stPrkBrk'] == 1 # 0: disengaged, 1: engaged, 3: brake actuator moving

    # steering wheel
    STEERING_ALT_BUS = {
      CAR.PSA_PEUGEOT_208: cp.vl,
      CAR.PSA_PEUGEOT_508: cp_cam.vl,
      CAR.PSA_PEUGEOT_3008: cp.vl,
    }
    bus = STEERING_ALT_BUS[self.CP.carFingerprint]
    ret.steeringAngleDeg = bus['STEERING_ALT']['ANGLE'] # EPS
    ret.steeringRateDeg  = bus['STEERING_ALT']['RATE'] * (2 * bus['STEERING_ALT']['RATE_SIGN'] - 1) # convert [0,1] to [-1,1] EPS: rot. speed * rot. sign

    if self.CP.carFingerprint == CAR.PSA_PEUGEOT_3008:
      # In the electric power steering (EPS) system of the Peugeot 3008,
      # the torque sensor is mounted on the steering column, not inside the assist motor.
      # Therefore, it's not possible to obtain the actual assist (motor) torque separately.
      # Instead, we can use the smoothed EPS_TORQUE value from IS_DAT_DIRA as the driver torque,
      # since it doesn’t appear to include the LKA-commanded torque from the CarController.
        ret.steeringTorque = cp.vl['IS_DAT_DIRA']['EPS_TORQUE'] * 10
        ret.steeringTorqueEps = 0
    else:
        ret.steeringTorque = cp.vl['STEERING']['DRIVER_TORQUE']
        ret.steeringTorqueEps = cp.vl['IS_DAT_DIRA']['EPS_TORQUE']

    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > CarControllerParams.STEER_DRIVER_ALLOWANCE, 5)
    self.eps_active = cp.vl['IS_DAT_DIRA']['EPS_STATE_LKA'] == 3 # 0: Unauthorized, 1: Authorized, 2: Available, 3: Active, 4: Defect
    self.is_dat_dira = copy.copy(cp.vl['IS_DAT_DIRA'])
    self.steering = copy.copy(cp.vl['STEERING'])

    # cruise
    ret.cruiseState.speed = cp_adas.vl['HS2_DAT_MDD_CMD_452']['SPEED_SETPOINT'] * CV.KPH_TO_MS # set to 255 when ACC is off, -2 kph offset from dash speed
    ret.cruiseState.enabled = cp_adas.vl['HS2_DAT_MDD_CMD_452']['RVV_ACC_ACTIVATION_REQ'] == 1
    ret.cruiseState.available = True # not available for CC-only
    ret.cruiseState.nonAdaptive = False # not available for CC-only
    ret.cruiseState.standstill = False # not available for CC-only
    ret.accFaulted = False # not available for CC-only

    # gear
    if bool(cp_cam.vl['Dat_BSI']['P103_Com_bRevGear']):
      ret.gearShifter = GearShifter.reverse
    else:
      ret.gearShifter = GearShifter.drive

    # blinkers
    blinker = cp_cam.vl['HS2_DAT7_BSI_612']['CDE_CLG_ET_HDC']
    if self.CP.carFingerprint == CAR.PSA_PEUGEOT_3008:
      ret.leftBlinker = blinker == 2
      ret.rightBlinker = blinker == 1
    else:
      ret.leftBlinker = blinker == 1
      ret.rightBlinker = blinker == 2

    # Blind sensor ( there is not left and right )
    if self.CP.carFingerprint == CAR.PSA_PEUGEOT_3008:
      ret.leftBlindspot = cp_adas.vl["HS2_DYN_MDD_ETAT_2F6"]["BLIND_SENSOR"] != 0
      ret.rightBlindspot = cp_adas.vl["HS2_DYN_MDD_ETAT_2F6"]["BLIND_SENSOR"] != 0

    # Auto Braking in progress
    if self.CP.carFingerprint == CAR.PSA_PEUGEOT_3008:
      ret.stockAeb = cp_adas.vl["HS2_DYN1_MDD_ETAT_2B6"]["AUTO_BRAKING_STATUS"] == 1

    # lock info
    ret.doorOpen = any((cp_cam.vl['Dat_BSI']['DRIVER_DOOR'], cp_cam.vl['Dat_BSI']['PASSENGER_DOOR']))
    ret.seatbeltUnlatched = cp_cam.vl['RESTRAINTS']['DRIVER_SEATBELT'] != 2

    return ret

  @staticmethod
  def get_can_parsers(CP):
    return {
      Bus.main: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 0),
      Bus.adas: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 1),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 2),
    }
