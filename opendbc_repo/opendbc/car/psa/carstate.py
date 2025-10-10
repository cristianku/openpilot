from opendbc.car import structs, Bus
from opendbc.can.parser import CANParser
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.psa.values import CAR, DBC, CarControllerParams
from opendbc.car.interfaces import CarStateBase

GearShifter = structs.CarState.GearShifter
TransmissionType = structs.CarParams.TransmissionType


class CarState(CarStateBase):
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
    if self.CP.carFingerprint == CAR.PEUGEOT_3008:
      ret.gasPressed = cp.vl['Dyn5_CMM']['P334_ACCPed_Position'] > 0
    else:
      ret.gasPressed = cp_cam.vl['DRIVER']['GAS_PEDAL'] > 0

    # brake
    ret.brakePressed = bool(cp_cam.vl['Dat_BSI']['P013_MainBrake'])
    ret.parkingBrake = cp.vl['Dyn_EasyMove']['P337_Com_stPrkBrk'] == 1 # 0: disengaged, 1: engaged, 3: brake actuator moving

    # steering wheel
    STEERING_ALT_BUS = {
      CAR.PSA_PEUGEOT_208: cp.vl,
      CAR.PSA_PEUGEOT_508: cp_cam.vl,
      CAR.PEUGEOT_3008: cp.vl,
    }
    bus = STEERING_ALT_BUS[self.CP.carFingerprint]
    ret.steeringAngleDeg = bus['STEERING_ALT']['ANGLE'] # EPS
    ret.steeringRateDeg  = bus['STEERING_ALT']['RATE'] * (2 * bus['STEERING_ALT']['RATE_SIGN'] - 1) # convert [0,1] to [-1,1] EPS: rot. speed * rot. sign
    ret.steeringTorque = cp.vl['STEERING']['DRIVER_TORQUE']
    ret.steeringTorqueEps = cp.vl['IS_DAT_DIRA']['EPS_TORQUE']
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > CarControllerParams.STEER_DRIVER_ALLOWANCE, 5)
    self.eps_active = cp.vl['IS_DAT_DIRA']['EPS_STATE_LKA'] == 3 # 0: Unauthorized, 1: Authorized, 2: Available, 3: Active, 4: Defect

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
    ret.leftBlinker = blinker == 1
    ret.rightBlinker = blinker == 2

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
