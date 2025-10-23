def psa_checksum(address: int, sig, d: bytearray) -> int:
  chk_ini = {0x452: 0x4, 0x38D: 0x7, 0x42D: 0xC}.get(address, 0xB)
  byte = sig.start_bit // 8
  d[byte] &= 0x0F if sig.start_bit % 8 >= 4 else 0xF0
  checksum = sum((b >> 4) + (b & 0xF) for b in d)
  return (chk_ini - checksum) & 0xF

def create_lka_steering(packer, lat_active: bool, apply_torque: int, torque_factor: int, status: int):
# def create_lka_steering(packer, lat_active: bool, apply_torque: float, status: int):
  values = {
    'TORQUE': apply_torque if lat_active else 0,
    # 'LANE_DEPARTURE':0 if not lat_active else 1 if torque>0 else 2,
    # 'DRIVE': 1,
    'STATUS': status,
    # 'LXA_ACTIVATION': 1,
    'TORQUE_FACTOR': torque_factor if lat_active else 0, #lat_active * 70,
    # 'TORQUE_FACTOR': torque_factor if lat_active else 0,
    'SET_ANGLE': 0,
  }

  return packer.make_can_msg('LANE_KEEP_ASSIST', 0, values)


def create_driver_torque(packer, steering):
  # abs(driver_torque) > 10 to keep EPS engaged
  torque = steering['DRIVER_TORQUE']

  if abs(torque) < 10:
    steering['DRIVER_TORQUE'] = 10 if torque > 0 else -10

  return packer.make_can_msg('STEERING', 0, steering)


def create_steering_hold(packer, lat_active: bool, is_dat_dira):
  # set STEERWHL_HOLD_BY_DRV to keep EPS engaged when lat active
  if lat_active:
    is_dat_dira['STEERWHL_HOLD_BY_DRV'] = 1
  return packer.make_can_msg('IS_DAT_DIRA', 2, is_dat_dira)
