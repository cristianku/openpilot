//asdfadsfasd
def psa_checksum(address: int, sig, d: bytearray) -> int:
  chk_ini = {0x452: 0x4, 0x38D: 0x7, 0x42D: 0xC}.get(address, 0xB)
  byte = sig.start_bit // 8
  d[byte] &= 0x0F if sig.start_bit % 8 >= 4 else 0xF0
  checksum = sum((b >> 4) + (b & 0xF) for b in d)
  return (chk_ini - checksum) & 0xF


def create_lka_steering(packer, lat_active: bool, apply_torque: float, status: int):
  values = {
    'TORQUE': apply_torque,
    # 'LANE_DEPARTURE':0 if not lat_active else 1 if torque>0 else 2,
    # 'DRIVE': 1,
    'STATUS': status,
    # 'LXA_ACTIVATION': 1,
    'TORQUE_FACTOR': lat_active * 100,
    'SET_ANGLE': 0,
  }

  return packer.make_can_msg('LANE_KEEP_ASSIST', 0, values)
// ciao sdfads 
