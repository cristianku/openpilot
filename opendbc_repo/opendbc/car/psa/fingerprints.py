from opendbc.car.structs import CarParams
from opendbc.car.psa.values import CAR

Ecu = CarParams.Ecu

FW_VERSIONS = {
  CAR.PSA_PEUGEOT_208: {
    # ARTIV - Radar
    (Ecu.fwdRadar, 0x6B6, None): [
        b'212053276', # Peugeot e208 Allure Pack
        b'194504751', # Peugeot e208 GT CC-only
        b'222256113', # Peugeot e208 GT NZ
    ],
  },
  CAR.PSA_PEUGEOT_508: {
    # ARTIV - Radar
    (Ecu.fwdRadar, 0x6B6, None): [
        b'200603842', # Peugeot 508 Hybrid
    ],
  }
}
