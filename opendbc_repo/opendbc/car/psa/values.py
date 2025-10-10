from dataclasses import dataclass, field

from opendbc.car.structs import CarParams
from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, uds

Ecu = CarParams.Ecu


class CarControllerParams:
  # TODO: tune these params
  STEER_MAX = 10  # TODO: find max torque
  # STEER_MAX_LOOKUP = [9, 17], [200, 100]
  STEER_STEP = 1
  STEER_DELTA_UP = 1  # TODO:torque increase per refresh
  STEER_DELTA_DOWN = 1  # TODO:torque decrease per refresh
  STEER_DRIVER_MULTIPLIER = 1  #  TODO:weight driver torque
  STEER_DRIVER_FACTOR = 1
  STEER_DRIVER_ALLOWANCE = 5  # TODO: allowed driver torque before start limiting

  def __init__(self, CP):
    pass

@dataclass
class PSACarDocs(CarDocs):
  package: str = "Adaptive Cruise Control (ACC) & Lane Assist"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.psa_a]))


@dataclass
class PSAPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.pt: 'psa_aee2010_r3',
  })


class CAR(Platforms):
  PSA_PEUGEOT_208 = PSAPlatformConfig(
    [PSACarDocs("Peugeot 208 2019-25")],
    CarSpecs(mass=1530, wheelbase=2.54, steerRatio=17.6),
  )
  PSA_PEUGEOT_508 = PSAPlatformConfig(
    [PSACarDocs("Peugeot 508 2019-23")],
    CarSpecs(mass=1720, wheelbase=2.79, steerRatio=17.6), # TODO: set steerRatio
  )
  PEUGEOT_3008 = PSAPlatformConfig(
    [PSACarDocs("Peugeot 3008 2016-20")],
    CarSpecs(mass=1577, wheelbase=2675, steerRatio=16.2), # TODO: set steerRatio
  )


PSA_DIAG_REQ  = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL, 0x01])
PSA_DIAG_RESP = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL + 0x40, 0x01])

PSA_SERIAL_REQ = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER,  0xF1, 0x8C])
PSA_SERIAL_RESP = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40, 0xF1, 0x8C])

PSA_VERSION_REQ  = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER, 0xF0, 0xFE])
PSA_VERSION_RESP = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40, 0xF0, 0xFE])

PSA_RX_OFFSET = -0x20

FW_QUERY_CONFIG = FwQueryConfig(
  requests=[request for bus in (0, 1, 2) for request in [
    Request(
      [PSA_DIAG_REQ, PSA_SERIAL_REQ],
      [PSA_DIAG_RESP, PSA_SERIAL_RESP],
      rx_offset=PSA_RX_OFFSET,
      bus=bus,
      obd_multiplexing=False,
    ),
    Request(
      [PSA_DIAG_REQ, PSA_VERSION_REQ],
      [PSA_DIAG_RESP, PSA_VERSION_RESP],
      rx_offset=PSA_RX_OFFSET,
      bus=bus,
      obd_multiplexing=False,
    ),
  ]]
)

DBC = CAR.create_dbc_map()
