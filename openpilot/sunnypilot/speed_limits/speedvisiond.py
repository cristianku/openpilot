#!/usr/bin/env python3
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

speedvisiond: the speed limit read on the road camera by the speed-limits networks (speedVisionSP, m/s, 0 when there
is no reading), a few times a second. SignNet finds the signs in the top of the frame, the classifier reads each,
and a reading becomes the limit when two frames agree; it stays until another sign. A sign of the kind of road (a
motorway, a town, their ends) gives the road's general limit, and the limit of a motorway ramp ends at the start of
the acceleration lane, or once the car is on the carriageway (vision.SpeedLimitMemory). Once a second the road-type network, when the release has one, sees the
whole frame with the car's speed, steering, blinkers and yaw (carState, deviceMotion): where no sign gives the limit
the scene does (a garage 10, a car park in the open 20, out of it the street's general limit, a town 50). IT NEVER
READS THE MAP: MAP AND VISION ARE TWO SEPARATE SOURCES, WEIGHED ONLY BY THE RESOLVER. It runs when Show Vision Speed
Limit is on or Speed Limit Source is Smartspeed. For the logs, each message also says what the limit says
(speedLimitState), the rule that set the limit held (speedLimitReason) and the last sign read, taken or not (rawSign*);
the screen's reading square shows that last sign, and whether it is in view now (rawSignSeen).
The networks read only the number on a sign; it becomes m/s in the unit of the country (vision.sign_unit: mph in the UK,
the US and the places that follow them, km/h elsewhere): the last country GPS confirmed when the release has borders
and a country policy, else the one of the settings.
While a round sign (a number, the end of a limit) or a motorway or expressway sign or its end is ahead, the distance
to it from its size on the camera and the metres driven (signDistance, vision.SignDistance) shows on the screen for
the driver to check. A limit read on such a sign starts at the sign, as the map's: until the car gets there it is the
limit ahead, with its distance (speedLimitAhead, speedLimitAheadDistance, vision.LimitAhead).
A release with a brain (brain.BrainReader) is another vision limit: what was read, the distance to the sign, the car
ahead, the hour and the country go into that network at the rate of its contract (once a second for the first brains,
at every reading for a brain at 4 Hz), the speed it says is speedLimit, and a brain trained with them gives the next
limit and the metres to it, speedLimitAhead and speedLimitAheadDistance, counted down by the metres driven between its
steps. Neither the rules of the memory, nor the road-type network, nor vision.LimitAhead are run. The distance to a sign
stays on the screen; signRead pulses on a number confirmed.
When the release has the applicability network (applicability.Applicability), it rules right after the classifier the
signs of the families it learnt: a sign it gives to another road (a side street, an exit, the lane beside) is not
taken, neither by the memory, nor by the brain, nor by the distance; the reading square still shows it, with
rawSignTaken false and rawSignRoad other_road. A release without it takes every sign, as before.
With the Developer switch Read Signs at Full Resolution the classifier's crops come from the full stream of camerad, the
same frame before the scaling; SignNet, the road-type network and the distance stay on the low frame.
"""
import os
os.environ['GMMU'] = '0'
import time
from typing import NoReturn

import numpy as np
from msgq.visionipc import VisionIpcClient
from openpilot.cereal import messaging
from openpilot.cereal.visionipc import VisionStreamType
from openpilot.common.params import Params
from openpilot.common.gps import get_gps_location_service
from openpilot.common.swaglog import cloudlog
from openpilot.common.transformations.camera import DEVICE_CAMERAS
from openpilot.sunnypilot.speed_limits import applicability, brain, runner, vision
from openpilot.sunnypilot.speed_limits.country import CountryResolver, OfflineBorders, GpsFix

PERIOD = 0.2  # seconds between the frames read
FULL_FRAME_WAIT = 10  # ms for the full stream's frame, already sent when the low one comes
NICENESS = 10  # the frame's preparation takes 100 ms on a little core: the driving processes come first


def full_stream(params: Params) -> VisionIpcClient | None:
  """The road camera before the scaling when the Developer switch asks for it and camerad has it, else None."""
  if not params.get_bool("ReadSignsFullResolution"):
    return None
  if VisionStreamType.VISION_STREAM_NARROW_ROAD_FULL not in VisionIpcClient.available_streams("camerad", False):
    cloudlog.warning("speedvisiond: no full road stream, the signs from the low frames")
    return None
  client = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_NARROW_ROAD_FULL, True)
  while not client.connect(False):
    time.sleep(0.1)
  cloudlog.warning(f"speedvisiond: the signs from the full road stream, {client.width} x {client.height}")
  return client


def full_planes(client: VisionIpcClient | None, frame_id: int) -> tuple[np.ndarray, np.ndarray] | None:
  """The planes of the full stream's frame frame_id, None when it has no such frame right now: camerad sends it just
  before the low one, so it is there, unless a newer one came in the meantime and the low frame serves this time."""
  buf = client.recv(FULL_FRAME_WAIT) if client is not None else None
  if buf is None or client.frame_id != frame_id:
    return None
  return vision.nv12_planes(np.frombuffer(buf.data, dtype=np.uint8), buf.width, buf.height, buf.stride, buf.uv_offset)


def focal_length(width: int, height: int) -> float:
  """The road camera's focal length (pixels) for its frames of width x height; 0 for a camera not known (no distance)."""
  return next((config.narrow_road.focal_length for config in DEVICE_CAMERAS.values()
               if (config.narrow_road.width, config.narrow_road.height) == (width, height)), 0.0)


def idle(reason: str) -> NoReturn:
  cloudlog.warning(f"speedvisiond: {reason}, no reading")
  while True:
    time.sleep(60)


def main() -> None:
  os.nice(NICENESS)
  files = runner.networks()
  if files is None:
    idle("the models repository has no speed-limits networks")
  taken = vision.apply_settings(files.settings)
  if taken:
    cloudlog.warning(f"speedvisiond: settings of the release {taken}")
  try:
    signnet = runner.Network(runner.compile_network(files.finder))
    classifier = runner.Network(runner.compile_network(files.classifier))
  except Exception:
    cloudlog.exception("speedvisiond: the networks did not compile or load")
    idle("no networks")

  # SignNet takes the GPU for about 20 ms, which the driving model shares at the same priority: it runs right after
  # the driving model has given its output, when the GPU is free until the next frame (or after 100 ms without one).
  sm = messaging.SubMaster(["modelV2"])

  def finder(x):
    sm.update(0)
    sm.update(100)
    return signnet(x)

  # The car's own signals for the road-type network and the brain, every message (carState 100 Hz, deviceMotion 20 Hz,
  # selfdriveState 100 Hz: whether openpilot is engaged).
  car = {name: messaging.sub_sock(name, conflate=False) for name in ("carState", "deviceMotion", "selfdriveState")}
  radar = messaging.sub_sock("radarState", conflate=True)

  client = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_NARROW_ROAD, True)
  while not client.connect(False):
    time.sleep(0.1)
  cloudlog.warning(f"speedvisiond: {files.name} on the road camera, {client.width} x {client.height}")

  full = full_stream(Params())

  reader = None
  if files.brain is not None:
    try:
      reader = brain.BrainReader(runner.StepNetwork(runner.compile_network(files.brain)), files.brain_contract, vision,
                                 client.width, client.height)
    except Exception:
      cloudlog.exception("speedvisiond: the brain did not compile or load")
      idle("no brain")

  road_types = None
  if files.road_type is not None and reader is None:
    try:
      road_types = vision.RoadTypeReader(runner.StepNetwork(runner.compile_network(files.road_type)), files.road_types,
                                         client.width, client.height)
    except Exception:
      cloudlog.exception("speedvisiond: the road type network did not compile or load, the signs only")

  ruler = None
  if files.decider is not None:
    try:
      ruler = applicability.Applicability(runner.StepNetwork(runner.compile_network(files.decider)), files.decider_contract,
                                          vision, client.width, client.height)
    except Exception:
      cloudlog.exception("speedvisiond: the applicability network did not compile or load, every sign is taken")

  pm = messaging.PubMaster(["speedVisionSP"])
  signnet_input = vision.SignNetInput(client.width, client.height)
  # GPS selects only a country and sign unit. No map speed/road/junction data enters vision.
  # Until GPS gives one: the country chosen in Settings › OSM (an ISO code), else Switzerland. Then the last one GPS gave.
  params = Params()
  settings_country = params.get("OsmLocationName") or "CH"
  gps_country: str | None = None
  memory = vision.SpeedLimitMemory(settings_country)
  gps_service = get_gps_location_service(params)
  gps = messaging.sub_sock(gps_service, conflate=True)
  borders = OfflineBorders.load()
  country = CountryResolver.from_settings(borders, files.settings.get("country_policy", {}))
  if country is None:
    cloudlog.warning(f"speedvisiond: no country policy/borders, the country of the settings: {settings_country}")
  memory.garage_known = road_types is not None and "garage" in road_types.classes
  motion = vision.CarMotion()
  distance = vision.SignDistance(focal_length(client.width, client.height))
  limit_ahead = vision.LimitAhead()
  v_ego: float | None = None
  lead: tuple[float, float] | None = None  # metres and km/h of the car ahead
  gps_clock: tuple[float, float] | None = None  # the GPS clock of the last fix (s) and when it came
  step_odometer = 0.0  # the odometer at the brain's last step
  next_frame = 0.0
  while True:
    buf = client.recv()
    if buf is None:
      continue
    now = time.monotonic()
    for event in messaging.drain_sock(car["carState"]):
      state = event.carState
      v_ego = state.vEgo
      distance.add_speed(event.logMonoTime * 1e-9, v_ego)
      motion.add_car_state(now, state.steeringAngleDeg, state.leftBlinker, state.rightBlinker, state.yawRate)
      if reader is not None:
        reader.add_pedals(now, state.gasPressed, state.brakePressed)
        reader.add_car_state(now, state.steeringAngleDeg, state.leftBlinker, state.rightBlinker, state.yawRate)
    for event in messaging.drain_sock(car["selfdriveState"]):
      if reader is not None:
        reader.add_enabled(now, event.selfdriveState.enabled)
    for event in messaging.drain_sock(car["deviceMotion"]):
      yaw = event.deviceMotion.angularVelocityDevice
      motion.add_device_motion(now, yaw.z, yaw.valid)
      if reader is not None:
        reader.add_device_motion(now, yaw.z, yaw.valid)
    for event in messaging.drain_sock(radar):
      one = event.radarState.leadOne
      lead = (one.dRel, one.vLead * 3.6) if one.present else None
    if now < next_frame:
      continue
    next_frame = now + PERIOD

    fix = None
    for event in messaging.drain_sock(gps):
      g = getattr(event, gps_service)
      fix = GpsFix(event.logMonoTime * 1e-9, g.latitude, g.longitude, g.horizontalAccuracy,
                   bool(event.valid and g.hasFix))
      if fix.valid and g.unixTimestampMillis:
        gps_clock = g.unixTimestampMillis / 1000, now
    gps_country = (country.update(now, fix) if country is not None else None) or gps_country
    memory.set_country(gps_country or settings_country)

    data = np.frombuffer(buf.data, dtype=np.uint8)  # buf.data is a memoryview of the camera's buffer
    y, uv = vision.nv12_planes(data, buf.width, buf.height, buf.stride, buf.uv_offset)
    signs = vision.read_signs(y, uv, signnet_input, finder, classifier, files.classes, full_planes(full, client.frame_id))
    raw = vision.raw_reading(signs)  # of all the signs: the reading square shows a sign of another road too
    seen = signs
    if ruler is not None:
      # Right after the classifier, while the GPU is still free of the driving model.
      ruler.rule(y, uv, signs, v_ego)
      signs = applicability.taken(signs)
    msg = messaging.new_message("speedVisionSP", valid=True)
    if reader is not None:
      reader.add_signs(now, signs)
      sign_distance = distance.update(now, signs, None)
      if reader.due(now):
        # Right after SignNet, while the GPU is still free of the driving model.
        unix = gps_clock[0] + now - gps_clock[1] if gps_clock is not None else None
        reader.step(now, y, uv, lead, unix, gps_country, v_ego, sign_distance)
        step_odometer = distance.odometer
      if raw is not None:
        memory.note_raw(raw)
        # Given to the brain: read sure enough and not of another road.
        memory.raw_taken = raw["confidence"] >= vision.MIN_CONFIDENCE and memory.raw_road != vision.OTHER_ROAD
      msg.speedVisionSP.speedLimit = float(reader.speed_limit)
      if reader.speed_limit_ahead > 0:
        msg.speedVisionSP.speedLimitAhead = float(reader.speed_limit_ahead)
        msg.speedVisionSP.speedLimitAheadDistance = float(max(0.0, reader.ahead_distance - (distance.odometer - step_odometer)))
      msg.speedVisionSP.signRead = reader.signs.read_at == now
      memory.fill_log(msg.speedVisionSP)  # for the logs: the last sign read
      msg.speedVisionSP.speedLimitState = "limit" if reader.speed_limit > 0 else "unknown"
      msg.speedVisionSP.speedLimitReason = "brain"
    else:
      if road_types is not None and road_types.due(now):
        # Right after SignNet, while the GPU is still free of the driving model.
        road_types.step(now, y, uv, v_ego, memory.road_sign_age(now), motion.features(now),
                         memory.board_age(now))
      road_type = road_types.road_type if road_types is not None else None
      speed_limit = memory.update_frame(now, signs, road_type=road_type, v_ego=v_ego, seen=seen)
      sign_distance = distance.update(now, signs, memory.road)
      speed_limit = limit_ahead.update(speed_limit, memory.limit_reason, sign_distance, distance.sign_class, distance.odometer)
      msg.speedVisionSP.speedLimit = float(speed_limit)
      msg.speedVisionSP.speedLimitAhead = float(limit_ahead.ahead)
      msg.speedVisionSP.speedLimitAheadDistance = float(limit_ahead.distance)
      msg.speedVisionSP.signRead = memory.sign_read
      msg.speedVisionSP.roadType = road_type or ""
      memory.fill_log(msg.speedVisionSP)  # for the logs: why the limit is what it is, the last sign read taken or not
    msg.speedVisionSP.modelName = files.name
    msg.speedVisionSP.signDistance = sign_distance
    msg.speedVisionSP.signSize = distance.side
    msg.speedVisionSP.rawSignSeen = raw is not None
    pm.send("speedVisionSP", msg)


if __name__ == "__main__":
  main()
