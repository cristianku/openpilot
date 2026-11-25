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
READS THE MAP: MAP AND VISION ARE TWO SEPARATE SOURCES, WEIGHED ONLY BY THE RESOLVER. ONE EXCEPTION (CRISTIAN, 14 NOV 2026):
A BRAIN TRAINED WITH THE MAP'S LIMIT AS AN INPUT IS GIVEN liveMapDataSP's LIMIT, AND ONLY SUCH A BRAIN. It runs when Show Vision Speed
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
at every reading for a brain at 4 Hz), and the speed it says is speedLimit. That speed is all that is read from a brain:
with one speedLimitAhead and speedLimitAheadDistance stay 0, also when the network gives a next limit and the metres to it
(the brains trained until 12 Nov 2026). Neither the rules of the memory, nor the road-type network, nor vision.LimitAhead
are run. The distance to a sign stays on the screen; signRead pulses on a number confirmed.
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
from openpilot.sunnypilot.speed_limits import applicability, brain, pipeline, runner, vision
from openpilot.sunnypilot.speed_limits.country import GpsFix

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
    classifier_path = runner.compile_network(files.classifier)
    classifier = runner.StepNetwork(classifier_path) if getattr(files, "classifier_countries", ()) else runner.Network(classifier_path)
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
  # The map's limit (liveMapDataSP, once a second), only for a brain trained with it as an input (brain_inputs' group
  # map_limit, Cristian 14 Nov 2026): BrainReader takes nothing of it otherwise, and no rule here reads it.
  map_data = messaging.sub_sock("liveMapDataSP", conflate=True)

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
  # Until GPS gives one: the country chosen in Settings › OSM (an ISO code), else Switzerland. Then the last one GPS gave.
  params = Params()
  settings_country = params.get("OsmLocationName") or pipeline.DEFAULT_COUNTRY
  gps_service = get_gps_location_service(params)
  gps = messaging.sub_sock(gps_service, conflate=True)
  # Everything between these sockets and the message sent (pipeline.py): the same code replays a recorded drive.
  speed_vision = pipeline.SpeedVision(files, client.width, client.height, focal_length(client.width, client.height),
                                      finder, classifier, reader=reader, road_types=road_types, ruler=ruler,
                                      settings_country=settings_country)
  if speed_vision.country is None:
    cloudlog.warning(f"speedvisiond: no country policy/borders, the country of the settings: {settings_country}")
  while True:
    buf = client.recv()
    if buf is None:
      continue
    now = time.monotonic()
    for event in messaging.drain_sock(car["carState"]):
      state = event.carState
      speed_vision.car_state(now, event.logMonoTime * 1e-9, state.vEgo, state.steeringAngleDeg, state.leftBlinker,
                             state.rightBlinker, state.yawRate, state.gasPressed, state.brakePressed, state.standstill)
    for event in messaging.drain_sock(car["selfdriveState"]):
      speed_vision.enabled(now, event.selfdriveState.enabled)
    for event in messaging.drain_sock(car["deviceMotion"]):
      yaw = event.deviceMotion.angularVelocityDevice
      speed_vision.device_motion(now, yaw.z, yaw.valid)
    for event in messaging.drain_sock(radar):
      one = event.radarState.leadOne
      speed_vision.radar(one.present, one.dRel, one.vLead)
    for event in messaging.drain_sock(map_data):
      speed_vision.map_limit(now, event.liveMapDataSP.speedLimit, event.liveMapDataSP.speedLimitValid)
    if not speed_vision.due(now):
      continue

    for event in messaging.drain_sock(gps):
      g = getattr(event, gps_service)
      speed_vision.gps(GpsFix(event.logMonoTime * 1e-9, g.latitude, g.longitude, g.horizontalAccuracy,
                              bool(event.valid and g.hasFix)), g.unixTimestampMillis)

    data = np.frombuffer(buf.data, dtype=np.uint8)  # buf.data is a memoryview of the camera's buffer
    y, uv = vision.nv12_planes(data, buf.width, buf.height, buf.stride, buf.uv_offset)
    msg = messaging.new_message("speedVisionSP", valid=True)
    for name, value in speed_vision.frame(now, y, uv, full_planes(full, client.frame_id)).items():
      setattr(msg.speedVisionSP, name, value)
    pm.send("speedVisionSP", msg)


if __name__ == "__main__":
  main()
