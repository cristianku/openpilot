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
Limit is on or Speed Limit Source is Map Data + Vision.
"""
import os
os.environ['GMMU'] = '0'
import time
from typing import NoReturn

import numpy as np
from msgq.visionipc import VisionIpcClient
from openpilot.cereal import messaging
from openpilot.cereal.visionipc import VisionStreamType
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.speed_limits import runner, vision

PERIOD = 0.2  # seconds between the frames read
NICENESS = 10  # the frame's preparation takes 100 ms on a little core: the driving processes come first


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

  # The car's own signals for the road-type network, every message (carState 100 Hz, deviceMotion 20 Hz).
  car = {name: messaging.sub_sock(name, conflate=False) for name in ("carState", "deviceMotion")}

  client = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_NARROW_ROAD, True)
  while not client.connect(False):
    time.sleep(0.1)
  cloudlog.warning(f"speedvisiond: {files.name} on the road camera, {client.width} x {client.height}")

  road_types = None
  if files.road_type is not None:
    try:
      road_types = vision.RoadTypeReader(runner.StepNetwork(runner.compile_network(files.road_type)), files.road_types,
                                         client.width, client.height)
    except Exception:
      cloudlog.exception("speedvisiond: the road type network did not compile or load, the signs only")

  pm = messaging.PubMaster(["speedVisionSP"])
  signnet_input = vision.SignNetInput(client.width, client.height)
  # The country of the general limits: the one chosen for the maps (Settings › OSM, an ISO code), else Switzerland.
  memory = vision.SpeedLimitMemory(Params().get("OsmLocationName") or vision.DEFAULT_COUNTRY)
  memory.garage_known = road_types is not None and "garage" in road_types.classes
  motion = vision.CarMotion()
  v_ego: float | None = None
  next_frame = 0.0
  while True:
    buf = client.recv()
    if buf is None:
      continue
    now = time.monotonic()
    for event in messaging.drain_sock(car["carState"]):
      state = event.carState
      v_ego = state.vEgo
      motion.add_car_state(now, state.steeringAngleDeg, state.leftBlinker, state.rightBlinker, state.yawRate)
    for event in messaging.drain_sock(car["deviceMotion"]):
      yaw = event.deviceMotion.angularVelocityDevice
      motion.add_device_motion(now, yaw.z, yaw.valid)
    if now < next_frame:
      continue
    next_frame = now + PERIOD

    data = np.frombuffer(buf.data, dtype=np.uint8)  # buf.data is a memoryview of the camera's buffer
    y, uv = vision.nv12_planes(data, buf.width, buf.height, buf.stride, buf.uv_offset)
    signs = vision.read_signs(y, uv, signnet_input, finder, classifier, files.classes)
    sure = [sign for sign in signs if sign["confidence"] >= vision.MIN_CONFIDENCE]
    if road_types is not None and road_types.due(now):
      # Right after SignNet, while the GPU is still free of the driving model.
      road_types.step(now, y, uv, v_ego, memory.road_sign_age(now), motion.features(now))
    road_type = road_types.road_type if road_types is not None else None
    speed_limit = memory.update(now, [sign["speed"] for sign in sure if sign["speed"] is not None],
                                [sign["class"] for sign in sure], road_type=road_type, v_ego=v_ego)

    msg = messaging.new_message("speedVisionSP", valid=True)
    msg.speedVisionSP.speedLimit = float(speed_limit * CV.KPH_TO_MS)
    msg.speedVisionSP.modelName = files.name
    msg.speedVisionSP.signRead = memory.sign_read
    msg.speedVisionSP.roadType = road_type or ""
    pm.send("speedVisionSP", msg)


if __name__ == "__main__":
  main()
