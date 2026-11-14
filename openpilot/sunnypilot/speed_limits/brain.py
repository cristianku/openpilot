"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

VISION AND MAP ARE TWO SEPARATE SOURCES: NOTHING HERE READS THE MAP (liveMapDataSP).

The brain: the network with memory that says the speed to hold. It is the limit of the vision side when the release has
one: no rule of vision.SpeedLimitMemory and no road-type network decides then. What the reader read goes in as an input
(brain_inputs), the speed that comes out is published as it is. The speed is all the car reads from it: a brain that also
gives the next limit and the metres to it (outputs next and metres, the brains trained until 12 Nov 2026) steps as any
other and those two are not read. The contract's step says how: its rate, its clock ("last": a step once a period has
passed since the last one, the first brains, at 1 Hz; "schedule": steps on a fixed grid of that rate, at the reading that
comes first, as the training's frames are) and its picture ("road_type": vision.RoadTypeInput; "brain":
vision.BrainInput).
"""
import hashlib
import math
from pathlib import Path

import numpy as np

from openpilot.sunnypilot.speed_limits import brain_inputs


class BrainReader:
  """The network step by step: add_signs() on every frame read, step() when due. The picture is the one of its contract,
  bit for bit the frames of the training. speed_limit is the speed said (m/s), held between the steps, 0 before the first;
  road_type the road type it sees (contract output.road_types), for the screen only, "" for a brain without it.
  Raises ValueError on a network that was trained with another brain_inputs.py or asks for what the car does not give."""
  def __init__(self, network, contract: dict, vision, width: int, height: int):
    if contract["inputs_sha256"] not in (hashlib.sha256(Path(brain_inputs.__file__).read_bytes()).hexdigest(), *brain_inputs.SAME_GROUPS):
      raise ValueError("the brain was trained with another brain_inputs.py")
    self.groups = tuple(contract["step"]["groups"])
    if not set(self.groups) <= set(brain_inputs.INPUT_SIZES):
      raise ValueError(f"the brain asks for inputs the car does not give: {self.groups}")
    self.network = network
    self.kph = contract["output"]["kph"]
    self.period = 1.0 / contract["step"]["rate_hz"]
    self.schedule = {"last": False, "schedule": True}[contract["step"].get("clock", "last")]
    self.memory_gap = contract["step"]["memory_gap_seconds"]
    pictures = {"road_type": vision.RoadTypeInput, "brain": vision.BrainInput}
    self.image = pictures[contract["step"].get("picture", "road_type")](width, height)
    self.width, self.height = width, height  # of the frames given to step(): the pixels of the signs' boxes
    self.road_types = contract["output"].get("road_types", [])
    self.signs = brain_inputs.SignInputs(vision)
    self.sign_boxes = brain_inputs.BoxInputs(vision, brain_inputs.SIGN_BOX_CHANNELS, brain_inputs.sign_box_channel(vision))
    self.light_boxes = brain_inputs.BoxInputs(vision, len(brain_inputs.LIGHTS), brain_inputs.light_box_channel)
    self.class_plate = vision.class_plate
    self.plates = {plate for plate, trained in (("time", contract["step"].get("plates", False)),
                                                ("wet", contract["step"].get("wet", False))) if trained}
    self.class_hours = vision.class_hours if contract["step"].get("hours", False) else lambda name: None
    self.hidden = np.zeros(network.input_shape("hidden"), dtype=np.float32)
    self.speed_limit = 0.0
    self.road_type = ""
    self.last_step = -math.inf
    self.next_step = -math.inf
    self.pedals = brain_inputs.PedalInputs()
    self.engaged = brain_inputs.EngagedInputs()
    self.motion = brain_inputs.MotionInputs()

  def add_signs(self, now: float, signs: list[dict]) -> None:
    """A frame's signs as vision.read_signs gives them. A plate only to a brain trained with that kind of plate, and their
    hours only to one trained with hours (its contract's step): to another one, trained with them unknown on every frame,
    they would change the speed at random. Their boxes go as SignNet gave them (sign_boxes, light_boxes), in the pixels of the frame
    it read: the low one, also when the classifier's crops come from the full definition."""
    boxes = [(sign["class"], sign["confidence"], sign["box"]) for sign in signs]
    self.sign_boxes.add(now, boxes, self.width, self.height)
    self.light_boxes.add(now, boxes, self.width, self.height)
    if not self.plates:
      self.signs.add(now, [(sign["class"], sign["confidence"]) for sign in signs])
    else:
      self.signs.add(now, [(sign["class"], sign["confidence"], self._plate(sign["class"]), self.class_hours(sign["class"]))
                           for sign in signs])

  def _plate(self, name: str) -> str | None:
    plate = self.class_plate(name)
    return plate if plate in self.plates else None

  def add_pedals(self, now: float, gas: bool, brake: bool) -> None:
    """Every carState."""
    self.pedals.add(now, gas, brake)
    self.engaged.add_pedals(now, gas, brake)

  def add_car_state(self, now: float, steering_deg: float, left: bool, right: bool, yaw_rate: float) -> None:
    """Every carState: the steering wheel, the turn signals and the car's yaw rate."""
    self.motion.add_car_state(now, steering_deg, left, right, yaw_rate)

  def add_device_motion(self, now: float, z: float, valid: bool) -> None:
    """Every deviceMotion: angularVelocityDevice.z and its valid flag."""
    self.motion.add_device_motion(now, z, valid)

  def add_enabled(self, now: float, enabled: bool) -> None:
    """Every selfdriveState, after the carStates that came before it."""
    self.engaged.add_enabled(now, enabled)

  def due(self, now: float) -> bool:
    if self.schedule:
      return now >= self.next_step
    return now - self.last_step >= self.period

  def step(self, now: float, y: np.ndarray, uv: np.ndarray, lead: tuple[float, float] | None, unix_seconds: float | None,
           country: str | None, v_ego: float | None, sign_distance: float = 0.0,
           position: tuple[float, float] | None = None) -> float:
    """lead: metres and km/h of the car ahead, None without one; unix_seconds: the GPS clock now, None before a fix; country:
    the last one GPS gave, None before the first; v_ego: the car's speed (m/s), None before the first carState;
    sign_distance: vision.SignDistance's metres to the sign ahead, 0 without one; position: latitude and longitude of the
    last valid fix, None before it (the time zone of hour_tz in a country of several)."""
    if now - self.last_step > self.memory_gap:
      self.hidden = np.zeros_like(self.hidden)
    self.last_step = now
    # the next step on the grid of the rate (readings slower than it step at each one); after a pause, a new grid from here
    self.next_step += self.period
    if self.next_step < now:
      self.next_step = now + self.period
    metres, kph = lead if lead is not None else (None, None)
    rows = {"signs": self.signs.features(now), "plate": self.signs.plate_row(), "plate_hours": self.signs.plate_hours_row(),
            "hour": brain_inputs.hour_row(brain_inputs.local_hour(unix_seconds, country)),
            "lead": brain_inputs.lead_row(metres, kph), "country": brain_inputs.country_row(country),
            "pedals": self.pedals.row(now), "speed": brain_inputs.speed_row(v_ego),
            "plate_wet": self.signs.plate_wet_row(), "engaged": self.engaged.row(now),
            "sign_distance": brain_inputs.sign_distance_row(sign_distance), "motion": self.motion.row(now),
            "road_signs": self.signs.road_signs_row(now), "sign_boxes": self.sign_boxes.row(now),
            "light_boxes": self.light_boxes.row(now), "country_iso": brain_inputs.country_iso_row(country),
            "hour_tz": brain_inputs.hour_row(brain_inputs.zone_hour(unix_seconds, country, position), "hour_tz")}
    outputs = self.network(image=self.image(y, uv), extra=brain_inputs.pack_inputs(rows, self.groups)[None], hidden=self.hidden)
    self.hidden = outputs["hidden_out"]
    self.speed_limit = self.kph[int(np.argmax(outputs["speed"]))] / 3.6
    if "road_type" in outputs:
      self.road_type = self.road_types[int(np.argmax(outputs["road_type"]))]
    return self.speed_limit
