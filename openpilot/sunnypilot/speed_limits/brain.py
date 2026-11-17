"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

VISION AND MAP ARE TWO SEPARATE SOURCES: NOTHING HERE READS THE MAP (liveMapDataSP), EXCEPT FOR A BRAIN TRAINED WITH THE MAP'S
LIMIT AS AN INPUT (ITS GROUP map_limit, CRISTIAN 14 NOV 2026): add_map_limit() GIVES IT TO THAT BRAIN ONLY.

The brain: the network with memory that says the speed to hold. It is the limit of the vision side when the release has
one: no rule of vision.SpeedLimitMemory and no road-type network decides then. What the reader read goes in as an input
(brain_inputs), the speed that comes out is published as it is. The speed is all the car reads from it: a brain that also
gives the next limit and the metres to it (outputs next and metres, the brains trained until 12 Nov 2026) steps as any
other and those two are not read. The contract's step says how: its rate, its clock ("last": a step once a period has
passed since the last one, the first brains, at 1 Hz; "schedule": steps on a fixed grid of that rate, at the reading that
comes first, as the training's frames are) and its picture ("road_type": vision.RoadTypeInput; "brain":
vision.BrainInput). Its step.other_road_boxes says which signs' boxes it was trained with (sign_boxes): every speed
sign read, those the applicability network gives to another road too, for a brain that learns whose road a sign is of
(the server's applicability_head, a head only the training has for now); without it, only the signs taken. The number
in force (signs) comes from the signs taken for every brain: the applicability network still decides that. Its
step.road_sign_kinds says how many kinds of other sign it was trained with (road_signs): it is given the first so many
of brain_inputs.ROAD_SIGNS, ROAD_SIGN_KINDS when a brain trained with an earlier file does not say.
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
  Nothing that is not a finite number stays in its memory (#104): one would stay there at every later step of the drive,
  with kph[0] as the speed said. A car ahead whose metres or speed are not finite is no car ahead, any other number of
  the row that is not finite goes in as 0, and a step whose memory or speed comes out not finite says no speed (0) and
  no road type, the memory from zeros again at the next one.
  Raises ValueError on a network that was trained with another brain_inputs.py or asks for what the car does not give."""
  def __init__(self, network, contract: dict, vision, width: int, height: int):
    if contract["inputs_sha256"] not in (hashlib.sha256(Path(brain_inputs.__file__).read_bytes()).hexdigest(), *brain_inputs.SAME_GROUPS):
      raise ValueError("the brain was trained with another brain_inputs.py")
    self.groups = tuple(contract["step"]["groups"])
    if not set(self.groups) <= set(brain_inputs.INPUT_SIZES):
      raise ValueError(f"the brain asks for inputs the car does not give: {self.groups}")
    # The kinds of other sign it was trained with: the first so many (input_sizes raises on a number that cannot be)
    kinds = contract["step"].get("road_sign_kinds")
    if kinds is None and "road_signs" in self.groups and contract["inputs_sha256"] not in brain_inputs.SAME_GROUPS:
      raise ValueError("the brain does not say how many kinds of road sign it was trained with")
    self.sizes = brain_inputs.input_sizes(brain_inputs.ROAD_SIGN_KINDS if kinds is None else kinds)
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
    self.sign_slots = brain_inputs.SlotInputs(vision)  # the signs one by one with their numbers (#110)
    self.class_plate = vision.class_plate
    self.plates = {plate for plate, trained in (("time", contract["step"].get("plates", False)),
                                                ("wet", contract["step"].get("wet", False))) if trained}
    self.class_hours = vision.class_hours if contract["step"].get("hours", False) else lambda name: None
    # Trained with the boxes of every sign read, those of another road too: only such a brain gets them (sign_boxes,
    # sign_slots)
    self.other_road_boxes = bool(contract["step"].get("other_road_boxes", False))
    self.hidden = np.zeros(network.input_shape("hidden"), dtype=np.float32)
    self.speed_limit = 0.0
    self.road_type = ""
    self.last_step = -math.inf
    self.next_step = -math.inf
    self.pedals = brain_inputs.PedalInputs()
    self.engaged = brain_inputs.EngagedInputs()
    self.map = brain_inputs.MapInputs()
    self.wants_map = "map_limit" in self.groups  # only such a brain is ever given the map's limit
    self.motion = brain_inputs.MotionInputs()

  def add_signs(self, now: float, signs: list[dict], seen: list[dict] | None = None) -> None:
    """A frame's signs as vision.read_signs gives them, those taken: without the ones the applicability network gave to
    another road. seen: all of them, the ones of another road too (None: there is no such network, signs is all of them).
    A plate only to a brain trained with that kind of plate, and their
    hours only to one trained with hours (its contract's step): to another one, trained with them unknown on every frame,
    they would change the speed at random. Their boxes go as SignNet gave them (sign_boxes, light_boxes), in the pixels of the frame
    it read: the low one, also when the classifier's crops come from the full definition. sign_boxes gets the boxes of
    seen for a brain trained with them (other_road_boxes), of the signs taken for any other, and so does sign_slots;
    the number in force, the
    plates, the other signs confirmed and the lights always come from the signs taken."""
    boxes = [(sign["class"], sign["confidence"], sign["box"]) for sign in signs]
    every = [(sign["class"], sign["confidence"], sign["box"]) for sign in seen] if self.other_road_boxes and seen is not None else boxes
    self.sign_boxes.add(now, every, self.width, self.height)
    self.sign_slots.add(now, every, self.width, self.height)
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

  def add_map_limit(self, now: float, metres_per_second: float, valid: bool) -> None:
    """Every liveMapDataSP (speedLimit, speedLimitValid), for a brain trained with the map's limit (wants_map): any other
    brain takes nothing of it."""
    if self.wants_map:
      self.map.add(now, metres_per_second, valid)

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
    if lead is not None and not (math.isfinite(lead[0]) and math.isfinite(lead[1])):
      lead = None  # a car ahead whose numbers are not finite is no car ahead: 0 would be a car at 0 m (#104)
    metres, kph = lead if lead is not None else (None, None)
    rows = {"signs": self.signs.features(now), "plate": self.signs.plate_row(), "plate_hours": self.signs.plate_hours_row(),
            "hour": brain_inputs.hour_row(brain_inputs.local_hour(unix_seconds, country)),
            "lead": brain_inputs.lead_row(metres, kph), "country": brain_inputs.country_row(country),
            "pedals": self.pedals.row(now), "speed": brain_inputs.speed_row(v_ego),
            "plate_wet": self.signs.plate_wet_row(), "engaged": self.engaged.row(now),
            "sign_distance": brain_inputs.sign_distance_row(sign_distance), "motion": self.motion.row(now),
            "road_signs": self.signs.road_signs_row(now), "sign_boxes": self.sign_boxes.row(now),
            "light_boxes": self.light_boxes.row(now), "country_iso": brain_inputs.country_iso_row(country),
            "hour_tz": brain_inputs.hour_row(brain_inputs.zone_hour(unix_seconds, country, position), "hour_tz"),
            "map_limit": self.map.row(now), "sign_slots": self.sign_slots.row(now)}
    # A number of the row that is not finite (a NaN steering angle or yaw) goes in as 0; the finite ones as they are, bit
    # for bit what the training gave (#104)
    extra = np.nan_to_num(brain_inputs.pack_inputs(rows, self.groups, self.sizes), nan=0.0, posinf=0.0, neginf=0.0)
    outputs = self.network(image=self.image(y, uv), extra=extra[None], hidden=self.hidden)
    if not (np.isfinite(outputs["hidden_out"]).all() and np.isfinite(outputs["speed"]).all()):
      # Not into the memory, where it would stay: this step says no speed, and the next one starts from zeros (#104)
      self.hidden = np.zeros_like(self.hidden)
      self.speed_limit, self.road_type = 0.0, ""
      return self.speed_limit
    self.hidden = outputs["hidden_out"]
    self.speed_limit = self.kph[int(np.argmax(outputs["speed"]))] / 3.6
    if "road_type" in outputs:
      self.road_type = self.road_types[int(np.argmax(outputs["road_type"]))]
    return self.speed_limit
