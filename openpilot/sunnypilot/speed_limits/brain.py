"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

VISION AND MAP ARE TWO SEPARATE SOURCES: NOTHING HERE READS THE MAP (liveMapDataSP).

The brain: the network with memory that says the speed to hold, once a second. It is the limit of the vision side when the
release has one: no rule of vision.SpeedLimitMemory and no road-type network decides then. What the reader read goes in as
an input (brain_inputs), the number that comes out is published as it is.
"""
import hashlib
import math
from pathlib import Path

import numpy as np

from openpilot.sunnypilot.speed_limits import brain_inputs


class BrainReader:
  """The network step by step: add_signs() on every frame read, step() when due. The picture is the road-type network's,
  bit for bit the frames of the training. speed_limit is the speed said (m/s), held between the steps, 0 before the first.
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
    self.memory_gap = contract["step"]["memory_gap_seconds"]
    self.image = vision.RoadTypeInput(width, height)
    self.signs = brain_inputs.SignInputs(vision)
    self.class_plate = vision.class_plate
    self.plates = {plate for plate, trained in (("time", contract["step"].get("plates", False)),
                                                ("wet", contract["step"].get("wet", False))) if trained}
    self.class_hours = vision.class_hours if contract["step"].get("hours", False) else lambda name: None
    self.hidden = np.zeros(network.input_shape("hidden"), dtype=np.float32)
    self.speed_limit = 0.0
    self.last_step = -math.inf
    self.pedals = brain_inputs.PedalInputs()

  def add_signs(self, now: float, signs: list[dict]) -> None:
    """A frame's signs as vision.read_signs gives them. A plate only to a brain trained with that kind of plate, and their
    hours only to one trained with hours (its contract's step): to another one, trained with them unknown on every frame,
    they would change the speed at random."""
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

  def due(self, now: float) -> bool:
    return now - self.last_step >= self.period

  def step(self, now: float, y: np.ndarray, uv: np.ndarray, lead: tuple[float, float] | None, unix_seconds: float | None,
           country: str | None, v_ego: float | None) -> float:
    """lead: metres and km/h of the car ahead, None without one; unix_seconds: the GPS clock now, None before a fix; country:
    the last one GPS gave, None before the first; v_ego: the car's speed (m/s), None before the first carState."""
    if now - self.last_step > self.memory_gap:
      self.hidden = np.zeros_like(self.hidden)
    self.last_step = now
    metres, kph = lead if lead is not None else (None, None)
    rows = {"signs": self.signs.features(now), "plate": self.signs.plate_row(), "plate_hours": self.signs.plate_hours_row(),
            "hour": brain_inputs.hour_row(brain_inputs.local_hour(unix_seconds, country)),
            "lead": brain_inputs.lead_row(metres, kph), "country": brain_inputs.country_row(country),
            "pedals": self.pedals.row(now), "speed": brain_inputs.speed_row(v_ego),
            "plate_wet": self.signs.plate_wet_row()}
    outputs = self.network(image=self.image(y, uv), extra=brain_inputs.pack_inputs(rows, self.groups)[None], hidden=self.hidden)
    self.hidden = outputs["hidden_out"]
    self.speed_limit = self.kph[int(np.argmax(outputs["speed"]))] / 3.6
    return self.speed_limit
