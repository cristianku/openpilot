"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

pipeline.SpeedVision: what speedvisiond does between its sockets, and what a replay of a recorded drive runs. The
daemon's own behaviour around it is in international/test_daemon.py.
"""
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from openpilot.sunnypilot.speed_limits import brain, brain_inputs, pipeline, vision
from openpilot.sunnypilot.speed_limits.country import GpsFix
from openpilot.sunnypilot.speed_limits.tests.test_speed_vision_sp import FIELDS

WIDTH, HEIGHT = 1344, 760
Y, UV = np.zeros((HEIGHT, WIDTH), dtype=np.uint8), np.zeros((HEIGHT // 2, WIDTH), dtype=np.uint8)


def sign(name="maximum_speed:50", confidence=0.95, box=(600, 200, 640, 240)):
  return {"class": name, "box": list(box), "confidence": confidence, "speed": vision.class_speed(name)}


def files(**settings):
  return NS(name="synthetic", settings=settings, classes=["maximum_speed:50"])


@pytest.fixture
def frames(monkeypatch):
  """The signs each frame read gives, in order: what vision.read_signs returns."""
  given = []
  monkeypatch.setattr(vision, "read_signs", lambda *args: given.pop(0) if given else [])
  return given


class Brain:
  """A brain.BrainReader saying 80 km/h from its first step, with what it was given."""
  def __init__(self):
    self.speed_limit = 0.0
    self.road_type = ""
    self.signs = NS(read_at=None)
    self.given, self.seen, self.steps, self.enabled, self.map = [], [], [], [], []

  def add_signs(self, now, signs, seen=None):
    self.given.append(signs)
    self.seen.append(seen)
    if signs:
      self.signs.read_at = now

  def add_pedals(self, now, gas, brake):
    pass

  def add_car_state(self, now, steering_deg, left, right, yaw_rate):
    pass

  def add_device_motion(self, now, z, valid):
    pass

  def add_enabled(self, now, enabled):
    self.enabled.append(enabled)

  def add_map_limit(self, now, metres_per_second, valid):
    self.map.append((now, metres_per_second, valid))

  def due(self, now):
    return True

  def step(self, now, y, uv, lead, unix_seconds, country, v_ego, sign_distance, position=None):
    self.steps.append({"lead": lead, "unix": unix_seconds, "country": country, "v_ego": v_ego})
    self.speed_limit = 80 / 3.6


def test_a_release_is_read_from_its_directory(tmp_path):
  manifest = {"format": 2, "version": "2026-11-10", "finder": {"file": "signnet.onnx", "contract": "signnet.json"},
              "classifier": {"file": "classifier.onnx", "contract": "classifier.json"},
              "decider": {"file": "decider.onnx", "contract": "decider.json"}, "settings": {"min_confidence": 0.8}}
  (tmp_path / "speed_limits.json").write_text(json.dumps(manifest))
  (tmp_path / "classifier.json").write_text(json.dumps({"output": {"classes": ["maximum_speed:50", "cancellation"]}}))
  (tmp_path / "decider.json").write_text(json.dumps({"task": "decider"}))
  found = pipeline.release(tmp_path)
  assert found.name == "speed-limits 2026-11-10" and found.classes == ["maximum_speed:50", "cancellation"]
  assert (found.finder, found.classifier, found.decider) == tuple(tmp_path / name for name in
                                                                 ("signnet.onnx", "classifier.onnx", "decider.onnx"))
  assert found.brain is None and found.road_type is None and found.decider_contract == {"task": "decider"}
  assert found.onnx() == [tmp_path / "signnet.onnx", tmp_path / "classifier.onnx", tmp_path / "decider.onnx"]
  assert found.settings == {"min_confidence": 0.8}


def test_no_release_in_an_older_format_or_an_empty_directory(tmp_path):
  assert pipeline.release(tmp_path) is None
  (tmp_path / "speed_limits.json").write_text(json.dumps({"format": 1}))
  assert pipeline.release(tmp_path) is None


def test_the_maps_limit_goes_to_the_brain_and_to_nothing_else(frames):
  """Only a brain trained with the map's limit uses it (BrainReader.add_map_limit); the rules never see it."""
  frames.extend([[sign()], [sign()]])
  brain = Brain()
  with_brain = pipeline.SpeedVision(files(), WIDTH, HEIGHT, 1141.5, None, None, reader=brain)
  with_brain.map_limit(1.0, 80 / 3.6, True)
  assert brain.map == [(1.0, 80 / 3.6, True)]
  rules = pipeline.SpeedVision(files(), WIDTH, HEIGHT, 1141.5, None, None)
  rules.map_limit(1.0, 80 / 3.6, True)
  rules.frame(1.0, Y, UV)
  assert rules.frame(1.2, Y, UV)["speedLimit"] == pytest.approx(50 / 3.6)


def test_a_frame_is_read_a_period_after_the_last_one(frames):
  speed_vision = pipeline.SpeedVision(files(), WIDTH, HEIGHT, 1141.5, None, None)
  assert speed_vision.due(10.0)
  speed_vision.frame(10.0, Y, UV)
  assert not speed_vision.due(10.0 + pipeline.PERIOD - 0.01)
  assert speed_vision.due(10.0 + pipeline.PERIOD)


def test_the_message_has_only_fields_of_speed_vision_sp(frames):
  frames.extend([[sign()], [sign()]])
  memory = pipeline.SpeedVision(files(), WIDTH, HEIGHT, 1141.5, None, None)
  memory.frame(1.0, Y, UV)
  sent = memory.frame(1.2, Y, UV)
  assert set(sent) <= set(FIELDS)
  assert sent["speedLimit"] == pytest.approx(50 / 3.6) and sent["signRead"] and sent["rawSignSeen"]
  assert (sent["modelName"], sent["rawSignClass"], sent["speedLimitReason"]) == ("synthetic", "maximum_speed:50", "sign")
  frames.append([sign()])
  with_brain = pipeline.SpeedVision(files(), WIDTH, HEIGHT, 1141.5, None, None, reader=Brain())
  sent = with_brain.frame(1.0, Y, UV)
  assert set(sent) <= set(FIELDS)
  assert sent["speedLimit"] == pytest.approx(80 / 3.6) and sent["signRead"]
  assert (sent["speedLimitState"], sent["speedLimitReason"], sent["rawSignTaken"]) == ("limit", "brain", True)
  assert "speedLimitAhead" not in sent and "speedLimitAheadDistance" not in sent and "roadType" not in sent


def test_the_limit_ahead_is_the_rules_and_never_a_brains(frames):
  # A 50 held, then a 30 on a sign ahead: the rules hold the 50 up to the sign and say the 30 with the metres to it
  ahead = [sign("maximum_speed:30", box=(600, 200, 620 + n, 220 + n)) for n in range(3)]
  frames.extend([[sign()], [sign()], *([one] for one in ahead)])
  rules = pipeline.SpeedVision(files(), WIDTH, HEIGHT, 1141.5, None, None)
  sent = [rules.frame(1.0 + n * pipeline.PERIOD, Y, UV) for n in range(5)][-1]
  assert sent["speedLimit"] == pytest.approx(50 / 3.6) and sent["speedLimitAhead"] == pytest.approx(30 / 3.6)
  assert sent["speedLimitAheadDistance"] == pytest.approx(sent["signDistance"]) and sent["signDistance"] > 0
  # The same road with a brain: its speed alone, and the distance to the sign for the screen's reading square
  frames.extend([[sign()], [sign()], *([one] for one in ahead)])
  with_brain = pipeline.SpeedVision(files(), WIDTH, HEIGHT, 1141.5, None, None, reader=Brain())
  sent = [with_brain.frame(1.0 + n * pipeline.PERIOD, Y, UV) for n in range(5)][-1]
  assert sent["speedLimit"] == pytest.approx(80 / 3.6) and sent["signDistance"] > 0
  assert "speedLimitAhead" not in sent and "speedLimitAheadDistance" not in sent


def test_the_brain_gets_the_cars_signals_at_its_step(frames):
  brain = Brain()
  speed_vision = pipeline.SpeedVision(files(), WIDTH, HEIGHT, 1141.5, None, None, reader=brain)
  speed_vision.car_state(5.0, 4.99, 20.0, 0.0, False, False, 0.0, False, False)
  speed_vision.enabled(5.0, True)
  speed_vision.radar(True, 30.0, 10.0)
  speed_vision.gps(GpsFix(4.9, 47.0, 8.0, 5.0, True), 1_770_000_000_000)
  speed_vision.frame(5.0, Y, UV)
  speed_vision.radar(False, 0.0, 0.0)
  speed_vision.frame(6.0, Y, UV)
  assert brain.enabled == [True]
  assert brain.steps[0] == {"lead": (30.0, 36.0), "unix": pytest.approx(1_770_000_000), "country": None, "v_ego": 20.0}
  assert brain.steps[1]["lead"] is None and brain.steps[1]["unix"] == pytest.approx(1_770_000_001)  # the clock runs on


def test_only_the_last_gps_message_before_a_frame_counts(frames):
  brain = Brain()
  speed_vision = pipeline.SpeedVision(files(), WIDTH, HEIGHT, 1141.5, None, None, reader=brain)
  speed_vision.gps(GpsFix(0.8, 47.0, 8.0, 5.0, True), 1_770_000_000_000)
  speed_vision.gps(GpsFix(0.9, 47.0, 8.0, 5.0, False), 1_770_000_000_100)  # the fix lost: speedvisiond's socket keeps this one
  speed_vision.frame(1.0, Y, UV)
  assert brain.steps[0]["unix"] is None


def test_a_sign_of_another_road_shows_and_is_not_taken(frames):
  class SideRoad:
    def rule(self, y, uv, signs, v_ego):
      for one in signs:
        one["road"] = "other_road"

  frames.extend([[sign()], [sign()]])
  brain = Brain()
  speed_vision = pipeline.SpeedVision(files(), WIDTH, HEIGHT, 1141.5, None, None, reader=brain, ruler=SideRoad())
  speed_vision.frame(1.0, Y, UV)
  sent = speed_vision.frame(1.2, Y, UV)
  assert brain.given == [[], []]
  # Every sign read reaches the reader beside the ones taken: a brain trained with the boxes of all of them uses these
  assert [[one["road"] for one in seen] for seen in brain.seen] == [["other_road"], ["other_road"]]
  assert (sent["rawSignClass"], sent["rawSignRoad"], sent["rawSignTaken"], sent["rawSignSeen"]) == \
         ("maximum_speed:50", "other_road", False, True)
  assert [one["road"] for one in speed_vision.signs] == ["other_road"]  # a replay draws its box too


def test_without_an_applicability_network_the_signs_taken_are_all_of_them(frames):
  frames.extend([[sign()]])
  brain = Brain()
  pipeline.SpeedVision(files(), WIDTH, HEIGHT, 1141.5, None, None, reader=brain).frame(1.0, Y, UV)
  assert brain.given == brain.seen == [[sign()]]


def test_the_road_type_network_only_without_a_brain(frames):
  road_types = NS(classes=["town", "garage"], due=lambda now: False, road_type="town")
  assert pipeline.SpeedVision(files(), WIDTH, HEIGHT, 0.0, None, None, reader=Brain(), road_types=road_types).road_types is None
  alone = pipeline.SpeedVision(files(), WIDTH, HEIGHT, 0.0, None, None, road_types=road_types)
  assert alone.memory.garage_known and alone.frame(1.0, Y, UV)["roadType"] == "town"


def test_the_road_type_networks_motion_is_kept_only_for_it(frames):
  """(#104) vision.CarMotion drops its old samples only when the road-type network steps. With a brain, or with neither,
  nothing reads it and it grew for the whole drive, about 57 MB an hour: it is filled only for that network."""
  def a_second(speed_vision):
    for n in range(100):  # of carState, and a deviceMotion at every fifth
      at = 1.0 + n / 100
      speed_vision.car_state(at, at, 20.0, 90.0, True, False, 0.0, False, False)
      if n % 5 == 0:
        speed_vision.device_motion(at, -np.radians(15.0), True)
    return speed_vision.motion

  reader, got = Brain(), []
  reader.add_car_state = reader.add_device_motion = lambda *given: got.append(given)
  for motion in (a_second(pipeline.SpeedVision(files(), WIDTH, HEIGHT, 0.0, None, None, reader=reader)),
                 a_second(pipeline.SpeedVision(files(), WIDTH, HEIGHT, 0.0, None, None))):
    assert motion.car == [] and motion.yaw == []
  assert len(got) == 120  # the brain still gets them all: its own brain_inputs.MotionInputs keeps the last ones only
  # With a road-type network, what its step is given is what it was, and the step drops the old samples
  given = []
  road_types = NS(classes=["town"], due=lambda now: True, road_type="town",
                  step=lambda now, y, uv, v_ego, sign_age, motion, board_age: given.append(motion))
  alone = pipeline.SpeedVision(files(), WIDTH, HEIGHT, 0.0, None, None, road_types=road_types)
  motion = a_second(alone)
  assert (len(motion.car), len(motion.yaw)) == (100, 20)
  alone.frame(2.0, Y, UV)
  assert np.allclose(given[0], [[0.5, 0.5, 1, 0.5, 1, 1, 0, 1]])
  alone.frame(2.0 + vision.BLINKER_SECONDS, Y, UV)
  assert motion.car == [] and motion.yaw == [] and not given[1].any()


class Memory:
  """A brain's network whose memory keeps what it is given, as a GRU's does: 80 km/h while it holds numbers, and kph[0],
  1 km/h, once it holds one that is not finite (np.argmax of NaNs). Its step `overflow` comes out infinite."""
  def __init__(self, overflow=None):
    self.steps, self.overflow = 0, overflow

  def input_shape(self, name):
    return (1, 4)

  def __call__(self, image, extra, hidden):
    memory = hidden + extra.sum() + (np.float32(np.inf) if self.steps == self.overflow else 0)
    self.steps += 1
    speed = np.array([[0.0, 1.0]], dtype=np.float32)
    return {"speed": speed if np.isfinite(memory).all() else speed * np.nan, "hidden_out": memory}


def a_brain(network):
  """brain.BrainReader as speedvisiond makes it: the car ahead and the car's motion beside the picture, 4 steps a second."""
  step = {"rate_hz": 4.0, "groups": ["lead", "motion"], "memory_gap_seconds": 5.0, "clock": "schedule", "picture": "brain"}
  contract = {"inputs_sha256": hashlib.sha256(Path(brain_inputs.__file__).read_bytes()).hexdigest(),
              "output": {"kph": [1, 80]}, "step": step}
  return brain.BrainReader(network, contract, vision, WIDTH, HEIGHT)


def said(speed_vision, steps):
  """The limit sent at each of so many steps of the brain, a quarter of a second apart."""
  sent = [speed_vision.frame(1.0 + n * 0.25, Y, UV) for n in range(steps)]
  return [(round(one["speedLimit"] * 3.6), one["speedLimitState"], one["speedLimitReason"]) for one in sent]


def test_one_number_that_is_not_finite_does_not_stay_in_the_brains_memory(frames):
  """(#104) The memory starts again only after 5 s without a step, so one NaN in it was there for the rest of the drive,
  and the car sent 1 km/h as the limit. A NaN from the radar goes in as 0; a step that comes out not finite is sent as
  no limit (0, "unknown"), and the next one starts from zeros."""
  speed_vision = pipeline.SpeedVision(files(), WIDTH, HEIGHT, 1141.5, None, None, reader=a_brain(Memory()))
  speed_vision.radar(True, float("nan"), 10.0)  # leadOne's dRel
  assert said(speed_vision, 3) == [(80, "limit", "brain")] * 3 and np.isfinite(speed_vision.reader.hidden).all()
  speed_vision = pipeline.SpeedVision(files(), WIDTH, HEIGHT, 1141.5, None, None, reader=a_brain(Memory(overflow=1)))
  speed_vision.radar(True, 30.0, 10.0)
  assert said(speed_vision, 3) == [(80, "limit", "brain"), (0, "unknown", "brain"), (80, "limit", "brain")]
