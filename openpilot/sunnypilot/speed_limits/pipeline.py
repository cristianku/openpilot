"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

VISION AND MAP ARE TWO SEPARATE SOURCES: NOTHING HERE READS THE MAP (liveMapDataSP), EXCEPT map_limit(), WHICH PASSES THE
MAP'S LIMIT TO A BRAIN TRAINED WITH IT AS AN INPUT (brain_inputs' GROUP map_limit, CRISTIAN 14 NOV 2026) AND TO NOTHING ELSE.

The vision limit from a frame of the road camera and the car's own signals to the fields of speedVisionSP: what
speedvisiond does between its sockets. It is here, apart from the sockets, the camera and the device's GPU, so that
the same code runs on the car and over a recorded drive (sunnypilot.speedlimits, nnslr_tools.simulation: the page that
plays a drive with the squares of the comma's screen). speedvisiond gives it what it receives, in the order it
receives it, and sends what frame() returns; a replay gives it the same messages from the drive's log and the frames of
its video. The networks come ready to call: numpy in, numpy out (runner.Network and runner.StepNetwork on the car).

Only numpy and the modules of this package that need nothing else: no socket, no Params, no tinygrad.
"""
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from openpilot.sunnypilot.speed_limits import applicability, vision
from openpilot.sunnypilot.speed_limits.country import CountryResolver, GpsFix, OfflineBorders

MANIFEST = "speed_limits.json"
MANIFEST_FORMAT = 2
PERIOD = 0.2  # seconds between the frames read
DEFAULT_COUNTRY = "CH"  # until GPS gives one, without a country chosen in Settings › OSM


def classifier_country_codes(contract: dict) -> tuple[str, ...]:
  if "country_input" not in contract:
    return ()
  field = contract["country_input"]
  if not isinstance(field, dict) or not isinstance(field.get("codes"), list):
    raise ValueError("Invalid classifier country input")
  codes = tuple(field["codes"])
  if (not codes or codes[0] != "unknown" or any(
      not isinstance(code, str) or len(code) != 2 or not code.isascii() or not code.isalpha() or not code.isupper()
      for code in codes[1:]) or len(set(codes)) != len(codes) or
      (field.get("name"), field.get("encoding"), field.get("dtype"), field.get("shape")) !=
      ("country", "one_hot", "float32", [1, len(codes)])):
    raise ValueError("Invalid classifier country input")
  return codes


class Networks:
  """The files of a release in its directory: SignNet (finder), the classifier and its classes, the name of the release;
  the road type network and its classes when the release has one (road_type, optional in format 2); the brain and its
  contract when the release has one (brain, optional in format 2: it is the limit then); the applicability network and
  its contract when the release has one (decider, optional in format 2: the signs of another road are not taken); the
  settings of the vision limit (settings, optional: vision.apply_settings)."""
  def __init__(self, manifest: dict, directory: Path):
    self.name = f"speed-limits {manifest['version']}"
    self.settings: dict = manifest.get("settings") if isinstance(manifest.get("settings"), dict) else {}
    self.finder = directory / manifest["finder"]["file"]
    self.classifier = directory / manifest["classifier"]["file"]
    contract = json.loads((directory / manifest["classifier"]["contract"]).read_text())
    self.classes: list[str] = contract["output"]["classes"]
    self.classifier_countries = classifier_country_codes(contract)
    self.road_type: Path | None = None
    self.road_types: list[str] = []
    if "road_type" in manifest:
      self.road_type = directory / manifest["road_type"]["file"]
      self.road_types = json.loads((directory / manifest["road_type"]["contract"]).read_text())["output"]["classes"]
    self.brain: Path | None = None
    self.brain_contract: dict = {}
    if "brain" in manifest:
      self.brain = directory / manifest["brain"]["file"]
      self.brain_contract = json.loads((directory / manifest["brain"]["contract"]).read_text())
    self.decider: Path | None = None
    self.decider_contract: dict = {}
    if "decider" in manifest:
      self.decider = directory / manifest["decider"]["file"]
      self.decider_contract = json.loads((directory / manifest["decider"]["contract"]).read_text())

  def onnx(self) -> list[Path]:
    """Every network of the release, the road type's, the brain's and the applicability's too."""
    return [self.finder, self.classifier] + [path for path in (self.road_type, self.brain, self.decider) if path]


def release(directory: Path) -> Networks | None:
  """The networks of a release's directory, None when it has none (an older format, no model yet)."""
  try:
    manifest = json.loads((directory / MANIFEST).read_text())
    return Networks(manifest, directory) if manifest.get("format") == MANIFEST_FORMAT else None
  except (OSError, ValueError, KeyError, TypeError):
    return None


def classifier_for_country(classifier, codes: tuple[str, ...], country: str | None):
  """Bind the classifier's optional input to one GPS country for every crop of the frame."""
  if not codes:
    return classifier
  encoding = np.zeros((1, len(codes)), dtype=np.float32)
  code = country.strip().upper() if isinstance(country, str) else None
  encoding[0, codes.index(code) if code in codes else 0] = 1
  return lambda images: classifier(images=images, country=encoding)["logits"]


class SpeedVision:
  """speedvisiond's state and what it does with every message and every frame.

  files: the release (Networks), its settings already in place (vision.apply_settings). width, height: the road
  camera's frames; focal_length: its focal length in pixels for them, 0 for a camera not known (no distance to a sign).
  finder, classifier: SignNet and the classifier, the network's input in and its output out. reader: the brain
  (brain.BrainReader) when the release has one: it is the limit then, and neither the memory's rules nor the road-type
  network run. road_types: the road-type network (vision.RoadTypeReader), without a brain only. ruler: the applicability
  network (applicability.Applicability), None for a release without it: every sign is taken. With it a sign of another
  road is left out of the limit and of the brain's number in force; its box still goes to a brain trained with the boxes
  of every sign read (BrainReader.add_signs). settings_country: the
  country until GPS gives one. country: None without borders or a country policy in the release's settings.

  Every time is in seconds on one clock, the one of the frames: the car's messages take the time of the frame they
  came before (car_state also the message's own, for the metres driven), as speedvisiond drains its sockets at each
  frame of the camera."""
  def __init__(self, files: Networks, width: int, height: int, focal_length: float, finder, classifier, *,
               reader=None, road_types=None, ruler=None, settings_country: str = DEFAULT_COUNTRY):
    self.files = files
    self.finder, self.classifier = finder, classifier
    self.reader = reader
    self.road_types = road_types if reader is None else None
    self.ruler = ruler
    self.signnet_input = vision.SignNetInput(width, height)
    # GPS selects only a country and sign unit. No map speed/road/junction data enters vision.
    self.settings_country = settings_country
    self.gps_country: str | None = None
    self.gps_position: tuple[float, float] | None = None  # of the last valid fix: the brain's time zone (hour_tz)
    self.memory = vision.SpeedLimitMemory(settings_country)
    borders = OfflineBorders.load()
    self.country = CountryResolver.from_settings(borders, files.settings.get("country_policy", {}))
    self.classifier_country = (CountryResolver.from_settings(borders, files.settings.get("classifier_country_policy", {}))
                               if self.country is None and getattr(files, "classifier_countries", ()) else None)
    self.memory.garage_known = self.road_types is not None and "garage" in self.road_types.classes
    # The road-type network's: its step alone drops the old samples, so it is filled only for one (#104)
    self.motion = vision.CarMotion()
    self.distance = vision.SignDistance(focal_length)
    self.limit_ahead = vision.LimitAhead()
    self.v_ego: float | None = None
    self.lead: tuple[float, float] | None = None  # metres and km/h of the car ahead
    self.gps_clock: tuple[float, float] | None = None  # the GPS clock of the last fix (s) and when it came
    self.next_frame = 0.0
    self.signs: list[dict] = []  # the signs of the last frame read, all of them (read_signs, ruled when there is a ruler)
    self._fix: GpsFix | None = None  # the last one since the last frame read
    self._fix_unix: float | None = None  # its GPS clock (s), when it is a fix with one

  def car_state(self, now: float, log_time: float, v_ego: float, steering_deg: float, left: bool, right: bool,
                yaw_rate: float, gas: bool, brake: bool, standstill: bool | None = None) -> None:
    """Every carState. log_time: the message's own time (logMonoTime, s), for the metres driven. standstill:
    carState.standstill, None from a caller that does not give it: the brain's standstill group stays unknown then,
    never "moving" (#121)."""
    self.v_ego = v_ego
    self.distance.add_speed(log_time, v_ego)
    if self.road_types is not None:
      self.motion.add_car_state(now, steering_deg, left, right, yaw_rate)
    if self.reader is not None:
      self.reader.add_pedals(now, gas, brake)
      self.reader.add_car_state(now, steering_deg, left, right, yaw_rate)
      if standstill is not None:
        self.reader.add_standstill(now, standstill)

  def enabled(self, now: float, enabled: bool) -> None:
    """Every selfdriveState, after the carStates that came before it: whether openpilot is engaged."""
    if self.reader is not None:
      self.reader.add_enabled(now, enabled)

  def map_limit(self, now: float, metres_per_second: float, valid: bool) -> None:
    """Every liveMapDataSP: its speedLimit and speedLimitValid, for a brain trained with the map's limit as an input
    (BrainReader.add_map_limit: any other brain takes nothing of it). No rule and no other network reads it."""
    if self.reader is not None:
      self.reader.add_map_limit(now, metres_per_second, valid)

  def device_motion(self, now: float, z: float, valid: bool) -> None:
    """Every deviceMotion: angularVelocityDevice.z and its valid flag."""
    if self.road_types is not None:
      self.motion.add_device_motion(now, z, valid)
    if self.reader is not None:
      self.reader.add_device_motion(now, z, valid)

  def radar(self, present: bool, metres: float, v_lead: float) -> None:
    """The last radarState: its leadOne (present, dRel, vLead in m/s)."""
    self.lead = (metres, v_lead * 3.6) if present else None

  def due(self, now: float) -> bool:
    """Whether the frame that came now is read: PERIOD after the last one read."""
    return now >= self.next_frame

  def gps(self, fix: GpsFix, unix_millis: int) -> None:
    """The last GPS message since the last frame read, given before frame(): its fix and its clock (unixTimestampMillis)."""
    self._fix = fix
    self._fix_unix = unix_millis / 1000 if fix.valid and unix_millis else None

  def frame(self, now: float, y: np.ndarray, uv: np.ndarray, full: tuple[np.ndarray, np.ndarray] | None = None) -> dict:
    """A frame of the road camera that is due, its planes (vision.nv12_planes); full: the same frame's planes before the
    scaling, where the classifier's crops come from, None without them. The fields of speedVisionSP to send, by name."""
    self.next_frame = now + PERIOD
    fix, self._fix = self._fix, None
    if fix is not None and fix.valid:
      self.gps_position = fix.latitude, fix.longitude
    if self._fix_unix is not None:
      self.gps_clock = self._fix_unix, now
      self._fix_unix = None
    resolved_country = self.country.update(now, fix) if self.country is not None else None
    self.gps_country = resolved_country or self.gps_country
    memory, distance, reader = self.memory, self.distance, self.reader
    memory.set_country(self.gps_country or self.settings_country)

    classifier_country = self.classifier_country.update(now, fix) if self.classifier_country is not None else resolved_country
    classifier = classifier_for_country(self.classifier, getattr(self.files, "classifier_countries", ()), classifier_country)
    signs = vision.read_signs(y, uv, self.signnet_input, self.finder, classifier, self.files.classes, full)
    raw = vision.raw_reading(signs)  # of all the signs: the reading square shows a sign of another road too
    seen = self.signs = signs
    if self.ruler is not None:
      # Right after the classifier, while the GPU is still free of the driving model.
      self.ruler.rule(y, uv, signs, self.v_ego)
      signs = applicability.taken(signs)
    out = SimpleNamespace()
    if reader is not None:
      # The signs taken, and all of them: a brain trained with the boxes of every sign read takes those from seen
      reader.add_signs(now, signs, seen)
      sign_distance = distance.update(now, signs, None)
      if reader.due(now):
        # Right after SignNet, while the GPU is still free of the driving model.
        unix = self.gps_clock[0] + now - self.gps_clock[1] if self.gps_clock is not None else None
        reader.step(now, y, uv, self.lead, unix, self.gps_country, self.v_ego, sign_distance, self.gps_position)
      if raw is not None:
        memory.note_raw(raw)
        # Given to the brain: read sure enough and not of another road.
        memory.raw_taken = raw["confidence"] >= vision.MIN_CONFIDENCE and memory.raw_road != vision.OTHER_ROAD
      out.speedLimit = float(reader.speed_limit)
      out.signRead = reader.signs.read_at == now
      memory.fill_log(out)  # for the logs: the last sign read
      out.speedLimitState = "limit" if reader.speed_limit > 0 else "unknown"
      out.speedLimitReason = "brain"
      # What a 0 of speedLimit is: "valid" is a brain trained to stand saying "stand" (#117 step B). Shadow mode: the
      # resolver logs it, nothing on the car acts on it yet.
      out.brainStatus = reader.status(now)
      if reader.road_type:  # shown on the screen only; a brain exported before 12 Nov 2026 has none
        out.roadType = reader.road_type
    else:
      road_types = self.road_types
      if road_types is not None and road_types.due(now):
        # Right after SignNet, while the GPU is still free of the driving model.
        road_types.step(now, y, uv, self.v_ego, memory.road_sign_age(now), self.motion.features(now),
                        memory.board_age(now))
      road_type = road_types.road_type if road_types is not None else None
      speed_limit = memory.update_frame(now, signs, road_type=road_type, v_ego=self.v_ego, seen=seen)
      sign_distance = distance.update(now, signs, memory.road)
      speed_limit = self.limit_ahead.update(speed_limit, memory.limit_reason, sign_distance, distance.sign_class,
                                            distance.odometer)
      out.speedLimit = float(speed_limit)
      out.speedLimitAhead = float(self.limit_ahead.ahead)
      out.speedLimitAheadDistance = float(self.limit_ahead.distance)
      out.signRead = memory.sign_read
      out.roadType = road_type or ""
      memory.fill_log(out)  # for the logs: why the limit is what it is, the last sign read taken or not
      out.brainStatus = "legacy"  # the rules: a 0 is "not sure"
    out.modelName = self.files.name
    out.signDistance = sign_distance
    out.signSize = distance.side
    out.rawSignSeen = raw is not None
    return vars(out)
