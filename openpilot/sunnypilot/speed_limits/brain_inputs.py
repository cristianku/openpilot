"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

VISION AND MAP ARE TWO SEPARATE SOURCES: NOTHING HERE READS THE MAP (liveMapDataSP). THE BRAIN'S INPUTS COME FROM THE CAMERA
AND THE CAR'S OWN SIGNALS ONLY; THE COUNTRY IS THE ONLY THING GPS GIVES, AS IT DOES TO THE SIGNS' UNIT.

What the brain is given beside the picture, frame by frame, all from what comes at or before the frame. One file for the
car and for the training: the server loads this very file by its path (numpy only, nothing of openpilot), so that what the
network learned on is what it gets on the car, and the checkpoint keeps the hash of the file it was trained with.

The groups, each one apart so that a comparison can leave it out:

- signs: the number confirmed as the limit confirms it (vision.SpeedLimitMemory's own rule), what kind of sign gave it, its
  age, whether an end was confirmed since, whether it was read in the last second; and the last single reading, since one
  frame is all a far or covered sign may give. No limit is decided here: the network weighs them;
- plate: the panel below the number confirmed: hours when the classifier read a number with hours, else unknown;
- plate_hours: the hours written on that plate, from and to, when the classifier read them, else unknown;
- plate_wet: that plate says the number holds only on a wet road, when the classifier read so. A plate of its own group and
  not one more of PLATES, so that the plate group of a published brain stays as it is. Whether the road is wet the network
  sees in the picture: no code turns a plate and a wet road into a limit;
- hour: the local hour, on a circle, so that a limit of some hours can be told from the rest. The network decides, no code
  turns a plate and an hour into a limit;
- lead: distance and speed of the car ahead;
- country: the last one GPS gave, none before the first;
- pedals: gas and brake pressed at some moment of the last second, as carState says them;
- speed: the car's own speed.
- engaged: whether openpilot is engaged (selfdriveState.enabled), how long ago it last went off and what turned it off. A
  driver who takes over where the limit said is wrong often brakes once and then lets the car slow down by itself, with no
  pedal pressed: the pedals of the last second no longer show it, openpilot off still does.
- sign_distance: the metres to the round sign ahead as vision.SignDistance measures them from its box and the car's speed
  (the same class on the car and on the server), known; nothing once it is passed. With it the brain can say where its
  next limit starts.
- motion: the car's own turn signals, steering wheel and yaw (MotionInputs): a blinker on in the last BLINKER_SECONDS and
  how long ago it was last on, left and right; the steering angle; the yaw rate (deviceMotion, the car's yawRate when the
  device gives none); each known. A turn signal before an exit or a junction says the limit is about to change.
"""
from datetime import datetime, timedelta, timezone
import math

import numpy as np

COUNTRIES = ("CH", "DE", "FR", "IT", "AT", "US")
PLATES = ("none", "time", "other")  # below a number: none, hours written on it, anything else
SIGN_KPH_SCALE = 130.0
SIGN_AGE_SCALE = math.log1p(3600.0)
LEAD_METRES_SCALE = 100.0
LEAD_METRES_MAX = 200.0
SIGN_METRES_SCALE = 100.0
SIGN_METRES_MAX = 200.0
SPEED_MAX = 60.0  # m/s
SPEED_SCALE = 30.0
PEDAL_SECONDS = 1.0
BLINKER_SECONDS = 1.0  # a blinker lamp flashes: on at some moment of the last second is on
BLINKER_AGE_MAX = 60.0  # s: longer ago, or never, is 1
STEER_SCALE, STEER_MAX = 180.0, 4.0  # degrees of the steering wheel, clipped at +-STEER_MAX after the scale
YAW_SCALE, YAW_MAX = 30.0, 3.0  # deg/s, left positive
MOTION_SECONDS = 1.0  # a signal not heard for longer is unknown
CAR_YAW_MIN = 1e-4  # rad/s: a car whose yawRate stays 0 has no sensor behind it
MEMORY_GAP_SECONDS = 5.0  # longer without a frame: its memory starts again from zeros
# The hours from UTC in winter, of the countries with the EU's summer time. Arithmetic on UTC: a comma may have no time zones.
WINTER_UTC_HOURS = {"CH": 1, "DE": 1, "FR": 1, "IT": 1, "AT": 1}
# The earlier files of this one a published brain may have been trained with: since each of them groups were only added, so
# that brain's own groups still give the same numbers. Without this a new group would leave the car with no vision limit at
# all until a new brain is published. A change to what a group gives takes the hashes out of here.
SAME_GROUPS = ("29e3c8b0717884ee545da22ee27520c6a3366f11d4ac28d2028e96a016dcf467",
               "1147b4ecbc7b9d9334ee29af5afb74c24a49c3aa2bc6f6eeb299754ce11f4244",
               "4b982cdaf22df69a9030229b6acfa7dee6bbab4c6a04857b19aadbb78d4fc4a7",
               "3406d6ab17f201aac62e37521393f42d1cfd2d319b6f89a21d59ee38ad056aa0",
               "c2303311c6b92f362273754c88fe87321ac0a3ac71967c259c9ad1161be1366d")
INPUT_SIZES = {"signs": 11, "plate": len(PLATES) + 1, "plate_hours": 5, "hour": 3, "lead": 3, "country": len(COUNTRIES) + 1,
               "pedals": 3, "speed": 2, "plate_wet": 1, "engaged": 7, "sign_distance": 2,
               "motion": 8}


class SignInputs:
  """The signs, plate, plate_hours and plate_wet groups from the signs of a drive: add() each frame's in order, then features(),
  plate_row(), plate_hours_row() and plate_wet_row() at a moment after them. On the car add() comes right after read_signs."""

  def __init__(self, vision):
    self._vision = vision
    self._memory = vision.SpeedLimitMemory()  # only its rule of what is confirmed
    self._plates: dict = {}  # number: (when, plate) of its last reading with a plate
    self._hours: dict = {}  # number: (when, (from, to)) of its last reading with hours: one without them takes nothing away
    self.number, self.kind, self.read_at, self.ended, self.plate, self.hours = None, "", None, False, None, None
    self.single, self.single_at = None, None

  def add(self, now: float, signs: list) -> None:
    """signs: (class, confidence), (class, confidence, plate) or (class, confidence, plate, (from, to)) of a frame. Of a
    number read twice in it, the surest."""
    read: dict = {}
    for name, confidence, *said in signs:
      value = self._vision.class_speed(name)
      if value is not None and confidence >= self._vision.MIN_CONFIDENCE and (confidence, name) > read.get(value, (0.0, ""))[:2]:
        read[value] = (confidence, name, *said, None, None)[:4]
    if not read:
      return
    self.single, self.single_at = max(read, key=lambda value: read[value][:2]), now
    self._plates.update({value: (now, item[2]) for value, item in read.items() if item[2] is not None})
    self._hours.update({value: (now, item[3]) for value, item in read.items() if item[3] is not None})
    confirmed = self._memory._confirmed(now, list(read))
    if confirmed and confirmed[0] == 0.0:
      self.ended = True
    elif confirmed:
      if confirmed[0] != self.number or self.ended:
        self.plate = self.hours = None  # another sign
      self.number, self.kind, self.ended = confirmed[0], read[confirmed[0]][1].partition(":")[0], False
    if self.number in read:
      self.read_at = now
    when, plate = self._plates.get(self.number, (-math.inf, None))
    if now - when <= self._vision.CONFIRM_SECONDS:
      self.plate = plate
    when, hours = self._hours.get(self.number, (-math.inf, None))
    if now - when <= self._vision.CONFIRM_SECONDS:
      self.hours = hours

  def features(self, now: float) -> np.ndarray:
    """The number confirmed / SIGN_KPH_SCALE, known, from a display, a zone, a marking on the road, the age of its last
    reading, an end confirmed since, read in the last second; the last single reading / SIGN_KPH_SCALE (0 for an end), known,
    its age."""
    row = np.zeros(INPUT_SIZES["signs"], dtype=np.float32)
    if self.number is not None:
      age = now - self.read_at
      row[:8] = (self.number / SIGN_KPH_SCALE, 1.0, self.kind == "variable_display", self.kind == "zone",
                 self.kind == "road_marking", math.log1p(age) / SIGN_AGE_SCALE, self.ended, age < 1)
    if self.single is not None:
      row[8:] = self.single / SIGN_KPH_SCALE, 1.0, math.log1p(now - self.single_at) / SIGN_AGE_SCALE
    return row

  def plate_row(self) -> np.ndarray:
    """One of PLATES for the number confirmed, the last place when unknown."""
    row = np.zeros(INPUT_SIZES["plate"], dtype=np.float32)
    row[PLATES.index(self.plate) if self.plate in PLATES else len(PLATES)] = 1.0
    return row

  def plate_wet_row(self) -> np.ndarray:
    """A plate for wet roads below the number confirmed."""
    return np.array([self.plate == "wet"], dtype=np.float32)

  def plate_hours_row(self) -> np.ndarray:
    """From and to of the plate of the number confirmed, each on a circle as the hour is, known."""
    row = np.zeros(INPUT_SIZES["plate_hours"], dtype=np.float32)
    if self.hours is not None:
      row[:] = *hour_row(self.hours[0])[:2], *hour_row(self.hours[1])[:2], 1.0
    return row


def _summer_time(moment: datetime) -> bool:
  """The EU's: from 01:00 UTC of the last Sunday of March to 01:00 UTC of the last Sunday of October."""
  def last_sunday(month: int) -> datetime:
    last = datetime(moment.year, month + 1, 1, 1, tzinfo=timezone.utc) - timedelta(days=1)
    return last - timedelta(days=(last.weekday() + 1) % 7)

  return last_sunday(3) <= moment < last_sunday(10)


def local_hour(unix_seconds: float | None, country: str | None) -> float | None:
  """The hour of the day in a country at a moment, 0 to 24 with its fraction. None: no time from GPS yet, or another country."""
  offset = WINTER_UTC_HOURS.get(country)
  if unix_seconds is None or offset is None:
    return None
  moment = datetime.fromtimestamp(unix_seconds, timezone.utc)
  return (offset + _summer_time(moment) + (unix_seconds % 86400) / 3600) % 24


def hour_row(hour: float | None) -> np.ndarray:
  """The hour on a circle, so that a minute to midnight is next to midnight, known."""
  row = np.zeros(INPUT_SIZES["hour"], dtype=np.float32)
  if hour is not None:
    row[:] = math.sin(2 * math.pi * hour / 24), math.cos(2 * math.pi * hour / 24), 1.0
  return row


def lead_row(metres: float | None, kph: float | None) -> np.ndarray:
  """Distance and speed of the car ahead, known."""
  row = np.zeros(INPUT_SIZES["lead"], dtype=np.float32)
  if metres is not None and kph is not None:
    row[:] = min(metres, LEAD_METRES_MAX) / LEAD_METRES_SCALE, kph / SIGN_KPH_SCALE, 1.0
  return row


def sign_distance_row(metres: float) -> np.ndarray:
  """vision.SignDistance's metres to the sign ahead, clipped, and known; 0 (none, or passed) is not known."""
  row = np.zeros(INPUT_SIZES["sign_distance"], dtype=np.float32)
  if metres > 0:
    row[:] = min(metres, SIGN_METRES_MAX) / SIGN_METRES_SCALE, 1.0
  return row


def country_row(name: str | None) -> np.ndarray:
  """One of COUNTRIES, the last place for any other and for none."""
  row = np.zeros(INPUT_SIZES["country"], dtype=np.float32)
  row[COUNTRIES.index(name) if name in COUNTRIES else len(COUNTRIES)] = 1.0
  return row


class PedalInputs:
  """Gas and brake of every carState: add(), then row() at the frame."""

  def __init__(self):
    self.gas_at = -math.inf
    self.brake_at = -math.inf
    self.known = False

  def add(self, now: float, gas: bool, brake: bool) -> None:
    self.known = True
    if gas:
      self.gas_at = now
    if brake:
      self.brake_at = now

  def row(self, now: float) -> np.ndarray:
    """Gas pressed in the last second, brake pressed in the last second, known."""
    row = np.zeros(INPUT_SIZES["pedals"], dtype=np.float32)
    row[0] = now - self.gas_at <= PEDAL_SECONDS
    row[1] = now - self.brake_at <= PEDAL_SECONDS
    row[2] = self.known
    return row


class EngagedInputs:
  """Whether openpilot is engaged (selfdriveState.enabled), how long ago it last went off and what turned it off: the brake or
  the gas when it was down at that moment or within PEDAL_SECONDS of it, before or after (the car may see the pedal a little
  after openpilot goes off), the brake first; else something else (a button, an alert). add_pedals() every carState,
  add_enabled() every selfdriveState after them, then row() at the frame. The training gives it the changes of the logs in
  their order, and a pedal let go counts as down until that moment, as it is on the car at 100 Hz."""

  def __init__(self):
    self.on = False
    self.known = False
    self.off_at: float | None = None
    self.cause: str | None = None
    self.down = {"brake": False, "gas": False}
    self.down_at = {"brake": -math.inf, "gas": -math.inf}

  def add_pedals(self, now: float, gas: bool, brake: bool) -> None:
    for name, down in (("gas", bool(gas)), ("brake", bool(brake))):
      if down or self.down[name]:
        self.down_at[name] = now
      self.down[name] = down
    self._settle(now)

  def add_enabled(self, now: float, enabled: bool) -> None:
    if self.on and not enabled:
      self.off_at = now
    self.on, self.known = bool(enabled), True
    self._settle(now)

  def _settle(self, now: float) -> None:
    """The cause of the last going off, while a pedal may still come."""
    if self.off_at is None or now - self.off_at > PEDAL_SECONDS:
      return
    self.cause = next((name for name in ("brake", "gas")
                       if self.down[name] or abs(self.down_at[name] - self.off_at) <= PEDAL_SECONDS), "other")

  def row(self, now: float) -> np.ndarray:
    """Engaged, gone off since the start, the age of the last going off as a sign's age, by the brake, by the gas, by
    something else, known."""
    row = np.zeros(INPUT_SIZES["engaged"], dtype=np.float32)
    row[0] = self.on
    if self.off_at is not None:
      row[1:6] = (1.0, math.log1p(max(now - self.off_at, 0.0)) / SIGN_AGE_SCALE, self.cause == "brake", self.cause == "gas",
                  self.cause == "other")
    row[6] = self.known
    return row


class MotionInputs:
  """The car's turn signals, steering wheel and yaw: add_car_state() every carState, add_device_motion() every deviceMotion,
  then row() at the frame. The training gives it the changes of the logs in their order: a blinker that goes off counts as on
  until that moment, as it is on the car at 100 Hz, and a value is given again at each frame while the log holds it."""

  def __init__(self):
    self.left, self.right = False, False
    self.left_at, self.right_at = -math.inf, -math.inf
    self.steer, self.car_yaw, self.car_at = 0.0, 0.0, -math.inf
    self.yaw, self.yaw_at = 0.0, -math.inf

  def add_car_state(self, now: float, steering_deg: float, left: bool, right: bool, yaw_rate: float) -> None:
    """carState: steeringAngleDeg (left positive), leftBlinker, rightBlinker, yawRate (rad/s, left positive; 0 on cars
    without the sensor)."""
    if left or self.left:
      self.left_at = now
    if right or self.right:
      self.right_at = now
    self.left, self.right = bool(left), bool(right)
    self.steer, self.car_yaw, self.car_at = float(steering_deg), float(yaw_rate), now

  def add_device_motion(self, now: float, z: float, valid: bool) -> None:
    """deviceMotion.angularVelocityDevice.z (rad/s): the device's z points down, so the yaw (left positive) is -z."""
    if valid:
      self.yaw, self.yaw_at = -math.degrees(z), now

  def row(self, now: float) -> np.ndarray:
    """Left on in the last BLINKER_SECONDS, right the same, the age of the last left / right on over BLINKER_AGE_MAX (1 for
    longer or never), the steering angle / STEER_SCALE, the yaw rate / YAW_SCALE, yaw known, car known."""
    row = np.zeros(INPUT_SIZES["motion"], dtype=np.float32)
    if now - self.car_at <= MOTION_SECONDS:
      row[0] = now - self.left_at <= BLINKER_SECONDS
      row[1] = now - self.right_at <= BLINKER_SECONDS
      row[2] = min(max(now - self.left_at, 0.0), BLINKER_AGE_MAX) / BLINKER_AGE_MAX
      row[3] = min(max(now - self.right_at, 0.0), BLINKER_AGE_MAX) / BLINKER_AGE_MAX
      row[4] = np.clip(self.steer / STEER_SCALE, -STEER_MAX, STEER_MAX)
      row[7] = 1.0
    else:
      row[2:4] = 1.0
    if now - self.yaw_at <= MOTION_SECONDS:
      yaw = self.yaw
    elif now - self.car_at <= MOTION_SECONDS and abs(self.car_yaw) > CAR_YAW_MIN:
      yaw = math.degrees(self.car_yaw)
    else:
      return row
    row[5], row[6] = np.clip(yaw / YAW_SCALE, -YAW_MAX, YAW_MAX), 1.0
    return row


def speed_row(metres_per_second: float | None) -> np.ndarray:
  """The speed clipped to 0..SPEED_MAX, over SPEED_SCALE, and known."""
  row = np.zeros(INPUT_SIZES["speed"], dtype=np.float32)
  if metres_per_second is not None and math.isfinite(metres_per_second):
    row[:] = min(max(float(metres_per_second), 0.0), SPEED_MAX) / SPEED_SCALE, 1.0
  return row


def pack_inputs(inputs: dict, groups: tuple) -> np.ndarray:
  """The extra input of the network: the groups side by side, in the order the network was trained with."""
  return np.concatenate([inputs[name] for name in groups], axis=-1)
