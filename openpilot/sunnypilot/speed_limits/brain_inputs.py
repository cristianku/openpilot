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
- country: the last one GPS gave, none before the first, one of COUNTRIES only. Only for the brains trained until 13 Nov
  2026 (the server's LEFT_OUT since): country_iso takes every country of the world;
- country_iso: the same country among every ISO 3166-1 code (COUNTRY_CODES), none before the first;
- hour_tz: the local hour, as hour, from the real time zones of the whole world (zone_hour: the zone of the country nearest
  to the last fix, summer time included). hour knows only the countries of WINTER_UTC_HOURS and is kept only for the
  brains trained until 13 Nov 2026 (the server's LEFT_OUT since);
- pedals: gas and brake pressed at some moment of the last second, as carState says them;
- speed: the car's own speed.
- engaged: whether openpilot is engaged (selfdriveState.enabled), how long ago it last went off and what turned it off. A
  driver who takes over where the limit said is wrong often brakes once and then lets the car slow down by itself, with no
  pedal pressed: the pedals of the last second no longer show it, openpilot off still does.
- sign_distance: the metres to the round sign ahead as vision.SignDistance measures them from its box and the car's speed
  (the same class on the car and on the server), known; nothing once it is passed. Only for the brains trained until 12
  Nov 2026, which said where their next limit starts: no brain learns from it since (the server's LEFT_OUT), a brain gets
  the boxes themselves (sign_boxes) and no metres worked out by a formula.
- motion: the car's own turn signals, steering wheel and yaw (MotionInputs): a blinker on in the last BLINKER_SECONDS and
  how long ago it was last on, left and right; the steering angle; the yaw rate (deviceMotion, the car's yawRate when the
  device gives none); each known. A turn signal before an exit or a junction says the limit is about to change.
- road_signs: every other sign the classifier reads (ROAD_SIGNS, all the kinds of other_sign, none picked out): for each,
  whether one was confirmed (2 readings within CONFIRM_SECONDS, the numbers' own rule) and the age of its last reading
  since. Stop, give way, a crossing, a roundabout, a town or a motorway: the network weighs what each says.
- sign_boxes: where the signs of the last frame read are in the picture and how big (BoxInputs), as SignNet's boxes say,
  with no metres worked out: the picture cut in BOX_COLUMNS columns and, for the speed signs and for the other signs
  (sign_box_channel), the tallest box of each column; then whether a sign was ever seen and the age of the last one. A
  sign grows and moves to the side as the car nears it: what that means for the speed the network learns.
- light_boxes: where the traffic lights of the last frame read are in the picture and how big, the same BoxInputs with a
  channel for each of LIGHTS (light_box_channel): the tallest light of each column by what it has lit, then whether a light
  was ever seen and the age of the last one. A red light near and in front is not one far to the side.
"""
from bisect import bisect_right
from datetime import datetime, timedelta, timezone
import gzip
import json
import math
from pathlib import Path

import numpy as np

COUNTRIES = ("CH", "DE", "FR", "IT", "AT", "US")
# Every country of the world (country_iso): ISO 3166-1 alpha-2 as tzdata's iso3166.tab lists it, and Kosovo (XK), which the
# borders give. The order is fixed: a published brain learned each place; a new code goes at the end.
COUNTRY_CODES = tuple("""
  AD AE AF AG AI AL AM AO AQ AR AS AT AU AW AX AZ BA BB BD BE BF BG BH BI BJ BL BM BN BO BQ BR BS BT BV BW BY BZ CA CC CD CF CG
  CH CI CK CL CM CN CO CR CU CV CW CX CY CZ DE DJ DK DM DO DZ EC EE EG EH ER ES ET FI FJ FK FM FO FR GA GB GD GE GF GG GH GI GL
  GM GN GP GQ GR GS GT GU GW GY HK HM HN HR HT HU ID IE IL IM IN IO IQ IR IS IT JE JM JO JP KE KG KH KI KM KN KP KR KW KY KZ LA
  LB LC LI LK LR LS LT LU LV LY MA MC MD ME MF MG MH MK ML MM MN MO MP MQ MR MS MT MU MV MW MX MY MZ NA NC NE NF NG NI NL NO NP
  NR NU NZ OM PA PE PF PG PH PK PL PM PN PR PS PT PW PY QA RE RO RS RU RW SA SB SC SD SE SG SH SI SJ SK SL SM SN SO SR SS ST SV
  SX SY SZ TC TD TF TG TH TJ TK TL TM TN TO TR TT TV TW TZ UA UG UM US UY UZ VA VC VE VG VI VN VU WF WS XK YE YT ZA ZM ZW
""".split())
# The time zones of hour_tz, made by the server's scripts/make_time_zones.py from tzdata (public domain).
TIME_ZONES = Path(__file__).with_name("time-zones.json.gz")
PLATES = ("none", "time", "other")  # below a number: none, hours written on it, anything else
# Every kind of other_sign the classifier can read (the server's sign_types.SIGN_TYPES, checked equal by its tests)
ROAD_SIGNS = ("pedestrian_crossing", "children", "cyclists", "traffic_queue", "roadworks", "danger_other",
              "give_way", "stop", "priority_to_right", "main_road", "main_road_end",
              "no_overtaking", "no_overtaking_end", "no_entry", "no_vehicles", "minimum_speed", "weight_limit",
              "height_limit", "width_limit", "keep_right", "keep_left", "ahead_only",
              "town_entry", "town_exit", "motorway_start", "motorway_end", "expressway_start", "expressway_end",
              "roundabout", "one_way", "direction_left", "direction_right", "direction_straight", "direction_several",
              "motorway_direction", "motorway_exit", "other",
              # A traffic light by what is lit: a flashing amber is amber, off, amber... in the ages of the two
              "traffic_light_red", "traffic_light_amber", "traffic_light_green", "traffic_light_off",
              # Who is on the road, as the classifier reads them
              "road_user_person", "road_user_bicycle", "road_user_motorcycle", "road_user_car", "road_user_truck",
              "road_user_bus", "road_user_tram", "road_user_other")
SIGN_KPH_SCALE = 130.0
SIGN_AGE_SCALE = math.log1p(3600.0)
LEAD_METRES_SCALE = 100.0
LEAD_METRES_MAX = 200.0
SIGN_METRES_SCALE = 100.0
SIGN_METRES_MAX = 200.0
BOX_COLUMNS = 8  # a group of boxes (BoxInputs) cuts the picture in so many columns, left to right
BOX_NUMBERS = 3  # of a box: its width and its height over the frame's, its centre y over the frame's height
SIGN_BOX_CHANNELS = 2  # of sign_boxes (sign_box_channel): a speed sign or the end of a limit, any other sign
LIGHTS = ("red", "amber", "green", "off")  # the channels of light_boxes (light_box_channel): what a traffic light has lit
NOT_SIGNS = ("traffic_light_", "road_user_")  # the kinds of other_sign that are no signs: a light, who is on the road
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
               "c2303311c6b92f362273754c88fe87321ac0a3ac71967c259c9ad1161be1366d",
               "0d309056a32e48037e804302916398b71d4edd4fc765a62ea4906957d2d7e629",
               "d7016ace080049d8b8ae75119dd8f2f3b80634ebd417e5e4c8a99bcc4fdc4cd0",
               "c12716e61d873c3a34510ae58747ec3af934e17fc3e0344fa7945bca5cec8d36")


def box_size(channels: int) -> int:
  """The size of a group of boxes with so many channels: BOX_NUMBERS a channel and column, then known and the age."""
  return channels * BOX_COLUMNS * BOX_NUMBERS + 2


INPUT_SIZES = {"signs": 11, "plate": len(PLATES) + 1, "plate_hours": 5, "hour": 3, "lead": 3, "country": len(COUNTRIES) + 1,
               "pedals": 3, "speed": 2, "plate_wet": 1, "engaged": 7, "sign_distance": 2,
               "motion": 8, "road_signs": 2 * len(ROAD_SIGNS), "sign_boxes": box_size(SIGN_BOX_CHANNELS),
               "light_boxes": box_size(len(LIGHTS)), "country_iso": len(COUNTRY_CODES) + 1, "hour_tz": 3}


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
    self._road_read: dict = {}  # kind: its readings of the last CONFIRM_SECONDS
    self._road_seen: dict = {}  # kind: its last reading once confirmed

  def add(self, now: float, signs: list) -> None:
    """signs: (class, confidence), (class, confidence, plate) or (class, confidence, plate, (from, to)) of a frame. Of a
    number read twice in it, the surest."""
    kinds = set()
    for name, confidence, *_ in signs:
      family, _, kind = name.partition(":")
      if family == "other_sign" and kind in ROAD_SIGNS and confidence >= self._vision.MIN_CONFIDENCE:
        kinds.add(kind)
    for kind in kinds:
      times = [at for at in self._road_read.get(kind, []) if now - at <= self._vision.CONFIRM_SECONDS] + [now]
      self._road_read[kind] = times
      if len(times) >= self._vision.CONFIRM_FRAMES:
        self._road_seen[kind] = now
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

  def road_signs_row(self, now: float) -> np.ndarray:
    """For each of ROAD_SIGNS: confirmed, and the age of its last reading since; 0, 0 for one never confirmed."""
    row = np.zeros(INPUT_SIZES["road_signs"], dtype=np.float32)
    for at, kind in enumerate(ROAD_SIGNS):
      if kind in self._road_seen:
        row[2 * at:2 * at + 2] = 1.0, math.log1p(max(now - self._road_seen[kind], 0.0)) / SIGN_AGE_SCALE
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


def sign_box_channel(vision):
  """What gives the channel of sign_boxes for a class of the classifier: 0 for a speed sign or the end of a limit (every
  speed kind, its number read or not yet: the far sign is maximum_speed:unknown; the end of a limit; a variable display
  lit or not), 1 for any other sign (other_sign, but not NOT_SIGNS) and for a sign the classifier cannot read
  (unreadable), None for the rest (not a sign, a light, a road user)."""
  def channel(name: str) -> int | None:
    kind, _, value = name.partition(":")
    if kind in vision.SPEED_KINDS or kind == vision.CANCELLATION:
      return 0
    if kind == "other_sign" and not value.startswith(NOT_SIGNS) or kind == "unreadable":
      return 1
    return None

  return channel


def light_box_channel(name: str) -> int | None:
  """The channel of light_boxes for a class of the classifier: the place in LIGHTS of what a traffic light has lit
  (other_sign:traffic_light_red is 0), None for every other class."""
  kind, _, value = name.partition(":")
  if kind == "other_sign" and value.startswith("traffic_light_") and value[len("traffic_light_"):] in LIGHTS:
    return LIGHTS.index(value[len("traffic_light_"):])
  return None


class BoxInputs:
  """A group of boxes: where what a network found is in the picture and how big, as the network gives it. The picture is
  cut in BOX_COLUMNS columns; for each channel and column, the tallest box whose centre is in the column: its width and
  its height over the frame's, its centre y over the frame's height; 0, 0, 0 with none. Then, for the group, known (1
  once a box was given) and the age of the last reading that had one.

  add() each frame read, in order, on the car right after read_signs; row() at a step. The columns are those of the last
  reading given, held until the next: a step may come without a new reading. A reading with no box empties them; known
  and the age keep the memory.

  channels: how many the group has; channel: what gives the channel of a class, None for a class the group does not take
  (sign_box_channel for sign_boxes, light_box_channel for light_boxes). Nothing here knows what a channel holds: another group of boxes is this class with
  other channels."""

  def __init__(self, vision, channels: int, channel):
    self._vision = vision
    self._channel = channel
    self._columns = np.zeros((channels, BOX_COLUMNS, BOX_NUMBERS), dtype=np.float32)
    self._seen_at: float | None = None

  def add(self, now: float, boxes: list, width: float, height: float) -> None:
    """boxes: (class, confidence, (x1, y1, x2, y2)) of a frame, the corners in the pixels of a frame width x height.
    Only the ones read sure enough (vision.MIN_CONFIDENCE), as SignInputs.add; of a box, its part inside the frame."""
    self._columns[:] = 0.0
    tallest: dict = {}  # (channel, column): the height of its box
    for name, confidence, box in boxes:
      channel = self._channel(name)
      if channel is None or confidence < self._vision.MIN_CONFIDENCE:
        continue
      left, right = (min(max(float(x), 0.0), width) for x in (box[0], box[2]))
      top, bottom = (min(max(float(y), 0.0), height) for y in (box[1], box[3]))
      if right <= left or bottom <= top:
        continue
      column = min(int((left + right) / 2 / width * BOX_COLUMNS), BOX_COLUMNS - 1)
      tall = (bottom - top) / height
      if tall > tallest.get((channel, column), 0.0):
        tallest[channel, column] = tall
        self._columns[channel, column] = (right - left) / width, tall, (top + bottom) / 2 / height
    if tallest:
      self._seen_at = now

  def row(self, now: float) -> np.ndarray:
    """Channel after channel, its columns from the left, of each the width, the height and the centre y of its box; then
    known and the age of the last reading with a box, as a sign's age."""
    row = np.zeros(self._columns.size + 2, dtype=np.float32)
    row[:-2] = self._columns.ravel()
    if self._seen_at is not None:
      row[-2:] = 1.0, math.log1p(max(now - self._seen_at, 0.0)) / SIGN_AGE_SCALE
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


_zones: dict | None = None


def _time_zones() -> dict:
  """TIME_ZONES by country: [(latitude, longitude, starts, offsets in minutes)]; none without the file."""
  global _zones
  if _zones is None:
    _zones = {}
    try:
      for zone in json.loads(gzip.decompress(TIME_ZONES.read_bytes()))["zones"]:
        starts, minutes = zip(*zone["offsets"], strict=True)
        for code in zone["countries"]:
          _zones.setdefault(code, []).append((*zone["position"], starts, minutes))
    except (OSError, ValueError, KeyError, TypeError, EOFError):
      _zones = {}
  return _zones


def zone_hour(unix_seconds: float | None, country: str | None, position: tuple[float, float] | None = None) -> float | None:
  """The hour of the day, 0 to 24 with its fraction, in the time zone of the country nearest to position (the last fix:
  latitude, longitude). A country of one zone needs no position. None: no time from GPS yet, no country, or a country of
  several zones and no position."""
  zones = _time_zones().get(country)
  if unix_seconds is None or not zones or (len(zones) > 1 and position is None):
    return None
  if len(zones) > 1:
    latitude, longitude = position
    scale = math.cos(math.radians(latitude))
    zones = [min(zones, key=lambda z: (z[0] - latitude) ** 2 + (((z[1] - longitude + 180) % 360 - 180) * scale) ** 2)]
  _, _, starts, minutes = zones[0]
  offset = minutes[max(bisect_right(starts, unix_seconds) - 1, 0)]
  return ((unix_seconds + offset * 60) % 86400) / 3600


def hour_row(hour: float | None, group: str = "hour") -> np.ndarray:
  """The hour on a circle, so that a minute to midnight is next to midnight, known. group: hour or hour_tz."""
  row = np.zeros(INPUT_SIZES[group], dtype=np.float32)
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


def country_iso_row(name: str | None) -> np.ndarray:
  """One of COUNTRY_CODES, the last place for none (and for a code it does not know)."""
  row = np.zeros(INPUT_SIZES["country_iso"], dtype=np.float32)
  row[COUNTRY_CODES.index(name) if name in COUNTRY_CODES else len(COUNTRY_CODES)] = 1.0
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
