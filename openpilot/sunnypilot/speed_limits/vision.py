"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

VISION AND MAP ARE TWO SEPARATE SOURCES: NOTHING HERE OR IN SPEEDVISIOND READS THE MAP (liveMapDataSP). THE VISION LIMIT
COMES FROM THE CAMERA AND THE CAR'S OWN SIGNALS ONLY; SUNNYPILOT'S RESOLVER ("MAP DATA + VISION") WEIGHS THE TWO.

The speed-limits networks' inputs and outputs, in numpy: the road camera's NV12 frame to SignNet's band and the
classifier's crops, SignNet's boxes, the classifier's class to a speed, and the memory that turns the readings of
single frames into the limit shown. The networks were trained on frames that ffmpeg decoded to RGB (swscale's BT.601
limited range in 16-bit fixed point, each chroma sample on 2 x 2 pixels) and that PIL resized (bilinear,
antialiased, in 8-bit fixed point): the same arithmetic is done here, bit for bit, so the networks see on the car what
they saw in training.
"""
import math

import numpy as np

SIGNNET_WIDTH = 896
SIGNNET_HEIGHT = 288  # the top of the frame scaled to SIGNNET_WIDTH: 97 % of the signs end above row 420 of 760
SIGNNET_STRIDE = 8
MAX_BOXES = 5  # the classifier takes about 6 ms a box on the Adreno
MIN_BOX_SCORE = 0.3

CLASSIFIER_SIZE = 160
CLASSIFIER_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32).reshape(3, 1, 1)
CLASSIFIER_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32).reshape(3, 1, 1)
MIN_CROP_SIDE = 2
MIN_CONFIDENCE = 0.7
OTHER_ROAD = "other_road"  # a sign the applicability network gave to another road (applicability.OTHER_ROAD): not taken

# A reading becomes the limit when it is in CONFIRM_FRAMES frames within CONFIRM_SECONDS; it stays until another does.
CONFIRM_FRAMES = 2
CONFIRM_SECONDS = 1.5

# A number valid at some hours, its plate below it, one box around both. To the rules it is its number: no code turns the
# hours into a limit, that is the brain's (class_plate).
MAXIMUM_SPEED_HOURS = "maximum_speed_hours"
# The same for a number valid only on a wet road (the German "bei Nässe"): no code turns it into a limit either.
MAXIMUM_SPEED_WET = "maximum_speed_wet"
PLATED_KINDS = (MAXIMUM_SPEED_HOURS, MAXIMUM_SPEED_WET)  # taller by their plate
SPEED_KINDS = ("maximum_speed", *PLATED_KINDS, "variable_display", "zone", "road_marking")
CANCELLATION = "cancellation"

# The general limits that hold where no number is signed (km/h; None: no general limit), by country, for the kind of
# road: in a town, outside towns, on an expressway (Autostrasse), on a motorway. From the laws of each country,
# sunnypilot.speedlimits docs/road-rules/<cc>.md (29 Oct 2026). Where a kind's limit depends on what the camera cannot
# tell (a single or a dual carriageway, the region), the lower one. GB/US implicit rules remain unverified here;
# support for their sign units is independent of this legal table.
IMPLICIT_LIMITS = {
  "CH": {"town": 50, "rural": 80, "expressway": 100, "motorway": 120},
  "LI": {"town": 50, "rural": 80, "expressway": 80, "motorway": 80},
  "DE": {"town": 50, "rural": 100, "expressway": 100, "motorway": None},
  "AT": {"town": 50, "rural": 100, "expressway": 100, "motorway": 130},
  "IT": {"town": 50, "rural": 90, "expressway": 110, "motorway": 130},
  "FR": {"town": 50, "rural": 80, "expressway": 110, "motorway": 130},
  "NL": {"town": 50, "rural": 80, "expressway": 100, "motorway": 130},
  "BE": {"town": 50, "rural": 70, "expressway": 70, "motorway": 120},  # Flanders; Brussels 30 in town, Wallonia 90
  "LU": {"town": 50, "rural": 90, "expressway": 90, "motorway": 130},
  "ES": {"town": 50, "rural": 90, "expressway": 90, "motorway": 120},
  "PT": {"town": 50, "rural": 90, "expressway": 100, "motorway": 120},
  "IE": {"town": 50, "rural": 80, "expressway": 100, "motorway": 120},
  "DK": {"town": 50, "rural": 80, "expressway": 80, "motorway": 130},
  "SE": {"town": 50, "rural": 70, "expressway": 70, "motorway": 110},
  "NO": {"town": 50, "rural": 80, "expressway": 80, "motorway": 80},
  "FI": {"town": 50, "rural": 80, "expressway": 80, "motorway": 80},
  "PL": {"town": 50, "rural": 90, "expressway": 100, "motorway": 140},  # an expressway of two carriageways is 120
  "CZ": {"town": 50, "rural": 90, "expressway": 110, "motorway": 130},
  "SK": {"town": 50, "rural": 90, "expressway": 90, "motorway": 130},
  "HU": {"town": 50, "rural": 90, "expressway": 110, "motorway": 130},
  "SI": {"town": 50, "rural": 90, "expressway": 110, "motorway": 130},
  "HR": {"town": 50, "rural": 90, "expressway": 110, "motorway": 130},
}
DEFAULT_COUNTRY = None  # no GPS country: new numeric observations are blocked
# Where a sign of the kind of road does not end the limit held (it holds until its end sign or the next number): it
# only says which road the car is on.
ROAD_SIGN_KEEPS_LIMIT = ("SE", "NO", "FI")
# The signs that say which kind of road begins: their general limit replaces the limit held.
ROAD_SIGNS = {
  "other_sign:motorway_start": "motorway",
  "other_sign:motorway_end": "rural",
  "other_sign:expressway_start": "expressway",
  "other_sign:expressway_end": "rural",
  "other_sign:town_entry": "town",
  "other_sign:town_exit": "rural",
}
# The green boards towards a motorway or an expressway (in Switzerland where the entry ramp starts, and at the
# interchanges): a carriageway is near, never its limit (the ramp has its own). Confirmed as the other signs are, they
# only give the road-type network its input board (board_age): the limit, the road and the ramp never read them.
BOARD_SIGNS = ("other_sign:motorway_direction",)
# A number read on a ramp (the road-type network says so) or within RAMP_SECONDS of a motorway or expressway sign is the
# limit of the ramp: it ends where the acceleration lane starts, at the end of the entry ramp, when the road-type network
# (which remembers the ramp) says "merge" and the carriageway is known from its sign; else when it sees the carriageway
# (MERGE_SECONDS of "merge", "motorway" or "expressway" in a row). Without the network it holds until the next sign.
# Once on the acceleration lane, or past a ramp's limit, a number is the carriageway's, however near its sign.
RAMP_SECONDS = 45.0
MERGE_SECONDS = 8.0
# A number read off the carriageways (in a town, on a rural road, before a ramp the network did not see: its end sign
# missed) does not go onto a motorway or an expressway: once the network has seen the carriageway for ROAD_TYPE_SECONDS,
# its general limit. Roadworks there repeat their sign on the carriageway, which the camera reads again. A number read
# less than CARRIAGEWAY_LAG seconds before the network saw the carriageway may be the carriageway's (its debounce lags
# the scene): it holds.
CARRIAGEWAY_LAG = 5.0

# The road-type network (optional in a release, RoadTypeReader): one step a second on the whole frame at 256 x 144, its
# memory carried from step to step; its answer counts once the same class comes ROAD_TYPE_DEBOUNCE steps in a row.
ROAD_TYPE_WIDTH = 256
ROAD_TYPE_HEIGHT = 144
BRAIN_WIDTH = 256
BRAIN_HEIGHT = 144
ROAD_TYPE_PERIOD = 1.0
ROAD_TYPE_DEBOUNCE = 3
ROAD_TYPE_RESET_SECONDS = 5.0  # longer without a step: its memory starts again from zeros
ROAD_KINDS = ("town", "rural", "expressway", "motorway", "zone30")  # the road types with a general limit
CARRIAGEWAYS = ("motorway", "expressway")
# A 30 zone (the road-type network's zone30): the legacy physical value is 30 km/h where implicit rules are known.
# It never becomes 30 mph. Seen for ROAD_TYPE_SECONDS
# after the last sign, it also lowers a town's general limit held from a sign (its gate sign missed, a 50 read on the
# street before): the car must not speed up to 50 inside it. A higher number (60, 80) holds: the network can be wrong.
ZONE_LIMIT = 30.0
# zone30 is said only this sure (its probability), else town: with Smartspeed the resolver takes only vision,
# and a wrong 30 on a 50 street is worse than a zone missed. 0.9 for the first network with the class; 0.99 the aim.
ZONE_CONFIDENCE = 0.9
SIGN_AGE_MAX = 120.0
# The car's motion input (car_motion.motion_features of the training): the samples of the last second, the blinkers of
# the last two; fewer than MOTION_MIN_SAMPLES in the second and that part is unknown.
MOTION_SECONDS = 1.0
BLINKER_SECONDS = 2.0
MOTION_MIN_SAMPLES = 10
# Where no sign gives the limit the scene does: a car park (CAR_PARKS) gives its limit once the network has seen it for
# PARKING_SECONDS below PARKING_MAX_SPEED (m/s), and drops the limit of the road before: GARAGE_LIMIT underground or
# indoors (garage), PARKING_LIMIT in the open (parking). A network trained before the class garage says parking for
# both: then GARAGE_LIMIT (SpeedLimitMemory.garage_known). Out of it (a road type with a general limit seen for
# PARKING_EXIT_SECONDS after the debounce: a moment of a street inside a car park keeps it, and the car speeds up a few
# seconds after it is out), or where nothing is held, the road type seen gives its general limit (town: 50). Such a
# limit follows another road type only after ROAD_TYPE_SECONDS of it; a sign always wins.
CAR_PARKS = ("parking", "garage")
GARAGE_LIMIT = 10.0
PARKING_LIMIT = 20.0
PARKING_SECONDS = 5.0
PARKING_MAX_SPEED = 25 / 3.6
# 3 s: on the test drives of the scene networks (2026-10-30_1216 on the car, 0007, 0046) no car park left while still
# inside, where 0 to 2 s left one once; 10 s (ROAD_TYPE_SECONDS, before) kept the car at the car park's limit 10 to
# 13 s on the street.
PARKING_EXIT_SECONDS = 3.0
ROAD_TYPE_SECONDS = 10.0

# For the logs only (speedVisionSP), nothing here reads them. The rule that set the limit held (SpeedLimitMemory.
# limit_reason; "" before the first), kept while a rule that reads no sign only sets the same limit and state again: a
# limit has no timeout, it holds until one of these sets another.
LIMIT_REASONS = (
  "country_change",  # a confirmed jurisdiction change invalidates/re-evaluates memory
  "sign",  # a number confirmed (in a car park, one lower than its limit)
  "end_sign",  # the end of a limit confirmed: the general limit of the road whose kind is known, else 0 (unknown)
  "road_sign",  # a sign of the kind of road confirmed (ROAD_SIGNS): its general limit
  "ramp_end",  # the ramp's limit ended: on the acceleration lane of a known carriageway, or MERGE_SECONDS on one
  "carriageway",  # a number read off the carriageways ended once the carriageway was seen: its general limit
  "car_park",  # a car park seen for PARKING_SECONDS below PARKING_MAX_SPEED: GARAGE_LIMIT or PARKING_LIMIT
  "road_type",  # the road type seen where no sign gives the limit (nothing held, out of a car park, another road type
                # for ROAD_TYPE_SECONDS): its general limit
  "zone30",  # a 30 zone seen for ROAD_TYPE_SECONDS lowers a town's general limit held from a sign
)
# What the limit held says (SpeedLimitMemory.limit_state): "limit" (speed_limit is the limit), "none" (the road's kind
# is known and has no general limit: the German motorway), "unknown" (nothing gives one: the start of a drive, the end of
# a limit on a road of unknown kind). speed_limit is 0 unless "limit".
LIMIT_STATES = ("limit", "none", "unknown")

# The distance to the sign ahead (SignDistance, speedVisionSP.signDistance): shown on the screen for the driver to
# check, logged, and where a limit read on that sign starts (LimitAhead). A round sign (ROUND_SIGNS: a number, the end of a limit) of side s
# pixels is f * D / s metres away, f the road camera's focal length in pixels and D the sign's diameter, by the kind of
# road the memory knows (SIGN_DIAMETERS, the Swiss sizes; DEFAULT_SIGN_DIAMETER when not known). The box of a number with
# hours holds the plate too: its width is taken as the disc's side. A motorway or
# expressway sign or its end (ROAD_SIGN_HEIGHTS) is measured the same way by its height, which stays when it is seen
# from the side: its formats differ more (on our drives about 1.3 m on a post and 2.1 m on a gantry, still to be
# checked), so its size is held more loosely (ROAD_SIGN_SPREAD). Of the signs of a frame, the nearest by its size.
# As the car drives towards it, 1 / s falls in a line with the metres driven, to 0 at the sign: the sightings of the
# sign are fitted to that line, D held near its size (within about DISTANCE_DIAMETER_SPREAD of it) and the box's side
# taken as right within about DISTANCE_SIDE_NOISE, so the distance gets surer, and a wrong size is corrected, as the
# sign grows. Between the sightings, and once the sign has left the frame, the metres driven are taken off, down to 0
# when the car passes it; DISTANCE_SECONDS after the last sighting it is 0 too (the car turned, or stopped).
# A gantry's LED limit (variable_display with a number; not one that is off) is round too, VARIABLE_DISPLAY_DIAMETER
# across: 1.4 m, the median of 10 approaches on our drives, from how its box grew with the metres driven.
# The distance shown is shorter by DISTANCE_LEAD_SECONDS x the speed + DISTANCE_LEAD_METRES. Without it the shown
# zero came late (Data › Sign distance, 28 signs marked by hand): the frame is already old when it is read, the networks
# take their time before the message goes, each value stays on the screen for speedvisiond's PERIOD (0.2 s), and the zero
# a person marks is the frame where the sign leaves the picture, a few metres before the car passes it. 20 signs marked
# on builds with it (5 Nov 2026) still came about 5 m late at any speed, so the metres went from 1.8 to 6.8.
ROUND_SIGNS = ("maximum_speed", *PLATED_KINDS, "cancellation")
VARIABLE_DISPLAY = "variable_display"
VARIABLE_DISPLAY_DIAMETER = 1.4
SIGN_DIAMETERS = {"town": 0.6, "zone30": 0.6, "rural": 0.8, "expressway": 1.0, "motorway": 1.2}
DEFAULT_SIGN_DIAMETER = 0.8
DISTANCE_DIAMETER_SPREAD = 0.25
ROAD_SIGN_HEIGHTS = dict.fromkeys(("other_sign:motorway_start", "other_sign:motorway_end", "other_sign:expressway_start",
                                   "other_sign:expressway_end"), 1.7)
ROAD_SIGN_SPREAD = 0.4
DISTANCE_SIDE_NOISE = 0.1
DISTANCE_SECONDS = 15.0
DISTANCE_LEAD_SECONDS = 0.34
DISTANCE_LEAD_METRES = 6.8
# The same sign again: its class, seen within TRACK_SECONDS of the last sighting and not smaller than TRACK_SHRINK of
# its last side (the car drives towards it). Else another sign, its sightings from scratch.
TRACK_SECONDS = 2.0
TRACK_SHRINK = 0.8
# The metres the distance to the same sign may correct LimitAhead's count upwards: its size on the camera wavers.
LIMIT_AHEAD_SLACK = 5.0
TRACK_SIGHTINGS = 50  # the last 10 s of sightings at most (stopped in front of a sign)
ODOMETER_MAX_STEP = 0.1  # seconds: a longer gap between two carState messages adds nothing
# The road camera's focal length (pixels) by the size of its frames, for the server, which has no openpilot: speedvisiond
# takes it from DEVICE_CAMERAS (a test checks that they agree).
FOCAL_LENGTHS = {(1344, 760): 1141.5, (1928, 1208): 2648.0, (1164, 874): 910.0}

# The settings the release may change (speed_limits.json "settings", set on the training server's portal, Models › On
# the car): its key, the constant here, its bounds. A key not known, not a number or out of bounds keeps the default.
SETTINGS = {
  "min_confidence": ("MIN_CONFIDENCE", 0.3, 0.99),
  "confirm_frames": ("CONFIRM_FRAMES", 1, 10),
  "confirm_seconds": ("CONFIRM_SECONDS", 0.2, 5.0),
  "road_type_debounce": ("ROAD_TYPE_DEBOUNCE", 1, 10),
  "road_type_seconds": ("ROAD_TYPE_SECONDS", 2.0, 60.0),
  "zone_confidence": ("ZONE_CONFIDENCE", 0.5, 0.999),
  "parking_seconds": ("PARKING_SECONDS", 1.0, 30.0),
  "parking_exit_seconds": ("PARKING_EXIT_SECONDS", 0.0, 30.0),
  "parking_limit": ("PARKING_LIMIT", 5.0, 30.0),
  "garage_limit": ("GARAGE_LIMIT", 5.0, 30.0),
  "ramp_seconds": ("RAMP_SECONDS", 5.0, 120.0),
  "merge_seconds": ("MERGE_SECONDS", 2.0, 30.0),
  "carriageway_lag": ("CARRIAGEWAY_LAG", 0.0, 30.0),
  "distance_lead_seconds": ("DISTANCE_LEAD_SECONDS", 0.0, 1.5),
  "distance_lead_metres": ("DISTANCE_LEAD_METRES", 0.0, 15.0),
  "variable_display_diameter": ("VARIABLE_DISPLAY_DIAMETER", 0.6, 3.0),
}

PRECISION_BITS = 22  # PIL's fixed point for 8-bit images (Resample.c)


def apply_settings(settings) -> dict:
  """The release's settings put in place of the constants (once, before the memory and the readers are made); returns
  the ones taken. An integer setting takes only whole numbers."""
  taken = {}
  for key, value in (settings.items() if isinstance(settings, dict) else ()):
    if key not in SETTINGS or isinstance(value, bool) or not isinstance(value, (int, float)):
      continue
    name, low, high = SETTINGS[key]
    whole = isinstance(globals()[name], int)
    if not low <= value <= high or not math.isfinite(value) or (whole and value != int(value)):
      continue
    globals()[name] = taken[key] = int(value) if whole else float(value)
  return taken


def _yuv_term(coefficient: int, offset: int) -> np.ndarray:
  """swscale's term of a channel for each 8-bit value (x86 yuv2rgb: the value * 8 less the offset, times the
  coefficient in 13-bit fixed point, the high 16 bits)."""
  return ((np.arange(256) * 8 - offset) * coefficient >> 16).astype(np.int16)


# BT.601 limited range: 255 / 219 for luma, 1.596, -0.392, -0.813 and 2.017 for chroma, times 8192.
LUMA = _yuv_term(9539, 16 * 8)
RED_V = _yuv_term(13075, 128 * 8)
GREEN_U = _yuv_term(-3209, 128 * 8)
GREEN_V = _yuv_term(-6660, 128 * 8)
BLUE_U = _yuv_term(16525, 128 * 8)


def resample_coefficients(in_size: int, out_size: int, start: float = 0.0,
                          end: float | None = None) -> tuple[np.ndarray, np.ndarray]:
  """PIL's bilinear resampling of in_size pixels to out_size (Resample.c precompute_coeffs and
  normalize_coeffs_8bpc): for each output pixel the input pixels it reads (out_size, taps) and their weights in
  fixed point. Taps of weight 0 are left out where they can be (they add nothing), the others have weight 0.
  start, end: only the part start..end of the in_size pixels, as Image.resize with a box; its taps may read the pixels
  just outside it. PIL takes the box as float32 and subtracts in float32: so here."""
  start, end = np.float32(start), np.float32(in_size if end is None else end)
  scale = float(end - start) / out_size
  start = float(start)
  filterscale = max(scale, 1.0)
  support = filterscale  # the bilinear filter's support is 1
  ss = 1.0 / filterscale
  first, weights = [], []
  for xx in range(out_size):
    center = start + (xx + 0.5) * scale
    xmin = max(int(center - support + 0.5), 0)
    xmax = min(int(center + support + 0.5), in_size) - xmin
    k = [max(0.0, 1.0 - abs((x + xmin - center + 0.5) * ss)) for x in range(xmax)]
    ww = 0.0
    for w in k:
      ww += w
    fixed = [int(0.5 + (w / ww if ww != 0.0 else w) * (1 << PRECISION_BITS)) for w in k]
    used = [x for x, w in enumerate(fixed) if w != 0] or [0]
    first.append(xmin + used[0])
    weights.append(fixed[used[0]:used[-1] + 1])
  taps = max(len(w) for w in weights)
  index = np.minimum(np.array(first)[:, None] + np.arange(taps), in_size - 1)
  fixed = np.array([w + [0] * (taps - len(w)) for w in weights], dtype=np.int32)
  return index, fixed


class ColumnResampler:
  """PIL's resampling of the columns of uint8 planes (C, H, in_size) to out_size, fast where the scale is a ratio
  of small numbers p / q (1344 to 896 is 3 / 2): away from the edges every q-th output reads the same taps of every
  p-th input with the same weights, so the inputs are split by their place in p and each tap is one multiply-add of
  whole planes. The edges, and sizes without such a ratio, take the general way."""
  def __init__(self, in_size: int, out_size: int):
    self.in_size, self.out_size = in_size, out_size
    self.index, self.fixed = resample_coefficients(in_size, out_size)
    g = math.gcd(in_size, out_size)
    self.p, self.q = in_size // g, out_size // g
    self.phases: list[tuple[int, int, list[tuple[int, int, int]]]] = []  # (first, end, [(plane, offset, weight)])
    covered = np.zeros(out_size, dtype=bool)
    if self.q <= 4 and out_size >= 4 * self.q:
      cycles = out_size // self.q
      for r in range(self.q):
        middle = (cycles // 2) * self.q + r
        base = self.index[middle] - self.p * (cycles // 2)
        weights = self.fixed[middle]
        outputs = np.arange(r, out_size, self.q)
        m = (outputs - r) // self.q
        same = np.all((self.index[outputs] == self.p * m[:, None] + base) & (self.fixed[outputs] == weights), axis=1)
        taps = [(int(b) % self.p, int(b) // self.p, int(w)) for b, w in zip(base, weights, strict=True) if w != 0]
        # the longest run of matching outputs around the middle whose inputs are all inside the frame
        same &= np.all([(m + offset >= 0) & (m + offset < in_size // self.p) for _, offset, _ in taps], axis=0)
        first = end = cycles // 2
        while first > 0 and same[first - 1]:
          first -= 1
        while end < len(m) and same[end]:
          end += 1
        if end > first:
          self.phases.append((first, end, taps))
          covered[outputs[first:end]] = True
    if in_size % self.p:
      self.phases, covered[:] = [], False
    self.edges = np.flatnonzero(~covered)

  def __call__(self, planes: np.ndarray) -> np.ndarray:
    channels, height, width = planes.shape
    out = np.empty((channels, height, self.out_size), dtype=np.uint8)
    if self.phases:
      split = np.moveaxis(planes.reshape(channels, height, width // self.p, self.p), 3, 0).astype(np.int32)
      for r, (first, end, taps) in enumerate(self.phases):
        total = np.full((channels, height, end - first), 1 << (PRECISION_BITS - 1), dtype=np.int32)
        for plane, offset, weight in taps:
          total += split[plane, :, :, first + offset:end + offset] * weight
        total >>= PRECISION_BITS
        out[:, :, first * self.q + r:(end - 1) * self.q + r + 1:self.q] = np.minimum(total, 255)
    if len(self.edges) > self.out_size // 4:  # the general way: the columns as rows, whole rows are fast to gather
      out[:, :, self.edges] = transpose(resample_rows(transpose(planes), self.index[self.edges], self.fixed[self.edges]))
    elif len(self.edges):  # a few columns at the edges
      total = np.full((channels, height, len(self.edges)), 1 << (PRECISION_BITS - 1), dtype=np.int32)
      for tap in range(self.index.shape[1]):
        total += planes[:, :, self.index[self.edges, tap]] * self.fixed[self.edges, tap]
      out[:, :, self.edges] = np.minimum(total >> PRECISION_BITS, 255)
    return out


def resample_rows(planes: np.ndarray, index: np.ndarray, fixed: np.ndarray) -> np.ndarray:
  """One pass of PIL's resampling of uint8 planes (C, H, W) along their rows (axis 1: whole rows, the fast way in
  numpy)."""
  total = np.full((planes.shape[0], index.shape[0], planes.shape[2]), 1 << (PRECISION_BITS - 1), dtype=np.int32)
  for tap in range(index.shape[1]):
    total += planes[:, index[:, tap]] * fixed[:, tap].reshape(1, -1, 1)
  return np.minimum(total >> PRECISION_BITS, 255).astype(np.uint8)


def transpose(planes: np.ndarray) -> np.ndarray:
  return np.ascontiguousarray(planes.transpose(0, 2, 1))


def resize(planes: np.ndarray, width: int, height: int) -> np.ndarray:
  """PIL's Image.resize((width, height), BILINEAR) of uint8 planes (C, H, W): columns first, then rows."""
  if planes.shape[2] != width:
    planes = transpose(resample_rows(transpose(planes), *resample_coefficients(planes.shape[2], width)))
  if planes.shape[1] != height:
    planes = resample_rows(planes, *resample_coefficients(planes.shape[1], height))
  return planes


def nv12_planes(data: np.ndarray, width: int, height: int, stride: int, uv_offset: int) -> tuple[np.ndarray, np.ndarray]:
  """Copies of the luma (height, width) and the interleaved chroma (height / 2, width) of an NV12 buffer: the
  camera may write the buffer again while the networks read it."""
  y = data[:stride * height].reshape(height, stride)[:, :width].copy()
  uv = data[uv_offset:uv_offset + stride * (height // 2)].reshape(height // 2, stride)[:, :width].copy()
  return y, uv


def yuv_to_rgb(y: np.ndarray, uv: np.ndarray, top: int, bottom: int, left: int, right: int) -> np.ndarray:
  """The RGB planes (uint8, 3 x rows x columns) of rows top..bottom and columns left..right, as ffmpeg decodes the
  road camera's video to the frames of the dataset: each chroma sample on its 2 x 2 pixels, each channel the sum of
  swscale's terms."""
  # The pairs of rows that hold the region (pairs, 2, columns): each chroma row, its terms on 2 columns, added to both.
  r0, c0, r1, c1 = top // 2, left // 2, (bottom + 1) // 2, (right + 1) // 2
  u, v = uv[r0:r1, 2 * c0:2 * c1:2], uv[r0:r1, 2 * c0 + 1:2 * c1:2]
  luma = LUMA[y[2 * r0:2 * r1, 2 * c0:2 * c1]].reshape(r1 - r0, 2, 2 * (c1 - c0))
  pairs = np.empty((3, r1 - r0, 2, 2 * (c1 - c0)), dtype=np.uint8)
  for plane, term in zip(pairs, (RED_V[v], GREEN_U[u] + GREEN_V[v], BLUE_U[u]), strict=True):
    np.clip(luma + np.repeat(term, 2, axis=1)[:, None, :], 0, 255, out=plane, casting="unsafe")
  dy, dx = top - 2 * r0, left - 2 * c0
  return pairs.reshape(3, 2 * (r1 - r0), 2 * (c1 - c0))[:, dy:dy + bottom - top, dx:dx + right - left]


class SignNetInput:
  """SignNet's input for frames of one size: the frame scaled to SIGNNET_WIDTH and its top SIGNNET_HEIGHT rows,
  as float 0..1 (1, 3, SIGNNET_HEIGHT, SIGNNET_WIDTH)."""
  def __init__(self, width: int, height: int):
    self.width, self.height = width, height
    self.scale = width / SIGNNET_WIDTH  # a band pixel in frame pixels
    self.columns = ColumnResampler(width, SIGNNET_WIDTH) if width != SIGNNET_WIDTH else None
    rows_index, rows_fixed = resample_coefficients(height, round(height * SIGNNET_WIDTH / width))
    self.rows = rows_index[:SIGNNET_HEIGHT], rows_fixed[:SIGNNET_HEIGHT]
    self.used_rows = int(self.rows[0].max()) + 1

  def band(self, y: np.ndarray, uv: np.ndarray) -> np.ndarray:
    """The band as uint8 planes (3, SIGNNET_HEIGHT, SIGNNET_WIDTH)."""
    planes = yuv_to_rgb(y, uv, 0, self.used_rows, 0, self.width)
    if self.columns is not None:
      planes = self.columns(planes)
    return resample_rows(planes, *self.rows)

  def __call__(self, y: np.ndarray, uv: np.ndarray) -> np.ndarray:
    pixels = self.band(y, uv)[None].astype(np.float32)
    return np.divide(pixels, 255.0, out=pixels)


class RoadTypeInput:
  """The road-type network's image for frames of one size: the whole frame at ROAD_TYPE_WIDTH x ROAD_TYPE_HEIGHT, RGB
  float 0..1 (1, 3, ROAD_TYPE_HEIGHT, ROAD_TYPE_WIDTH), bit for bit the frames of its training (road_type_data)."""
  def __init__(self, width: int, height: int):
    self.width, self.height = width, height
    self.columns = ColumnResampler(width, ROAD_TYPE_WIDTH) if width != ROAD_TYPE_WIDTH else None
    self.rows = resample_coefficients(height, ROAD_TYPE_HEIGHT)

  def __call__(self, y: np.ndarray, uv: np.ndarray) -> np.ndarray:
    planes = yuv_to_rgb(y, uv, 0, self.height, 0, self.width)
    if self.columns is not None:
      planes = self.columns(planes)
    pixels = resample_rows(planes, *self.rows)[None].astype(np.float32)
    return np.divide(pixels, 255.0, out=pixels)


class BrainInput:
  """The brain's image for frames of one size: the luma and the chroma planes scaled first (PIL's bilinear weights, rows
  then columns), and only then RGB at BRAIN_WIDTH x BRAIN_HEIGHT as yuv_to_rgb makes it, float 0..1 (1, 3, BRAIN_HEIGHT,
  BRAIN_WIDTH). Far cheaper than RoadTypeInput, which turns the whole frame to RGB first, so that the brain can step at
  every reading. The server makes the training pictures with this same class from the decoded NV12 frames."""
  def __init__(self, width: int, height: int):
    self.luma_rows = resample_coefficients(height, BRAIN_HEIGHT)
    self.luma_columns = ColumnResampler(width, BRAIN_WIDTH)
    self.chroma_rows = resample_coefficients(height // 2, BRAIN_HEIGHT // 2)
    self.chroma_columns = ColumnResampler(width // 2, BRAIN_WIDTH // 2)

  def planes(self, y: np.ndarray, uv: np.ndarray) -> np.ndarray:
    """The image as uint8 planes (3, BRAIN_HEIGHT, BRAIN_WIDTH)."""
    luma = self.luma_columns(resample_rows(y[None], *self.luma_rows))[0]
    chroma = resample_rows(uv.reshape(uv.shape[0], -1, 2).transpose(2, 0, 1), *self.chroma_rows)
    chroma = self.chroma_columns(chroma)
    interleaved = np.empty((BRAIN_HEIGHT // 2, BRAIN_WIDTH), dtype=np.uint8)
    interleaved[:, 0::2], interleaved[:, 1::2] = chroma
    return yuv_to_rgb(luma, interleaved, 0, BRAIN_HEIGHT, 0, BRAIN_WIDTH)

  def __call__(self, y: np.ndarray, uv: np.ndarray) -> np.ndarray:
    pixels = self.planes(y, uv)[None].astype(np.float32)
    return np.divide(pixels, 255.0, out=pixels)


class CarMotion:
  """The car's own signals of the last seconds as the road-type network's motion input: the yaw rate (deviceMotion), the
  steering angle and the blinkers (carState). Never the map."""
  def __init__(self):
    self.yaw: list[tuple[float, float]] = []  # (time, yaw rate deg/s, left positive) of the valid samples
    self.car: list[tuple[float, float, bool, bool, float]] = []  # (time, steering deg, left, right, yawRate deg/s)

  def add_device_motion(self, now: float, z: float, valid: bool) -> None:
    """angularVelocityDevice.z (rad/s): the device's z points down, so the yaw (left positive) is -z."""
    if valid:
      self.yaw.append((now, -math.degrees(z)))

  def add_car_state(self, now: float, steering_deg: float, left: bool, right: bool, yaw_rate: float) -> None:
    """carState: steeringAngleDeg (left positive), the blinkers, yawRate (rad/s, left positive; 0 on cars without it)."""
    self.car.append((now, steering_deg, left, right, math.degrees(yaw_rate)))

  def features(self, now: float) -> np.ndarray:
    """(1, 8): heading change, largest yaw, yaw known, steering, steering known, left, right, blinkers known."""
    self.yaw = [s for s in self.yaw if now - s[0] < BLINKER_SECONDS]
    self.car = [s for s in self.car if now - s[0] < BLINKER_SECONDS]
    step = [s for s in self.car if now - s[0] < MOTION_SECONDS]
    yaw = [rate for t, rate in self.yaw if now - t < MOTION_SECONDS]
    if len(yaw) < MOTION_MIN_SAMPLES and len(step) >= MOTION_MIN_SAMPLES and any(s[4] != 0 for s in step):
      yaw = [s[4] for s in step]
    motion = np.zeros((1, 8), dtype=np.float32)
    if len(yaw) >= MOTION_MIN_SAMPLES:
      motion[0, :3] = np.clip(np.mean(yaw) / 30, -3, 3), np.clip(max(abs(rate) for rate in yaw) / 30, 0, 3), 1
    if len(step) >= MOTION_MIN_SAMPLES:
      motion[0, 3:5] = np.clip(np.mean([s[1] for s in step]) / 180, -4, 4), 1
      motion[0, 5:8] = any(s[2] for s in self.car), any(s[3] for s in self.car), 1
    return motion


def age_input(age: float) -> np.ndarray:
  """The road-type network's input for a sign or a board confirmed age seconds ago: [age / 60, 1] up to SIGN_AGE_MAX,
  [2, 0] longer ago or never."""
  return np.array([[min(age, SIGN_AGE_MAX) / 60.0, 1.0]] if age <= SIGN_AGE_MAX else [[2.0, 0.0]], dtype=np.float32)


class RoadTypeReader:
  """The road-type network step by step (one step every ROAD_TYPE_PERIOD): the frame, the speed, how long ago a
  motorway or expressway sign was confirmed, the car's motion and, for a network with the input board, how long ago a
  green board towards a motorway was confirmed (BOARD_SIGNS) in, the probabilities out; each memory output
  (<name>_out) goes back as the input <name> of the next step. road_type is the class once ROAD_TYPE_DEBOUNCE steps in a
  row agree, None before; zone30 only at ZONE_CONFIDENCE, else town. The junction output is not used: it is not good
  enough yet."""
  def __init__(self, network, classes: list[str], width: int, height: int):
    self.network = network
    self.classes = classes
    self.image = RoadTypeInput(width, height)
    self.memory: dict[str, np.ndarray] = {}
    self.answers: list[str] = []
    self.road_type: str | None = None
    self.last_step = -math.inf

  def due(self, now: float) -> bool:
    return now - self.last_step >= ROAD_TYPE_PERIOD

  def step(self, now: float, y: np.ndarray, uv: np.ndarray, v_ego: float | None, sign_age: float,
           motion: np.ndarray, board_age: float = math.inf) -> str | None:
    if now - self.last_step > ROAD_TYPE_RESET_SECONDS:
      self.memory, self.answers, self.road_type = {}, [], None
    self.last_step = now
    speed = [[min(max(v_ego, 0.0), 60.0) / 30.0, 1.0]] if v_ego is not None else [[0.0, 0.0]]
    inputs = {"image": self.image(y, uv), "speed": np.array(speed, dtype=np.float32), "sign": age_input(sign_age),
              "motion": motion, "board": age_input(board_age)}
    names = self.network.input_names
    for name in names:
      if name not in inputs:  # a memory: zeros at the start
        inputs[name] = self.memory.get(name, np.zeros(self.network.input_shape(name), dtype=np.float32))
    outputs = self.network(**{name: value for name, value in inputs.items() if name in names})
    self.memory = {name[:-len("_out")]: value for name, value in outputs.items() if name.endswith("_out")}
    probabilities = np.asarray(outputs["probabilities"]).reshape(-1)
    index = int(np.argmax(probabilities))
    answer = self.classes[index]
    if answer == "zone30" and probabilities[index] < ZONE_CONFIDENCE:
      answer = "town"
    self.answers = (self.answers + [answer])[-ROAD_TYPE_DEBOUNCE:]
    if len(self.answers) == ROAD_TYPE_DEBOUNCE and len(set(self.answers)) == 1:
      self.road_type = self.answers[0]
    return self.road_type


def signnet_boxes(output: np.ndarray, scale: float, limit: int = MAX_BOXES, min_score: float = MIN_BOX_SCORE) -> list[dict]:
  """SignNet's boxes (frame pixels, the best first): the local maxima (3 x 3) of the heat map of its output
  (1, 5, H, W: heat, log width, log height, offset x, offset y) of at least min_score, at most limit."""
  heat, shape = output[0, 0], output[0, 1:5]
  padded = np.pad(heat, 1, constant_values=-np.inf)
  height, width = heat.shape
  pooled = np.max([padded[dy:dy + height, dx:dx + width] for dy in range(3) for dx in range(3)], axis=0)
  peaks = np.where(pooled == heat, heat, 0.0).ravel()
  boxes = []
  for index in np.argsort(-peaks, kind="stable")[:limit]:
    score = float(peaks[index])
    if score < min_score:
      break
    iy, ix = divmod(int(index), width)
    log_w, log_h, ox, oy = (float(value) for value in shape[:, iy, ix])
    w, h = math.exp(min(log_w, 8.0)), math.exp(min(log_h, 8.0))
    cx, cy = (ix + ox) * SIGNNET_STRIDE, (iy + oy) * SIGNNET_STRIDE
    box = [(cx - w / 2) * scale, (cy - h / 2) * scale, (cx + w / 2) * scale, (cy + h / 2) * scale]
    boxes.append({"box": box, "score": score})
  return boxes


def crop_rectangle(box: list[float], width: int, height: int) -> tuple[int, int, int, int]:
  """The pixels of a box to cut from a frame of width x height: rounded outwards, inside the frame, at least
  MIN_CROP_SIDE pixels on each side (nnslr_tools.training.crop_rectangle)."""
  x1, y1, x2, y2 = (float(value) for value in box)
  left, top = math.floor(min(x1, x2)), math.floor(min(y1, y2))
  right, bottom = math.ceil(max(x1, x2)), math.ceil(max(y1, y2))

  def widen(low: int, high: int, limit: int) -> tuple[int, int]:
    side = min(MIN_CROP_SIDE, limit)
    if high - low < side:
      middle = (low + high) / 2
      low = int(math.floor(middle - side / 2))
      high = low + side
    low = max(0, min(low, limit - side))
    high = min(limit, max(high, low + side))
    return low, high

  left, right = widen(max(0, left), min(int(width), right), int(width))
  top, bottom = widen(max(0, top), min(int(height), bottom), int(height))
  return left, top, right, bottom


def classifier_input(y: np.ndarray, uv: np.ndarray, box: list[float],
                     full: tuple[np.ndarray, np.ndarray] | None = None) -> np.ndarray:
  """The classifier's input for a box: the box cut from the full frame, resized to CLASSIFIER_SIZE and normalised,
  float (1, 3, CLASSIFIER_SIZE, CLASSIFIER_SIZE). full: the planes of the same frame at a higher definition, a whole
  multiple of y's: the same window of the scene, cut from their pixels."""
  left, top, right, bottom = crop_rectangle(box, y.shape[1], y.shape[0])
  if full is not None:
    scale = full[0].shape[1] // y.shape[1]
    left, top, right, bottom = left * scale, top * scale, right * scale, bottom * scale
    y, uv = full
  crop = resize(yuv_to_rgb(y, uv, top, bottom, left, right), CLASSIFIER_SIZE, CLASSIFIER_SIZE)
  pixels = crop.astype(np.float32) / 255.0
  return ((pixels - CLASSIFIER_MEAN) / CLASSIFIER_STD)[None]


def softmax(logits: np.ndarray) -> np.ndarray:
  exp = np.exp(logits - logits.max())
  return exp / exp.sum()


# The unit of the numbers on the signs, independent of the legal implicit defaults: mph in the UK, the US and the places
# that follow them (crown dependencies, territories, part of the Caribbean, Liberia, Samoa), km/h everywhere else.
MPH_COUNTRIES = frozenset("GB JE GG IM US PR GU VI AS MP AG AI BS BZ DM FK GD KN KY LC MS SH TC VC VG LR WS".split())


def sign_unit(country: str | None) -> str:
  """'mph' or 'km/h' for an ISO country code, 'unknown' without one (its numbers are not taken)."""
  return "unknown" if not country else "mph" if country in MPH_COUNTRIES else "km/h"


def sign_mps(number: float, unit: str) -> float | None:
  if type(number) not in (int, float) or not math.isfinite(number) or number < 0:
    return None
  if number == 0:  # cancellation does not depend on a numeric unit
    return 0.0
  return number / 3.6 if unit == "km/h" else number * 0.44704 if unit == "mph" else None


def class_speed(name: str) -> float | None:
  """The written number a class gives (unit-free): its number for a speed sign (with hours or not), a variable display, a
  zone or a marking on the road; 0 for the end of a limit; None for the others (other signs, unreadable, not a sign). A
  number with hours may end with them: maximum_speed_hours:30:22-6 gives 30."""
  if name == CANCELLATION:
    return 0.0
  kind, _, value = name.partition(":")
  value = value.partition(":")[0]
  if kind in SPEED_KINDS and value.isdigit():
    return float(value)
  return None


def class_plate(name: str) -> str | None:
  """The plate a class says is below its number: "time" for a number with hours, one of brain_inputs.PLATES; "wet" for one
  valid on wet roads, brain_inputs.SignInputs.plate_wet_row; None when it does not say."""
  return {MAXIMUM_SPEED_HOURS: "time", MAXIMUM_SPEED_WET: "wet"}.get(name.partition(":")[0])


def class_hours(name: str) -> tuple[int, int] | None:
  """The hours a class says are on its plate, from and to: (22, 6) for maximum_speed_hours:30:22-6; None when it does not
  say them. No rule reads them: they go to the brain as they are."""
  kind, _, value = name.partition(":")
  hours = value.partition(":")[2]
  if kind != MAXIMUM_SPEED_HOURS or not hours:
    return None
  start, _, end = hours.partition("-")
  return int(start), int(end)


def read_signs(y: np.ndarray, uv: np.ndarray, signnet_input: SignNetInput, finder, classifier,
               classes: list[str], full: tuple[np.ndarray, np.ndarray] | None = None) -> list[dict]:
  """The signs of a frame: SignNet's boxes, each read by the classifier (its class, how sure, and the speed it gives
  when it is sure enough: written number, 0 for the end of a limit, None for no speed). finder and classifier take the
  networks' inputs and give their outputs as numpy arrays. full: the frame's planes at a higher definition, where the
  classifier's crops come from (classifier_input); SignNet stays on y and uv."""
  signs = signnet_boxes(finder(signnet_input(y, uv)), signnet_input.scale)
  for sign in signs:
    probabilities = softmax(classifier(classifier_input(y, uv, sign["box"], full)).reshape(-1))
    index = int(np.argmax(probabilities))
    sign["class"], sign["confidence"] = classes[index], float(probabilities[index])
    sign["speed"] = class_speed(classes[index]) if sign["confidence"] >= MIN_CONFIDENCE else None
  return signs


def raw_reading(signs: list[dict]) -> dict | None:
  """The sign of a frame (read_signs) the memory can take, a speed, the end of a limit or a sign of the kind of road,
  read with the highest confidence, sure enough or not; None without one. For the logs: a wrong reading shows even
  when it was dropped."""
  best = None
  for sign in signs:
    if (best is None or sign["confidence"] > best["confidence"]) and \
        (sign["class"] in ROAD_SIGNS or class_speed(sign["class"]) is not None):
      best = sign
  return best


class SpeedLimitMemory:
  """The physical limit held (m/s, 0: unavailable). New written numbers are normalized before confirmation.
  A number becomes the limit when it is read in CONFIRM_FRAMES frames within CONFIRM_SECONDS, and stays until another
  one is. A sign of the kind of road (ROAD_SIGNS), confirmed the same way,
  sets the general limit of that road (IMPLICIT_LIMITS of the country; where ROAD_SIGN_KEEPS_LIMIT, only when no limit
  is held); so does the end of a limit, on a road whose kind is known (else 0, no limit). The camera and the car only:
  the map is another source, which the speed-limit resolver weighs against this one.

  The limit of a ramp ends on the main carriageway, where no sign ends it: a number read on a ramp or within
  RAMP_SECONDS of a motorway or expressway sign (not on an acceleration lane or past one since that sign: there it is
  the carriageway's) is taken as the ramp's, and from the acceleration lane (the road-type
  network's merge, the carriageway known from its sign) or once the network sees the carriageway the general limit of
  the road comes back, so the car can speed up to join the traffic. A number read off the carriageways ends the same
  way once the network has seen the carriageway for a while (CARRIAGEWAY_LAG): its end sign or the ramp was missed.

  Where no sign gives the limit, the scene does (the road-type network): a car park gives its limit (GARAGE_LIMIT
  underground or indoors, PARKING_LIMIT in the open; another kind seen for PARKING_SECONDS inside, its limit; a sign
  there only when lower) and drops the limit of the road before; leaving it (a road seen for PARKING_EXIT_SECONDS), or with
  nothing held (the start of a drive), the road type seen gives its general limit (a town: 50), which follows another
  road type only after ROAD_TYPE_SECONDS.

  For the logs, and nothing else: the rule that set the limit held (limit_reason, LIMIT_REASONS), what the limit says
  (limit_state, LIMIT_STATES) and the last raw reading given to update (raw_reading) with whether the limit was set
  from it in its frame (raw_class, raw_speed, raw_confidence, raw_taken) and the road the applicability network gave
  it (raw_road); fill_log writes them in speedVisionSP."""
  def __init__(self, country: str | None = DEFAULT_COUNTRY):
    self.speed_limit = 0.0
    self.country = country
    self._confirmed_country = country
    self._explicit_number = False
    self.road: str | None = None  # the kind of road from the last road sign or the road-type network
    self._seen: list[tuple[float, float | str]] = []  # observations are physical m/s, or a nonnumeric sign kind
    self._road_sign_at = -math.inf  # when the last motorway or expressway sign was confirmed
    self._board_at = -math.inf  # when the last green board towards a motorway was confirmed (BOARD_SIGNS)
    self._ramp_limit = False  # the limit held is a ramp's
    self._road_sign_kind: str | None = None  # motorway or expressway: the last of their signs confirmed
    self._ramp_road: str | None = None  # the carriageway the ramp joins, known from its sign; None: not known
    self._joined_at = -math.inf  # when the car was last on an acceleration lane or past a ramp's limit
    self._number_at = -math.inf  # when the number held was read off the carriageways; -inf: on one, or not known
    self._merge_since: float | None = None
    self.sign_read = False  # a number or a road sign was confirmed in the last update (the limit held again too)
    # What gives the limit held: "sign", "road" (the road type seen), "parking", None (no limit).
    self.source: str | None = None
    self._road_type: str | None = None  # the road type of the last update, since _road_type_since
    self._road_type_since = -math.inf
    self._park: str | None = None  # the kind of car park seen below PARKING_MAX_SPEED, since _parking_since
    self._parking_since: float | None = None
    self._held_park: str | None = None  # the kind of car park whose limit is held
    # The road-type network tells a garage from a car park in the open (it has the class garage): else its parking
    # may be either and gives GARAGE_LIMIT. speedvisiond sets it from the network's classes.
    self.garage_known = False
    self._sign_at = -math.inf  # when a number or a road sign was last confirmed
    self.limit_reason = ""  # the rule that set the limit held (LIMIT_REASONS), "" before the first
    self.limit_state = "unknown"  # what the limit says (LIMIT_STATES)
    self._taken: float | str | None = None  # what set the limit in this update: a number, a road sign's kind; None else
    # The last raw reading: class, physical km/h (raw_speed, for old logs), original number/unit/country,
    # confidence and whether it set the limit in its frame. Unknown units give raw_speed 0, never a km/h guess.
    self.raw_class = ""
    self.raw_speed = 0.0
    self.raw_confidence = 0.0
    self.raw_taken = False
    self.raw_value = 0.0
    self.raw_unit = "unknown"
    self.raw_country = ""
    self.raw_road = ""  # the applicability network's road of the last raw reading, "" when it did not rule it

  def set_country(self, country: str | None) -> None:
    """Only a confirmed new country invalidates memory. GPS loss blocks observations but holds m/s.
    The last confirmed country survives GPS loss, so a later crossing cannot reuse its old memory."""
    if country == self.country:
      return
    self._seen.clear()  # never confirm candidates accumulated under another unit/unknown interval
    self.country = country
    if country is None:
      return
    previous = self._confirmed_country
    self._confirmed_country = country
    if previous is None or previous == country:
      return
    parking = self.source == "parking"
    self._ramp_limit, self._ramp_road, self._road_sign_kind = False, None, None
    self._road_sign_at = self._board_at = self._joined_at = self._number_at = -math.inf
    self._merge_since = None
    self._explicit_number = False
    if not parking:
      # Re-evaluate legal defaults, but never reinterpret the old written number.
      self._set(self._general(self.road), "road", "country_change")

  def _confirmed(self, now: float, values: list) -> list:
    self._seen = [(t, value) for t, value in self._seen if now - t <= CONFIRM_SECONDS]
    self._seen += [(now, value) for value in set(values)]
    counts: dict = {}
    for _, value in self._seen:
      counts[value] = counts.get(value, 0) + 1
    confirmed = [value for value in set(values) if counts[value] >= CONFIRM_FRAMES]
    # Two at once (a gantry, a side road, roadworks): the one seen most, the lower limit on a tie; the end of a limit
    # (0) gives the road's general limit, never lower than a number, so a number wins the tie.
    return sorted(confirmed, key=lambda value: (-counts[value], (value or math.inf) if isinstance(value, float) else 0.0))

  def _general(self, road: str | None) -> float:
    if road == "zone30":
      return ZONE_LIMIT / 3.6 if self.country in IMPLICIT_LIMITS else 0.0
    limit = IMPLICIT_LIMITS.get(self.country, {}).get(road) if road else None
    return float(limit or 0) / 3.6

  def _park_limit(self, park: str | None) -> float:
    # Release settings and legacy constants are km/h physical values, even in mph jurisdictions.
    return (PARKING_LIMIT if park == "parking" and self.garage_known else GARAGE_LIMIT) / 3.6

  def road_sign_age(self, now: float) -> float:
    """Seconds since a motorway or expressway sign was confirmed (inf before the first): the road-type network's input."""
    return now - self._road_sign_at

  def board_age(self, now: float) -> float:
    """Seconds since a green board towards a motorway was confirmed (inf before the first): the road-type network's
    input board, nothing else."""
    return now - self._board_at

  def _set(self, limit: float, source: str | None, reason: str, taken: float | str | None = None) -> None:
    """taken: the confirmed number (0: the end of a limit) or road sign's kind the limit is set from, None for the rules
    that read no sign."""
    self._explicit_number = reason == "sign" and isinstance(taken, float) and taken > 0
    source = source if limit > 0 else None
    # 0 on a road whose kind is known (the road the general limit was taken from): it has none. Else nothing gives one.
    state = "limit" if limit > 0 else "none" if self.road in IMPLICIT_LIMITS.get(self.country, {}) else "unknown"
    # The same limit of the same source and state set again by a rule that reads no sign (the scene holding a 30 zone's
    # 30, the German motorway's none) keeps the rule that set it.
    if taken is not None or (limit, source, state) != (self.speed_limit, self.source, self.limit_state) or \
        not self.limit_reason:
      self.limit_reason = reason
    self.speed_limit, self.source, self.limit_state, self._taken = limit, source, state, taken

  def update(self, now: float, readings: list[float], signs: list[str] | tuple[str, ...] = (),
             road_type: str | None = None, v_ego: float | None = None, raw: dict | None = None) -> float:
    """readings: the original written numbers in this frame (class_speed), normalized here before confirmation;
    signs: the classes of the other signs read sure enough;
    road_type: what the road-type network sees (town, rural, expressway, motorway, ramp, parking, zone30, garage,
    merge: the acceleration lane after an entry ramp), None without it;
    v_ego: the car's speed (m/s), None when unknown (then no car park);
    raw: the frame's raw reading (raw_reading), None without one: only kept for the logs (raw_*), the limit never
    reads it."""
    self._taken = None
    normalized = [speed for value in readings if (speed := sign_mps(value, sign_unit(self.country))) is not None]
    limit = self._update(now, normalized, signs, road_type, v_ego)
    if raw is not None:
      self.raw_taken = self._taken == self.note_raw(raw) and self._taken is not None and self.raw_road != OTHER_ROAD
    return limit

  def note_raw(self, raw: dict) -> float | str | None:
    """A frame's raw reading (raw_reading) kept for the logs; what the limit would be set from if it were taken."""
    speed = class_speed(raw["class"])
    unit = sign_unit(self.country)
    physical = sign_mps(speed, unit) if speed is not None else None
    self.raw_class, self.raw_confidence = raw["class"], float(raw["confidence"])
    self.raw_value, self.raw_unit, self.raw_country = speed or 0.0, unit, self.country or ""
    self.raw_speed = (speed or 0.0) if unit == "km/h" else physical * 3.6 if physical is not None else 0.0
    self.raw_road = raw.get("road", "")
    return ROAD_SIGNS.get(raw["class"]) if speed is None else physical

  def update_frame(self, now: float, signs: list[dict], road_type: str | None = None,
                   v_ego: float | None = None, seen: list[dict] | None = None) -> float:
    """update with a frame's signs as read_signs gives them, as speedvisiond: the ones read sure enough (MIN_CONFIDENCE)
    are its readings and signs, the surest of all its raw reading. seen: all the signs of the frame when signs are only
    the ones taken (applicability.taken), for the raw reading: the logs show a sign of another road too."""
    sure = [sign for sign in signs if sign["confidence"] >= MIN_CONFIDENCE]
    return self.update(now, [sign["speed"] for sign in sure if sign["speed"] is not None],
                       [sign["class"] for sign in sure], road_type=road_type, v_ego=v_ego,
                       raw=raw_reading(signs if seen is None else seen))

  def fill_log(self, message) -> None:
    """speedVisionSP's fields for the logs (cereal/custom.capnp) on its message: what the limit says, the rule that set
    it, the last raw reading, whether the limit was set from it and the road the applicability network gave it."""
    message.speedLimitState = self.limit_state
    message.speedLimitReason = self.limit_reason
    message.rawSignClass = self.raw_class
    message.rawSignSpeed = self.raw_speed
    message.rawSignConfidence = self.raw_confidence
    message.rawSignTaken = self.raw_taken
    message.rawSignValue = self.raw_value
    message.rawSignUnit = self.raw_unit
    message.rawSignCountry = self.raw_country
    message.rawSignRoad = self.raw_road

  def _update(self, now: float, readings: list[float], signs: list[str] | tuple[str, ...], road_type: str | None,
              v_ego: float | None) -> float:
    roads = [ROAD_SIGNS[name] for name in signs if name in ROAD_SIGNS]
    boards = [name for name in signs if name in BOARD_SIGNS]
    confirmed = self._confirmed(now, [float(value) for value in readings] + roads + boards)
    if any(value in BOARD_SIGNS for value in confirmed):
      self._board_at = now  # the road-type network's input only
    confirmed = [value for value in confirmed if value not in BOARD_SIGNS]
    numbers = [value for value in confirmed if isinstance(value, float)]
    road_signs = [value for value in confirmed if isinstance(value, str)]
    self.sign_read = bool(confirmed)
    if confirmed:
      self._sign_at = now
    if road_type != self._road_type:
      if not (self._road_type == "merge" and road_type in CARRIAGEWAYS):
        self._road_type_since = now  # the lane is the carriageway's first metres: its time counts for it
      self._road_type = road_type
    if road_type == "merge":
      self._joined_at = now
    slow = v_ego is not None and v_ego < PARKING_MAX_SPEED
    park = road_type if road_type in CAR_PARKS and slow else None
    if park != self._park:
      self._park, self._parking_since = park, now if park else None
    parking = park is not None and now - self._parking_since >= PARKING_SECONDS

    left = road_type in ROAD_KINDS and now - self._road_type_since >= PARKING_EXIT_SECONDS
    if parking or self.source == "parking" and not left:
      # In a car park: its limit (another kind seen long enough inside: its limit), a sign only when lower; the road
      # before is dropped.
      if parking and (self.source != "parking" or self._park_limit(park) != self._park_limit(self._held_park)):
        self._held_park = park
        self._set(self._park_limit(park), "parking", "car_park")
        self.road, self._ramp_limit, self._merge_since, self._number_at = None, False, None, -math.inf
      if numbers and 0 < numbers[0] < self.speed_limit:
        self._set(numbers[0], "parking", "sign", numbers[0])
      return self.speed_limit

    if road_signs:
      self.road = road_signs[0]
      if self.country not in ROAD_SIGN_KEEPS_LIMIT or self.speed_limit <= 0:
        self._set(self._general(self.road), "sign", "road_sign", self.road)
      self._ramp_limit = False
      self._number_at = -math.inf
      if self.road in CARRIAGEWAYS:
        self._road_sign_at, self._road_sign_kind = now, self.road
      else:
        self._road_sign_kind = None  # its carriageway has ended (its end sign, a town)
    if road_type in ROAD_KINDS and not road_signs:
      self.road = road_type
      if road_type in CARRIAGEWAYS and now - max(self._road_type_since, self._road_sign_at) >= ROAD_TYPE_SECONDS:
        # On a carriageway for a while since its sign: a later ramp may join the other kind. Not in the first seconds,
        # when the road type is still the one of before the sign (the debounce; an expressway's link to a motorway).
        self._road_sign_kind = None
    if numbers:
      value = numbers[0]
      if value == 0:
        # The end of a limit in a 30 zone is its exit (the network still sees the zone for a few seconds): the town.
        if self.road == "zone30":
          self.road = "town"
        self._set(self._general(self.road), "sign", "end_sign", value)
        self._ramp_limit = False
        self._number_at = -math.inf
      else:
        self._set(value, "sign", "sign", value)
        off = road_type is not None and road_type not in CARRIAGEWAYS and road_type != "merge"
        self._number_at = now if off else -math.inf
        near = now - self._road_sign_at <= RAMP_SECONDS and self._joined_at < self._road_sign_at
        self._ramp_limit = road_type == "ramp" or near
        self._ramp_road = self._road_sign_kind if near else None
      self._merge_since = None
      return self.speed_limit

    if self.country is None:
      return self.speed_limit

    if road_type in CARRIAGEWAYS and not road_signs and self._number_at > -math.inf and \
        self._road_type_since - self._number_at >= CARRIAGEWAY_LAG and now - self._road_type_since >= ROAD_TYPE_SECONDS:
      self.road = road_type
      self._set(self._general(road_type), "sign", "carriageway")  # the carriageway's own limit: until the next sign
      self._ramp_limit, self._merge_since, self._number_at = False, None, -math.inf
      self._joined_at = now
      return self.speed_limit

    # The scene where no sign gives the limit: out of a car park or with nothing held, the road type's general limit
    # (also where it has none: the motorway in Germany, 0); a limit of the scene follows a new road type held long enough.
    if road_type in ROAD_KINDS and not self._ramp_limit and not road_signs and (
        self.source in (None, "parking") or self.source == "road" and now - self._road_type_since >= ROAD_TYPE_SECONDS):
      self._set(self._general(road_type), "road", "road_type")
      return self.speed_limit
    if road_type == "zone30" and self.source == "sign" and not self._ramp_limit and \
        not self._explicit_number and self.speed_limit == self._general("town") and now - max(self._road_type_since, self._sign_at) >= ROAD_TYPE_SECONDS:
      self._set(self._general("zone30"), "road", "zone30")
      return self.speed_limit

    if self._ramp_limit:
      merged = road_type == "merge" or road_type in CARRIAGEWAYS
      if not merged:
        self._merge_since = None
      elif self._merge_since is None:
        self._merge_since = now
      if road_type == "merge" and self._ramp_road:
        carriageway = self._ramp_road  # the acceleration lane of a known carriageway: its limit at once
      elif road_type in CARRIAGEWAYS and now - self._merge_since >= MERGE_SECONDS:
        carriageway = road_type
      else:
        return self.speed_limit
      self.road = carriageway
      self._set(self._general(carriageway), "sign", "ramp_end")  # the road's own limit after the ramp's sign: until the next sign
      self._ramp_limit = False
      self._number_at = -math.inf
      self._merge_since = None
      self._joined_at = now
    return self.speed_limit


def _lit_display(name: str) -> bool:
  """A gantry's LED limit showing a number (variable_display:unknown is off, or not readable)."""
  kind, _, value = name.partition(":")
  return kind == VARIABLE_DISPLAY and value.isdigit()


def _limit_from_sign(reason: str, sign_class: str) -> bool:
  """Whether the limit of the rule reason comes from a sign of the kind sign_class (SignDistance's)."""
  kind = sign_class.partition(":")[0]
  return (reason == "sign" and kind in ("maximum_speed", *PLATED_KINDS, VARIABLE_DISPLAY)) or \
    (reason == "end_sign" and kind == "cancellation") or (reason == "road_sign" and sign_class in ROAD_SIGN_HEIGHTS)


class SignDistance:
  """The distance (m) to the sign ahead the classifier reads sure enough (ROUND_SIGNS, a gantry's LED limit,
  ROAD_SIGN_HEIGHTS; the nearest of a frame), 0 without one or with a focal length of 0: see SIGN_DIAMETERS. The camera
  and the car's speed only, never the map."""
  def __init__(self, focal_length: float):
    self.focal_length = focal_length
    self.odometer = 0.0  # metres driven since the start
    self._speed: tuple[float, float] | None = None  # the last carState: its time (s), vEgo (m/s)
    self.sign_class = ""  # the sign of the distance, kept after it is passed
    self._sightings: list[tuple[float, float]] = []  # the sign's: odometer, side (frame pixels)
    self._seen_at = -math.inf
    self._distance = 0.0  # at the last sighting
    self.side = 0.0  # the side (frame pixels) of the sign seen in the last update, 0 when none

  def add_speed(self, t: float, v_ego: float) -> None:
    """A carState: its time (s, logMonoTime) and vEgo (m/s)."""
    if self._speed is not None:
      last_t, last_v = self._speed
      if 0 < t - last_t <= ODOMETER_MAX_STEP:
        self.odometer += max(0.0, last_v) * (t - last_t)
    self._speed = (t, v_ego)

  def _measured(self, sign: dict, road: str | None) -> tuple[float, float, float, float, str]:
    """A sign of the frame: the distance its size gives, its side (frame pixels), its size (m), how loosely the size is
    held, its class. Seen from the side a round sign is narrower, never lower; a road sign's height stays; a number with
    a plate is taller by it, so its width."""
    x1, y1, x2, y2 = sign["box"]
    if sign["class"] in ROAD_SIGN_HEIGHTS:
      side, size, spread = y2 - y1, ROAD_SIGN_HEIGHTS[sign["class"]], ROAD_SIGN_SPREAD
    elif sign["class"].partition(":")[0] == VARIABLE_DISPLAY:
      side, size, spread = max(x2 - x1, y2 - y1), VARIABLE_DISPLAY_DIAMETER, DISTANCE_DIAMETER_SPREAD
    else:
      side = x2 - x1 if sign["class"].partition(":")[0] in PLATED_KINDS else max(x2 - x1, y2 - y1)
      size, spread = SIGN_DIAMETERS.get(road or "", DEFAULT_SIGN_DIAMETER), DISTANCE_DIAMETER_SPREAD
    return size / max(side, 1e-6), side, size, spread, sign["class"]

  def _fitted(self, diameter: float, spread: float) -> float:
    """The distance (m) at the last sighting: 1 / s = a - b x (x the metres driven, b = 1 / (f D)), least squares
    weighted by the side's noise, b drawn to the sign's size within about spread of it."""
    x = np.array([odometer for odometer, _ in self._sightings])
    y = 1.0 / np.array([side for _, side in self._sightings])
    w = 1.0 / (DISTANCE_SIDE_NOISE * y) ** 2
    b0 = 1.0 / (self.focal_length * diameter)
    pull = 1.0 / (spread * b0) ** 2
    sx, sxx = (w * x).sum(), (w * x * x).sum()
    a, b = np.linalg.solve([[w.sum(), -sx], [-sx, sxx + pull]], [(w * y).sum(), pull * b0 - (w * x * y).sum()])
    if b < b0 / 4:  # sides that do not grow (a parked car's sign, noise): the sign's size alone
      return self.focal_length * diameter / self._sightings[-1][1]
    return float((a - b * x[-1]) / b)

  def update(self, now: float, signs: list[dict], road: str | None) -> float:
    """signs: the frame's (read_signs); road: the kind of road the memory knows (SpeedLimitMemory.road)."""
    seen = [self._measured(sign, road) for sign in signs if sign["confidence"] >= MIN_CONFIDENCE and
            (sign["class"].partition(":")[0] in ROUND_SIGNS or sign["class"] in ROAD_SIGN_HEIGHTS or
             _lit_display(sign["class"]))]
    self.side = 0.0
    if seen and self.focal_length > 0:  # 0: a camera not known
      _, side, size, spread, name = min(seen)  # the nearest: a road sign is larger than a round one
      if name != self.sign_class or now - self._seen_at > TRACK_SECONDS or not self._sightings or \
          side < TRACK_SHRINK * self._sightings[-1][1]:
        self._sightings = []
      self.sign_class, self._seen_at, self.side = name, now, side
      self._sightings = self._sightings[-(TRACK_SIGHTINGS - 1):] + [(self.odometer, side)]
      self._distance = self._fitted(size, spread)
    if now - self._seen_at > DISTANCE_SECONDS:
      return 0.0
    speed = max(0.0, self._speed[1]) if self._speed is not None else 0.0
    lead = DISTANCE_LEAD_SECONDS * speed + DISTANCE_LEAD_METRES
    return max(0.0, self._distance - (self.odometer - self._sightings[-1][0]) - lead)


class LimitAhead:
  """A limit read on a sign ahead starts at the sign, as the map's does: until the car gets there the limit held stays
  and the new one is the limit ahead (speedVisionSP.speedLimitAhead, speedLimitAheadDistance), for sunnypilot's
  resolver to take it as the map's limit ahead. A limit no sign ahead gives applies at once: another rule, a sign with
  no distance (a town sign, a camera not known), or the first limit, with none held to keep.
  From the distance when the limit changes the metres driven count down: SignDistance moves on to the next sign and
  can show a larger distance than the one left, so it only corrects the count on the same sign, and not upwards."""
  def __init__(self):
    self.limit = 0.0  # the limit held (m/s), speedVisionSP.speedLimit
    self.distance = 0.0  # the metres left to the sign, 0 with no limit ahead
    self._target = 0.0  # the memory's limit, the limit ahead while distance > 0
    self._class = ""  # the sign it was read on
    self._odometer = 0.0  # at the last update

  @property
  def ahead(self) -> float:
    return self._target if self.distance > 0 else 0.0

  def update(self, limit: float, reason: str, sign_distance: float, sign_class: str, odometer: float) -> float:
    """limit, reason: the memory's (SpeedLimitMemory.limit_reason); sign_distance, sign_class, odometer: SignDistance's.
    The limit held."""
    if limit != self._target:
      self._target, self._class = limit, sign_class
      self.distance = sign_distance if self.limit > 0 and _limit_from_sign(reason, sign_class) else 0.0
    else:
      self.distance -= odometer - self._odometer
      if sign_class == self._class and sign_distance <= self.distance + LIMIT_AHEAD_SLACK:
        self.distance = sign_distance
    self._odometer = odometer
    if limit == self.limit or self.distance <= 0:
      self.limit, self.distance = limit, 0.0
    return self.limit
