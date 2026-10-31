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

# A reading becomes the limit when it is in CONFIRM_FRAMES frames within CONFIRM_SECONDS; it stays until another does.
CONFIRM_FRAMES = 2
CONFIRM_SECONDS = 1.5

SPEED_KINDS = ("maximum_speed", "variable_display", "zone", "road_marking")
CANCELLATION = "cancellation"

# The general limits that hold where no number is signed (km/h; None: no general limit), by country, for the kind of
# road: in a town, outside towns, on an expressway (Autostrasse), on a motorway. From the laws of each country,
# sunnypilot.speedlimits docs/road-rules/<cc>.md (29 Oct 2026). Where a kind's limit depends on what the camera cannot
# tell (a single or a dual carriageway, the region), the lower one. GB is left out: its signs are in mph.
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
DEFAULT_COUNTRY = "CH"
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
ROAD_TYPE_PERIOD = 1.0
ROAD_TYPE_DEBOUNCE = 3
ROAD_TYPE_RESET_SECONDS = 5.0  # longer without a step: its memory starts again from zeros
ROAD_KINDS = ("town", "rural", "expressway", "motorway", "zone30")  # the road types with a general limit
CARRIAGEWAYS = ("motorway", "expressway")
# A 30 zone (the road-type network's zone30, from its next training): 30 in every country. Seen for ROAD_TYPE_SECONDS
# after the last sign, it also lowers a town's general limit held from a sign (its gate sign missed, a 50 read on the
# street before): the car must not speed up to 50 inside it. A higher number (60, 80) holds: the network can be wrong.
ZONE_LIMIT = 30.0
# zone30 is said only this sure (its probability), else town: with Map Data + Vision the resolver takes vision first,
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


def resample_coefficients(in_size: int, out_size: int) -> tuple[np.ndarray, np.ndarray]:
  """PIL's bilinear resampling of in_size pixels to out_size (Resample.c precompute_coeffs and
  normalize_coeffs_8bpc): for each output pixel the input pixels it reads (out_size, taps) and their weights in
  fixed point. Taps of weight 0 are left out where they can be (they add nothing), the others have weight 0."""
  scale = in_size / out_size
  filterscale = max(scale, 1.0)
  support = filterscale  # the bilinear filter's support is 1
  ss = 1.0 / filterscale
  first, weights = [], []
  for xx in range(out_size):
    center = (xx + 0.5) * scale
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


class RoadTypeReader:
  """The road-type network step by step (one step every ROAD_TYPE_PERIOD): the frame, the speed, how long ago a
  motorway or expressway sign was confirmed and the car's motion in, the probabilities out; each memory output
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
           motion: np.ndarray) -> str | None:
    if now - self.last_step > ROAD_TYPE_RESET_SECONDS:
      self.memory, self.answers, self.road_type = {}, [], None
    self.last_step = now
    speed = [[min(max(v_ego, 0.0), 60.0) / 30.0, 1.0]] if v_ego is not None else [[0.0, 0.0]]
    sign = [[min(sign_age, SIGN_AGE_MAX) / 60.0, 1.0]] if sign_age <= SIGN_AGE_MAX else [[2.0, 0.0]]
    inputs = {"image": self.image(y, uv), "speed": np.array(speed, dtype=np.float32),
              "sign": np.array(sign, dtype=np.float32), "motion": motion}
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


def classifier_input(y: np.ndarray, uv: np.ndarray, box: list[float]) -> np.ndarray:
  """The classifier's input for a box: the box cut from the full frame, resized to CLASSIFIER_SIZE and normalised,
  float (1, 3, CLASSIFIER_SIZE, CLASSIFIER_SIZE)."""
  left, top, right, bottom = crop_rectangle(box, y.shape[1], y.shape[0])
  crop = resize(yuv_to_rgb(y, uv, top, bottom, left, right), CLASSIFIER_SIZE, CLASSIFIER_SIZE)
  pixels = crop.astype(np.float32) / 255.0
  return ((pixels - CLASSIFIER_MEAN) / CLASSIFIER_STD)[None]


def softmax(logits: np.ndarray) -> np.ndarray:
  exp = np.exp(logits - logits.max())
  return exp / exp.sum()


def class_speed(name: str) -> float | None:
  """The speed (km/h) a class gives: its number for a speed sign, a variable display, a zone or a marking on the
  road; 0 for the end of a limit; None for the others (other signs, unreadable, not a sign)."""
  if name == CANCELLATION:
    return 0.0
  kind, _, value = name.partition(":")
  if kind in SPEED_KINDS and value.isdigit():
    return float(value)
  return None


def read_signs(y: np.ndarray, uv: np.ndarray, signnet_input: SignNetInput, finder, classifier,
               classes: list[str]) -> list[dict]:
  """The signs of a frame: SignNet's boxes, each read by the classifier (its class, how sure, and the speed it gives
  when it is sure enough: km/h, 0 for the end of a limit, None for no speed). finder and classifier take the
  networks' inputs and give their outputs as numpy arrays."""
  signs = signnet_boxes(finder(signnet_input(y, uv)), signnet_input.scale)
  for sign in signs:
    probabilities = softmax(classifier(classifier_input(y, uv, sign["box"])).reshape(-1))
    index = int(np.argmax(probabilities))
    sign["class"], sign["confidence"] = classes[index], float(probabilities[index])
    sign["speed"] = class_speed(classes[index]) if sign["confidence"] >= MIN_CONFIDENCE else None
  return signs


class SpeedLimitMemory:
  """The limit shown (km/h, 0: none). A number becomes the limit when it is read in CONFIRM_FRAMES frames within
  CONFIRM_SECONDS, and stays until another one is. A sign of the kind of road (ROAD_SIGNS), confirmed the same way,
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
  road type only after ROAD_TYPE_SECONDS."""
  def __init__(self, country: str = DEFAULT_COUNTRY):
    self.speed_limit = 0.0
    self.country = country if country in IMPLICIT_LIMITS else DEFAULT_COUNTRY
    self.road: str | None = None  # the kind of road from the last road sign or the road-type network
    self._seen: list[tuple[float, float | str]] = []
    self._road_sign_at = -math.inf  # when the last motorway or expressway sign was confirmed
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
      return ZONE_LIMIT
    limit = IMPLICIT_LIMITS[self.country].get(road) if road else None
    return float(limit or 0)

  def _park_limit(self, park: str | None) -> float:
    return PARKING_LIMIT if park == "parking" and self.garage_known else GARAGE_LIMIT

  def road_sign_age(self, now: float) -> float:
    """Seconds since a motorway or expressway sign was confirmed (inf before the first): the road-type network's input."""
    return now - self._road_sign_at

  def _set(self, limit: float, source: str | None) -> None:
    self.speed_limit = limit
    self.source = source if limit > 0 else None

  def update(self, now: float, readings: list[float], signs: list[str] | tuple[str, ...] = (),
             road_type: str | None = None, v_ego: float | None = None) -> float:
    """readings: the speeds read in this frame (class_speed); signs: the classes of the other signs read sure enough;
    road_type: what the road-type network sees (town, rural, expressway, motorway, ramp, parking, zone30, garage,
    merge: the acceleration lane after an entry ramp), None without it;
    v_ego: the car's speed (m/s), None when unknown (then no car park)."""
    roads = [ROAD_SIGNS[name] for name in signs if name in ROAD_SIGNS]
    confirmed = self._confirmed(now, [float(value) for value in readings] + roads)
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
        self._set(self._park_limit(park), "parking")
        self.road, self._ramp_limit, self._merge_since, self._number_at = None, False, None, -math.inf
      if numbers and 0 < numbers[0] < self.speed_limit:
        self._set(numbers[0], "parking")
      return self.speed_limit

    if road_signs:
      self.road = road_signs[0]
      if self.country not in ROAD_SIGN_KEEPS_LIMIT or self.speed_limit <= 0:
        self._set(self._general(self.road), "sign")
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
        self._set(self._general(self.road), "sign")
        self._ramp_limit = False
        self._number_at = -math.inf
      else:
        self._set(value, "sign")
        off = road_type is not None and road_type not in CARRIAGEWAYS and road_type != "merge"
        self._number_at = now if off else -math.inf
        near = now - self._road_sign_at <= RAMP_SECONDS and self._joined_at < self._road_sign_at
        self._ramp_limit = road_type == "ramp" or near
        self._ramp_road = self._road_sign_kind if near else None
      self._merge_since = None
      return self.speed_limit

    if road_type in CARRIAGEWAYS and not road_signs and self._number_at > -math.inf and \
        self._road_type_since - self._number_at >= CARRIAGEWAY_LAG and now - self._road_type_since >= ROAD_TYPE_SECONDS:
      self.road = road_type
      self._set(self._general(road_type), "sign")  # the carriageway's own limit: until the next sign
      self._ramp_limit, self._merge_since, self._number_at = False, None, -math.inf
      self._joined_at = now
      return self.speed_limit

    # The scene where no sign gives the limit: out of a car park or with nothing held, the road type's general limit
    # (also where it has none: the motorway in Germany, 0); a limit of the scene follows a new road type held long enough.
    if road_type in ROAD_KINDS and not self._ramp_limit and not road_signs and (
        self.source in (None, "parking") or self.source == "road" and now - self._road_type_since >= ROAD_TYPE_SECONDS):
      self._set(self._general(road_type), "road")
      return self.speed_limit
    if road_type == "zone30" and self.source == "sign" and not self._ramp_limit and \
        self.speed_limit == self._general("town") and now - max(self._road_type_since, self._sign_at) >= ROAD_TYPE_SECONDS:
      self._set(ZONE_LIMIT, "road")
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
      self._set(self._general(carriageway), "sign")  # the road's own limit after the ramp's sign: until the next sign
      self._ramp_limit = False
      self._number_at = -math.inf
      self._merge_since = None
      self._joined_at = now
    return self.speed_limit
