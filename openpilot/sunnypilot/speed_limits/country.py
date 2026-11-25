"""Offline GPS country lookup. No speed limits, road names, junctions or map services.

Policy values must be supplied explicitly; none have been validated for driving.
Natural Earth borders are a research asset, with significant boundary error.
"""

import gzip
import hashlib
import json
import math
import zlib
from dataclasses import dataclass
from pathlib import Path


def release_context(config_path=None):
  """Load a self-contained, digest-checked release resource; never fetch at runtime."""
  config_path = Path(config_path) if config_path else Path(__file__).with_name('resources') / 'country-context.json'
  try:
    doc = json.loads(config_path.read_text())
    asset = config_path.parent / doc['asset']
    if asset.resolve().parent != config_path.parent.resolve():
      return None
    if hashlib.sha256(asset.read_bytes()).hexdigest() != doc['sha256']:
      return None
    return CountryResolver.from_settings(OfflineBorders.load(asset), doc['policy'])
  except (OSError, KeyError, TypeError, ValueError):
    return None


@dataclass(frozen=True)
class GpsFix:
  time: float  # event logMonoTime seconds, same monotonic clock as now
  latitude: float
  longitude: float
  accuracy: float  # horizontalAccuracy, metres
  valid: bool


@dataclass(frozen=True)
class CountryPolicy:
  gps_max_age_s: float
  gps_max_accuracy_m: float
  border_margin_m: float
  confirm_s: float
  max_sample_gap_s: float

  def __post_init__(self):
    if any(type(v) not in (int, float) or not math.isfinite(v) or v <= 0 for v in vars(self).values()):
      raise ValueError("country policy requires finite positive experimental values")


def _inside(lon, lat, ring):
  inside = False
  for a, b in zip(ring, ring[1:] + ring[:1], strict=True):
    if (a[1] > lat) != (b[1] > lat) and lon < (b[0] - a[0]) * (lat - a[1]) / (b[1] - a[1]) + a[0]:
      inside = not inside
  return inside


def _distance(lon, lat, a, b):
  # Local tangent approximation, only used to reject positions near a border.
  scale = math.cos(math.radians(lat))
  ax, ay = (a[0] - lon) * scale * 111195, (a[1] - lat) * 111195
  bx, by = (b[0] - lon) * scale * 111195, (b[1] - lat) * 111195
  dx, dy = bx - ax, by - ay
  t = max(0.0, min(1.0, -(ax * dx + ay * dy) / (dx * dx + dy * dy))) if dx or dy else 0.0
  return math.hypot(ax + t * dx, ay + t * dy)


def _bbox(ring):
  lons, lats = [p[0] for p in ring], [p[1] for p in ring]
  return min(lons), min(lats), max(lons), max(lats)


class OfflineBorders:
  """The borders, in tiers: a country of tier 1 (outside Europe, coarser) is asked only where no country of tier 0 is."""
  def __init__(self, countries):
    self.countries = countries
    self._boxes = {}

  def _box(self, polygon):
    box = self._boxes.get(id(polygon))
    if box is None:
      box = self._boxes[id(polygon)] = _bbox(polygon[0])
    return box

  @classmethod
  def load(cls, path=None):
    try:
      path = Path(path) if path is not None else Path(__file__).with_name('country-borders.json.gz')
      data = json.loads(gzip.decompress(path.read_bytes()))
      countries = data['countries']
      if not isinstance(countries, list):
        raise ValueError('countries must be a list')
      for c in countries:
        if not isinstance(c['code'], str) or len(c['code']) != 2 or not c['code'].isupper():
          raise ValueError('invalid country code')
        if len(c['bbox']) != 4 or not all(type(x) in (int, float) and math.isfinite(x) for x in c['bbox']):
          raise ValueError('invalid bbox')
        if c.get('tier', 0) not in (0, 1):
          raise ValueError('invalid tier')
        if not c['polygons']:
          raise ValueError('empty geometry')
        for polygon in c['polygons']:
          if not polygon:
            raise ValueError('empty polygon')
          for ring in polygon:
            if len(ring) < 4 or any(len(p) != 2 or not all(type(v) in (int, float) and math.isfinite(v) for v in p) for p in ring):
              raise ValueError('invalid ring')
      return cls(countries)
    except (OSError, ValueError, KeyError, TypeError, EOFError, zlib.error):
      return cls([])  # unavailable/corrupt asset never defaults to a country

  def country_at(self, lat, lon, margin_m):
    if not all(type(v) in (int, float) and math.isfinite(v) for v in (lat, lon, margin_m)) or not -90 <= lat <= 90 or not -180 <= lon <= 180 or margin_m < 0:
      return None
    for tier in sorted({c.get('tier', 0) for c in self.countries}):
      matches = set()
      for c in self.countries:
        x0, y0, x1, y1 = c['bbox']
        if c.get('tier', 0) != tier or not x0 <= lon <= x1 or not y0 <= lat <= y1:
          continue
        for polygon in c['polygons']:
          px0, py0, px1, py1 = self._box(polygon)
          if not px0 <= lon <= px1 or not py0 <= lat <= py1:
            continue
          if not _inside(lon, lat, polygon[0]) or any(_inside(lon, lat, hole) for hole in polygon[1:]):
            continue
          if any(_distance(lon, lat, a, b) <= margin_m for ring in polygon for a, b in zip(ring, ring[1:] + ring[:1], strict=True)):
            return None
          matches.add(c['code'])
          break
      if matches:
        return matches.pop() if len(matches) == 1 else None
    return None


class CountryResolver:
  def __init__(self, borders, policy):
    self.borders, self.policy = borders, policy
    self.fix = None
    self.candidate = None
    self.since = None
    self.country = None

  @classmethod
  def from_settings(cls, borders, settings):
    try:
      policy = CountryPolicy(**settings)
    except (TypeError, ValueError):
      return None
    return cls(borders, policy) if borders.countries else None

  def update(self, now, fix=None):
    p = self.policy
    if not math.isfinite(now):
      self.country = self.candidate = self.since = None
      return None

    def valid(f):
      return (
        f is not None
        and f.valid
        and all(type(v) in (int, float) and math.isfinite(v) for v in (f.time, f.latitude, f.longitude, f.accuracy))
        and 0 <= now - f.time <= p.gps_max_age_s
        and 0 < f.accuracy <= p.gps_max_accuracy_m
      )

    if fix is not None and not valid(fix):
      self.country = self.candidate = self.since = None
      return None
    new = fix is not None and (self.fix is None or fix.time > self.fix.time)
    f = fix if new else self.fix
    if not valid(f):
      self.country = self.candidate = self.since = None
      return None
    if not new:
      return self.country  # duplicate/old samples cannot advance confirmation
    candidate = self.borders.country_at(f.latitude, f.longitude, p.border_margin_m + f.accuracy)
    gap = f.time - self.fix.time if self.fix is not None else math.inf
    self.fix = f
    if candidate is None:
      self.country = self.candidate = self.since = None
      return None
    if candidate != self.candidate or gap > p.max_sample_gap_s:
      self.candidate, self.since, self.country = candidate, f.time, None
    self.country = candidate if f.time - self.since >= p.confirm_s else None
    return self.country
