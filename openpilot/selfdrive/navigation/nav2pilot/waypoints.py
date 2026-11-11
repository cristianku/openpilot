"""
Waypoints carried by the NavDestination param, next to comma's latitude and longitude:

  {"latitude": 45.47, "longitude": 9.19, "waypoints": [{"latitude": 45.4, "longitude": 9.1, "via": false}, ...]}

- a stop ("via": false) is a Mapbox waypoint: the route has one leg per stop and announces the arrival there
- a via point ("via": true) only shapes the route, silently: it is how a route alternative picked on the phone is
  followed, Mapbox recomputing the same road through a few of its points

comma's navd reads only latitude and longitude, so the param stays valid for it. No numpy here: athenad imports this.
"""
import math

DESTINATION_PARAM = "NavDestination"

# Mapbox Directions takes up to 25 coordinates; the ego position and the destination are two of them
MAX_WAYPOINTS = 20


def valid_coordinate(latitude, longitude) -> bool:
  """Same range rule as athena's setNavDestination, for numbers only (no bools, nan or inf)."""
  for value in (latitude, longitude):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
      return False
    if not math.isfinite(value):
      return False
  return -90 <= latitude <= 90 and -180 <= longitude <= 180


def parse_waypoints(waypoints) -> list[dict] | None:
  """The waypoints to store, normalized to {"latitude", "longitude", "via"}; None when the list is not valid."""
  if waypoints is None:
    return []
  if not isinstance(waypoints, list) or len(waypoints) > MAX_WAYPOINTS:
    return None

  parsed = []
  for waypoint in waypoints:
    if not isinstance(waypoint, dict):
      return None
    latitude = waypoint.get("latitude")
    longitude = waypoint.get("longitude")
    via = waypoint.get("via", False)
    if not valid_coordinate(latitude, longitude) or not isinstance(via, bool):
      return None
    parsed.append({"latitude": float(latitude), "longitude": float(longitude), "via": via})
  return parsed


def waypoints_from_param(destination) -> tuple[tuple[float, float, bool], ...]:
  """(longitude, latitude, via) of each waypoint in the NavDestination param; () without any or when malformed."""
  if not isinstance(destination, dict):
    return ()
  parsed = parse_waypoints(destination.get("waypoints"))
  if parsed is None:
    return ()
  return tuple((w["longitude"], w["latitude"], w["via"]) for w in parsed)


def param_with_waypoints(destination: dict, waypoints) -> dict:
  """A copy of the NavDestination param with `waypoints` ((longitude, latitude, via) tuples), without the key when empty."""
  updated = {key: value for key, value in destination.items() if key != "waypoints"}
  if waypoints:
    updated["waypoints"] = [{"latitude": latitude, "longitude": longitude, "via": via} for longitude, latitude, via in waypoints]
  return updated
