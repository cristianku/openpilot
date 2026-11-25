"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import re

from openpilot.common.params import Params

DESTINATION_PARAM = "NavDestination"


def parse_destination(text: str) -> dict[str, float] | None:
  """The NavDestination for "latitude, longitude" (comma or space separated), or None when the text isn't one."""
  parts = re.split(r"[\s,;]+", text.strip())
  if len(parts) != 2:
    return None

  try:
    latitude = float(parts[0])
    longitude = float(parts[1])
  except ValueError:
    return None

  # same range check as athena's setNavDestination; also rejects nan and inf
  if not (-90 <= latitude <= 90 and -180 <= longitude <= 180):
    return None

  return {"latitude": latitude, "longitude": longitude}


def format_destination(destination) -> str:
  """The NavDestination param as "latitude, longitude", or "" when unset or malformed."""
  if not isinstance(destination, dict):
    return ""

  latitude = destination.get("latitude")
  longitude = destination.get("longitude")
  if not isinstance(latitude, (int, float)) or not isinstance(longitude, (int, float)):
    return ""

  return f"{latitude:.6f}, {longitude:.6f}"


def set_destination_from_text(text: str, params: Params) -> bool:
  """Store the typed destination, or clear it for empty text. False, leaving the param untouched, when invalid."""
  if not text.strip():
    params.remove(DESTINATION_PARAM)
    return True

  destination = parse_destination(text)
  if destination is None:
    return False

  params.put(DESTINATION_PARAM, destination, block=True)
  return True
