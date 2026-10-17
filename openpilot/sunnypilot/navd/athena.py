"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

setNavRoute: comma's setNavDestination with optional waypoints (see waypoints.py). Registered on athena's
dispatcher when athenad imports this module, so it is reachable over athena and over sunnylink alike.
"""
from openpilot.common.params import Params
from openpilot.sunnypilot.navd.waypoints import DESTINATION_PARAM, MAX_WAYPOINTS, parse_waypoints, valid_coordinate
from openpilot.system.athena.rpc import dispatcher


@dispatcher.add_method
def setNavRoute(latitude: float, longitude: float, waypoints: list | None = None) -> dict[str, int | str]:
  """Destination plus waypoints in driving order: [{"latitude": .., "longitude": .., "via": false}, ...]."""
  if not valid_coordinate(latitude, longitude):
    return {"success": 0, "error": "Invalid destination coordinates"}

  parsed = parse_waypoints(waypoints)
  if parsed is None:
    return {"success": 0, "error": f"Invalid waypoints (at most {MAX_WAYPOINTS}, each with latitude, longitude and an optional via flag)"}

  destination: dict = {"latitude": float(latitude), "longitude": float(longitude)}
  if parsed:
    destination["waypoints"] = parsed
  Params().put(DESTINATION_PARAM, destination, block=True)
  return {"success": 1}
