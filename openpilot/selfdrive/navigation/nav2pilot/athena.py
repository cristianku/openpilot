"""
setNavRoute: comma's setNavDestination with optional waypoints (see waypoints.py).
setMapboxToken: the user's Mapbox public token for navd's route requests (see mapbox.py).
They are what the Nav2Pilot phone apps send, registered on athena's dispatcher when athenad imports this module.
"""
from openpilot.common.params import Params
from openpilot.selfdrive.navigation.nav2pilot.mapbox import MAPBOX_TOKEN_PARAM, valid_mapbox_token
from openpilot.selfdrive.navigation.nav2pilot.waypoints import DESTINATION_PARAM, MAX_WAYPOINTS, parse_waypoints, valid_coordinate
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


@dispatcher.add_method
def setMapboxToken(token: str) -> dict[str, int | str]:
  """Stores a Mapbox public token (pk.); an empty string removes it, and navd goes back to comma's proxy.
  The phone sends it with every destination, so an unchanged token is not written again."""
  params = Params()
  if token == "":
    params.remove(MAPBOX_TOKEN_PARAM)
    return {"success": 1}
  if not valid_mapbox_token(token):
    return {"success": 0, "error": "Invalid Mapbox token: a public token starts with pk."}
  if params.get(MAPBOX_TOKEN_PARAM) != token:
    params.put(MAPBOX_TOKEN_PARAM, token, block=True)
  return {"success": 1}
