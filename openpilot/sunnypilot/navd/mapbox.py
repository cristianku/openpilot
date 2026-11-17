"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The user's own Mapbox public token, sent from the phone app with setMapboxToken and kept in the MapboxToken param.
With it navd asks Mapbox for the route directly, instead of comma's /v1/nav proxy. No numpy here: athenad imports this.
"""
import re

MAPBOX_TOKEN_PARAM = "MapboxToken"
MAPBOX_HOST = "https://api.mapbox.com"

# "pk." then the two base64url parts of a public token; secret (sk.) and temporary (tk.) tokens are refused
PUBLIC_TOKEN_PATTERN = re.compile(r"pk\.[A-Za-z0-9_=-]+\.[A-Za-z0-9_=-]+")
MAX_TOKEN_LENGTH = 512


def valid_mapbox_token(token) -> bool:
  if not isinstance(token, str):
    return False
  if len(token) > MAX_TOKEN_LENGTH:
    return False
  return PUBLIC_TOKEN_PATTERN.fullmatch(token) is not None
