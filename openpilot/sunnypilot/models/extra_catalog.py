"""
Extra driving model catalog, merged into sunnypilot's qcom catalog so that its models show up in the
Models menu and on sunnylink like the official ones.

Added by openpilot_scripts (OP-setup-psa-torque*, sunny variant). The catalog is published on Hugging Face
by openpilot.distill helpers/publish_model.sh, in the same layout as sunnypilot's driving_models_vNN.json.
"""

import requests
from requests.exceptions import RequestException

from openpilot.common.swaglog import cloudlog

EXTRA_MODEL_URL = "https://huggingface.co/cristianku/openpilot-distill-models/resolve/main/driving_models.json"
# Marks the merged bundles inside the cached catalog (ModelManager_ModelsCache); the parser ignores it.
EXTRA_KEY = "extra_catalog"


def fetch_extra_bundles() -> list[dict] | None:
  """Bundles of the extra catalog, or None when it cannot be read (offline, 404, bad json)."""
  try:
    response = requests.get(EXTRA_MODEL_URL, timeout=10)
    response.raise_for_status()
    json_data = response.json()
  except (RequestException, ValueError) as e:
    cloudlog.warning(f"Extra models catalog not read: {e}")
    return None

  if not isinstance(json_data, dict) or not isinstance(json_data.get("bundles"), list):
    cloudlog.warning("Extra models catalog has no bundles list")
    return None
  return json_data["bundles"]


def merge_extra_bundles(json_data: dict, extra_bundles: list[dict] | None, previous_cache: dict | None, is_valid) -> dict:
  """The official catalog plus the extra bundles.

  When the extra catalog could not be read (extra_bundles None), the extra bundles of the previous cache are
  kept: a selected model that disappears from the list is reset to Default by the validation.
  A bundle is skipped when its ref is already in the official catalog, when it is a big model, or when
  is_valid(bundle) is False: one bad bundle would make the whole catalog unparsable.
  """
  if extra_bundles is None:
    extra_bundles = []
    if isinstance(previous_cache, dict):
      for bundle in previous_cache.get("bundles", []):
        if isinstance(bundle, dict) and bundle.get(EXTRA_KEY) is True:
          extra_bundles.append(bundle)

  official_refs = set()
  for bundle in json_data.get("bundles", []):
    if isinstance(bundle, dict):
      official_refs.add(bundle.get("ref"))

  merged_bundles = list(json_data.get("bundles", []))
  for bundle in extra_bundles:
    if not isinstance(bundle, dict):
      continue
    if bundle.get("ref") in official_refs:
      cloudlog.warning(f"Extra model {bundle.get('ref')} has the ref of an official model, skipped")
      continue
    if bundle.get("is_big") is True:
      continue
    if not is_valid(bundle):
      cloudlog.warning(f"Extra model {bundle.get('ref')} is not a valid bundle, skipped")
      continue
    marked = dict(bundle)
    marked[EXTRA_KEY] = True
    merged_bundles.append(marked)

  merged = dict(json_data)
  merged["bundles"] = merged_bundles
  return merged
