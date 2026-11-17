import gzip
import hashlib
import json

from openpilot.sunnypilot.speed_limits.country import release_context


def test_release_resource_loads_policy_and_checks_digest(tmp_path):
  asset = gzip.compress(json.dumps({'countries': [{'code': 'FR', 'bbox': [-1, -1, 1, 1],
    'polygons': [[[[-1,-1],[1,-1],[1,1],[-1,1],[-1,-1]]]]}]}).encode())
  (tmp_path/'borders.json.gz').write_bytes(asset)
  document = {'asset': 'borders.json.gz', 'sha256': hashlib.sha256(asset).hexdigest(),
    'policy': {'gps_max_age_s': 2, 'gps_max_accuracy_m': 20, 'border_margin_m': 100,
      'confirm_s': 1, 'max_sample_gap_s': 1}}
  config = tmp_path/'country-context.json'
  config.write_text(json.dumps(document))
  assert release_context(config) is not None
  (tmp_path/'borders.json.gz').write_bytes(b'corrupt')
  assert release_context(config) is None
