"""Run speedvisiond.main with simulated camera/network/socket I/O; no inference."""

from types import SimpleNamespace as NS

import pytest
import numpy as np

from test_chain import execute

POLICY = {'gps_max_age_s': 2, 'gps_max_accuracy_m': 20, 'border_margin_m': 100, 'confirm_s': 0.1, 'max_sample_gap_s': 1}


def run_daemon(candidate, vision, country, settings, osm_country, gps_fix_until=float('inf'), signs_from=0.0, reader=None,
               brain_contract=None, sleep=lambda _: None, ruler=None):
  """The messages speedvisiond sends until 1.3 s: GPS in GB (with a fix until gps_fix_until), a 30 sign from signs_from;
  ruler: the applicability network's class (applicability.Applicability), None for a release without it."""
  class Stop(Exception):
    pass

  clock = NS(now=0.0)
  outputs = []

  class Client:
    width = 16
    height = 16
    frame_id = 0

    def __init__(self, *args):
      pass

    def connect(self, *args):
      return True

    def recv(self):
      clock.now += 0.2
      if clock.now > 1.3:
        raise Stop
      return NS(data=bytes(16 * 16 * 3 // 2), width=16, height=16, stride=16, uv_offset=256)

  def drain(sock):
    if sock == 'gpsLocationExternal':
      g = NS(latitude=0, longitude=0.5, horizontalAccuracy=5, hasFix=clock.now < gps_fix_until, unixTimestampMillis=1_770_000_000_000)
      return [NS(logMonoTime=int((clock.now - 0.01) * 1e9), valid=True, gpsLocationExternal=g)]
    if sock == 'radarState':
      return [NS(radarState=NS(leadOne=NS(present=True, dRel=30.0, vLead=10.0)))]
    return []

  class Pub:
    def __init__(self, topics):
      pass

    def send(self, topic, msg):
      outputs.append(msg.speedVisionSP)

  class Sub:
    def __init__(self, *args):
      pass

    def update(self, *args):
      pass

  class Params:
    def get(self, key):
      return osm_country if key == 'OsmLocationName' else None

    def get_bool(self, key):
      return False

  files = NS(settings=settings, name='synthetic', finder='finder', classifier='classifier', road_type=None,
             brain='brain' if reader is not None else None, brain_contract=brain_contract, classes=['maximum_speed:30'],
             decider='decider' if ruler is not None else None, decider_contract={})
  signs = [dict(box=[0, 0, 10, 10], confidence=0.99, speed=30.0, **{'class': 'maximum_speed:30'})]
  v = NS(**vars(vision))
  v.SignNetInput = lambda *args: None
  v.read_signs = lambda *args: signs if clock.now >= signs_from else []
  b = country.OfflineBorders([{'code': 'GB', 'bbox': [0, -1, 1, 1], 'polygons': [[[[0, -1], [1, -1], [1, 1], [0, 1], [0, -1]]]]}])
  env = {
    'os': NS(nice=lambda _: None, environ={}),
    'time': NS(monotonic=lambda: clock.now, sleep=sleep),
    'np': np,
    'NoReturn': object,
    'VisionIpcClient': Client,
    'VisionStreamType': NS(VISION_STREAM_NARROW_ROAD=0),
    'messaging': NS(sub_sock=lambda name, **kw: name, drain_sock=drain, SubMaster=Sub, PubMaster=Pub, new_message=lambda *a, **kw: NS(speedVisionSP=NS())),
    'Params': Params,
    'get_gps_location_service': lambda _: 'gpsLocationExternal',
    'cloudlog': NS(warning=lambda _: None, exception=lambda _: None),
    'DEVICE_CAMERAS': {},
    'vision': v,
    'runner': NS(networks=lambda: files, Network=lambda _: None, compile_network=lambda p: p, StepNetwork=lambda p: None),
    'brain': NS(BrainReader=reader),
    'applicability': NS(**{**execute(candidate / 'openpilot/sunnypilot/speed_limits/applicability.py', {'np': np}),
                           'Applicability': ruler}),
    'CountryResolver': country.CountryResolver,
    'OfflineBorders': NS(load=lambda: b),
    'GpsFix': country.GpsFix,
    '__name__': 'units_daemon',
  }
  execute(candidate / 'openpilot/sunnypilot/speed_limits/speedvisiond.py', env)
  with pytest.raises(Stop):
    env['main']()
  return outputs[-1]


def test_daemon_publishes_memory_mps_once(candidate, vision, country):
  # The country GPS confirms comes before the one of the settings
  out = run_daemon(candidate, vision, country, {'country_policy': POLICY}, 'CH')
  assert out.speedLimit == pytest.approx(13.4112)
  assert (out.rawSignValue, out.rawSignUnit, out.rawSignCountry) == (30, 'mph', 'GB')


@pytest.mark.parametrize('osm_country, unit, country_code, limit', [
  ('US', 'mph', 'US', 30 * 0.44704),
  ('DE', 'km/h', 'DE', 30 / 3.6),
  (None, 'km/h', 'CH', 30 / 3.6),  # no country chosen: Switzerland
])
def test_daemon_without_country_policy_takes_the_settings_country(candidate, vision, country, osm_country, unit, country_code, limit):
  out = run_daemon(candidate, vision, country, {}, osm_country)
  assert out.speedLimit == pytest.approx(limit)
  assert (out.rawSignValue, out.rawSignUnit, out.rawSignCountry) == (30, unit, country_code)


def test_daemon_keeps_the_last_gps_country_when_the_fix_is_lost(candidate, vision, country):
  out = run_daemon(candidate, vision, country, {'country_policy': POLICY}, 'CH', gps_fix_until=0.7, signs_from=0.8)
  assert out.speedLimit == pytest.approx(13.4112)
  assert (out.rawSignValue, out.rawSignUnit, out.rawSignCountry) == (30, 'mph', 'GB')


class FakeBrain:
  """A BrainReader saying 77 km/h from its first step, with what the daemon gave it."""
  road_type = ""

  def __init__(self, network, contract, vision, width, height):
    self.speed_limit = 0.0
    self.seen, self.steps = [], []
    self.signs = NS(read_at=None)
    FakeBrain.last = self

  def add_signs(self, now, signs):
    self.seen.append(signs)
    self.signs.read_at = now

  def due(self, now):
    return not self.steps

  def step(self, now, y, uv, lead, unix_seconds, country, v_ego, sign_distance, position=None):
    self.steps.append((lead, unix_seconds, country))
    self.speed_limit = 77 / 3.6


def test_daemon_with_a_brain_publishes_what_it_says(candidate, vision, country):
  out = run_daemon(candidate, vision, country, {'country_policy': POLICY}, 'CH', reader=FakeBrain, brain_contract={})
  assert out.speedLimit == pytest.approx(77 / 3.6)  # never the 30 sign's mph
  assert (out.speedLimitState, out.speedLimitReason) == ('limit', 'brain')
  assert out.signRead
  assert (out.rawSignValue, out.rawSignUnit) == (30, 'mph')  # the logs keep what was read
  assert FakeBrain.last.seen[0][0]['class'] == 'maximum_speed:30'
  lead, unix_seconds, _ = FakeBrain.last.steps[0]
  assert lead == (30.0, 36.0)
  assert unix_seconds >= 1_770_000_000
  # The speed is all a brain says: no limit ahead is published, whatever is ahead
  assert not hasattr(out, 'speedLimitAhead') and not hasattr(out, 'speedLimitAheadDistance')


def test_daemon_without_a_brain_still_publishes_the_memorys_limit_ahead(candidate, vision, country):
  # vision.LimitAhead's two fields are in every message; 0 here: the first limit applies at once
  out = run_daemon(candidate, vision, country, {'country_policy': POLICY}, 'CH')
  assert (out.speedLimitAhead, out.speedLimitAheadDistance) == (0.0, 0.0)


class SideRoad:
  """An applicability network giving every sign it rules to another road, with what the daemon gave it."""
  def __init__(self, network, contract, vision, width, height):
    self.v_egos = []
    SideRoad.last = self

  def rule(self, y, uv, signs, v_ego):
    self.v_egos.append(v_ego)
    for sign in signs:
      sign['road'], sign['road_confidence'] = 'other_road', 0.9
    return signs


def test_daemon_does_not_take_a_sign_of_another_road(candidate, vision, country):
  out = run_daemon(candidate, vision, country, {'country_policy': POLICY}, 'CH', ruler=SideRoad)
  assert out.speedLimit == 0  # the 30 of the side road is never the limit
  assert (out.rawSignClass, out.rawSignRoad, out.rawSignTaken) == ('maximum_speed:30', 'other_road', False)  # but it shows
  assert out.rawSignSeen
  assert out.signDistance == 0


def test_daemon_gives_the_brain_only_the_signs_of_our_road(candidate, vision, country):
  out = run_daemon(candidate, vision, country, {'country_policy': POLICY}, 'CH', reader=FakeBrain, brain_contract={},
                   ruler=SideRoad)
  assert all(seen == [] for seen in FakeBrain.last.seen)
  assert (out.rawSignClass, out.rawSignRoad, out.rawSignTaken) == ('maximum_speed:30', 'other_road', False)


def test_daemon_with_a_brain_says_the_sign_was_given_to_it(candidate, vision, country):
  out = run_daemon(candidate, vision, country, {'country_policy': POLICY}, 'CH', reader=FakeBrain, brain_contract={})
  assert (out.rawSignRoad, out.rawSignTaken) == ('', True)


def test_daemon_takes_every_sign_when_the_applicability_network_does_not_load(candidate, vision, country):
  def refuse(*args):
    raise ValueError("not the car's applicability network")

  out = run_daemon(candidate, vision, country, {'country_policy': POLICY}, 'CH', ruler=refuse)
  assert out.speedLimit == pytest.approx(13.4112)
  assert (out.rawSignRoad, out.rawSignTaken) == ('', True)


def test_daemon_goes_idle_when_the_brain_does_not_load(candidate, vision, country):
  # No message: the resolver falls to the map, never to the old rules with the network missing
  class Idle(Exception):
    pass

  def refuse(*args):
    raise ValueError('another brain_inputs.py')

  def sleep(_):
    raise Idle

  with pytest.raises(Idle):
    run_daemon(candidate, vision, country, {}, 'CH', reader=refuse, brain_contract={}, sleep=sleep)


def full_stream_functions(candidate, vision, streams, switch):
  """speedvisiond's full_stream and full_planes with a camerad offering streams and the Developer switch at switch."""
  class Client:
    width, height, frame_id = 4, 4, 7

    def __init__(self, *args):
      pass

    @staticmethod
    def available_streams(name, block):
      return streams

    def connect(self, *args):
      return True

    def recv(self, timeout):
      return NS(data=bytes(range(24)), width=4, height=4, stride=4, uv_offset=16)

  env = {'os': NS(environ={}), 'Params': object, 'VisionIpcClient': Client, 'VisionStreamType': NS(VISION_STREAM_NARROW_ROAD_FULL=4),
         'cloudlog': NS(warning=lambda _: None),
         'time': NS(sleep=lambda _: None), 'np': np, 'vision': vision, 'NoReturn': object, '__name__': 'units_daemon'}
  execute(candidate / 'openpilot/sunnypilot/speed_limits/speedvisiond.py', env)
  return env['full_stream'](NS(get_bool=lambda key: switch and key == 'ReadSignsFullResolution')), env['full_planes']


def test_full_stream_only_with_the_switch_and_the_stream(candidate, vision):
  assert full_stream_functions(candidate, vision, {0, 4}, False)[0] is None
  assert full_stream_functions(candidate, vision, {0}, True)[0] is None
  assert full_stream_functions(candidate, vision, {0, 4}, True)[0] is not None


def test_full_planes_of_the_same_frame_only(candidate, vision):
  client, full_planes = full_stream_functions(candidate, vision, {0, 4}, True)
  y, uv = full_planes(client, 7)
  assert y.shape == (4, 4) and uv.shape == (2, 4)
  assert full_planes(client, 6) is None
  assert full_planes(None, 7) is None
