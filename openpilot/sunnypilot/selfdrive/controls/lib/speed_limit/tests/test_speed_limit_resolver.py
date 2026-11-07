"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import random
import time

from openpilot.common.constants import CV
from openpilot.common.parameterized import parameterized

from openpilot.cereal import custom
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit import LIMIT_MAX_MAP_DATA_AGE

from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_resolver import SpeedLimitResolver, ALL_SOURCES
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Policy
from openpilot.common.test import OpenpilotTestCase

SpeedLimitSource = custom.LongitudinalPlanSP.SpeedLimit.Source


def create_mock(properties, mocker):
  mock = mocker.MagicMock()
  for _property, value in properties.items():
    setattr(mock, _property, value)
  return mock


def setup_sm_mock(mocker):
  cruise_speed_limit = random.uniform(0, 120)
  live_map_data_limit = random.uniform(0, 120)

  car_state = create_mock({
    'gasPressed': False,
    'brakePressed': False,
    'standstill': False,
  }, mocker)
  car_state_sp = create_mock({
    'speedLimit': cruise_speed_limit,
  }, mocker)
  live_map_data = create_mock({
    'speedLimit': live_map_data_limit,
    'speedLimitValid': True,
    'speedLimitAhead': 0.,
    'speedLimitAheadValid': 0.,
    'speedLimitAheadDistance': 0.,
  }, mocker)
  sm_mock = mocker.MagicMock()
  sm_mock.__getitem__.side_effect = lambda key: {
    'carState': car_state,
    'liveMapDataSP': live_map_data,
    'carStateSP': car_state_sp,
  }[key]
  sm_mock.recv_time = {'gpsLocation': time.monotonic(), 'liveMapDataSP': time.monotonic()}
  return sm_mock


parametrized_policies = parameterized.expand(
  [
    (Policy.car_state_only, 'carStateSP', SpeedLimitSource.car),
    (Policy.car_state_priority, 'carStateSP', SpeedLimitSource.car),
    (Policy.map_data_only, 'liveMapDataSP', SpeedLimitSource.map),
    (Policy.map_data_priority, 'liveMapDataSP', SpeedLimitSource.map),
  ],
  names=["policy", "sm_key", "function_key"]
)


def resolver_class():
  return SpeedLimitResolver


class TestSpeedLimitResolverValidation(OpenpilotTestCase):

  @parameterized.expand(list(Policy), names=["policy"])
  def test_initial_state(self, resolver_class, policy):
    resolver = resolver_class()
    resolver.policy = policy
    for source in ALL_SOURCES:
      if source in resolver.limit_solutions:
        assert resolver.limit_solutions[source] == 0.
        assert resolver.distance_solutions[source] == 0.

  @parametrized_policies
  def test_resolver(self, resolver_class, policy, sm_key, function_key, mocker):
    resolver = resolver_class()
    resolver.policy = policy
    sm_mock = setup_sm_mock(mocker)
    source_speed_limit = sm_mock[sm_key].speedLimit

    # Assert the resolver
    resolver.update(source_speed_limit, sm_mock)
    assert resolver.speed_limit == source_speed_limit
    assert resolver.source == ALL_SOURCES[function_key]

  def test_resolver_combined(self, resolver_class, mocker):
    resolver = resolver_class()
    resolver.policy = Policy.combined
    sm_mock = setup_sm_mock(mocker)
    socket_to_source = {'carStateSP': SpeedLimitSource.car, 'liveMapDataSP': SpeedLimitSource.map}
    minimum_key, minimum_speed_limit = min(
      ((key, sm_mock[key].speedLimit) for key in
       socket_to_source.keys()), key=lambda x: x[1])

    # Assert the resolver
    resolver.update(minimum_speed_limit, sm_mock)
    assert resolver.speed_limit == minimum_speed_limit
    assert resolver.source == socket_to_source[minimum_key]

  @parametrized_policies
  def test_parser(self, resolver_class, policy, sm_key, function_key, mocker):
    resolver = resolver_class()
    resolver.policy = policy
    sm_mock = setup_sm_mock(mocker)
    source_speed_limit = sm_mock[sm_key].speedLimit

    # Assert the parsing
    resolver.update(source_speed_limit, sm_mock)
    assert resolver.limit_solutions[ALL_SOURCES[function_key]] == source_speed_limit
    assert resolver.distance_solutions[ALL_SOURCES[function_key]] == 0.

  @parameterized.expand(list(Policy), names=["policy"])
  def test_resolve_interaction_in_update(self, resolver_class, policy, mocker):
    v_ego = 50
    resolver = resolver_class()
    resolver.policy = policy

    sm_mock = setup_sm_mock(mocker)
    resolver.update(v_ego, sm_mock)

    # After resolution
    assert resolver.speed_limit is not None
    assert resolver.distance is not None
    assert resolver.source is not None

  @parameterized.expand(list(Policy), names=["policy"])
  def test_old_map_data_ignored(self, resolver_class, policy, mocker):
    resolver = resolver_class()
    resolver.policy = policy
    sm_mock = setup_sm_mock(mocker)
    sm_mock.recv_time['gpsLocation'] = time.monotonic() - 2 * LIMIT_MAX_MAP_DATA_AGE
    resolver._get_from_map_data(sm_mock)
    assert resolver.limit_solutions[SpeedLimitSource.map] == 0.
    assert resolver.distance_solutions[SpeedLimitSource.map] == 0.

  @parameterized.expand([
    (80., 300., 0., 80., 0.),  # beyond the braking distance: still the current limit
    (80., 50., 0., 50., 50.),  # within it: the lower limit ahead, with its distance
    (80., 50., 1., 50., 50. - 80 * CV.KPH_TO_MS),  # less the distance driven since the map data was received
    (45., 50., 0., 50., 50.),  # already slower than the limit ahead: it holds
    (45., 300., 0., 80., 0.),  # but not before the braking distance from the current limit
  ], names=["v_kph", "ahead_distance", "map_data_age", "limit_kph", "distance"])
  def test_map_data_lower_limit_ahead(self, resolver_class, v_kph, ahead_distance, map_data_age, limit_kph, distance, mocker):
    resolver = resolver_class()
    resolver.policy = Policy.map_data_only
    sm_mock = setup_sm_mock(mocker)
    live_map_data = sm_mock['liveMapDataSP']
    live_map_data.speedLimit = 80 * CV.KPH_TO_MS
    live_map_data.speedLimitAhead = 50 * CV.KPH_TO_MS
    live_map_data.speedLimitAheadValid = True
    live_map_data.speedLimitAheadDistance = ahead_distance
    sm_mock.recv_time['liveMapDataSP'] = time.monotonic() - map_data_age

    resolver.update(v_kph * CV.KPH_TO_MS, sm_mock)
    assert resolver.source == SpeedLimitSource.map
    self.assertAlmostEqual(resolver.speed_limit, limit_kph * CV.KPH_TO_MS, places=5)
    self.assertAlmostEqual(resolver.distance, distance, delta=0.1)


def setup_vision_sm_mock(mocker, car, vision, map_limit, vision_alive=True, vision_ahead=0., vision_ahead_distance=0.,
                         vision_age=0.):
  """carStateSP, speedVisionSP and liveMapDataSP with the given limits (0: none)."""
  sources = {
    'carState': create_mock({'gasPressed': False, 'brakePressed': False, 'standstill': False}, mocker),
    'carStateSP': create_mock({'speedLimit': car}, mocker),
    'speedVisionSP': create_mock({'speedLimit': vision, 'speedLimitAhead': vision_ahead,
                                  'speedLimitAheadDistance': vision_ahead_distance}, mocker),
    'liveMapDataSP': create_mock({'speedLimit': map_limit, 'speedLimitValid': map_limit > 0., 'speedLimitAhead': 0.,
                                  'speedLimitAheadValid': 0., 'speedLimitAheadDistance': 0.}, mocker),
  }
  sm_mock = mocker.MagicMock()
  sm_mock.__getitem__.side_effect = lambda key: sources[key]
  sm_mock.recv_time = {'gpsLocation': time.monotonic(), 'liveMapDataSP': time.monotonic(),
                       'speedVisionSP': time.monotonic() - vision_age}
  sm_mock.alive = {'speedVisionSP': vision_alive}
  return sm_mock


class TestSpeedLimitResolverVision(OpenpilotTestCase):

  @parameterized.expand([
    # policy, car, vision, map, expected limit, expected source
    (Policy.map_data_vision, 0., 22., 25., 22., SpeedLimitSource.vision),
    (Policy.map_data_vision, 0., 22., 0., 22., SpeedLimitSource.vision),
    # The network is not sure (0): the map.
    (Policy.map_data_vision, 0., 0., 25., 25., SpeedLimitSource.map),
    (Policy.map_data_vision, 0., 0., 0., 0., SpeedLimitSource.none),
    # Map Data + Vision does not use the car's own limit.
    (Policy.map_data_vision, 30., 0., 25., 25., SpeedLimitSource.map),
    (Policy.map_data_vision, 30., 22., 25., 22., SpeedLimitSource.vision),
    # Vision Only: the network's limit, never the map or the car, and no limit when the network has none.
    (Policy.vision_only, 0., 22., 25., 22., SpeedLimitSource.vision),
    (Policy.vision_only, 30., 0., 25., 0., SpeedLimitSource.none),
    (Policy.vision_only, 0., 0., 0., 0., SpeedLimitSource.none),
    # The other policies ignore the network, as before.
    (Policy.car_state_priority, 0., 22., 25., 25., SpeedLimitSource.map),
    (Policy.car_state_only, 0., 22., 25., 0., SpeedLimitSource.none),
    (Policy.map_data_priority, 0., 22., 0., 0., SpeedLimitSource.none),
    (Policy.combined, 0., 22., 25., 25., SpeedLimitSource.map),
  ], names=["policy", "car", "vision", "map_limit", "expected", "expected_source"])
  def test_map_data_vision(self, resolver_class, policy, car, vision, map_limit, expected, expected_source, mocker):
    resolver = resolver_class()
    resolver.policy = policy
    resolver.update(20., setup_vision_sm_mock(mocker, car, vision, map_limit))
    assert resolver.speed_limit == expected
    assert resolver.source == expected_source

  @parameterized.expand([
    (80., 50., 300., 0., 80., 0.),  # beyond the braking distance: still the limit held
    (80., 50., 50., 0., 50., 50.),  # within it: the lower limit ahead, with its distance
    (80., 50., 50., 1., 50., 50. - 80 * CV.KPH_TO_MS),  # less the distance driven since speedvisiond sent it
    (45., 50., 50., 0., 50., 50.),  # already slower than the limit ahead: it holds
    (80., 120., 30., 0., 80., 0.),  # a higher limit waits for the sign, as the map's
    (80., 0., 30., 0., 80., 0.),  # nothing ahead
  ], names=["v_kph", "ahead_kph", "ahead_distance", "age", "limit_kph", "distance"])
  def test_vision_limit_ahead(self, resolver_class, v_kph, ahead_kph, ahead_distance, age, limit_kph, distance, mocker):
    resolver = resolver_class()
    resolver.policy = Policy.vision_only
    resolver.update(v_kph * CV.KPH_TO_MS, setup_vision_sm_mock(mocker, 0., 80 * CV.KPH_TO_MS, 0.,
                                                               vision_ahead=ahead_kph * CV.KPH_TO_MS,
                                                               vision_ahead_distance=ahead_distance, vision_age=age))
    assert resolver.source == SpeedLimitSource.vision
    self.assertAlmostEqual(resolver.speed_limit, limit_kph * CV.KPH_TO_MS, places=5)
    self.assertAlmostEqual(resolver.distance, distance, delta=0.1)

  def test_stale_vision_ignored(self, resolver_class, mocker):
    # speedvisiond stopped: its last message is never used, the map takes over.
    resolver = resolver_class()
    resolver.policy = Policy.map_data_vision
    resolver.update(20., setup_vision_sm_mock(mocker, 0., 22., 25., vision_alive=False))
    assert resolver.speed_limit == 25.
    assert resolver.source == SpeedLimitSource.map

  def test_vision_not_subscribed(self, resolver_class, mocker):
    # A SubMaster without speedVisionSP (an older planner, a debug tool): no vision limit, no error.
    resolver = resolver_class()
    resolver.policy = Policy.map_data_vision
    sm_mock = setup_sm_mock(mocker)
    resolver.update(20., sm_mock)
    assert resolver.limit_solutions[SpeedLimitSource.vision] == 0.
