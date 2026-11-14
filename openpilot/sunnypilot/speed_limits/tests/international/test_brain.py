"""The brain's reader on a fake network: what it sends in, what it says, what it refuses."""

import hashlib
import json
from pathlib import Path

import numpy as np
import pytest
from openpilot.sunnypilot.speed_limits import brain, brain_inputs

GROUPS = ['signs', 'plate', 'plate_hours', 'hour', 'lead', 'country', 'pedals', 'speed']
SIZE = sum(brain_inputs.INPUT_SIZES[name] for name in GROUPS)
BOX = [10.0, 4.0, 20.0, 14.0]  # of a sign, in the pixels of the 64 x 32 frames below


def contract(**changes):
  inputs = hashlib.sha256(Path(brain_inputs.__file__).read_bytes()).hexdigest()
  step = {'rate_hz': 1.0, 'groups': GROUPS, 'memory_gap_seconds': 5.0}
  return {'inputs_sha256': inputs, 'output': {'kph': [5, 10, 15, 20]}, 'step': step, **changes}


class Network:
  """Says class 2 (15 km/h); its memory is how many steps it has made since zeros."""
  def __init__(self):
    self.calls = []

  def input_shape(self, name):
    return (1, 4)

  def __call__(self, image, extra, hidden):
    self.calls.append((image, extra, hidden))
    return {'speed': np.array([[0.0, 0.1, 0.9, 0.0]], dtype=np.float32), 'hidden_out': hidden + 1}


@pytest.fixture
def reader(vision):
  return brain.BrainReader(Network(), contract(), vision, 64, 32)


def frame():
  return np.zeros((32, 64), dtype=np.uint8), np.full((16, 64), 128, dtype=np.uint8)


def test_refuses_a_network_made_with_other_inputs(vision):
  with pytest.raises(ValueError, match='another brain_inputs'):
    brain.BrainReader(Network(), contract(inputs_sha256='0' * 64), vision, 64, 32)


def test_a_brain_of_an_earlier_file_with_the_same_groups_stays(vision):
  step = {'rate_hz': 1.0, 'groups': ['signs', 'plate', 'hour', 'lead', 'country'], 'memory_gap_seconds': 5.0}
  reader = brain.BrainReader(Network(), contract(inputs_sha256=brain_inputs.SAME_GROUPS[0], step=step), vision, 64, 32)
  reader.step(1.0, *frame(), None, None, None, None)
  assert reader.network.calls[0][1].shape == (1, sum(brain_inputs.INPUT_SIZES[name] for name in step['groups']))


def test_refuses_a_network_asking_for_what_the_car_does_not_give(vision):
  wrong = contract(step={'rate_hz': 1.0, 'groups': ['signs', 'map'], 'memory_gap_seconds': 5.0})
  with pytest.raises(ValueError, match='does not give'):
    brain.BrainReader(Network(), wrong, vision, 64, 32)


def test_says_the_class_in_metres_per_second(reader):
  assert reader.speed_limit == 0.0
  assert reader.step(1.0, *frame(), None, None, None, None) == pytest.approx(15 / 3.6)
  assert reader.speed_limit == pytest.approx(15 / 3.6)


def test_steps_once_a_second(reader):
  assert reader.due(0.0)
  reader.step(0.0, *frame(), None, None, None, None)
  assert not reader.due(0.9)
  assert reader.due(1.0)


def test_the_groups_go_in_side_by_side(reader):
  reader.step(1.0, *frame(), (30.0, 36.0), 1_770_000_000.0, 'DE', 15.0)
  _, extra, _ = reader.network.calls[0]
  assert extra.shape == (1, SIZE)
  at = sum(brain_inputs.INPUT_SIZES[name] for name in GROUPS[:4])
  assert np.array_equal(extra[0, at:at + 3], brain_inputs.lead_row(30.0, 36.0))
  at += brain_inputs.INPUT_SIZES['lead']
  assert np.array_equal(extra[0, at:at + brain_inputs.INPUT_SIZES['country']], brain_inputs.country_row('DE'))
  assert np.array_equal(extra[0, -brain_inputs.INPUT_SIZES['speed']:], brain_inputs.speed_row(15.0))


def test_a_sign_read_reaches_the_network(reader):
  for at in (0.0, 0.2, 0.4, 0.6):
    reader.add_signs(at, [{'class': 'maximum_speed:80', 'confidence': 0.99, 'box': BOX}])
  reader.step(1.0, *frame(), None, None, None, None)
  assert reader.signs.number == 80
  assert reader.network.calls[0][1][0, 0] == pytest.approx(80 / brain_inputs.SIGN_KPH_SCALE)


def read_hours(reader):
  for at in (0.0, 0.2, 0.4, 0.6):
    reader.add_signs(at, [{'class': 'maximum_speed_hours:30', 'confidence': 0.99, 'box': BOX}])


def unknown_plate():
  return np.eye(brain_inputs.INPUT_SIZES['plate'], dtype=np.float32)[-1]


def test_a_brain_trained_without_plates_gets_no_plate(reader, monkeypatch):
  seen = []
  monkeypatch.setattr(reader.signs, 'add', lambda now, signs: seen.append(signs))
  reader.add_signs(0.0, [{'class': 'maximum_speed_hours:30', 'confidence': 0.99, 'box': BOX}])
  assert seen == [[('maximum_speed_hours:30', 0.99)]]


def test_without_plates_a_number_with_hours_is_its_number(reader):
  read_hours(reader)
  assert reader.signs.number == 30
  assert np.array_equal(reader.signs.plate_row(), unknown_plate())


def test_a_brain_trained_with_plates_gets_the_plate(vision):
  reader = brain.BrainReader(Network(), contract(step=contract()['step'] | {'plates': True}), vision, 64, 32)
  read_hours(reader)
  assert reader.signs.number == 30
  reader.step(1.0, *frame(), None, None, None, None)
  at = brain_inputs.INPUT_SIZES['signs']
  plate = reader.network.calls[0][1][0, at:at + brain_inputs.INPUT_SIZES['plate']]
  assert plate[brain_inputs.PLATES.index('time')] == 1.0 and plate.sum() == 1.0


def test_a_brain_trained_with_plates_gets_none_from_a_plain_number(vision):
  reader = brain.BrainReader(Network(), contract(step=contract()['step'] | {'plates': True}), vision, 64, 32)
  for at in (0.0, 0.2, 0.4, 0.6):
    reader.add_signs(at, [{'class': 'maximum_speed:30', 'confidence': 0.99, 'box': BOX}])
  assert np.array_equal(reader.signs.plate_row(), unknown_plate())


def read_wet(reader):
  for at in (0.0, 0.2):
    reader.add_signs(at, [{'class': 'maximum_speed_wet:80', 'confidence': 0.99, 'box': BOX}])


def test_a_brain_trained_with_wet_plates_gets_them(vision):
  groups = [*GROUPS, 'plate_wet']
  reader = brain.BrainReader(Network(), contract(step=contract()['step'] | {'groups': groups, 'wet': True}), vision, 64, 32)
  read_wet(reader)
  reader.step(1.0, *frame(), None, None, None, None)
  at = sum(brain_inputs.INPUT_SIZES[name] for name in GROUPS)
  assert reader.network.calls[0][1][0, at] == 1.0


@pytest.mark.parametrize('step', [{}, {'plates': True}])
def test_a_brain_trained_without_wet_plates_takes_the_number_alone(vision, step):
  reader = brain.BrainReader(Network(), contract(step=contract()['step'] | step), vision, 64, 32)
  read_wet(reader)
  assert not reader.signs.plate_wet_row().any() and np.array_equal(reader.signs.plate_row(), unknown_plate())
  assert reader.signs.number == 80


def test_a_brain_trained_with_wet_plates_and_no_hours_plates_takes_a_number_with_hours_as_a_number(vision):
  reader = brain.BrainReader(Network(), contract(step=contract()['step'] | {'wet': True}), vision, 64, 32)
  read_hours(reader)
  assert np.array_equal(reader.signs.plate_row(), unknown_plate())


def read_with_hours(reader):
  for at in (0.0, 0.2, 0.4, 0.6):
    reader.add_signs(at, [{'class': 'maximum_speed_hours:30:22-6', 'confidence': 0.99, 'box': BOX}])


def test_a_brain_trained_with_hours_gets_them(vision):
  reader = brain.BrainReader(Network(), contract(step=contract()['step'] | {'plates': True, 'hours': True}), vision, 64, 32)
  read_with_hours(reader)
  reader.step(1.0, *frame(), None, None, None, None)
  at = sum(brain_inputs.INPUT_SIZES[name] for name in GROUPS[:2])
  hours = reader.network.calls[0][1][0, at:at + brain_inputs.INPUT_SIZES['plate_hours']]
  assert np.allclose(hours[:2], brain_inputs.hour_row(22)[:2]) and np.allclose(hours[2:4], brain_inputs.hour_row(6)[:2])
  assert hours[4] == 1.0


def test_a_brain_trained_with_plates_and_no_hours_gets_the_plate_alone(vision):
  reader = brain.BrainReader(Network(), contract(step=contract()['step'] | {'plates': True}), vision, 64, 32)
  read_with_hours(reader)
  assert reader.signs.plate == 'time' and not reader.signs.plate_hours_row().any()


def test_the_memory_goes_on_and_starts_again_after_a_gap(reader):
  reader.step(0.0, *frame(), None, None, None, None)
  reader.step(1.0, *frame(), None, None, None, None)
  assert reader.network.calls[1][2][0, 0] == 1
  reader.step(7.0, *frame(), None, None, None, None)
  assert reader.network.calls[2][2][0, 0] == 0


def test_it_gets_the_pedals_of_the_last_second_and_the_car_speed(reader):
  reader.add_pedals(9.5, True, False)
  reader.add_pedals(9.6, False, False)
  reader.step(10.0, *frame(), None, None, None, 15.0)
  reader.step(11.0, *frame(), None, None, None, None)
  at = sum(brain_inputs.INPUT_SIZES[name] for name in GROUPS[:6])
  first, second = (call[1][0][at:] for call in reader.network.calls)
  assert list(first) == [1, 0, 1, 0.5, 1]
  assert list(second) == [0, 0, 1, 0, 0]


def test_a_pedal_is_not_known_before_the_first_car_state(reader):
  reader.step(1.0, *frame(), None, None, None, None)
  at = sum(brain_inputs.INPUT_SIZES[name] for name in GROUPS[:6])
  assert list(reader.network.calls[0][1][0][at:at + 3]) == [0, 0, 0]


def test_it_gets_openpilot_engaged_and_what_turned_it_off(vision):
  groups = [*GROUPS, 'engaged']
  reader = brain.BrainReader(Network(), contract(step={'rate_hz': 1.0, 'groups': groups, 'memory_gap_seconds': 5.0}), vision, 64, 32)
  reader.add_enabled(1.0, True)
  reader.add_pedals(9.0, False, True)
  reader.add_enabled(9.0, False)
  reader.add_pedals(9.4, False, False)
  reader.step(10.0, *frame(), None, None, None, None)
  row = reader.network.calls[0][1][0][SIZE:]
  assert len(row) == brain_inputs.INPUT_SIZES['engaged']
  assert [row[0], row[1], row[3], row[4], row[5], row[6]] == [0, 1, 1, 0, 0, 1]


def test_a_brain_of_the_first_contracts_steps_as_before(vision):
  # No clock or picture in its contract: the road-type picture once a second, the sign distance not sent
  reader = brain.BrainReader(Network(), contract(), vision, 64, 32)
  assert isinstance(reader.image, vision.RoadTypeInput) and not reader.schedule
  reader.step(0.0, *frame(), None, None, None, None, 0.0)
  reader.step(1.0, *frame(), None, None, None, None, 50.0)
  first, second = (call[1] for call in reader.network.calls)
  assert first.shape == second.shape == (1, SIZE) and np.array_equal(first, second)


def test_the_published_brain_still_loads(vision):
  published = Path(brain.__file__).parent / 'models' / 'brain.onnx.contract.json'
  if not published.exists():
    pytest.skip('no published brain in this checkout')
  said = json.loads(published.read_text())
  reader = brain.BrainReader(AheadNetwork(1, 0.8), said, vision, 64, 32)
  assert reader.due(0.0)
  # Its network (11 Nov 2026) also gives the next limit and the metres to it: stepped all the same, its speed is the limit
  assert reader.step(0.0, *frame(), None, None, None, None, 50.0) == pytest.approx(said['output']['kph'][2] / 3.6)


AHEAD_GROUPS = [*GROUPS, 'sign_distance']


def four_hz(vision, network=None):
  """A brain of 4 steps a second on the grid, with the brain's picture and the sign distance; its contract has the scale
  of the metres to the next limit, as the brains trained until 12 Nov 2026, which the car does not read."""
  step = {'rate_hz': 4.0, 'groups': AHEAD_GROUPS, 'memory_gap_seconds': 5.0, 'clock': 'schedule', 'picture': 'brain'}
  output = {'kph': [5, 10, 15, 20], 'metres_scale': 100.0}
  return brain.BrainReader(network or Network(), contract(step=step, output=output), vision, 64, 32)


def readings(reader, every, until):
  """The readings at which the reader steps, of readings every `every` seconds from 0 to until."""
  steps = []
  for n in range(round(until / every)):
    if reader.due(n * every):
      reader.step(n * every, *frame(), None, None, None, None)
      steps.append(round(n * every, 3))
  return steps


def test_a_brain_at_4_hz_steps_on_its_grid(vision):
  # Readings at 5 a second: 4 of every 5 step, as the training's frames every 0.25 s
  assert len(readings(four_hz(vision), 0.2, 10.0)) == 40
  # Slower readings step at each one
  assert readings(four_hz(vision), 0.3, 3.0) == [round(n * 0.3, 3) for n in range(10)]


def test_a_brain_at_4_hz_starts_a_new_grid_after_a_pause(vision):
  reader = four_hz(vision)
  reader.step(0.0, *frame(), None, None, None, None)
  assert reader.due(3.0)
  reader.step(3.0, *frame(), None, None, None, None)
  assert not reader.due(3.2) and reader.due(3.25)  # one step at 3.0, not the ones missed


def test_a_brain_at_4_hz_sees_its_picture_and_the_sign_distance(vision):
  reader = four_hz(vision)
  for at, metres in ((0.0, 50.0), (0.25, 0.0), (0.5, 500.0)):
    reader.step(at, *frame(), None, None, None, None, metres)
  assert reader.network.calls[0][0].shape == (1, 3, vision.BRAIN_HEIGHT, vision.BRAIN_WIDTH)
  assert [list(call[1][0, -2:]) for call in reader.network.calls] == [[0.5, 1.0], [0.0, 0.0], [2.0, 1.0]]


class AheadNetwork(Network):
  """Says 15 km/h and, as the brains trained until 12 Nov 2026, the next limit of class `ahead` (4: none) and the
  metres / 100 to it."""
  def __init__(self, ahead, metres):
    super().__init__()
    self.ahead, self.metres = ahead, metres

  def __call__(self, image, extra, hidden):
    outputs = super().__call__(image, extra, hidden)
    return outputs | {'next': np.eye(5, dtype=np.float32)[self.ahead][None], 'metres': np.array([[self.metres]], dtype=np.float32)}


@pytest.mark.parametrize('ahead, metres', [
  (1, 0.8),  # a lower limit in 80 m
  (0, -0.1),
  (4, 0.8),  # none ahead
  (2, 0.8),  # the limit it holds
])
def test_a_network_that_gives_the_next_limit_and_the_metres_says_only_its_speed(vision, ahead, metres):
  reader = four_hz(vision, AheadNetwork(ahead, metres))
  assert reader.step(0.0, *frame(), None, None, None, None, 0.0) == pytest.approx(15 / 3.6)
  assert reader.step(0.25, *frame(), None, None, None, None, 0.0) == pytest.approx(15 / 3.6)
  assert reader.speed_limit == pytest.approx(15 / 3.6)
  assert reader.network.calls[1][2][0, 0] == 1  # stepped as any other: its memory goes on
  # Nothing else is said: no limit ahead, no metres, no scale for them
  assert not {'speed_limit_ahead', 'ahead_distance', 'metres_scale'} & set(vars(reader))


class RoadTypeNetwork(Network):
  """Says 15 km/h and the second road type."""
  def __call__(self, image, extra, hidden):
    return super().__call__(image, extra, hidden) | {'road_type': np.array([[0.1, 0.8, 0.1]], dtype=np.float32)}


def test_it_says_the_road_type_for_the_screen(vision):
  types = contract(output={'kph': [5, 10, 15, 20], 'road_types': ['town', 'motorway', 'rural']})
  reader = brain.BrainReader(RoadTypeNetwork(), types, vision, 64, 32)
  assert reader.road_type == ""
  reader.step(0.0, *frame(), None, None, None, None, 0.0)
  assert (reader.road_type, reader.speed_limit) == ("motorway", pytest.approx(15 / 3.6))


def test_a_brain_without_the_road_type_says_none(reader):
  reader.step(0.0, *frame(), None, None, None, None, 0.0)
  assert reader.road_type == ""


BOX_GROUPS = [*GROUPS, 'sign_boxes']


def boxed(vision, width, height):
  """A brain trained with the boxes of the signs, on frames of width x height."""
  step = {'rate_hz': 4.0, 'groups': BOX_GROUPS, 'memory_gap_seconds': 5.0, 'clock': 'schedule', 'picture': 'brain'}
  return brain.BrainReader(Network(), contract(step=step), vision, width, height)


def big_frame(width, height):
  return np.zeros((height, width), dtype=np.uint8), np.full((height // 2, width), 128, dtype=np.uint8)


@pytest.mark.parametrize('width, height', [(1344, 760), (2688, 1520)])
def test_a_brain_trained_with_the_boxes_gets_where_each_sign_is_and_how_big(vision, width, height):
  reader = boxed(vision, width, height)
  scale = width / 1344
  signs = [{'class': 'maximum_speed:80', 'confidence': 0.99, 'box': [x * scale for x in (1008.0, 190.0, 1050.0, 232.0)]},
           {'class': 'other_sign:give_way', 'confidence': 0.9, 'box': [x * scale for x in (84.0, 304.0, 168.0, 380.0)]},
           {'class': 'maximum_speed:60', 'confidence': 0.4, 'box': [x * scale for x in (500.0, 100.0, 540.0, 140.0)]}]
  reader.add_signs(0.0, signs)
  reader.step(0.0, *big_frame(width, height), None, None, None, None)
  size = brain_inputs.INPUT_SIZES['sign_boxes']
  assert size == 50
  row = reader.network.calls[0][1][0, -size:]
  grid = row[:-2].reshape(brain_inputs.SIGN_BOX_CHANNELS, brain_inputs.BOX_COLUMNS, brain_inputs.BOX_NUMBERS)
  # The number in the seventh column of the speed signs, the give way in the first of the other signs, the unsure one
  # nowhere: the same fractions of the frame whatever its definition
  assert grid[0, 6] == pytest.approx([42 / 1344, 42 / 760, 211 / 760])
  assert grid[1, 0] == pytest.approx([84 / 1344, 76 / 760, 342 / 760])
  assert np.count_nonzero(grid) == 6 and list(row[-2:]) == [1.0, 0.0]
  # The next reading has no sign: the columns empty, the memory of the last one stays
  reader.add_signs(0.25, [])
  reader.step(0.25, *big_frame(width, height), None, None, None, None)
  row = reader.network.calls[1][1][0, -size:]
  assert not row[:-2].any() and row[-2] == 1 and row[-1] == pytest.approx(np.log1p(0.25) / brain_inputs.SIGN_AGE_SCALE)


def test_a_brain_trained_without_the_boxes_gets_what_it_got(reader):
  # The published brain's groups have no sign_boxes: a sign with its box changes nothing of what goes in
  reader.add_signs(0.0, [{'class': 'other_sign:stop', 'confidence': 0.99, 'box': BOX}])
  reader.step(0.0, *frame(), None, None, None, None)
  assert reader.network.calls[0][1].shape == (1, SIZE)


def test_the_sign_distance_row():
  assert list(brain_inputs.sign_distance_row(0.0)) == [0.0, 0.0]
  assert list(brain_inputs.sign_distance_row(-3.0)) == [0.0, 0.0]
  assert brain_inputs.sign_distance_row(30.0) == pytest.approx([0.3, 1.0])
  assert brain_inputs.sign_distance_row(900.0) == pytest.approx([brain_inputs.SIGN_METRES_MAX / brain_inputs.SIGN_METRES_SCALE, 1.0])
