"""The brain's reader on a fake network: what it sends in, what it says, what it refuses."""

import hashlib
from pathlib import Path

import numpy as np
import pytest
from openpilot.sunnypilot.speed_limits import brain, brain_inputs

GROUPS = ['signs', 'plate', 'plate_hours', 'hour', 'lead', 'country', 'pedals', 'speed']
SIZE = sum(brain_inputs.INPUT_SIZES[name] for name in GROUPS)


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
    reader.add_signs(at, [{'class': 'maximum_speed:80', 'confidence': 0.99}])
  reader.step(1.0, *frame(), None, None, None, None)
  assert reader.signs.number == 80
  assert reader.network.calls[0][1][0, 0] == pytest.approx(80 / brain_inputs.SIGN_KPH_SCALE)


def read_hours(reader):
  for at in (0.0, 0.2, 0.4, 0.6):
    reader.add_signs(at, [{'class': 'maximum_speed_hours:30', 'confidence': 0.99}])


def unknown_plate():
  return np.eye(brain_inputs.INPUT_SIZES['plate'], dtype=np.float32)[-1]


def test_a_brain_trained_without_plates_gets_no_plate(reader, monkeypatch):
  seen = []
  monkeypatch.setattr(reader.signs, 'add', lambda now, signs: seen.append(signs))
  reader.add_signs(0.0, [{'class': 'maximum_speed_hours:30', 'confidence': 0.99}])
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
    reader.add_signs(at, [{'class': 'maximum_speed:30', 'confidence': 0.99}])
  assert np.array_equal(reader.signs.plate_row(), unknown_plate())


def read_wet(reader):
  for at in (0.0, 0.2):
    reader.add_signs(at, [{'class': 'maximum_speed_wet:80', 'confidence': 0.99}])


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
    reader.add_signs(at, [{'class': 'maximum_speed_hours:30:22-6', 'confidence': 0.99}])


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
