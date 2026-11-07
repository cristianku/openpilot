"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The expected values come from ffmpeg 7.1 (yuv420p to rgb24, as the dataset's frames were decoded) and Pillow 12
(Image.resize BILINEAR, as the networks' inputs were resized in training).
"""
import numpy as np
import pytest

from openpilot.sunnypilot.speed_limits import vision

IMAGE = np.array([[139, 241, 6, 106, 188, 229, 10], [86, 66, 213, 167, 81, 153, 76], [250, 50, 112, 233, 89, 6, 143],
                  [210, 166, 189, 33, 30, 202, 220], [122, 77, 115, 230, 240, 184, 63]], dtype=np.uint8)
PIL_DOWN_4X3 = [[136, 120, 150, 111], [148, 143, 104, 115], [140, 140, 157, 157]]
PIL_UP_9X8 = [[139, 207, 137, 28, 106, 170, 211, 156, 10], [116, 148, 134, 105, 133, 139, 172, 143, 39],
              [96, 76, 128, 199, 171, 101, 116, 122, 80], [199, 103, 95, 159, 212, 114, 67, 75, 122],
              [238, 137, 109, 144, 171, 93, 69, 101, 167], [213, 177, 170, 153, 46, 37, 121, 198, 215],
              [161, 131, 130, 147, 144, 147, 173, 172, 132], [122, 92, 94, 141, 230, 238, 209, 144, 63]]
FFMPEG_RGB = {(16, 128, 128): (0, 0, 0), (235, 128, 128): (255, 255, 255), (128, 128, 128): (130, 130, 130),
              (17, 128, 129): (2, 0, 1), (120, 77, 150): (156, 122, 18), (90, 128, 100): (41, 108, 86),
              (60, 160, 77): (0, 79, 115), (200, 100, 77): (132, 255, 157), (81, 90, 240): (253, 0, 0)}


def nv12(y: np.ndarray, u: np.ndarray, v: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
  uv = np.empty((u.shape[0], u.shape[1] * 2), dtype=np.uint8)
  uv[:, 0::2], uv[:, 1::2] = u, v
  return y.astype(np.uint8), uv


def test_resize_is_pillow():
  planes = IMAGE[None]
  assert vision.resize(planes, 4, 3)[0].tolist() == PIL_DOWN_4X3
  assert vision.resize(planes, 9, 8)[0].tolist() == PIL_UP_9X8
  assert np.array_equal(vision.resize(planes, 7, 5), planes)


def test_resample_coefficients_drop_only_zero_taps():
  index, fixed = vision.resample_coefficients(1344, 896)
  assert index.shape == (896, 3)
  assert np.all(np.abs(fixed.sum(axis=1) - (1 << vision.PRECISION_BITS)) <= 3)
  assert index.min() >= 0 and index.max() <= 1343


@pytest.mark.parametrize("in_size, out_size", [(1344, 896), (1928, 896), (300, 200), (96, 64), (90, 60), (64, 48), (9, 6)])
def test_column_resampler_is_the_general_way(in_size, out_size):
  planes = np.random.default_rng(in_size).integers(0, 256, (3, 5, in_size), dtype=np.uint8)
  resampler = vision.ColumnResampler(in_size, out_size)
  general = vision.transpose(vision.resample_rows(vision.transpose(planes), resampler.index, resampler.fixed))
  assert np.array_equal(resampler(planes), general)
  if (in_size, out_size) == (1344, 896):
    assert len(resampler.phases) == 2 and len(resampler.edges) <= 4


def test_yuv_to_rgb_is_ffmpeg():
  for (y, u, v), rgb in FFMPEG_RGB.items():
    planes = vision.yuv_to_rgb(*nv12(np.full((2, 2), y), np.full((1, 1), u), np.full((1, 1), v)), 0, 2, 0, 2)
    assert planes[:, 0, 0].tolist() == list(rgb), (y, u, v)


def test_yuv_to_rgb_region_is_the_frame_cut():
  rng = np.random.default_rng(0)
  y, uv = nv12(rng.integers(0, 256, (10, 12)), rng.integers(0, 256, (5, 6)), rng.integers(0, 256, (5, 6)))
  whole = vision.yuv_to_rgb(y, uv, 0, 10, 0, 12)
  for top, bottom, left, right in ((1, 8, 3, 12), (0, 1, 0, 1), (9, 10, 11, 12), (3, 7, 1, 6)):
    assert np.array_equal(vision.yuv_to_rgb(y, uv, top, bottom, left, right), whole[:, top:bottom, left:right])


def test_nv12_planes_with_stride():
  data = np.arange(256 * 6, dtype=np.int64).astype(np.uint8)
  y, uv = vision.nv12_planes(data, 6, 4, 128, 128 * 4)
  assert y.shape == (4, 6) and uv.shape == (2, 6)
  assert y[1, 0] == 128 % 256 and uv[1, 2] == (128 * 4 + 128 + 2) % 256


def test_signnet_input_shape():
  height, width = 760, 1344
  y, uv = nv12(np.full((height, width), 235), np.full((height // 2, width // 2), 128), np.full((height // 2, width // 2), 128))
  signnet_input = vision.SignNetInput(width, height)
  x = signnet_input(y, uv)
  assert x.shape == (1, 3, vision.SIGNNET_HEIGHT, vision.SIGNNET_WIDTH) and x.dtype == np.float32
  assert np.all(x == 1.0)
  assert signnet_input.used_rows == 432 and signnet_input.scale == 1.5


def test_signnet_boxes():
  output = np.zeros((1, 5, 36, 112), dtype=np.float32)
  output[0, 0, 10, 20] = 0.9
  output[0, 0, 10, 21] = 0.5  # beside a higher peak: not a local maximum
  output[0, 0, 30, 100] = 0.4
  output[0, 0, 5, 5] = 0.2  # under MIN_BOX_SCORE
  output[0, 1:, 10, 20] = [np.log(16), np.log(24), 0.5, 0.25]
  boxes = vision.signnet_boxes(output, 1.5)
  assert [round(box["score"], 2) for box in boxes] == [0.9, 0.4]
  # centre (20.5 * 8, 10.25 * 8) = (164, 82), 16 x 24 in band pixels, times 1.5
  assert np.allclose(boxes[0]["box"], [234, 105, 258, 141])


def test_crop_rectangle():
  assert vision.crop_rectangle([10.2, 20.7, 30.1, 40.0], 100, 50) == (10, 20, 31, 40)
  assert vision.crop_rectangle([-5, -5, 0.5, 0.2], 100, 50) == (0, 0, 2, 2)
  assert vision.crop_rectangle([99.6, 49.9, 120, 80], 100, 50) == (98, 48, 100, 50)


def test_classifier_input():
  y, uv = nv12(np.full((20, 30), 128), np.full((10, 15), 128), np.full((10, 15), 128))
  x = vision.classifier_input(y, uv, [3, 4, 13, 9])
  assert x.shape == (1, 3, vision.CLASSIFIER_SIZE, vision.CLASSIFIER_SIZE)
  expected = (130 / 255 - vision.CLASSIFIER_MEAN.reshape(3)) / vision.CLASSIFIER_STD.reshape(3)
  assert np.allclose(x[0, :, 80, 80], expected, atol=1e-6)


def test_classifier_input_from_a_higher_definition():
  # The low frame's pixels doubled give the crop of the low frame, a detail only the high one has changes it
  low = np.zeros((40, 60), dtype=np.uint8)
  low[10:20, 20:30] = 200
  high = np.repeat(np.repeat(low, 2, axis=0), 2, axis=1)
  chroma = np.full((20, 30), 128, dtype=np.uint8)
  y, uv = nv12(low, chroma, chroma)
  high_chroma = np.full((40, 60), 128, dtype=np.uint8)
  box = [20, 10, 30, 20]
  expected = vision.classifier_input(y, uv, box)
  assert np.allclose(vision.classifier_input(y, uv, box, nv12(high, high_chroma, high_chroma)), expected, atol=1e-5)

  high[25:35, 45:55] = 0
  assert not np.allclose(vision.classifier_input(y, uv, box, nv12(high, high_chroma, high_chroma)), expected, atol=1e-5)


def test_classifier_input_cuts_the_same_window_of_the_scene():
  # A window at the frame's border stays the low frame's window: it is multiplied, never worked out again on the high one
  y, uv = nv12(np.full((20, 30), 128), np.full((10, 15), 128), np.full((10, 15), 128))
  high = np.zeros((40, 60), dtype=np.uint8)
  left, top, right, bottom = vision.crop_rectangle([29.6, 19.9, 40, 30], 30, 20)
  high[top * 2:bottom * 2, left * 2:right * 2] = 255
  chroma = np.full((20, 30), 128, dtype=np.uint8)
  x = vision.classifier_input(y, uv, [29.6, 19.9, 40, 30], nv12(high, chroma, chroma))
  white = vision.classifier_input(*nv12(np.full((20, 30), 255), np.full((10, 15), 128), np.full((10, 15), 128)), [0, 0, 10, 10])
  assert np.allclose(x, white, atol=1e-5)


@pytest.mark.parametrize("name, speed", [("maximum_speed:80", 80), ("maximum_speed_hours:30", 30),
                                         ("maximum_speed_hours:30:22-6", 30), ("maximum_speed_wet:80", 80),
                                         ("variable_display:100", 100), ("zone:30", 30),
                                         ("road_marking:30", 30), ("cancellation", 0), ("maximum_speed:unknown", None),
                                         ("zone:unknown", None), ("other_sign:give_way", None), ("not_a_sign", None),
                                         ("unreadable", None)])
def test_class_speed(name, speed):
  assert vision.class_speed(name) == speed


@pytest.mark.parametrize("name, plate", [("maximum_speed_hours:30", "time"), ("maximum_speed_hours:30:22-6", "time"),
                                         ("maximum_speed_wet:80", "wet"), ("maximum_speed:30", None), ("zone:30", None),
                                         ("cancellation", None), ("not_a_sign", None)])
def test_class_plate(name, plate):
  assert vision.class_plate(name) == plate


@pytest.mark.parametrize("name, hours", [("maximum_speed_hours:30:22-6", (22, 6)), ("maximum_speed_hours:30:7-17", (7, 17)),
                                         ("maximum_speed_hours:30", None), ("maximum_speed:30", None), ("cancellation", None)])
def test_class_hours(name, hours):
  assert vision.class_hours(name) == hours


def test_read_signs():
  output = np.zeros((1, 5, 36, 112), dtype=np.float32)
  output[0, 0, 10, 20], output[0, 0, 20, 50] = 0.9, 0.8
  output[0, 1:3] = np.log(20)
  classes = ["maximum_speed:50", "maximum_speed:80", "not_a_sign"]
  answers = iter([np.array([[5.0, 0.0, 0.0]]), np.array([[0.0, 0.5, 0.0]])])
  y, uv = nv12(np.full((760, 1344), 128), np.full((380, 672), 128), np.full((380, 672), 128))
  signs = vision.read_signs(y, uv, vision.SignNetInput(1344, 760), lambda x: output, lambda x: next(answers), classes)
  assert [(sign["class"], sign["speed"]) for sign in signs] == [("maximum_speed:50", 50), ("maximum_speed:80", None)]

  # With the planes of a higher definition SignNet stays on y and uv and the crops come from them
  answers = iter([np.array([[5.0, 0.0, 0.0]]), np.array([[0.0, 0.5, 0.0]])])
  crops = []

  def classifier(x):
    crops.append(x)
    return next(answers)

  full = nv12(np.full((1520, 2688), 255), np.full((760, 1344), 128), np.full((760, 1344), 128))
  signs = vision.read_signs(y, uv, vision.SignNetInput(1344, 760), lambda x: output, classifier, classes, full)
  assert [sign["box"] for sign in signs] == [sign["box"] for sign in vision.read_signs(
    y, uv, vision.SignNetInput(1344, 760), lambda x: output, lambda x: np.array([[5.0, 0.0, 0.0]]), classes)]
  assert all(crop.max() > 1.5 for crop in crops)  # white, where the low frame is grey


def test_memory_confirms_and_holds():
  memory = vision.SpeedLimitMemory("CH")
  assert memory.update(0.0, [60]) == pytest.approx(0 / 3.6)  # one frame is not enough
  assert memory.update(0.2, [60]) == pytest.approx(60 / 3.6) and memory.sign_read
  assert memory.update(0.4, []) == pytest.approx(60 / 3.6) and not memory.sign_read  # held without a sign
  assert memory.update(10.0, [80]) == pytest.approx(60 / 3.6)
  assert memory.update(12.0, [80]) == pytest.approx(60 / 3.6)  # too far apart
  assert memory.update(12.4, [80]) == pytest.approx(80 / 3.6)
  assert memory.update(15.0, [50, 50]) == pytest.approx(80 / 3.6)  # a value counts once a frame
  assert memory.update(20.0, [0]) == pytest.approx(80 / 3.6)
  assert memory.update(20.2, [0]) == pytest.approx(0 / 3.6)  # the end of the limit clears it


def test_memory_two_signs_at_once():
  memory = vision.SpeedLimitMemory("CH")
  memory.update(0.0, [80, 60])
  assert memory.update(0.2, [80, 60]) == pytest.approx(60 / 3.6)  # a tie: the lower
  memory.update(5.0, [100, 120])
  memory.update(5.2, [120])
  assert memory.update(5.4, [100, 120]) == pytest.approx(120 / 3.6)  # the one seen most
  memory.update(9.0, [0, 60])
  assert memory.update(9.2, [0, 60]) == pytest.approx(60 / 3.6)  # the end of a limit beside roadworks: not the general limit


def confirm(memory, at, readings=(), signs=(), **context):
  memory.update(at, list(readings), list(signs), **context)
  return memory.update(at + 0.2, list(readings), list(signs), **context)


def test_memory_road_signs_set_the_general_limit():
  memory = vision.SpeedLimitMemory("CH")
  assert confirm(memory, 0.0, signs=["other_sign:town_entry"]) == pytest.approx(50 / 3.6)
  assert confirm(memory, 5.0, [30]) == pytest.approx(30 / 3.6)
  assert confirm(memory, 9.0, [0]) == pytest.approx(50 / 3.6)  # the end of the 30 in a town: the town's 50
  assert confirm(memory, 20.0, signs=["other_sign:town_exit"]) == pytest.approx(80 / 3.6)
  assert confirm(memory, 30.0, signs=["other_sign:give_way"]) == pytest.approx(80 / 3.6)  # not a sign of the road's kind
  assert not memory.sign_read
  assert confirm(memory, 40.0, signs=["other_sign:town_exit"]) == pytest.approx(80 / 3.6) and memory.sign_read  # the same limit read again
  assert vision.SpeedLimitMemory("IT").update(0.0, [], ["other_sign:motorway_start"]) == 0
  assert confirm(vision.SpeedLimitMemory("IT"), 0.0, signs=["other_sign:motorway_start"]) == pytest.approx(130 / 3.6)


def test_memory_a_green_board_is_only_the_road_type_input():
  memory = vision.SpeedLimitMemory("CH")
  assert confirm(memory, 0.0, signs=["other_sign:town_exit"]) == pytest.approx(80 / 3.6)
  assert memory.update(10.0, [], ["other_sign:motorway_direction"]) == pytest.approx(80 / 3.6)
  assert memory.board_age(10.0) == float("inf")  # seen once: not confirmed
  assert confirm(memory, 20.0, signs=["other_sign:motorway_direction"], road_type="ramp") == pytest.approx(80 / 3.6)  # no 120 on the ramp
  assert abs(memory.board_age(25.0) - 4.8) < 1e-9 and memory.road_sign_age(25.0) == float("inf")
  assert memory.road == "rural" and not memory.sign_read and memory.limit_reason == "road_sign"
  # With a number in the same frames, the number is the limit, and the board still counts.
  assert confirm(memory, 30.0, [60], signs=["other_sign:motorway_direction"]) == pytest.approx(60 / 3.6) and memory.sign_read
  assert memory.board_age(30.2) == 0.0


def test_memory_ramp_limit_ends_by_the_road_type_network():
  memory = vision.SpeedLimitMemory("CH")
  assert confirm(memory, 0.0, signs=["other_sign:motorway_start"], road_type="motorway") == pytest.approx(120 / 3.6)
  assert confirm(memory, 10.0, [60], road_type="motorway") == pytest.approx(60 / 3.6)  # the ramp's 60, by the sign, whatever the network
  for t in range(12, 20):
    assert memory.update(float(t), [], road_type="ramp") == pytest.approx(60 / 3.6)  # still on the ramp
  for t in range(20, 27):
    assert memory.update(float(t), [], road_type="motorway") == pytest.approx(60 / 3.6)  # on the carriageway, not long enough
  assert memory.update(27.0, [], road_type="town") == pytest.approx(60 / 3.6)  # a doubt starts the count again
  for t in range(28, 36):
    memory.update(float(t), [], road_type="motorway")
  assert memory.update(36.0, [], road_type="motorway") == pytest.approx(120 / 3.6)
  assert memory.road == "motorway"


def test_memory_ramp_limit_far_from_the_sign_by_the_network():
  memory = vision.SpeedLimitMemory("CH")
  confirm(memory, 0.0, signs=["other_sign:motorway_start"])
  assert confirm(memory, 100.0, [60], road_type="ramp") == pytest.approx(60 / 3.6)  # far from the sign, the network says ramp
  for t in range(102, 110):
    memory.update(float(t), [], road_type="expressway")
  assert memory.update(110.0, [], road_type="expressway") == pytest.approx(100 / 3.6)


def test_memory_ramp_limit_ends_at_the_acceleration_lane():
  memory = vision.SpeedLimitMemory("CH")
  confirm(memory, 0.0, signs=["other_sign:expressway_start"])
  assert confirm(memory, 10.0, [60], road_type="ramp") == pytest.approx(60 / 3.6)
  for t in range(11, 20):
    assert memory.update(float(t), [], road_type="ramp") == pytest.approx(60 / 3.6)
  # The expressway's 100 from the start of the acceleration lane, not 8 s into the carriageway.
  assert memory.update(20.0, [], road_type="merge") == pytest.approx(100 / 3.6) and memory.road == "expressway" and memory.source == "sign"
  for t in range(21, 40):
    assert memory.update(float(t), [], road_type="rural") == pytest.approx(100 / 3.6)  # held as a sign's: the network's doubts change nothing


def test_memory_acceleration_lane_of_an_unknown_carriageway_waits_for_it():
  memory = vision.SpeedLimitMemory("CH")
  assert confirm(memory, 0.0, [60], road_type="ramp") == pytest.approx(60 / 3.6)  # no motorway or expressway sign seen
  for t in range(2, 8):
    assert memory.update(float(t), [], road_type="merge") == pytest.approx(60 / 3.6)  # which carriageway: not known yet
  # The lane counts towards MERGE_SECONDS: the carriageway named 8 s after the lane started.
  assert memory.update(8.0, [], road_type="motorway") == pytest.approx(60 / 3.6)
  assert memory.update(10.0, [], road_type="motorway") == pytest.approx(120 / 3.6) and memory.road == "motorway"


def test_memory_acceleration_lane_far_from_the_sign_is_not_known():
  memory = vision.SpeedLimitMemory("CH")
  confirm(memory, 0.0, signs=["other_sign:motorway_start"])
  assert confirm(memory, 100.0, [60], road_type="ramp") == pytest.approx(60 / 3.6)  # another ramp, long after the motorway's sign
  assert memory.update(105.0, [], road_type="merge") == pytest.approx(60 / 3.6)
  assert memory.update(113.0, [], road_type="expressway") == pytest.approx(100 / 3.6)


def test_memory_acceleration_lane_without_a_ramp_limit_changes_nothing():
  memory = vision.SpeedLimitMemory("CH")
  confirm(memory, 0.0, signs=["other_sign:motorway_start"])
  assert confirm(memory, 300.0, [80], road_type="motorway") == pytest.approx(80 / 3.6)  # roadworks long after the entry
  for t in range(301, 320):
    assert memory.update(float(t), [], road_type="merge") == pytest.approx(80 / 3.6)  # another entry joins the motorway here


def test_memory_a_limit_read_on_the_acceleration_lane_is_the_carriageways():
  memory = vision.SpeedLimitMemory("CH")
  confirm(memory, 0.0, signs=["other_sign:motorway_start"])
  assert confirm(memory, 8.0, [60], road_type="ramp") == pytest.approx(60 / 3.6)
  assert memory.update(20.0, [], road_type="merge") == pytest.approx(120 / 3.6)
  assert confirm(memory, 25.0, [100], road_type="merge") == pytest.approx(100 / 3.6)  # the motorway's own 100, beside the lane
  for t in range(26, 70):
    assert memory.update(float(t), [], road_type="merge" if t < 30 else "motorway") == pytest.approx(100 / 3.6)



def test_memory_a_limit_read_on_the_lane_without_a_ramps_number_is_the_carriageways():
  memory = vision.SpeedLimitMemory("CH")
  confirm(memory, 0.0, signs=["other_sign:motorway_start"])
  for t in range(1, 20):
    memory.update(float(t), [], road_type="ramp")  # no number on the ramp
  memory.update(20.0, [], road_type="merge")
  assert confirm(memory, 22.0, [100], road_type="merge") == pytest.approx(100 / 3.6)
  for t in range(23, 60):
    assert memory.update(float(t), [], road_type="merge" if t < 30 else "motorway") == pytest.approx(100 / 3.6)


def test_memory_an_expressways_link_to_the_motorway_ends_its_limit_on_the_lane():
  memory = vision.SpeedLimitMemory("CH")
  for t in range(10):
    memory.update(float(t), [], road_type="expressway")
  confirm(memory, 10.0, signs=["other_sign:motorway_start"])
  for t in range(11, 13):
    memory.update(float(t), [], road_type="expressway")  # the debounce: still the road before the sign
  assert confirm(memory, 14.0, [80], road_type="ramp") == pytest.approx(80 / 3.6)
  for t in range(15, 30):
    assert memory.update(float(t), [], road_type="ramp") == pytest.approx(80 / 3.6)
  assert memory.update(30.0, [], road_type="merge") == pytest.approx(120 / 3.6) and memory.road == "motorway"

def test_memory_a_limit_read_past_a_ramps_limit_is_the_carriageways():
  memory = vision.SpeedLimitMemory("CH")
  confirm(memory, 0.0, signs=["other_sign:motorway_start"])
  assert confirm(memory, 8.0, [60], road_type="ramp") == pytest.approx(60 / 3.6)
  for t in range(20, 28):
    memory.update(float(t), [], road_type="motorway")  # a network without the lane
  assert memory.update(28.0, [], road_type="motorway") == pytest.approx(120 / 3.6)
  assert confirm(memory, 35.0, [100], road_type="motorway") == pytest.approx(100 / 3.6)  # 35 s after the sign, but on the carriageway
  for t in range(36, 80):
    assert memory.update(float(t), [], road_type="motorway") == pytest.approx(100 / 3.6)


def test_memory_a_ramp_off_the_motorway_does_not_take_its_limit_to_an_expressway():
  memory = vision.SpeedLimitMemory("CH")
  confirm(memory, 0.0, signs=["other_sign:motorway_start"])
  for t in range(1, 20):
    memory.update(float(t), [], road_type="motorway")
  assert confirm(memory, 20.0, [80], road_type="ramp") == pytest.approx(80 / 3.6)  # a link to an expressway, 20 s after the sign
  assert memory.update(30.0, [], road_type="merge") == pytest.approx(80 / 3.6)  # which carriageway it joins: not known
  for t in range(31, 38):
    assert memory.update(float(t), [], road_type="expressway") == pytest.approx(80 / 3.6)
  assert memory.update(38.0, [], road_type="expressway") == pytest.approx(100 / 3.6) and memory.road == "expressway"


def test_memory_the_acceleration_lane_counts_for_the_carriageways_scene_limit():
  memory = vision.SpeedLimitMemory("CH")
  assert drive(memory, 0.0, 12.0, road_type="rural") == pytest.approx(80 / 3.6)
  drive(memory, 12.0, 20.0, road_type="ramp")
  assert drive(memory, 20.0, 26.0, road_type="merge") == pytest.approx(80 / 3.6)
  assert drive(memory, 26.0, 29.0, road_type="motorway") == pytest.approx(80 / 3.6)
  assert drive(memory, 29.0, 30.4, road_type="motorway") == pytest.approx(120 / 3.6)  # ROAD_TYPE_SECONDS from the lane's start


def test_memory_a_limit_read_off_the_carriageway_ends_on_it():
  memory = vision.SpeedLimitMemory("CH")
  assert confirm(memory, 0.0, [60], road_type="rural") == pytest.approx(60 / 3.6)  # before the entry; its end sign and the ramp missed
  assert drive(memory, 1.0, 20.0, road_type="rural") == pytest.approx(60 / 3.6)
  assert drive(memory, 20.0, 29.8, road_type="motorway") == pytest.approx(60 / 3.6)  # on the motorway, not long enough
  assert drive(memory, 29.8, 31.0, road_type="motorway") == pytest.approx(120 / 3.6) and memory.road == "motorway"
  assert confirm(memory, 40.0, [80], road_type="motorway") == pytest.approx(80 / 3.6)  # roadworks on the motorway hold
  assert drive(memory, 41.0, 100.0, road_type="motorway") == pytest.approx(80 / 3.6)


def test_memory_a_limit_read_just_before_the_network_sees_the_carriageway_holds():
  memory = vision.SpeedLimitMemory("CH")
  drive(memory, 0.0, 17.0, road_type="rural")
  assert confirm(memory, 17.0, [80], road_type="rural") == pytest.approx(80 / 3.6)  # on the motorway already: the debounce lags
  assert drive(memory, 18.0, 60.0, road_type="motorway") == pytest.approx(80 / 3.6)


def test_memory_ramp_limit_holds_without_the_network():
  memory = vision.SpeedLimitMemory("CH")
  confirm(memory, 0.0, signs=["other_sign:motorway_start"])
  assert confirm(memory, 10.0, [60]) == pytest.approx(60 / 3.6)
  for t in range(12, 200):
    assert memory.update(float(t), []) == pytest.approx(60 / 3.6)  # nothing says the ramp is over: the 60 holds until a sign


def test_memory_motorway_limit_is_not_a_ramp_limit():
  memory = vision.SpeedLimitMemory("CH")
  confirm(memory, 0.0, signs=["other_sign:motorway_start"])
  assert confirm(memory, 300.0, [80], road_type="motorway") == pytest.approx(80 / 3.6)  # roadworks long after the entry
  for t in range(301, 320):
    assert memory.update(float(t), [], road_type="motorway") == pytest.approx(80 / 3.6)  # the motorway does not end it


def test_memory_germany_has_no_general_motorway_limit():
  memory = vision.SpeedLimitMemory("DE")
  confirm(memory, 0.0, signs=["other_sign:motorway_start"])
  assert confirm(memory, 5.0, [80]) == pytest.approx(80 / 3.6)
  for t in range(7, 16):
    memory.update(float(t), [], road_type="motorway")
  assert memory.speed_limit == pytest.approx(0 / 3.6)  # no limit


def test_memory_general_limits_by_country():
  assert confirm(vision.SpeedLimitMemory("LI"), 0.0, signs=["other_sign:motorway_start"]) == pytest.approx(80 / 3.6)
  assert confirm(vision.SpeedLimitMemory("DE"), 0.0, signs=["other_sign:expressway_start"]) == pytest.approx(100 / 3.6)
  assert confirm(vision.SpeedLimitMemory("PL"), 0.0, signs=["other_sign:motorway_start"]) == pytest.approx(140 / 3.6)
  assert vision.SpeedLimitMemory("GB").country == "GB"  # unit support is independent of implicit rules


def test_memory_a_road_sign_keeps_the_limit_in_sweden():
  memory = vision.SpeedLimitMemory("SE")
  assert confirm(memory, 0.0, signs=["other_sign:motorway_start"]) == pytest.approx(110 / 3.6)  # nothing held: the motorway's limit
  assert confirm(memory, 5.0, [90]) == pytest.approx(90 / 3.6)
  assert confirm(memory, 10.0, signs=["other_sign:motorway_end"]) == pytest.approx(90 / 3.6)  # the 90 holds until its end sign
  assert memory.road == "rural"


def drive(memory, start, end, readings=(), signs=(), **context):
  """Updates five times a second from start to end (excluded), as speedvisiond; the limit at the last one."""
  for step in range(round((end - start) * 5)):
    memory.update(start + step * 0.2, list(readings), list(signs), **context)
  return memory.speed_limit


def test_memory_out_of_a_car_park_into_a_town():
  memory = vision.SpeedLimitMemory("CH")
  assert drive(memory, 0.0, 4.0, road_type="parking", v_ego=2.0) == pytest.approx(0 / 3.6)  # not long enough yet
  assert drive(memory, 4.0, 6.0, road_type="parking", v_ego=2.0) == pytest.approx(vision.GARAGE_LIMIT / 3.6) and memory.source == "parking"
  assert drive(memory, 6.0, 9.0, road_type="ramp", v_ego=4.0) == pytest.approx(vision.GARAGE_LIMIT / 3.6)  # not a road with a limit
  assert drive(memory, 9.0, 12.0, road_type="town", v_ego=6.0) == pytest.approx(vision.GARAGE_LIMIT / 3.6)  # not long enough yet
  assert drive(memory, 12.0, 20.0, road_type="town", v_ego=6.0) == pytest.approx(50 / 3.6) and memory.source == "road"  # the street
  assert confirm(memory, 25.0, [30], road_type="town") == pytest.approx(30 / 3.6) and memory.source == "sign"
  assert drive(memory, 26.0, 45.0, road_type="rural", v_ego=15.0) == pytest.approx(30 / 3.6)  # a sign holds whatever the scene


def test_memory_a_moment_of_a_street_inside_a_car_park_keeps_it():
  memory = vision.SpeedLimitMemory("CH")
  assert drive(memory, 0.0, 6.0, road_type="parking", v_ego=2.0) == pytest.approx(vision.GARAGE_LIMIT / 3.6)
  assert drive(memory, 6.0, 8.8, road_type="town", v_ego=5.0) == pytest.approx(vision.GARAGE_LIMIT / 3.6)  # the network unsure: 2.8 s
  assert drive(memory, 8.8, 20.0, road_type="parking", v_ego=0.0) == pytest.approx(vision.GARAGE_LIMIT / 3.6)


def test_memory_the_scene_gives_the_limit_only_without_a_sign():
  memory = vision.SpeedLimitMemory("CH")
  assert memory.update(0.0, [], road_type="town") == pytest.approx(50 / 3.6)  # the start of a drive in a town
  assert drive(memory, 0.2, 9.0, road_type="rural") == pytest.approx(50 / 3.6)  # another road type: not long enough
  assert drive(memory, 9.0, 11.0, road_type="rural") == pytest.approx(80 / 3.6)
  assert drive(memory, 11.0, 15.0, road_type=None) == pytest.approx(80 / 3.6)  # no answer: it holds
  assert confirm(vision.SpeedLimitMemory("DE"), 0.0, road_type="motorway") == pytest.approx(0 / 3.6)  # no general limit there
  assert vision.SpeedLimitMemory("CH").update(0.0, [], road_type="ramp") == 0


def test_memory_a_car_park_needs_a_slow_car_and_drops_the_road_before():
  memory = vision.SpeedLimitMemory("CH")
  assert confirm(memory, 0.0, [80]) == pytest.approx(80 / 3.6)
  assert drive(memory, 1.0, 20.0, road_type="parking", v_ego=20.0) == pytest.approx(80 / 3.6)  # too fast for a car park
  assert drive(memory, 20.0, 24.0, road_type="parking") == pytest.approx(80 / 3.6)  # the speed unknown
  assert drive(memory, 24.0, 30.0, road_type="parking", v_ego=3.0) == pytest.approx(vision.GARAGE_LIMIT / 3.6)
  assert confirm(memory, 40.0, [30], road_type="parking", v_ego=3.0) == pytest.approx(vision.GARAGE_LIMIT / 3.6)  # higher: not taken
  assert confirm(memory, 45.0, [5], road_type="parking", v_ego=3.0) == pytest.approx(5 / 3.6)  # lower: taken
  assert drive(memory, 50.0, 53.0, road_type="rural", v_ego=8.0) == pytest.approx(5 / 3.6)  # still the car park
  assert drive(memory, 53.0, 54.0, road_type="rural", v_ego=8.0) == pytest.approx(80 / 3.6)


def test_memory_a_garage_and_a_car_park_in_the_open():
  memory = vision.SpeedLimitMemory("CH")
  memory.garage_known = True
  assert confirm(memory, 0.0, [50], road_type="town") == pytest.approx(50 / 3.6)
  assert drive(memory, 1.0, 7.0, road_type="parking", v_ego=3.0) == pytest.approx(vision.PARKING_LIMIT / 3.6) and vision.PARKING_LIMIT == 20
  assert drive(memory, 7.0, 11.0, road_type="garage", v_ego=3.0) == pytest.approx(vision.PARKING_LIMIT / 3.6)  # not long enough yet
  assert drive(memory, 11.0, 13.0, road_type="garage", v_ego=2.0) == pytest.approx(vision.GARAGE_LIMIT / 3.6) and vision.GARAGE_LIMIT == 10  # down the ramp
  assert confirm(memory, 20.0, [5], road_type="garage", v_ego=2.0) == pytest.approx(5 / 3.6)  # lower: taken
  assert drive(memory, 21.0, 26.0, road_type="garage", v_ego=2.0) == pytest.approx(5 / 3.6)  # the same garage: the sign holds
  assert drive(memory, 26.0, 32.0, road_type="parking", v_ego=3.0) == pytest.approx(vision.PARKING_LIMIT / 3.6)  # out in the open
  assert drive(memory, 32.0, 43.0, road_type="town", v_ego=8.0) == pytest.approx(50 / 3.6) and memory.source == "road"


def test_memory_a_network_without_garage_keeps_a_car_park_at_the_garage_limit():
  memory = vision.SpeedLimitMemory("CH")
  assert drive(memory, 0.0, 6.0, road_type="parking", v_ego=2.0) == pytest.approx(vision.GARAGE_LIMIT / 3.6)
  memory = vision.SpeedLimitMemory("CH")
  memory.garage_known = True
  assert drive(memory, 0.0, 6.0, road_type="garage", v_ego=2.0) == pytest.approx(vision.GARAGE_LIMIT / 3.6)


def test_car_motion_features():
  motion = vision.CarMotion()
  assert motion.features(0.0).tolist() == [[0.0] * 8]  # nothing known
  for i in range(20):
    t = 10.0 + i * 0.05
    motion.add_device_motion(t, -np.radians(15.0), True)  # z down: a left turn of 15 deg/s
    motion.add_car_state(t, 90.0, i == 0, False, 0.0)
  features = motion.features(11.0)
  assert np.allclose(features, [[0.5, 0.5, 1, 0.5, 1, 1, 0, 1]])
  assert motion.features(12.5).tolist() == [[0.0] * 8]  # too old: unknown again
  later = vision.CarMotion()
  for i in range(20):
    later.add_device_motion(i * 0.05, 0.3, False)  # not valid: the car's own yaw rate
    later.add_car_state(i * 0.05, -360.0, False, True, np.radians(-90.0))
  assert np.allclose(later.features(1.0), [[-3, 3, 1, -2, 1, 0, 1, 1]])


class FakeRoadType:
  """The road-type network's inputs and outputs: the class asked in turn, its memory counting the steps."""
  input_names = ("image", "speed", "sign", "motion", "hidden")

  def __init__(self, answers):
    self.answers = iter(answers)
    self.calls = []

  @staticmethod
  def input_shape(name):
    return (1, 4)

  def __call__(self, **inputs):
    self.calls.append(inputs)
    probabilities = np.zeros((1, 6), dtype=np.float32)
    probabilities[0, ["town", "rural", "expressway", "motorway", "ramp", "parking"].index(next(self.answers))] = 1
    return {"probabilities": probabilities, "hidden_out": inputs["hidden"] + 1, "junction": np.zeros((1, 2))}


def test_road_type_reader_debounces_and_carries_its_memory():
  height, width = 760, 1344
  y, uv = nv12(np.full((height, width), 235), np.full((height // 2, width // 2), 128), np.full((height // 2, width // 2), 128))
  network = FakeRoadType(["parking", "parking", "town", "town", "town", "rural", "town"])
  reader = vision.RoadTypeReader(network, ["town", "rural", "expressway", "motorway", "ramp", "parking"], width, height)
  motion = np.zeros((1, 8), dtype=np.float32)
  answers = [reader.step(float(t), y, uv, 10.0, 30.0, motion) for t in range(5)]
  assert answers == [None, None, None, None, "town"]  # the same class three steps in a row
  assert reader.step(5.0, y, uv, None, 500.0, motion) == "town"  # one other answer: still town
  first, last = network.calls[0], network.calls[-1]
  assert first["image"].shape == (1, 3, 144, 256) and np.all(first["image"] == 1.0)
  assert np.allclose(first["speed"], [[1 / 3, 1]]) and np.allclose(first["sign"], [[0.5, 1]])
  assert np.allclose(last["speed"], [[0, 0]]) and np.allclose(last["sign"], [[2, 0]])
  assert np.all(first["hidden"] == 0) and np.all(last["hidden"] == 5)  # its memory, step after step
  assert reader.step(20.0, y, uv, 10.0, 30.0, motion) is None  # a long pause: from zeros again
  assert np.all(network.calls[-1]["hidden"] == 0)


class FakeBoardRoadType(FakeRoadType):
  """A network with the input board."""
  input_names = ("image", "speed", "sign", "motion", "board", "hidden")


def test_road_type_reader_gives_the_board_to_a_network_that_has_it():
  height, width = 76, 134
  y, uv = nv12(np.full((height, width), 128), np.full((height // 2, width // 2), 128), np.full((height // 2, width // 2), 128))
  motion = np.zeros((1, 8), dtype=np.float32)
  network = FakeBoardRoadType(["town"] * 3)
  reader = vision.RoadTypeReader(network, ["town", "rural", "expressway", "motorway", "ramp", "parking"], width, height)
  reader.step(0.0, y, uv, 10.0, 500.0, motion, 30.0)
  reader.step(1.0, y, uv, 10.0, 500.0, motion)
  assert np.allclose(network.calls[0]["board"], [[0.5, 1]]) and np.allclose(network.calls[0]["sign"], [[2, 0]])
  assert np.allclose(network.calls[1]["board"], [[2, 0]])  # never confirmed: none seen, not zeros
  older = FakeRoadType(["town"])
  vision.RoadTypeReader(older, ["town", "rural", "expressway", "motorway", "ramp", "parking"], width, height).step(
    0.0, y, uv, 10.0, 500.0, motion, 30.0)
  assert "board" not in older.calls[0]  # a network without the input does not get it


def test_road_type_input_is_the_resized_frame():
  rng = np.random.default_rng(0)
  height, width = 76, 134
  y, uv = nv12(rng.integers(16, 236, (height, width)), rng.integers(16, 241, (height // 2, width // 2)),
               rng.integers(16, 241, (height // 2, width // 2)))
  expected = vision.resize(vision.yuv_to_rgb(y, uv, 0, height, 0, width), vision.ROAD_TYPE_WIDTH, vision.ROAD_TYPE_HEIGHT)
  assert np.array_equal(vision.RoadTypeInput(width, height)(y, uv)[0], expected.astype(np.float32) / 255.0)



def test_memory_a_30_zone_seen_keeps_the_car_at_30():
  memory = vision.SpeedLimitMemory("CH")
  assert memory.update(0.0, [], road_type="zone30") == pytest.approx(30 / 3.6)  # the start of a drive inside a zone
  assert drive(memory, 0.2, 9.0, road_type="town") == pytest.approx(30 / 3.6)  # out of it: not long enough yet
  assert drive(memory, 9.0, 11.0, road_type="town") == pytest.approx(50 / 3.6)
  # An explicit 50 is a signed number, even when it equals the town default: the scene cannot replace it.
  memory = vision.SpeedLimitMemory("CH")
  assert confirm(memory, 0.0, [50], road_type="town") == pytest.approx(50 / 3.6) and memory.source == "sign"
  assert drive(memory, 1.0, 10.0, road_type="zone30") == pytest.approx(50 / 3.6)
  assert drive(memory, 10.0, 12.0, road_type="zone30") == pytest.approx(50 / 3.6) and memory.source == "sign"
  # Its end sign: the town again, though the network still sees the zone for a moment.
  assert confirm(memory, 20.0, [0], road_type="zone30") == pytest.approx(50 / 3.6)
  assert drive(memory, 21.0, 26.0, road_type="zone30") == pytest.approx(50 / 3.6)
  assert drive(memory, 26.0, 40.0, road_type="town") == pytest.approx(50 / 3.6)
  # A higher number holds: the network may be wrong about the zone.
  memory = vision.SpeedLimitMemory("CH")
  assert confirm(memory, 0.0, [60], road_type="town") == pytest.approx(60 / 3.6)
  assert drive(memory, 1.0, 30.0, road_type="zone30") == pytest.approx(60 / 3.6)


class FakeZoneRoadType(FakeRoadType):
  """A network of seven classes whose answer is zone30 as sure as asked in turn."""
  CLASSES = ["town", "rural", "expressway", "motorway", "ramp", "parking", "zone30"]

  def __call__(self, **inputs):
    self.calls.append(inputs)
    probabilities = np.zeros((1, 7), dtype=np.float32)
    sure = next(self.answers)
    probabilities[0, 6], probabilities[0, 0] = sure, 1 - sure
    return {"probabilities": probabilities, "hidden_out": np.zeros((1, 4), dtype=np.float32)}


def test_road_type_reader_says_zone30_only_when_sure():
  height, width = 76, 134
  y, uv = nv12(np.full((height, width), 128), np.full((height // 2, width // 2), 128), np.full((height // 2, width // 2), 128))
  network = FakeZoneRoadType([0.85] * 3 + [0.95] * 3)
  reader = vision.RoadTypeReader(network, FakeZoneRoadType.CLASSES, width, height)
  motion = np.zeros((1, 8), dtype=np.float32)
  assert [reader.step(float(t), y, uv, 8.0, 500.0, motion) for t in range(6)] == [None, None, "town", "town", "town", "zone30"]


def test_apply_settings_takes_only_known_numbers_in_bounds(monkeypatch):
  for name, _, _ in vision.SETTINGS.values():
    monkeypatch.setattr(vision, name, getattr(vision, name))
  taken = vision.apply_settings({"zone_confidence": 0.95, "confirm_frames": 3, "road_type_debounce": 2.5,
                                 "min_confidence": 2.0, "merge_seconds": True, "parking_limit": "20", "unknown": 1,
                                 "road_type_seconds": float("nan")})
  assert taken == {"zone_confidence": 0.95, "confirm_frames": 3}
  assert vision.ZONE_CONFIDENCE == 0.95 and vision.CONFIRM_FRAMES == 3 and isinstance(vision.CONFIRM_FRAMES, int)
  assert vision.ROAD_TYPE_DEBOUNCE == 3 and vision.MIN_CONFIDENCE == 0.7 and vision.MERGE_SECONDS == 8.0
  assert vision.apply_settings(None) == {} and vision.apply_settings([1]) == {}


def test_a_limit_read_off_the_carriageway_follows_its_setting(monkeypatch):
  monkeypatch.setattr(vision, "CARRIAGEWAY_LAG", vision.CARRIAGEWAY_LAG)
  assert vision.apply_settings({"carriageway_lag": 0.0}) == {"carriageway_lag": 0.0}
  memory = vision.SpeedLimitMemory("CH")
  drive(memory, 0.0, 17.0, road_type="rural")
  assert confirm(memory, 17.0, [80], road_type="rural") == pytest.approx(80 / 3.6)
  assert drive(memory, 18.0, 30.0, road_type="motorway") == pytest.approx(120 / 3.6)  # no lag: read before the motorway, it ends


def test_the_exit_of_a_car_park_follows_its_setting(monkeypatch):
  monkeypatch.setattr(vision, "PARKING_EXIT_SECONDS", vision.PARKING_EXIT_SECONDS)
  assert vision.apply_settings({"parking_exit_seconds": 8.0}) == {"parking_exit_seconds": 8.0}
  memory = vision.SpeedLimitMemory("CH")
  assert drive(memory, 0.0, 6.0, road_type="parking", v_ego=2.0) == pytest.approx(vision.GARAGE_LIMIT / 3.6)
  assert drive(memory, 6.0, 14.0, road_type="town", v_ego=6.0) == pytest.approx(vision.GARAGE_LIMIT / 3.6)
  assert drive(memory, 14.0, 15.0, road_type="town", v_ego=6.0) == pytest.approx(50 / 3.6)


def test_settings_change_the_memory(monkeypatch):
  monkeypatch.setattr(vision, "CONFIRM_FRAMES", 2)
  vision.apply_settings({"confirm_frames": 1})
  memory = vision.SpeedLimitMemory("CH")
  assert memory.update(0.0, [80]) == pytest.approx(80 / 3.6)


def test_memory_says_which_rule_set_the_limit():
  memory = vision.SpeedLimitMemory("CH")
  assert (memory.limit_reason, memory.limit_state) == ("", "unknown")
  assert memory.update(0.0, [], road_type="town") == pytest.approx(50 / 3.6) and (memory.limit_reason, memory.limit_state) == ("road_type", "limit")
  assert confirm(memory, 5.0, [30], road_type="town") == pytest.approx(30 / 3.6) and memory.limit_reason == "sign"
  assert confirm(memory, 9.0, [0], road_type="town") == pytest.approx(50 / 3.6) and memory.limit_reason == "end_sign"
  assert confirm(memory, 20.0, signs=["other_sign:town_exit"]) == pytest.approx(80 / 3.6) and memory.limit_reason == "road_sign"
  assert drive(memory, 21.0, 300.0) == pytest.approx(80 / 3.6) and memory.limit_reason == "road_sign"  # no timeout: it holds
  assert confirm(memory, 300.0, [80]) == pytest.approx(80 / 3.6) and memory.limit_reason == "sign"  # the same limit, from a sign now

  memory = vision.SpeedLimitMemory("CH")
  confirm(memory, 0.0, signs=["other_sign:expressway_start"])
  assert confirm(memory, 10.0, [60], road_type="ramp") == pytest.approx(60 / 3.6) and memory.limit_reason == "sign"
  assert memory.update(20.0, [], road_type="merge") == pytest.approx(100 / 3.6) and memory.limit_reason == "ramp_end"

  memory = vision.SpeedLimitMemory("CH")
  confirm(memory, 0.0, [60], road_type="rural")
  drive(memory, 1.0, 20.0, road_type="rural")
  assert drive(memory, 20.0, 31.0, road_type="motorway") == pytest.approx(120 / 3.6) and memory.limit_reason == "carriageway"

  memory = vision.SpeedLimitMemory("CH")
  assert drive(memory, 0.0, 6.0, road_type="parking", v_ego=2.0) == pytest.approx(vision.GARAGE_LIMIT / 3.6) and memory.limit_reason == "car_park"
  assert confirm(memory, 10.0, [5], road_type="parking", v_ego=2.0) == pytest.approx(5 / 3.6) and memory.limit_reason == "sign"
  assert drive(memory, 12.0, 20.0, road_type="town", v_ego=6.0) == pytest.approx(50 / 3.6) and memory.limit_reason == "road_type"
  memory = vision.SpeedLimitMemory("CH")
  assert confirm(memory, 0.0, [vision.GARAGE_LIMIT], road_type="town") == pytest.approx(vision.GARAGE_LIMIT / 3.6)
  # The same limit from a car park: its rule now.
  assert drive(memory, 1.0, 8.0, road_type="garage", v_ego=2.0) == pytest.approx(vision.GARAGE_LIMIT / 3.6) and memory.source == "parking"
  assert memory.limit_reason == "car_park"

  memory = vision.SpeedLimitMemory("CH")
  confirm(memory, 0.0, signs=["other_sign:town_entry"], road_type="town")
  assert drive(memory, 1.0, 12.0, road_type="zone30") == pytest.approx(30 / 3.6) and memory.limit_reason == "zone30"
  assert drive(memory, 12.0, 30.0, road_type="zone30") == pytest.approx(30 / 3.6) and memory.limit_reason == "zone30"  # the scene holds it


def test_memory_tells_no_limit_from_an_unknown_one():
  memory = vision.SpeedLimitMemory("DE")
  assert confirm(memory, 0.0, signs=["other_sign:motorway_start"]) == pytest.approx(0 / 3.6)
  assert (memory.limit_reason, memory.limit_state) == ("road_sign", "none")  # the motorway in Germany: no limit
  assert confirm(memory, 5.0, [120]) == pytest.approx(120 / 3.6) and memory.limit_state == "limit"
  assert confirm(memory, 10.0, [0]) == pytest.approx(0 / 3.6) and (memory.limit_reason, memory.limit_state) == ("end_sign", "none")
  assert confirm(vision.SpeedLimitMemory("DE"), 0.0, road_type="motorway") == pytest.approx(0 / 3.6)
  memory = vision.SpeedLimitMemory("CH")
  assert confirm(memory, 0.0, [80]) == pytest.approx(80 / 3.6)
  # The end of a limit on a road of unknown kind: its general limit is not known.
  assert confirm(memory, 5.0, [0]) == pytest.approx(0 / 3.6) and (memory.limit_reason, memory.limit_state) == ("end_sign", "unknown")
  assert drive(memory, 6.0, 10.0, road_type="ramp") == pytest.approx(0 / 3.6) and memory.limit_state == "unknown"
  memory = vision.SpeedLimitMemory("DE")
  confirm(memory, 0.0, [60], road_type="ramp")
  assert confirm(memory, 5.0, [0], road_type="ramp") == pytest.approx(0 / 3.6)
  assert (memory.limit_reason, memory.limit_state) == ("end_sign", "unknown")
  # Then the road type says the road has none: its rule, not the end sign's.
  assert drive(memory, 6.0, 8.0, road_type="motorway") == pytest.approx(0 / 3.6)
  assert (memory.limit_reason, memory.limit_state) == ("road_type", "none")


def test_memory_reason_and_state_on_a_drive():
  """Step by step, a reason of the vocabulary and the state "limit" exactly when there is a limit."""
  memory = vision.SpeedLimitMemory("DE")
  steps = [((0.0, 4.0), {"road_type": "town"}), ((4.0, 6.0), {"readings": [30], "road_type": "town"}),
           ((6.0, 20.0), {"road_type": "parking", "v_ego": 2.0}), ((20.0, 40.0), {"road_type": "rural", "v_ego": 20.0}),
           ((40.0, 41.0), {"signs": ["other_sign:motorway_start"], "road_type": "rural"}),
           ((41.0, 80.0), {"road_type": "motorway", "v_ego": 30.0})]
  for (start, end), context in steps:
    for step in range(round((end - start) * 5)):
      memory.update(start + step * 0.2, context.get("readings", []), context.get("signs", []),
                    road_type=context.get("road_type"), v_ego=context.get("v_ego"))
      assert memory.limit_reason in vision.LIMIT_REASONS and memory.limit_state in vision.LIMIT_STATES
      assert (memory.limit_state == "limit") == (memory.speed_limit > 0)
  assert (memory.speed_limit, memory.limit_state) == (pytest.approx(0 / 3.6), "none")


def reading(name, confidence):
  """A sign of read_signs: its class and how sure."""
  return {"class": name, "confidence": confidence}


def test_raw_reading_is_the_surest_sign_the_memory_can_take():
  assert vision.raw_reading([]) is None
  assert vision.raw_reading([reading("not_a_sign", 0.99), reading("other_sign:give_way", 0.9),
                             reading("maximum_speed:unknown", 0.95), reading("unreadable", 0.8)]) is None
  signs = [reading("maximum_speed:80", 0.5), reading("not_a_sign", 0.99), reading("other_sign:town_entry", 0.6),
           reading("cancellation", 0.55)]
  assert vision.raw_reading(signs) == reading("other_sign:town_entry", 0.6)  # under MIN_CONFIDENCE too
  assert vision.raw_reading(signs[:2] + signs[3:]) == reading("cancellation", 0.55)


def raw(memory):
  return memory.raw_class, memory.raw_speed, round(memory.raw_confidence, 2), memory.raw_taken


def test_memory_keeps_the_last_raw_reading_and_whether_it_took_it():
  memory = vision.SpeedLimitMemory("CH")
  assert raw(memory) == ("", 0.0, 0.0, False)
  unsure = reading("maximum_speed:30", 0.6)  # under MIN_CONFIDENCE: not among the readings
  assert memory.update(0.0, [], raw=unsure) == pytest.approx(0 / 3.6) and memory.update(0.2, [], raw=unsure) == pytest.approx(0 / 3.6)
  assert raw(memory) == ("maximum_speed:30", 30.0, 0.6, False)
  # One frame is not enough: taken in the frame that confirms it, then kept until another reading.
  assert memory.update(1.0, [80], raw=reading("maximum_speed:80", 0.9)) == pytest.approx(0 / 3.6)
  assert raw(memory) == ("maximum_speed:80", 80.0, 0.9, False)
  assert memory.update(1.2, [80], raw=reading("maximum_speed:80", 0.95)) == pytest.approx(80 / 3.6) and raw(memory)[3]
  assert drive(memory, 1.4, 3.0) == pytest.approx(80 / 3.6) and raw(memory) == ("maximum_speed:80", 80.0, 0.95, True)
  memory.update(3.2, [], raw=reading("maximum_speed:80", 0.6))  # read again, unsure: the limit held is not from it
  assert raw(memory) == ("maximum_speed:80", 80.0, 0.6, False)
  memory.update(5.0, [0], raw=reading("cancellation", 0.9))
  assert memory.update(5.2, [0], raw=reading("cancellation", 0.92)) == pytest.approx(0 / 3.6) and raw(memory) == ("cancellation", 0.0, 0.92, True)
  town = reading("other_sign:town_entry", 0.85)
  memory.update(8.0, [], ["other_sign:town_entry"], raw=town)
  assert memory.update(8.2, [], ["other_sign:town_entry"], raw=town) == pytest.approx(50 / 3.6)
  assert raw(memory) == ("other_sign:town_entry", 0.0, 0.85, True)  # a road sign gives no speed
  # Two at once: the limit is the lower, the surer one read is not taken.
  memory.update(10.0, [80, 60], raw=reading("maximum_speed:80", 0.97))
  assert memory.update(10.2, [80, 60], raw=reading("maximum_speed:80", 0.97)) == pytest.approx(60 / 3.6) and not memory.raw_taken


def test_memory_a_raw_reading_confirmed_but_overruled_is_not_taken():
  memory = vision.SpeedLimitMemory("CH")
  drive(memory, 0.0, 6.0, road_type="parking", v_ego=2.0)
  higher = reading("maximum_speed:30", 0.9)
  memory.update(10.0, [30], road_type="parking", v_ego=2.0, raw=higher)
  assert memory.update(10.2, [30], road_type="parking", v_ego=2.0, raw=higher) == pytest.approx(vision.GARAGE_LIMIT / 3.6)
  assert memory.sign_read and raw(memory) == ("maximum_speed:30", 30.0, 0.9, False)  # higher than the car park's
  lower = reading("maximum_speed:5", 0.9)
  memory.update(12.0, [5], road_type="parking", v_ego=2.0, raw=lower)
  assert memory.update(12.2, [5], road_type="parking", v_ego=2.0, raw=lower) == pytest.approx(5 / 3.6)
  assert raw(memory) == ("maximum_speed:5", 5.0, 0.9, True) and memory.limit_reason == "sign"  # lower: it is taken
  memory = vision.SpeedLimitMemory("SE")
  confirm(memory, 0.0, [90])
  end = reading("other_sign:motorway_end", 0.9)
  memory.update(5.0, [], ["other_sign:motorway_end"], raw=end)
  assert memory.update(5.2, [], ["other_sign:motorway_end"], raw=end) == pytest.approx(90 / 3.6) and not memory.raw_taken  # it keeps the 90


FOCAL = 1141.5  # the comma 4's road camera, 1344 x 760


def sign_at(distance: float, diameter: float = 0.8, name: str = "maximum_speed:80", confidence: float = 0.9) -> dict:
  side = FOCAL * diameter / distance
  return {"box": [600.0, 100.0, 600.0 + side * 0.7, 100.0 + side], "class": name, "confidence": confidence}


@pytest.fixture
def no_lead(monkeypatch):
  """The distance as measured, without the lead taken off for the screen (DISTANCE_LEAD_SECONDS, _METRES)."""
  monkeypatch.setattr(vision, "DISTANCE_LEAD_SECONDS", 0.0)
  monkeypatch.setattr(vision, "DISTANCE_LEAD_METRES", 0.0)


def approach(signs_at, road: str | None = "rural", speed: float = 20.0, seconds: float = 6.0, start: float = 90.0):
  """A drive at speed (m/s) towards a sign start metres ahead, a frame every 0.2 s: signs_at(distance) gives the
  frame's signs. The distance shown at each frame, with the true one."""
  sign_distance = vision.SignDistance(FOCAL)
  shown = []
  for step in range(int(seconds * 100) + 1):
    t = step / 100
    sign_distance.add_speed(t, speed)
    if step % 20 == 0:
      true = start - sign_distance.odometer
      shown.append((true, sign_distance.update(t, signs_at(true), road)))
  return shown


@pytest.mark.usefixtures("no_lead")
def test_sign_distance_from_the_size_of_the_road():
  shown = approach(lambda d: [sign_at(d)] if d > 15 else [])
  assert all(estimate == pytest.approx(true, abs=0.01) for true, estimate in shown if true > 0)
  # Out of the frame from 15 m, the metres driven are taken off; passed, 0.
  assert [estimate for true, estimate in shown if true <= 0] == [0.0] * len([true for true, _ in shown if true <= 0])
  assert shown[0][1] == pytest.approx(90, abs=0.01)


@pytest.mark.usefixtures("no_lead")
def test_sign_distance_corrects_a_wrong_size_as_the_sign_grows():
  # A 60 cm sign on a road taken as rural (80 cm): first seen a third too far, then corrected.
  shown = [(true, estimate) for true, estimate in approach(lambda d: [sign_at(d, 0.6)] if 15 < d < 85 else [])
           if 15 < true < 85]
  assert shown[0][1] == pytest.approx(shown[0][0] * 0.8 / 0.6, rel=0.01)
  assert shown[-1][1] == pytest.approx(shown[-1][0], abs=1.5)
  errors = [abs(estimate - true) for true, estimate in shown]
  assert errors == sorted(errors, reverse=True)


@pytest.mark.usefixtures("no_lead")
def test_sign_distance_takes_the_size_of_the_road_kind():
  town = approach(lambda d: [sign_at(d, 0.6)] if d < 60 else [], road="town")
  motorway = approach(lambda d: [sign_at(d, 1.2)] if d < 60 else [], road="motorway")
  first = [next(estimate for true, estimate in shown if estimate > 0) for shown in (town, motorway)]
  assert first == [pytest.approx(next(true for true, _ in town if true < 60), abs=0.01),
                   pytest.approx(next(true for true, _ in motorway if true < 60), abs=0.01)]


@pytest.mark.usefixtures("no_lead")
def test_sign_distance_reads_only_round_and_road_signs_read_sure_enough():
  for name, confidence in (("road_marking:50", 0.9), ("variable_display:unknown", 0.9), ("other_sign:town_entry", 0.9),
                           ("maximum_speed:80", 0.5), ("variable_display:80", 0.5), ("other_sign:motorway_end", 0.5)):
    shown = approach(lambda d, name=name, confidence=confidence: [sign_at(d, name=name, confidence=confidence)])
    assert all(estimate == 0 for _, estimate in shown)
  assert approach(lambda d: [sign_at(d, name="cancellation")])[0][1] == pytest.approx(90, abs=0.01)


def hours_sign_at(distance: float, name: str = "maximum_speed_hours:30") -> dict:
  """A disc of 0.8 m with its plate below it: the box is as wide as the disc and half as tall again."""
  side = FOCAL * 0.8 / distance
  return {"box": [600.0, 100.0, 600.0 + side, 100.0 + 1.5 * side], "class": name, "confidence": 0.9}


@pytest.mark.usefixtures("no_lead")
def test_sign_distance_to_a_number_with_hours_from_its_width():
  shown = approach(lambda d: [hours_sign_at(d)] if d > 15 else [])
  assert all(estimate == pytest.approx(true, abs=0.01) for true, estimate in shown if true > 0)


@pytest.mark.usefixtures("no_lead")
def test_sign_distance_starts_again_when_the_plate_is_read():
  shown = approach(lambda d: [hours_sign_at(d, "maximum_speed:30" if d > 50 else "maximum_speed_hours:30")] if d > 15 else [])
  assert all(estimate == pytest.approx(true, abs=0.01) for true, estimate in shown if 15 < true <= 50)


@pytest.mark.usefixtures("no_lead")
def test_sign_distance_to_a_gantry_led_limit_from_its_own_size():
  # On a motorway, where a round sign is taken as 1.2 m: the LED display keeps its own diameter.
  diameter = vision.VARIABLE_DISPLAY_DIAMETER
  shown = approach(lambda d: [sign_at(d, diameter, "variable_display:100")] if d > 15 else [], road="motorway")
  assert all(estimate == pytest.approx(true, abs=0.01) for true, estimate in shown if true > 0)


@pytest.mark.usefixtures("no_lead")
def test_sign_distance_to_a_motorway_sign_or_its_end_from_its_height():
  for name in vision.ROAD_SIGN_HEIGHTS:
    height = vision.ROAD_SIGN_HEIGHTS[name]
    shown = approach(lambda d, name=name, height=height: [sign_at(d, height, name)] if d > 15 else [], road="motorway")
    assert all(estimate == pytest.approx(true, abs=0.01) for true, estimate in shown if true > 0)
  # Its formats differ (a post's, a gantry's): first seen at the usual height, corrected as it grows.
  for height in (1.3, 2.1):
    shown = [(true, estimate) for true, estimate in
             approach(lambda d, height=height: [sign_at(d, height, "other_sign:motorway_end")] if 15 < d < 85 else [])
             if 15 < true < 85]
    assert shown[0][1] == pytest.approx(shown[0][0] * 1.7 / height, rel=0.01)
    assert shown[-1][1] == pytest.approx(shown[-1][0], abs=2.0)


@pytest.mark.usefixtures("no_lead")
def test_sign_distance_takes_the_nearest_by_its_size_not_the_largest():
  # A round 80 at d and the end of the motorway 20 m farther on, larger on the camera: the 80 is the nearer.
  shown = approach(lambda d: [sign_at(d + 20, 1.7, "other_sign:motorway_end"), sign_at(d)] if d > 15 else [])
  assert all(estimate == pytest.approx(true, abs=0.01) for true, estimate in shown if true > 15)


@pytest.mark.usefixtures("no_lead")
def test_sign_distance_takes_the_nearest_sign_and_starts_again_on_another():
  near = approach(lambda d: [sign_at(d + 30), sign_at(d)] if d > 15 else [])
  assert near[0][1] == pytest.approx(90, abs=0.01)
  # A sign of another class farther on: its own distance, from its own size.
  shown = approach(lambda d: [sign_at(d)] if d > 40 else [sign_at(d + 50, name="maximum_speed:60")] if d > 15 else [])
  after = [(true, estimate) for true, estimate in shown if 15 < true <= 40]
  assert all(estimate == pytest.approx(true + 50, abs=0.01) for true, estimate in after)


@pytest.mark.usefixtures("no_lead")
def test_sign_distance_ends_some_seconds_after_the_last_sighting():
  # Stopped 30 m before a sign seen once: it holds, then goes after DISTANCE_SECONDS.
  sign_distance = vision.SignDistance(FOCAL)
  assert sign_distance.update(0.0, [sign_at(30)], "rural") == pytest.approx(30)
  assert sign_distance.update(vision.DISTANCE_SECONDS, [], "rural") == pytest.approx(30)
  assert sign_distance.update(vision.DISTANCE_SECONDS + 0.2, [], "rural") == 0
  assert sign_distance.side == 0


def test_sign_distance_shown_is_short_by_the_lead():
  # 0.34 s x 20 m/s + 1.8 m: the screen's zero comes that much before the sign, where it leaves the picture.
  lead = vision.DISTANCE_LEAD_SECONDS * 20.0 + vision.DISTANCE_LEAD_METRES
  shown = approach(lambda d: [sign_at(d)] if d > 15 else [])
  assert all(estimate == pytest.approx(max(0.0, true - lead), abs=0.01) for true, estimate in shown)
  assert any(0 < true <= lead and estimate == 0 for true, estimate in shown)


def test_sign_distance_stopped_takes_off_only_the_metres():
  sign_distance = vision.SignDistance(FOCAL)
  sign_distance.add_speed(0.0, 0.0)
  assert sign_distance.update(0.0, [sign_at(30)], "rural") == pytest.approx(30 - vision.DISTANCE_LEAD_METRES)
  assert vision.SignDistance(FOCAL).update(0.0, [sign_at(1.5)], "rural") == 0


def test_sign_distance_lead_and_display_size_are_settings(monkeypatch):
  for name in ("DISTANCE_LEAD_SECONDS", "DISTANCE_LEAD_METRES", "VARIABLE_DISPLAY_DIAMETER"):
    monkeypatch.setattr(vision, name, getattr(vision, name))
  taken = vision.apply_settings({"distance_lead_seconds": 0.5, "distance_lead_metres": 20.0,
                                 "variable_display_diameter": 1.6})
  assert taken == {"distance_lead_seconds": 0.5, "variable_display_diameter": 1.6}
  assert (vision.DISTANCE_LEAD_SECONDS, vision.DISTANCE_LEAD_METRES, vision.VARIABLE_DISPLAY_DIAMETER) == (0.5, 6.8, 1.6)


def test_sign_distance_needs_the_focal_length():
  assert vision.SignDistance(0.0).update(0.0, [sign_at(30)], "rural") == 0


def held(limit_ahead, limit=80 / 3.6, reason="sign"):
  """limit_ahead holding limit, set with no sign ahead."""
  limit_ahead.update(limit, reason, 0.0, "", 0.0)
  return limit_ahead


def test_limit_ahead_the_first_limit_applies_at_once():
  limit_ahead = vision.LimitAhead()
  assert limit_ahead.update(80 / 3.6, "sign", 30.0, "maximum_speed:80", 0.0) == 80 / 3.6
  assert (limit_ahead.ahead, limit_ahead.distance) == (0, 0)


@pytest.mark.parametrize("reason, sign_class", [("sign", "maximum_speed:120"), ("sign", "maximum_speed_hours:120"),
                                                ("sign", "variable_display:120"), ("end_sign", "cancellation"),
                                                ("road_sign", "other_sign:motorway_start")])
def test_limit_ahead_a_sign_holds_until_the_car_gets_there(reason, sign_class):
  limit_ahead = held(vision.LimitAhead())
  assert limit_ahead.update(120 / 3.6, reason, 30.0, sign_class, 100.0) == 80 / 3.6
  assert (limit_ahead.ahead, limit_ahead.distance) == (120 / 3.6, 30.0)
  assert limit_ahead.update(120 / 3.6, reason, 12.0, sign_class, 118.0) == 80 / 3.6
  assert limit_ahead.distance == 12.0
  assert limit_ahead.update(120 / 3.6, reason, 0.0, sign_class, 130.0) == 120 / 3.6
  assert (limit_ahead.ahead, limit_ahead.distance) == (0, 0)


def test_limit_ahead_the_end_of_a_limit_with_no_general_limit_holds_too():
  limit_ahead = held(vision.LimitAhead())
  assert limit_ahead.update(0.0, "end_sign", 30.0, "cancellation", 100.0) == 80 / 3.6
  assert limit_ahead.ahead == 0
  assert limit_ahead.update(0.0, "end_sign", 0.0, "cancellation", 130.0) == 0


@pytest.mark.parametrize("reason, sign_distance, sign_class", [
  ("road_type", 30.0, "maximum_speed:120"),  # another rule
  ("carriageway", 30.0, "maximum_speed:120"),
  ("sign", 0.0, "maximum_speed:120"),  # the sign is at the car, or the camera is not known
  ("road_sign", 30.0, "maximum_speed:120"),  # a town sign has no distance: the one shown is of another sign
  ("end_sign", 30.0, "maximum_speed:120"),
  ("sign", 30.0, ""),
])
def test_limit_ahead_applies_at_once_with_no_sign_ahead(reason, sign_distance, sign_class):
  limit_ahead = held(vision.LimitAhead())
  assert limit_ahead.update(120 / 3.6, reason, sign_distance, sign_class, 100.0) == 120 / 3.6
  assert (limit_ahead.ahead, limit_ahead.distance) == (0, 0)


def test_limit_ahead_counts_the_metres_when_the_sign_is_lost_or_another_is_seen():
  limit_ahead = held(vision.LimitAhead())
  limit_ahead.update(120 / 3.6, "sign", 30.0, "maximum_speed:120", 100.0)
  assert limit_ahead.update(120 / 3.6, "sign", 150.0, "maximum_speed:60", 110.0) == 80 / 3.6
  assert limit_ahead.distance == 20.0
  assert limit_ahead.update(120 / 3.6, "sign", 140.0, "maximum_speed:60", 130.0) == 120 / 3.6


def test_limit_ahead_the_same_class_further_on_is_the_next_sign():
  limit_ahead = held(vision.LimitAhead())
  limit_ahead.update(120 / 3.6, "sign", 30.0, "maximum_speed:120", 100.0)
  limit_ahead.update(120 / 3.6, "sign", 33.0, "maximum_speed:120", 101.0)  # the size wavers: the same sign
  assert limit_ahead.distance == 33.0
  assert limit_ahead.update(120 / 3.6, "sign", 200.0, "maximum_speed:120", 140.0) == 120 / 3.6


def test_limit_ahead_stopped_it_waits_for_the_sign():
  limit_ahead = held(vision.LimitAhead())
  limit_ahead.update(120 / 3.6, "sign", 30.0, "maximum_speed:120", 100.0)
  for _ in range(5):
    assert limit_ahead.update(120 / 3.6, "sign", 30.0, "maximum_speed:120", 100.0) == 80 / 3.6


def test_limit_ahead_a_new_reading_takes_its_place():
  limit_ahead = held(vision.LimitAhead())
  limit_ahead.update(60 / 3.6, "sign", 30.0, "maximum_speed:60", 100.0)
  assert limit_ahead.update(40 / 3.6, "sign", 50.0, "maximum_speed:40", 110.0) == 80 / 3.6
  assert (limit_ahead.ahead, limit_ahead.distance) == (40 / 3.6, 50.0)


def test_limit_ahead_ends_when_the_memory_goes_back_to_the_limit_held():
  limit_ahead = held(vision.LimitAhead())
  limit_ahead.update(60 / 3.6, "sign", 30.0, "maximum_speed:60", 100.0)
  assert limit_ahead.update(80 / 3.6, "sign", 20.0, "maximum_speed:80", 110.0) == 80 / 3.6
  assert (limit_ahead.ahead, limit_ahead.distance) == (0, 0)


def test_limit_ahead_another_rule_during_the_wait_applies_at_once():
  limit_ahead = held(vision.LimitAhead())
  limit_ahead.update(60 / 3.6, "sign", 30.0, "maximum_speed:60", 100.0)
  assert limit_ahead.update(30 / 3.6, "car_park", 20.0, "maximum_speed:60", 110.0) == 30 / 3.6
  assert (limit_ahead.ahead, limit_ahead.distance) == (0, 0)


def test_limit_ahead_with_a_sign_distance_from_the_camera():
  """The limit held until the true sign, through SignDistance's real distance."""
  limit_ahead = held(vision.LimitAhead())
  sign_distance = vision.SignDistance(FOCAL)
  released = None
  for step in range(int(6 * 100) + 1):
    t = step / 100
    sign_distance.add_speed(t, 20.0)
    if step % 20 == 0:
      true = 90.0 - sign_distance.odometer
      signs = [sign_at(true, name="maximum_speed:120")] if true > 10 else []
      shown = sign_distance.update(t, signs, "rural")
      limit = limit_ahead.update(120 / 3.6, "sign", shown, sign_distance.sign_class, sign_distance.odometer)
      if t < 0.5:
        assert limit == 80 / 3.6 or not shown
      if limit == 120 / 3.6 and released is None:
        released = true
  assert released is not None and -5 < released < 20


def test_odometer_skips_gaps_in_the_car_state():
  sign_distance = vision.SignDistance(FOCAL)
  for t, v in ((0.0, 10.0), (0.01, 10.0), (0.02, 10.0), (5.0, 10.0), (5.01, -1.0), (5.02, 0.0)):
    sign_distance.add_speed(t, v)
  assert sign_distance.odometer == pytest.approx(0.3)
