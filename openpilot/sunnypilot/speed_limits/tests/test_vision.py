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


@pytest.mark.parametrize("name, speed", [("maximum_speed:80", 80), ("variable_display:100", 100), ("zone:30", 30),
                                         ("road_marking:30", 30), ("cancellation", 0), ("maximum_speed:unknown", None),
                                         ("zone:unknown", None), ("other_sign:give_way", None), ("not_a_sign", None),
                                         ("unreadable", None)])
def test_class_speed(name, speed):
  assert vision.class_speed(name) == speed


def test_read_signs():
  output = np.zeros((1, 5, 36, 112), dtype=np.float32)
  output[0, 0, 10, 20], output[0, 0, 20, 50] = 0.9, 0.8
  output[0, 1:3] = np.log(20)
  classes = ["maximum_speed:50", "maximum_speed:80", "not_a_sign"]
  answers = iter([np.array([[5.0, 0.0, 0.0]]), np.array([[0.0, 0.5, 0.0]])])
  y, uv = nv12(np.full((760, 1344), 128), np.full((380, 672), 128), np.full((380, 672), 128))
  signs = vision.read_signs(y, uv, vision.SignNetInput(1344, 760), lambda x: output, lambda x: next(answers), classes)
  assert [(sign["class"], sign["speed"]) for sign in signs] == [("maximum_speed:50", 50), ("maximum_speed:80", None)]


def test_memory_confirms_and_holds():
  memory = vision.SpeedLimitMemory()
  assert memory.update(0.0, [60]) == 0  # one frame is not enough
  assert memory.update(0.2, [60]) == 60 and memory.sign_read
  assert memory.update(0.4, []) == 60 and not memory.sign_read  # held without a sign
  assert memory.update(10.0, [80]) == 60
  assert memory.update(12.0, [80]) == 60  # too far apart
  assert memory.update(12.4, [80]) == 80
  assert memory.update(15.0, [50, 50]) == 80  # a value counts once a frame
  assert memory.update(20.0, [0]) == 80
  assert memory.update(20.2, [0]) == 0  # the end of the limit clears it


def test_memory_two_signs_at_once():
  memory = vision.SpeedLimitMemory()
  memory.update(0.0, [80, 60])
  assert memory.update(0.2, [80, 60]) == 60  # a tie: the lower
  memory.update(5.0, [100, 120])
  memory.update(5.2, [120])
  assert memory.update(5.4, [100, 120]) == 120  # the one seen most
  memory.update(9.0, [0, 60])
  assert memory.update(9.2, [0, 60]) == 60  # the end of a limit beside roadworks: not the general limit


def confirm(memory, at, readings=(), signs=(), **context):
  memory.update(at, list(readings), list(signs), **context)
  return memory.update(at + 0.2, list(readings), list(signs), **context)


def test_memory_road_signs_set_the_general_limit():
  memory = vision.SpeedLimitMemory()
  assert confirm(memory, 0.0, signs=["other_sign:town_entry"]) == 50
  assert confirm(memory, 5.0, [30]) == 30
  assert confirm(memory, 9.0, [0]) == 50  # the end of the 30 in a town: the town's 50
  assert confirm(memory, 20.0, signs=["other_sign:town_exit"]) == 80
  assert confirm(memory, 30.0, signs=["other_sign:give_way"]) == 80  # not a sign of the road's kind
  assert not memory.sign_read
  assert confirm(memory, 40.0, signs=["other_sign:town_exit"]) == 80 and memory.sign_read  # the same limit read again
  assert vision.SpeedLimitMemory("IT").update(0.0, [], ["other_sign:motorway_start"]) == 0
  assert confirm(vision.SpeedLimitMemory("IT"), 0.0, signs=["other_sign:motorway_start"]) == 130


def test_memory_ramp_limit_ends_by_the_road_type_network():
  memory = vision.SpeedLimitMemory()
  assert confirm(memory, 0.0, signs=["other_sign:motorway_start"], road_type="motorway") == 120
  assert confirm(memory, 10.0, [60], road_type="motorway") == 60  # the ramp's 60, by the sign, whatever the network
  for t in range(12, 20):
    assert memory.update(float(t), [], road_type="ramp") == 60  # still on the ramp
  for t in range(20, 27):
    assert memory.update(float(t), [], road_type="motorway") == 60  # on the carriageway, not long enough
  assert memory.update(27.0, [], road_type="town") == 60  # a doubt starts the count again
  for t in range(28, 36):
    memory.update(float(t), [], road_type="motorway")
  assert memory.update(36.0, [], road_type="motorway") == 120
  assert memory.road == "motorway"


def test_memory_ramp_limit_far_from_the_sign_by_the_network():
  memory = vision.SpeedLimitMemory()
  confirm(memory, 0.0, signs=["other_sign:motorway_start"])
  assert confirm(memory, 100.0, [60], road_type="ramp") == 60  # far from the sign, the network says ramp
  for t in range(102, 110):
    memory.update(float(t), [], road_type="expressway")
  assert memory.update(110.0, [], road_type="expressway") == 100


def test_memory_ramp_limit_ends_at_the_acceleration_lane():
  memory = vision.SpeedLimitMemory()
  confirm(memory, 0.0, signs=["other_sign:expressway_start"])
  assert confirm(memory, 10.0, [60], road_type="ramp") == 60
  for t in range(11, 20):
    assert memory.update(float(t), [], road_type="ramp") == 60
  # The expressway's 100 from the start of the acceleration lane, not 8 s into the carriageway.
  assert memory.update(20.0, [], road_type="merge") == 100 and memory.road == "expressway" and memory.source == "sign"
  for t in range(21, 40):
    assert memory.update(float(t), [], road_type="rural") == 100  # held as a sign's: the network's doubts change nothing


def test_memory_acceleration_lane_of_an_unknown_carriageway_waits_for_it():
  memory = vision.SpeedLimitMemory()
  assert confirm(memory, 0.0, [60], road_type="ramp") == 60  # no motorway or expressway sign seen
  for t in range(2, 8):
    assert memory.update(float(t), [], road_type="merge") == 60  # which carriageway: not known yet
  # The lane counts towards MERGE_SECONDS: the carriageway named 8 s after the lane started.
  assert memory.update(8.0, [], road_type="motorway") == 60
  assert memory.update(10.0, [], road_type="motorway") == 120 and memory.road == "motorway"


def test_memory_acceleration_lane_far_from_the_sign_is_not_known():
  memory = vision.SpeedLimitMemory()
  confirm(memory, 0.0, signs=["other_sign:motorway_start"])
  assert confirm(memory, 100.0, [60], road_type="ramp") == 60  # another ramp, long after the motorway's sign
  assert memory.update(105.0, [], road_type="merge") == 60
  assert memory.update(113.0, [], road_type="expressway") == 100


def test_memory_acceleration_lane_without_a_ramp_limit_changes_nothing():
  memory = vision.SpeedLimitMemory()
  confirm(memory, 0.0, signs=["other_sign:motorway_start"])
  assert confirm(memory, 300.0, [80], road_type="motorway") == 80  # roadworks long after the entry
  for t in range(301, 320):
    assert memory.update(float(t), [], road_type="merge") == 80  # another entry joins the motorway here


def test_memory_a_limit_read_on_the_acceleration_lane_is_the_carriageways():
  memory = vision.SpeedLimitMemory()
  confirm(memory, 0.0, signs=["other_sign:motorway_start"])
  assert confirm(memory, 8.0, [60], road_type="ramp") == 60
  assert memory.update(20.0, [], road_type="merge") == 120
  assert confirm(memory, 25.0, [100], road_type="merge") == 100  # the motorway's own 100, beside the lane
  for t in range(26, 70):
    assert memory.update(float(t), [], road_type="merge" if t < 30 else "motorway") == 100



def test_memory_a_limit_read_on_the_lane_without_a_ramps_number_is_the_carriageways():
  memory = vision.SpeedLimitMemory()
  confirm(memory, 0.0, signs=["other_sign:motorway_start"])
  for t in range(1, 20):
    memory.update(float(t), [], road_type="ramp")  # no number on the ramp
  memory.update(20.0, [], road_type="merge")
  assert confirm(memory, 22.0, [100], road_type="merge") == 100
  for t in range(23, 60):
    assert memory.update(float(t), [], road_type="merge" if t < 30 else "motorway") == 100


def test_memory_an_expressways_link_to_the_motorway_ends_its_limit_on_the_lane():
  memory = vision.SpeedLimitMemory()
  for t in range(0, 10):
    memory.update(float(t), [], road_type="expressway")
  confirm(memory, 10.0, signs=["other_sign:motorway_start"])
  for t in range(11, 13):
    memory.update(float(t), [], road_type="expressway")  # the debounce: still the road before the sign
  assert confirm(memory, 14.0, [80], road_type="ramp") == 80
  for t in range(15, 30):
    assert memory.update(float(t), [], road_type="ramp") == 80
  assert memory.update(30.0, [], road_type="merge") == 120 and memory.road == "motorway"

def test_memory_a_limit_read_past_a_ramps_limit_is_the_carriageways():
  memory = vision.SpeedLimitMemory()
  confirm(memory, 0.0, signs=["other_sign:motorway_start"])
  assert confirm(memory, 8.0, [60], road_type="ramp") == 60
  for t in range(20, 28):
    memory.update(float(t), [], road_type="motorway")  # a network without the lane
  assert memory.update(28.0, [], road_type="motorway") == 120
  assert confirm(memory, 35.0, [100], road_type="motorway") == 100  # 35 s after the sign, but on the carriageway
  for t in range(36, 80):
    assert memory.update(float(t), [], road_type="motorway") == 100


def test_memory_a_ramp_off_the_motorway_does_not_take_its_limit_to_an_expressway():
  memory = vision.SpeedLimitMemory()
  confirm(memory, 0.0, signs=["other_sign:motorway_start"])
  for t in range(1, 20):
    memory.update(float(t), [], road_type="motorway")
  assert confirm(memory, 20.0, [80], road_type="ramp") == 80  # a link to an expressway, 20 s after the sign
  assert memory.update(30.0, [], road_type="merge") == 80  # which carriageway it joins: not known
  for t in range(31, 38):
    assert memory.update(float(t), [], road_type="expressway") == 80
  assert memory.update(38.0, [], road_type="expressway") == 100 and memory.road == "expressway"


def test_memory_the_acceleration_lane_counts_for_the_carriageways_scene_limit():
  memory = vision.SpeedLimitMemory()
  assert drive(memory, 0.0, 12.0, road_type="rural") == 80
  drive(memory, 12.0, 20.0, road_type="ramp")
  assert drive(memory, 20.0, 26.0, road_type="merge") == 80
  assert drive(memory, 26.0, 29.0, road_type="motorway") == 80
  assert drive(memory, 29.0, 30.4, road_type="motorway") == 120  # ROAD_TYPE_SECONDS from the lane's start


def test_memory_a_limit_read_off_the_carriageway_ends_on_it():
  memory = vision.SpeedLimitMemory()
  assert confirm(memory, 0.0, [60], road_type="rural") == 60  # before the entry; its end sign and the ramp missed
  assert drive(memory, 1.0, 20.0, road_type="rural") == 60
  assert drive(memory, 20.0, 29.8, road_type="motorway") == 60  # on the motorway, not long enough
  assert drive(memory, 29.8, 31.0, road_type="motorway") == 120 and memory.road == "motorway"
  assert confirm(memory, 40.0, [80], road_type="motorway") == 80  # roadworks on the motorway hold
  assert drive(memory, 41.0, 100.0, road_type="motorway") == 80


def test_memory_a_limit_read_just_before_the_network_sees_the_carriageway_holds():
  memory = vision.SpeedLimitMemory()
  drive(memory, 0.0, 17.0, road_type="rural")
  assert confirm(memory, 17.0, [80], road_type="rural") == 80  # on the motorway already: the debounce lags
  assert drive(memory, 18.0, 60.0, road_type="motorway") == 80


def test_memory_ramp_limit_holds_without_the_network():
  memory = vision.SpeedLimitMemory()
  confirm(memory, 0.0, signs=["other_sign:motorway_start"])
  assert confirm(memory, 10.0, [60]) == 60
  for t in range(12, 200):
    assert memory.update(float(t), []) == 60  # nothing says the ramp is over: the 60 holds until a sign


def test_memory_motorway_limit_is_not_a_ramp_limit():
  memory = vision.SpeedLimitMemory()
  confirm(memory, 0.0, signs=["other_sign:motorway_start"])
  assert confirm(memory, 300.0, [80], road_type="motorway") == 80  # roadworks long after the entry
  for t in range(301, 320):
    assert memory.update(float(t), [], road_type="motorway") == 80  # the motorway does not end it


def test_memory_germany_has_no_general_motorway_limit():
  memory = vision.SpeedLimitMemory("DE")
  confirm(memory, 0.0, signs=["other_sign:motorway_start"])
  assert confirm(memory, 5.0, [80]) == 80
  for t in range(7, 16):
    memory.update(float(t), [], road_type="motorway")
  assert memory.speed_limit == 0  # no limit


def test_memory_general_limits_by_country():
  assert confirm(vision.SpeedLimitMemory("LI"), 0.0, signs=["other_sign:motorway_start"]) == 80
  assert confirm(vision.SpeedLimitMemory("DE"), 0.0, signs=["other_sign:expressway_start"]) == 100
  assert confirm(vision.SpeedLimitMemory("PL"), 0.0, signs=["other_sign:motorway_start"]) == 140
  assert vision.SpeedLimitMemory("GB").country == vision.DEFAULT_COUNTRY  # mph: not handled


def test_memory_a_road_sign_keeps_the_limit_in_sweden():
  memory = vision.SpeedLimitMemory("SE")
  assert confirm(memory, 0.0, signs=["other_sign:motorway_start"]) == 110  # nothing held: the motorway's limit
  assert confirm(memory, 5.0, [90]) == 90
  assert confirm(memory, 10.0, signs=["other_sign:motorway_end"]) == 90  # the 90 holds until its end sign
  assert memory.road == "rural"


def drive(memory, start, end, readings=(), signs=(), **context):
  """Updates five times a second from start to end (excluded), as speedvisiond; the limit at the last one."""
  for step in range(round((end - start) * 5)):
    memory.update(start + step * 0.2, list(readings), list(signs), **context)
  return memory.speed_limit


def test_memory_out_of_a_car_park_into_a_town():
  memory = vision.SpeedLimitMemory()
  assert drive(memory, 0.0, 4.0, road_type="parking", v_ego=2.0) == 0  # not long enough yet
  assert drive(memory, 4.0, 6.0, road_type="parking", v_ego=2.0) == vision.GARAGE_LIMIT and memory.source == "parking"
  assert drive(memory, 6.0, 9.0, road_type="ramp", v_ego=4.0) == vision.GARAGE_LIMIT  # not a road with a limit
  assert drive(memory, 9.0, 12.0, road_type="town", v_ego=6.0) == vision.GARAGE_LIMIT  # not long enough yet
  assert drive(memory, 12.0, 20.0, road_type="town", v_ego=6.0) == 50 and memory.source == "road"  # the street
  assert confirm(memory, 25.0, [30], road_type="town") == 30 and memory.source == "sign"
  assert drive(memory, 26.0, 45.0, road_type="rural", v_ego=15.0) == 30  # a sign holds whatever the scene


def test_memory_a_moment_of_a_street_inside_a_car_park_keeps_it():
  memory = vision.SpeedLimitMemory()
  assert drive(memory, 0.0, 6.0, road_type="parking", v_ego=2.0) == vision.GARAGE_LIMIT
  assert drive(memory, 6.0, 8.8, road_type="town", v_ego=5.0) == vision.GARAGE_LIMIT  # the network unsure: 2.8 s
  assert drive(memory, 8.8, 20.0, road_type="parking", v_ego=0.0) == vision.GARAGE_LIMIT


def test_memory_the_scene_gives_the_limit_only_without_a_sign():
  memory = vision.SpeedLimitMemory()
  assert memory.update(0.0, [], road_type="town") == 50  # the start of a drive in a town
  assert drive(memory, 0.2, 9.0, road_type="rural") == 50  # another road type: not long enough
  assert drive(memory, 9.0, 11.0, road_type="rural") == 80
  assert drive(memory, 11.0, 15.0, road_type=None) == 80  # no answer: it holds
  assert confirm(vision.SpeedLimitMemory("DE"), 0.0, road_type="motorway") == 0  # no general limit there
  assert vision.SpeedLimitMemory().update(0.0, [], road_type="ramp") == 0


def test_memory_a_car_park_needs_a_slow_car_and_drops_the_road_before():
  memory = vision.SpeedLimitMemory()
  assert confirm(memory, 0.0, [80]) == 80
  assert drive(memory, 1.0, 20.0, road_type="parking", v_ego=20.0) == 80  # too fast for a car park
  assert drive(memory, 20.0, 24.0, road_type="parking") == 80  # the speed unknown
  assert drive(memory, 24.0, 30.0, road_type="parking", v_ego=3.0) == vision.GARAGE_LIMIT
  assert confirm(memory, 40.0, [30], road_type="parking", v_ego=3.0) == vision.GARAGE_LIMIT  # higher: not taken
  assert confirm(memory, 45.0, [5], road_type="parking", v_ego=3.0) == 5  # lower: taken
  assert drive(memory, 50.0, 53.0, road_type="rural", v_ego=8.0) == 5  # still the car park
  assert drive(memory, 53.0, 54.0, road_type="rural", v_ego=8.0) == 80


def test_memory_a_garage_and_a_car_park_in_the_open():
  memory = vision.SpeedLimitMemory()
  memory.garage_known = True
  assert confirm(memory, 0.0, [50], road_type="town") == 50
  assert drive(memory, 1.0, 7.0, road_type="parking", v_ego=3.0) == vision.PARKING_LIMIT == 20
  assert drive(memory, 7.0, 11.0, road_type="garage", v_ego=3.0) == vision.PARKING_LIMIT  # not long enough yet
  assert drive(memory, 11.0, 13.0, road_type="garage", v_ego=2.0) == vision.GARAGE_LIMIT == 10  # down the ramp
  assert confirm(memory, 20.0, [5], road_type="garage", v_ego=2.0) == 5  # lower: taken
  assert drive(memory, 21.0, 26.0, road_type="garage", v_ego=2.0) == 5  # the same garage: the sign holds
  assert drive(memory, 26.0, 32.0, road_type="parking", v_ego=3.0) == vision.PARKING_LIMIT  # out in the open
  assert drive(memory, 32.0, 43.0, road_type="town", v_ego=8.0) == 50 and memory.source == "road"


def test_memory_a_network_without_garage_keeps_a_car_park_at_the_garage_limit():
  memory = vision.SpeedLimitMemory()
  assert drive(memory, 0.0, 6.0, road_type="parking", v_ego=2.0) == vision.GARAGE_LIMIT
  memory = vision.SpeedLimitMemory()
  memory.garage_known = True
  assert drive(memory, 0.0, 6.0, road_type="garage", v_ego=2.0) == vision.GARAGE_LIMIT


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


def test_road_type_input_is_the_resized_frame():
  rng = np.random.default_rng(0)
  height, width = 76, 134
  y, uv = nv12(rng.integers(16, 236, (height, width)), rng.integers(16, 241, (height // 2, width // 2)),
               rng.integers(16, 241, (height // 2, width // 2)))
  expected = vision.resize(vision.yuv_to_rgb(y, uv, 0, height, 0, width), vision.ROAD_TYPE_WIDTH, vision.ROAD_TYPE_HEIGHT)
  assert np.array_equal(vision.RoadTypeInput(width, height)(y, uv)[0], expected.astype(np.float32) / 255.0)



def test_memory_a_30_zone_seen_keeps_the_car_at_30():
  memory = vision.SpeedLimitMemory()
  assert memory.update(0.0, [], road_type="zone30") == 30  # the start of a drive inside a zone
  assert drive(memory, 0.2, 9.0, road_type="town") == 30  # out of it: not long enough yet
  assert drive(memory, 9.0, 11.0, road_type="town") == 50
  # A 50 read on the main street, the gate of the zone missed: once the zone is seen long enough, 30.
  memory = vision.SpeedLimitMemory()
  assert confirm(memory, 0.0, [50], road_type="town") == 50 and memory.source == "sign"
  assert drive(memory, 1.0, 10.0, road_type="zone30") == 50
  assert drive(memory, 10.0, 12.0, road_type="zone30") == 30 and memory.source == "road"
  # Its end sign: the town again, though the network still sees the zone for a moment.
  assert confirm(memory, 20.0, [0], road_type="zone30") == 50
  assert drive(memory, 21.0, 26.0, road_type="zone30") == 50
  assert drive(memory, 26.0, 40.0, road_type="town") == 50
  # A higher number holds: the network may be wrong about the zone.
  memory = vision.SpeedLimitMemory()
  assert confirm(memory, 0.0, [60], road_type="town") == 60
  assert drive(memory, 1.0, 30.0, road_type="zone30") == 60


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
  memory = vision.SpeedLimitMemory()
  drive(memory, 0.0, 17.0, road_type="rural")
  assert confirm(memory, 17.0, [80], road_type="rural") == 80
  assert drive(memory, 18.0, 30.0, road_type="motorway") == 120  # no lag: read before the motorway, it ends


def test_the_exit_of_a_car_park_follows_its_setting(monkeypatch):
  monkeypatch.setattr(vision, "PARKING_EXIT_SECONDS", vision.PARKING_EXIT_SECONDS)
  assert vision.apply_settings({"parking_exit_seconds": 8.0}) == {"parking_exit_seconds": 8.0}
  memory = vision.SpeedLimitMemory()
  assert drive(memory, 0.0, 6.0, road_type="parking", v_ego=2.0) == vision.GARAGE_LIMIT
  assert drive(memory, 6.0, 14.0, road_type="town", v_ego=6.0) == vision.GARAGE_LIMIT
  assert drive(memory, 14.0, 15.0, road_type="town", v_ego=6.0) == 50


def test_settings_change_the_memory(monkeypatch):
  monkeypatch.setattr(vision, "CONFIRM_FRAMES", 2)
  vision.apply_settings({"confirm_frames": 1})
  memory = vision.SpeedLimitMemory()
  assert memory.update(0.0, [80]) == 80
