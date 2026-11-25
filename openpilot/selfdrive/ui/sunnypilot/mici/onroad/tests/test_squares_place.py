import pyray as rl
import pytest

from openpilot.selfdrive.ui.sunnypilot.mici.onroad.final_speed_limit import final_box
from openpilot.selfdrive.ui.sunnypilot.mici.onroad.sign_reading import PULSE_GROW
from openpilot.selfdrive.ui.sunnypilot.mici.onroad.vision_speed_limit import GAP, MARGIN, READING_SIZE, SIZE, \
  reading_box, road_type_place, vision_box

RECT = rl.Rectangle(0, 0, 500, 240)
# The GPU icon of the big model (HudRenderer._draw_model_source), the orange one, the widest.
GPU_ICON = rl.Rectangle(RECT.width - 10 - 75, RECT.height - 14 - (50 + 44) / 2, 75, 44)


def _overlap(a: rl.Rectangle, b: rl.Rectangle) -> bool:
  return a.x < b.x + b.width and b.x < a.x + a.width and a.y < b.y + b.height and b.y < a.y + a.height


@pytest.mark.parametrize('box', [vision_box(RECT), vision_box(RECT, True), final_box(RECT), reading_box(RECT),
                                 reading_box(RECT, True)])
def test_no_square_covers_the_gpu_icon(box):
  assert not _overlap(box, GPU_ICON)


def test_the_reading_square_is_as_big_as_the_vision_square():
  assert READING_SIZE == SIZE


@pytest.mark.parametrize('beside_final', [False, True])
def test_the_reading_square_at_its_biggest_pulse_stays_off_the_gpu_icon(beside_final):
  box = reading_box(RECT, beside_final)
  grow = READING_SIZE * PULSE_GROW / 2 + 2  # a side, with its border
  assert not _overlap(rl.Rectangle(box.x - grow, box.y - grow, box.width + 2 * grow, box.height + 2 * grow), GPU_ICON)


@pytest.mark.parametrize('beside_final', [False, True])
def test_the_road_type_goes_left_of_the_reading_square_inside_the_screen(beside_final):
  size = rl.Vector2(136, 24)  # "expressway", the widest, at ROAD_FONT_SIZE
  place = road_type_place(RECT, size, beside_final)
  text = rl.Rectangle(place.x, place.y, size.x, size.y)
  reading = reading_box(RECT, beside_final)
  assert text.x >= RECT.x and text.x + text.width + GAP <= reading.x
  assert text.y + text.height / 2 == reading.y + READING_SIZE / 2
  for box in (vision_box(RECT, beside_final), final_box(RECT), GPU_ICON):
    assert not _overlap(text, box)


@pytest.mark.parametrize('beside_final', [False, True])
def test_the_reading_goes_under_the_vision_square(beside_final):
  vision, reading = vision_box(RECT, beside_final), reading_box(RECT, beside_final)
  assert reading.y == vision.y + SIZE + GAP and reading.x - vision.x == vision.x + SIZE - reading.x - READING_SIZE
  assert not _overlap(reading, final_box(RECT))


def test_the_final_limit_takes_the_corner_and_the_vision_square_goes_beside_it():
  final, vision = final_box(RECT), vision_box(RECT, True)
  assert (final.x, final.y) == (RECT.width - MARGIN - SIZE, MARGIN) == (vision_box(RECT).x, vision_box(RECT).y)
  assert not _overlap(final, vision)
  assert vision.x + SIZE + GAP == final.x and vision.y == final.y
