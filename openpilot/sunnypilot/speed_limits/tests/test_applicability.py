"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The applicability network's inputs as its training made them (PIL's crops, torchvision's RoIAlign) and which signs it
rules and takes. The network itself is checked against PyTorch when it is exported (sunnypilot.speedlimits,
exporting.export_decider_car_onnx).
"""
import numpy as np
import pytest

from openpilot.sunnypilot.speed_limits import applicability, vision

WIDTH, HEIGHT = 1344, 760
CLASSES = ["own_road", "other_road", "unresolved"]


def contract(boxes=5):
  return {
    "task": "decider", "target": "car", "boxes": boxes,
    "ownership_families": ["cancellation", "maximum_speed", "variable_display", "zone", "road_marking"],
    "plate_families": {"maximum_speed_hours": "maximum_speed", "maximum_speed_wet": "maximum_speed"},
    "inputs": {
      "crops": {"shape": [boxes, 3, 96, 96], "crop_margin": 0.25,
                "normalization": {"mean": [0.485, 0.456, 0.406], "std": [0.229, 0.224, 0.225]}},
      "geometry": {"shape": [boxes, 10], "speed_scale_mps": 40.0},
      "scene": {"shape": [1, 3, 216, 384]},
      "roi_weights": {"shape": [boxes, 4, 84], "map": [7, 12], "stride": 32, "size": 2, "sampling_ratio": 2},
    },
    "outputs": {"ownership": {"shape": [boxes, 3], "classes": CLASSES}},
  }


class Network:
  """Says other_road for the rows whose crop is not zeros, with what it was given."""
  def __init__(self):
    self.given = []

  def __call__(self, **inputs):
    self.given.append({name: value.copy() for name, value in inputs.items()})
    out = np.zeros((inputs["crops"].shape[0], 3), dtype=np.float32)
    out[:, 1] = 0.8
    out[:, 0] = 0.2
    return {"ownership": out}


def frame(seed=0):
  """A frame's NV12 planes, of random pixels."""
  rng = np.random.default_rng(seed)
  return rng.integers(16, 236, (HEIGHT, WIDTH), dtype=np.uint8), rng.integers(16, 240, (HEIGHT // 2, WIDTH), dtype=np.uint8)


def sign(name, box=(600, 200, 640, 240), confidence=0.95):
  return {"class": name, "box": list(box), "confidence": confidence, "speed": vision.class_speed(name)}


def test_roi_weights_are_torchvisions_roi_align_on_a_linear_map():
  # On a map whose value is 4 * row + col the bilinear samples are exact: each bin is the mean of its samples.
  cells = np.arange(16, dtype=np.float32)
  weights = applicability.roi_weights([0.5, 0.5, 2.5, 2.5], 4, 4, stride=1)
  assert weights @ cells == pytest.approx([4 * 0.5 + 0.5, 4 * 0.5 + 1.5, 4 * 1.5 + 0.5, 4 * 1.5 + 1.5])
  assert weights.sum(axis=1) == pytest.approx(1.0)
  # A box in pixels of the scene: the map's cells are stride pixels.
  assert applicability.roi_weights([16, 16, 80, 80], 4, 4, stride=32) @ cells == pytest.approx(weights @ cells)


def test_roi_weights_beyond_the_map():
  # Samples more than one cell outside count zero; less than one, taken at the edge.
  assert not applicability.roi_weights([-200, -200, -100, -100], 7, 12, stride=32).any()
  edge = applicability.roi_weights([-10, -10, 10, 10], 7, 12, stride=32)
  assert edge.sum(axis=1) == pytest.approx(1.0) and edge[:, 0] == pytest.approx(1.0)
  corner = applicability.roi_weights([380, 210, 384, 216], 7, 12, stride=32)
  assert corner[:, 7 * 12 - 1] == pytest.approx(1.0)


def test_the_crop_and_the_scene_are_pils():
  Image = pytest.importorskip("PIL.Image")
  y, uv = frame()
  rgb = Image.fromarray(np.ascontiguousarray(vision.yuv_to_rgb(y, uv, 0, HEIGHT, 0, WIDTH).transpose(1, 2, 0)))
  ruler = applicability.Applicability(Network(), contract(), vision, WIDTH, HEIGHT)
  for box in ([600.3, 200.7, 640.1, 239.9], [0, 0, 7.5, 9.25], [1300.2, 700.6, 1344, 760], [655.71, 313.4, 677.33, 337.9]):
    region = applicability.crop_region(applicability.clip_box(box, WIDTH, HEIGHT), WIDTH, HEIGHT, 0.25)
    theirs = np.asarray(rgb.resize((96, 96), Image.BILINEAR, box=tuple(region))).transpose(2, 0, 1)
    assert np.array_equal(ruler.crop(y, uv, region), theirs), box
  assert np.array_equal(ruler.scene_planes(y, uv), np.asarray(rgb.resize((384, 216), Image.BILINEAR)).transpose(2, 0, 1))


def test_it_rules_the_families_it_learnt():
  network = Network()
  ruler = applicability.Applicability(network, contract(), vision, WIDTH, HEIGHT)
  y, uv = frame()
  signs = [sign("other_sign:motorway_start"), sign("maximum_speed:50", (100, 100, 140, 140)),
           sign("maximum_speed_hours:30", (200, 100, 240, 140)), sign("zone:30", (300, 100, 340, 140))]
  ruled = ruler.rule(y, uv, signs, 20.0)
  assert ruled == signs[1:]
  assert "road" not in signs[0]
  assert [s["road"] for s in signs[1:]] == ["other_road"] * 3 and signs[1]["road_confidence"] == pytest.approx(0.8)
  given = network.given[-1]
  assert given["crops"][3:].max() == 0 and given["roi_weights"][3:].max() == 0  # the rows left over are zeros
  assert given["geometry"][0][:4] == pytest.approx([100 / WIDTH, 100 / HEIGHT, 140 / WIDTH, 140 / HEIGHT])
  assert given["geometry"][0][8:] == pytest.approx([0.5, 0.0])
  assert applicability.taken(signs) == [signs[0]]


def test_no_speed_and_no_sign_to_rule():
  network = Network()
  ruler = applicability.Applicability(network, contract(), vision, WIDTH, HEIGHT)
  y, uv = frame()
  assert ruler.rule(y, uv, [sign("other_sign:town_start")], 10.0) == [] and not network.given  # nothing ran
  ruler.rule(y, uv, [sign("cancellation")], None)
  assert network.given[-1]["geometry"][0][8:] == pytest.approx([0.0, 1.0])


def test_no_more_boxes_than_the_network_takes():
  ruler = applicability.Applicability(Network(), contract(boxes=2), vision, WIDTH, HEIGHT)
  y, uv = frame()
  signs = [sign("maximum_speed:80", (100 * i, 100, 100 * i + 40, 140)) for i in range(1, 5)]
  assert ruler.rule(y, uv, signs, 30.0) == signs[:2]
  assert [s.get("road") for s in signs] == ["other_road", "other_road", None, None]  # the others are taken


def test_a_contract_it_cannot_feed_is_refused():
  for change in ({"target": "server"}, {"task": "reader"}):
    with pytest.raises(ValueError):
      applicability.Applicability(Network(), {**contract(), **change}, vision, WIDTH, HEIGHT)
  wrong = contract()
  wrong["inputs"]["rois"] = {"shape": [5, 5]}
  with pytest.raises(ValueError):
    applicability.Applicability(Network(), wrong, vision, WIDTH, HEIGHT)
