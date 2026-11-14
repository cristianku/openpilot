"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

VISION AND MAP ARE TWO SEPARATE SOURCES: NOTHING HERE READS THE MAP (liveMapDataSP).

The applicability network (the decider of sunnypilot.speedlimits): whom each speed sign read applies to, our road
(own_road), another road (other_road: a side street, an exit, the lane beside) or unsure (unresolved, it abstains). A
sign of another road is not taken: speedvisiond leaves it out of the limit, the memory and the brain's input; the reading
square still shows it. It rules only the signs of the families it learnt (its contract's ownership_families: the
numbers, their end, the zones, the variable displays, the markings on the road); the others are taken as before.

Its inputs are made as in its training (nnslr_tools.context_experiment.Decider), bit for bit where PIL made them: each box
clipped to the frame, the box and a margin of its size cut from the frame and resized bilinear to a square (crops), its
corners, size and centre over the frame size with the car's speed (geometry), the whole frame resized bilinear (scene).
torchvision's RoIAlign, which the comma's tinygrad has not, is made a weighted sum of the cells of the scene's map: the
weights of each box (roi_weights) are computed here, the sum is in the network. The network takes a fixed number of
boxes (the contract's boxes); the rows left over are zeros, their outputs not read.

No import of openpilot: the server loads this file by path with the car's vision.py (nnslr_tools.road_type_signs), so that
the brain learns from the signs the car takes.
"""
import numpy as np

OWN_ROAD, OTHER_ROAD, UNRESOLVED = "own_road", "other_road", "unresolved"


def clip_box(box, width: float, height: float) -> list[float]:
  """The box inside the frame, at least one pixel wide and high (context_experiment.clip_box)."""
  x1, y1, x2, y2 = (float(v) for v in box)
  x1 = min(max(x1, 0.0), width - 1.0)
  y1 = min(max(y1, 0.0), height - 1.0)
  x2 = min(max(x2, x1 + 1.0), float(width))
  y2 = min(max(y2, y1 + 1.0), float(height))
  return [x1, y1, x2, y2]


def crop_region(box: list[float], width: float, height: float, margin: float) -> list[float]:
  """The part of the frame cut for a box's crop: the box plus margin of its size on each side, clipped
  (context_experiment.crop_box)."""
  x1, y1, x2, y2 = box
  dx, dy = (x2 - x1) * margin, (y2 - y1) * margin
  return clip_box([x1 - dx, y1 - dy, x2 + dx, y2 + dy], width, height)


def roi_weights(box: list[float], map_height: int, map_width: int, stride: float, size: int = 2,
                ratio: int = 2) -> np.ndarray:
  """torchvision's roi_align (aligned=True, output size x size, sampling_ratio ratio, spatial_scale 1 / stride) of one box
  (x1, y1, x2, y2 in the pixels of the scene) as weights over the cells of the map: (size * size, map_height * map_width),
  bin ph * size + pw, cell row * map_width + col. Each bin is the mean of ratio x ratio bilinear samples; a sample beyond
  one cell outside the map counts zero, one less than zero is taken at zero, one on the last row or column there."""
  x1, y1, x2, y2 = (float(v) / stride - 0.5 for v in box)
  bin_width, bin_height = (x2 - x1) / size, (y2 - y1) / size
  weights = np.zeros((size * size, map_height * map_width), dtype=np.float64)
  share = 1.0 / (ratio * ratio)
  for ph in range(size):
    for iy in range(ratio):
      y = y1 + ph * bin_height + (iy + 0.5) * bin_height / ratio
      for pw in range(size):
        for ix in range(ratio):
          x = x1 + pw * bin_width + (ix + 0.5) * bin_width / ratio
          if y < -1.0 or y > map_height or x < -1.0 or x > map_width:
            continue
          yy, xx = max(y, 0.0), max(x, 0.0)
          y_low, x_low = int(yy), int(xx)
          if y_low >= map_height - 1:
            y_low = y_high = map_height - 1
            yy = float(y_low)
          else:
            y_high = y_low + 1
          if x_low >= map_width - 1:
            x_low = x_high = map_width - 1
            xx = float(x_low)
          else:
            x_high = x_low + 1
          ly, lx = yy - y_low, xx - x_low
          hy, hx = 1.0 - ly, 1.0 - lx
          row = weights[ph * size + pw]
          row[y_low * map_width + x_low] += hy * hx * share
          row[y_low * map_width + x_high] += hy * lx * share
          row[y_high * map_width + x_low] += ly * hx * share
          row[y_high * map_width + x_high] += ly * lx * share
  return weights.astype(np.float32)


class Applicability:
  """The network on a frame's signs (vision.read_signs): rule() writes into each sign it rules its road (own_road,
  other_road, unresolved) and how sure (road_confidence); taken() keeps the signs that are not of another road. network
  takes its inputs by name and gives its outputs by name (runner.StepNetwork on the car, onnxruntime on the server).
  Raises ValueError on a contract this code cannot feed."""
  def __init__(self, network, contract: dict, vision, width: int, height: int):
    if contract.get("task") != "decider" or contract.get("target") != "car":
      raise ValueError("not the car's applicability network")
    inputs = contract["inputs"]
    if not set(inputs) <= {"crops", "geometry", "scene", "roi_weights"} or "crops" not in inputs:
      raise ValueError(f"the applicability network asks for inputs the car does not give: {sorted(inputs)}")
    self.network, self.vision = network, vision
    self.width, self.height = width, height
    self.boxes = int(contract["boxes"])
    self.families = frozenset(contract["ownership_families"])
    self.plate_families = dict(contract.get("plate_families", {}))
    self.classes = list(contract["outputs"]["ownership"]["classes"])
    crops = inputs["crops"]
    self.crop_size = int(crops["shape"][-1])
    self.margin = float(crops["crop_margin"])
    self.mean = np.array(crops["normalization"]["mean"], dtype=np.float32).reshape(3, 1, 1)
    self.std = np.array(crops["normalization"]["std"], dtype=np.float32).reshape(3, 1, 1)
    self.geometry = "geometry" in inputs
    self.speed_scale = float(inputs["geometry"]["speed_scale_mps"]) if self.geometry else 1.0
    self.scene = None
    if "scene" in inputs:
      _, _, scene_height, scene_width = inputs["scene"]["shape"]
      self.scene_size = (int(scene_width), int(scene_height))
      self.scene_columns = vision.ColumnResampler(width, scene_width) if width != scene_width else None
      self.scene_rows = vision.resample_coefficients(height, scene_height)
      roi = inputs["roi_weights"]
      self.map_height, self.map_width = (int(v) for v in roi["map"])
      self.stride, self.roi_size, self.ratio = float(roi["stride"]), int(roi["size"]), int(roi["sampling_ratio"])
      self.scene = True
    self.host = {name: np.zeros(spec["shape"], dtype=np.float32) for name, spec in inputs.items()}

  def family(self, name: str) -> str:
    """The family of a classifier class as the network learnt them: maximum_speed_hours:30 is maximum_speed."""
    kind = name.partition(":")[0]
    return self.plate_families.get(kind, kind)

  def crop(self, y: np.ndarray, uv: np.ndarray, region: list[float]) -> np.ndarray:
    """PIL's Image.resize((size, size), BILINEAR, box=region) of the frame as uint8 planes (3, size, size): only the
    pixels the taps read are made RGB."""
    vision = self.vision
    columns = vision.resample_coefficients(self.width, self.crop_size, region[0], region[2])
    rows = vision.resample_coefficients(self.height, self.crop_size, region[1], region[3])
    left, right = int(columns[0].min()), int(columns[0].max()) + 1
    top, bottom = int(rows[0].min()), int(rows[0].max()) + 1
    planes = vision.yuv_to_rgb(y, uv, top, bottom, left, right)
    planes = vision.transpose(vision.resample_rows(vision.transpose(planes), columns[0] - left, columns[1]))
    return vision.resample_rows(planes, rows[0] - top, rows[1])

  def scene_planes(self, y: np.ndarray, uv: np.ndarray) -> np.ndarray:
    """PIL's Image.resize(scene size, BILINEAR) of the whole frame as uint8 planes (3, height, width)."""
    planes = self.vision.yuv_to_rgb(y, uv, 0, self.height, 0, self.width)
    if self.scene_columns is not None:
      planes = self.scene_columns(planes)
    return self.vision.resample_rows(planes, *self.scene_rows)

  def inputs(self, y: np.ndarray, uv: np.ndarray, boxes: list, v_ego: float | None) -> dict[str, np.ndarray]:
    """The network's inputs for up to self.boxes boxes (x1, y1, x2, y2 in frame pixels) of one frame."""
    for host in self.host.values():
      host[...] = 0.0
    speed = [v_ego / self.speed_scale, 0.0] if v_ego is not None else [0.0, 1.0]
    for i, raw in enumerate(boxes[:self.boxes]):
      box = clip_box(raw, self.width, self.height)
      pixels = self.crop(y, uv, crop_region(box, self.width, self.height, self.margin)).astype(np.float32) / 255.0
      self.host["crops"][i] = (pixels - self.mean) / self.std
      x1, y1, x2, y2 = box
      if self.geometry:
        self.host["geometry"][i] = [x1 / self.width, y1 / self.height, x2 / self.width, y2 / self.height,
                                    (x2 - x1) / self.width, (y2 - y1) / self.height,
                                    (x1 + x2) / (2 * self.width), (y1 + y2) / (2 * self.height), *speed]
      if self.scene:
        sx, sy = self.scene_size[0] / self.width, self.scene_size[1] / self.height
        self.host["roi_weights"][i] = roi_weights([x1 * sx, y1 * sy, x2 * sx, y2 * sy], self.map_height, self.map_width,
                                                  self.stride, self.roi_size, self.ratio)
    if self.scene and boxes:
      pixels = self.scene_planes(y, uv).astype(np.float32) / 255.0
      self.host["scene"][0] = (pixels - self.mean) / self.std
    return self.host

  def rule(self, y: np.ndarray, uv: np.ndarray, signs: list[dict], v_ego: float | None) -> list[dict]:
    """Writes road and road_confidence into the signs of the families it learnt (the first self.boxes of them: the
    car reads no more a frame), all in one run; the others get none. Returns the signs ruled."""
    ruled = [sign for sign in signs if self.family(sign["class"]) in self.families][:self.boxes]
    if not ruled:
      return []
    outputs = self.network(**self.inputs(y, uv, [sign["box"] for sign in ruled], v_ego))["ownership"]
    for sign, probabilities in zip(ruled, outputs, strict=False):
      index = int(np.argmax(probabilities))
      sign["road"], sign["road_confidence"] = self.classes[index], float(probabilities[index])
    return ruled


def taken(signs: list[dict]) -> list[dict]:
  """The signs of a frame that count: all but those the network gave to another road (vision.OTHER_ROAD)."""
  return [sign for sign in signs if sign.get("road") != OTHER_ROAD]
