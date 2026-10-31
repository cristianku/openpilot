"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The speed-limits networks of the models repository (models/speed_limits.json), compiled for the device's GPU with
tinygrad and run. They are compiled on the device by the build at boot (SConscript), the warmup offroad and
speedvisiond when they are still missing; never failing the build: a network that fails to compile leaves speedvisiond
without a reading, never the build or openpilot without its driving model.
"""
import hashlib
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

from openpilot.common.basedir import BASEDIR
from openpilot.common.hardware import AGNOS
from openpilot.common.hardware.hw import Paths

MODELS_DIR = Path(__file__).parent / "models"
MANIFEST = MODELS_DIR / "speed_limits.json"
MANIFEST_FORMAT = 2
TINYGRAD = Path(BASEDIR) / "tinygrad_repo"
COMPILER = TINYGRAD / "examples" / "openpilot" / "compile_onnx.py"
# The flags SConscript compiles the driving model with.
if AGNOS:
  TINYGRAD_FLAGS = {"DEV": "QCOM", "IMAGE": "1", "FLOAT16": "1", "NOLOCALS": "1", "JIT_BATCH_SIZE": "0", "OPENPILOT_HACKS": "1"}
elif sys.platform == "darwin":
  TINYGRAD_FLAGS = {"DEV": "METAL", "JIT": "2"}
else:
  TINYGRAD_FLAGS = {"DEV": "CPU:LLVM"}
STALE_PARTIAL_SECONDS = 3600


class Networks:
  """The files of the networks: SignNet (finder), the classifier and its classes, the name of the release; the road
  type network and its classes when the release has one (road_type, optional in format 2); the settings of the vision
  limit (settings, optional: vision.apply_settings)."""
  def __init__(self, manifest: dict):
    self.name = f"speed-limits {manifest['version']}"
    self.settings: dict = manifest.get("settings") if isinstance(manifest.get("settings"), dict) else {}
    self.finder = MODELS_DIR / manifest["finder"]["file"]
    self.classifier = MODELS_DIR / manifest["classifier"]["file"]
    contract = json.loads((MODELS_DIR / manifest["classifier"]["contract"]).read_text())
    self.classes: list[str] = contract["output"]["classes"]
    self.road_type: Path | None = None
    self.road_types: list[str] = []
    if "road_type" in manifest:
      self.road_type = MODELS_DIR / manifest["road_type"]["file"]
      self.road_types = json.loads((MODELS_DIR / manifest["road_type"]["contract"]).read_text())["output"]["classes"]

  def onnx(self) -> list[Path]:
    """Every network of the release, the road type's too."""
    return [self.finder, self.classifier] + ([self.road_type] if self.road_type else [])


def networks() -> Networks | None:
  """The networks of the models repository, None when it has none (an older format, no model yet)."""
  try:
    manifest = json.loads(MANIFEST.read_text())
    return Networks(manifest) if manifest.get("format") == MANIFEST_FORMAT else None
  except (OSError, ValueError, KeyError, TypeError):
    return None


def compiled_path(onnx: Path) -> Path:
  """Where a network compiled for this device is kept: by the hash of its ONNX, in a folder of the models' root
  of its own (the models manager clears only the root's files)."""
  digest = hashlib.sha256(onnx.read_bytes()).hexdigest()[:16]
  return Path(Paths.model_root()) / "speed_limits" / f"{onnx.stem}_{digest}_tinygrad.pkl"


def compile_network(onnx: Path) -> Path:
  """The network compiled for this device, compiled now if it is not yet (tinygrad's compile_onnx.py, as SConscript
  compiles the driving model; a minute or more on the device). Raises CalledProcessError when it fails."""
  target = compiled_path(onnx)
  if target.is_file():
    return target
  target.parent.mkdir(parents=True, exist_ok=True)
  for partial in target.parent.glob("*.partial"):
    if time.time() - partial.stat().st_mtime > STALE_PARTIAL_SECONDS:  # noqa: TID251 - a file's mtime is wall clock
      partial.unlink(missing_ok=True)
  # Unique to this process: speedvisiond and the warmup may compile at the same time.
  partial = target.with_name(f"{target.name}.{os.getpid()}.partial")
  path = os.pathsep.join(p for p in (str(TINYGRAD), BASEDIR, os.environ.get("PYTHONPATH", "")) if p)
  env = {**os.environ, **TINYGRAD_FLAGS, "PYTHONPATH": path, "CACHELEVEL": "0"}
  try:
    subprocess.run([sys.executable, str(COMPILER), str(onnx), str(partial), "--out-of-band", "--benchmark-runs", "1"],
                   cwd=target.parent, env=env, check=True, capture_output=True, text=True)
    partial.replace(target)
  finally:
    partial.unlink(missing_ok=True)
  for old in target.parent.glob(f"{onnx.stem}_*_tinygrad.pkl"):
    if old != target:
      old.unlink(missing_ok=True)
  return target


class Network:
  """A compiled network with one input and one output: numpy in, numpy (float32) out."""
  def __init__(self, path: Path):
    from tinygrad.tensor import Tensor
    from openpilot.selfdrive.modeld.helpers import load_oob

    jits = load_oob(path)
    self.run = jits["run"]
    ((self.input_name, (shape, dtype, device)),) = jits["input_specs"].items()
    self.host = np.zeros(shape, dtype=dtype)
    self.host_buffer = Tensor(self.host, device="NPY")._buffer()
    self.input = Tensor(np.zeros(shape, dtype=dtype), device=device).realize()
    self.outputs = {name: Tensor(np.zeros(shape, dtype=dtype), device=device).realize()
                    for name, (shape, dtype, device) in jits["output_specs"].items()}
    self.output_name = next(iter(self.outputs))

  def __call__(self, x: np.ndarray) -> np.ndarray:
    self.host[...] = x
    self.input._buffer().copy_from(self.host_buffer)
    self.run(output_buffers=self.outputs, **{self.input_name: self.input})
    return self.outputs[self.output_name].numpy().astype(np.float32)


class StepNetwork:
  """A compiled network with several named inputs and outputs (the road type: the frame, the car's speed and motion,
  its memory): numpy in by name, numpy (float32) out by name."""
  def __init__(self, path: Path):
    from tinygrad.tensor import Tensor
    from openpilot.selfdrive.modeld.helpers import load_oob

    jits = load_oob(path)
    self.run = jits["run"]
    specs = jits["input_specs"].items()
    self.hosts = {name: np.zeros(shape, dtype=dtype) for name, (shape, dtype, _) in specs}
    self.input_names = tuple(self.hosts)
    self.host_buffers = {name: Tensor(host, device="NPY")._buffer() for name, host in self.hosts.items()}
    self.inputs = {name: Tensor(np.zeros(shape, dtype=dtype), device=device).realize() for name, (shape, dtype, device) in specs}
    self.outputs = {name: Tensor(np.zeros(shape, dtype=dtype), device=device).realize()
                    for name, (shape, dtype, device) in jits["output_specs"].items()}

  def input_shape(self, name: str) -> tuple[int, ...]:
    return self.hosts[name].shape

  def __call__(self, **inputs: np.ndarray) -> dict[str, np.ndarray]:
    for name, value in inputs.items():
      self.hosts[name][...] = value
      self.inputs[name]._buffer().copy_from(self.host_buffers[name])
    self.run(output_buffers=self.outputs, **self.inputs)
    return {name: output.numpy().astype(np.float32) for name, output in self.outputs.items()}
