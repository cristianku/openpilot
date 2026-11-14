#!/usr/bin/env python3
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Compiles offroad the speed-limits networks the build at boot did not (SConscript), so that speedvisiond reads from the
start of the drive. It runs when speedvisiond would; once the networks are compiled it only waits.
"""
import os
os.environ['GMMU'] = '0'
import subprocess
import time

from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.speed_limits import runner


def main() -> None:
  failed = set()
  while True:
    files = runner.networks()
    for onnx in files.onnx() if files else ():
      attempt = (onnx, onnx.stat().st_mtime_ns)
      if attempt in failed or runner.compiled_path(onnx).is_file():
        continue
      started = time.monotonic()
      try:
        runner.compile_network(onnx)
        cloudlog.warning(f"speed limits warmup: {onnx.name} compiled in {time.monotonic() - started:.0f} s")
      except subprocess.CalledProcessError as e:
        cloudlog.error(f"speed limits warmup: {onnx.name} did not compile: {(e.stderr or '')[-2000:]}")
        failed.add(attempt)
    time.sleep(60)


if __name__ == "__main__":
  main()
