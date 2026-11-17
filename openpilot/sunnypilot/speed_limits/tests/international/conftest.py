"""Use the actual checkout sources, with no patch application or shadow source tree."""

from pathlib import Path

import pytest
from openpilot.sunnypilot.speed_limits import vision as vision_module, country as country_module


@pytest.fixture(scope='session')
def candidate():
  return Path(vision_module.__file__).resolve().parents[3]


@pytest.fixture(scope='session')
def vision():
  return vision_module


@pytest.fixture(scope='session')
def country():
  return country_module
