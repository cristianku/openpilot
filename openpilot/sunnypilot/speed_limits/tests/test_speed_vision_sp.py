"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

speedVisionSP (cereal/custom.capnp) as speedvisiond writes it: the fields of the old logs keep their numbers, the
fields for the logs take what the memory gives, and their comment lists the vocabulary of vision.py.
"""
from pathlib import Path

import pytest

from openpilot.sunnypilot.speed_limits import vision

capnp = pytest.importorskip("capnp")
CEREAL = Path(vision.__file__).parents[2] / "cereal"
try:
  # The schema cereal has loaded: loading custom.capnp again in its process aborts it (a duplicate id).
  from openpilot.cereal import custom as CEREAL_CUSTOM
except ImportError:  # no opendbc (cereal's import path): the file alone
  CEREAL_CUSTOM = None
FIELDS = {"speedLimit": 0, "modelName": 1, "signRead": 2, "roadType": 3, "speedLimitState": 4, "speedLimitReason": 5,
          "rawSignClass": 6, "rawSignSpeed": 7, "rawSignConfidence": 8, "rawSignTaken": 9, "signDistance": 10,
          "signSize": 11, "rawSignValue": 12, "rawSignUnit": 13, "rawSignCountry": 14,
          "speedLimitAhead": 15, "speedLimitAheadDistance": 16, "rawSignSeen": 17, "rawSignRoad": 18}
# speedVisionSP before the fields for the logs: an old log's message.
OLD = """@0xe0b6b2b0c0a1d2f3;
struct SpeedVisionSP {
  speedLimit @0 :Float32;
  modelName @1 :Text;
  signRead @2 :Bool;
  roadType @3 :Text;
}
"""


@pytest.fixture(scope="module")
def custom():
  return CEREAL_CUSTOM or capnp.load(str(CEREAL / "custom.capnp"), imports=[str(CEREAL)])


def test_the_fields_keep_their_numbers(custom):
  fields = custom.SpeedVisionSP.schema.fields
  assert {name: field.proto.ordinal.explicit for name, field in fields.items()} == FIELDS


def test_an_old_message_reads_with_the_new_fields_empty(custom, tmp_path):
  (tmp_path / "old.capnp").write_text(OLD)
  old = capnp.load(str(tmp_path / "old.capnp")).SpeedVisionSP.new_message(speedLimit=50 / 3.6, modelName="m",
                                                                         signRead=True, roadType="town")
  with custom.SpeedVisionSP.from_bytes(old.to_bytes()) as message:
    assert message.modelName == "m" and message.signRead and message.roadType == "town"
    assert message.speedLimit == pytest.approx(50 / 3.6)
    assert (message.speedLimitState, message.speedLimitReason, message.rawSignClass) == ("", "", "")
    assert (message.rawSignSpeed, message.rawSignConfidence, message.rawSignTaken) == (0, 0, False)


def read(name, confidence):
  """A sign of read_signs: its class, how sure, the speed it gives when sure enough."""
  speed = vision.class_speed(name) if confidence >= vision.MIN_CONFIDENCE else None
  return {"class": name, "confidence": confidence, "speed": speed}


def logged(custom, memory):
  """The fields for the logs as speedvisiond writes them, read back from the message's bytes."""
  builder = custom.SpeedVisionSP.new_message()
  memory.fill_log(builder)
  with custom.SpeedVisionSP.from_bytes(builder.to_bytes()) as read_back:
    return (read_back.speedLimitState, read_back.speedLimitReason, read_back.rawSignClass, read_back.rawSignSpeed,
            round(read_back.rawSignConfidence, 2), read_back.rawSignTaken)


def test_the_memory_fills_the_fields(custom):
  memory = vision.SpeedLimitMemory("CH")
  assert logged(custom, memory) == ("unknown", "", "", 0, 0, False)
  # An unsure read shows, though the memory drops it.
  assert memory.update_frame(0.0, [read("maximum_speed:30", 0.6), read("not_a_sign", 0.99)]) == 0
  assert memory.update_frame(0.2, [read("maximum_speed:30", 0.6)]) == 0
  assert logged(custom, memory) == ("unknown", "", "maximum_speed:30", 30, 0.6, False)
  assert memory.update_frame(1.0, [read("maximum_speed:80", 0.93)]) == 0
  assert memory.update_frame(1.2, [read("maximum_speed:80", 0.95), read("maximum_speed:30", 0.6)]) == pytest.approx(80 / 3.6)
  assert logged(custom, memory) == ("limit", "sign", "maximum_speed:80", 80, 0.95, True)
  assert memory.update_frame(1.4, []) == pytest.approx(80 / 3.6)  # no sign: the last read and whether it was taken are held
  assert logged(custom, memory) == ("limit", "sign", "maximum_speed:80", 80, 0.95, True)


def test_a_sign_of_another_road_shows_and_is_not_taken(custom):
  # speedvisiond gives the memory the signs taken (applicability.taken) and all of them for the logs (seen).
  memory = vision.SpeedLimitMemory("CH")
  side, ours = dict(read("maximum_speed:30", 0.99), road="other_road"), dict(read("maximum_speed:80", 0.9), road="own_road")
  for now in (0.0, 0.2):
    limit = memory.update_frame(now, [ours], seen=[side, ours])
  assert limit == pytest.approx(80 / 3.6)
  assert logged(custom, memory) == ("limit", "sign", "maximum_speed:30", 30, 0.99, False)  # the surest read, not taken
  builder = custom.SpeedVisionSP.new_message()
  memory.fill_log(builder)
  assert builder.rawSignRoad == "other_road"
  # The same number on our road and on the side road: the one of the side road is still not taken.
  memory = vision.SpeedLimitMemory("CH")
  side80 = dict(read("maximum_speed:80", 0.99), road="other_road")
  for now in (0.0, 0.2):
    limit = memory.update_frame(now, [ours], seen=[side80, ours])
  assert limit == pytest.approx(80 / 3.6) and not memory.raw_taken and memory.raw_road == "other_road"
  # Unresolved: taken, as a sign not ruled.
  memory = vision.SpeedLimitMemory("CH")
  unsure = dict(read("maximum_speed:60", 0.95), road="unresolved")
  for now in (0.0, 0.2):
    limit = memory.update_frame(now, [unsure], seen=[unsure])
  assert limit == pytest.approx(60 / 3.6) and memory.raw_taken and memory.raw_road == "unresolved"


def test_a_number_with_hours_is_its_number_to_the_memory(custom):
  memory = vision.SpeedLimitMemory("DE")
  assert memory.update_frame(0.0, [read("maximum_speed:30", 0.9)]) == 0
  assert memory.update_frame(0.2, [read("maximum_speed_hours:30", 0.95)]) == pytest.approx(30 / 3.6)  # the plate read when near
  assert logged(custom, memory) == ("limit", "sign", "maximum_speed_hours:30", 30, 0.95, True)


def test_the_comment_lists_the_vocabulary():
  text = (CEREAL / "custom.capnp").read_text()
  struct = text[text.index("struct SpeedVisionSP"):]
  struct = struct[:struct.index("\n}")]
  for value in vision.LIMIT_STATES + vision.LIMIT_REASONS:
    assert f'"{value}"' in struct, value
