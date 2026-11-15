"""Offline checks for truncation, time regressions, numeric quality, and MP4 tables."""
import json
import math
import struct
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from piper_collection.diagnostics.cdr import decode_joint_state, decode_camera_info_prefix
from piper_collection.diagnostics.dataset import deep_mcap, mcap_summary
from piper_collection.diagnostics.mp4 import read_mp4_metadata
from piper_collection.diagnostics.timing import nearest_offsets_ns, timestamp_stats


def joint_bytes(stamp, position=(0.2, 0.01)):
    data = bytearray(b"\x00\x01\x00\x00")

    def pack(code, value, align=4):
        data.extend(b"\x00" * ((-(len(data) - 4)) % align))
        data.extend(struct.pack("<" + code, value))

    def string(value):
        raw = value.encode() + b"\x00"
        pack("I", len(raw))
        data.extend(raw)

    pack("i", stamp // 1_000_000_000)
    pack("I", stamp % 1_000_000_000)
    string("")
    pack("I", 2)
    string("joint1")
    string("gripper")
    for values in (position, (), ()):
        pack("I", len(values))
        for value in values:
            pack("d", value, 8)
    return bytes(data)


def atom(kind, data):
    return struct.pack(">I4s", len(data) + 8, kind) + data


def video_bytes(frames=60, delta=512):
    mdhd = atom(b"mdhd", b"\0" * 12 + struct.pack(">II", 15360, frames * delta) + b"\0" * 4)
    hdlr = atom(b"hdlr", b"\0" * 8 + b"vide" + b"\0" * 12)
    sample = bytearray(86)
    struct.pack_into(">I4s", sample, 0, 86, b"avc1")
    struct.pack_into(">HH", sample, 32, 640, 480)
    stsd = atom(b"stsd", b"\0" * 4 + struct.pack(">I", 1) + sample)
    stts = atom(b"stts", b"\0" * 4 + struct.pack(">III", 1, frames, delta))
    stsz = atom(b"stsz", b"\0" * 4 + struct.pack(">II", 1, frames))
    stbl = atom(b"stbl", stsd + stts + stsz)
    trak = atom(b"trak", atom(b"mdia", mdhd + hdlr + atom(b"minf", stbl)))
    return atom(b"ftyp", b"isom\0\0\0\0") + atom(b"moov", trak) + atom(b"mdat", b"0")


def test_cdr_alignment_empty_arrays():
    row = decode_joint_state(joint_bytes(2_123_456_789))
    assert row["stamp_ns"] == 2_123_456_789
    assert row["name"] == ["joint1", "gripper"]
    assert row["position"] == [0.2, 0.01]
    assert row["velocity"] == row["effort"] == []


@pytest.mark.parametrize("data", [b"", b"\0\x03\0\0", joint_bytes(1)[:-3]])
def test_cdr_rejects_unsupported_or_truncated(data):
    with pytest.raises((ValueError, struct.error)):
        decode_joint_state(data)


def test_timestamp_order_is_not_hidden_by_sorting():
    stats = timestamp_stats([10, 20, 20, 15])
    assert stats["adjacent_duplicates"] == 1
    assert stats["duplicate_values"] == 1
    assert stats["regressions"] == 1


def test_nearest_offset_includes_uncovered_edges():
    assert list(nearest_offsets_ns([100, 200], [50, 130, 300])) == [50, 30, 100]
    assert len(nearest_offsets_ns([], [1])) == 0


def test_mp4_sample_table(tmp_path):
    p = tmp_path / "test.mp4"
    p.write_bytes(video_bytes())
    result = read_mp4_metadata(p)
    assert (result["width"], result["height"], result["frames"]) == (640, 480, 60)
    assert result["codec"] == "avc1"
    assert result["container_fps"] == 30
    assert result["duration_s"] == 2


@pytest.mark.parametrize("data", [b"", video_bytes()[:-3], atom(b"mdat", b"0")])
def test_mp4_rejects_missing_or_truncated_metadata(tmp_path, data):
    p = tmp_path / "broken.mp4"
    p.write_bytes(data)
    with pytest.raises(ValueError):
        read_mp4_metadata(p)


def test_mcap_empty_and_invalid_files(tmp_path):
    p = tmp_path / "empty.mcap"
    p.touch()
    assert mcap_summary(p)["status"] == "empty"
    p.write_bytes(b"not an mcap")
    assert mcap_summary(p)["status"] == "error"


def test_mcap_quality_detects_nan_and_duplicate_physical_stamps(tmp_path):
    from mcap.writer import Writer
    p = tmp_path / "quality.mcap"
    with p.open("wb") as stream:
        writer = Writer(stream)
        writer.start()
        schema = writer.register_schema("sensor_msgs/msg/JointState", "ros2msg", b"")
        channel = writer.register_channel("/robot/arm_left_follow/joint_states", "cdr", schema)
        for stamp, physical, position in [(100, 90, (0.2, 0.01)), (120, 90, (math.nan, 0.2)), (140, 80, (0.3, 0.02))]:
            writer.add_message(channel, stamp, joint_bytes(physical, position), publish_time=stamp)
        writer.finish()
    result = deep_mcap(p)
    row = result["topics"]["/robot/arm_left_follow/joint_states"]
    assert result["status"] == "ok"
    assert row["header"]["regressions"] == row["header"]["adjacent_duplicates"] == 1
    assert row["quality"]["position"]["nan"] == 1
    assert row["quality"]["gripper_outside_0_to_0_1m"] == 1
    json.dumps(result, allow_nan=False)
