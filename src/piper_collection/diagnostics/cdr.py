"""Decode only the ROS 2 CDR fields needed by this project's offline audit.

Supports classic CDR encapsulations 0/1, not arbitrary ROS messages or XCDR2.
Does not depend on ROS, and can inspect historical files with empty schemas.
"""
import struct


class CDRReader:
    def __init__(self, data):
        if len(data) < 4 or data[:2] not in (b"\x00\x00", b"\x00\x01"):
            raise ValueError("Unsupported CDR encapsulation")
        self.data = data
        self.pos = 4
        self.endian = "<" if data[1] else ">"

    def unpack(self, code, alignment):
        self.pos += (-(self.pos - 4)) % alignment
        size = struct.calcsize(code)
        value = struct.unpack_from(self.endian + code, self.data, self.pos)
        self.pos += size
        return value[0] if len(value) == 1 else value

    def count(self):
        n = self.unpack("I", 4)
        if n > 1_000_000:
            raise ValueError("Implausible CDR sequence length")
        return n

    def string(self):
        n = self.count()
        if n < 1 or self.pos + n > len(self.data) or self.data[self.pos + n - 1] != 0:
            raise ValueError("Truncated or unterminated CDR string")
        value = self.data[self.pos:self.pos + n - 1].decode("utf-8")
        self.pos += n
        return value

    def doubles(self):
        n = self.count()
        if not n:
            return []
        self.pos += (-(self.pos - 4)) % 8
        result = list(struct.unpack_from(self.endian + str(n) + "d", self.data, self.pos))
        self.pos += 8 * n
        return result


def decode_joint_state(data):
    reader = CDRReader(data)
    sec = reader.unpack("i", 4)
    nsec = reader.unpack("I", 4)
    if nsec >= 1_000_000_000:
        raise ValueError("Invalid ROS nanosec")
    frame = reader.string()
    names = [reader.string() for _ in range(reader.count())]
    return {"stamp_ns": sec * 1_000_000_000 + nsec, "frame_id": frame,
            "name": names, "position": reader.doubles(),
            "velocity": reader.doubles(), "effort": reader.doubles()}


def decode_camera_info_prefix(data):
    reader = CDRReader(data)
    sec = reader.unpack("i", 4)
    nsec = reader.unpack("I", 4)
    if nsec >= 1_000_000_000:
        raise ValueError("Invalid ROS nanosec")
    frame = reader.string()
    return {"stamp_ns": sec * 1_000_000_000 + nsec, "frame_id": frame,
            "height": reader.unpack("I", 4), "width": reader.unpack("I", 4)}
