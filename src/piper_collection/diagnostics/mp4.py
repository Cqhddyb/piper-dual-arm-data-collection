"""Read classic MP4 sample tables without decoding or reading video payloads.

This is a metadata check, not an image-corruption test. Fragmented MP4 and
encrypted/multiple-video-track files are outside this project's audit scope.
"""
import struct
from pathlib import Path


def atoms(stream, start, end):
    pos = start
    while pos < end:
        if end - pos < 8:
            raise ValueError("Truncated MP4 atom header")
        stream.seek(pos)
        size, kind = struct.unpack(">I4s", stream.read(8))
        header = 8
        if size == 1:
            raw = stream.read(8)
            if len(raw) != 8:
                raise ValueError("Truncated extended atom")
            size = struct.unpack(">Q", raw)[0]
            header = 16
        elif size == 0:
            size = end - pos
        if size < header or pos + size > end:
            raise ValueError("MP4 atom extends beyond file/parent")
        yield kind, pos + header, pos + size
        pos += size


def read_mp4_metadata(path):
    path = Path(path)
    tracks = []
    with path.open("rb") as stream:
        for kind, start, end in atoms(stream, 0, path.stat().st_size):
            if kind == b"moof":
                raise ValueError("Fragmented MP4 is unsupported")
            if kind != b"moov":
                continue
            for tk, ts, te in atoms(stream, start, end):
                if tk != b"trak":
                    continue
                track = {}

                def walk(begin, finish):
                    for k, s, e in atoms(stream, begin, finish):
                        if k in (b"mdia", b"minf", b"stbl"):
                            walk(s, e)
                        elif k in (b"mdhd", b"hdlr", b"stsd", b"stts", b"stsz"):
                            if e - s > 64_000_000:
                                raise ValueError("Implausibly large MP4 metadata atom")
                            stream.seek(s)
                            data = stream.read(e - s)
                            if k == b"hdlr":
                                track["handler"] = data[8:12].decode("ascii", "replace")
                            elif k == b"mdhd":
                                if data[0] == 1:
                                    scale = struct.unpack_from(">I", data, 20)[0]
                                    duration = struct.unpack_from(">Q", data, 24)[0]
                                else:
                                    scale, duration = struct.unpack_from(">II", data, 12)
                                track.update(timescale=scale, mdhd_ticks=duration)
                            elif k == b"stsd" and len(data) >= 44:
                                track["codec"] = data[12:16].decode("ascii", "replace")
                                track["width"], track["height"] = struct.unpack_from(">HH", data, 40)
                            elif k == b"stts":
                                n = struct.unpack_from(">I", data, 4)[0]
                                if len(data) < 8 + n * 8:
                                    raise ValueError("Truncated MP4 stts table")
                                entries = [struct.unpack_from(">II", data, 8 + i * 8) for i in range(n)]
                                track.update(frames=sum(c for c, d in entries),
                                             sample_ticks=sum(c * d for c, d in entries),
                                             zero_duration_samples=sum(c for c, d in entries if d == 0),
                                             min_delta_ticks=min((d for c, d in entries), default=0),
                                             max_delta_ticks=max((d for c, d in entries), default=0))
                            elif k == b"stsz":
                                track["sample_count"] = struct.unpack_from(">I", data, 8)[0]

                walk(ts, te)
                if track.get("handler") == "vide":
                    tracks.append(track)
    if len(tracks) != 1:
        raise ValueError("Expected exactly one video track with moov metadata")
    t = tracks[0]
    if not t.get("timescale") or not t.get("frames") or not t.get("sample_ticks"):
        raise ValueError("Empty or missing video sample table")
    if t.get("sample_count") != t["frames"]:
        raise ValueError("MP4 stsz/stts sample count mismatch")
    t["duration_s"] = t["sample_ticks"] / t["timescale"]
    t["container_fps"] = t["frames"] / t["duration_s"]
    t["max_dts_interval_ms"] = t["max_delta_ticks"] * 1000 / t["timescale"]
    t["mean_dts_interval_ms"] = t["duration_s"] * 1000 / t["frames"]
    return t
