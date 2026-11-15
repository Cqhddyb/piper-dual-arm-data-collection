"""Read-only MCAP/video audit with explicit full-inventory and sample scopes.

Outputs contain file metadata and numeric aggregates, not images, credentials,
task instructions, or operator names. Source files are opened in binary read mode.
"""
import argparse
import csv
import json
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np

from .cdr import decode_joint_state, decode_camera_info_prefix
from .mp4 import read_mp4_metadata
from .timing import describe, nearest_offsets_ns, timestamp_stats

ARM_TOPICS = [f"/robot/arm_{side}_{role}/joint_states"
              for role in ("leader", "follow") for side in ("left", "right")]
CAM_TOPICS = [f"/robot/camera/cam_{side}/image_raw/compressed/camera_info"
              for side in ("high", "left", "right")]


def mcap_summary(path):
    from mcap.reader import make_reader
    row = {"bytes": path.stat().st_size}
    if row["bytes"] == 0:
        return {**row, "status": "empty"}
    try:
        with path.open("rb") as stream:
            if stream.read(8) != b"\x89MCAP0\r\n":
                raise ValueError("Missing initial MCAP magic")
            stream.seek(-8, 2)
            if stream.read(8) != b"\x89MCAP0\r\n":
                raise ValueError("Missing final MCAP magic")
            stream.seek(0)
            summary = make_reader(stream).get_summary()
        if summary is None or summary.statistics is None:
            return {**row, "status": "no_summary"}
        stats = summary.statistics
        counts = {c.topic: stats.channel_message_counts.get(c.id, 0)
                  for c in summary.channels.values()}
        return {**row, "status": "summary_ok", "messages": stats.message_count,
                "start_ns": stats.message_start_time, "end_ns": stats.message_end_time,
                "duration_s": (stats.message_end_time - stats.message_start_time) / 1e9,
                "topics": counts,
                "schema_bytes": {s.name: len(s.data) for s in summary.schemas.values()},
                "missing_arm_topics": [t for t in ARM_TOPICS if not counts.get(t)],
                "missing_camera_info_topics": [t for t in CAM_TOPICS if not counts.get(t)],
                "equal_arm_counts": len({counts.get(t, 0) for t in ARM_TOPICS}) == 1}
    except Exception as error:
        return {**row, "status": "error", "error": f"{type(error).__name__}: {error}"}


def deep_mcap(path, limits=None):
    """Decode selected messages in file order; validate MCAP CRCs where present."""
    from mcap.reader import make_reader
    streams = defaultdict(lambda: {"header": [], "log": [], "age": [], "values": [],
                                   "dims": Counter(), "names": Counter(), "res": Counter(),
                                   "decode_errors": Counter()})
    result = {"path": str(path), "status": "ok", "topics": {}, "sync": {}}
    decoded = 0
    try:
        with path.open("rb") as stream:
            for schema, channel, msg in make_reader(stream, validate_crcs=True).iter_messages(log_time_order=False):
                if not (channel.topic.endswith("/joint_states") or channel.topic.endswith("/camera_info")):
                    continue
                t = streams[channel.topic]
                t["log"].append(msg.log_time)
                try:
                    if channel.message_encoding != "cdr":
                        raise ValueError("Expected CDR message encoding")
                    is_joint = channel.topic.endswith("/joint_states")
                    d = decode_joint_state(msg.data) if is_joint else decode_camera_info_prefix(msg.data)
                    decoded += 1
                    t["header"].append(d["stamp_ns"])
                    t["age"].append(msg.log_time - d["stamp_ns"])
                    if is_joint:
                        t["values"].append(d)
                        t["dims"][tuple(len(d[k]) for k in ("name", "position", "velocity", "effort"))] += 1
                        t["names"][tuple(d["name"])] += 1
                    else:
                        t["res"][(d["width"], d["height"])] += 1
                except Exception as error:
                    t["decode_errors"][f"{type(error).__name__}: {error}"] += 1
    except Exception as error:
        result.update(status="read_error", error=f"{type(error).__name__}: {error}")
    result["decoded_messages"] = decoded
    for topic, t in streams.items():
        item = {"log": timestamp_stats(t["log"]), "header": timestamp_stats(t["header"]),
                "log_minus_header_ms": describe(np.asarray(t["age"], dtype=np.int64) / 1e6),
                "decode_errors": dict(t["decode_errors"])}
        if t["res"]:
            item["resolutions"] = {f"{w}x{h}": n for (w, h), n in t["res"].items()}
        if t["values"]:
            vals = t["values"]
            item["dimension_counts_name_position_velocity_effort"] = {str(k): v for k, v in t["dims"].items()}
            item["joint_names"] = {",".join(k): v for k, v in t["names"].items()}
            item["example"] = {k: [x if not isinstance(x, float) or np.isfinite(x) else None
                                   for x in vals[0][k]]
                               for k in ("name", "position", "velocity", "effort")}
            quality = {}
            for field in ("position", "velocity", "effort"):
                flat = np.array([x for d in vals for x in d[field]], dtype=np.float64)
                finite = flat[np.isfinite(flat)]
                quality[field] = {"values": len(flat), "nan": int(np.isnan(flat).sum()),
                                  "inf": int(np.isinf(flat).sum()),
                                  "min": float(finite.min()) if len(finite) else None,
                                  "max": float(finite.max()) if len(finite) else None,
                                  "nonzero": int(np.count_nonzero(finite))}
            quality["adjacent_identical_positions"] = sum(a["position"] == b["position"] for a, b in zip(vals, vals[1:]))
            quality["empty_position_messages"] = sum(not d["position"] for d in vals)
            quality["name_position_mismatches"] = sum(len(d["name"]) != len(d["position"]) for d in vals)
            quality["velocity_abs_over_5rad_s_heuristic"] = sum(
                abs(x) > 5 for d in vals for x in d["velocity"] if np.isfinite(x))
            grip = [d["position"][d["name"].index("gripper")] for d in vals
                    if "gripper" in d["name"] and len(d["position"]) == len(d["name"])]
            quality["gripper"] = describe(grip)
            quality["gripper_outside_0_to_0_1m"] = sum(not 0 <= x <= 0.1 for x in grip)
            # Model bounds are a diagnostic reference, not verified hardware safety limits.
            checked = violations = 0
            if limits:
                for d in vals:
                    for name, value in zip(d["name"], d["position"]):
                        if name in limits:
                            checked += 1
                            lo, hi = limits[name]
                            violations += not lo <= value <= hi
            quality["urdf_reference_bounds_checked_values"] = checked
            quality["urdf_reference_bound_violations"] = violations
            item["quality"] = quality
        result["topics"][topic] = item
    for role in ("follow", "leader"):
        a = streams.get(f"/robot/arm_left_{role}/joint_states", {}).get("header", [])
        b = streams.get(f"/robot/arm_right_{role}/joint_states", {}).get("header", [])
        if a and b:
            n = min(len(a), len(b))
            result["sync"][f"{role}_left_right_same_index_ms"] = describe(np.abs(np.asarray(a[:n], dtype=np.int64) - np.asarray(b[:n], dtype=np.int64)) / 1e6)
            av, bv = np.asarray(a[:n], dtype=np.int64), np.asarray(b[:n], dtype=np.int64)
            valid = (av > 0) & (bv > 0)
            result["sync"][f"{role}_left_right_same_index_positive_ms"] = {
                **describe(np.abs(av[valid] - bv[valid]) / 1e6), "invalid_pairs": int(np.sum(~valid))}
            result["sync"][f"{role}_left_right_nearest_ms"] = describe(nearest_offsets_ns(a, b) / 1e6)
    for i, a in enumerate(CAM_TOPICS):
        for b in CAM_TOPICS[i + 1:]:
            aa = streams.get(a, {}).get("header", [])
            bb = streams.get(b, {}).get("header", [])
            if aa and bb:
                result["sync"][a + " -> " + b] = describe(nearest_offsets_ns(aa, bb) / 1e6)
        for b in ARM_TOPICS:
            aa = streams.get(a, {}).get("header", [])
            bb = streams.get(b, {}).get("header", [])
            if aa and bb:
                offsets = nearest_offsets_ns(bb, aa)
                result["sync"][a + " -> " + b] = {**describe(offsets / 1e6),
                                                                    "over_34ms": int(np.sum(offsets >= 34_000_000))}
    return result


def write_csv(path, rows):
    keys = sorted({k for row in rows for k in row})
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, keys)
        writer.writeheader()
        writer.writerows({k: json.dumps(v, ensure_ascii=False) if isinstance(v, (list, dict)) else v
                         for k, v in row.items()} for row in rows)


def audit(root, output, deep_per_group=3, limits=None):
    root, output = Path(root).resolve(), Path(output).resolve()
    # Prevent output from overwriting inputs or creating audit files inside data directories.
    inputs = [p for base in (root / "data", root / "data_backup") if base.exists() for p in base.rglob("*") if p.is_file()]
    if not inputs:
        inputs = [p for p in root.rglob("*") if p.is_file() and output not in p.parents]
    data_roots = [p for p in (root / "data", root / "data_backup") if p.exists()]
    if any(output == p or p in output.parents for p in data_roots):
        raise ValueError("Place audit output outside source data directories")
    output.mkdir(parents=True, exist_ok=True)
    mcaps, videos = [], []
    groups = defaultdict(list)
    for p in sorted(inputs):
        if p.suffix.lower() != ".mcap":
            continue
        rel = p.relative_to(root).as_posix()
        row = {"path": rel, "group": p.parent.relative_to(root).as_posix(),
               "scope": "non_piper_reference" if "realman" in p.parts else "piper_candidate",
               **mcap_summary(p)}
        possible = (p.parent / "videos" / p.stem, p.parent / p.stem)
        existing = [d for d in possible if d.is_dir()]
        row["video_layout"] = "videos/id" if existing and existing[0] == possible[0] else "id" if existing else "missing"
        row["video_count"] = sum((existing[0] / f"cam_{n}.mp4").is_file() for n in ("high", "left", "right")) if existing else 0
        mcaps.append(row)
        if row["status"] == "summary_ok" and row["scope"] == "piper_candidate":
            groups[row["group"]].append(p)
    write_csv(output / "mcap_inventory.csv", mcaps)
    (output / "mcap_inventory.json").write_text(json.dumps(mcaps, ensure_ascii=False, indent=2))
    print(f"MCAP inventory: {len(mcaps)} files; {dict(Counter(r['status'] for r in mcaps))}", flush=True)
    for p in sorted(inputs):
        if p.suffix.lower() != ".mp4":
            continue
        row = {"path": p.relative_to(root).as_posix(), "bytes": p.stat().st_size, "camera": p.stem}
        try:
            row.update(status="metadata_ok", **read_mp4_metadata(p))
        except Exception as error:
            row.update(status="error", error=f"{type(error).__name__}: {error}")
        videos.append(row)
    write_csv(output / "video_inventory.csv", videos)
    print(f"Video inventory: {len(videos)} files; {dict(Counter(r['status'] for r in videos))}", flush=True)
    selected = []
    for group, paths in sorted(groups.items()):
        paths = sorted(paths, key=lambda p: int(p.stem) if p.stem.isdigit() else -1)
        for index in sorted(set(int(x) for x in np.linspace(0, len(paths) - 1, min(deep_per_group, len(paths))))):
            selected.append(paths[index])
    samples = []
    for index, p in enumerate(selected):
        row = deep_mcap(p, limits=limits)
        row["path"] = p.relative_to(root).as_posix()
        samples.append(row)
        print(f"Deep sample {index + 1}/{len(selected)}: {row['path']} ({row['decoded_messages']} decoded)", flush=True)
    (output / "deep_samples.json").write_text(json.dumps(samples, ensure_ascii=False, indent=2, allow_nan=False))
    grouped = {}
    for group in sorted({r["group"] for r in mcaps}):
        rows = [r for r in mcaps if r["group"] == group]
        good = [r for r in rows if r["status"] == "summary_ok"]
        grouped[group] = {"files": len(rows), "bytes": sum(r["bytes"] for r in rows),
                          "status": dict(Counter(r["status"] for r in rows)),
                          "duration_s": describe([r["duration_s"] for r in good]),
                          "total_duration_s": sum(r["duration_s"] for r in good),
                          "missing_video_triplets": sum(r["video_count"] != 3 for r in rows),
                          "missing_arm_topics": sum(bool(r.get("missing_arm_topics")) for r in good),
                          "missing_camera_info_topics": sum(bool(r.get("missing_camera_info_topics")) for r in good)}
    summary = {"scope": "MCAP summaries and MP4 tables: all files; deep CDR: deterministic first/middle/last per eligible group",
               "mcap_count": len(mcaps), "mcap_bytes": sum(r["bytes"] for r in mcaps),
               "mcap_status": dict(Counter(r["status"] for r in mcaps)),
               "video_count": len(videos), "video_bytes": sum(r["bytes"] for r in videos),
               "video_status": dict(Counter(r["status"] for r in videos)),
               "video_frames": sum(r.get("frames", 0) for r in videos),
               "video_codecs": dict(Counter(r.get("codec", "error") for r in videos)),
               "video_resolutions": dict(Counter(f"{r.get('width')}x{r.get('height')}" for r in videos)),
               "deep_sample_count": len(samples), "deep_decoded_messages": sum(r["decoded_messages"] for r in samples),
               "groups": grouped}
    (output / "dataset_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2))
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--deep-per-group", type=int, default=3)
    parser.add_argument("--urdf-reference", type=Path)
    args = parser.parse_args()
    if args.deep_per_group < 0:
        parser.error("--deep-per-group must be non-negative")
    limits = {}
    if args.urdf_reference:
        import xml.etree.ElementTree as ET
        for joint in ET.parse(args.urdf_reference).getroot().findall("joint"):
            lim = joint.find("limit")
            if joint.attrib.get("name", "") in [f"joint{i}" for i in range(1, 7)] and lim is not None:
                limits[joint.attrib["name"]] = float(lim.attrib["lower"]), float(lim.attrib["upper"])
    print(json.dumps(audit(args.root, args.output, args.deep_per_group, limits), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
