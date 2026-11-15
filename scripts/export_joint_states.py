#!/usr/bin/env python3
"""Export original JointState arrays/timestamps to JSONL; no action relabeling."""
import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from piper_collection.diagnostics.cdr import decode_joint_state


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mcap", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if args.output.resolve() == args.mcap.resolve():
        parser.error("Output cannot replace input")
    from mcap.reader import make_reader
    with args.mcap.open("rb") as source, args.output.open("x", encoding="utf-8") as target:
        for schema, channel, message in make_reader(source, validate_crcs=True).iter_messages(log_time_order=False):
            if not channel.topic.endswith("/joint_states"):
                continue
            if channel.message_encoding != "cdr":
                raise ValueError("Expected ROS 2 CDR encoding")
            row = {"topic": channel.topic, "log_time_ns": message.log_time,
                   "publish_time_ns": message.publish_time, **decode_joint_state(message.data)}
            # Fail visibly on invalid numeric input instead of emitting non-standard JSON.
            target.write(json.dumps(row, allow_nan=False) + "\n")


if __name__ == "__main__":
    main()
