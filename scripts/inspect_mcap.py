#!/usr/bin/env python3
"""Inspect one MCAP without ROS; optionally write a numeric audit JSON."""
import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from piper_collection.diagnostics.dataset import deep_mcap, mcap_summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mcap", type=Path)
    parser.add_argument("--output", type=Path)
    options = parser.parse_args()
    if options.output and options.output.resolve() == options.mcap.resolve():
        parser.error("Output cannot replace input")
    result = {"summary": mcap_summary(options.mcap), "deep": deep_mcap(options.mcap)}
    encoded = json.dumps(result, indent=2, allow_nan=False)
    if options.output:
        with options.output.open("x", encoding="utf-8") as stream:
            stream.write(encoded + "\n")
    else:
        print(encoded)
    decoded_ok = not any(t["decode_errors"] for t in result["deep"]["topics"].values())
    return 0 if result["summary"]["status"] == "summary_ok" and result["deep"]["status"] == "ok" and decoded_ok else 1


if __name__ == "__main__":
    sys.exit(main())
