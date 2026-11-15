#!/usr/bin/env python3
"""Run the unchanged collector snapshot with an explicit configuration path."""
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, type=Path)
    options = parser.parse_args()
    if not options.config.is_file():
        parser.error("Configuration file does not exist")
    # Deferred imports allow --help and offline diagnostics without ROS.
    import cv2
    import rclpy
    from piper_collection.collector import VLADataCollector
    rclpy.init()
    collector = None
    try:
        collector = VLADataCollector(str(options.config.resolve()))
        rclpy.spin(collector)
    except KeyboardInterrupt:
        pass
    finally:
        if collector is not None:
            if collector.state in ("WAITING_FOR_MOVEMENT", "COLLECTING"):
                collector.discard_current_episode()
            collector.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
