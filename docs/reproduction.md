# Reproduction guide

See [English README](../README.md) or [中文 README](../README_zh.md) for the complete feature and key reference. The commands below are for a human operator; release preparation did not execute live driver startup, enabling, robot motion or a full collection run.

## Offline reproduction

```bash
python3.10 -m venv .venv-offline
source .venv-offline/bin/activate
python -m pip install -r requirements-dev.txt
python -m pytest -q -p no:cacheprovider tests
python scripts/inspect_mcap.py /path/to/0.mcap --output /tmp/episode-audit.json
python scripts/audit_dataset.py /path/to/dataset-root --output /tmp/piper-audit
python scripts/export_joint_states.py /path/to/0.mcap --output /tmp/joints.jsonl
```

Use a separate output directory outside source data. Inspect/export refuse to overwrite an existing output file. Reports expose numeric quality and completeness issues; successful decoding is not a full dataset acceptance decision. Export preserves separate target/feedback topics and original array lengths.

## Hardware and environment

Use Ubuntu 22.04, Python 3.10 and ROS 2 Humble with compatible NumPy/OpenCV. Install the separately authorized Piper ROS workspace/message package/SDK, usb_cam, RealSense ROS, compressed image transport, standard messages, rclpy and launch packages. Review [dependency evidence](dependencies.md), especially historical/current version drift. A virtual environment for live work must be able to import ROS, for example one created with `--system-site-packages` on a compatible system.

Confirm the two execution-arm identities, firmware, grippers, upstream teleoperation arrangement, emergency stop, power and workspace clearance. Bind two distinct CAN interfaces at the vendor-required 1 Mbit/s. Logical left/right names alone do not verify physical identity. Camera device enumeration, mounting and calibration must be checked locally; exact hardware models were not fully recorded.

## External robot startup

Source ROS and the separately built robot workspace. The original workflow requests automatic enabling and can move the gripper:

```bash
source /opt/ros/humble/setup.bash
source /path/to/piper_ros/install/setup.bash
ros2 launch piper start_two_piper.launch.py \
  can_left_port:=can_piper_left can_right_port:=can_piper_right \
  auto_enable:=true girpper_exist:=true
```

The launch spelling `girpper_exist` differs from the inspected node's `gripper_exist`; confirm your driver version and parameters. Connection/limit queries do not constitute zero calibration. Follow manufacturer procedures for zeroing or reset because this project does not contain those routines.

```bash
ros2 topic hz /joint_states_left
ros2 topic hz /joint_states_right
ros2 topic hz /joint_states_ctrl_left
ros2 topic hz /joint_states_ctrl_right
ros2 topic echo /joint_states_left --once
```

Verify names, units, header freshness and physical sides, rather than topic existence alone. The recorder cannot provide the missing upstream target-generation system.

## Camera startup and calibration

```bash
ros2 launch ./launch/dualcam.launch.py
ros2 topic hz /dabai_left/image_raw/compressed
ros2 topic hz /dabai_right/image_raw/compressed
ros2 topic hz /camera/camera/color/image_rect_raw/compressed
ros2 topic echo /camera/camera/color/camera_info --once
cp configs/collection.example.yaml configs/collection.local.yaml
```

Review the launch's video0/video2 identities and RealSense profile parameter. Confirm compressed topics and actual resolution. Replace all placeholder side-camera intrinsics with calibration for that exact image mode. Previously recorded 640×480 intrinsics cannot simply be relabeled as 1280×720.

## Recording

Set the local output directory, anonymous task description, save rates and frame limit. Ensure sufficient disk space and a working GUI, then:

```bash
python scripts/collect.py --config configs/collection.local.yaml
```

Focus a preview window. Space prepares, `s` saves, `q` discards, idle `r` permanently deletes the preceding numbered episode, and `z` exits/discards unfinished work. There is no pause/resume. Recording starts when four state caches exist and a leader position is nonzero, not necessarily when a new movement occurs.

Save the terminal's raw callback-rate diagnostics as a separate local log. Confirm one MCAP and three playable MP4 files. Inspect the exported arrays and both log/header timing; a constant-FPS MP4 may contain reused frames. Check all four state streams, not a pooled subset.

## Validation and recovery boundaries

Re-run offline tools on the new files. Inspect schema availability, dimensions, finite values, gaps, order, side completeness, video structure and expected resolutions. The original client validator is not redistributed; its rate checks partly use scheduled CameraInfo log times, not unique source images. A mandatory PASS is only one validation layer.

Historical postprocessing can rewrite fields, transcode videos and synthesize CameraInfo. Those tools are excluded and were not run in this audit. Preserve original input data and an explicit processing manifest if any future repair is performed. No defaults in this package recover missing exposures, device timestamps or unrecorded control inputs.
