# Dependencies and environment evidence

Live collection targets Ubuntu 22.04 / ROS 2 Humble / Python 3.10. Offline diagnostics use Python 3.10+ and do not import ROS. The recorded historical environment and the currently inspectable environment differ; neither proves the version used for every historical episode.

| Component | Historical export | Observed during audit | License evidence / distribution |
|---|---|---|---|
| Python | 3.10.19 | 3.10.19 in recording environment | External runtime |
| mcap | 1.3.1 | 1.3.1 | Installed metadata: MIT classifier |
| mcap-ros2-support | 0.5.7 | 0.5.7 | Installed metadata: MIT classifier; optional cross-check/testing |
| NumPy | 1.24.0 | 2.2.6 | BSD family and bundled component notices |
| OpenCV | opencv-python 4.7.0.72 | cv2 reports 4.13.0; pip distribution metadata absent | Version/build-specific Apache/BSD and third-party terms |
| PyYAML | Not recorded | 6.0.3 | MIT metadata; required by collector |
| piper-sdk | Not recorded | 0.6.1 | MIT License in installed metadata; external dependency |
| python-can | Not recorded | 4.6.1 | External SDK transport dependency; verify package notices |
| piper ROS package | Not recorded | package.xml says 0.0.0 | External LICENSE Apache-2.0; exact deployment revision unavailable |
| usb_cam | Not recorded | 0.8.1 | package.xml: BSD |
| realsense2_camera | Not recorded | 4.56.4 | package.xml: Apache License 2.0 |
| image_transport_plugins | Not recorded | 2.5.4 | package.xml: BSD |
| ROS standard messages / launch / rclpy | Not recorded | Humble installation present | External ROS packages; retain their upstream notices |
| pytest | Not recorded | Available to audit runner | Development only, MIT |
| MoviePy / FFmpeg | Not essential for live recorder | Historical postprocessor imports MoviePy | Not bundled; FFmpeg licensing depends on build/components |

`requirements.txt` installs the offline core and YAML support. `requirements-dev.txt` adds pytest and the ROS-free MCAP decoder used for cross-checking. `requirements-hardware.txt` constrains the historical NumPy 1.24.0 / OpenCV 4.7.0.72 pair and the observed SDK 0.6.1. It does not pin the full ROS graph, system libraries, firmware or camera configuration. That hardware profile has not been installed or live-tested during this audit.

Do not combine an older NumPy-1-built OpenCV wheel with NumPy 2.x. A system-site-packages virtual environment can expose ROS packages, but all NumPy/OpenCV/ROS ABI versions must be checked together. The machine's default Python may be 3.13 and cannot load Humble's Python 3.10 extensions.

Typical offline setup:

```bash
python3.10 -m venv .venv-offline
source .venv-offline/bin/activate
python -m pip install -r requirements-dev.txt
python -m pytest -q -p no:cacheprovider tests
```

For live operation, install ROS, usb_cam, compressed image transport and RealSense ROS using their supported installation procedures. Obtain and build an authorized, compatible Piper ROS workspace and its message package separately. Source ROS and that workspace before running the recorder. This candidate package includes no third-party SDK/driver source or binaries.

Upstream acknowledgements: AgileX Robotics Piper SDK/ROS, ROS 2, Foxglove MCAP, Intel RealSense ROS, usb_cam, OpenCV, NumPy, PyYAML and pytest. Dependency license declarations do not assign a license to this project's own source; see [publication review](publication_review.md).
