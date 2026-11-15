# Timestamp and synchronization limitations

Joint headers are derived from SDK CAN receive timestamps. Multiple CAN frames contribute to a joint/gripper state; there is no proven atomic sampling instant. Four latest joint messages receive the same recorder log_time during a state save tick, while their headers remain independent.

MCAP publish_time is set equal to log_time; it is not the original publisher timestamp. The ROS clock drives save times; `time.time()` drives watchdog age and printed recording duration. No monotonic-clock or clock-domain alignment mechanism is explicitly implemented.

Image callbacks retain decoded BGR pixels but discard the original CompressedImage object and header. Side CameraInfo is synthesized from YAML. The code attempts to read an image message from `latest_msgs`, but that message is never cached there, so it falls back to the current save time. High CameraInfo is a separately cached message and is not explicitly paired with the selected high image.

MP4 timing comes from the configured constant playback rate. It cannot recover original image exposure times or reveal repeated source images. Small side-CameraInfo offsets and 30 FPS video metadata are therefore not evidence of physical camera synchronization.

Offline diagnostics distinguish log and header intervals, keep file order to detect regressions, report zero/duplicate headers, compare same-index left/right saved headers, and compute nearest-neighbor offsets. Same-index differences are not actuator execution latency. Positive-header filtering only excludes zero/negative stamps; it does not cure stale targets.

The client validator's camera rate and maximum gap use log_time, while synchronization uses headers. It merges all joint streams for camera-to-joint matching, which may hide stale target streams. The added audit reports matching to each joint topic independently. Neither method can create absent image timestamps.
