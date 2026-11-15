# Three-camera frame-rate investigation

The user reported frequent early frame-rate problems with one of three views. A retained diagnostic excerpt reports 56 failing high-CameraInfo rolling windows out of 493, with a maximum deviation of 1.88 FPS. Complete online logs, CPU/USB profiles and a final acceptance record were unavailable.

The strongest visible final performance change is the difference between the last backup and the current collector: preview frequency drops from 30 to 10 Hz and each displayed image is resized to 640×360. Because preview, JPEG decode and video encoding share one ROS callback thread, reducing GUI work is a plausible mitigation. There is no evidence that the final fix was hardware trigger wiring, separate USB controllers, multithreaded recording or a different camera model.

Earlier code adds raw callback counters and prints camera/joint arrival rates when an episode is saved. This helps separate message arrival from periodic saving, but those counters are not stored in the dataset. A validator variant adds per-camera rate printing without changing its core thresholds.

A separate compatibility issue was repaired by embedding complete ROS message schemas in MCAP. Older files contained CDR bytes with empty schema definitions: a ROS-aware reader could show fields, while the client's standalone decoder could not obtain resolution or headers. This explains several early validation failures without establishing a camera hardware failure. All visible collector snapshots already use the low-level MCAP writer and ROS C++ CDR serialization; an intermediate slow writer implementation is not available for comparison.

Offline review sampled 53 Piper files: 49 pass the client's six mandatory checks. Three old schema-deficient samples fail semantic decoding checks and one sample has a save gap above 69 ms. However, high CameraInfo headers contain repeated stamps and a sampled maximum interval near 267 ms. Side info uses generated save timestamps; MP4 can repeat cached pixels at a constant playback rate. The remaining physical camera drop rate is not recoverable from these files.

The supportable conclusion is that preview load reduction, schema compatibility and additional diagnostics improved the collection/validation workflow. Full elimination of physical three-camera frame loss is unproven. Future measurements need image source timestamps and frame numbers, receive/write times, queue lengths and drop counters. These recommendations were not implemented in the original collector during release preparation.
