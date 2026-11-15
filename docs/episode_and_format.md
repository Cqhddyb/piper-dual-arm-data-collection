# Episodes and files

The online format is MCAP plus three MP4 videos. Typical channels are `/task_info`, four `/robot/arm_{side}_{role}/joint_states`, and three `/robot/camera/cam_{view}/image_raw/compressed/camera_info`. Despite their names, the camera channels carry CameraInfo, not RGB payloads.

Space opens a numeric MCAP and writes task JSON before the movement trigger. The collector waits for all four state topics and any nonzero leader position. Image writes additionally require high CameraInfo and all three image caches. The configured state batch count controls automatic saving; there is no pause/resume or minimum duration check.

Save releases videos, finishes MCAP and increments the ID. Discard deletes the active MCAP and video directory. Idle `r` permanently deletes ID-1. Unfinished exit discards the active recording when cleanup executes. A force kill/power loss can leave incomplete files; there is no transactional multi-file commit, fsync protocol or recovery journal.

The output layout is `<dataset>/<id>.mcap` and `<dataset>/videos/<id>/cam_*.mp4`. Some historical delivered data uses `<dataset>/<id>/cam_*.mp4`; the offline audit recognizes both. `record_config.yaml` is dataset-level and can be overwritten at recorder startup.

The recorder uses `mp4v`, not live H.264. Historical postprocessing converted some videos to H.264 and rewrote metadata; that processing is not bundled. Re-encoding does not recover missing exposures. Synthesizing missing high CameraInfo from side timestamps does not establish real high-camera timing or calibration.

The original recorder checks topic callback liveness, but not all file integrity, image count, NaN/Inf or timestamp conditions. The new offline tools supplement these checks without modifying input files. A successful structural inspection is not a complete training-quality verdict.
