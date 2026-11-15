# Audit scope and interpretation

The local engineering audit inventoried 3,825 MCAP files and 11,458 dataset videos, including backup and processed copies. These counts are not unique valid episodes. A separate non-Piper reference file was excluded from Piper sampling.

MCAP magic/summary inspection found 3,489 readable summaries, 333 empty files and three incomplete files. Full MP4 metadata inspection found 11,171 readable video tables, 99 empty videos and 188 files with atom boundaries beyond the available file/parent extent. File-existence checks alone would miss these failures.

Deep CDR/CRC checks sampled the first/middle/last eligible file per group: 53 Piper files and 1,777,613 joint/CameraInfo messages. No NaN/Inf was found in those sampled numeric arrays. Different historical files save state near 50 or 100 Hz, while camera metadata is saved near 30 Hz. Repeated/zero/stale headers and model-reference range mismatches remain visible. No claim of full-dataset numerical validation is made.

Eighteen valid videos were cross-checked with ffprobe and decoded at the beginning, middle and end: 54 single-frame probes succeeded. This is not full decoding of all frames or proof that images are never duplicated. Constant MP4 playback FPS is not exposure FPS. CameraInfo records are not image payloads, and generated side-camera stamps cannot establish hardware synchronization.

For each dataset audit, inspect the following outputs:

| Output | Scope |
|---|---|
| mcap_inventory.csv/json | All MCAP sizes, summary counts, schema text sizes and missing topics |
| video_inventory.csv | All video sample-table sizes, counts, dimensions, codec and errors |
| deep_samples.json | Selected CDR fields, dimensions, numeric ranges, timing and synchronization |
| dataset_summary.json | Group totals, durations and explicit sampling scope |

The recorded audit used extra local cross-checks for video pairing and the unmodified client validator; those scripts/results and raw filenames are retained in the private full audit, not bundled here. Re-running `audit_dataset.py` generates its documented four output types, not the client-validation report.

`inspect_mcap.py` returns structural/decoding status, not a certification of every quality metric. Its JSON may report issues even when the command exits successfully. The audit lists timestamps in file order so it can expose regressions that a time-sorted reader would hide. Range flags against a supplied URDF or the 0–0.1 m gripper heuristic are diagnostic references, not verified hardware safety limits.

Aggregate statistics and client acceptance thresholds also require the owner's publication review. No raw messages, images, original task metadata or private dataset filenames are included in this candidate package.
