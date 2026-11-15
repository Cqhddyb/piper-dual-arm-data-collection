# Publication and licensing review

This directory is a local candidate prepared for human review, not a declaration that all original project material is licensed for public redistribution. No project MIT/Apache license has been added. Third-party dependency licenses do not determine ownership of the recorder.

Before a public upload, the owner must confirm:

1. Copyright in the original collector and launch, including any employer, customer, university or collaborator agreement.
2. Permission to disclose client acceptance requirements and aggregate data statistics. Client validator source is excluded because redistribution permission is unavailable.
3. Whether any task, operator, location, device or recording details may be disclosed. Raw data, real calibration, original task metadata and internal paths are excluded.
4. Exact upstream driver/SDK versions and their license/notice requirements if their source is later added. They are dependencies only in this candidate.
5. A project-wide licensing decision by the actual rights holder. There was no root project license in the reviewed workspace; the model package had a TODO license declaration, so URDF/meshes are excluded.

The candidate keeps only the unchanged recorder and camera launch snapshots, anonymous templates, new offline diagnostics/tests and rewritten documentation. Its manifest distinguishes copied project code from newly added tools. These two original project files still require ownership confirmation; local preparation does not grant publication rights.

Excluded categories: original MCAP, MP4 and images; backup/processed datasets; environment archives and caches; client validator variants; destructive or semantically modifying historical repair tools; SDK/ROS driver/message source; robot models and calibration; original machine-specific README/environment export and raw diagnostic notes.

The local full audit contains a complete exclusion list and private evidence paths and should not be uploaded wholesale. Automated package scans check for private paths, private-address patterns, credentials/keys, forbidden data formats and archives. Their absence does not replace source ownership and contract review.

No GitHub upload, git staging, commit, push or history modification was performed during preparation. The owner handles publication after deciding the above matters.
