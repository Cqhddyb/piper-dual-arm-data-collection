# State and action semantics

| Stream | Position | Velocity | Effort |
|---|---|---|---|
| left/right follow | 6 rotary feedback angles + gripper opening | Usually 6 joint values | Usually 6 estimated joint efforts + gripper effort |
| left/right leader | 6 absolute target angles + target opening, read from CAN commands | Initialized zeros or empty after processing | Initialized zeros or empty after processing |

Joint order is `joint1..joint6,gripper`, with float64 arrays. Six rotary positions are converted to radians by the external driver; gripper stroke is divided by 1,000,000 to meters. Feedback velocity is SDK motor speed divided by 1,000, documented as rad/s. Effort is current-derived through SDK coefficients; the inspected SDK calls its unit `N/m`, which requires vendor clarification before interpreting it as calibrated torque or fingertip force.

Leader positions come from SDK `GetArmJointCtrl()` (CAN command IDs 0x155–0x157) and `GetArmGripperCtrl()` (0x159). They are absolute control targets, not increments, not measured follower states, and not a proven raw input-device measurement. The upstream target generator is not included.

There is no saved unified 14-dimensional `state` or `action` field. A downstream layout could concatenate left seven values then right seven values, but must explicitly define names, timestamp matching, latency and missing-value handling. The current exporter retains separate topics, original lengths and timestamps. No normalization, padding, action mask, interpolation or LeRobot conversion is supplied.

The collector treats all-zero leader positions as suspicious and can hold previous arrays with a new header. Those samples are not marked. Its trigger tests nonzero position, not actual movement between consecutive frames. Feedback and target values in one save batch can have different source ages.

The external driver can publish end-effector pose and status, but this recorder does not subscribe to or save those fields. Gripper status codes, joint temperature/voltage/current and per-frame validity are not recorded. A six-element velocity array must not be padded and described as measured gripper velocity.
