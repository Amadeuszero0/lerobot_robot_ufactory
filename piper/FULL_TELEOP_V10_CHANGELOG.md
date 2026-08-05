# Pika to Piper full teleoperation V10

Date: 2026-08-05

V10 replaces the translation mapping with one measured from the real tracker
axes. Earlier profiles are not modified.

## Measurement results (2026-08-05, tool run #2)

Translation (raw tracker/lighthouse world frame -> Piper base frame):

- Pika forward measured dir (+0.889, -0.422, -0.179) -> Piper +X
- Pika right measured dir (-0.338, -0.941, -0.017) -> Piper -Y
- Pika up measured dir (+0.181, -0.060, +0.982) -> Piper +Z

Kabsch fit error: forward 1.1 deg, right 1.1 deg, up 0.8 deg.

Rotation pass-through was verified as healthy at 0.75 gain:

- pitch down 40.1 deg -> commanded -RX 30.1 deg (visual down)
- yaw right 51.9 deg -> commanded +RY 38.9 deg (visual right)
- roll 54.7 deg -> commanded +RZ 41.0 deg

The previously observed "arm rises when pushing forward" and "arm swings
during wrist twist" were caused by the old fitted translation matrix
(cross terms up to 0.82), not by the rotation channel.

## V10 changes

- New code option `use_raw_translation_mapping` (default False) in the piper
  plugin; V10 enables it.
- When enabled, the teleop computes translation from the RAW tracker
  position delta (world frame) with the measured orthonormal matrix
  `_RAW_TO_PIPER_TRANSLATION`, bypassing the old fitted matrix path.
- Rotation mapping, workspace, scale (0.8), 30 Hz, MOVE P, 15% speed and the
  direct gripper mapping are unchanged from V9.

## Remaining known issue

`tracker_to_robot_eef` translation offset is still `[0,0,0]`. The tracker is
not at the Pika rotation center, so wrist rotations still produce phantom
translation (tens of mm). The direction of that phantom is now corrected by
the measured matrix, but its amplitude remains until the offset is
calibrated. If V10 feels acceptable except for wrist-induced drift, the next
step is a careful offset calibration (2-3 consistent runs).
