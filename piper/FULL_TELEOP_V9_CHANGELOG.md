# Pika to Piper full teleoperation V9

Date: 2026-08-05

V9 targets the remaining two issues on top of V8: the gripper rising while
reaching forward, and residual jitter. It does not modify earlier profiles.

## Send frequency decision: keep 30 Hz, do not raise

The official `PikaAnyArm` ROS stack is smooth for a different reason than
frequency:

- `teleop_piper_publish.py` composes the Pika delta on the CURRENT arm pose
  every cycle (closed-loop incremental).
- `piper_IK.py` streams joint targets at 50 Hz and linearly interpolates big
  steps in 1-degree increments at 200 Hz.
- `piper_ctrl_single_node.py` streams joint positions to the CAN bus at
  200 Hz in joint control mode (MotionCtrl_2 0x01,0x01,30).

Our stack uses Piper MOVE P: every `EndPoseCtrl` makes the firmware re-plan a
trajectory. Raising the Cartesian send rate therefore adds interrupts rather
than smoothness. 30 Hz is kept, matching the peer project's proven-smooth
profile.

## Root causes addressed

1. `rotation_scale 1.0` on the calibrated cross-coupled mapping amplifies the
   natural wrist pitch a human makes while pushing the Pika forward into an
   explicit lift command - the gripper rises with forward reach. The peer
   profile runs 0.5; V9 uses 0.75, the gain at which directions were verified.
2. Tracker noise on the wrist channel is smoothed by a mild low-pass
   (`rotation_filter_alpha 0.6`); this channel was unfiltered in V8.

## V9 changes vs V8

- `rotation_scale`: 1.0 -> 0.75.
- `rotation_filter_alpha`: 1.0 (off) -> 0.6 (mild low-pass on wrist delta).
- Unchanged: wide workspace, `scale_xyz 0.8`, 6 mm / 0.10 rad steps, 15%
  speed, MOVE P, 30 Hz, calibrated rotation mapping, direct gripper mapping.

## Two-motion diagnostic (run before/with V9)

1. Push the Pika forward WITHOUT rotating the wrist: if the gripper still
   rises, the translation mapping matrix is the suspect (it was fitted from
   noisy 08-03 data and has large cross terms); if it stays level, the
   rotation channel is the cause.
2. Hold the Pika still and only pitch the wrist up/down: if the gripper
   pitches visibly with 0.75 gain, rotation is responsive and the rise was
   the coupling; if not, re-check the tool-axis correction at that pose.

## Still open (bigger changes from official code, not in this profile)

- Closed-loop incremental composition on the CURRENT arm pose instead of the
  Enter pose (official approach; needs feeding robot feedback into the
  teleop).
- Joint-space streaming with 1-degree interpolation at 200 Hz (official
  approach; needs pinocchio + URDF IK inside the LeRobot plugin).
- Tool offset parameter (official `gripper_xyzrpy: [0.19, 0, 0, 0, 0, 0]`)
  for the end-effector frame.
