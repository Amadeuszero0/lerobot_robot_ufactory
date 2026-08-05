# Pika to Piper full teleoperation V11

Date: 2026-08-05

V11 keeps the V10 measured translation mapping and verified rotation, and
targets the remaining jitter. Earlier profiles are not modified.

## What still causes jitter in V10

1. MOVE P replanning: every ~33 ms the firmware receives a new `EndPoseCtrl`
   endpoint and re-plans a short trajectory, producing stop-and-go motion.
   Raising the Cartesian send rate would make this worse (more interrupts),
   which is why 30 Hz is kept.
2. Tracker noise on the wrist channel passes through at 0.6 low-pass.
3. With zero deadband, micro-tremor from the operator/tracker is sent as
   commands even when the Pika is meant to be still.

## V11 changes

- `translation_deadband_mm`: 0 -> 1.0 (no pose command while holding still).
- `rotation_deadband_rad`: 0 -> 0.006 (~0.34 deg).
- `rotation_filter_alpha`: 0.6 -> 0.4 (stronger wrist smoothing, ~80 ms
  time constant at 30 Hz).
- Everything else identical to V10 (measured translation, workspace, 0.8
  scale, MOVE P, 30 Hz, 15% speed, direct gripper).

## Test order

1. Hold the Pika still: the arm should now be completely stationary.
2. Slow moves: should be smoother, wrist slightly softer (0.4 alpha).
3. Normal teleop: confirm the direction/amplitude from V10 is preserved.
4. If wrist feels too laggy, raise `rotation_filter_alpha` back to 0.6.

## If motion jitter remains (the "彻底" paths)

- **Joint-space streaming (official PikaAnyArm approach)**: stream joint
  targets at 50-200 Hz with 1-degree interpolation, using pinocchio + the
  Piper URDF for IK. This is how the official stack is smooth; it requires
  porting the IK/URDF into this plugin. Larger change, but the proven way.
- **MOVE CPV streaming experiment**: the SDK has a continuous-path mode
  (0x05) already wired into the follower; untested on this hardware. Quick
  to try in a locked-orientation profile first, but V6's MOVE L incident
  means it must be staged carefully.
