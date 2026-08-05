# Pika to Piper full teleoperation V7

Date: 2026-08-05

V7 restores wrist rotation amplitude on top of the verified
`final_v2_gripper` envelope. It does not modify `final_v2_gripper` or any
earlier profile.

## Problem (user feedback on final_v2_gripper)

1. The arm cannot pitch down far enough to reach a low, slightly forward
   object while translating; it tends to rise and move forward instead of
   bending the wrist down.
2. J5/J6 rotate noticeably less than the Pika wrist input.
3. A small amount of jitter remains, most visible during vertical motion.

## Root causes addressed

1. `rotation_scale` was 0.75, so only 75 percent of the Pika wrist rotation
   was commanded.
2. `max_rotation_step_rad` was 0.012 rad at 30 Hz, capping the wrist at
   roughly 21 degrees per second; faster Pika gestures were cut down by the
   step limiter.
3. `move_speed_percent` 10 shared one trajectory speed budget between
   translation and rotation, throttling J5/J6 further.
4. Residual vertical jitter comes from MOVE P replanning every ~33 ms plus a
   staircase-limited rotation channel; a mild low-pass on the mapped rotation
   delta is added to reduce it.

## V7 changes

- `rotation_scale`: 0.75 -> 1.0 (1:1 wrist amplitude).
- `max_rotation_step_rad`: 0.012 -> 0.03 (~57 deg/s headroom at 30 Hz).
- `move_speed_percent`: 10 -> 15.
- `rotation_deadband_rad`: 0.002 -> 0.003.
- New `rotation_filter_alpha` option (code default 1.0 = off); V7 uses 0.7
  for a light exponential low-pass on the mapped rotation delta. Set it back
  to 1.0 if wrist lag reappears.
- Translation (scale_xyz 0.30, 5 mm step), MOVE P, 30 Hz and the direct
  gripper mapping are unchanged from `final_v2_gripper`.

## Still open

- `tracker_to_robot_eef` translation offset is still `[0, 0, 0]`. Rotating
  the Pika around its handle can still produce phantom translation because
  the tracker is not at the gripper center. Re-run the offset calibration
  2-3 times and write a consistent result into a separate test profile
  before adopting it.
- `workspace_z` lower bound is 260 mm. If the target object is below that,
  the command is clamped; verify physical reachability before widening the
  box.

## Test sequence (after pull)

1. Confirm the arm is in the verified safe pose (XYZ inside the workspace).
2. Hold the Pika still, press Enter, then only pitch the Pika up/down in
   place: J5/J6 should follow with roughly 1:1 amplitude.
3. Then try a slow diagonal move down-forward toward the low object; the
   wrist should pitch down while translating.
4. If wrist lag appears, set `rotation_filter_alpha: 1.0` and retest.
5. For the first run keep Pika translation under 5 cm and rotation under
   10 degrees.
