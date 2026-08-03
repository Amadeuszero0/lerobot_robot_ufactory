# Pika to Piper smooth MOVE P and gripper experiment

Date: 2026-08-03

## Preserved baseline

The known first working configurations are unchanged. This experiment uses
`config/single_pika_piper_smooth_gripper_test.yaml` and the new teleoperator
type `uf::piper_pika_teleop`.

## Findings

- The control loop printed every action and every Piper pose command. Terminal
  I/O introduces variable latency into a real-time loop, so those unconditional
  prints were removed.
- Gripper and pose commands shared one timer. The gripper was resent even when
  unchanged, while a new gripper command could also influence pose scheduling.
- Raw Pika gripper distance was sent without median filtering, low-pass
  filtering, hysteresis, or a rate limit.
- The first profile sent MOVE P endpoints only 1 mm ahead every 0.10 seconds.
  Piper could finish each tiny segment before the next command, producing the
  visible stop/start or frame-by-frame motion.

## Changes

- Added a three-sample median, low-pass, deadband, and per-frame rate limit for
  Pika gripper input. Filtering starts from Piper's measured gripper position
  when teleoperation is enabled, avoiding a command jump at Enter.
- Split pose and gripper scheduling. Gripper commands are sent only after a
  meaningful change, with a low-rate keepalive.
- Removed per-frame terminal output from the hardware control path.
- Added a separate experimental profile with a 4 mm MOVE P look-ahead while
  keeping the proven 0.10 second command interval and 5 percent speed.

## Reference projects

- `lerobot_robot_piper-master` confirms Piper's normalized gripper range and
  direct `GripperCtrl` command path.
- `lerobot_pika_piper` contains gripper low-pass/deadband/rate-limit and pose
  filtering ideas. Its forced fixed start pose is intentionally not copied,
  because that behavior is unsafe for the current physical setup.

## Required staged validation

1. With Piper motion disabled, verify Pika gripper input still spans 0 to 1.
2. Run the new profile with an empty Piper gripper and test only gripper motion.
3. Test a 5 cm translation with wrist held fixed.
4. Test a wrist rotation below 10 degrees.
5. Stop immediately if the arm jumps, sags, or produces abnormal noise.
