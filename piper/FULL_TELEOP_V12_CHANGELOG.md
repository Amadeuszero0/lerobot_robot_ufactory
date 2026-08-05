# Pika to Piper full teleoperation V12

Date: 2026-08-05

V12 targets the residual latency reported after V11. Earlier profiles are
not modified.

## Why latency remains in V11

1. The control loop ran at 30 Hz (~33 ms) while the Piper X controller
   appears to tick at 50 Hz, so each command can wait up to one controller
   tick before being picked up.
2. The rotation low-pass at 0.4 alpha and 30 Hz had a ~50 ms time constant.
3. Step-limited chasing: at 15% speed the arm lags the command stream.

## V12 changes vs V11

- `fps`: 30 -> 50 (command cadence aligned with the arm's native 50 Hz).
- `max_cartesian_step_mm`: 6.0 -> 3.6 (same ~180 mm/s command rate).
- `max_rotation_step_rad`: 0.10 -> 0.06 (same ~3 rad/s command rate).
- `move_speed_percent`: 15 -> 20 (arm tracks the faster command stream).
- `translation_deadband_mm`: 1.0 -> 0.5, `rotation_deadband_rad`:
  0.006 -> 0.004 (same relative deadband at 50 Hz).
- `rotation_filter_alpha` stays 0.4; at 50 Hz its time constant drops to
  ~30 ms.
- Everything else identical to V11 (measured translation, workspace, 0.8
  scale, MOVE P, direct gripper).

## Test order

1. Hold still: should still be rock-solid (deadband preserved).
2. Slow then fast moves: latency should be visibly lower.
3. If jitter returns at 50 Hz / 20% speed, first revert `move_speed_percent`
   to 15, then 12 (or drop `fps` to 40) before touching the mapping.

## Note

If 50 Hz does not reduce latency, the remaining lag is the MOVE P planning +
execution itself; the next step is the official-style joint-space streaming
(see V11 changelog).
