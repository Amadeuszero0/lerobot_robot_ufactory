# Pika to Piper full teleoperation V14

Date: 2026-08-05

V14 raises the two speed caps that bind when the Pika accelerates. Earlier
profiles are not modified.

## Why the arm cannot follow acceleration (MOVE P nature)

MOVE P plans each segment at a fixed speed. Two caps bind during fast moves:

1. `move_speed_percent 20`: the arm's actual velocity is capped.
2. Per-cycle steps: 3.6 mm at 50 Hz = ~180 mm/s commanded; 0.06 rad at
   50 Hz = ~172 deg/s commanded.

When the Pika accelerates, the target runs ahead faster than either cap, so
the arm chases with a constant lag.

## V14 changes vs V13

- `move_speed_percent`: 20 -> 30.
- `max_cartesian_step_mm`: 3.6 -> 5.0 (~250 mm/s commanded).
- `max_rotation_step_rad`: 0.06 -> 0.08 (~230 deg/s commanded).
- Unchanged: 1:1 scales, 50 Hz, deadbands, rotation filter 0.4, measured
  translation mapping, MOVE P.

## Expectations

- Following during acceleration should improve; residual lag will remain
  because MOVE P cannot match velocity/acceleration profiles.
- Jitter may stay similar or increase slightly with the higher speed. If it
  increases, first lower `move_speed_percent` to 25, then 20.
- If the follow is still not acceptable, the next step is a velocity-following
  mode: either the SDK's MOVE CPV streaming mode (untested; stage with
  locked orientation first) or the official joint-space streaming stack.
