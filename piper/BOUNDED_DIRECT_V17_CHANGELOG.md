# Pika -> Piper V17: bounded direct MOVE P

Date: 2026-08-05

V17 keeps V16's direct mode but bounds the single-cycle jump, targeting the
1-2 s stalls reported at both high and low arm positions.

## Analysis

- Pika tracking is healthy (indicator green) and the stall happens at any
  height, so it is not a workspace clamp or a singularity region.
- Direct mode sends the FULL target every 20 ms with no per-cycle bound.
  Fast gestures create large target jumps between consecutive cycles; the
  firmware MOVE P replanner can stall for 1-2 s planning the big trajectory,
  then resume. This is position-independent and correlates with gesture
  speed.

## Changes

- New optional caps in direct mode: `direct_max_step_mm` /
  `direct_max_step_rad` (default None = V16 unbounded behavior).
- V17 sets 25 mm / 0.35 rad: slow/medium motion is identical to direct
  (the cap is not reached), fast flicks are bounded so the replanner is not
  overloaded.

## Test

1. Slow and medium moves: should be exactly like V16 (smooth).
2. Fast flicks: should no longer freeze for 1-2 s.
3. If a stall still occurs, lower the caps (e.g. 15 mm / 0.20 rad) or reduce
   `move_speed_percent` to 25.
