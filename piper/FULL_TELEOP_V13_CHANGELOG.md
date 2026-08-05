# Pika to Piper full teleoperation V13

Date: 2026-08-05

V13 makes the Pika -> Piper mapping 1:1 after V12 was accepted for jitter and
latency but felt small in amplitude. Earlier profiles are not modified.

## What the mapping ratios were

- V12 translation: `scale_xyz 0.80` (Pika 10 cm -> Piper 8 cm).
- V12 rotation: `rotation_scale 0.75` (Pika 40 deg -> Piper 30 deg).

## V13 changes

- `scale_xyz`: 0.80 -> 1.00 (1:1 translation).
- `rotation_scale`: 0.75 -> 1.00 (1:1 wrist rotation).
- Everything else identical to V12 (50 Hz, measured translation matrix,
  deadbands 0.5 mm / 0.004 rad, MOVE P, 20% speed, direct gripper).

## Notes

- The earlier "gripper rises when pushing forward" with rotation gain 1.0
  was traced to the broken fitted translation matrix (fixed in V10), not to
  the rotation gain itself. If the wrist feels over-responsive or coupling
  artifacts return, first revert `rotation_scale` to 0.75, then
  `scale_xyz` to 0.80.
- Rate limits still apply: 3.6 mm / cycle at 50 Hz (~180 mm/s) and
  0.06 rad / cycle at 50 Hz (~172 deg/s). Fast gestures are followed with a
  small chase; the full 1:1 amplitude is reached when the gesture is held.
  To raise the rate cap, increase `max_cartesian_step_mm` /
  `max_rotation_step_rad` before touching the scales.
