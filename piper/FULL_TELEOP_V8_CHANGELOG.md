# Pika to Piper full teleoperation V8

Date: 2026-08-05

V8 ports the reach/smoothness recipe from the peer project
(`lerobot_robot_ufactory-other`) on top of V7. It does not modify V7 or any
earlier profile.

## What the peer project does differently (and what we adopt)

The peer's working profile is `config/pika/pika_piper_record_config.yaml`
using `uf::pika_split_teleop`:

- workspace `x [100,600]`, `y [-500,500]`, `z [50,600]` (our V7 was
  `z [260,360]`, which clamped every low target and caused the arm to stop
  short and fight the wall - the direct cause of "cannot bend down" plus
  jitter);
- `scale_xyz 0.8` (our V7 was 0.30, so 70% of the operator's reach was lost);
- `max_rotation_step_rad 0.15`, `max_cartesian_step_mm 8` (generous headroom);
- no deadband, no command gating, no rotation filter: every control cycle
  sends the next MOVE P target cleanly.

## What we deliberately do NOT adopt

The peer fixes wrist direction with `rotation_axis_map [1,2,0]` and
`rotation_axis_sign [-1,-1,1]`. Per the user's feedback, that J5/J6 mapping
is completely wrong. V8 keeps our calibrated `_PIKA_TO_PIPER_ROTATION`
matrix + `_PIPER_TOOL_AXIS_CORRECTION`, which were physically verified on
2026-08-03 (V3/V4 direction checks passed).

## V8 changes vs V7

- `workspace_x/y/z`: `[110,230]/[-20,110]/[260,360]` ->
  `[100,600]/[-500,500]/[50,600]` (peer-verified box).
- `scale_xyz`: 0.30 -> 0.80.
- `max_rotation_step_rad`: 0.03 -> 0.10.
- `max_cartesian_step_mm`: 5.0 -> 6.0.
- `rotation_deadband_rad`: 0.003 -> 0.0; `rotation_filter_alpha`: 0.7 -> 1.0
  (disable the low-pass; the peer path has none and reports no jitter).
- Unchanged: our calibrated rotation mapping, `rotation_scale 1.0`, MOVE P,
  30 Hz, 15% speed, direct official-gripper mapping.

## Safety notes

The workspace is now the full peer-verified box. The Piper firmware joint
limits still protect the arm, but the software no longer restricts the
end-effector to the previously calibrated region. First session: start from
the safe pose, keep translation under 5 cm and rotation under 10 degrees,
and verify no collision. The tool-axis correction was calibrated near the
old box; at far poses the wrist mapping may drift - report any direction
change rather than continuing.

## Test sequence (after pull)

1. Confirm the arm is in a safe pose, press Enter, then only pitch the Pika
   up/down in place: J5/J6 should follow.
2. Then reach slowly down-forward toward the low object: with the workspace
   unclamped and 0.8 scale, the arm should now be able to descend and pitch
   down together.
3. If wrist lag returns, compare against V7; the only remaining differences
   are workspace/scale/step - the rotation math is identical.
