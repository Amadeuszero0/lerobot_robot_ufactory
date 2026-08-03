# Pika wrist and gripper test change log

Date: 2026-08-03

## Symptom

- Piper translation followed Pika correctly.
- Wrist joints appeared not to follow visibly.
- The Piper gripper did not follow the Pika gripper.

## Root cause

- The rollback configuration explicitly set both `use_gripper` and
  `send_gripper` to `false`, so no gripper action was produced or sent.
- `max_rotation_step_rad` was `0.0002`.  With a `0.10 s` minimum command
  interval this limited visible orientation motion to about `0.115 deg/s`.
- The Pika SDK may transiently return no gripper distance; the old code tried
  to clamp the value before checking for `None`.
- The Piper translation mapping modified the mutable parent action cache in
  place, which could reapply the mapping if tracking temporarily dropped out.

## Changes

- Added `single_pika_piper_wrist_gripper_test.yaml` without modifying the
  known-working rollback configuration.
- Enabled Pika gripper input and Piper gripper output in the new configuration.
- Raised the rotation step limit to a still-conservative `0.002 rad`.
- Preserved the last valid gripper command across a transient missing sample.
- Applied the Pika-to-Piper translation mapping to a copied action dictionary.

## Safety and test scope

- Test the gripper with no object between the fingers.
- Test wrist rotation with translation held still and only a small angle first.
- Do not increase the rotation limit again until measured motion is confirmed.
- The original `single_pika_piper_first_version_nocam.yaml` remains the rollback
  baseline.
