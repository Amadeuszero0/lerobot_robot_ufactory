# PiperX full tool-frame preflight

This experiment is isolated from the verified single-arm and formal dual-arm
profiles. It addresses the case where Cartesian motion works but the PiperX
wrist appears pinned or rotates around the J6 flange instead of the gripper
centre.

## Root cause

`piper_sdk.EndPoseCtrl` commands the native J6 frame. A translation-only TCP
offset cannot represent the PikaAnyArm tool because its gripper-centre frame is
also rotated relative to J6.

The official AgileX PikaAnyArm configuration specifies a 190 mm tool length.
The supplied Lerobot-Real Piper implementation expresses the full transform as:

```text
E (J6 -> gripper centre) = Ry(-90 deg) @ Tx(190 mm)
C = inverse(E) = [-190, 0, 0, 0, 90, 0]
```

The command path therefore computes the target gripper-centre pose `G` first,
then sends the native J6 pose `S = G @ C` to Piper.

## Changes

- Added `dual_pika_piper_piperx_preflight.yaml`.
- Selected the existing full `rotation_style: senior` rigid-transform path.
- Configured `tracker_to_robot_eef: [-190, 0, 0, 0, 90, 0]`.
- Kept follower `tcp_offset_mm` disabled to prevent double compensation.
- Applied the measured Vive-world to Piper-base rotation independently from the
  tool transform.
- Added rotation scaling and filtering to the full-transform path so the first
  hardware test can be deliberately small.
- Disabled both grippers in the preflight to isolate wrist/J5/J6 behaviour.

## First hardware validation

Test one side at a time. Keep the other Pika still. Use only 5-10 degree wrist
gestures and small translations. Stop if the gripper centre moves rapidly,
directions are wrong, or either arm approaches a joint/workspace limit.

Do not add `tcp_offset_mm` to this profile. Once the tool-frame result is
verified, copy the mapping into a separate full-speed profile and re-enable the
grippers there.
