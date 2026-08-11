# PiPER-X J5/J6 and tool-centre control

## Root cause

The normal `uf::piper` Cartesian path sends `EndPoseCtrl`.  The Piper
firmware then chooses a joint solution internally, so a software TCP offset
cannot force J5/J6 to participate.

The repository also contained an experimental software-IK path, but it used
the ordinary Piper URDF.  PiPER-X has different J4/J5/J6 origins, fixed
rotations, and limits.  Applying the ordinary Piper model to PiPER-X produces
the wrong wrist Jacobian even if the Pika axis mapping is correct.

The supplied `Lerobot-Real-main` reference was checked carefully.  Its Pika
rigid transforms are useful for gesture axes, but its Piper Cartesian backend
still calls `ModeCtrl(MOVE_P)` plus `EndPoseCtrl`; it does not implement a
PiPER-X software IK or deterministic wrist allocation.

## Implemented path

- `piper/urdf/piper_x_kinematic.urdf` contains the six-joint chain from
  AgileX's official `agx_arm_urdf` PiPER-X model.
- PiPER-X J6 FK was checked against two real SDK feedback samples supplied
  during dual-arm testing.  Each position error was 0.259 mm and orientation
  component error was at most 0.0011 degrees.
- The official gripper centre is represented as local link6 Z = 142.5 mm
  (4.5 mm gripper base offset + 138 mm finger-centre origin).
- `uf::dual_piper_joint_stream` solves each arm independently from its current
  joint feedback and sends normal-position `JointCtrl` targets.
- MIT mode is not used.
- Existing `uf::piper`, `dual_pika_piper.yaml`, and all single-arm profiles
  remain unchanged.

## Mandatory read-only verification

After pulling the commit on Linux and reinstalling the editable packages:

```bash
cd ~/lerobot_robot_ufactory
conda activate uf_lerobot
unset PYTHONPATH
python -m pip install -e .
python -m pip install -e ./piper
hash -r

python piper/tools/verify_piper_x_model.py
```

Both CAN ports must print `Result: PASS`.  This tool never enables torque and
never sends a motion or gripper command.

Then run the Pika-to-IK preview:

```bash
uf-piper-teleop \
  --config_path piper/config/dual_pika_piper_x_ik_preview.yaml
```

Press Enter, then test one Pika at a time while both Piper arms remain
stationary.  The profile is read-only.  It prints lines such as:

```text
IK PREVIEW left_piper_x_preview ... wrist(J4/J5/J6)=[...]
```

For each hand, test front up/down, front left/right, and roll.  Send the full
preview output for review.  Do not run the actuated profile if:

- either model check reports FAIL;
- a wrist gesture predicts only J1-J3 while J4-J6 remain near zero;
- any predicted joint delta jumps discontinuously or heads into a joint limit;
- the preview reports repeated large IK residual warnings.

## Guarded actuated profile

`piper/config/dual_pika_piper_x_joint_stream_stage1.yaml` is intentionally
slow and is only for the next stage after the read-only output is reviewed.
It keeps torque enabled on disconnect and uses 0.20 degree maximum joint
steps.  It must not be used as the first test of this change.

## Official model sources

- https://github.com/agilexrobotics/agx_arm_urdf/tree/main/piper_x/urdf
- https://github.com/agilexrobotics/piper_sdk/blob/master/asserts/V2/INTERFACE_V2.MD
