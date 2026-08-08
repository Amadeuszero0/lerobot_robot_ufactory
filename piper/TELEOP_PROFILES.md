# Supported Pika/Piper teleoperation profiles

The configuration directory is intentionally kept small. The profiles below
are the only recommended entry points for the current hardware.

## Default: old working dual-arm behaviour

`config/dual_pika_piper.yaml`

- Restores the pre-TCP/pre-software-IK MOVE P behaviour requested after the
  PiPER-X joint-stream experiment produced severe shaking.
- Uses the persistent left/right tracker serials and the verified direct
  official-gripper path added later.
- High response and relatively large arm swing are known characteristics of
  this profile.

Run:

```bash
uf-piper-teleop --config_path piper/config/dual_pika_piper.yaml
```

## Preserved calmer profile: verified wrist axes

`config/dual_pika_piper_verified_axes.yaml`

- Preserves the guarded-speed behaviour from commit `153d283`, after the user
  confirmed that the individual wrist gesture directions were correct.
- Uses 40% Piper speed, 6 mm Cartesian steps, 0.025 rad rotation steps and a
  0.35 Pika rotation scale.
- This is the fallback if the old working profile swings too much.

Run:

```bash
uf-piper-teleop \
  --config_path piper/config/dual_pika_piper_verified_axes.yaml
```

## Hardware preflight

`config/dual_pika_piper_preflight.yaml` remains available for guarded checks.

## Stable single-arm profile

`config/single_pika_piper_setting.yaml` was not changed by the dual-arm
cleanup.

Run:

```bash
uf-piper-teleop \
  --config_path piper/config/single_pika_piper_setting.yaml
```

## Rejected experiments

The runnable configs for TCP compensation, alternate wrist frames and
PiPER-X software joint streaming were removed after physical tests showed no
improvement or severe shaking. Their source history remains recoverable in
Git, but they must not be used for hardware operation without a new review.
