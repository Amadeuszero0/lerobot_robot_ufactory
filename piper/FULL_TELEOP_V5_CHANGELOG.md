# Pika to Piper full teleoperation V5

Date: 2026-08-03

This profile combines the previously validated components without changing any
first-version, V2, V3, or V4 profile.

## Included behavior

- calibrated Pika-to-Piper XYZ translation mapping;
- V3 visual orientation-axis correction;
- dominant-axis orientation control at 40 percent gain;
- conservative MOVE P rate and workspace limits;
- filtered and rate-limited gripper commands;
- no CPV mode;
- no automatic parking;
- no intentional torque disable on disconnect.

## Verified starting pose

The profile was prepared around this manually verified pose:

```text
XYZ: [153.7, 45.3, 310.1] mm
Joints: [11.0, 36.6, -35.5, 33.5, -25.6, -5.1] deg
```

Before every run, confirm that the arm is in a safe, bent posture and its XYZ
position is inside the configured workspace. Initial tests must keep Pika
translation below 5 cm and rotation below 10 degrees.

## Known limitation

Small residual jitter can still be visible during vertical or pitch motion.
V5 intentionally does not introduce another filter or controller change; it
preserves the behavior that was physically verified on 2026-08-03.
