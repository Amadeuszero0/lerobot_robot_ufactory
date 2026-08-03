# Pika to Piper rotation-axis V3

Date: 2026-08-03

This experimental profile leaves the first-version and V2 profiles unchanged.

## Observed V2 behavior

- Pika up made Piper move right.
- Pika down made Piper move left.
- Pika left made Piper move up.
- Pika right made Piper move down.

These four observations are consistent with a 90-degree frame mismatch, not
four independent sign errors. V2 mapped the Pika gesture axes correctly, but
treated Piper's local MOVE P rotation components as visual up/right axes.

## V3 correction

V3 applies the following opt-in correction after the calibrated Pika rotation
mapping and before dominant-axis selection:

```text
[new RX, new RY, new RZ] = [old RY, -old RX, old RZ]
```

Expected behavior:

- Pika up -> Piper up
- Pika down -> Piper down
- Pika left -> Piper left
- Pika right -> Piper right
- roll direction remains unchanged

The guarded V3 profile keeps the V2 translation bounds, 25 percent rotation
scale, disabled gripper commands, and MOVE P control. If Piper is manually
moved, regenerate the XYZ bounds before using this profile.
