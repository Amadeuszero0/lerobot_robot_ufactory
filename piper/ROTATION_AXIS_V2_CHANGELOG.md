# Pika to Piper rotation-axis V2

Date: 2026-08-03

This experimental profile leaves all existing and first-version profiles
unchanged.

## Evidence

Three Pika-only gestures produced these dominant axes:

- tip up: `[-5.123, 38.717, -7.275] deg` -> `+RY`
- tip right: `[-33.646, -3.847, 1.413] deg` -> `-RX`
- clockwise roll: `[10.516, 18.661, 34.806] deg` -> `+RZ`

The roll gesture contained substantial natural `RY` cross coupling. The new
mapping orthogonalizes the measured gesture axes. The guarded test profile
also selects the dominant mapped axis and scales rotation to 25 percent.

## Safety scope

`single_pika_piper_orientation_axis_v2.yaml` restricts XYZ to +/-1 mm around
the verified test pose and disables gripper commands. Its bounds must be
regenerated after any manual Piper movement. It remains MOVE P; CPV is not
used.
