# Pika to Piper full synchronization V6

Date: 2026-08-04

V6 is an opt-in experiment. It does not modify the V5 configuration or any
earlier profile.

## Root causes addressed

1. V5 intentionally scaled translation to 35 percent, limited each target to
   3 mm, issued commands every 0.10 seconds, and used 5 percent Piper speed.
   These settings necessarily created visible target chasing.
2. Pika reports physical gripper opening distance (closed near 0 mm, open near
   100 mm), but the inherited xArm mapping inverted it. The Piper convention is
   also 0 closed and positive open, so the prior mapping was reversed.
3. Five stacked gripper filters and rate limits added substantial delay.
4. Repeated short MOVE P joint-interpolated segments are especially visible on
   Cartesian vertical motion.

## V6 changes

- translation scale: 0.35 -> 0.60;
- Cartesian look-ahead: 3 -> 6 mm;
- Piper speed: 5 -> 10 percent;
- pose interval: 0.10 -> 0.08 seconds;
- MOVE P -> MOVE L;
- direct Pika gripper distance mapping using the measured 0.4-98.1 mm range;
- one-sample gripper median, 0.80 low-pass alpha, 0.25 maximum normalized step;
- smaller gripper command deadband and 0.03 second send interval.

## Limits

Pika, Vive tracking, the host control loop, SocketCAN, and Piper's internal
trajectory controller all add latency. Exact zero-latency synchronization is
not physically achievable. V6 targets substantially closer temporal response
while retaining workspace, step, and speed limits.
