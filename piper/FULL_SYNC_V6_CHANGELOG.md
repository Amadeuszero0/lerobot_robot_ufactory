# Pika to Piper full synchronization V6

Date: 2026-08-04

V6 is an opt-in gripper-mapping experiment. It does not modify the V5
configuration or any earlier profile.

## Safety update

The first V6 revision used MOVE L, 10 percent speed, a 6 mm Cartesian step,
and a 0.08 second command interval. Physical testing on 2026-08-04 produced
stronger jitter and one unexpected upward movement. That motion configuration
is rejected and must not be used.

The checked-in V6 profile has therefore been reverted to the previously
verified V5 MOVE P motion envelope. Only the direct gripper-distance mapping
and its faster filter remain enabled for staged testing.

## Root causes addressed

1. V5 intentionally scaled translation to 35 percent, limited each target to
   3 mm, issued commands every 0.10 seconds, and used 5 percent Piper speed.
   These settings necessarily created visible target chasing.
2. Pika reports physical gripper opening distance (closed near 0 mm, open near
   100 mm), but the inherited xArm mapping inverted it. The Piper convention is
   also 0 closed and positive open, so the prior mapping was reversed.
3. Five stacked gripper filters and rate limits added substantial delay.
4. Cartesian vertical motion still shows residual MOVE P jitter and requires a
   separate controller change after instrumented testing.

## V6 changes

- direct Pika gripper distance mapping using the measured 0.4-98.1 mm range;
- one-sample gripper median, 0.80 low-pass alpha, 0.25 maximum normalized step;
- smaller gripper command deadband and 0.03 second send interval.

Motion remains at 0.35 translation scale, 3 mm Cartesian step, 5 percent
speed, 0.10 second command interval, and MOVE P.

## Limits

Pika, Vive tracking, the host control loop, SocketCAN, and Piper's internal
trajectory controller all add latency. Exact zero-latency synchronization is
not physically achievable. V6 targets substantially closer temporal response
while retaining workspace, step, and speed limits.
