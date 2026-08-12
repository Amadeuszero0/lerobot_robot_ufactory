# Third-party notices

This integration contains adapted portions of the local
`lerobot_robot_piper-master` project, whose package metadata declares the
Apache License 2.0. The adapted portions are the Piper motor calibration,
normalization, CAN follower, and leader interfaces.

It also interoperates with and imports `lerobot_robot_ufactory` and LeRobot.
The parent UFACTORY repository includes an Apache-2.0 license file, and
LeRobot is distributed under Apache-2.0.

No copyright or license ownership is transferred by this integration.

The optional `official_ik` dual-arm path also adapts the Piper-X tool-frame
formula and Pinocchio/CasADi IK worker from the local Apache-2.0
`Lerobot-Real-main` project. That implementation in turn documents its IK as
adapted from AgileX PikaAnyArm. Pinocchio, CasADi, the Piper SDK and the
external AgileX URDF remain subject to their own licenses.
