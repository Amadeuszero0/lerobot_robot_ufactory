#!/usr/bin/env python3
"""Read-only PiPER-X URDF-vs-SDK verification for one or two CAN ports.

The program never enables torque and never sends a motion/gripper command.
It compares current J1..J6 feedback through the official PiPER-X URDF with
the SDK's native J6 pose, then reports the configured official-gripper TCP.
"""

from __future__ import annotations

import argparse
import math
import time

import numpy as np

from lerobot_robot_ufactory_piper_history_ed30ad0.motors import PiperMotorsBus
from lerobot_robot_ufactory_piper_history_ed30ad0.motors.tables import CALIBRATION, MOTORS
from lerobot_robot_ufactory_piper_history_ed30ad0.piper_kinematics import (
    PiperKinematics,
    _rpy_to_matrix,
)


def _joint_feedback_rad(bus: PiperMotorsBus) -> np.ndarray:
    state = bus.piper.GetArmJointMsgs().joint_state
    millideg = np.array(
        [
            state.joint_1,
            state.joint_2,
            state.joint_3,
            state.joint_4,
            state.joint_5,
            state.joint_6,
        ],
        dtype=float,
    )
    return np.radians(millideg / 1000.0)


def _rotation_error_deg(model_rotation: np.ndarray, sdk_rpy_deg) -> float:
    sdk_rotation = _rpy_to_matrix(*(math.radians(v) for v in sdk_rpy_deg))
    delta = model_rotation.T @ sdk_rotation
    cosine = min(1.0, max(-1.0, (np.trace(delta) - 1.0) / 2.0))
    return math.degrees(math.acos(cosine))


def verify_port(port: str, tcp_z_mm: float) -> bool:
    bus = PiperMotorsBus(
        id=f"verify_{port}",
        port=port,
        motors=MOTORS.copy(),
        calibration=CALIBRATION.copy(),
    )
    print(f"\n========== {port}（只读） ==========")
    bus.connect(piper_init=False)
    try:
        time.sleep(1.0)
        q = _joint_feedback_rad(bus)
        sdk = np.array(bus.get_end_pose(), dtype=float)
        j6_kin = PiperKinematics(model="piper_x")
        model_j6 = j6_kin.forward(q)
        pos_error_mm = float(
            np.linalg.norm(model_j6[:3, 3] * 1000.0 - sdk[:3])
        )
        rot_error_deg = _rotation_error_deg(model_j6[:3, :3], sdk[3:6])

        tcp_kin = PiperKinematics(
            model="piper_x", tool_xyzrpy_m=(0.0, 0.0, tcp_z_mm / 1000.0)
        )
        tcp_pose = tcp_kin.forward_xyzrpy(q)

        print("J1~J6(deg):", np.round(np.degrees(q), 3).tolist())
        print("SDK native J6 XYZ/RPY:", np.round(sdk, 3).tolist())
        print(
            "Official PiPER-X FK error: "
            f"{pos_error_mm:.3f} mm / {rot_error_deg:.4f} deg"
        )
        print(
            f"Official-gripper TCP (local Z={tcp_z_mm:.1f} mm):",
            np.round(tcp_pose, 3).tolist(),
        )
        passed = pos_error_mm <= 2.0 and rot_error_deg <= 1.0
        print("Result:", "PASS" if passed else "FAIL - do not actuate joint stream")
        return passed
    finally:
        bus.disconnect(disable_torque=False, park=False)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ports", nargs="+", default=["can_left", "can_right"])
    parser.add_argument("--tcp-z-mm", type=float, default=142.5)
    args = parser.parse_args()
    results = [verify_port(port, args.tcp_z_mm) for port in args.ports]
    passed = all(results)
    if not passed:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
