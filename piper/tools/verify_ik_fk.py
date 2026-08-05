#!/usr/bin/env python3
"""Pre-flight check for the joint-stream motion layer (V15).

Connects to the arm over CAN only (no torque enable, no motion), reads the
current joint angles and the SDK-reported end pose, then:

1. runs our pinocchio FK on the current joints for several end-effector
   frame candidates and compares each with the SDK pose;
2. runs our IK on the SDK pose seeded with the current joints and prints the
   residual.

The candidate with the smallest position/orientation error is the one to
write into the V15 profile (``ee_xyz_mm`` / ``ee_rpy_deg``).

Usage (CAN must be up):

    conda activate uf_lerobot
    cd ~/lerobot_robot_ufactory
    python piper/tools/verify_ik_fk.py --port can0
"""

import argparse
import math
import time

import numpy as np

from lerobot_robot_ufactory_piper.motors import PiperMotorsBus
from lerobot_robot_ufactory_piper.motors.tables import CALIBRATION, MOTORS
from lerobot_robot_ufactory_piper.piper_kinematics import (
    PiperKinematics,
    _rpy_to_matrix,
)


def rotation_error_deg(rpy_a_deg, rpy_b_deg) -> float:
    ra = _rpy_to_matrix(*(math.radians(v) for v in rpy_a_deg))
    rb = _rpy_to_matrix(*(math.radians(v) for v in rpy_b_deg))
    d = ra.T @ rb
    cos_theta = min(1.0, max(-1.0, (np.trace(d) - 1.0) / 2.0))
    return math.degrees(math.acos(cos_theta))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="can0")
    args = parser.parse_args()

    bus = PiperMotorsBus(
        id="verify",
        port=args.port,
        motors=MOTORS.copy(),
        calibration=CALIBRATION.copy(),
    )
    print(f"连接 CAN 端口 {args.port}（只读，不使能、不运动）...")
    bus.connect(piper_init=False)
    try:
        # The SDK needs ~1 s to receive the first valid feedback frames.
        time.sleep(1.5)
        x, y, z, roll, pitch, yaw = bus.get_end_pose()
        sdk_pose = np.array([x, y, z, roll, pitch, yaw], dtype=float)
        js = bus.piper.GetArmJointMsgs().joint_state
        q_rad = np.radians(
            np.array(
                [js.joint_1, js.joint_2, js.joint_3, js.joint_4, js.joint_5, js.joint_6],
                dtype=float,
            )
            / 1000.0
        )
    finally:
        bus.disconnect(disable_torque=False, park=False)

    print()
    print("当前关节角 (rad):", np.round(q_rad, 5).tolist())
    print("SDK 末端位姿 (mm / deg):", np.round(sdk_pose, 3).tolist())
    if np.allclose(q_rad, 0.0) or np.allclose(sdk_pose[:3], 0.0):
        print()
        print("警告: 关节/位姿仍是 0，说明还没收到有效反馈帧。")
        print("请确认机械臂已上电、CAN 正常（candump can0 有 2A1~2A8 帧），再重跑。")
    print()

    candidates = [
        ((190.0, 0.0, 0.0), (0.0, -90.0, 0.0), "官方默认 (X=190mm, pitch=-90)"),
        ((0.0, 0.0, 0.0), (0.0, -90.0, 0.0), "仅 pitch=-90"),
        ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), "joint6 原帧 (无偏移)"),
        ((190.0, 0.0, 0.0), (0.0, 0.0, 0.0), "仅 X=190mm"),
    ]

    print("========== FK 对比（我们的 FK vs SDK 位姿） ==========")
    results = []
    for ee_xyz, ee_rpy, label in candidates:
        try:
            kin = PiperKinematics(ee_xyzrpy_m=ee_xyz, ee_rpy_rad=ee_rpy)
            fk = np.asarray(kin.forward_xyzrpy(q_rad), dtype=float)
            pos_err = float(np.linalg.norm(fk[:3] - sdk_pose[:3]))
            rot_err = rotation_error_deg(fk[3:6], sdk_pose[3:6])
            results.append((pos_err, rot_err, ee_xyz, ee_rpy, label, fk))
            print(
                f"{label}\n"
                f"  FK: {np.round(fk, 2).tolist()}\n"
                f"  位置误差: {pos_err:.2f} mm   姿态误差: {rot_err:.2f} deg"
            )
        except Exception as exc:  # pragma: no cover - dependency errors
            print(f"{label} 失败: {exc}")

    if results:
        best = min(results, key=lambda r: r[0] + r[1])
        print()
        print("========== 最佳候选 ==========")
        print(
            f"{best[4]}: 位置 {best[0]:.2f} mm, 姿态 {best[1]:.2f} deg"
        )
        print(f"ee_xyz_mm: {list(best[2])}")
        print(f"ee_rpy_deg: {list(best[3])}")
        print()
        print("========== IK 自洽性检查（用最佳候选） ==========")
        kin = PiperKinematics(ee_xyzrpy_m=best[2], ee_rpy_rad=best[3])
        q_sol, residual = kin.ik(
            sdk_pose[:3], sdk_pose[3:6], q_rad
        )
        fk_back = np.asarray(kin.forward_xyzrpy(q_sol), dtype=float)
        print(f"IK 残差: {residual:.6f}")
        print(f"IK 解关节角 (rad): {np.round(q_sol, 5).tolist()}")
        print(f"IK->FK 位姿 (mm/deg): {np.round(fk_back, 2).tolist()}")
        print(
            f"IK->FK 与 SDK 位姿差: "
            f"{np.linalg.norm(fk_back[:3] - sdk_pose[:3]):.2f} mm, "
            f"{rotation_error_deg(fk_back[3:6], sdk_pose[3:6]):.2f} deg"
        )
        if best[0] > 10.0 or best[1] > 5.0:
            print()
            print("警告: 最佳候选误差仍较大，说明 SDK 位姿帧与这些候选都不匹配，")
            print("请把上面的 FK 对比结果贴给 Codex 继续调 EE 帧。")
        else:
            print()
            print("结论: 该 EE 帧与 SDK 匹配，可把候选值写进 V15 配置后开始低速测试。")


if __name__ == "__main__":
    main()
