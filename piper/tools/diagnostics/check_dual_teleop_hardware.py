#!/usr/bin/env python3
"""只读检查双 Pika、双 Tracker 和双 Piper，不发送任何机器人命令。"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import yaml

from lerobot_robot_ufactory.devices.pika.pika_device import PikaDevice
from lerobot_robot_ufactory_piper.motors import PiperMotorsBus
from lerobot_robot_ufactory_piper.motors.tables import CALIBRATION, MOTORS


SIDES = ("left", "right")
DEFAULT_CONFIG = (
    Path(__file__).resolve().parents[2]
    / "config"
    / "dual_pika_piper_x_full_speed_v7.yaml"
)


def _load_config(path: Path) -> dict[str, Any]:
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    robots = data.get("robot", {}).get("robots", {})
    teleops = data.get("teleop", {}).get("teleops", {})
    if set(robots) != set(SIDES) or set(teleops) != set(SIDES):
        raise ValueError("配置必须同时包含 left/right robot 与 teleop")
    return data


def _read_pipers(config: dict[str, Any]) -> dict[str, Any]:
    buses: dict[str, PiperMotorsBus] = {}
    try:
        for side in SIDES:
            robot = config["robot"]["robots"][side]
            bus = PiperMotorsBus(
                id=f"{side}_readonly_check",
                port=robot["port"],
                motors=MOTORS.copy(),
                calibration=CALIBRATION.copy(),
            )
            bus.connect(handshake=False, piper_init=False)
            buses[side] = bus

        result: dict[str, Any] = {}
        for side, bus in buses.items():
            pose = bus.get_end_pose()
            joints = bus.get_joint_radians()
            arm_status = bus.get_arm_status()
            motor_status = bus.get_motor_status(require_enabled=False)
            if not all(math.isfinite(value) for value in (*pose, *joints)):
                raise RuntimeError(f"{side} Piper 返回非有限反馈")
            result[side] = {
                "port": bus.port,
                "end_pose_xyz_mm_rpy_deg": [round(value, 3) for value in pose],
                "joints_rad": [round(value, 6) for value in joints],
                "arm_status": int(getattr(arm_status, "arm_status", 0)),
                "error_code": int(getattr(arm_status, "err_code", 0)),
                "motors_enabled": [
                    bool(
                        getattr(motor_status, f"motor_{joint_index}")
                        .foc_status.driver_enable_status
                    )
                    for joint_index in range(1, 7)
                ],
            }
        return result
    finally:
        for bus in reversed(list(buses.values())):
            bus.disconnect(disable_torque=False, park=False)


def _read_pikas(config: dict[str, Any]) -> dict[str, Any]:
    senses: list[Any] = []
    try:
        result: dict[str, Any] = {}
        for side in SIDES:
            teleop = config["teleop"]["teleops"][side]
            device = PikaDevice(
                1,
                pika_sense_port=teleop["port"],
                pika_tracker_device=teleop["tracker_device_id"],
            )
            sense = device.pika_sense
            senses.append(sense)
            pose = sense.get_pose(device.pika_tracker_device)
            if pose is None:
                raise RuntimeError(
                    f"{side} Tracker {device.pika_tracker_device} 没有新鲜位姿"
                )
            distance = sense.get_gripper_distance()
            result[side] = {
                "port": teleop["port"],
                "tracker_device_id": device.pika_tracker_device,
                "position_m": [round(float(value), 6) for value in pose.position],
                "quaternion_xyzw": [
                    round(float(value), 6) for value in pose.rotation
                ],
                "gripper_distance_mm": (
                    None if distance is None else round(float(distance), 3)
                ),
            }
        return result
    finally:
        for sense in reversed(senses):
            try:
                sense.disconnect()
            except Exception:
                pass


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", nargs="?", type=Path, default=DEFAULT_CONFIG)
    args = parser.parse_args()
    config = _load_config(args.config)
    print(
        json.dumps(
            {
                "piper": _read_pipers(config),
                "pika": _read_pikas(config),
                "robot_commands_sent": False,
            },
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
        )
    )


if __name__ == "__main__":
    main()
