"""Piper/PiPER-X joint-streaming follower.

Receives the same Cartesian Pika teleop actions as ``PiperFollower``, but
executes them by solving IK with the URDF kinematics (pure numpy) and
streaming joint targets through the SDK's normal joint-position mode
(``MotionCtrl_2(0x01,0x01,speed,0x00)`` + ``JointCtrl``), with per-cycle
Cartesian and joint step limits.  Unlike ``EndPoseCtrl``, this makes the
J4/J5/J6 allocation explicit and testable.

Kept as a separate robot type (``uf::piper_joint_stream``) so the MOVE P
path (V13/V14, ``uf::piper``) is completely untouched.
"""

from __future__ import annotations

import logging
import math
import time
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass, field
from functools import cached_property
from typing import Any, Callable, TypeVar

import numpy as np

from lerobot.cameras import Camera, CameraConfig
from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.robots import Robot, RobotConfig
from lerobot.utils.errors import DeviceNotConnectedError

from .motors import PiperMotorsBus
from .motors.tables import CALIBRATION, MOTORS
from .joint_stream_stabilizer import JointStreamStabilizer
from .piper_kinematics import PiperKinematics
from .pose import (
    axis_angle_to_rpy_degrees,
    clamp,
    rpy_degrees_to_axis_angle,
    vector_step_towards,
)

logger = logging.getLogger(__name__)

POSE_KEYS = ("pose.x", "pose.y", "pose.z", "pose.rx", "pose.ry", "pose.rz")
_RAD_TO_MILLIDEG = 1000.0 * 180.0 / math.pi
T = TypeVar("T")


@RobotConfig.register_subclass("uf::piper_joint_stream")
@dataclass(kw_only=True)
class PiperJointStreamConfig(RobotConfig):
    """One Piper arm controlled by joint-space streaming."""

    port: str
    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    configure_role_on_connect: bool = True
    piper_init_on_connect: bool = False
    enable_torque_on_connect: bool = True
    park_on_connect: bool = False
    park_on_disconnect: bool = False
    disable_torque_on_disconnect: bool = False
    # Calculate and report IK without sending ModeCtrl/JointCtrl/GripperCtrl.
    dry_run: bool = False
    dry_run_log_interval_s: float = 0.25

    # Cartesian target clamps (same safety semantics as PiperFollower).
    max_cartesian_step_mm: float = 8.0
    max_rotation_step_rad: float = 0.08
    translation_deadband_mm: float = 0.5
    rotation_deadband_rad: float = 0.004
    workspace_x: tuple[float, float] | None = None
    workspace_y: tuple[float, float] | None = None
    workspace_z: tuple[float, float] | None = None

    # Joint streaming.
    max_joint_step_deg: float = 3.0
    # 修复模式默认关闭，确保历史配置仍可原样复现。开启后，IK 以上一次关节命令为连续
    # 参考，并增加速度、加速度、跟随误差和 IK 解跳变保护。
    stabilized_stream: bool = False
    control_frequency_hz: float = 50.0
    max_joint_velocity_deg_s: float = 6.0
    max_joint_acceleration_deg_s2: float = 60.0
    max_joint_following_error_deg: float = 1.5
    max_ik_solution_jump_deg: float = 2.0
    ik_max_iter: int = 10
    ik_damping: float = 1e-3
    ik_weight_ori: float = 1.0
    ik_seed_weight: float = 0.0
    ik_residual_limit: float = 0.03
    move_speed_percent: int = 30
    # Select the actual mechanism. PiPER-X must never use the ordinary Piper
    # model because its J4/J5/J6 joint origins and fixed rotations differ.
    kinematic_model: str = "piper"
    # Base frame transform (from verify_ik_fk.py --calibrate; identity by
    # default). T_sdk = X_base @ chain(q) @ X_ee.
    base_xyz_mm: tuple[float, float, float] = (0.0, 0.0, 0.0)
    base_rpy_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)
    # End-effector frame that matches the SDK pose. Verify with
    # piper/tools/verify_ik_fk.py.
    ee_xyz_mm: tuple[float, float, float] = (0.0, 0.0, 0.0)
    ee_rpy_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)
    # Physical TCP after the calibrated native J6 frame.  AgileX's official
    # PiPER-X gripper centre is 4.5 + 138 = 142.5 mm along local link6 Z.
    tool_xyz_mm: tuple[float, float, float] = (0.0, 0.0, 0.0)
    tool_rpy_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)

    send_gripper: bool = True
    gripper_effort: int = 1000
    gripper_ctrl_code: int = 0x03
    gripper_command_deadband: float = 0.005
    gripper_min_command_interval_s: float | None = None
    gripper_keepalive_s: float | None = None

    def __post_init__(self) -> None:
        super().__post_init__()
        self.id = "piper_joint_stream_follower" if self.id is None else self.id
        if not 1 <= self.move_speed_percent <= 100:
            raise ValueError("move_speed_percent must be in [1, 100]")
        if self.max_cartesian_step_mm <= 0 or self.max_rotation_step_rad <= 0:
            raise ValueError("Cartesian step limits must be positive")
        if self.max_joint_step_deg <= 0:
            raise ValueError("max_joint_step_deg must be positive")
        for name in (
            "control_frequency_hz",
            "max_joint_velocity_deg_s",
            "max_joint_acceleration_deg_s2",
            "max_joint_following_error_deg",
            "max_ik_solution_jump_deg",
        ):
            if not math.isfinite(float(getattr(self, name))) or getattr(self, name) <= 0:
                raise ValueError(f"{name} must be a finite positive value")
        if not 0 < self.ik_damping:
            raise ValueError("ik_damping must be positive")
        if not math.isfinite(float(self.ik_seed_weight)) or self.ik_seed_weight < 0:
            raise ValueError("ik_seed_weight must be finite and non-negative")
        if self.kinematic_model not in ("piper", "piper_x"):
            raise ValueError("kinematic_model must be 'piper' or 'piper_x'")
        if self.dry_run_log_interval_s <= 0:
            raise ValueError("dry_run_log_interval_s must be positive")
        for name in (
            "base_xyz_mm",
            "base_rpy_deg",
            "ee_xyz_mm",
            "ee_rpy_deg",
            "tool_xyz_mm",
            "tool_rpy_deg",
        ):
            values = getattr(self, name)
            if len(values) != 3 or not all(math.isfinite(float(v)) for v in values):
                raise ValueError(f"{name} must contain three finite values")
        for name in ("workspace_x", "workspace_y", "workspace_z"):
            bounds = getattr(self, name)
            if bounds is not None and bounds[0] >= bounds[1]:
                raise ValueError(f"{name} must be ordered as (min, max)")


@RobotConfig.register_subclass("uf::dual_piper_joint_stream")
@dataclass(kw_only=True)
class DualPiperJointStreamConfig(RobotConfig):
    """Exactly two independently modelled joint-stream followers."""

    robots: dict[str, RobotConfig] = field(default_factory=dict)
    parallel_connect: bool = True
    parallel_observation: bool = True
    parallel_action: bool = False

    def __post_init__(self) -> None:
        super().__post_init__()
        self.id = "dual_piper_joint_stream" if self.id is None else self.id
        if len(self.robots) != 2:
            raise ValueError("uf::dual_piper_joint_stream requires exactly two arms")
        ports = [getattr(robot, "port", None) for robot in self.robots.values()]
        if any(port is None for port in ports) or len(set(ports)) != 2:
            raise ValueError("dual joint-stream followers require distinct CAN ports")


class PiperJointStreamFollower(Robot):
    config_class = PiperJointStreamConfig
    name = "piper_joint_stream_follower"

    def __init__(self, config: PiperJointStreamConfig, prefix: str = "") -> None:
        super().__init__(config)
        self.config = config
        self.prefix = f"{prefix}." if prefix else ""
        self.bus = PiperMotorsBus(
            id=config.id or prefix or "piper",
            port=config.port,
            motors=MOTORS.copy(),
            calibration=CALIBRATION.copy(),
        )
        self.cameras: dict[str, Camera] = make_cameras_from_configs(config.cameras)
        self._camera_executor: ThreadPoolExecutor | None = None
        self._kin: PiperKinematics | None = None
        self._last_motion_ctrl: tuple[int, int, int, int] | None = None
        self._last_joint_cmd_rad: Any = None
        self._last_gripper_cmd: float | None = None
        self._last_gripper_time_s: float = 0.0
        self._last_ik_warning_s: float = 0.0
        self._last_guard_warning_s: float = 0.0
        self._last_dry_run_log_s: float = 0.0
        self._stabilizer: JointStreamStabilizer | None = None
        if self.cameras:
            self._camera_executor = ThreadPoolExecutor(
                max_workers=len(self.cameras),
                thread_name_prefix=f"{prefix or 'piper'}-camera",
            )

    def _key(self, local_key: str) -> str:
        return f"{self.prefix}{local_key}"

    @cached_property
    def observation_features(self) -> dict[str, type | tuple[int, int, int]]:
        state = {self._key(key): float for key in POSE_KEYS}
        if self.config.send_gripper:
            state[self._key("gripper.pos")] = float
        images = {
            self._key(name): (camera.height, camera.width, 3)
            for name, camera in self.cameras.items()
        }
        return {**state, **images}

    @cached_property
    def action_features(self) -> dict[str, type]:
        keys = POSE_KEYS
        if self.config.send_gripper:
            keys = POSE_KEYS + ("gripper.pos",)
        return {self._key(key): float for key in keys}

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected and all(
            camera.is_connected for camera in self.cameras.values()
        )

    @property
    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated

    def _kinematics(self) -> PiperKinematics:
        if self._kin is None:
            self._kin = PiperKinematics(
                model=self.config.kinematic_model,
                base_xyzrpy_m=tuple(v / 1000.0 for v in self.config.base_xyz_mm),
                base_rpy_rad=tuple(math.radians(v) for v in self.config.base_rpy_deg),
                ee_xyzrpy_m=tuple(v / 1000.0 for v in self.config.ee_xyz_mm),
                ee_rpy_rad=tuple(math.radians(v) for v in self.config.ee_rpy_deg),
                tool_xyzrpy_m=tuple(v / 1000.0 for v in self.config.tool_xyz_mm),
                tool_rpy_rad=tuple(math.radians(v) for v in self.config.tool_rpy_deg),
            )
        return self._kin

    def connect(self, calibrate: bool = True) -> None:
        self.bus.connect(piper_init=self.config.piper_init_on_connect)
        try:
            if self.config.configure_role_on_connect and not self.config.dry_run:
                self.bus.set_follower()
                time.sleep(0.1)
            if self.config.enable_torque_on_connect and not self.config.dry_run:
                self.bus.enable_torque()
            for camera in self.cameras.values():
                camera.connect()
            if self.cameras and self._camera_executor is None:
                self._camera_executor = ThreadPoolExecutor(
                    max_workers=len(self.cameras),
                    thread_name_prefix=f"{self.id or 'piper'}-camera",
                )
            time.sleep(0.2)
        except BaseException:
            for camera in self.cameras.values():
                if camera.is_connected:
                    camera.disconnect()
            self.bus.disconnect(
                disable_torque=self.config.disable_torque_on_disconnect,
                park=False,
            )
            raise
        logger.info(
            "Connected joint-stream follower %s on %s%s",
            self.id,
            self.config.port,
            " (read-only dry run)" if self.config.dry_run else "",
        )

    def calibrate(self) -> None:
        self.bus.clear_gripper()

    def configure(self) -> None:
        return None

    def setup_motors(self) -> None:
        self.bus.connect()
        self.bus.set_follower()

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")
        # Expose the configured physical TCP, not the native SDK/J6 origin.
        # That makes the Pika origin and the IK target use the same centre.
        x, y, z, roll, pitch, yaw = self._kinematics().forward_xyzrpy(
            self._current_joints_rad()
        )
        rx, ry, rz = rpy_degrees_to_axis_angle(roll, pitch, yaw)
        local = dict(zip(POSE_KEYS, (x, y, z, rx, ry, rz), strict=True))
        if self.config.send_gripper:
            local["gripper.pos"] = self.bus.get_joint_position()["gripper"] / 100.0
        observation = {self._key(key): value for key, value in local.items()}
        if self._camera_executor is not None:
            futures = {
                name: self._camera_executor.submit(camera.async_read)
                for name, camera in self.cameras.items()
            }
            for name, future in futures.items():
                observation[self._key(name)] = future.result()
        return observation

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")
        local = self._strip_and_validate(action)
        sent = self._send_cartesian_joint_stream(local)
        return {self._key(key): value for key, value in sent.items()}

    def _strip_and_validate(self, action: dict[str, Any]) -> dict[str, float]:
        local: dict[str, float] = {}
        for key, value in action.items():
            if self.prefix:
                if not key.startswith(self.prefix):
                    continue
                key = key[len(self.prefix) :]
            local[key] = float(value)
        required = set(POSE_KEYS)
        if self.config.send_gripper:
            required.add("gripper.pos")
        missing = required - set(local)
        if missing:
            raise KeyError(f"Action for {self.id} is missing keys: {sorted(missing)}")
        return {key: local[key] for key in required}

    def _current_joints_rad(self) -> np.ndarray:
        js = self.bus.piper.GetArmJointMsgs().joint_state
        raw = np.array(
            [js.joint_1, js.joint_2, js.joint_3, js.joint_4, js.joint_5, js.joint_6],
            dtype=float,
        )
        return np.radians(raw / 1000.0)

    def _send_cartesian_joint_stream(
        self, local: dict[str, float]
    ) -> dict[str, float]:
        if self.config.stabilized_stream:
            return self._send_cartesian_joint_stream_stabilized(local)

        q_current = self._current_joints_rad()
        x, y, z, roll, pitch, yaw = self._kinematics().forward_xyzrpy(q_current)
        current_rx, current_ry, current_rz = rpy_degrees_to_axis_angle(
            roll, pitch, yaw
        )
        current_xyz = np.array([x, y, z], dtype=float)
        current_rot = np.array([current_rx, current_ry, current_rz], dtype=float)
        target = np.array([local[key] for key in POSE_KEYS], dtype=float)

        bounded_xyz = np.array(
            [
                clamp(target[0], self.config.workspace_x),
                clamp(target[1], self.config.workspace_y),
                clamp(target[2], self.config.workspace_z),
            ],
            dtype=float,
        )
        in_deadband = (
            np.linalg.norm(bounded_xyz - current_xyz)
            <= self.config.translation_deadband_mm
            and np.linalg.norm(target[3:6] - current_rot)
            <= self.config.rotation_deadband_rad
        )
        sent = dict(zip(POSE_KEYS, target, strict=True))
        if self.config.send_gripper:
            sent["gripper.pos"] = local["gripper.pos"]
        if in_deadband:
            if self.config.send_gripper and not self.config.dry_run:
                self._send_gripper(local["gripper.pos"], sent)
            return sent

        limited_xyz = vector_step_towards(
            tuple(current_xyz), tuple(bounded_xyz), self.config.max_cartesian_step_mm
        )
        limited_rot = vector_step_towards(
            tuple(current_rot),
            tuple(target[3:6]),
            self.config.max_rotation_step_rad,
        )
        roll_deg, pitch_deg, yaw_deg = axis_angle_to_rpy_degrees(*limited_rot)

        kin = self._kinematics()
        q_target, residual = kin.ik(
            limited_xyz,
            (roll_deg, pitch_deg, yaw_deg),
            q_current,
            max_iter=self.config.ik_max_iter,
            damping=self.config.ik_damping,
            weight_ori=self.config.ik_weight_ori,
            seed_weight=self.config.ik_seed_weight,
        )
        if residual > self.config.ik_residual_limit:
            now_s = time.monotonic()
            if now_s - self._last_ik_warning_s >= 1.0:
                logger.warning(
                    "IK residual %.4f > %.4f; holding last joint target",
                    residual,
                    self.config.ik_residual_limit,
                )
                self._last_ik_warning_s = now_s
            q_target = (
                self._last_joint_cmd_rad
                if self._last_joint_cmd_rad is not None
                else q_current
            )
        else:
            self._last_joint_cmd_rad = q_target

        max_joint_step = math.radians(self.config.max_joint_step_deg)
        delta = np.clip(q_target - q_current, -max_joint_step, max_joint_step)
        q_send = q_current + delta
        if self.config.dry_run:
            now_s = time.monotonic()
            if now_s - self._last_dry_run_log_s >= self.config.dry_run_log_interval_s:
                delta_deg = np.degrees(q_target - q_current)
                logger.warning(
                    "IK PREVIEW %s residual=%.5f deg_delta=%s wrist(J4/J5/J6)=%s",
                    self.id,
                    residual,
                    np.round(delta_deg, 3).tolist(),
                    np.round(delta_deg[3:6], 3).tolist(),
                )
                self._last_dry_run_log_s = now_s
        else:
            millideg = tuple(int(round(v * _RAD_TO_MILLIDEG)) for v in q_send)
            self._send_joints(millideg)

        if self.config.send_gripper and not self.config.dry_run:
            self._send_gripper(local["gripper.pos"], sent)
        return sent

    def _get_stabilizer(self, q_feedback: np.ndarray, now_s: float) -> JointStreamStabilizer:
        if self._stabilizer is None:
            self._stabilizer = JointStreamStabilizer(
                frequency_hz=self.config.control_frequency_hz,
                max_velocity_deg_s=self.config.max_joint_velocity_deg_s,
                max_acceleration_deg_s2=self.config.max_joint_acceleration_deg_s2,
                max_step_deg=self.config.max_joint_step_deg,
                max_following_error_deg=self.config.max_joint_following_error_deg,
                max_ik_jump_deg=self.config.max_ik_solution_jump_deg,
            )
            self._stabilizer.reset(q_feedback, now_s)
        return self._stabilizer

    def _send_cartesian_joint_stream_stabilized(
        self, local: dict[str, float]
    ) -> dict[str, float]:
        """以连续命令为参考求 IK，并对关节速度、加速度和跟随误差实施保护。"""

        now_s = time.monotonic()
        q_feedback = self._current_joints_rad()
        stabilizer = self._get_stabilizer(q_feedback, now_s)
        assert stabilizer.command_rad is not None
        q_command = stabilizer.command_rad.copy()

        # 关键修复：从上一次已发送命令的预测位姿推进，不再从滞后的真实反馈重复起步。
        x, y, z, roll, pitch, yaw = self._kinematics().forward_xyzrpy(q_command)
        current_rx, current_ry, current_rz = rpy_degrees_to_axis_angle(
            roll, pitch, yaw
        )
        command_xyz = np.array([x, y, z], dtype=float)
        command_rot = np.array([current_rx, current_ry, current_rz], dtype=float)
        target = np.array([local[key] for key in POSE_KEYS], dtype=float)
        bounded_xyz = np.array(
            [
                clamp(target[0], self.config.workspace_x),
                clamp(target[1], self.config.workspace_y),
                clamp(target[2], self.config.workspace_z),
            ],
            dtype=float,
        )

        sent = dict(zip(POSE_KEYS, target, strict=True))
        if self.config.send_gripper:
            sent["gripper.pos"] = local["gripper.pos"]

        in_deadband = (
            np.linalg.norm(bounded_xyz - command_xyz)
            <= self.config.translation_deadband_mm
            and np.linalg.norm(target[3:6] - command_rot)
            <= self.config.rotation_deadband_rad
        )
        if in_deadband:
            stabilizer.stop(now_s)
            if self.config.send_gripper and not self.config.dry_run:
                self._send_gripper(local["gripper.pos"], sent)
            return sent

        limited_xyz = vector_step_towards(
            tuple(command_xyz),
            tuple(bounded_xyz),
            self.config.max_cartesian_step_mm,
        )
        limited_rot = vector_step_towards(
            tuple(command_rot),
            tuple(target[3:6]),
            self.config.max_rotation_step_rad,
        )
        roll_deg, pitch_deg, yaw_deg = axis_angle_to_rpy_degrees(*limited_rot)

        kin = self._kinematics()
        q_target, residual = kin.ik(
            limited_xyz,
            (roll_deg, pitch_deg, yaw_deg),
            q_command,
            max_iter=self.config.ik_max_iter,
            damping=self.config.ik_damping,
            weight_ori=self.config.ik_weight_ori,
            seed_weight=self.config.ik_seed_weight,
        )
        if residual > self.config.ik_residual_limit:
            if now_s - self._last_ik_warning_s >= 1.0:
                logger.warning(
                    "STABLE IK residual %.4f > %.4f; holding command",
                    residual,
                    self.config.ik_residual_limit,
                )
                self._last_ik_warning_s = now_s
            stabilizer.stop(now_s)
            q_target = q_command

        result = stabilizer.advance(
            q_target,
            q_feedback,
            now_s=now_s,
            dry_run=self.config.dry_run,
        )
        if result.reason != "ok" and now_s - self._last_guard_warning_s >= 1.0:
            logger.warning(
                "STABLE STREAM HOLD %s reason=%s following=%.3fdeg ik_jump=%.3fdeg",
                self.id,
                result.reason,
                result.following_error_deg,
                result.ik_jump_deg,
            )
            self._last_guard_warning_s = now_s

        q_send = result.command_rad
        self._last_joint_cmd_rad = q_send.copy()
        if self.config.dry_run:
            if now_s - self._last_dry_run_log_s >= self.config.dry_run_log_interval_s:
                logger.warning(
                    "STABLE IK PREVIEW %s residual=%.5f reason=%s "
                    "follow=%.3fdeg ik_jump=%.3fdeg step=%s wrist_step=%s",
                    self.id,
                    residual,
                    result.reason,
                    result.following_error_deg,
                    result.ik_jump_deg,
                    np.round(np.degrees(result.step_rad), 4).tolist(),
                    np.round(np.degrees(result.step_rad[3:6]), 4).tolist(),
                )
                self._last_dry_run_log_s = now_s
        else:
            millideg = tuple(int(round(v * _RAD_TO_MILLIDEG)) for v in q_send)
            self._send_joints(millideg)

        if self.config.send_gripper and not self.config.dry_run:
            self._send_gripper(local["gripper.pos"], sent)
        return sent

    def _send_joints(self, millideg: tuple[int, ...]) -> None:
        # Normal position/velocity mode. MIT mode (0xAD) is deliberately not
        # used: it is an advanced torque-oriented mode and is unnecessary for
        # absolute JointCtrl targets.
        cmd = (0x01, 0x01, self.config.move_speed_percent, 0x00)
        if self._last_motion_ctrl != cmd:
            self.bus.piper.MotionCtrl_2(*cmd)
            self._last_motion_ctrl = cmd
        self.bus.piper.JointCtrl(*millideg)

    def _send_gripper(self, gripper_unit: float, sent: dict[str, float]) -> None:
        gripper_unit = min(1.0, max(0.0, gripper_unit))
        interval_s = (
            self.config.gripper_min_command_interval_s
            if self.config.gripper_min_command_interval_s is not None
            else 0.03
        )
        keepalive_s = (
            self.config.gripper_keepalive_s
            if self.config.gripper_keepalive_s is not None
            else max(interval_s, 0.001)
        )
        now_s = time.monotonic()
        due = now_s - self._last_gripper_time_s >= interval_s
        changed = (
            self._last_gripper_cmd is None
            or abs(gripper_unit - self._last_gripper_cmd)
            >= self.config.gripper_command_deadband
        )
        keepalive_due = now_s - self._last_gripper_time_s >= keepalive_s
        if due and (changed or keepalive_due):
            self.bus.set_gripper_percent(
                gripper_unit * 100.0,
                effort=self.config.gripper_effort,
                ctrl_code=self.config.gripper_ctrl_code,
            )
            self._last_gripper_cmd = gripper_unit
            self._last_gripper_time_s = now_s
        sent["gripper.pos"] = gripper_unit

    def parking(self) -> None:
        self.bus.parking()

    def disconnect(self) -> None:
        for camera in self.cameras.values():
            if camera.is_connected:
                camera.disconnect()
        if self._camera_executor is not None:
            self._camera_executor.shutdown(wait=True)
            self._camera_executor = None
        self.bus.disconnect(
            disable_torque=self.config.disable_torque_on_disconnect,
            park=self.config.park_on_disconnect,
        )
        self._last_joint_cmd_rad = None
        self._stabilizer = None


class DualPiperJointStreamFollower(Robot):
    """Dual-arm wrapper that keeps each PiPER-X IK seed and CAN bus separate."""

    config_class = DualPiperJointStreamConfig
    name = "dual_piper_joint_stream_follower"

    def __init__(self, config: DualPiperJointStreamConfig) -> None:
        super().__init__(config)
        self.config = config
        self.robots: dict[str, PiperJointStreamFollower] = {}
        for side, robot_config in config.robots.items():
            if not isinstance(robot_config, PiperJointStreamConfig):
                raise TypeError(
                    f"{side} must use type uf::piper_joint_stream, "
                    f"got {robot_config.type}"
                )
            self.robots[side] = PiperJointStreamFollower(robot_config, prefix=side)
        self.cameras = {
            f"{side}.{name}": camera
            for side, robot in self.robots.items()
            for name, camera in robot.cameras.items()
        }
        self._executor = ThreadPoolExecutor(
            max_workers=2, thread_name_prefix="dual-piper-joint-stream"
        )

    @property
    def observation_features(self) -> dict:
        return self._merge(lambda robot: robot.observation_features)

    @property
    def action_features(self) -> dict:
        return self._merge(lambda robot: robot.action_features)

    @property
    def is_connected(self) -> bool:
        return all(robot.is_connected for robot in self.robots.values())

    @property
    def is_calibrated(self) -> bool:
        return all(robot.is_calibrated for robot in self.robots.values())

    def _merge(self, getter: Callable[[PiperJointStreamFollower], dict]) -> dict:
        merged: dict = {}
        for robot in self.robots.values():
            merged.update(getter(robot))
        return merged

    def _parallel(self, fn: Callable[[PiperJointStreamFollower], T]) -> list[T]:
        futures: list[Future[T]] = [
            self._executor.submit(fn, robot) for robot in self.robots.values()
        ]
        return [future.result() for future in futures]

    def connect(self, calibrate: bool = True) -> None:
        try:
            if self.config.parallel_connect:
                self._parallel(lambda robot: robot.connect(calibrate=calibrate))
            else:
                for robot in self.robots.values():
                    robot.connect(calibrate=calibrate)
        except BaseException:
            for robot in self.robots.values():
                if robot.bus.is_connected:
                    try:
                        robot.disconnect()
                    except Exception:
                        logger.exception("Failed to clean up %s", robot.id)
            raise

    def calibrate(self) -> None:
        for robot in self.robots.values():
            robot.calibrate()

    def configure(self) -> None:
        return None

    def get_observation(self) -> dict[str, Any]:
        observations = (
            self._parallel(lambda robot: robot.get_observation())
            if self.config.parallel_observation
            else [robot.get_observation() for robot in self.robots.values()]
        )
        merged: dict[str, Any] = {}
        for observation in observations:
            merged.update(observation)
        return merged

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        def send(robot: PiperJointStreamFollower) -> dict[str, Any]:
            return robot.send_action(action)

        sent_actions = (
            self._parallel(send)
            if self.config.parallel_action
            else [send(robot) for robot in self.robots.values()]
        )
        merged: dict[str, Any] = {}
        for sent in sent_actions:
            merged.update(sent)
        return merged

    def disconnect(self) -> None:
        try:
            self._parallel(lambda robot: robot.disconnect())
        finally:
            self._executor.shutdown(wait=True)


# LeRobot discovers a device class by removing ``Config`` from the config
# class name.  Keep the descriptive ``*Follower`` names as backwards-
# compatible aliases, while exporting the exact names expected by its device
# factory (PiperJointStreamConfig -> PiperJointStream, etc.).
PiperJointStream = PiperJointStreamFollower
DualPiperJointStream = DualPiperJointStreamFollower
