"""关节流命令的连续性与动态保护。

本模块只处理关节数组，不依赖 LeRobot 或 Piper SDK，方便离线验证。它把 IK 解转换成
速度、加速度和单周期位移均连续的关节命令，并在真实关节反馈落后或 IK 解发生跳变时保持
上一条命令。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class StabilizerResult:
    command_rad: np.ndarray
    step_rad: np.ndarray
    reason: str
    dt_s: float
    following_error_deg: float
    ik_jump_deg: float


class JointStreamStabilizer:
    """将离散 IK 解变成连续的关节位置命令。"""

    def __init__(
        self,
        *,
        frequency_hz: float,
        max_velocity_deg_s: float,
        max_acceleration_deg_s2: float,
        max_step_deg: float,
        max_following_error_deg: float,
        max_ik_jump_deg: float,
    ) -> None:
        values = {
            "frequency_hz": frequency_hz,
            "max_velocity_deg_s": max_velocity_deg_s,
            "max_acceleration_deg_s2": max_acceleration_deg_s2,
            "max_step_deg": max_step_deg,
            "max_following_error_deg": max_following_error_deg,
            "max_ik_jump_deg": max_ik_jump_deg,
        }
        if not all(np.isfinite(float(value)) and float(value) > 0 for value in values.values()):
            raise ValueError(f"稳定器参数必须为有限正数: {values}")

        self.nominal_dt_s = 1.0 / float(frequency_hz)
        self.max_velocity_rad_s = np.deg2rad(float(max_velocity_deg_s))
        self.max_acceleration_rad_s2 = np.deg2rad(float(max_acceleration_deg_s2))
        self.max_step_rad = np.deg2rad(float(max_step_deg))
        self.max_following_error_deg = float(max_following_error_deg)
        self.max_ik_jump_deg = float(max_ik_jump_deg)

        self.command_rad: np.ndarray | None = None
        self.velocity_rad_s: np.ndarray | None = None
        self.last_time_s: float | None = None

    def reset(self, q_rad: np.ndarray, now_s: float | None = None) -> None:
        q = np.asarray(q_rad, dtype=float).reshape(-1)[:6]
        if q.shape != (6,) or not np.all(np.isfinite(q)):
            raise ValueError("稳定器初始关节角必须是 6 个有限值")
        self.command_rad = q.copy()
        self.velocity_rad_s = np.zeros(6, dtype=float)
        self.last_time_s = now_s

    def stop(self, now_s: float | None = None) -> None:
        if self.velocity_rad_s is not None:
            self.velocity_rad_s.fill(0.0)
        if now_s is not None:
            self.last_time_s = float(now_s)

    def advance(
        self,
        q_ik_rad: np.ndarray,
        q_feedback_rad: np.ndarray,
        *,
        now_s: float,
        dry_run: bool,
    ) -> StabilizerResult:
        q_ik = np.asarray(q_ik_rad, dtype=float).reshape(-1)[:6]
        q_feedback = np.asarray(q_feedback_rad, dtype=float).reshape(-1)[:6]
        if q_ik.shape != (6,) or q_feedback.shape != (6,):
            raise ValueError("IK 与反馈关节角必须各包含 6 个值")
        if not np.all(np.isfinite(q_ik)) or not np.all(np.isfinite(q_feedback)):
            raise ValueError("IK 与反馈关节角必须为有限值")

        if self.command_rad is None or self.velocity_rad_s is None:
            self.reset(q_feedback, now_s)

        assert self.command_rad is not None
        assert self.velocity_rad_s is not None

        if self.last_time_s is None:
            dt_s = self.nominal_dt_s
        else:
            # 防止调试暂停或偶发调度延迟产生一次超大积分步长。
            dt_s = float(
                np.clip(
                    now_s - self.last_time_s,
                    0.25 * self.nominal_dt_s,
                    2.0 * self.nominal_dt_s,
                )
            )
        self.last_time_s = float(now_s)

        following_error_deg = float(
            np.max(np.abs(np.rad2deg(self.command_rad - q_feedback)))
        )
        ik_jump_deg = float(np.max(np.abs(np.rad2deg(q_ik - self.command_rad))))

        reason = "ok"
        if not dry_run and following_error_deg > self.max_following_error_deg:
            reason = "following_error"
        elif ik_jump_deg > self.max_ik_jump_deg:
            reason = "ik_jump"

        if reason != "ok":
            self.velocity_rad_s.fill(0.0)
            return StabilizerResult(
                command_rad=self.command_rad.copy(),
                step_rad=np.zeros(6, dtype=float),
                reason=reason,
                dt_s=dt_s,
                following_error_deg=following_error_deg,
                ik_jump_deg=ik_jump_deg,
            )

        remaining = q_ik - self.command_rad
        desired_velocity = np.clip(
            remaining / dt_s,
            -self.max_velocity_rad_s,
            self.max_velocity_rad_s,
        )
        max_velocity_change = self.max_acceleration_rad_s2 * dt_s
        self.velocity_rad_s += np.clip(
            desired_velocity - self.velocity_rad_s,
            -max_velocity_change,
            max_velocity_change,
        )

        step = np.clip(
            self.velocity_rad_s * dt_s,
            -self.max_step_rad,
            self.max_step_rad,
        )
        # 目标很近时直接落到目标，避免跨过目标后在两侧来回修正。
        step = np.where(np.abs(step) > np.abs(remaining), remaining, step)
        self.command_rad = self.command_rad + step
        self.velocity_rad_s = step / dt_s

        return StabilizerResult(
            command_rad=self.command_rad.copy(),
            step_rad=step.copy(),
            reason=reason,
            dt_s=dt_s,
            following_error_deg=following_error_deg,
            ik_jump_deg=ik_jump_deg,
        )
