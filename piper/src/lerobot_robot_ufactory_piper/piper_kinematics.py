"""Pure-pinocchio FK/IK for the Piper arm.

Ported from the official PikaAnyArm stack
(piper/pika_remote_piper/scripts/forward_inverse_kinematics.py and
piper/piper_ros/piper_description/urdf/piper_description.urdf) but without
ROS, CasADi or Meshcat dependencies. The IK is a damped least-squares solver
seeded with the current joint values, so successive solutions stay
continuous (the same idea as the official stack's "seed from previous
solution").

Only extra dependency: ``pinocchio``. Install on the robot PC:

    pip install pinocchio
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Iterable

import numpy as np

_URDF = Path(__file__).resolve().parents[2] / "urdf" / "piper_description.urdf"


def _rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cp * cy, -cr * sy + sr * sp * cy, sr * sy + cr * sp * cy],
            [cp * sy, cr * cy + sr * sp * sy, -sr * cy + cr * sp * sy],
            [-sp, sr * cp, cr * cp],
        ]
    )


def xyzrpy_to_matrix(
    xyz_m: Iterable[float], rpy_rad: Iterable[float]
) -> np.ndarray:
    xyz = np.asarray(list(xyz_m), dtype=float)
    rpy = np.asarray(list(rpy_rad), dtype=float)
    T = np.eye(4)
    T[:3, :3] = _rpy_to_matrix(*rpy)
    T[:3, 3] = xyz
    return T


def matrix_to_xyzrpy(T: np.ndarray) -> tuple[float, float, float, float, float, float]:
    R = T[:3, :3]
    roll = math.atan2(R[2, 1], R[2, 2])
    pitch = math.asin(-R[2, 0])
    yaw = math.atan2(R[1, 0], R[0, 0])
    return (T[0, 3], T[1, 3], T[2, 3], roll, pitch, yaw)


class PiperKinematics:
    """FK/IK for the 6-DOF Piper arm with a configurable end-effector frame.

    ``ee_xyzrpy_m``/``ee_rpy_rad`` define the transform from the URDF
    joint6 frame to the end-effector frame that matches the Piper SDK pose.
    The official PikaAnyArm convention is +190 mm along X and a -90 deg
    pitch; run piper/tools/verify_ik_fk.py to confirm which candidate
    matches your SDK.
    """

    def __init__(
        self,
        urdf_path: str | Path | None = None,
        ee_xyzrpy_m: Iterable[float] = (0.19, 0.0, 0.0),
        ee_rpy_rad: Iterable[float] = (0.0, -math.pi / 2.0, 0.0),
    ) -> None:
        import pinocchio as pin  # lazy: only needed when this module is used

        self._pin = pin
        path = Path(urdf_path) if urdf_path is not None else _URDF
        if not path.exists():
            raise FileNotFoundError(f"Piper URDF not found: {path}")

        self.model = self._build_model(pin, path)
        self.ee_tf = xyzrpy_to_matrix(
            np.asarray(list(ee_xyzrpy_m), dtype=float),
            np.asarray(list(ee_rpy_rad), dtype=float),
        )
        quat = pin.Quaternion(self.ee_tf[:3, :3])
        self.model.addFrame(
            pin.Frame(
                "ee",
                self.model.getJointId("joint6"),
                pin.SE3(quat, self.ee_tf[:3, 3]),
                pin.FrameType.OP_FRAME,
            )
        )
        # Create data AFTER adding the frame so data.oMf includes "ee".
        self.data = self.model.createData()
        self.frame_id = self.model.getFrameId("ee")
        self.lower = np.asarray(self.model.lowerPositionLimit[:6], dtype=float)
        self.upper = np.asarray(self.model.upperPositionLimit[:6], dtype=float)

    @staticmethod
    def _build_model(pin, path: Path):
        """Load the kinematic model across pinocchio versions."""
        last_error: Exception | None = None

        # pinocchio 2.x: buildModelFromUrdf(filename)
        fn = getattr(pin, "buildModelFromUrdf", None)
        if fn is not None:
            try:
                return fn(str(path))
            except Exception as exc:  # pragma: no cover - version dependent
                last_error = exc

        # pinocchio 3.x: buildModelFromXML(xml_string)
        fn = getattr(pin, "buildModelFromXML", None)
        if fn is not None:
            try:
                return fn(path.read_text(encoding="utf-8"))
            except Exception as exc:  # pragma: no cover - version dependent
                last_error = exc

        # legacy RobotWrapper path
        try:
            return pin.RobotWrapper.BuildFromURDF(str(path)).model
        except Exception as exc:  # pragma: no cover - version dependent
            last_error = exc

        raise RuntimeError(
            "无法用当前 pinocchio 加载 URDF；请升级/重装 pinocchio "
            f"(最后错误: {last_error})"
        ) from last_error

    def _q_full(self, q_rad: np.ndarray) -> np.ndarray:
        q = np.zeros(self.model.nq, dtype=float)
        q[:6] = np.asarray(q_rad, dtype=float).reshape(-1)[:6]
        return q

    def forward(self, q_rad: np.ndarray) -> np.ndarray:
        """FK: returns the 4x4 end-effector transform (meters)."""
        pin = self._pin
        pin.forwardKinematics(self.model, self.data, self._q_full(q_rad))
        pin.updateFramePlacements(self.model, self.data)
        return np.asarray(self.data.oMf[self.frame_id])

    def forward_xyzrpy(
        self, q_rad: np.ndarray
    ) -> tuple[float, float, float, float, float, float]:
        """FK in the same units as the SDK: mm + degrees."""
        x, y, z, roll, pitch, yaw = matrix_to_xyzrpy(self.forward(q_rad))
        return (
            x * 1000.0,
            y * 1000.0,
            z * 1000.0,
            math.degrees(roll),
            math.degrees(pitch),
            math.degrees(yaw),
        )

    def ik(
        self,
        target_xyz_mm: Iterable[float],
        target_rpy_deg: Iterable[float],
        q_seed_rad: Iterable[float],
        max_iter: int = 60,
        tol: float = 1e-4,
        damping: float = 1e-3,
        weight_ori: float = 0.1,
    ) -> tuple[np.ndarray, float]:
        """Damped least-squares IK. Returns (q_rad, residual)."""
        pin = self._pin
        target = xyzrpy_to_matrix(
            np.asarray(list(target_xyz_mm), dtype=float) / 1000.0,
            np.radians(np.asarray(list(target_rpy_deg), dtype=float)),
        )
        q = np.clip(
            np.asarray(q_seed_rad, dtype=float).reshape(-1)[:6],
            self.lower,
            self.upper,
        )
        W = np.diag([1.0, 1.0, 1.0, weight_ori, weight_ori, weight_ori])
        residual = float("inf")
        for _ in range(max_iter):
            pin.forwardKinematics(self.model, self.data, self._q_full(q))
            pin.updateFramePlacements(self.model, self.data)
            M = np.asarray(self.data.oMf[self.frame_id])
            err = np.asarray(
                pin.log6(pin.SE3(M).inverse() * pin.SE3(target)).vector
            )
            residual = float(np.linalg.norm(err))
            if residual < tol:
                break
            J = np.asarray(
                pin.computeFrameJacobian(
                    self.model,
                    self.data,
                    self._q_full(q),
                    self.frame_id,
                    pin.LOCAL_WORLD_ALIGNED,
                )
            )[:, :6]
            Jw = W @ J
            dq = Jw.T @ np.linalg.solve(
                Jw @ Jw.T + damping * np.eye(6), err
            )
            q = np.clip(q + dq, self.lower, self.upper)
        return q, residual
