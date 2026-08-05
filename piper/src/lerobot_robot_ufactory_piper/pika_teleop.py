import time
from collections import deque

import numpy as np
from concurrent.futures import ThreadPoolExecutor
from typing import Any

from lerobot_robot_ufactory.teleoperators.base_teleop import UFBaseTeleop
from lerobot_robot_ufactory.devices.umi.vive_tracker.transformations import (
    Transformations,
)
from lerobot_robot_ufactory.teleoperators.pika_teleop import (
    PikaTeleop as UFactoryPikaTeleop,
)
from lerobot_robot_ufactory.teleoperators.pika_teleop import PikaTeleopConfig

from .config import DualPikaTeleopConfig, PiperPikaTeleopConfig


# Calibrated on 2026-08-03 using Pika right/forward/up translation tests.
# Piper base convention: +X forward, -Y right, +Z up.
_ORIGINAL_PIKA_GET_ACTION = UFactoryPikaTeleop.get_action
_PIKA_TO_PIPER_TRANSLATION = np.array(
    [
        [0.52718794, -0.81988978, -0.22327926],
        [0.59104387,  0.16501522,  0.78958035],
        [-0.61052438, -0.54822508, 0.57158486],
    ],
    dtype=float,
)

# Calibrated from three Pika-only gestures on 2026-08-03.
# Measured 2026-08-05 with piper/tools/measure_pika_piper_mapping.py. Maps raw
# tracker deltas in the lighthouse/world frame directly to the Piper base
# frame: Pika forward -> Piper +X, Pika right -> Piper -Y, Pika up -> +Z.
# Kabsch fit error was ~1 degree. This replaces the old fitted matrix, which
# mapped forward to (0.52, -0.74, -0.43) instead of +X.
_RAW_TO_PIPER_TRANSLATION = np.array(
    [
        [0.908157378, -0.375801932, -0.184453476],
        [0.381913658, 0.924194400, -0.002582414],
        [0.171441346, -0.068100063, 0.982837854],
    ],
    dtype=float,
)
# The rows map the
# original local axis-angle vector to the Piper gesture convention:
#   Pika tip right -> -RX, Pika tip up -> +RY, clockwise roll -> +RZ.
# The matrix is orthonormal. Experimental profiles can additionally keep only
# the dominant mapped axis to reject natural hand-motion cross coupling.
_PIKA_TO_PIPER_ROTATION = np.array(
    [
        [0.99266317, 0.11349864, -0.04168796],
        [-0.11927281, 0.97574876, -0.18354388],
        [0.01984499, 0.18716949, 0.98212716],
    ],
    dtype=float,
)

# Piper MOVE P rotation components are expressed in the end-effector command
# frame.  Around the verified 2026-08-03 test pose, that frame is rotated by
# 90 degrees relative to the operator's visual up/right convention.  Keep this
# compensation opt-in so the V2 and first-version profiles remain unchanged.
_PIPER_TOOL_AXIS_CORRECTION = np.array(
    [
        [0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ],
    dtype=float,
)


class _PikaTeleop(UFactoryPikaTeleop):
    """Local bug-fix subclass; the parent Pika implementation is left untouched."""

    def get_action(self) -> dict[str, Any]:
        # The parent returns its mutable internal cache.  Apply the Piper axis
        # mapping to a copy so a temporary tracker dropout cannot transform the
        # already-transformed cache again on the next control cycle.
        action = dict(_ORIGINAL_PIKA_GET_ACTION(self))

        if not self._teleop_enabled:
            return action

        keys = (
            f"{self.prefix}pose.x",
            f"{self.prefix}pose.y",
            f"{self.prefix}pose.z",
        )
        origin = np.asarray(self._last_robot_pose[:3], dtype=float)
        raw_target = np.asarray([action[key] for key in keys], dtype=float)
        corrected_target = self._translation_target(raw_target, origin)

        for key, value in zip(keys, corrected_target, strict=True):
            action[key] = float(value)

        return action

    def _translation_target(
        self, raw_target: np.ndarray, origin: np.ndarray
    ) -> np.ndarray:
        if not getattr(self.config, "use_raw_translation_mapping", False):
            return origin + _PIKA_TO_PIPER_TRANSLATION @ (raw_target - origin)
        pose = self.pika_sense.get_pose(self.pika_device.pika_tracker_device)
        if pose is None:
            return origin + _PIKA_TO_PIPER_TRANSLATION @ (raw_target - origin)
        raw_m = np.asarray(pose.position, dtype=float)
        if not hasattr(self, "_raw_start_xyz") or self._raw_start_xyz is None:
            self._raw_start_xyz = raw_m.copy()
        delta_mm = (raw_m - self._raw_start_xyz) * 1000.0 * self.config.scale_xyz
        return origin + _RAW_TO_PIPER_TRANSLATION @ delta_mm

    def run(self) -> None:
        self._is_connected = True
        initial_state = self.pika_sense.get_command_state()
        current_state = initial_state
        sleep_time = 1.0 / self.config.frequency
        while not self.stop_event.is_set():
            time.sleep(sleep_time)
            state = self.pika_sense.get_command_state()
            if state == current_state:
                continue
            current_state = state
            if not self._teleop_enabled and current_state != initial_state:
                self.set_teleop_enabled(True, self._last_action)
                time.sleep(1.0)
            elif self._teleop_enabled and current_state == initial_state:
                if self._last_action is not None:
                    self.set_teleop_enabled(False)
                else:
                    with self._data_lock:
                        self._teleop_enabled = False
                        self.begin_tracker_robot_matrix = None


class PiperPikaTeleop(_PikaTeleop):
    """Single-Pika profile with stable, rate-limited Piper gripper input."""

    config_class = PiperPikaTeleopConfig
    name = "piper_pika_teleop"

    def __init__(self, config: PiperPikaTeleopConfig, prefix: str = "") -> None:
        super().__init__(config, prefix=prefix)
        self._gripper_samples: deque[float] = deque(
            maxlen=config.gripper_filter_window
        )
        self._filtered_gripper: float | None = None
        self._last_direct_gripper: float | None = None
        self._filtered_rotation_delta: np.ndarray | None = None

    def set_teleop_enabled(self, enabled: bool, obs: dict | None = None) -> None:
        super().set_teleop_enabled(enabled, obs)
        self._gripper_samples.clear()
        self._last_direct_gripper = None
        self._filtered_rotation_delta = None
        # Begin at the actual Piper gripper position supplied in ``obs`` and
        # approach the Pika value gradually. This avoids a jump on Enter.
        self._filtered_gripper = (
            float(self._last_gripper_pos) if self.config.use_gripper else None
        )

    def get_action(self) -> dict[str, Any]:
        action = super().get_action()

        if (
            self._teleop_enabled
            and self.config.use_calibrated_rotation_mapping
        ):
            rotation_keys = (
                f"{self.prefix}pose.rx",
                f"{self.prefix}pose.ry",
                f"{self.prefix}pose.rz",
            )
            origin_rotation = Transformations.rxryrz_to_rotation_matrix(
                *self._last_robot_pose[3:6]
            )
            target_rotation = Transformations.rxryrz_to_rotation_matrix(
                *(float(action[key]) for key in rotation_keys)
            )
            relative_rotation = origin_rotation.T @ target_rotation
            relative_vector = Transformations.rotation_matrix_to_rxryrz(
                relative_rotation
            )
            mapped_vector = (
                _PIKA_TO_PIPER_ROTATION @ relative_vector
            )
            if self.config.apply_piper_tool_axis_correction:
                mapped_vector = _PIPER_TOOL_AXIS_CORRECTION @ mapped_vector
            if self.config.rotation_dominant_axis:
                dominant_index = int(np.argmax(np.abs(mapped_vector)))
                dominant_vector = np.zeros(3, dtype=float)
                dominant_vector[dominant_index] = mapped_vector[dominant_index]
                mapped_vector = dominant_vector
            if self.config.rotation_filter_alpha < 1.0:
                if self._filtered_rotation_delta is None:
                    self._filtered_rotation_delta = mapped_vector.copy()
                else:
                    alpha = self.config.rotation_filter_alpha
                    self._filtered_rotation_delta = (
                        alpha * mapped_vector
                        + (1.0 - alpha) * self._filtered_rotation_delta
                    )
                mapped_vector = self._filtered_rotation_delta
            mapped_vector *= self.config.rotation_scale
            corrected_rotation = (
                origin_rotation
                @ Transformations.rxryrz_to_rotation_matrix(*mapped_vector)
            )
            corrected_vector = Transformations.rotation_matrix_to_rxryrz(
                corrected_rotation
            )
            for key, value in zip(
                rotation_keys, corrected_vector, strict=True
            ):
                action[key] = float(value)

        if not self.config.use_gripper or not self._teleop_enabled:
            return action

        key = f"{self.prefix}gripper.pos"
        if self.config.gripper_use_direct_distance:
            # Pika reports physical jaw opening: approximately 0 mm closed and
            # 100 mm open. Piper's normalized gripper command follows the same
            # direction (0 closed, 1 open). The base UFACTORY implementation
            # inverts this value for xArm, so read the cached SDK distance here
            # and keep the Piper mapping explicit.
            distance_mm = self.pika_sense.get_gripper_distance()
            if distance_mm is not None:
                span_mm = (
                    self.config.gripper_distance_max_mm
                    - self.config.gripper_distance_min_mm
                )
                raw = (
                    float(distance_mm) - self.config.gripper_distance_min_mm
                ) / span_mm
                raw = min(1.0, max(0.0, raw))
                self._last_direct_gripper = raw
            elif self._last_direct_gripper is not None:
                raw = self._last_direct_gripper
            else:
                # On a first-frame serial dropout, hold the actual Piper
                # opening captured in set_teleop_enabled instead of jumping.
                raw = min(1.0, max(0.0, float(self._last_gripper_pos)))
        else:
            raw = min(1.0, max(0.0, float(action[key])))
        self._gripper_samples.append(raw)
        target = float(np.median(np.asarray(self._gripper_samples, dtype=float)))

        if self._filtered_gripper is None:
            self._filtered_gripper = target
        error = target - self._filtered_gripper
        if abs(error) > self.config.gripper_deadband:
            delta = self.config.gripper_filter_alpha * error
            delta = min(
                self.config.gripper_max_step,
                max(-self.config.gripper_max_step, delta),
            )
            self._filtered_gripper = min(
                1.0, max(0.0, self._filtered_gripper + delta)
            )

        action[key] = float(self._filtered_gripper)
        return action


# Apply the local fixed state-monitor loop to single-Pika teleoperation.
UFactoryPikaTeleop.run = _PikaTeleop.run
UFactoryPikaTeleop.get_action = _PikaTeleop.get_action

class DualPikaTeleop(UFBaseTeleop):
    config_class = DualPikaTeleopConfig
    name = "dual_pika_teleop"

    def __init__(self, config: DualPikaTeleopConfig) -> None:
        super().__init__(config)
        self.config = config
        self.teleops: dict[str, _PikaTeleop] = {}
        for side, teleop_config in config.teleops.items():
            if not isinstance(teleop_config, PikaTeleopConfig):
                raise TypeError(f"{side} must use type uf::pika_teleop, got {teleop_config.type}")
            self.teleops[side] = _PikaTeleop(teleop_config, prefix=side)
        self._executor = ThreadPoolExecutor(max_workers=2, thread_name_prefix="dual-pika")

    @property
    def action_features(self) -> dict[str, type]:
        result: dict[str, type] = {}
        for side, teleop in self.teleops.items():
            names = teleop.action_features["names"]
            result.update({f"{side}.{name}": float for name in names})
        return result

    @property
    def feedback_features(self) -> dict[str, type]:
        return self.action_features

    @property
    def is_connected(self) -> bool:
        return all(teleop.is_connected for teleop in self.teleops.values())

    @property
    def is_calibrated(self) -> bool:
        return all(teleop.is_calibrated for teleop in self.teleops.values())

    def connect(self, calibrate: bool = True) -> None:
        super().connect(calibrate)
        try:
            for teleop in self.teleops.values():
                teleop.connect(calibrate=calibrate)
        except BaseException:
            for teleop in self.teleops.values():
                if teleop.is_connected:
                    try:
                        teleop.disconnect()
                    except Exception:
                        pass
            super().disconnect()
            raise

    def calibrate(self) -> None:
        for teleop in self.teleops.values():
            teleop.calibrate()

    def configure(self) -> None:
        for teleop in self.teleops.values():
            teleop.configure()

    def set_teleop_enabled(self, enabled: bool, obs: dict | None = None) -> None:
        for teleop in self.teleops.values():
            teleop.set_teleop_enabled(enabled, obs)

    def get_action(self) -> dict[str, Any]:
        if self.config.parallel_read:
            futures = [self._executor.submit(teleop.get_action) for teleop in self.teleops.values()]
            actions = [future.result() for future in futures]
        else:
            actions = [teleop.get_action() for teleop in self.teleops.values()]
        merged: dict[str, Any] = {}
        for action in actions:
            merged.update(action)
        return merged

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        return None

    def disconnect(self) -> None:
        try:
            for teleop in self.teleops.values():
                teleop.disconnect()
        finally:
            self._executor.shutdown(wait=True)
            super().disconnect()
