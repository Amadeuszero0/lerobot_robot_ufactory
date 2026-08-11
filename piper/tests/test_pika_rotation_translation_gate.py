from types import SimpleNamespace

import numpy as np

from lerobot_robot_ufactory_piper.pika_teleop import _PikaTeleop


def _quaternion_z(degrees: float) -> np.ndarray:
    half = np.radians(degrees) / 2.0
    return np.array([0.0, 0.0, np.sin(half), np.cos(half)], dtype=float)


def _gate() -> _PikaTeleop:
    gate = object.__new__(_PikaTeleop)
    gate.id = "test_pika"
    gate.config = SimpleNamespace(
        freeze_translation_while_rotating=True,
        translation_rotation_lock_speed_rad_s=0.15,
        translation_rotation_release_speed_rad_s=0.05,
        translation_rotation_release_delay_s=0.15,
        translation_rotation_speed_window_s=0.08,
    )
    gate._last_translation_gate_quaternion = None
    gate._last_translation_gate_time_s = None
    gate._translation_rotation_locked = False
    gate._translation_rotation_quiet_since_s = None
    return gate


def test_gate_freezes_fast_rotation_and_releases_after_quiet_delay() -> None:
    gate = _gate()

    assert gate._update_translation_rotation_gate(_quaternion_z(0.0), 0.00) is False
    assert gate._update_translation_rotation_gate(_quaternion_z(2.0), 0.10) is False
    assert gate._translation_rotation_locked is True

    # First quiet sample starts the release timer; it must not release early.
    assert gate._update_translation_rotation_gate(_quaternion_z(2.0), 0.20) is False
    assert gate._update_translation_rotation_gate(_quaternion_z(2.0), 0.30) is False
    assert gate._translation_rotation_locked is True

    assert gate._update_translation_rotation_gate(_quaternion_z(2.0), 0.40) is True
    assert gate._translation_rotation_locked is False


def test_gate_stays_open_for_slow_tracker_jitter() -> None:
    gate = _gate()
    assert gate._update_translation_rotation_gate(_quaternion_z(0.0), 0.0) is False
    assert gate._update_translation_rotation_gate(_quaternion_z(0.1), 0.1) is False
    assert gate._translation_rotation_locked is False


def test_gate_ignores_single_frame_orientation_noise() -> None:
    gate = _gate()
    assert gate._update_translation_rotation_gate(_quaternion_z(0.0), 0.00) is False

    # A one-frame spike is never promoted to a speed estimate.  At the end of
    # the 80 ms window the net orientation has returned to its starting pose.
    assert gate._update_translation_rotation_gate(_quaternion_z(0.3), 0.01) is False
    assert gate._update_translation_rotation_gate(_quaternion_z(0.0), 0.08) is False
    assert gate._translation_rotation_locked is False
