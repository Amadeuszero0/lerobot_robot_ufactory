import numpy as np

from lerobot_robot_ufactory_piper.pika_teleop import tracker_control_point_mm


def _quaternion_z(degrees: float) -> np.ndarray:
    half = np.radians(degrees) / 2.0
    # The Vive transformation helper uses [x, y, z, w].
    return np.array([0.0, 0.0, np.sin(half), np.cos(half)], dtype=float)


def test_zero_offset_keeps_previous_raw_translation_behaviour() -> None:
    result = tracker_control_point_mm(
        np.array([0.12, -0.08, 0.50]),
        _quaternion_z(45.0),
        0.50,
        np.zeros(3),
    )
    np.testing.assert_allclose(result, [60.0, -40.0, 250.0], atol=1e-9)


def test_offset_cancels_tracker_arc_during_pure_rotation() -> None:
    scale = 0.50
    physical_offset_mm = np.array([100.0, 0.0, 0.0])
    configured_offset_mm = physical_offset_mm * scale
    fixed_control_point_m = np.array([0.4, -0.2, 0.3])

    results = []
    for degrees in (0.0, 30.0, 75.0, 120.0):
        quaternion = _quaternion_z(degrees)
        half = np.radians(degrees) / 2.0
        rotation = np.array(
            [
                [np.cos(2 * half), -np.sin(2 * half), 0.0],
                [np.sin(2 * half), np.cos(2 * half), 0.0],
                [0.0, 0.0, 1.0],
            ]
        )
        tracker_position_m = (
            fixed_control_point_m
            - rotation @ (physical_offset_mm / 1000.0)
        )
        results.append(
            tracker_control_point_mm(
                tracker_position_m,
                quaternion,
                scale,
                configured_offset_mm,
            )
        )

    for result in results[1:]:
        np.testing.assert_allclose(result, results[0], atol=1e-9)
