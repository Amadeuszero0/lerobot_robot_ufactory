import numpy as np

from lerobot_robot_ufactory_piper_history_ed30ad0.joint_stream_stabilizer import (
    JointStreamStabilizer,
)


def _stabilizer(**overrides) -> JointStreamStabilizer:
    values = {
        "frequency_hz": 50.0,
        "max_velocity_deg_s": 3.0,
        "max_acceleration_deg_s2": 20.0,
        "max_step_deg": 0.10,
        "max_following_error_deg": 0.75,
        "max_ik_jump_deg": 1.0,
    }
    values.update(overrides)
    return JointStreamStabilizer(**values)


def test_acceleration_and_step_are_bounded() -> None:
    limiter = _stabilizer()
    feedback = np.zeros(6)
    limiter.reset(feedback, now_s=0.0)
    target = np.radians(np.full(6, 0.8))

    first = limiter.advance(target, feedback, now_s=0.02, dry_run=True)
    second = limiter.advance(target, first.command_rad, now_s=0.04, dry_run=True)

    assert np.max(np.abs(np.degrees(first.step_rad))) <= 0.0081
    assert np.max(np.abs(np.degrees(second.step_rad))) <= 0.0161
    assert np.max(np.abs(np.degrees(second.step_rad))) <= 0.10


def test_large_ik_branch_jump_is_rejected() -> None:
    limiter = _stabilizer(max_ik_jump_deg=1.0)
    feedback = np.zeros(6)
    limiter.reset(feedback, now_s=0.0)

    result = limiter.advance(
        np.radians([0.0, 0.0, 0.0, 0.0, 5.0, 0.0]),
        feedback,
        now_s=0.02,
        dry_run=False,
    )

    assert result.reason == "ik_jump"
    assert np.allclose(result.command_rad, feedback)
    assert np.allclose(result.step_rad, 0.0)


def test_feedback_lag_holds_last_command() -> None:
    limiter = _stabilizer(max_following_error_deg=0.05)
    feedback = np.zeros(6)
    limiter.reset(feedback, now_s=0.0)
    target = np.radians(np.full(6, 0.5))

    moving = limiter.advance(target, feedback, now_s=0.02, dry_run=False)
    assert moving.reason == "ok"

    lagging = limiter.advance(
        target,
        feedback,
        now_s=0.04,
        dry_run=False,
    )
    # 第一周期只有约 0.008°，尚未超过 0.05°；继续推进到保护门触发。
    for index in range(2, 20):
        lagging = limiter.advance(
            target,
            feedback,
            now_s=0.02 * (index + 1),
            dry_run=False,
        )
        if lagging.reason == "following_error":
            break

    assert lagging.reason == "following_error"
    assert np.allclose(lagging.step_rad, 0.0)


def test_dry_run_ignores_feedback_lag_but_keeps_other_guards() -> None:
    limiter = _stabilizer(max_following_error_deg=0.01)
    feedback = np.zeros(6)
    limiter.reset(feedback, now_s=0.0)
    target = np.radians(np.full(6, 0.5))

    result = None
    for index in range(10):
        result = limiter.advance(
            target,
            feedback,
            now_s=0.02 * (index + 1),
            dry_run=True,
        )

    assert result is not None
    assert result.reason == "ok"
    assert np.max(np.abs(result.command_rad)) > 0.0
