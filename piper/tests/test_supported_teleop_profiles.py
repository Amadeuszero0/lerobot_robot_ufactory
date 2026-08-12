from pathlib import Path

import numpy as np
import yaml


CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"


def _load(name: str) -> dict:
    return yaml.safe_load((CONFIG_DIR / name).read_text(encoding="utf-8"))


def test_v7_profile_remains_full_speed_move_p() -> None:
    config = _load("dual_pika_piper_x_full_speed_v7.yaml")

    assert config["robot"]["type"] == "uf::dual_piper"
    assert config["robot"]["parallel_action"] is False
    for side in ("left", "right"):
        robot = config["robot"]["robots"][side]
        teleop = config["teleop"]["teleops"][side]
        assert robot["cartesian_command_mode"] == "direct"
        assert robot["move_mode"] == "move_p"
        assert robot["move_speed_percent"] == 100
        assert teleop["rotation_style"] == "calibrated"
        assert teleop["rotation_scale"] == 0.60
        assert teleop["scale_xyz"] == 0.50


def test_single_arm_final_profile_remains_bounded_direct() -> None:
    config = _load("single_pika_piper_setting.yaml")

    assert config["robot"]["type"] == "uf::piper"
    assert config["robot"]["cartesian_command_mode"] == "direct"
    assert config["robot"]["direct_max_step_mm"] == 25.0
    assert config["robot"]["direct_max_step_rad"] == 0.35
    assert config["robot"]["move_speed_percent"] == 70
    assert config["teleop"]["rotation_style"] == "calibrated"
    assert config["teleop"]["rotation_scale"] == 1.0


def test_senior_profile_uses_official_tool_frame_and_ik() -> None:
    config = _load("dual_pika_piper_official_ik.yaml")

    assert config["robot"]["parallel_action"] is True
    for side in ("left", "right"):
        robot = config["robot"]["robots"][side]
        teleop = config["teleop"]["teleops"][side]
        assert robot["cartesian_command_mode"] == "official_ik"
        assert robot["ik_urdf_path"].endswith("piper_x_description.urdf")
        assert teleop["rotation_style"] == "official"
        assert teleop["tracker_to_robot_eef"] == [-190, 0, 0, -90, 0, -90]


def test_official_tool_transform_matches_piper_x_formula() -> None:
    # Rz(+90) @ Ry(-90) @ Tx(190 mm), then invert to obtain tracker -> J6.
    rz = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]])
    tool = np.eye(4)
    tool[:3, :3] = rz @ ry
    tool[:3, 3] = tool[:3, :3] @ np.array([190.0, 0.0, 0.0])
    inverse = np.linalg.inv(tool)

    np.testing.assert_allclose(inverse[:3, 3], [-190.0, 0.0, 0.0], atol=1e-9)
