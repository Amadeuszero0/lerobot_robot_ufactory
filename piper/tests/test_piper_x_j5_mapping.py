from pathlib import Path

import numpy as np
import yaml


_CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"

# tracker_to_robot_eef RPY = [180, -90, 0] degrees.  A tracker-local
# axis-angle vector is conjugated into the parent Pika action by C.T @ raw.
_TRACKER_TO_EEF_ROTATION = np.array(
    [
        [0.0, 0.0, 1.0],
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
    ],
    dtype=float,
)

_MEASURED_ROTATIONS = {
    "left": np.array(
        [
            [-0.0310, 0.7018, -0.3957],  # pitch down
            [0.7278, 0.0531, -0.0632],  # wrist right -> J5 positive
            [0.2546, -0.4809, -0.6417],  # handle roll -> J6
        ],
        dtype=float,
    ),
    "right": np.array(
        [
            [-0.0928, 0.6318, -0.4610],
            [1.1580, 0.1235, -0.0646],
            [0.1261, -0.6097, -0.7316],
        ],
        dtype=float,
    ),
}

_EXPECTED_AXES = np.array(
    [
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ],
    dtype=float,
)


def _load_teleops(filename: str) -> dict:
    data = yaml.safe_load((_CONFIG_DIR / filename).read_text(encoding="utf-8"))
    return data["teleop"]["teleops"]


def _load_config(filename: str) -> dict:
    return yaml.safe_load((_CONFIG_DIR / filename).read_text(encoding="utf-8"))


def test_preview_and_actuated_profiles_use_the_same_j5_mapping() -> None:
    preview = _load_teleops("dual_pika_piper_x_j5_preview.yaml")
    actuated = _load_teleops("dual_pika_piper_x_j5_test.yaml")

    for side in ("left", "right"):
        assert preview[side]["rotation_mapping_matrix"] == actuated[side][
            "rotation_mapping_matrix"
        ]
        assert preview[side]["pose_adaptive_rotation"] is False
        assert actuated[side]["pose_adaptive_rotation"] is False


def test_measured_gestures_map_to_piper_x_wrist_axes() -> None:
    teleops = _load_teleops("dual_pika_piper_x_j5_test.yaml")

    for side, raw_vectors in _MEASURED_ROTATIONS.items():
        code_vectors = (_TRACKER_TO_EEF_ROTATION.T @ raw_vectors.T).T
        code_vectors /= np.linalg.norm(code_vectors, axis=1)[:, None]
        mapping = np.asarray(teleops[side]["rotation_mapping_matrix"], dtype=float)
        mapped = (mapping @ code_vectors.T).T
        np.testing.assert_allclose(mapped, _EXPECTED_AXES, atol=2e-7)


def test_j5_test_keeps_measured_translation_and_stable_motion_settings() -> None:
    baseline = _load_teleops("dual_pika_piper_measured_translation.yaml")
    candidate = _load_teleops("dual_pika_piper_x_j5_test.yaml")

    for side in ("left", "right"):
        assert candidate[side]["raw_translation_matrix"] == baseline[side][
            "raw_translation_matrix"
        ]
        assert candidate[side]["scale_xyz"] == baseline[side]["scale_xyz"] == 0.50
        assert candidate[side]["rotation_scale"] == 0.60


def test_rotation_lock_profile_only_adds_separate_gesture_gate() -> None:
    baseline = _load_teleops("dual_pika_piper_x_j5_test.yaml")
    candidate = _load_teleops("dual_pika_piper_x_j5_rotation_lock_test.yaml")

    for side in ("left", "right"):
        assert candidate[side]["raw_translation_matrix"] == baseline[side][
            "raw_translation_matrix"
        ]
        assert candidate[side]["rotation_mapping_matrix"] == baseline[side][
            "rotation_mapping_matrix"
        ]
        assert candidate[side]["rotation_scale"] == baseline[side]["rotation_scale"]
        assert candidate[side]["freeze_translation_while_rotating"] is True
        assert candidate[side]["translation_rotation_lock_speed_rad_s"] == 0.12
        assert candidate[side]["translation_rotation_release_speed_rad_s"] == 0.04
        assert candidate[side]["translation_rotation_release_delay_s"] == 0.15
        assert candidate[side]["translation_rotation_speed_window_s"] == 0.08


def test_left_fix_v2_uses_an_orthogonal_left_rotation_mapping() -> None:
    teleops = _load_teleops(
        "dual_pika_piper_x_j5_left_fix_preview_v2.yaml"
    )
    mapping = np.asarray(teleops["left"]["rotation_mapping_matrix"], dtype=float)

    np.testing.assert_allclose(mapping.T @ mapping, np.eye(3), atol=1e-8)
    np.testing.assert_allclose(np.linalg.det(mapping), -1.0, atol=1e-8)

    raw_vectors = _MEASURED_ROTATIONS["left"]
    code_vectors = (_TRACKER_TO_EEF_ROTATION.T @ raw_vectors.T).T
    code_vectors /= np.linalg.norm(code_vectors, axis=1)[:, None]
    mapped = (mapping @ code_vectors.T).T
    direction_errors_deg = np.degrees(
        np.arccos(np.clip(np.sum(mapped * _EXPECTED_AXES, axis=1), -1.0, 1.0))
    )
    assert np.max(direction_errors_deg) < 11.0


def test_left_fix_v2_is_read_only_and_matches_rotation_lock_gate() -> None:
    baseline = _load_teleops("dual_pika_piper_x_j5_test.yaml")
    candidate = _load_config(
        "dual_pika_piper_x_j5_left_fix_preview_v2.yaml"
    )

    for side in ("left", "right"):
        robot = candidate["robot"]["robots"][side]
        teleop = candidate["teleop"]["teleops"][side]
        assert robot["dry_run"] is True
        assert robot["enable_torque_on_connect"] is False
        assert robot["send_gripper"] is False
        assert robot["dry_run_max_branch_jump_deg"] == 8.0
        assert robot["dry_run_j5_singularity_deg"] == 3.0
        assert teleop["freeze_translation_while_rotating"] is True
        assert teleop["translation_rotation_lock_speed_rad_s"] == 0.12
        assert teleop["translation_rotation_release_speed_rad_s"] == 0.04
        assert teleop["translation_rotation_release_delay_s"] == 0.15
        assert teleop["translation_rotation_speed_window_s"] == 0.08
        assert teleop["rotation_debug"] is True
        assert teleop["raw_translation_matrix"] == baseline[side][
            "raw_translation_matrix"
        ]

    assert candidate["teleop"]["teleops"]["right"][
        "rotation_mapping_matrix"
    ] == baseline["right"]["rotation_mapping_matrix"]


def test_left_wrist_v3_is_single_arm_move_p_and_rotation_only() -> None:
    preview = _load_config(
        "dual_pika_piper_x_j5_left_fix_preview_v2.yaml"
    )
    candidate = _load_config(
        "single_pika_piper_x_left_wrist_v3.yaml"
    )
    robot = candidate["robot"]
    teleop = candidate["teleop"]

    assert robot["type"] == "uf::piper"
    assert robot["port"] == "can_left"
    assert robot["cartesian_command_mode"] == "direct"
    assert robot["move_mode"] == "move_p"
    assert robot["enable_torque_on_connect"] is True
    assert robot["move_speed_percent"] == 10
    assert robot["max_rotation_step_rad"] == 0.025
    assert robot["direct_max_step_rad"] == 0.025
    assert robot["send_gripper"] is False

    assert teleop["port"] == "/dev/pika_left"
    assert teleop["tracker_device_id"] == "LHR-818D4A5D"
    assert teleop["scale_xyz"] == 0.0
    assert teleop["use_gripper"] is False
    assert teleop["rotation_scale"] == 0.30
    assert teleop["rotation_mapping_matrix"] == preview["teleop"]["teleops"][
        "left"
    ]["rotation_mapping_matrix"]


def test_left_wrist_v4_only_raises_v3_move_speed_to_baseline() -> None:
    v3 = _load_config("single_pika_piper_x_left_wrist_v3.yaml")
    v4 = _load_config(
        "single_pika_piper_x_left_wrist_full_speed_v4.yaml"
    )

    assert v4["robot"]["move_speed_percent"] == 55
    assert v3["robot"]["move_speed_percent"] == 10

    ignored = {"id", "move_speed_percent"}
    assert {
        key: value for key, value in v4["robot"].items() if key not in ignored
    } == {
        key: value for key, value in v3["robot"].items() if key not in ignored
    }
    assert {
        key: value for key, value in v4["teleop"].items() if key != "id"
    } == {
        key: value for key, value in v3["teleop"].items() if key != "id"
    }


def test_full_v5_uses_verified_dual_arm_move_p_settings() -> None:
    preview = _load_config(
        "dual_pika_piper_x_j5_left_fix_preview_v2.yaml"
    )
    baseline = _load_config(
        "dual_pika_piper_x_j5_rotation_lock_test.yaml"
    )
    candidate = _load_config("dual_pika_piper_x_full_v5.yaml")

    assert candidate["robot"]["type"] == "uf::dual_piper"
    assert candidate["robot"]["parallel_action"] is False
    for side in ("left", "right"):
        robot = candidate["robot"]["robots"][side]
        teleop = candidate["teleop"]["teleops"][side]
        assert robot["type"] == "uf::piper"
        assert robot["move_mode"] == "move_p"
        assert robot["move_speed_percent"] == 55
        assert robot["send_gripper"] is True
        assert teleop["scale_xyz"] == 0.50
        assert teleop["use_gripper"] is True
        assert teleop["freeze_translation_while_rotating"] is True
        assert teleop["rotation_scale"] == 0.30
        assert teleop["rotation_debug"] is False
        assert teleop["raw_translation_matrix"] == baseline["teleop"][
            "teleops"
        ][side]["raw_translation_matrix"]

    assert candidate["teleop"]["teleops"]["left"][
        "rotation_mapping_matrix"
    ] == preview["teleop"]["teleops"]["left"]["rotation_mapping_matrix"]
    assert candidate["teleop"]["teleops"]["right"][
        "rotation_mapping_matrix"
    ] == baseline["teleop"]["teleops"]["right"][
        "rotation_mapping_matrix"
    ]


def test_full_motion_v6_only_adds_translation_intent_to_v5() -> None:
    v5 = _load_config("dual_pika_piper_x_full_v5.yaml")
    v6 = _load_config("dual_pika_piper_x_full_motion_v6.yaml")

    assert v6["robot"]["type"] == "uf::dual_piper"
    assert v6["robot"]["parallel_action"] is False
    for side in ("left", "right"):
        robot = v6["robot"]["robots"][side]
        teleop = v6["teleop"]["teleops"][side]
        assert robot["move_speed_percent"] == 55
        assert teleop["scale_xyz"] == 0.50
        assert teleop[
            "translation_rotation_lock_max_translation_speed_mm_s"
        ] == 20.0

        assert {
            key: value
            for key, value in robot.items()
            if key != "id"
        } == {
            key: value
            for key, value in v5["robot"]["robots"][side].items()
            if key != "id"
        }
        assert {
            key: value
            for key, value in teleop.items()
            if key
            not in {
                "translation_rotation_lock_max_translation_speed_mm_s"
            }
        } == v5["teleop"]["teleops"][side]
