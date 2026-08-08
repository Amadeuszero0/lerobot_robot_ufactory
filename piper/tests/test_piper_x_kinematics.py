import numpy as np

from lerobot_robot_ufactory_piper.piper_kinematics import PiperKinematics


REAL_FEEDBACK_SAMPLES = (
    (
        (-29.964, 35.606, -26.016, 17.086, 14.494, -18.139),
        (140.785, -91.352, 278.685, -112.724, -22.159, -126.534),
    ),
    (
        (-14.333, 45.728, -38.186, 53.916, -24.841, -44.033),
        (161.466, -25.968, 273.115, -139.742, -7.596, -55.495),
    ),
)


def test_official_piper_x_fk_matches_real_sdk_feedback() -> None:
    kin = PiperKinematics(model="piper_x")
    for joints_deg, sdk_pose in REAL_FEEDBACK_SAMPLES:
        fk = np.asarray(kin.forward_xyzrpy(np.radians(joints_deg)))
        assert np.linalg.norm(fk[:3] - sdk_pose[:3]) < 0.5
        assert np.linalg.norm(fk[3:] - sdk_pose[3:]) < 0.01


def test_tool_center_rotation_requires_distal_wrist_motion() -> None:
    kin = PiperKinematics(
        model="piper_x", tool_xyzrpy_m=(0.0, 0.0, 0.1425)
    )
    for joints_deg, _ in REAL_FEEDBACK_SAMPLES:
        seed = np.radians(joints_deg)
        pose = np.asarray(kin.forward_xyzrpy(seed))
        distal_deltas = []
        for rpy_axis in range(3):
            target = pose.copy()
            target[3 + rpy_axis] += 5.0
            solution, residual = kin.ik(
                target[:3], target[3:], seed, max_iter=60, tol=1e-6
            )
            assert residual < 1e-4
            distal_deltas.append(np.degrees(solution - seed)[3:6])
        # A fixed TCP rotation cannot be solved by J1-J3 alone on PiPER-X.
        assert np.max(np.abs(np.asarray(distal_deltas)[:, 1:])) > 1.0
