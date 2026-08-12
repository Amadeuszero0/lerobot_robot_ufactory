import math

from dataclasses import dataclass, field
from lerobot.cameras import CameraConfig
from lerobot.robots import RobotConfig
from lerobot.teleoperators import TeleoperatorConfig

from lerobot_robot_ufactory.teleoperators.pika_teleop import PikaTeleopConfig


@RobotConfig.register_subclass("uf::piper")
@dataclass(kw_only=True)
class PiperFollowerConfig(RobotConfig):
    """One Piper follower arm.

    ``cartesian`` actions use millimetres for xyz and radians as an axis-angle
    vector for rx/ry/rz, matching the UFACTORY Pika teleoperator.
    """

    port: str
    control_space: str = "joint"
    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    configure_role_on_connect: bool = True
    # The SDK's PiperInit helper performs its own control writes. Keep the
    # historical default for existing single-arm profiles, while allowing
    # dual-arm profiles to use the same explicit, piper_init=False connection
    # path as the verified direct gripper test.
    piper_init_on_connect: bool = True
    # Existing arm profiles explicitly enable all joints on connect. A
    # gripper-only diagnostic can leave the already-running arm state untouched
    # to exactly match a direct SDK GripperCtrl test.
    enable_torque_on_connect: bool = True
    park_on_connect: bool = False
    park_on_disconnect: bool = False
    disable_torque_on_disconnect: bool = True

    # Joint mode values are normalized to [-100, 100] (gripper [0, 100]).
    max_relative_target: float | dict[str, float] | None = 5.0

    # Cartesian/Pika safety limits. Translation is mm, rotation is rad.
    max_cartesian_step_mm: float = 10.0
    max_rotation_step_rad: float = 0.10
    translation_deadband_mm: float = 0.0
    rotation_deadband_rad: float = 0.0
    lock_orientation: bool = False
    # Allows a true gripper-only profile: observations and gripper commands
    # remain active, but EndPoseCtrl/ModeCtrl are never sent by send_action.
    send_cartesian_pose: bool = True
    # 'official_ik' is the independent Lerobot-Real/PikaAnyArm control chain:
    # solve the Piper-X official URDF in a worker and stream physical joints.
    cartesian_command_mode: str = "step"
    ik_urdf_path: str | None = None
    ik_package_dir: str | None = None
    max_cartesian_following_error_mm: float = 600.0
    max_rotation_following_error_rad: float = 3.2
    # Optional per-cycle caps in direct mode (None = unbounded, V16 behavior).
    # Set to ~25 mm / 0.35 rad to prevent large single-cycle target jumps
    # from stalling the firmware MOVE P replanner on fast gestures.
    direct_max_step_mm: float | None = None
    direct_max_step_rad: float | None = None
    # Piper SDK EndPose feedback/control is located at the J6 origin. This
    # optional translation-only offset targets a local-frame TCP. Tools whose
    # frame also rotates relative to J6 (including the PikaAnyArm PiperX tool)
    # must instead use the full tracker_to_robot_eef rigid transform in the
    # teleoperator; do not enable both compensations for the same tool.
    tcp_offset_mm: tuple[float, ...] = (0.0, 0.0, 0.0)
    startup_tcp_pose: tuple[float, ...] | None = None
    startup_move_timeout_s: float = 30.0
    hold_position_on_disconnect: bool = False
    feedback_startup_timeout_s: float = 5.0
    max_tracking_error_mm: float | None = None
    workspace_x: tuple[float, float] | None = None
    workspace_y: tuple[float, float] | None = None
    workspace_z: tuple[float, float] | None = None
    move_mode: str = "move_p"
    move_speed_percent: int = 20
    gripper_effort: int = 1000
    # Piper SDK gripper control code. 0x01 enables the gripper; the official
    # Piper LeRobot adapter uses 0x03 to enable it while clearing gripper errors.
    gripper_ctrl_code: int = 0x01
    send_gripper: bool = True
    min_command_interval_s: float = 0.0
    gripper_command_deadband: float = 0.0
    gripper_min_command_interval_s: float | None = None
    gripper_keepalive_s: float | None = None
    gripper_debug: bool = False

    def __post_init__(self) -> None:
        super().__post_init__()
        self.id = "piper_follower" if self.id is None else self.id
        if self.control_space not in ("joint", "cartesian"):
            raise ValueError(f"Unsupported Piper control_space: {self.control_space}")
        if self.move_mode not in ("move_p", "move_l", "move_cpv"):
            raise ValueError(f"Unsupported Piper move_mode: {self.move_mode}")
        if not 1 <= self.move_speed_percent <= 100:
            raise ValueError("move_speed_percent must be in [1, 100]")
        if self.max_cartesian_step_mm <= 0 or self.max_rotation_step_rad <= 0:
            raise ValueError("Cartesian step limits must be positive")
        if self.translation_deadband_mm < 0 or self.rotation_deadband_rad < 0:
            raise ValueError("Cartesian deadbands must be non-negative")
        if self.max_tracking_error_mm is not None and self.max_tracking_error_mm <= 0:
            raise ValueError("max_tracking_error_mm must be positive when set")
        if self.min_command_interval_s < 0:
            raise ValueError("min_command_interval_s must be non-negative")
        if self.gripper_command_deadband < 0:
            raise ValueError("gripper_command_deadband must be non-negative")
        if self.gripper_ctrl_code not in (0x01, 0x03):
            raise ValueError("gripper_ctrl_code must be 0x01 or 0x03")
        if (
            self.gripper_min_command_interval_s is not None
            and self.gripper_min_command_interval_s < 0
        ):
            raise ValueError("gripper_min_command_interval_s must be non-negative")
        if self.gripper_keepalive_s is not None and self.gripper_keepalive_s <= 0:
            raise ValueError("gripper_keepalive_s must be positive when set")
        for name in ("workspace_x", "workspace_y", "workspace_z"):
            bounds = getattr(self, name)
            if bounds is not None and bounds[0] >= bounds[1]:
                raise ValueError(f"{name} must be ordered as (min, max)")
        if self.cartesian_command_mode not in ("step", "direct", "official_ik"):
            raise ValueError(
                "cartesian_command_mode must be 'step', 'direct', or 'official_ik'"
            )
        if self.cartesian_command_mode == "official_ik" and (
            not self.ik_urdf_path or not self.ik_package_dir
        ):
            raise ValueError(
                "official_ik requires both ik_urdf_path and ik_package_dir"
            )
        if self.max_cartesian_following_error_mm <= 0 or self.max_rotation_following_error_rad <= 0:
            raise ValueError("following error limits must be positive")
        if self.startup_tcp_pose is not None and len(self.startup_tcp_pose) != 6:
            raise ValueError("startup_tcp_pose must contain six values")
        if self.startup_move_timeout_s <= 0:
            raise ValueError("startup_move_timeout_s must be positive")
        if self.direct_max_step_mm is not None and self.direct_max_step_mm <= 0:
            raise ValueError("direct_max_step_mm must be positive when set")
        if self.direct_max_step_rad is not None and self.direct_max_step_rad <= 0:
            raise ValueError("direct_max_step_rad must be positive when set")
        if len(self.tcp_offset_mm) != 3 or not all(
            math.isfinite(float(value)) for value in self.tcp_offset_mm
        ):
            raise ValueError("tcp_offset_mm must contain three finite values")


@RobotConfig.register_subclass("uf::dual_piper")
@dataclass(kw_only=True)
class DualPiperFollowerConfig(RobotConfig):
    robots: dict[str, RobotConfig] = field(default_factory=dict)
    parallel_connect: bool = True
    parallel_observation: bool = True
    parallel_action: bool = True

    def __post_init__(self) -> None:
        super().__post_init__()
        self.id = "dual_piper_follower" if self.id is None else self.id
        if len(self.robots) != 2:
            raise ValueError("uf::dual_piper requires exactly two follower arms")
        ports = [getattr(robot, "port", None) for robot in self.robots.values()]
        if any(port is None for port in ports):
            raise ValueError("each dual Piper follower must define a CAN port")
        if len(set(ports)) != len(ports):
            raise ValueError("dual Piper followers must use distinct CAN ports")


@TeleoperatorConfig.register_subclass("uf::piper_leader")
@dataclass(kw_only=True)
class PiperLeaderConfig(TeleoperatorConfig):
    port: str
    configure_role_on_connect: bool = True
    disable_torque_on_disconnect: bool = True

    def __post_init__(self) -> None:
        self.id = "piper_leader" if self.id is None else self.id


@TeleoperatorConfig.register_subclass("uf::dual_piper_leader")
@dataclass(kw_only=True)
class DualPiperLeaderConfig(TeleoperatorConfig):
    teleops: dict[str, TeleoperatorConfig] = field(default_factory=dict)
    parallel_connect: bool = True
    parallel_read: bool = True

    def __post_init__(self) -> None:
        self.id = "dual_piper_leader" if self.id is None else self.id
        if len(self.teleops) != 2:
            raise ValueError("uf::dual_piper_leader requires exactly two leader arms")


@TeleoperatorConfig.register_subclass("uf::dual_pika_teleop")
@dataclass(kw_only=True)
class DualPikaTeleopConfig(TeleoperatorConfig):
    teleops: dict[str, TeleoperatorConfig] = field(default_factory=dict)
    parallel_read: bool = True

    def __post_init__(self) -> None:
        self.id = "dual_pika" if self.id is None else self.id
        if len(self.teleops) != 2:
            raise ValueError("uf::dual_pika_teleop requires exactly two Pika devices")
        ports = [getattr(teleop, "port", None) for teleop in self.teleops.values()]
        if any(port is None for port in ports):
            raise ValueError("each dual Pika teleoperator must define a serial port")
        if len(set(ports)) != len(ports):
            raise ValueError("dual Pika teleoperators must use distinct serial ports")
        tracker_ids = [
            getattr(teleop, "tracker_device_id", None)
            for teleop in self.teleops.values()
        ]
        if any(tracker_ids):
            if not all(tracker_ids):
                raise ValueError(
                    "configure tracker_device_id for both Pikas or for neither"
                )
            if len(set(tracker_ids)) != len(tracker_ids):
                raise ValueError("dual Pikas must use distinct tracker_device_id values")


@TeleoperatorConfig.register_subclass("uf::piper_pika_teleop")
@dataclass
class PiperPikaTeleopConfig(PikaTeleopConfig):
    """Pika input tuned for Piper without changing the base Pika profile."""

    gripper_filter_window: int = 3
    gripper_filter_alpha: float = 0.35
    gripper_deadband: float = 0.01
    gripper_max_step: float = 0.08
    gripper_use_direct_distance: bool = False
    gripper_distance_min_mm: float = 0.0
    gripper_distance_max_mm: float = 100.0
    use_calibrated_rotation_mapping: bool = False
    apply_piper_tool_axis_correction: bool = False
    # "calibrated" = fixed-matrix body-frame mapping; "calibrated_tool" keeps
    # that verified gesture mapping and additionally rotates about a physical
    # tool centre; "world_delta" = Lerobot-Real robot_base rotation only;
    # "senior" = the full Lerobot-Real robot_base control target;
    # "official" = the senior's unmodified local tool-frame formula.
    rotation_style: str = "calibrated"
    # Full rigid transform C (tool centre -> native Piper J6), expressed as
    # xyz(mm)+RPY(deg).  This is deliberately separate from
    # tracker_to_robot_eef so an already calibrated Pika gesture mapping does
    # not change axes when tool-centre compensation is enabled.
    piper_tool_center_to_j6: tuple[float, ...] | None = None
    # Rebuild the calibrated rotation mapping at teleop enable from the actual
    # end-effector orientation, so pitch/yaw/roll keep the correct axes at any
    # arm pose (not just the pose used when the matrix was derived).
    pose_adaptive_rotation: bool = False
    # Fixed rotation from the Vive/Pika tracking world to the robot base, in
    # degrees. Used by rotation_style=senior as Q (world -> base).
    tracker_world_to_robot_base_rpy: tuple[float, ...] = (0, 0, 0)
    rotation_dominant_axis: bool = False
    rotation_scale: float = 1.0
    rotation_filter_alpha: float = 1.0
    # Read-only/diagnostic profiles can emit the complete rotation mapping
    # chain at a bounded rate.  Keep this disabled in normal teleoperation so
    # logging cannot disturb the control-loop timing.
    rotation_debug: bool = False
    rotation_debug_interval_s: float = 0.20
    use_raw_translation_mapping: bool = False
    # Dual-arm: force which tracker this side uses (e.g. T20 / T21), and
    # optionally override the raw translation matrix per side (None = use the
    # measured module-level matrix).
    tracker_device_id: str | None = None
    raw_translation_matrix: tuple[tuple[float, float, float], ...] | None = None
    # Optional intent gate for operators who perform translation and rotation
    # as separate gestures.  While tracker angular speed is above the engage
    # threshold, hold XYZ and keep sending orientation/gripper commands.  Once
    # rotation has settled, re-anchor translation without an endpoint jump.
    freeze_translation_while_rotating: bool = False
    translation_rotation_lock_speed_rad_s: float = 0.15
    translation_rotation_release_speed_rad_s: float = 0.05
    translation_rotation_release_delay_s: float = 0.15
    translation_rotation_speed_window_s: float = 0.08
    # Optional translation-intent override for the rotation gate.  When the
    # scaled tracker control point moves faster than this, allow XYZ even if
    # the operator's arm motion also contains wrist rotation.  None preserves
    # the original rotation-only gate behavior.
    translation_rotation_lock_max_translation_speed_mm_s: float | None = None
    # Override the calibrated rotation mapping (raw tracker relative rotation
    # vector -> EEF-local rotation vector) when re-derived for a specific
    # pose. Columns are the mapped directions for raw X (roll), raw Y (pitch),
    # raw Z (yaw). None keeps the built-in calibrated matrices.
    rotation_mapping_matrix: tuple[tuple[float, float, float], ...] | None = None

    def __post_init__(self) -> None:
        super().__post_init__()
        if self.gripper_filter_window < 1 or self.gripper_filter_window % 2 == 0:
            raise ValueError("gripper_filter_window must be a positive odd number")
        if not 0 < self.gripper_filter_alpha <= 1:
            raise ValueError("gripper_filter_alpha must be in (0, 1]")
        if self.gripper_deadband < 0:
            raise ValueError("gripper_deadband must be non-negative")
        if not 0 < self.gripper_max_step <= 1:
            raise ValueError("gripper_max_step must be in (0, 1]")
        if self.gripper_distance_min_mm >= self.gripper_distance_max_mm:
            raise ValueError(
                "gripper_distance_min_mm must be smaller than "
                "gripper_distance_max_mm"
            )
        if self.rotation_scale == 0 or abs(self.rotation_scale) > 1:
            raise ValueError("rotation_scale must be in [-1, 0) or (0, 1]")
        if self.rotation_style not in (
            "calibrated",
            "calibrated_tool",
            "world_delta",
            "senior",
            "official",
        ):
            raise ValueError(
                "rotation_style must be 'calibrated', 'calibrated_tool', "
                "'world_delta', 'senior' or 'official'"
            )
        if self.piper_tool_center_to_j6 is not None:
            if len(self.piper_tool_center_to_j6) != 6 or not all(
                math.isfinite(value) for value in self.piper_tool_center_to_j6
            ):
                raise ValueError(
                    "piper_tool_center_to_j6 must contain six finite values"
                )
        if (
            self.rotation_style == "calibrated_tool"
            and self.piper_tool_center_to_j6 is None
        ):
            raise ValueError(
                "rotation_style='calibrated_tool' requires "
                "piper_tool_center_to_j6"
            )
        if len(self.tracker_world_to_robot_base_rpy) != 3 or not all(
            math.isfinite(value) for value in self.tracker_world_to_robot_base_rpy
        ):
            raise ValueError(
                "tracker_world_to_robot_base_rpy must contain three finite values"
            )
        if not 0 < self.rotation_filter_alpha <= 1:
            raise ValueError("rotation_filter_alpha must be in (0, 1]")
        if self.rotation_debug_interval_s <= 0:
            raise ValueError("rotation_debug_interval_s must be positive")
        if self.raw_translation_matrix is not None:
            if len(self.raw_translation_matrix) != 3 or any(
                len(row) != 3 for row in self.raw_translation_matrix
            ):
                raise ValueError("raw_translation_matrix must be a 3x3 matrix")
        if self.translation_rotation_lock_speed_rad_s <= 0:
            raise ValueError(
                "translation_rotation_lock_speed_rad_s must be positive"
            )
        if not (
            0 <= self.translation_rotation_release_speed_rad_s
            < self.translation_rotation_lock_speed_rad_s
        ):
            raise ValueError(
                "translation_rotation_release_speed_rad_s must be non-negative "
                "and smaller than the lock speed"
            )
        if self.translation_rotation_release_delay_s < 0:
            raise ValueError(
                "translation_rotation_release_delay_s must be non-negative"
            )
        if self.translation_rotation_speed_window_s <= 0:
            raise ValueError(
                "translation_rotation_speed_window_s must be positive"
            )
        if (
            self.translation_rotation_lock_max_translation_speed_mm_s
            is not None
            and self.translation_rotation_lock_max_translation_speed_mm_s <= 0
        ):
            raise ValueError(
                "translation_rotation_lock_max_translation_speed_mm_s must be "
                "positive when set"
            )
        if self.rotation_mapping_matrix is not None:
            if len(self.rotation_mapping_matrix) != 3 or any(
                len(row) != 3 for row in self.rotation_mapping_matrix
            ):
                raise ValueError("rotation_mapping_matrix must be a 3x3 matrix")
