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
    rotation_dominant_axis: bool = False
    rotation_scale: float = 1.0
    rotation_filter_alpha: float = 1.0

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
        if not 0 < self.rotation_scale <= 1:
            raise ValueError("rotation_scale must be in (0, 1]")
        if not 0 < self.rotation_filter_alpha <= 1:
            raise ValueError("rotation_filter_alpha must be in (0, 1]")
