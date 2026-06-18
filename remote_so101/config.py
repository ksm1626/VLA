"""Configuration for the A6000 RemoteSO101Robot adapter."""

from __future__ import annotations

from dataclasses import dataclass, field

from lerobot.robots.config import RobotConfig


DEFAULT_JOINT_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)


@RobotConfig.register_subclass("remote_so101")
@dataclass(kw_only=True)
class RemoteSO101Config(RobotConfig):
    """LeRobot RobotConfig for a SO101 controlled through a remote gateway."""

    bridge_id: str = "default"
    bridge_host: str = "127.0.0.1"
    bridge_port: int = 49100
    sensor_timeout_s: float = 2.0
    image_width: int = 256
    image_height: int = 256
    image_channels: int = 3
    front_camera_key: str = "camera1"
    top_camera_key: str = "camera2"
    joint_names: list[str] = field(default_factory=lambda: list(DEFAULT_JOINT_NAMES))
    arm_joint_names: list[str] = field(default_factory=lambda: list(DEFAULT_JOINT_NAMES[:5]))
    gripper_joint_name: str = "gripper"
    unit_adapter_enabled: bool = True
    gripper_action_mode: str = "linear"
    gripper_policy_min: float = 0.0
    gripper_policy_max: float = 33.0
    gripper_ros_closed: float = 0.0
    gripper_ros_open: float = 0.8
    gripper_ros_min: float = -0.174533
    gripper_ros_max: float = 1.74533
