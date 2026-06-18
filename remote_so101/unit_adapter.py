"""Unit conversions between SO101 ROS joints and SmolVLA policy joints."""

from __future__ import annotations

import math
from typing import Mapping

from lerobot.types import RobotAction

from remote_so101.config import RemoteSO101Config


def clamp(value: float, low: float, high: float) -> float:
    """Clamp a float into an inclusive range."""
    return max(low, min(high, value))


def _is_arm_joint(name: str, config: RemoteSO101Config) -> bool:
    return name in set(config.arm_joint_names)


def _is_gripper_joint(name: str, config: RemoteSO101Config) -> bool:
    return name == config.gripper_joint_name


def ros_joint_to_policy(name: str, value: float, config: RemoteSO101Config) -> float:
    """Convert a SO101 ROS joint value to the policy-facing value."""
    value = float(value)
    if not config.unit_adapter_enabled:
        return value
    if _is_arm_joint(name, config):
        return math.degrees(value)
    if _is_gripper_joint(name, config):
        span = config.gripper_ros_open - config.gripper_ros_closed
        if span == 0:
            raise ValueError("gripper_ros_open and gripper_ros_closed must differ")
        ratio = (value - config.gripper_ros_closed) / span
        policy = config.gripper_policy_min + ratio * (
            config.gripper_policy_max - config.gripper_policy_min
        )
        return clamp(policy, config.gripper_policy_min, config.gripper_policy_max)
    return value


def policy_joint_to_ros(
    name: str,
    value: float,
    config: RemoteSO101Config,
    current_ros_positions: Mapping[str, float] | None = None,
) -> float:
    """Convert a policy action value to the SO101 ROS joint target."""
    value = float(value)
    if not config.unit_adapter_enabled:
        return value
    if _is_arm_joint(name, config):
        return math.radians(value)
    if _is_gripper_joint(name, config):
        if config.gripper_action_mode == "hold":
            if current_ros_positions is None or name not in current_ros_positions:
                raise ValueError("gripper hold requires the latest ROS gripper position")
            return float(current_ros_positions[name])
        if config.gripper_action_mode != "linear":
            raise ValueError(f"Unsupported gripper_action_mode: {config.gripper_action_mode}")

        policy_span = config.gripper_policy_max - config.gripper_policy_min
        if policy_span == 0:
            raise ValueError("gripper_policy_min and gripper_policy_max must differ")
        clamped_policy = clamp(value, config.gripper_policy_min, config.gripper_policy_max)
        ratio = (clamped_policy - config.gripper_policy_min) / policy_span
        ros_value = config.gripper_ros_closed + ratio * (
            config.gripper_ros_open - config.gripper_ros_closed
        )
        return clamp(ros_value, config.gripper_ros_min, config.gripper_ros_max)
    return value


def policy_action_to_ros_targets(
    action: RobotAction,
    config: RemoteSO101Config,
    current_ros_positions: Mapping[str, float] | None = None,
) -> list[float]:
    """Return ordered SO101 ROS joint targets for a LeRobot action dict."""
    return [
        policy_joint_to_ros(
            name,
            float(action[name]),
            config,
            current_ros_positions=current_ros_positions,
        )
        for name in config.joint_names
    ]

