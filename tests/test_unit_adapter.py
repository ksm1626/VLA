"""Unit conversion tests for RemoteSO101."""

from __future__ import annotations

import math
import unittest

from remote_so101.config import RemoteSO101Config
from remote_so101.unit_adapter import policy_action_to_ros_targets, policy_joint_to_ros, ros_joint_to_policy


class UnitAdapterTest(unittest.TestCase):
    def test_arm_joints_convert_between_radians_and_degrees(self) -> None:
        config = RemoteSO101Config(id="unit")

        self.assertAlmostEqual(ros_joint_to_policy("shoulder_pan", math.pi / 2, config), 90.0)
        self.assertAlmostEqual(policy_joint_to_ros("shoulder_pan", 90.0, config), math.pi / 2)

    def test_gripper_linear_mapping(self) -> None:
        config = RemoteSO101Config(id="unit")

        self.assertAlmostEqual(ros_joint_to_policy("gripper", 0.0, config), 0.0)
        self.assertAlmostEqual(ros_joint_to_policy("gripper", 0.8, config), 33.0)
        self.assertAlmostEqual(policy_joint_to_ros("gripper", 0.0, config), 0.0)
        self.assertAlmostEqual(policy_joint_to_ros("gripper", 33.0, config), 0.8)
        self.assertAlmostEqual(policy_joint_to_ros("gripper", 100.0, config), 0.8)

    def test_gripper_hold_mapping(self) -> None:
        config = RemoteSO101Config(id="unit", gripper_action_mode="hold")

        target = policy_joint_to_ros("gripper", 33.0, config, current_ros_positions={"gripper": 0.42})
        self.assertEqual(target, 0.42)

    def test_ordered_policy_action_to_ros_targets(self) -> None:
        config = RemoteSO101Config(id="unit")
        action = {
            "shoulder_pan": 90.0,
            "shoulder_lift": -90.0,
            "elbow_flex": 0.0,
            "wrist_flex": 45.0,
            "wrist_roll": -45.0,
            "gripper": 16.5,
        }

        targets = policy_action_to_ros_targets(action, config)

        self.assertAlmostEqual(targets[0], math.pi / 2)
        self.assertAlmostEqual(targets[1], -math.pi / 2)
        self.assertAlmostEqual(targets[3], math.pi / 4)
        self.assertAlmostEqual(targets[4], -math.pi / 4)
        self.assertAlmostEqual(targets[5], 0.4)


if __name__ == "__main__":
    unittest.main()

