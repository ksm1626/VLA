"""SO101 gateway helper tests that do not require ROS2."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from remote_so101.config_loader import load_yaml
from remote_so101.proto_modules import pb2
from so101_gateway.so101_sensor_gateway import (
    EncodedImage,
    ImageFrame,
    JointSnapshot,
    SO101SensorGateway,
    build_sensor_packet,
    encode_rgb_image,
    image_msg_to_rgb_array,
    now_ns,
    validate_action_packet,
)


class SO101GatewayHelpersTest(unittest.TestCase):
    def test_ros_image_conversion_and_packet_build(self) -> None:
        image = np.zeros((2, 3, 3), dtype=np.uint8)
        image[..., 0] = 10
        image[..., 1] = 20
        image[..., 2] = 30
        msg = SimpleNamespace(
            height=2,
            width=3,
            step=9,
            encoding="rgb8",
            data=image.tobytes(),
        )

        rgb = image_msg_to_rgb_array(msg)
        self.assertEqual(rgb.shape, (2, 3, 3))
        self.assertEqual(int(rgb[0, 0, 0]), 10)

        encoded = encode_rgb_image(rgb, width=3, height=2, encoding="rgb8", jpeg_quality=80)
        packet = build_sensor_packet(
            sequence_id=7,
            instruction="test task",
            front=encoded,
            top=EncodedImage(encoded.data, encoded.width, encoded.height, encoded.encoding),
            joints=JointSnapshot(["j0", "j1"], [0.1, 0.2], timestamp_ns=123),
        )
        self.assertEqual(packet.sequence_id, 7)
        self.assertEqual(packet.front_image.encoding, "rgb8")
        self.assertEqual(list(packet.joint_names), ["j0", "j1"])

    def test_validate_action_rejects_invalid_and_missing_limits_when_actuating(self) -> None:
        joints = JointSnapshot(["j0", "j1"], [0.0, 0.0], timestamp_ns=123)
        packet = pb2.ActionPacket(
            sequence_id=1,
            timestamp_ns=1_000_000_000,
            joint_names=["j0", "j1"],
            joint_targets=[0.01, 0.02],
        )

        targets = validate_action_packet(
            packet,
            expected_joint_names=["j0", "j1"],
            current_joints=joints,
            action_cfg={"actuation_enabled": False, "stale_timeout_s": 10.0},
            safety_cfg={"limits_required_for_actuation": True, "max_delta_per_step": 0.05},
            current_time_ns=1_100_000_000,
        )
        self.assertAlmostEqual(targets[0], 0.01, places=6)
        self.assertAlmostEqual(targets[1], 0.02, places=6)

        with self.assertRaisesRegex(ValueError, "joint limits missing"):
            validate_action_packet(
                packet,
                expected_joint_names=["j0", "j1"],
                current_joints=joints,
                action_cfg={"actuation_enabled": True, "stale_timeout_s": 10.0},
                safety_cfg={"limits_required_for_actuation": True, "max_delta_per_step": 0.05},
                current_time_ns=1_100_000_000,
            )

        bad_packet = pb2.ActionPacket(
            sequence_id=2,
            timestamp_ns=1_000_000_000,
            joint_names=["j0", "j1"],
            joint_targets=[0.5, 0.0],
        )
        with self.assertRaisesRegex(ValueError, "exceeds"):
            validate_action_packet(
                bad_packet,
                expected_joint_names=["j0", "j1"],
                current_joints=joints,
                action_cfg={"actuation_enabled": False, "stale_timeout_s": 10.0},
                safety_cfg={"limits_required_for_actuation": True, "max_delta_per_step": 0.05},
                current_time_ns=1_100_000_000,
            )

    def test_gateway_config_defaults_match_actuated_so101_runtime(self) -> None:
        config = load_yaml("configs/so101_gateway.yaml")
        self.assertTrue(config["actions"]["actuation_enabled"])
        self.assertTrue(config["actions"]["clamp_to_joint_limits"])
        self.assertEqual(config["actions"]["publish_rate_limit_hz"], 0)
        self.assertIsNone(config["safety"]["max_delta_per_step"])
        self.assertTrue(config["safety"]["limits_required_for_actuation"])
        self.assertTrue(config["safety"]["limits_required_for_validation"])

        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "minimal.yaml"
            path.write_text("actions:\n  actuation_enabled: false\n", encoding="utf-8")
            minimal = load_yaml(path)
            self.assertFalse(minimal["actions"]["actuation_enabled"])

    def test_gateway_dry_run_validates_action(self) -> None:
        config = {
            "bridge": {"host": "127.0.0.1", "port": 49100},
            "ros": {
                "front_camera_topic": "/front",
                "top_camera_topic": "/top",
                "joint_states_topic": "/joint_states",
                "joint_targets_topic": "/joint_targets",
            },
            "sensor": {"joint_names": ["j0", "j1"], "stale_timeout_s": 1.0},
            "actions": {"actuation_enabled": False, "stale_timeout_s": 1.0},
            "safety": {
                "limits_required_for_actuation": True,
                "limits_required_for_validation": True,
                "max_delta_per_step": 0.05,
                "joint_limits": {"j0": [-1.0, 1.0], "j1": [-1.0, 1.0]},
            },
        }
        gateway = SO101SensorGateway(config)
        image = np.zeros((2, 2, 3), dtype=np.uint8)
        gateway._front = ImageFrame(image, timestamp_ns=now_ns())
        gateway._top = ImageFrame(image, timestamp_ns=now_ns())
        gateway._joints = JointSnapshot(["j0", "j1"], [0.0, 0.0], timestamp_ns=now_ns())

        packet = pb2.ActionPacket(
            sequence_id=1,
            timestamp_ns=now_ns(),
            joint_names=["j0", "j1"],
            joint_targets=[0.01, -0.01],
        )
        gateway._publish_action(packet)

        bad_packet = pb2.ActionPacket(
            sequence_id=2,
            timestamp_ns=now_ns(),
            joint_names=["j0", "j1"],
            joint_targets=[0.2, 0.0],
        )
        with self.assertRaisesRegex(ValueError, "exceeds"):
            gateway._publish_action(bad_packet)

    def test_gateway_rejects_action_when_camera_is_stale(self) -> None:
        config = {
            "bridge": {"host": "127.0.0.1", "port": 49100},
            "ros": {
                "front_camera_topic": "/front",
                "top_camera_topic": "/top",
                "joint_states_topic": "/joint_states",
                "joint_targets_topic": "/joint_targets",
            },
            "sensor": {"joint_names": ["j0", "j1"], "stale_timeout_s": 0.001},
            "actions": {"actuation_enabled": False, "stale_timeout_s": 1.0},
            "safety": {
                "limits_required_for_actuation": True,
                "limits_required_for_validation": True,
                "max_delta_per_step": 0.05,
                "joint_limits": {"j0": [-1.0, 1.0], "j1": [-1.0, 1.0]},
            },
        }
        gateway = SO101SensorGateway(config)
        image = np.zeros((2, 2, 3), dtype=np.uint8)
        old_timestamp = now_ns() - 1_000_000_000
        gateway._front = ImageFrame(image, timestamp_ns=old_timestamp)
        gateway._top = ImageFrame(image, timestamp_ns=old_timestamp)
        gateway._joints = JointSnapshot(["j0", "j1"], [0.0, 0.0], timestamp_ns=now_ns())

        packet = pb2.ActionPacket(
            sequence_id=1,
            timestamp_ns=now_ns(),
            joint_names=["j0", "j1"],
            joint_targets=[0.01, -0.01],
        )
        with self.assertRaisesRegex(TimeoutError, "stale sensor data"):
            gateway._publish_action(packet)


if __name__ == "__main__":
    unittest.main()
