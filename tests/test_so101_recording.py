"""SO101 native recorder tests that do not require ROS2."""

from __future__ import annotations

import math
import tempfile
import unittest
from pathlib import Path

import numpy as np

from recording.convert_native_to_lerobot import make_lerobot_features, ros_vector_to_policy
from recording.validate_native_dataset import validate_dataset
from so101_gateway.record_so101_episode import (
    ImageSample,
    JointSample,
    NativeEpisodeWriter,
    SO101EpisodeRecorder,
    make_frame_row,
    now_ns,
    order_joint_positions,
)


JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


def _config(root: str) -> dict:
    return {
        "dataset": {
            "root": root,
            "name": "so101_test",
            "robot": "so101",
            "fps": 5,
        },
        "topics": {
            "front_camera": "/arm/front_cam",
            "top_camera": "/arm/top_cam",
            "joint_states": "/follower/joint_states",
            "joint_targets": "/follower/joint_targets",
        },
        "recording": {
            "width": 8,
            "height": 6,
            "video_codec": "mp4v",
            "stale_timeout_s": 1.0,
            "require_action": True,
        },
        "schema": {
            "joint_names": JOINT_NAMES,
            "state_unit": "radian",
            "action_unit": "radian",
            "action_type": "absolute_joint_position",
        },
        "export": {
            "image_width": 8,
            "image_height": 6,
            "gripper_policy_min": 0.0,
            "gripper_policy_max": 33.0,
            "gripper_ros_closed": 0.0,
            "gripper_ros_open": 0.8,
            "gripper_ros_min": -0.174533,
            "gripper_ros_max": 1.74533,
        },
    }


def _add_dummy_frame(writer: NativeEpisodeWriter, *, value: float = 0.0) -> None:
    image = np.zeros((6, 8, 3), dtype=np.uint8)
    image[..., 0] = 255
    timestamp = now_ns()
    sample = JointSample(JOINT_NAMES, [value] * 6, timestamp)
    writer.add_frame(
        front=ImageSample(image, timestamp),
        top=ImageSample(image, timestamp),
        state=sample,
        action=sample,
        leader=None,
        joy=None,
        target_pose=None,
        gripper_open=None,
    )


class SO101RecordingTest(unittest.TestCase):
    def test_order_joint_positions_uses_expected_order(self) -> None:
        sample = JointSample(["b", "a"], [2.0, 1.0], timestamp_ns=now_ns())
        self.assertEqual(order_joint_positions(sample, ["a", "b"], field_name="state"), [1.0, 2.0])
        with self.assertRaisesRegex(ValueError, "missing joints"):
            order_joint_positions(sample, ["a", "c"], field_name="state")

    def test_make_frame_row_records_state_and_action_separately(self) -> None:
        timestamp = now_ns()
        image = np.zeros((4, 4, 3), dtype=np.uint8)
        state = JointSample(JOINT_NAMES, [0.0, -0.1, 0.2, 0.3, -0.4, 0.0], timestamp)
        action = JointSample(JOINT_NAMES, [0.1, -0.2, 0.3, 0.4, -0.5, 0.8], timestamp)
        row = make_frame_row(
            frame_index=3,
            episode_start_ns=timestamp - 1_000_000_000,
            task="test task",
            teleop_source="leader",
            joint_names=JOINT_NAMES,
            front=ImageSample(image, timestamp),
            top=ImageSample(image, timestamp),
            state=state,
            action=action,
        )
        self.assertEqual(row["frame_index"], 3)
        self.assertEqual(row["state_positions_rad"], state.positions)
        self.assertEqual(row["action_positions_rad"], action.positions)

    def test_native_episode_writer_and_validator(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            config = _config(tmpdir)
            writer = NativeEpisodeWriter(
                dataset_root=Path(tmpdir),
                dataset_name="so101_test",
                robot="so101",
                fps=5,
                joint_names=JOINT_NAMES,
                task="test task",
                teleop_source="joystick",
                width=8,
                height=6,
                video_codec="mp4v",
                config=config,
            )
            image = np.zeros((6, 8, 3), dtype=np.uint8)
            image[..., 0] = 255
            for idx in range(2):
                timestamp = now_ns()
                state = JointSample(JOINT_NAMES, [0.01 * idx] * 6, timestamp)
                action = JointSample(JOINT_NAMES, [0.02 * idx] * 6, timestamp)
                writer.add_frame(
                    front=ImageSample(image, timestamp),
                    top=ImageSample(image, timestamp),
                    state=state,
                    action=action,
                    leader=None,
                    joy=None,
                    target_pose=None,
                    gripper_open=None,
                )
            out = writer.close()
            self.assertTrue((out / "episode.json").exists())
            self.assertTrue((out / "frames.jsonl").exists())
            results = validate_dataset(Path(tmpdir))
            self.assertEqual(results[0]["frames"], 2)

    def test_native_episode_writer_discard_removes_tmp_dir(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            config = _config(tmpdir)
            writer = NativeEpisodeWriter(
                dataset_root=Path(tmpdir),
                dataset_name="so101_test",
                robot="so101",
                fps=5,
                joint_names=JOINT_NAMES,
                task="test task",
                teleop_source="leader",
                width=8,
                height=6,
                video_codec="mp4v",
                config=config,
            )
            tmp_path = writer.tmp_dir
            final_path = writer.final_dir
            self.assertTrue(tmp_path.exists())
            writer.discard()
            self.assertFalse(tmp_path.exists())
            self.assertFalse(final_path.exists())

    def test_recorder_save_next_finalizes_and_starts_next_tmp(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            config = _config(tmpdir)
            config["recording"]["min_frames_per_episode"] = 1
            recorder = SO101EpisodeRecorder(config, task="test task", teleop_source="leader")
            recorder._start_next_episode()
            assert recorder._writer is not None
            _add_dummy_frame(recorder._writer)

            recorder._finish_current_episode(save=True, start_next=True)

            self.assertTrue((Path(tmpdir) / "episodes" / "episode_000001").exists())
            assert recorder._writer is not None
            self.assertEqual(recorder._writer.episode_index, 2)
            self.assertTrue((Path(tmpdir) / "episodes" / "episode_000002.tmp").exists())
            recorder._finish_current_episode(save=False, start_next=False)

    def test_recorder_discards_episode_below_min_frames(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            config = _config(tmpdir)
            config["recording"]["min_frames_per_episode"] = 2
            recorder = SO101EpisodeRecorder(config, task="test task", teleop_source="leader")
            recorder._start_next_episode()
            assert recorder._writer is not None
            _add_dummy_frame(recorder._writer)

            recorder._finish_current_episode(save=True, start_next=False)

            self.assertFalse((Path(tmpdir) / "episodes" / "episode_000001").exists())
            self.assertFalse((Path(tmpdir) / "episodes" / "episode_000001.tmp").exists())

    def test_policy_unit_conversion(self) -> None:
        export_cfg = _config("/tmp/unused")["export"]
        values = [math.pi / 2, -math.pi / 2, 0.0, math.pi, -math.pi, 0.8]
        converted = ros_vector_to_policy(values, JOINT_NAMES, export_cfg)
        self.assertAlmostEqual(float(converted[0]), 90.0, places=5)
        self.assertAlmostEqual(float(converted[1]), -90.0, places=5)
        self.assertAlmostEqual(float(converted[5]), 33.0, places=5)

    def test_lerobot_features_use_video_keys(self) -> None:
        features = make_lerobot_features(joint_names=JOINT_NAMES, width=256, height=256)
        self.assertEqual(features["observation.state"]["shape"], (6,))
        self.assertEqual(features["observation.images.camera1"]["dtype"], "video")
        self.assertEqual(features["observation.images.camera3"]["shape"], (256, 256, 3))
        self.assertEqual(features["action"]["names"], JOINT_NAMES)


if __name__ == "__main__":
    unittest.main()
