"""SO101 native replay tests that do not require ROS2."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from so101_gateway.replay_so101_episode import (
    load_replay_episode,
    make_interpolation,
    publish_joint_targets,
    validate_target,
)


JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


def _rows() -> list[dict]:
    rows = []
    for frame_index in range(3):
        rows.append(
            {
                "frame_index": frame_index,
                "joint_names": JOINT_NAMES,
                "action_joint_names": JOINT_NAMES,
                "state_positions_rad": [0.1 * frame_index] * 6,
                "action_positions_rad": [0.1 * frame_index + 0.01] * 6,
            }
        )
    return rows


def _write_episode(root: Path, rows: list[dict] | None = None) -> Path:
    episode_dir = root / "episodes" / "episode_000001"
    episode_dir.mkdir(parents=True)
    (root / "dataset.yaml").write_text(
        "robot: so101\n"
        "fps: 5\n"
        "joint_names:\n"
        + "".join(f"  - {name}\n" for name in JOINT_NAMES)
        + "state_unit: radian\n"
        + "action_unit: radian\n"
        + "action_type: absolute_joint_position\n",
        encoding="utf-8",
    )
    (episode_dir / "episode.json").write_text(json.dumps({"fps": 5, "num_frames": 3}) + "\n", encoding="utf-8")
    with (episode_dir / "frames.jsonl").open("w", encoding="utf-8") as f:
        for row in rows or _rows():
            f.write(json.dumps(row) + "\n")
    return episode_dir


class SO101ReplayTest(unittest.TestCase):
    def test_load_episode_from_dir_or_frames_path(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            episode_dir = _write_episode(Path(tmpdir))

            episode = load_replay_episode(episode_dir, expected_joint_names=JOINT_NAMES)
            self.assertEqual(episode.fps, 5.0)
            self.assertEqual(episode.start_pose, [0.0] * 6)
            self.assertEqual(len(episode.trajectory), 3)

            segment = load_replay_episode(
                episode_dir / "frames.jsonl",
                expected_joint_names=JOINT_NAMES,
                start_frame=1,
                end_frame=2,
            )
            self.assertEqual(segment.start_frame, 1)
            self.assertEqual(segment.end_frame, 2)
            self.assertEqual(segment.start_pose, [0.1] * 6)
            self.assertEqual(len(segment.trajectory), 2)

    def test_interpolation_auto_limits_step_delta(self) -> None:
        points = make_interpolation(
            [0.0] * 6,
            [0.06] * 6,
            rate_hz=10,
            max_delta_rad=0.03,
            duration_s=None,
        )
        self.assertEqual(len(points), 2)
        self.assertAlmostEqual(points[0][0], 0.03, places=6)
        self.assertAlmostEqual(points[-1][0], 0.06, places=6)

    def test_joint_order_mismatch_rejected(self) -> None:
        rows = _rows()
        rows[1] = dict(rows[1])
        rows[1]["joint_names"] = list(reversed(JOINT_NAMES))
        with tempfile.TemporaryDirectory() as tmpdir:
            episode_dir = _write_episode(Path(tmpdir), rows)
            with self.assertRaisesRegex(ValueError, "joint_names mismatch"):
                load_replay_episode(episode_dir, expected_joint_names=JOINT_NAMES)

    def test_limit_and_delta_rejected(self) -> None:
        action_cfg = {"actuation_enabled": False, "stale_timeout_s": 10.0}
        safety_cfg = {
            "limits_required_for_actuation": True,
            "limits_required_for_validation": True,
            "max_delta_per_step": 0.05,
            "joint_limits": {name: [-1.0, 1.0] for name in JOINT_NAMES},
        }
        with self.assertRaisesRegex(ValueError, "exceeds"):
            validate_target(
                targets=[0.1] * 6,
                current_positions=[0.0] * 6,
                joint_names=JOINT_NAMES,
                action_cfg=action_cfg,
                safety_cfg=safety_cfg,
            )
        with self.assertRaisesRegex(ValueError, "outside"):
            validate_target(
                targets=[1.2] + [0.0] * 5,
                current_positions=[0.0] * 6,
                joint_names=JOINT_NAMES,
                action_cfg=action_cfg,
                safety_cfg=safety_cfg,
            )

    def test_dry_run_does_not_publish(self) -> None:
        published = []

        class FakePublisher:
            def publish(self, message) -> None:
                published.append(message)

        class FakeJointState:
            def __init__(self) -> None:
                self.header = SimpleNamespace(stamp=None)
                self.name = []
                self.position = []

        class FakeNode:
            def get_clock(self):
                return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: "stamp"))

        did_publish = publish_joint_targets(
            publisher=FakePublisher(),
            joint_state_msg_cls=FakeJointState,
            node=FakeNode(),
            joint_names=JOINT_NAMES,
            targets=[0.1] * 6,
            actuate=False,
        )
        self.assertFalse(did_publish)
        self.assertEqual(published, [])

        did_publish = publish_joint_targets(
            publisher=FakePublisher(),
            joint_state_msg_cls=FakeJointState,
            node=FakeNode(),
            joint_names=JOINT_NAMES,
            targets=[0.1] * 6,
            actuate=True,
        )
        self.assertTrue(did_publish)
        self.assertEqual(len(published), 1)
        self.assertEqual(published[0].position, [0.1] * 6)


if __name__ == "__main__":
    unittest.main()
