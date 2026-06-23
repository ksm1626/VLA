#!/usr/bin/env python3
"""Record SO101 teleop episodes as a local native dataset.

The recorder intentionally avoids LeRobot, torch, and transformers on the
SO101 PC. It subscribes to manufacturer ROS2 topics, writes camera streams to
MP4, and writes frame metadata/state/action to JSONL.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import queue
import shutil
import sys
import threading
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from remote_so101.config_loader import load_yaml, section  # noqa: E402
from so101_gateway.so101_sensor_gateway import image_msg_to_rgb_array, now_ns  # noqa: E402


@dataclass(frozen=True)
class ImageSample:
    image_rgb: np.ndarray
    timestamp_ns: int


@dataclass(frozen=True)
class JointSample:
    names: list[str]
    positions: list[float]
    timestamp_ns: int


@dataclass(frozen=True)
class JoySample:
    axes: list[float]
    buttons: list[int]
    timestamp_ns: int


@dataclass(frozen=True)
class PoseSample:
    position: list[float]
    orientation: list[float]
    timestamp_ns: int


def utc_now_iso() -> str:
    """Return an ISO-8601 UTC timestamp."""
    return datetime.now(timezone.utc).isoformat()


def expand_path(path: str | Path) -> Path:
    """Expand a user-facing path without requiring it to exist."""
    return Path(os.path.expandvars(os.path.expanduser(str(path)))).resolve()


def finite_list(values: list[float], *, field_name: str) -> list[float]:
    """Validate that all values are finite floats."""
    output = [float(value) for value in values]
    if not all(math.isfinite(value) for value in output):
        raise ValueError(f"{field_name} contains NaN or Inf")
    return output


def order_joint_positions(sample: JointSample, expected_joint_names: list[str], *, field_name: str) -> list[float]:
    """Return joint positions ordered by expected_joint_names."""
    if len(sample.names) != len(sample.positions):
        raise ValueError(f"{field_name} has mismatched name/position lengths")
    by_name = dict(zip(sample.names, sample.positions, strict=True))
    missing = [name for name in expected_joint_names if name not in by_name]
    if missing:
        raise ValueError(f"{field_name} missing joints: {', '.join(missing)}")
    return finite_list([by_name[name] for name in expected_joint_names], field_name=field_name)


def latest_is_fresh(timestamp_ns: int, current_ns: int, stale_timeout_s: float) -> bool:
    """Return true when a sample timestamp is not stale."""
    return (current_ns - int(timestamp_ns)) <= int(float(stale_timeout_s) * 1e9)


def resize_rgb(image_rgb: np.ndarray, width: int | None, height: int | None) -> np.ndarray:
    """Resize an RGB image when width and height are provided."""
    if image_rgb.dtype != np.uint8 or image_rgb.ndim != 3 or image_rgb.shape[2] != 3:
        raise ValueError("image_rgb must be uint8 HWC with 3 channels")
    if width is None or height is None:
        return np.ascontiguousarray(image_rgb)
    if image_rgb.shape[1] == int(width) and image_rgb.shape[0] == int(height):
        return np.ascontiguousarray(image_rgb)

    import cv2

    return cv2.resize(image_rgb, (int(width), int(height)), interpolation=cv2.INTER_AREA)


def make_frame_row(
    *,
    frame_index: int,
    episode_start_ns: int,
    task: str,
    teleop_source: str,
    joint_names: list[str],
    front: ImageSample,
    top: ImageSample,
    state: JointSample,
    action: JointSample,
    leader: JointSample | None = None,
    joy: JoySample | None = None,
    target_pose: PoseSample | None = None,
    gripper_open: bool | None = None,
) -> dict[str, Any]:
    """Build one JSONL row for a native SO101 episode."""
    current_ns = now_ns()
    row: dict[str, Any] = {
        "frame_index": int(frame_index),
        "timestamp_ns": int(current_ns),
        "relative_time_s": (current_ns - int(episode_start_ns)) / 1e9,
        "task": str(task),
        "teleop_source": str(teleop_source),
        "front_frame_index": int(frame_index),
        "top_frame_index": int(frame_index),
        "front_timestamp_ns": int(front.timestamp_ns),
        "top_timestamp_ns": int(top.timestamp_ns),
        "state_timestamp_ns": int(state.timestamp_ns),
        "action_timestamp_ns": int(action.timestamp_ns),
        "joint_names": list(joint_names),
        "state_positions_rad": order_joint_positions(state, joint_names, field_name="state"),
        "action_joint_names": list(joint_names),
        "action_positions_rad": order_joint_positions(action, joint_names, field_name="action"),
    }

    debug: dict[str, Any] = {}
    if leader is not None:
        try:
            debug["leader_positions_rad"] = order_joint_positions(leader, joint_names, field_name="leader")
            debug["leader_timestamp_ns"] = int(leader.timestamp_ns)
        except ValueError as exc:
            debug["leader_error"] = str(exc)
    if joy is not None:
        debug["joy_axes"] = list(joy.axes)
        debug["joy_buttons"] = list(joy.buttons)
        debug["joy_timestamp_ns"] = int(joy.timestamp_ns)
    if target_pose is not None:
        debug["target_pose_position"] = list(target_pose.position)
        debug["target_pose_orientation"] = list(target_pose.orientation)
        debug["target_pose_timestamp_ns"] = int(target_pose.timestamp_ns)
    if gripper_open is not None:
        debug["gripper_open"] = bool(gripper_open)
    if debug:
        row["debug"] = debug
    return row


class NativeEpisodeWriter:
    """Write one native SO101 episode to a temporary directory then finalize."""

    def __init__(
        self,
        *,
        dataset_root: Path,
        dataset_name: str,
        robot: str,
        fps: int,
        joint_names: list[str],
        task: str,
        teleop_source: str,
        width: int | None,
        height: int | None,
        video_codec: str,
        config: dict[str, Any],
    ) -> None:
        self.dataset_root = dataset_root
        self.dataset_name = dataset_name
        self.robot = robot
        self.fps = int(fps)
        self.joint_names = list(joint_names)
        self.task = task
        self.teleop_source = teleop_source
        self.width = width
        self.height = height
        self.video_codec = video_codec
        self.config = config
        self.frame_index = 0
        self.started_at = utc_now_iso()
        self.episode_start_ns = now_ns()
        self._front_writer = None
        self._top_writer = None
        self._frames_file = None
        self._front_shape: tuple[int, int, int] | None = None
        self._top_shape: tuple[int, int, int] | None = None
        self._closed = False

        self.dataset_root.mkdir(parents=True, exist_ok=True)
        (self.dataset_root / "episodes").mkdir(parents=True, exist_ok=True)
        self.episode_index = self._next_episode_index()
        self.tmp_dir = self.dataset_root / "episodes" / f"episode_{self.episode_index:06d}.tmp"
        self.final_dir = self.dataset_root / "episodes" / f"episode_{self.episode_index:06d}"
        if self.tmp_dir.exists() or self.final_dir.exists():
            raise FileExistsError(f"episode path already exists: {self.tmp_dir} or {self.final_dir}")
        (self.tmp_dir / "videos").mkdir(parents=True)
        self._frames_file = (self.tmp_dir / "frames.jsonl").open("w", encoding="utf-8")
        self._write_dataset_metadata()

    def _next_episode_index(self) -> int:
        existing = []
        for path in (self.dataset_root / "episodes").glob("episode_*"):
            name = path.name.removesuffix(".tmp")
            try:
                existing.append(int(name.split("_", 1)[1]))
            except (IndexError, ValueError):
                continue
        return max(existing, default=0) + 1

    def _write_dataset_metadata(self) -> None:
        import yaml

        metadata = {
            "dataset_name": self.dataset_name,
            "robot": self.robot,
            "created_with": "so101_native_recorder",
            "fps": self.fps,
            "joint_names": self.joint_names,
            "state_topic": self.config["topics"]["joint_states"],
            "action_topic": self.config["topics"]["joint_targets"],
            "camera_topics": {
                "front": self.config["topics"]["front_camera"],
                "top": self.config["topics"]["top_camera"],
            },
            "state_unit": self.config["schema"]["state_unit"],
            "action_unit": self.config["schema"]["action_unit"],
            "action_type": self.config["schema"]["action_type"],
            "native_format": "mp4_jsonl_v1",
            "export": self.config.get("export", {}),
        }
        path = self.dataset_root / "dataset.yaml"
        path.write_text(yaml.safe_dump(metadata, sort_keys=False), encoding="utf-8")

    def _open_video_writer(self, path: Path, image_rgb: np.ndarray):
        import cv2

        resized = resize_rgb(image_rgb, self.width, self.height)
        height, width = resized.shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*self.video_codec)
        writer = cv2.VideoWriter(str(path), fourcc, float(self.fps), (width, height))
        if not writer.isOpened():
            raise RuntimeError(f"failed to open video writer: {path}")
        return writer, (height, width, 3), resized

    def _write_video_frame(self, writer, image_rgb: np.ndarray) -> None:
        import cv2

        resized = resize_rgb(image_rgb, self.width, self.height)
        image_bgr = cv2.cvtColor(resized, cv2.COLOR_RGB2BGR)
        writer.write(image_bgr)

    def add_frame(
        self,
        *,
        front: ImageSample,
        top: ImageSample,
        state: JointSample,
        action: JointSample,
        leader: JointSample | None,
        joy: JoySample | None,
        target_pose: PoseSample | None,
        gripper_open: bool | None,
    ) -> None:
        if self._front_writer is None:
            self._front_writer, self._front_shape, first_front = self._open_video_writer(
                self.tmp_dir / "videos" / "front.mp4",
                front.image_rgb,
            )
            self._top_writer, self._top_shape, first_top = self._open_video_writer(
                self.tmp_dir / "videos" / "top.mp4",
                top.image_rgb,
            )
            self._write_video_frame(self._front_writer, first_front)
            self._write_video_frame(self._top_writer, first_top)
        else:
            self._write_video_frame(self._front_writer, front.image_rgb)
            self._write_video_frame(self._top_writer, top.image_rgb)

        row = make_frame_row(
            frame_index=self.frame_index,
            episode_start_ns=self.episode_start_ns,
            task=self.task,
            teleop_source=self.teleop_source,
            joint_names=self.joint_names,
            front=front,
            top=top,
            state=state,
            action=action,
            leader=leader,
            joy=joy,
            target_pose=target_pose,
            gripper_open=gripper_open,
        )
        assert self._frames_file is not None
        self._frames_file.write(json.dumps(row, separators=(",", ":")) + "\n")
        self._frames_file.flush()
        self.frame_index += 1

    def _release_handles(self) -> None:
        if self._front_writer is not None:
            self._front_writer.release()
            self._front_writer = None
        if self._top_writer is not None:
            self._top_writer.release()
            self._top_writer = None
        if self._frames_file is not None:
            self._frames_file.close()
            self._frames_file = None

    def close(self, *, status: str = "saved") -> Path:
        if self._closed:
            return self.final_dir

        self._release_handles()

        ended_at = utc_now_iso()
        episode = {
            "episode_index": self.episode_index,
            "robot": self.robot,
            "task": self.task,
            "teleop_source": self.teleop_source,
            "fps": self.fps,
            "num_frames": self.frame_index,
            "duration_s": self.frame_index / float(self.fps) if self.fps > 0 else 0.0,
            "status": status,
            "started_at": self.started_at,
            "ended_at": ended_at,
            "joint_names": self.joint_names,
            "state_unit": self.config["schema"]["state_unit"],
            "action_unit": self.config["schema"]["action_unit"],
            "action_type": self.config["schema"]["action_type"],
            "front_video": "videos/front.mp4",
            "top_video": "videos/top.mp4",
            "front_video_shape_hwc": list(self._front_shape or []),
            "top_video_shape_hwc": list(self._top_shape or []),
        }
        (self.tmp_dir / "episode.json").write_text(json.dumps(episode, indent=2) + "\n", encoding="utf-8")
        self.tmp_dir.rename(self.final_dir)
        self._closed = True
        return self.final_dir

    def discard(self) -> Path:
        """Close file handles and remove the temporary episode directory."""
        if self._closed:
            return self.tmp_dir

        self._release_handles()
        if self.tmp_dir.exists():
            shutil.rmtree(self.tmp_dir)
        self._closed = True
        return self.tmp_dir


class SO101EpisodeRecorder:
    """ROS2 node wrapper that records consecutive episodes."""

    def __init__(self, config: dict[str, Any], *, task: str, teleop_source: str) -> None:
        self.config = config
        self.dataset_cfg = section(config, "dataset")
        self.topic_cfg = section(config, "topics")
        self.recording_cfg = section(config, "recording")
        self.schema_cfg = section(config, "schema")
        self.task = task
        self.teleop_source = teleop_source
        self.joint_names = [str(name) for name in self.schema_cfg["joint_names"]]
        self.stale_timeout_s = float(self.recording_cfg.get("stale_timeout_s", 1.0))
        self.require_action = bool(self.recording_cfg.get("require_action", True))
        self.include_debug_topics = bool(self.recording_cfg.get("include_debug_topics", True))
        self.keyboard_controls = bool(self.recording_cfg.get("keyboard_controls", True))
        self.min_frames_per_episode = int(self.recording_cfg.get("min_frames_per_episode", 1))
        self._front: ImageSample | None = None
        self._top: ImageSample | None = None
        self._state: JointSample | None = None
        self._action: JointSample | None = None
        self._leader: JointSample | None = None
        self._joy: JoySample | None = None
        self._target_pose: PoseSample | None = None
        self._gripper_open: bool | None = None
        self._node = None
        self._writer: NativeEpisodeWriter | None = None
        self._last_wait_log_ns = 0
        self._writer_lock = threading.RLock()
        self._command_queue: queue.Queue[str] = queue.Queue()
        self._stop_event = threading.Event()
        self._keyboard_thread: threading.Thread | None = None

    def _front_callback(self, msg: Any) -> None:
        self._front = ImageSample(image_msg_to_rgb_array(msg), now_ns())

    def _top_callback(self, msg: Any) -> None:
        self._top = ImageSample(image_msg_to_rgb_array(msg), now_ns())

    def _state_callback(self, msg: Any) -> None:
        self._state = JointSample([str(name) for name in msg.name], [float(v) for v in msg.position], now_ns())

    def _action_callback(self, msg: Any) -> None:
        self._action = JointSample([str(name) for name in msg.name], [float(v) for v in msg.position], now_ns())

    def _leader_callback(self, msg: Any) -> None:
        self._leader = JointSample([str(name) for name in msg.name], [float(v) for v in msg.position], now_ns())

    def _joy_callback(self, msg: Any) -> None:
        self._joy = JoySample([float(v) for v in msg.axes], [int(v) for v in msg.buttons], now_ns())

    def _target_pose_callback(self, msg: Any) -> None:
        self._target_pose = PoseSample(
            [
                float(msg.pose.position.x),
                float(msg.pose.position.y),
                float(msg.pose.position.z),
            ],
            [
                float(msg.pose.orientation.x),
                float(msg.pose.orientation.y),
                float(msg.pose.orientation.z),
                float(msg.pose.orientation.w),
            ],
            now_ns(),
        )

    def _gripper_open_callback(self, msg: Any) -> None:
        self._gripper_open = bool(msg.data)

    def _missing_inputs(self) -> list[str]:
        current_ns = now_ns()
        checks = [
            ("front camera", self._front),
            ("top camera", self._top),
            ("joint state", self._state),
        ]
        if self.require_action:
            checks.append(("joint target action", self._action))

        missing = []
        for label, sample in checks:
            if sample is None:
                missing.append(f"{label} not received yet")
            elif not latest_is_fresh(sample.timestamp_ns, current_ns, self.stale_timeout_s):
                missing.append(f"{label} is stale")
        return missing

    def _tick(self) -> None:
        self._drain_commands()
        if self._stop_event.is_set():
            return

        missing = self._missing_inputs()
        if missing:
            current_ns = now_ns()
            if current_ns - self._last_wait_log_ns > 1_000_000_000:
                print("Waiting for recording inputs:", "; ".join(missing), flush=True)
                self._last_wait_log_ns = current_ns
            return

        assert self._front is not None
        assert self._top is not None
        assert self._state is not None
        action = self._action if self._action is not None else self._state
        try:
            with self._writer_lock:
                if self._writer is None:
                    return
                self._writer.add_frame(
                    front=self._front,
                    top=self._top,
                    state=self._state,
                    action=action,
                    leader=self._leader,
                    joy=self._joy,
                    target_pose=self._target_pose,
                    gripper_open=self._gripper_open,
                )
        except Exception as exc:  # noqa: BLE001
            print(f"Recording frame skipped: {exc}", flush=True)

    def _create_writer(self) -> NativeEpisodeWriter:
        fps = int(self.dataset_cfg.get("fps", 10))
        dataset_root = expand_path(self.dataset_cfg["root"])
        width = self.recording_cfg.get("width")
        height = self.recording_cfg.get("height")
        width = None if width is None else int(width)
        height = None if height is None else int(height)
        return NativeEpisodeWriter(
            dataset_root=dataset_root,
            dataset_name=str(self.dataset_cfg.get("name", dataset_root.name)),
            robot=str(self.dataset_cfg.get("robot", "so101")),
            fps=fps,
            joint_names=self.joint_names,
            task=self.task,
            teleop_source=self.teleop_source,
            width=width,
            height=height,
            video_codec=str(self.recording_cfg.get("video_codec", "mp4v")),
            config=self.config,
        )

    def _start_next_episode(self) -> None:
        with self._writer_lock:
            self._writer = self._create_writer()
            print(
                f"Recording episode_{self._writer.episode_index:06d} at {self._writer.fps} FPS",
                flush=True,
            )

    def _finish_current_episode(self, *, save: bool, start_next: bool) -> None:
        with self._writer_lock:
            writer = self._writer
            self._writer = None
            if writer is None:
                if start_next and not self._stop_event.is_set():
                    self._start_next_episode()
                return

            frame_count = writer.frame_index
            should_save = save and frame_count >= self.min_frames_per_episode
            if should_save:
                out = writer.close(status="saved")
                print(f"Saved {out}", flush=True)
                print(f"Frames: {frame_count}", flush=True)
            else:
                writer.discard()
                if save:
                    reason = f"only {frame_count} frames (< {self.min_frames_per_episode})"
                else:
                    reason = "user discarded"
                print(f"Discarded episode_{writer.episode_index:06d}: {reason}", flush=True)

            if start_next and not self._stop_event.is_set():
                self._writer = self._create_writer()
                print(
                    f"Recording episode_{self._writer.episode_index:06d} at {self._writer.fps} FPS",
                    flush=True,
                )

    def _keyboard_loop(self) -> None:
        while not self._stop_event.is_set():
            line = sys.stdin.readline()
            if line == "":
                return
            command = line.strip().lower()
            if command == "":
                self._command_queue.put("save_next")
            elif command in {"c", "cancel", "discard"}:
                self._command_queue.put("discard_next")
            elif command in {"q", "quit", "exit"}:
                self._command_queue.put("save_quit")
                return
            else:
                print("Unknown command. Use Enter=save+next, c=discard+next, q=save+quit.", flush=True)

    def _start_keyboard_controls(self) -> None:
        if not self.keyboard_controls:
            return
        self._keyboard_thread = threading.Thread(target=self._keyboard_loop, name="so101_recorder_keyboard", daemon=True)
        self._keyboard_thread.start()

    def _request_stop(self) -> None:
        self._stop_event.set()
        if self._node is None:
            return
        context = getattr(self._node, "context", None)
        if context is None:
            return
        try:
            if context.ok():
                context.shutdown()
        except Exception:  # noqa: BLE001
            pass

    def _handle_command(self, command: str) -> None:
        if command == "save_next":
            self._finish_current_episode(save=True, start_next=True)
        elif command == "discard_next":
            self._finish_current_episode(save=False, start_next=True)
        elif command == "save_quit":
            self._finish_current_episode(save=True, start_next=False)
            self._request_stop()

    def _drain_commands(self) -> None:
        while not self._stop_event.is_set():
            try:
                command = self._command_queue.get_nowait()
            except queue.Empty:
                return
            self._handle_command(command)

    def run(self) -> int:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.executors import ExternalShutdownException
        from sensor_msgs.msg import Image, JointState, Joy
        from std_msgs.msg import Bool

        fps = int(self.dataset_cfg.get("fps", 10))
        self._start_next_episode()

        rclpy.init()
        self._node = rclpy.create_node("so101_episode_recorder")
        qos_depth = int(self.recording_cfg.get("qos_depth", 5))
        self._node.create_subscription(Image, str(self.topic_cfg["front_camera"]), self._front_callback, qos_depth)
        self._node.create_subscription(Image, str(self.topic_cfg["top_camera"]), self._top_callback, qos_depth)
        self._node.create_subscription(JointState, str(self.topic_cfg["joint_states"]), self._state_callback, qos_depth)
        self._node.create_subscription(JointState, str(self.topic_cfg["joint_targets"]), self._action_callback, qos_depth)
        if self.include_debug_topics:
            self._node.create_subscription(
                JointState,
                str(self.topic_cfg.get("leader_joint_states", "/leader/joint_states")),
                self._leader_callback,
                qos_depth,
            )
            self._node.create_subscription(Joy, str(self.topic_cfg.get("joy", "/joy")), self._joy_callback, qos_depth)
            self._node.create_subscription(
                PoseStamped,
                str(self.topic_cfg.get("target_pose", "/target_pose")),
                self._target_pose_callback,
                qos_depth,
            )
            self._node.create_subscription(
                Bool,
                str(self.topic_cfg.get("gripper_open", "/gripper_open")),
                self._gripper_open_callback,
                qos_depth,
            )
        self._node.create_timer(1.0 / max(1, fps), self._tick)

        print(f"Task: {self.task}", flush=True)
        print(f"Teleop source: {self.teleop_source}", flush=True)
        if self.keyboard_controls:
            print("Controls: Enter=save+next, c=discard+next, q=save+quit, Ctrl+C=save+quit", flush=True)
            self._start_keyboard_controls()
        else:
            print("Press Ctrl+C to finish and save.", flush=True)
        try:
            rclpy.spin(self._node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            self._finish_current_episode(save=True, start_next=False)
            if self._node is not None:
                self._node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/so101_recording.yaml")
    parser.add_argument("--task", required=True)
    parser.add_argument("--teleop-source", choices=["leader", "joystick", "manual", "other"], required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = load_yaml(args.config)
    return SO101EpisodeRecorder(config, task=args.task, teleop_source=args.teleop_source).run()


if __name__ == "__main__":
    raise SystemExit(main())
