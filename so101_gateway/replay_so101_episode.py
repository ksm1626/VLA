#!/usr/bin/env python3
"""Replay a recorded SO101 native episode trajectory on the robot.

The script uses native recorder output: MP4 is ignored, and frames.jsonl
provides the absolute radian joint targets. By default this is a dry-run.
Actual publishing requires both --actuate and actions.actuation_enabled=true
in the config.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from recording.validate_native_dataset import read_json, read_jsonl  # noqa: E402
from remote_so101.config_loader import load_yaml, section  # noqa: E402
from remote_so101.proto_modules import pb2  # noqa: E402
from so101_gateway.so101_sensor_gateway import JointSnapshot, now_ns, validate_action_packet  # noqa: E402


@dataclass(frozen=True)
class ReplayEpisode:
    """Loaded replay data from one native episode or segment."""

    frames_path: Path
    fps: float
    joint_names: list[str]
    rows: list[dict[str, Any]]
    start_pose: list[float]
    trajectory: list[list[float]]
    start_frame: int
    end_frame: int


def expand_path(path: str | Path) -> Path:
    """Expand a user-facing path."""
    return Path(os.path.expandvars(os.path.expanduser(str(path)))).resolve()


def finite_vector(values: Any, *, expected_dim: int, field_name: str) -> list[float]:
    """Return a finite float vector with the expected dimension."""
    if not isinstance(values, list) or len(values) != expected_dim:
        raise ValueError(f"{field_name} must be a list of length {expected_dim}")
    output = [float(value) for value in values]
    if not all(math.isfinite(value) for value in output):
        raise ValueError(f"{field_name} contains NaN or Inf")
    return output


def max_abs_delta(a: list[float], b: list[float]) -> float:
    """Return the maximum absolute element-wise difference."""
    if len(a) != len(b):
        raise ValueError("joint vectors have mismatched dimensions")
    return max((abs(float(x) - float(y)) for x, y in zip(a, b, strict=True)), default=0.0)


def resolve_frames_path(path: str | Path) -> Path:
    """Resolve an episode directory or frames.jsonl path to frames.jsonl."""
    resolved = expand_path(path)
    if resolved.is_dir():
        frames_path = resolved / "frames.jsonl"
    else:
        frames_path = resolved
    if frames_path.name != "frames.jsonl":
        raise ValueError(f"expected an episode directory or frames.jsonl path, got: {resolved}")
    if not frames_path.exists():
        raise FileNotFoundError(f"frames.jsonl not found: {frames_path}")
    if frames_path.parent.name.endswith(".tmp"):
        raise ValueError(f"refusing to replay unfinished tmp episode: {frames_path.parent}")
    return frames_path


def infer_episode_fps(frames_path: Path) -> float:
    """Infer FPS from episode.json, dataset.yaml, then fallback to 10 Hz."""
    episode_json = frames_path.parent / "episode.json"
    if episode_json.exists():
        episode = read_json(episode_json)
        fps = float(episode.get("fps", 0.0))
        if fps > 0:
            return fps

    dataset_yaml = frames_path.parents[2] / "dataset.yaml" if len(frames_path.parents) >= 3 else None
    if dataset_yaml is not None and dataset_yaml.exists():
        dataset = load_yaml(dataset_yaml)
        fps = float(dataset.get("fps", 0.0))
        if fps > 0:
            return fps
    return 10.0


def validate_rows(rows: list[dict[str, Any]], *, expected_joint_names: list[str]) -> list[dict[str, Any]]:
    """Validate native frames for replay and normalize vector fields."""
    if not rows:
        raise ValueError("frames.jsonl has no frames")
    expected_dim = len(expected_joint_names)
    normalized = []
    for expected_index, row in enumerate(rows):
        frame_index = int(row.get("frame_index", -1))
        if frame_index != expected_index:
            raise ValueError(f"non-contiguous frame_index at row {expected_index}: got {frame_index}")
        if list(row.get("joint_names", [])) != expected_joint_names:
            raise ValueError(f"joint_names mismatch at frame {frame_index}")
        if list(row.get("action_joint_names", [])) != expected_joint_names:
            raise ValueError(f"action_joint_names mismatch at frame {frame_index}")

        copied = dict(row)
        copied["state_positions_rad"] = finite_vector(
            row.get("state_positions_rad"),
            expected_dim=expected_dim,
            field_name=f"frame {frame_index} state_positions_rad",
        )
        copied["action_positions_rad"] = finite_vector(
            row.get("action_positions_rad"),
            expected_dim=expected_dim,
            field_name=f"frame {frame_index} action_positions_rad",
        )
        normalized.append(copied)
    return normalized


def load_replay_episode(
    episode_path: str | Path,
    *,
    expected_joint_names: list[str],
    start_frame: int = 0,
    end_frame: int | None = None,
) -> ReplayEpisode:
    """Load and validate a native episode or frame segment for replay."""
    frames_path = resolve_frames_path(episode_path)
    rows = validate_rows(read_jsonl(frames_path), expected_joint_names=expected_joint_names)
    if start_frame < 0:
        raise ValueError("start_frame must be >= 0")
    if start_frame >= len(rows):
        raise ValueError(f"start_frame {start_frame} is outside episode with {len(rows)} frames")
    if end_frame is None:
        end_frame = len(rows) - 1
    if end_frame < start_frame:
        raise ValueError("end_frame must be >= start_frame")
    if end_frame >= len(rows):
        raise ValueError(f"end_frame {end_frame} is outside episode with {len(rows)} frames")

    selected = rows[start_frame : end_frame + 1]
    return ReplayEpisode(
        frames_path=frames_path,
        fps=infer_episode_fps(frames_path),
        joint_names=list(expected_joint_names),
        rows=selected,
        start_pose=list(selected[0]["state_positions_rad"]),
        trajectory=[list(row["action_positions_rad"]) for row in selected],
        start_frame=int(selected[0]["frame_index"]),
        end_frame=int(selected[-1]["frame_index"]),
    )


def parse_prepare_duration(value: str) -> float | None:
    """Parse prepare-duration-s, where 'auto' means delta-limited duration."""
    if value.strip().lower() == "auto":
        return None
    duration = float(value)
    if duration < 0:
        raise ValueError("prepare-duration-s must be non-negative or auto")
    return duration


def make_interpolation(
    current: list[float],
    target: list[float],
    *,
    rate_hz: float,
    max_delta_rad: float,
    duration_s: float | None,
) -> list[list[float]]:
    """Build a joint-space interpolation from current to target, excluding current."""
    if rate_hz <= 0:
        raise ValueError("rate_hz must be > 0")
    if len(current) != len(target):
        raise ValueError("current and target dimensions differ")

    if duration_s is None:
        if max_delta_rad <= 0:
            raise ValueError("prepare-max-delta-rad must be > 0 when duration is auto")
        steps = max(1, math.ceil(max_abs_delta(current, target) / max_delta_rad))
    else:
        steps = max(1, math.ceil(duration_s * rate_hz))

    return [
        [
            float(start) + (float(end) - float(start)) * (step / steps)
            for start, end in zip(current, target, strict=True)
        ]
        for step in range(1, steps + 1)
    ]


def action_packet(sequence_id: int, joint_names: list[str], targets: list[float]) -> pb2.ActionPacket:
    """Create a local action packet for shared safety validation."""
    return pb2.ActionPacket(
        sequence_id=int(sequence_id),
        timestamp_ns=now_ns(),
        joint_names=list(joint_names),
        joint_targets=[float(value) for value in targets],
    )


def validate_target(
    *,
    targets: list[float],
    current_positions: list[float],
    joint_names: list[str],
    action_cfg: dict[str, Any],
    safety_cfg: dict[str, Any],
    sequence_id: int = 0,
) -> list[float]:
    """Validate one replay target against a simulated or real current joint state."""
    return validate_action_packet(
        action_packet(sequence_id, joint_names, targets),
        expected_joint_names=joint_names,
        current_joints=JointSnapshot(joint_names, current_positions, timestamp_ns=now_ns()),
        action_cfg=action_cfg,
        safety_cfg=safety_cfg,
    )


def publish_joint_targets(
    *,
    publisher: Any,
    joint_state_msg_cls: Any,
    node: Any,
    joint_names: list[str],
    targets: list[float],
    actuate: bool,
) -> bool:
    """Publish a JointState target when actuate is true."""
    if not actuate:
        return False
    message = joint_state_msg_cls()
    message.header.stamp = node.get_clock().now().to_msg()
    message.name = list(joint_names)
    message.position = [float(value) for value in targets]
    publisher.publish(message)
    return True


class SO101EpisodeReplayer:
    """ROS2 runtime for dry-run or actuated native episode replay."""

    def __init__(self, config: dict[str, Any], episode: ReplayEpisode, args: argparse.Namespace) -> None:
        self.config = config
        self.episode = episode
        self.args = args
        self.ros_cfg = section(config, "ros")
        self.sensor_cfg = section(config, "sensor")
        self.action_cfg = section(config, "actions")
        self.safety_cfg = section(config, "safety")
        self.joint_names = [str(name) for name in self.sensor_cfg["joint_names"]]
        self.actuate = bool(args.actuate)
        self._lock = threading.Lock()
        self._joints: JointSnapshot | None = None
        self._node = None
        self._publisher = None
        self._joint_state_msg_cls = None
        self._sequence_id = 1

    def _joint_callback(self, msg: Any) -> None:
        snapshot = JointSnapshot(
            names=[str(name) for name in msg.name],
            positions=[float(value) for value in msg.position],
            timestamp_ns=now_ns(),
        )
        with self._lock:
            self._joints = snapshot

    def _sleep_with_spin(self, duration_s: float) -> None:
        import rclpy

        end_time = time.monotonic() + max(0.0, duration_s)
        while time.monotonic() < end_time:
            timeout = min(0.05, max(0.0, end_time - time.monotonic()))
            rclpy.spin_once(self._node, timeout_sec=timeout)

    def _latest_joints(self) -> JointSnapshot:
        with self._lock:
            joints = self._joints
        if joints is None:
            raise RuntimeError("joint state not received yet")
        stale_timeout_ns = int(float(self.sensor_cfg.get("stale_timeout_s", 1.0)) * 1e9)
        if now_ns() - joints.timestamp_ns > stale_timeout_ns:
            raise RuntimeError("joint state is stale")
        return joints

    def _ordered_latest_positions(self) -> list[float]:
        joints = self._latest_joints()
        if len(joints.names) != len(joints.positions):
            raise ValueError("joint state has mismatched name/position lengths")
        by_name = dict(zip(joints.names, joints.positions, strict=True))
        missing = [name for name in self.joint_names if name not in by_name]
        if missing:
            raise ValueError(f"joint state missing joints: {', '.join(missing)}")
        return [float(by_name[name]) for name in self.joint_names]

    def _wait_for_joint_state(self, timeout_s: float) -> list[float]:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                return self._ordered_latest_positions()
            except RuntimeError:
                self._sleep_with_spin(0.05)
        raise TimeoutError(f"joint state not received within {timeout_s:.1f}s")

    def _next_sequence_id(self) -> int:
        sequence_id = self._sequence_id
        self._sequence_id += 1
        return sequence_id

    def _validate_and_publish(self, targets: list[float], *, label: str) -> None:
        current = self._ordered_latest_positions()
        sequence_id = self._next_sequence_id()
        validate_target(
            targets=targets,
            current_positions=current,
            joint_names=self.joint_names,
            action_cfg=self.action_cfg,
            safety_cfg=self.safety_cfg,
            sequence_id=sequence_id,
        )
        publish_joint_targets(
            publisher=self._publisher,
            joint_state_msg_cls=self._joint_state_msg_cls,
            node=self._node,
            joint_names=self.joint_names,
            targets=targets,
            actuate=self.actuate,
        )
        print(
            f"{'Published' if self.actuate else 'DRY-RUN'} {label}",
            f"sequence_id={sequence_id}",
            f"max_delta={max_abs_delta(current, targets):.6f}",
            f"targets={[round(float(value), 5) for value in targets]}",
            flush=True,
        )

    def _validate_dry_run_sequence(self, current: list[float], targets: list[list[float]], *, label: str) -> list[float]:
        simulated = list(current)
        for index, target in enumerate(targets):
            validate_target(
                targets=target,
                current_positions=simulated,
                joint_names=self.joint_names,
                action_cfg=self.action_cfg,
                safety_cfg=self.safety_cfg,
                sequence_id=index + 1,
            )
            print(
                f"DRY-RUN {label}",
                f"index={index}",
                f"max_delta={max_abs_delta(simulated, target):.6f}",
                flush=True,
            )
            simulated = list(target)
        return simulated

    def _execute_sequence(self, targets: list[list[float]], *, rate_hz: float, label: str) -> None:
        if rate_hz <= 0:
            raise ValueError("execution rate must be > 0")
        publish_limit_hz = float(self.action_cfg.get("publish_rate_limit_hz", 0.0))
        if self.actuate and publish_limit_hz > 0 and rate_hz > publish_limit_hz + 1e-9:
            raise ValueError(f"{label} rate {rate_hz:.3f}Hz exceeds publish limit {publish_limit_hz:.3f}Hz")

        interval_s = 1.0 / rate_hz
        for index, target in enumerate(targets):
            self._validate_and_publish(target, label=f"{label}[{index}]")
            self._sleep_with_spin(interval_s)

    def _wait_for_start_tolerance(self) -> None:
        deadline = time.monotonic() + float(self.args.start_settle_timeout_s)
        tolerance = float(self.args.start_tolerance_rad)
        while time.monotonic() < deadline:
            current = self._ordered_latest_positions()
            error = max_abs_delta(current, self.episode.start_pose)
            if error <= tolerance:
                print(f"Start pose reached: max_error={error:.6f}", flush=True)
                return
            self._sleep_with_spin(0.05)
        current = self._ordered_latest_positions()
        error = max_abs_delta(current, self.episode.start_pose)
        raise RuntimeError(f"start pose tolerance failed: max_error={error:.6f} > {tolerance:.6f}")

    def _hold_final_target(self, final_target: list[float], replay_rate_hz: float) -> None:
        hold_s = float(self.args.hold_final_s)
        if hold_s <= 0:
            return
        hold_rate_hz = min(max(1.0, replay_rate_hz), float(self.action_cfg.get("publish_rate_limit_hz", 10.0)))
        steps = max(1, math.ceil(hold_s * hold_rate_hz))
        self._execute_sequence([final_target] * steps, rate_hz=hold_rate_hz, label="hold")

    def _dry_run(self, current: list[float], prepare_targets: list[list[float]], replay_rate_hz: float) -> int:
        print("Replay dry-run summary")
        print(f"episode={self.episode.frames_path.parent}")
        print(f"frames={len(self.episode.trajectory)} start_frame={self.episode.start_frame} end_frame={self.episode.end_frame}")
        print(f"episode_fps={self.episode.fps:.3f} replay_rate_hz={replay_rate_hz:.3f}")
        print(f"current={[round(value, 5) for value in current]}")
        print(f"episode_t0={[round(value, 5) for value in self.episode.start_pose]}")
        print(f"preposition_max_delta={max_abs_delta(current, self.episode.start_pose):.6f}")
        simulated = self._validate_dry_run_sequence(current, prepare_targets, label="prepare")
        self._validate_dry_run_sequence(simulated, self.episode.trajectory, label="replay")
        print("Dry-run complete. No /follower/joint_targets messages were published.")
        return 0

    def run(self) -> int:
        import rclpy
        from rclpy.executors import ExternalShutdownException
        from sensor_msgs.msg import JointState

        if self.actuate and not bool(self.action_cfg.get("actuation_enabled", False)):
            raise RuntimeError("--actuate requires actions.actuation_enabled=true in the config")

        self._joint_state_msg_cls = JointState
        rclpy.init()
        qos_depth = int(self.ros_cfg.get("qos_depth", 5))
        self._node = rclpy.create_node("so101_episode_replayer")
        self._node.create_subscription(JointState, str(self.ros_cfg["joint_states_topic"]), self._joint_callback, qos_depth)
        self._publisher = self._node.create_publisher(JointState, str(self.ros_cfg["joint_targets_topic"]), qos_depth)

        try:
            current = self._wait_for_joint_state(float(self.args.joint_state_timeout_s))
            prepare_duration_s = parse_prepare_duration(str(self.args.prepare_duration_s))
            prepare_targets = make_interpolation(
                current,
                self.episode.start_pose,
                rate_hz=float(self.args.prepare_rate_hz),
                max_delta_rad=float(self.args.prepare_max_delta_rad),
                duration_s=prepare_duration_s,
            )
            replay_rate_hz = self.episode.fps * float(self.args.rate_scale)
            if replay_rate_hz <= 0:
                raise ValueError("rate-scale must produce a positive replay rate")

            if not self.actuate:
                return self._dry_run(current, prepare_targets, replay_rate_hz)

            print("Starting actuated replay", flush=True)
            self._execute_sequence(prepare_targets, rate_hz=float(self.args.prepare_rate_hz), label="prepare")
            self._wait_for_start_tolerance()
            self._execute_sequence(self.episode.trajectory, rate_hz=replay_rate_hz, label="replay")
            self._hold_final_target(self.episode.trajectory[-1], replay_rate_hz)
            print("Replay complete.", flush=True)
            return 0
        except (KeyboardInterrupt, ExternalShutdownException):
            return 130
        finally:
            if self._node is not None:
                self._node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode", required=True, help="Episode directory or frames.jsonl path.")
    parser.add_argument("--config", default="configs/so101_gateway.yaml")
    parser.add_argument("--actuate", action="store_true", help="Actually publish /follower/joint_targets.")
    parser.add_argument("--dry-run", action="store_true", help="Explicitly keep dry-run mode; this is the default.")
    parser.add_argument("--prepare-rate-hz", type=float, default=10.0)
    parser.add_argument("--prepare-duration-s", default="auto")
    parser.add_argument("--prepare-max-delta-rad", type=float, default=0.03)
    parser.add_argument("--start-tolerance-rad", type=float, default=0.05)
    parser.add_argument("--start-settle-timeout-s", type=float, default=3.0)
    parser.add_argument("--hold-final-s", type=float, default=1.0)
    parser.add_argument("--rate-scale", type=float, default=1.0)
    parser.add_argument("--start-frame", type=int, default=0)
    parser.add_argument("--end-frame", type=int)
    parser.add_argument("--joint-state-timeout-s", type=float, default=5.0)
    args = parser.parse_args()
    if args.actuate and args.dry_run:
        parser.error("--actuate and --dry-run are mutually exclusive")
    return args


def main() -> int:
    """Run SO101 episode replay."""
    args = parse_args()
    config = load_yaml(args.config)
    joint_names = [str(name) for name in section(config, "sensor")["joint_names"]]
    try:
        episode = load_replay_episode(
            args.episode,
            expected_joint_names=joint_names,
            start_frame=int(args.start_frame),
            end_frame=args.end_frame,
        )
        return SO101EpisodeReplayer(config, episode, args).run()
    except Exception as exc:  # noqa: BLE001
        print(f"Replay failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
