#!/usr/bin/env python3
"""Validate SO101 native MP4 + JSONL teleop datasets."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from remote_so101.config_loader import load_yaml  # noqa: E402


def read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    rows = []
    with path.open("r", encoding="utf-8") as f:
        for line_no, line in enumerate(f, start=1):
            stripped = line.strip()
            if not stripped:
                continue
            try:
                rows.append(json.loads(stripped))
            except json.JSONDecodeError as exc:
                raise ValueError(f"{path}:{line_no}: invalid JSONL: {exc}") from exc
    return rows


def video_frame_count(path: Path) -> int:
    import cv2

    cap = cv2.VideoCapture(str(path))
    try:
        if not cap.isOpened():
            raise ValueError(f"failed to open video: {path}")
        count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        if count <= 0:
            raise ValueError(f"video has no readable frames: {path}")
        return count
    finally:
        cap.release()


def validate_vector(values: Any, *, expected_dim: int, context: str) -> None:
    if not isinstance(values, list) or len(values) != expected_dim:
        raise ValueError(f"{context} must be a list of length {expected_dim}")
    if not all(isinstance(value, (int, float)) and math.isfinite(float(value)) for value in values):
        raise ValueError(f"{context} contains NaN, Inf, or non-numeric values")


def validate_episode(episode_dir: Path, dataset_meta: dict[str, Any]) -> dict[str, Any]:
    episode_json = episode_dir / "episode.json"
    frames_jsonl = episode_dir / "frames.jsonl"
    front_video = episode_dir / "videos" / "front.mp4"
    top_video = episode_dir / "videos" / "top.mp4"
    for path in [episode_json, frames_jsonl, front_video, top_video]:
        if not path.exists():
            raise ValueError(f"missing required file: {path}")

    episode = read_json(episode_json)
    frames = read_jsonl(frames_jsonl)
    joint_names = list(dataset_meta["joint_names"])
    expected_dim = len(joint_names)

    if int(episode.get("num_frames", -1)) != len(frames):
        raise ValueError(f"{episode_dir}: episode num_frames does not match frames.jsonl")
    if len(frames) == 0:
        raise ValueError(f"{episode_dir}: episode has zero frames")

    front_count = video_frame_count(front_video)
    top_count = video_frame_count(top_video)
    if front_count != len(frames):
        raise ValueError(f"{episode_dir}: front video frame count {front_count} != {len(frames)} JSONL rows")
    if top_count != len(frames):
        raise ValueError(f"{episode_dir}: top video frame count {top_count} != {len(frames)} JSONL rows")

    for expected_index, row in enumerate(frames):
        if int(row.get("frame_index", -1)) != expected_index:
            raise ValueError(f"{episode_dir}: non-contiguous frame_index at row {expected_index}")
        if list(row.get("joint_names", [])) != joint_names:
            raise ValueError(f"{episode_dir}: joint_names mismatch at frame {expected_index}")
        if list(row.get("action_joint_names", [])) != joint_names:
            raise ValueError(f"{episode_dir}: action_joint_names mismatch at frame {expected_index}")
        validate_vector(row.get("state_positions_rad"), expected_dim=expected_dim, context="state_positions_rad")
        validate_vector(row.get("action_positions_rad"), expected_dim=expected_dim, context="action_positions_rad")

    return {
        "episode": episode_dir.name,
        "frames": len(frames),
        "task": episode.get("task"),
        "teleop_source": episode.get("teleop_source"),
    }


def validate_dataset(root: Path) -> list[dict[str, Any]]:
    dataset_yaml = root / "dataset.yaml"
    episodes_dir = root / "episodes"
    if not dataset_yaml.exists():
        raise ValueError(f"missing required file: {dataset_yaml}")
    if not episodes_dir.exists():
        raise ValueError(f"missing required directory: {episodes_dir}")

    dataset_meta = load_yaml(dataset_yaml)
    if dataset_meta.get("robot") != "so101":
        raise ValueError(f"expected robot=so101, got {dataset_meta.get('robot')!r}")
    if dataset_meta.get("action_type") != "absolute_joint_position":
        raise ValueError("expected action_type=absolute_joint_position")
    if dataset_meta.get("state_unit") != "radian" or dataset_meta.get("action_unit") != "radian":
        raise ValueError("expected state_unit/action_unit to be radian")
    if not dataset_meta.get("joint_names"):
        raise ValueError("dataset.yaml joint_names must be non-empty")

    episode_dirs = sorted(path for path in episodes_dir.glob("episode_*") if path.is_dir() and not path.name.endswith(".tmp"))
    if not episode_dirs:
        raise ValueError(f"no finalized episodes found under {episodes_dir}")
    return [validate_episode(path, dataset_meta) for path in episode_dirs]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", required=True, help="Native dataset root, e.g. datasets/native/so101_pickplace_v1")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    root = Path(args.root).expanduser().resolve()
    try:
        results = validate_dataset(root)
    except Exception as exc:  # noqa: BLE001
        print(f"INVALID: {exc}", file=sys.stderr)
        return 1

    total_frames = sum(int(result["frames"]) for result in results)
    print(f"VALID native dataset: {root}")
    print(f"Episodes: {len(results)}")
    print(f"Frames: {total_frames}")
    for result in results:
        print(
            f"- {result['episode']}: frames={result['frames']} "
            f"teleop_source={result['teleop_source']} task={result['task']}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
