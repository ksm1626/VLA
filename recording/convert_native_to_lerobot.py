#!/usr/bin/env python3
"""Convert SO101 native MP4 + JSONL datasets into LeRobotDataset format."""

from __future__ import annotations

import argparse
import json
import math
import shutil
import sys
from pathlib import Path
from typing import Any

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from remote_so101.config_loader import load_yaml  # noqa: E402
from recording.validate_native_dataset import read_json, read_jsonl, validate_dataset  # noqa: E402


ARM_JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
GRIPPER_JOINT_NAME = "gripper"


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def ros_gripper_to_policy(value: float, export_cfg: dict[str, Any]) -> float:
    closed = float(export_cfg.get("gripper_ros_closed", 0.0))
    open_ = float(export_cfg.get("gripper_ros_open", 0.8))
    policy_min = float(export_cfg.get("gripper_policy_min", 0.0))
    policy_max = float(export_cfg.get("gripper_policy_max", 33.0))
    ros_min = float(export_cfg.get("gripper_ros_min", -0.174533))
    ros_max = float(export_cfg.get("gripper_ros_max", 1.74533))
    value = clamp(float(value), ros_min, ros_max)
    if open_ == closed:
        raise ValueError("gripper_ros_closed and gripper_ros_open must differ")
    ratio = (value - closed) / (open_ - closed)
    return clamp(policy_min + ratio * (policy_max - policy_min), policy_min, policy_max)


def ros_vector_to_policy(values: list[float], joint_names: list[str], export_cfg: dict[str, Any]) -> np.ndarray:
    if len(values) != len(joint_names):
        raise ValueError("joint vector length mismatch")
    output = []
    for name, value in zip(joint_names, values, strict=True):
        value = float(value)
        if name in ARM_JOINT_NAMES:
            output.append(math.degrees(value))
        elif name == GRIPPER_JOINT_NAME:
            output.append(ros_gripper_to_policy(value, export_cfg))
        else:
            output.append(value)
    return np.asarray(output, dtype=np.float32)


def resize_rgb(image_rgb: np.ndarray, width: int, height: int) -> np.ndarray:
    if image_rgb.shape[1] == width and image_rgb.shape[0] == height:
        return np.ascontiguousarray(image_rgb)

    import cv2

    return cv2.resize(image_rgb, (width, height), interpolation=cv2.INTER_AREA)


def read_video_frames(path: Path, *, width: int, height: int) -> list[np.ndarray]:
    import cv2

    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise ValueError(f"failed to open video: {path}")
    frames = []
    try:
        while True:
            ok, image_bgr = cap.read()
            if not ok:
                break
            image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
            frames.append(resize_rgb(image_rgb, width, height))
    finally:
        cap.release()
    if not frames:
        raise ValueError(f"video has no readable frames: {path}")
    return frames


def make_lerobot_features(*, joint_names: list[str], width: int, height: int) -> dict[str, dict[str, Any]]:
    return {
        "observation.state": {
            "dtype": "float32",
            "shape": (len(joint_names),),
            "names": list(joint_names),
        },
        "observation.images.camera1": {
            "dtype": "video",
            "shape": (height, width, 3),
            "names": ["height", "width", "channel"],
        },
        "observation.images.camera2": {
            "dtype": "video",
            "shape": (height, width, 3),
            "names": ["height", "width", "channel"],
        },
        "observation.images.camera3": {
            "dtype": "video",
            "shape": (height, width, 3),
            "names": ["height", "width", "channel"],
        },
        "action": {
            "dtype": "float32",
            "shape": (len(joint_names),),
            "names": list(joint_names),
        },
    }


def iter_episode_dirs(root: Path) -> list[Path]:
    return sorted(path for path in (root / "episodes").glob("episode_*") if path.is_dir() and not path.name.endswith(".tmp"))


def convert_dataset(
    *,
    input_root: Path,
    output_root: Path,
    repo_id: str,
    overwrite: bool,
    video_backend: str | None,
    vcodec: str,
) -> None:
    validate_dataset(input_root)
    dataset_meta = load_yaml(input_root / "dataset.yaml")
    joint_names = list(dataset_meta["joint_names"])
    export_cfg = dataset_meta.get("export") or {}
    width = int(export_cfg.get("image_width", 256))
    height = int(export_cfg.get("image_height", 256))
    fps = int(dataset_meta.get("fps", 10))

    if output_root.exists() and any(output_root.iterdir()):
        if not overwrite:
            raise FileExistsError(f"output root is not empty: {output_root} (pass --overwrite to replace it)")
        shutil.rmtree(output_root)

    try:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
    except Exception as exc:  # noqa: BLE001
        raise RuntimeError("LeRobot is required on A6000 to convert native datasets") from exc

    dataset = LeRobotDataset.create(
        repo_id=repo_id,
        fps=fps,
        features=make_lerobot_features(joint_names=joint_names, width=width, height=height),
        root=output_root,
        robot_type="so101",
        use_videos=True,
        image_writer_processes=0,
        image_writer_threads=0,
        video_backend=video_backend,
        vcodec=vcodec,
    )

    empty_camera = np.zeros((height, width, 3), dtype=np.uint8)
    for episode_dir in iter_episode_dirs(input_root):
        episode = read_json(episode_dir / "episode.json")
        rows = read_jsonl(episode_dir / "frames.jsonl")
        front_frames = read_video_frames(episode_dir / "videos" / "front.mp4", width=width, height=height)
        top_frames = read_video_frames(episode_dir / "videos" / "top.mp4", width=width, height=height)
        if len(front_frames) != len(rows) or len(top_frames) != len(rows):
            raise ValueError(f"{episode_dir}: video frame count does not match JSONL rows")

        task = str(episode.get("task") or rows[0].get("task") or "perform the task")
        for index, row in enumerate(rows):
            state = ros_vector_to_policy(row["state_positions_rad"], joint_names, export_cfg)
            action = ros_vector_to_policy(row["action_positions_rad"], joint_names, export_cfg)
            dataset.add_frame(
                {
                    "observation.state": state,
                    "observation.images.camera1": front_frames[index],
                    "observation.images.camera2": top_frames[index],
                    "observation.images.camera3": empty_camera,
                    "action": action,
                    "task": task,
                }
            )
        dataset.save_episode()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, help="Native dataset root")
    parser.add_argument("--output", required=True, help="LeRobotDataset output root")
    parser.add_argument("--repo-id", required=True, help="Local or Hub repo id, e.g. local/so101_pickplace_v1")
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--video-backend", default=None)
    parser.add_argument("--vcodec", default="h264")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        convert_dataset(
            input_root=Path(args.input).expanduser().resolve(),
            output_root=Path(args.output).expanduser().resolve(),
            repo_id=args.repo_id,
            overwrite=bool(args.overwrite),
            video_backend=args.video_backend,
            vcodec=str(args.vcodec),
        )
    except Exception as exc:  # noqa: BLE001
        print(f"CONVERSION FAILED: {exc}", file=sys.stderr)
        return 1
    print(f"Converted native dataset to LeRobotDataset: {Path(args.output).expanduser().resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
