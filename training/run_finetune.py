#!/usr/bin/env python3
"""Build and run the official LeRobot SmolVLA fine-tuning command."""

from __future__ import annotations

import argparse
import json
import os
import shlex
import subprocess
from pathlib import Path
from typing import Any

from lerobot_config import (
    env_executable,
    load_yaml_config,
    optional_bool,
    require_mapping,
    require_value,
)


def build_command(config: dict[str, Any]) -> list[str]:
    """Build a `lerobot-train` command from project config."""
    policy = require_mapping(config, "policy")
    dataset = require_mapping(config, "dataset")
    training = require_mapping(config, "training")

    repo_id = dataset.get("repo_id")
    root = dataset.get("root")
    if bool(repo_id) == bool(root):
        raise ValueError("Set exactly one of `dataset.repo_id` or `dataset.root`")

    command = [
        env_executable("lerobot-train"),
        f"--policy.path={require_value(policy, 'path', 'policy')}",
        f"--policy.device={require_value(policy, 'device', 'policy')}",
        f"--policy.push_to_hub={optional_bool(policy.get('push_to_hub', False))}",
        f"--output_dir={require_value(training, 'output_dir', 'training')}",
        f"--job_name={require_value(training, 'job_name', 'training')}",
        f"--batch_size={require_value(training, 'batch_size', 'training')}",
        f"--steps={require_value(training, 'steps', 'training')}",
        f"--wandb.enable={optional_bool(training.get('wandb_enable', False))}",
    ]

    if repo_id:
        command.append(f"--dataset.repo_id={repo_id}")
    if root:
        command.append(f"--dataset.root={root}")

    policy_repo_id = policy.get("repo_id")
    if policy_repo_id:
        command.append(f"--policy.repo_id={policy_repo_id}")

    empty_cameras = policy.get("empty_cameras")
    if empty_cameras is not None:
        command.append(f"--policy.empty_cameras={empty_cameras}")

    rename_map = dataset.get("rename_map") or {}
    if not isinstance(rename_map, dict):
        raise ValueError("`dataset.rename_map` must be a mapping")
    if rename_map:
        command.append(f"--rename_map={json.dumps(rename_map, sort_keys=True)}")

    extra_args = training.get("extra_args") or []
    if not isinstance(extra_args, list):
        raise ValueError("`training.extra_args` must be a list")
    command.extend(str(arg) for arg in extra_args)

    return command


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        default="configs/finetune.smolvla.yaml",
        help="Path to the SmolVLA fine-tuning YAML config.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the LeRobot command without running it.",
    )
    return parser.parse_args()


def main() -> int:
    """Run or print the generated LeRobot fine-tuning command."""
    args = parse_args()
    config = load_yaml_config(Path(args.config))
    command = build_command(config)
    env = os.environ.copy()
    env["PYTHONNOUSERSITE"] = "1"

    print("PYTHONNOUSERSITE=1 " + shlex.join(command))
    if args.dry_run:
        return 0

    completed = subprocess.run(command, check=False, env=env)
    return completed.returncode


if __name__ == "__main__":
    raise SystemExit(main())
