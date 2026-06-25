#!/usr/bin/env python3
"""Build and run official LeRobot fine-tuning commands from project YAML configs."""

from __future__ import annotations

import argparse
import json
import os
import shlex
import subprocess
from pathlib import Path
from typing import Any

try:
    from lerobot_config import (
        env_executable,
        load_yaml_config,
        optional_bool,
        require_mapping,
        require_value,
    )
except ImportError:  # pragma: no cover - used when imported as `training.run_finetune`
    from training.lerobot_config import (
        env_executable,
        load_yaml_config,
        optional_bool,
        require_mapping,
        require_value,
    )


POLICY_SELECTOR_KEYS = {"path", "type", "pretrained_path"}
POLICY_STANDARD_KEYS = {
    *POLICY_SELECTOR_KEYS,
    "device",
    "push_to_hub",
    "repo_id",
    "options",
}


def _format_cli_value(value: Any) -> str:
    """Format a Python config value for a draccus/LeRobot CLI override."""
    if isinstance(value, bool):
        return optional_bool(value)
    if isinstance(value, (dict, list)):
        return json.dumps(value, sort_keys=True)
    return str(value)


def _append_policy_options(command: list[str], policy: dict[str, Any]) -> None:
    """Append generic `--policy.*` overrides while preventing duplicate keys."""
    nested_options = policy.get("options") or {}
    if not isinstance(nested_options, dict):
        raise ValueError("`policy.options` must be a mapping")

    direct_options = {
        key: value
        for key, value in policy.items()
        if key not in POLICY_STANDARD_KEYS and value is not None
    }
    duplicate_keys = sorted(set(direct_options).intersection(nested_options))
    if duplicate_keys:
        raise ValueError(
            "`policy.options` duplicates top-level policy keys: " + ", ".join(duplicate_keys)
        )

    for key, value in {**direct_options, **nested_options}.items():
        if value is None:
            continue
        command.append(f"--policy.{key}={_format_cli_value(value)}")


def _append_policy_selector(command: list[str], policy: dict[str, Any]) -> None:
    """Append either the legacy `policy.path` selector or generic type/pretrained_path."""
    policy_path = policy.get("path")
    policy_type = policy.get("type")
    pretrained_path = policy.get("pretrained_path")

    if policy_path and (policy_type or pretrained_path):
        raise ValueError(
            "Use either `policy.path` or `policy.type` + `policy.pretrained_path`, not both"
        )
    if policy_path:
        command.append(f"--policy.path={policy_path}")
        return

    command.append(f"--policy.type={require_value(policy, 'type', 'policy')}")
    command.append(f"--policy.pretrained_path={require_value(policy, 'pretrained_path', 'policy')}")


def build_command(config: dict[str, Any]) -> list[str]:
    """Build a `lerobot-train` command from project config."""
    policy = require_mapping(config, "policy")
    dataset = require_mapping(config, "dataset")
    training = require_mapping(config, "training")

    repo_id = dataset.get("repo_id")
    root = dataset.get("root")
    if not repo_id:
        raise ValueError("Set `dataset.repo_id`; LeRobot requires it even for local datasets")

    command = [env_executable("lerobot-train")]
    _append_policy_selector(command, policy)

    if policy.get("device") is not None:
        command.append(f"--policy.device={policy['device']}")
    command.extend(
        [
            f"--policy.push_to_hub={optional_bool(policy.get('push_to_hub', False))}",
        f"--output_dir={require_value(training, 'output_dir', 'training')}",
        f"--job_name={require_value(training, 'job_name', 'training')}",
        f"--batch_size={require_value(training, 'batch_size', 'training')}",
        f"--steps={require_value(training, 'steps', 'training')}",
        f"--wandb.enable={optional_bool(training.get('wandb_enable', False))}",
        ]
    )

    command.append(f"--dataset.repo_id={repo_id}")
    if root:
        command.append(f"--dataset.root={root}")

    policy_repo_id = policy.get("repo_id")
    if policy_repo_id:
        command.append(f"--policy.repo_id={policy_repo_id}")

    _append_policy_options(command, policy)

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
        help="Path to the LeRobot fine-tuning YAML config.",
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
