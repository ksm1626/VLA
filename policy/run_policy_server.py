#!/usr/bin/env python3
"""Build and run the official LeRobot async PolicyServer command."""

from __future__ import annotations

import argparse
import os
import shlex
import subprocess
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "training"))

from lerobot_config import load_yaml_config, require_mapping, require_value  # noqa: E402


def build_command(config_path: str | Path) -> list[str]:
    """Build a LeRobot async PolicyServer command from project config."""
    config = load_yaml_config(config_path)
    server = require_mapping(config, "server")
    host = require_value(server, "host", "server")
    port = require_value(server, "port", "server")

    return [
        sys.executable,
        "-m",
        "lerobot.async_inference.policy_server",
        f"--host={host}",
        f"--port={port}",
    ]


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        default="configs/policy_server.smolvla.yaml",
        help="Path to the SmolVLA policy server YAML config.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the LeRobot command without running it.",
    )
    return parser.parse_args()


def main() -> int:
    """Run or print the generated LeRobot PolicyServer command."""
    args = parse_args()
    command = build_command(args.config)
    env = os.environ.copy()
    env["PYTHONNOUSERSITE"] = "1"

    print("PYTHONNOUSERSITE=1 " + shlex.join(command))
    if args.dry_run:
        return 0

    completed = subprocess.run(command, check=False, env=env)
    return completed.returncode


if __name__ == "__main__":
    raise SystemExit(main())
