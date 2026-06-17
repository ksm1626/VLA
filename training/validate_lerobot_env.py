#!/usr/bin/env python3
"""Validate the local environment for LeRobot SmolVLA policy work."""

from __future__ import annotations

import importlib
import subprocess
import shutil
import sys
from pathlib import Path

from lerobot_config import env_executable


def check_module(module_name: str) -> bool:
    """Print whether a Python module can actually be imported."""
    try:
        importlib.import_module(module_name)
    except Exception as exc:
        print(f"FAIL: python module `{module_name}` import failed: {exc}")
        return False

    print(f"OK: python module `{module_name}`")
    return True


def check_command(command_name: str) -> bool:
    """Print whether an executable is on PATH and can show help."""
    path = env_executable(command_name)
    if not Path(path).exists():
        path = shutil.which(command_name)
    if not path:
        print(f"MISSING: command `{command_name}`")
        return False

    try:
        subprocess.run(
            [path, "--help"],
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
            timeout=30,
        )
    except subprocess.CalledProcessError as exc:
        detail = (exc.stderr or "").strip().splitlines()
        last_line = detail[-1] if detail else "no error output"
        print(f"FAIL: command `{command_name}` help failed: {last_line}")
        return False
    except subprocess.TimeoutExpired:
        print(f"FAIL: command `{command_name}` help timed out")
        return False

    print(f"OK: command `{command_name}` at {path}")
    return True


def check_cuda_warning() -> None:
    """Report CUDA availability if PyTorch is installed."""
    try:
        torch = importlib.import_module("torch")
    except ImportError:
        print("WARN: python module `torch` is not installed; CUDA availability not checked")
        return

    if torch.cuda.is_available():
        print(f"OK: CUDA available ({torch.cuda.device_count()} device(s))")
    else:
        print("WARN: CUDA is not available; CPU-only validation still passed")


def main() -> int:
    """Validate LeRobot imports and CLI entry points."""
    checks = [
        check_module("lerobot"),
        check_module("lerobot.async_inference.policy_server"),
        check_command("lerobot-train"),
    ]
    check_cuda_warning()

    if all(checks):
        print("Environment validation passed.")
        return 0

    print("Environment validation failed. Install LeRobot with SmolVLA/async extras.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
