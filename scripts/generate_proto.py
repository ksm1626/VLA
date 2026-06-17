#!/usr/bin/env python3
"""Generate Python gRPC modules for project proto files."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path


def main() -> int:
    """Run grpc_tools.protoc for the SO101 remote bridge proto."""
    project_root = Path(__file__).resolve().parents[1]
    proto_dir = project_root / "proto"
    output_dir = proto_dir / "generated"
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "__init__.py").touch()

    command = [
        sys.executable,
        "-m",
        "grpc_tools.protoc",
        f"--proto_path={proto_dir}",
        f"--python_out={output_dir}",
        f"--grpc_python_out={output_dir}",
        str(proto_dir / "so101_remote.proto"),
    ]
    print(" ".join(command))
    return subprocess.run(command, check=False).returncode


if __name__ == "__main__":
    raise SystemExit(main())
