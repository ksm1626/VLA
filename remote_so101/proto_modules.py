"""Import helpers for generated SO101 gRPC modules."""

from __future__ import annotations

import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
GENERATED_PROTO_DIR = PROJECT_ROOT / "proto" / "generated"
if str(GENERATED_PROTO_DIR) not in sys.path:
    sys.path.insert(0, str(GENERATED_PROTO_DIR))

import so101_remote_pb2 as pb2  # noqa: E402
import so101_remote_pb2_grpc as pb2_grpc  # noqa: E402

__all__ = ["pb2", "pb2_grpc"]
