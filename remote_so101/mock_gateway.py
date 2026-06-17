#!/usr/bin/env python3
"""A6000-only mock SO101 gateway for gRPC and official async validation."""

from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from pathlib import Path

import grpc
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from remote_so101.config_loader import load_yaml, section  # noqa: E402
from remote_so101.proto_modules import pb2, pb2_grpc  # noqa: E402


def _make_rgb_image(width: int, height: int, sequence_id: int, offset: int) -> np.ndarray:
    x = np.linspace(0, 255, width, dtype=np.uint8)
    y = np.linspace(0, 255, height, dtype=np.uint8)[:, None]
    image = np.zeros((height, width, 3), dtype=np.uint8)
    image[..., 0] = (x[None, :] + sequence_id + offset) % 255
    image[..., 1] = (y + 2 * sequence_id + offset) % 255
    image[..., 2] = (sequence_id * 7 + offset) % 255
    return image


def _encode_image(image_rgb: np.ndarray, encoding: str, jpeg_quality: int) -> bytes:
    encoding = encoding.lower()
    if encoding == "rgb8":
        return image_rgb.tobytes()
    if encoding == "bgr8":
        return image_rgb[..., ::-1].tobytes()
    if encoding in {"jpeg-rgb", "jpg-rgb", "jpeg-bgr", "jpg-bgr"}:
        import cv2

        image_bgr = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
        ok, encoded = cv2.imencode(".jpg", image_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality])
        if not ok:
            raise RuntimeError("Failed to JPEG-encode mock image")
        return encoded.tobytes()
    raise ValueError(f"Unsupported mock image encoding: {encoding}")


def _make_sensor_packet(sensor_cfg: dict, sequence_id: int) -> pb2.SensorPacket:
    width = int(sensor_cfg.get("width", 256))
    height = int(sensor_cfg.get("height", 256))
    encoding = str(sensor_cfg.get("encoding", "jpeg-rgb"))
    jpeg_quality = int(sensor_cfg.get("jpeg_quality", 85))
    front = _make_rgb_image(width, height, sequence_id, offset=3)
    top = _make_rgb_image(width, height, sequence_id, offset=41)
    joint_names = [str(name) for name in sensor_cfg["joint_names"]]
    joint_positions = [float(math.sin(sequence_id * 0.05 + idx * 0.1)) for idx in range(len(joint_names))]
    return pb2.SensorPacket(
        sequence_id=sequence_id,
        timestamp_ns=time.time_ns(),
        instruction=str(sensor_cfg.get("instruction", "Pick up the object")),
        front_image=pb2.ImagePayload(
            data=_encode_image(front, encoding, jpeg_quality),
            width=width,
            height=height,
            encoding=encoding,
        ),
        top_image=pb2.ImagePayload(
            data=_encode_image(top, encoding, jpeg_quality),
            width=width,
            height=height,
            encoding=encoding,
        ),
        joint_names=joint_names,
        joint_positions=joint_positions,
    )


def _stream_actions(
    stub: pb2_grpc.SO101RemoteBridgeStub,
    gateway_id: str,
    stop_after_actions: int,
    stop_event: threading.Event,
) -> None:
    received = 0
    try:
        for action in stub.StreamActions(pb2.GatewayStatus(gateway_id=gateway_id)):
            print(
                "ACTION",
                f"sequence_id={action.sequence_id}",
                f"joints={list(action.joint_names)}",
                f"targets={[round(v, 4) for v in action.joint_targets]}",
            )
            received += 1
            if received >= stop_after_actions:
                stop_event.set()
                return
    except grpc.RpcError as exc:
        if not stop_event.is_set():
            print(f"Action stream stopped: {exc}")


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/mock_gateway.so101.yaml")
    parser.add_argument("--duration-s", type=float, default=None)
    parser.add_argument("--stop-after-actions", type=int, default=None)
    return parser.parse_args()


def main() -> int:
    """Run the mock gateway."""
    args = parse_args()
    config = load_yaml(args.config)
    bridge = section(config, "bridge")
    sensor = section(config, "sensor")
    actions = section(config, "actions")

    duration_s = float(args.duration_s if args.duration_s is not None else sensor.get("duration_s", 20.0))
    stop_after_actions = int(
        args.stop_after_actions
        if args.stop_after_actions is not None
        else actions.get("stop_after_actions", 1)
    )
    target = f"{bridge.get('host', '127.0.0.1')}:{int(bridge.get('port', 49100))}"
    gateway_id = str(bridge.get("gateway_id", "mock_so101"))
    channel = grpc.insecure_channel(target)
    stub = pb2_grpc.SO101RemoteBridgeStub(channel)

    stop_event = threading.Event()
    action_thread = threading.Thread(
        target=_stream_actions,
        args=(stub, gateway_id, stop_after_actions, stop_event),
        daemon=True,
    )
    action_thread.start()

    fps = float(sensor.get("fps", 5))
    interval = 1.0 / fps
    deadline = time.monotonic() + duration_s
    sequence_id = 1
    accepted = 0
    while time.monotonic() < deadline and not stop_event.is_set():
        packet = _make_sensor_packet(sensor, sequence_id)
        reply = stub.PushSensorPacket(packet)
        if not reply.accepted:
            print(f"Sensor packet rejected: {reply.message}")
            return 1
        accepted += 1
        sequence_id += 1
        time.sleep(interval)

    stop_event.set()
    channel.close()
    action_thread.join(timeout=2)
    print(f"Mock gateway complete: sensor_packets={accepted}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
