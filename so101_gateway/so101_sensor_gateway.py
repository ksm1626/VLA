#!/usr/bin/env python3
"""SO101-side ROS2 Gateway for the A6000 RemoteSO101 bridge.

This file intentionally avoids importing ROS2 at module import time so helper
functions can be tested on non-ROS machines. Runtime dependencies on SO101 are
rclpy, sensor_msgs, grpcio, protobuf, numpy, and OpenCV.
"""

from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import grpc
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from remote_so101.config_loader import load_yaml, section  # noqa: E402
from remote_so101.proto_modules import pb2, pb2_grpc  # noqa: E402


@dataclass(frozen=True)
class ImageFrame:
    """RGB image frame ready to encode into a sensor packet."""

    image_rgb: np.ndarray
    timestamp_ns: int


@dataclass(frozen=True)
class EncodedImage:
    """Encoded image payload metadata."""

    data: bytes
    width: int
    height: int
    encoding: str


@dataclass(frozen=True)
class JointSnapshot:
    """Latest joint state snapshot."""

    names: list[str]
    positions: list[float]
    timestamp_ns: int


def now_ns() -> int:
    """Return current wall-clock time in nanoseconds."""
    return time.time_ns()


def image_msg_to_rgb_array(msg: Any) -> np.ndarray:
    """Convert a ROS2 sensor_msgs/Image-like object to RGB uint8 HWC."""
    encoding = str(msg.encoding).lower()
    height = int(msg.height)
    width = int(msg.width)
    step = int(msg.step)
    data = np.frombuffer(msg.data, dtype=np.uint8)

    if encoding in {"rgb8", "bgr8"}:
        channels = 3
        row_bytes = width * channels
        if step < row_bytes:
            raise ValueError(f"Image step {step} is smaller than expected row bytes {row_bytes}")
        rows = data.reshape((height, step))[:, :row_bytes]
        image = rows.reshape((height, width, channels))
        if encoding == "bgr8":
            image = image[..., ::-1]
        return np.ascontiguousarray(image)

    if encoding in {"rgba8", "bgra8"}:
        channels = 4
        row_bytes = width * channels
        if step < row_bytes:
            raise ValueError(f"Image step {step} is smaller than expected row bytes {row_bytes}")
        rows = data.reshape((height, step))[:, :row_bytes]
        image = rows.reshape((height, width, channels))
        if encoding == "bgra8":
            image = image[..., [2, 1, 0]]
        else:
            image = image[..., :3]
        return np.ascontiguousarray(image)

    if encoding == "mono8":
        row_bytes = width
        if step < row_bytes:
            raise ValueError(f"Image step {step} is smaller than expected row bytes {row_bytes}")
        rows = data.reshape((height, step))[:, :row_bytes]
        return np.ascontiguousarray(np.stack([rows, rows, rows], axis=-1))

    raise ValueError(f"Unsupported ROS image encoding: {msg.encoding}")


def encode_rgb_image(
    image_rgb: np.ndarray,
    *,
    width: int,
    height: int,
    encoding: str,
    jpeg_quality: int,
) -> EncodedImage:
    """Resize and encode an RGB uint8 HWC image."""
    if image_rgb.dtype != np.uint8 or image_rgb.ndim != 3 or image_rgb.shape[2] != 3:
        raise ValueError("image_rgb must be uint8 HWC with 3 channels")

    output = np.ascontiguousarray(image_rgb)
    if output.shape[1] != width or output.shape[0] != height:
        import cv2

        output = cv2.resize(output, (width, height), interpolation=cv2.INTER_AREA)

    normalized_encoding = encoding.lower()
    if normalized_encoding == "rgb8":
        return EncodedImage(output.tobytes(), width, height, "rgb8")
    if normalized_encoding == "bgr8":
        return EncodedImage(np.ascontiguousarray(output[..., ::-1]).tobytes(), width, height, "bgr8")
    if normalized_encoding in {"jpeg-rgb", "jpg-rgb", "jpeg-bgr", "jpg-bgr"}:
        import cv2

        image_bgr = cv2.cvtColor(output, cv2.COLOR_RGB2BGR)
        ok, encoded = cv2.imencode(".jpg", image_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality)])
        if not ok:
            raise RuntimeError("Failed to JPEG-encode camera frame")
        return EncodedImage(encoded.tobytes(), width, height, "jpeg-rgb")

    raise ValueError(f"Unsupported output image encoding: {encoding}")


def build_sensor_packet(
    *,
    sequence_id: int,
    instruction: str,
    front: EncodedImage,
    top: EncodedImage,
    joints: JointSnapshot,
) -> pb2.SensorPacket:
    """Build a SensorPacket from encoded cameras and joint state."""
    if len(joints.names) != len(joints.positions):
        raise ValueError("joint names and positions length mismatch")
    if not instruction:
        raise ValueError("instruction must be non-empty")
    return pb2.SensorPacket(
        sequence_id=int(sequence_id),
        timestamp_ns=now_ns(),
        instruction=instruction,
        front_image=pb2.ImagePayload(
            data=front.data,
            width=front.width,
            height=front.height,
            encoding=front.encoding,
        ),
        top_image=pb2.ImagePayload(
            data=top.data,
            width=top.width,
            height=top.height,
            encoding=top.encoding,
        ),
        joint_names=[str(name) for name in joints.names],
        joint_positions=[float(value) for value in joints.positions],
    )


def validate_action_packet(
    packet: pb2.ActionPacket,
    *,
    expected_joint_names: list[str],
    current_joints: JointSnapshot,
    action_cfg: dict[str, Any],
    safety_cfg: dict[str, Any],
    current_time_ns: int | None = None,
) -> list[float]:
    """Validate and order an action packet before publishing to ROS2."""
    current_time_ns = now_ns() if current_time_ns is None else int(current_time_ns)
    if list(packet.joint_names) != list(expected_joint_names):
        raise ValueError(
            "action joint order mismatch: "
            f"expected={expected_joint_names} got={list(packet.joint_names)}"
        )
    if len(packet.joint_targets) != len(expected_joint_names):
        raise ValueError("action target dimension mismatch")

    stale_timeout_s = float(action_cfg.get("stale_timeout_s", 0.5))
    if packet.timestamp_ns and (current_time_ns - int(packet.timestamp_ns)) > stale_timeout_s * 1e9:
        raise ValueError(f"stale action packet older than {stale_timeout_s:.3f}s")

    targets = [float(value) for value in packet.joint_targets]
    if not all(math.isfinite(value) for value in targets):
        raise ValueError("action contains NaN or Inf")

    current_by_name = dict(zip(current_joints.names, current_joints.positions, strict=True))
    missing_current = [name for name in expected_joint_names if name not in current_by_name]
    if missing_current:
        raise ValueError(f"current joint state missing joints: {', '.join(missing_current)}")

    joint_limits = safety_cfg.get("joint_limits") or {}
    limits_required = bool(safety_cfg.get("limits_required_for_actuation", True))
    limits_required_for_validation = bool(safety_cfg.get("limits_required_for_validation", False))
    actuation_enabled = bool(action_cfg.get("actuation_enabled", False))
    if (actuation_enabled or limits_required_for_validation) and limits_required:
        missing_limits = [name for name in expected_joint_names if name not in joint_limits]
        if missing_limits:
            raise ValueError(f"joint limits missing: {', '.join(missing_limits)}")

    for name, target in zip(expected_joint_names, targets, strict=True):
        limit = joint_limits.get(name)
        if limit is not None:
            if not isinstance(limit, (list, tuple)) or len(limit) != 2:
                raise ValueError(f"joint limit for {name} must be [min, max]")
            low, high = float(limit[0]), float(limit[1])
            if target < low or target > high:
                raise ValueError(f"joint target for {name}={target:.6f} outside [{low:.6f}, {high:.6f}]")

    max_delta = safety_cfg.get("max_delta_per_step")
    if max_delta is not None:
        max_delta = float(max_delta)
        for name, target in zip(expected_joint_names, targets, strict=True):
            delta = abs(target - float(current_by_name[name]))
            if delta > max_delta:
                raise ValueError(f"joint delta for {name}={delta:.6f} exceeds {max_delta:.6f}")

    return targets


class SO101SensorGateway:
    """Runtime SO101 ROS2/gRPC gateway."""

    def __init__(self, config: dict[str, Any]) -> None:
        self.config = config
        self.bridge_cfg = section(config, "bridge")
        self.ros_cfg = section(config, "ros")
        self.sensor_cfg = section(config, "sensor")
        self.action_cfg = section(config, "actions")
        self.safety_cfg = section(config, "safety")
        self.gateway_id = str(self.bridge_cfg.get("gateway_id", "so101_robot"))
        self.joint_names = [str(name) for name in self.sensor_cfg["joint_names"]]
        self._lock = threading.Lock()
        self._front: ImageFrame | None = None
        self._top: ImageFrame | None = None
        self._joints: JointSnapshot | None = None
        self._last_publish_ns = 0
        self._sequence_id = 1
        self._stop_event = threading.Event()
        self._node = None
        self._joint_target_publisher = None
        self._joint_state_msg_cls = None

    def _target(self) -> str:
        return f"{self.bridge_cfg.get('host', '127.0.0.1')}:{int(self.bridge_cfg.get('port', 49100))}"

    def _front_callback(self, msg: Any) -> None:
        frame = ImageFrame(image_msg_to_rgb_array(msg), now_ns())
        with self._lock:
            self._front = frame

    def _top_callback(self, msg: Any) -> None:
        frame = ImageFrame(image_msg_to_rgb_array(msg), now_ns())
        with self._lock:
            self._top = frame

    def _joint_callback(self, msg: Any) -> None:
        names = [str(name) for name in msg.name]
        positions = [float(value) for value in msg.position]
        with self._lock:
            self._joints = JointSnapshot(names=names, positions=positions, timestamp_ns=now_ns())

    def _snapshot(self) -> tuple[ImageFrame, ImageFrame, JointSnapshot]:
        with self._lock:
            front, top, joints = self._front, self._top, self._joints
        if front is None:
            raise TimeoutError("front camera frame not received yet")
        if top is None:
            raise TimeoutError("top camera frame not received yet")
        if joints is None:
            raise TimeoutError("joint state not received yet")

        stale_timeout_ns = int(float(self.sensor_cfg.get("stale_timeout_s", 1.0)) * 1e9)
        current_ns = now_ns()
        stale = []
        if current_ns - front.timestamp_ns > stale_timeout_ns:
            stale.append("front camera")
        if current_ns - top.timestamp_ns > stale_timeout_ns:
            stale.append("top camera")
        if current_ns - joints.timestamp_ns > stale_timeout_ns:
            stale.append("joint state")
        if stale:
            raise TimeoutError(f"stale sensor data: {', '.join(stale)}")
        return front, top, joints

    def _make_packet(self) -> pb2.SensorPacket:
        front_frame, top_frame, joints = self._snapshot()
        width = int(self.sensor_cfg.get("width", 256))
        height = int(self.sensor_cfg.get("height", 256))
        encoding = str(self.sensor_cfg.get("output_encoding", "jpeg-rgb"))
        jpeg_quality = int(self.sensor_cfg.get("jpeg_quality", 85))
        front = encode_rgb_image(
            front_frame.image_rgb,
            width=width,
            height=height,
            encoding=encoding,
            jpeg_quality=jpeg_quality,
        )
        top = encode_rgb_image(
            top_frame.image_rgb,
            width=width,
            height=height,
            encoding=encoding,
            jpeg_quality=jpeg_quality,
        )
        packet = build_sensor_packet(
            sequence_id=self._sequence_id,
            instruction=str(self.sensor_cfg.get("instruction", "")),
            front=front,
            top=top,
            joints=joints,
        )
        self._sequence_id += 1
        return packet

    def _sensor_loop(self, stub: pb2_grpc.SO101RemoteBridgeStub) -> None:
        fps = float(self.sensor_cfg.get("fps", 5))
        interval_s = 1.0 / fps
        while not self._stop_event.is_set():
            try:
                packet = self._make_packet()
                reply = stub.PushSensorPacket(packet, timeout=2.0)
                if not reply.accepted:
                    print(f"Sensor packet rejected by bridge: {reply.message}", flush=True)
            except TimeoutError as exc:
                print(f"Waiting for fresh sensors: {exc}", flush=True)
            except grpc.RpcError as exc:
                print(f"Sensor push failed: {exc}", flush=True)
            except Exception as exc:  # noqa: BLE001
                print(f"Sensor packet build failed: {exc}", flush=True)
            self._stop_event.wait(interval_s)

    def _latest_joints_for_action(self) -> JointSnapshot:
        with self._lock:
            joints = self._joints
        if joints is None:
            raise RuntimeError("cannot process action before joint state is available")

        stale_timeout_ns = int(float(self.sensor_cfg.get("stale_timeout_s", 1.0)) * 1e9)
        if now_ns() - joints.timestamp_ns > stale_timeout_ns:
            raise RuntimeError("cannot process action with stale joint state")
        return joints

    def _publish_action(self, packet: pb2.ActionPacket) -> None:
        joints = self._latest_joints_for_action()
        targets = validate_action_packet(
            packet,
            expected_joint_names=self.joint_names,
            current_joints=joints,
            action_cfg=self.action_cfg,
            safety_cfg=self.safety_cfg,
        )

        if not bool(self.action_cfg.get("actuation_enabled", False)):
            current_by_name = dict(zip(joints.names, joints.positions, strict=True))
            deltas = [
                abs(target - float(current_by_name[name]))
                for name, target in zip(self.joint_names, targets, strict=True)
            ]
            print(
                "DRY-RUN valid action",
                f"sequence_id={packet.sequence_id}",
                f"targets={[round(float(v), 5) for v in targets]}",
                f"max_delta={max(deltas):.6f}",
                flush=True,
            )
            return

        publish_rate_limit_hz = float(self.action_cfg.get("publish_rate_limit_hz", 20))
        min_interval_ns = int(1e9 / publish_rate_limit_hz) if publish_rate_limit_hz > 0 else 0
        current_ns = now_ns()
        if current_ns - self._last_publish_ns < min_interval_ns:
            raise RuntimeError("action publish rate limit exceeded")
        self._last_publish_ns = current_ns

        if self._joint_target_publisher is None or self._joint_state_msg_cls is None:
            raise RuntimeError("joint target publisher is not initialized")
        message = self._joint_state_msg_cls()
        message.header.stamp = self._node.get_clock().now().to_msg()
        message.name = list(self.joint_names)
        message.position = targets
        self._joint_target_publisher.publish(message)
        print(f"Published action sequence_id={packet.sequence_id}", flush=True)

    def _action_loop(self, stub: pb2_grpc.SO101RemoteBridgeStub) -> None:
        reconnect_s = float(self.action_cfg.get("stream_reconnect_s", 1.0))
        while not self._stop_event.is_set():
            try:
                status = pb2.GatewayStatus(gateway_id=self.gateway_id, last_sequence_id=self._sequence_id)
                for action in stub.StreamActions(status):
                    if self._stop_event.is_set():
                        return
                    local_action = pb2.ActionPacket(
                        sequence_id=action.sequence_id,
                        timestamp_ns=now_ns(),
                        joint_names=list(action.joint_names),
                        joint_targets=list(action.joint_targets),
                    )
                    try:
                        self._publish_action(local_action)
                    except Exception as exc:  # noqa: BLE001
                        print(
                            f"Rejected action sequence_id={action.sequence_id}: {exc}",
                            f"targets={[round(float(v), 5) for v in local_action.joint_targets]}",
                            flush=True,
                        )
            except grpc.RpcError as exc:
                if not self._stop_event.is_set():
                    print(f"Action stream disconnected: {exc}; reconnecting in {reconnect_s:.2f}s", flush=True)
                    self._stop_event.wait(reconnect_s)

    def run(self) -> int:
        """Run the ROS2 node and gateway workers."""
        import rclpy
        from sensor_msgs.msg import Image, JointState

        self._joint_state_msg_cls = JointState
        rclpy.init()
        node_name = str(self.ros_cfg.get("node_name", "so101_sensor_gateway"))
        qos_depth = int(self.ros_cfg.get("qos_depth", 5))
        self._node = rclpy.create_node(node_name)
        self._node.create_subscription(Image, str(self.ros_cfg["front_camera_topic"]), self._front_callback, qos_depth)
        self._node.create_subscription(Image, str(self.ros_cfg["top_camera_topic"]), self._top_callback, qos_depth)
        self._node.create_subscription(JointState, str(self.ros_cfg["joint_states_topic"]), self._joint_callback, qos_depth)
        self._joint_target_publisher = self._node.create_publisher(
            JointState,
            str(self.ros_cfg["joint_targets_topic"]),
            qos_depth,
        )

        target = self._target()
        channel = grpc.insecure_channel(target)
        stub = pb2_grpc.SO101RemoteBridgeStub(channel)
        sensor_thread = threading.Thread(target=self._sensor_loop, args=(stub,), daemon=True)
        action_thread = threading.Thread(target=self._action_loop, args=(stub,), daemon=True)
        sensor_thread.start()
        action_thread.start()
        print(
            f"SO101 gateway started: bridge={target} actuation_enabled={self.action_cfg.get('actuation_enabled')}",
            flush=True,
        )

        try:
            rclpy.spin(self._node)
        except KeyboardInterrupt:
            pass
        finally:
            self._stop_event.set()
            sensor_thread.join(timeout=2)
            action_thread.join(timeout=2)
            channel.close()
            self._node.destroy_node()
            rclpy.shutdown()
        return 0


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/so101_gateway.yaml")
    parser.add_argument("--dry-run", action="store_true", help="Force actuation_enabled=false for this run.")
    return parser.parse_args()


def main() -> int:
    """Run the SO101 gateway."""
    args = parse_args()
    config = load_yaml(args.config)
    if args.dry_run:
        actions = section(config, "actions")
        actions["actuation_enabled"] = False
    return SO101SensorGateway(config).run()


if __name__ == "__main__":
    raise SystemExit(main())
