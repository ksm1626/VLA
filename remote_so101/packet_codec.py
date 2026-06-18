"""Conversions between SO101 bridge packets and LeRobot robot observations/actions."""

from __future__ import annotations

import numpy as np

from remote_so101.config import RemoteSO101Config
from remote_so101.proto_modules import pb2
from remote_so101.unit_adapter import ros_joint_to_policy


def decode_image_payload(payload: pb2.ImagePayload) -> np.ndarray:
    """Decode ImagePayload to an RGB uint8 HWC numpy array."""
    encoding = payload.encoding.lower()
    if encoding in {"rgb8", "bgr8"}:
        image = np.frombuffer(payload.data, dtype=np.uint8)
        expected = int(payload.height) * int(payload.width) * 3
        if image.size != expected:
            raise ValueError(f"Image byte length {image.size} does not match expected {expected}")
        image = image.reshape((int(payload.height), int(payload.width), 3))
        if encoding == "bgr8":
            image = image[..., ::-1]
        return np.ascontiguousarray(image)

    if encoding in {"jpeg-rgb", "jpeg-bgr", "jpg-rgb", "jpg-bgr"}:
        import cv2

        encoded = np.frombuffer(payload.data, dtype=np.uint8)
        image_bgr = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if image_bgr is None:
            raise ValueError("Failed to decode JPEG image payload")
        image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        return np.ascontiguousarray(image_rgb)

    raise ValueError(f"Unsupported image encoding: {payload.encoding}")


def sensor_packet_to_robot_observation(
    packet: pb2.SensorPacket,
    config: RemoteSO101Config,
) -> dict:
    """Convert a SensorPacket to the raw observation expected by LeRobot RobotClient."""
    if len(packet.joint_names) != len(packet.joint_positions):
        raise ValueError("joint_names and joint_positions length mismatch")

    positions_by_name = dict(zip(packet.joint_names, packet.joint_positions, strict=True))
    missing = [name for name in config.joint_names if name not in positions_by_name]
    if missing:
        raise ValueError(f"Sensor packet is missing joints: {', '.join(missing)}")

    observation = {
        name: ros_joint_to_policy(name, float(positions_by_name[name]), config)
        for name in config.joint_names
    }
    observation[config.front_camera_key] = decode_image_payload(packet.front_image)
    observation[config.top_camera_key] = decode_image_payload(packet.top_image)
    return observation


def sensor_packet_positions_by_name(packet: pb2.SensorPacket) -> dict[str, float]:
    """Return raw SO101 ROS positions from a SensorPacket."""
    if len(packet.joint_names) != len(packet.joint_positions):
        raise ValueError("joint_names and joint_positions length mismatch")
    return dict(zip(packet.joint_names, (float(v) for v in packet.joint_positions), strict=True))
