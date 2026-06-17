"""LeRobot Robot adapter backed by SO101 sensor/action packets."""

from __future__ import annotations

from typing import Any

from lerobot.robots.robot import Robot
from lerobot.types import RobotAction, RobotObservation

from remote_so101.bridge import BridgeState, get_bridge_state
from remote_so101.config import RemoteSO101Config
from remote_so101.packet_codec import sensor_packet_to_robot_observation


class RemoteSO101(Robot):
    """Remote SO101 robot implementation for official LeRobot RobotClient."""

    config_class = RemoteSO101Config
    name = "remote_so101"

    def __init__(self, config: RemoteSO101Config):
        super().__init__(config)
        self.config = config
        self._is_connected = False
        self._bridge_state: BridgeState = get_bridge_state(config.bridge_id)
        self._last_sequence_id: int | None = None

    @property
    def observation_features(self) -> dict[str, type | tuple[int, int, int]]:
        """Return LeRobot raw observation feature description."""
        features: dict[str, Any] = {name: float for name in self.config.joint_names}
        camera_shape = (
            int(self.config.image_height),
            int(self.config.image_width),
            int(self.config.image_channels),
        )
        features[self.config.front_camera_key] = camera_shape
        features[self.config.top_camera_key] = camera_shape
        return features

    @property
    def action_features(self) -> dict[str, type]:
        """Return action feature description in policy action order."""
        return {name: float for name in self.config.joint_names}

    @property
    def is_connected(self) -> bool:
        """Return whether this remote adapter is connected."""
        return self._is_connected

    def connect(self, calibrate: bool = True) -> None:
        """Connect the adapter to process-local bridge state."""
        self._is_connected = True

    @property
    def is_calibrated(self) -> bool:
        """Remote adapter has no local calibration step."""
        return True

    def calibrate(self) -> None:
        """No-op calibration for the remote adapter."""
        return None

    def configure(self) -> None:
        """No-op runtime configuration for the remote adapter."""
        return None

    def get_observation(self) -> RobotObservation:
        """Return the latest SO101 sensor packet as a LeRobot raw observation."""
        if not self.is_connected:
            raise RuntimeError("RemoteSO101 is not connected")
        packet = self._bridge_state.wait_for_sensor_packet(
            timeout_s=self.config.sensor_timeout_s,
            last_sequence_id=self._last_sequence_id,
        )
        self._last_sequence_id = int(packet.sequence_id)
        return sensor_packet_to_robot_observation(packet, self.config)

    def send_action(self, action: RobotAction) -> RobotAction:
        """Queue an action packet for the SO101 gateway."""
        if not self.is_connected:
            raise RuntimeError("RemoteSO101 is not connected")
        ordered_targets = [float(action[name]) for name in self.config.joint_names]
        self._bridge_state.enqueue_action(self.config.joint_names, ordered_targets)
        return {name: float(action[name]) for name in self.config.joint_names}

    def disconnect(self) -> None:
        """Disconnect this adapter."""
        self._is_connected = False
