"""RemoteSO101 adapter unit tests without ROS2 or PolicyServer."""

from __future__ import annotations

import unittest

import numpy as np

from remote_so101.bridge import get_bridge_state
from remote_so101.config import RemoteSO101Config
from remote_so101.proto_modules import pb2
from remote_so101.remote_robot import RemoteSO101


JOINT_NAMES = ["j0", "j1", "j2", "j3", "j4", "j5"]


def _packet(sequence_id: int) -> pb2.SensorPacket:
    front = np.zeros((4, 4, 3), dtype=np.uint8)
    top = np.ones((4, 4, 3), dtype=np.uint8)
    return pb2.SensorPacket(
        sequence_id=sequence_id,
        timestamp_ns=123,
        instruction="test task",
        front_image=pb2.ImagePayload(data=front.tobytes(), width=4, height=4, encoding="rgb8"),
        top_image=pb2.ImagePayload(data=top.tobytes(), width=4, height=4, encoding="rgb8"),
        joint_names=JOINT_NAMES,
        joint_positions=[float(i) for i in range(6)],
    )


class RemoteSO101Test(unittest.TestCase):
    def test_remote_robot_observation_and_action(self) -> None:
        bridge_id = "unit-test-remote-so101"
        state = get_bridge_state(bridge_id)
        accepted, message = state.push_sensor_packet(_packet(1))
        self.assertTrue(accepted, message)

        config = RemoteSO101Config(
            id="unit",
            bridge_id=bridge_id,
            image_width=4,
            image_height=4,
            joint_names=JOINT_NAMES,
        )
        robot = RemoteSO101(config)
        robot.connect()

        observation = robot.get_observation()
        self.assertEqual(observation["j0"], 0.0)
        self.assertEqual(observation["j5"], 5.0)
        self.assertEqual(observation["camera1"].shape, (4, 4, 3))
        self.assertEqual(observation["camera2"].shape, (4, 4, 3))

        sent = robot.send_action({name: float(idx) / 10 for idx, name in enumerate(JOINT_NAMES)})
        self.assertEqual(sent["j5"], 0.5)
        action = state.get_action(timeout_s=0.1)
        self.assertIsNotNone(action)
        assert action is not None
        self.assertEqual(list(action.joint_names), JOINT_NAMES)
        self.assertEqual(list(action.joint_targets)[-1], 0.5)


if __name__ == "__main__":
    unittest.main()
