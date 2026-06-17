"""Basic proto serialization tests."""

from __future__ import annotations

import unittest

from remote_so101.proto_modules import pb2


class ProtoRoundtripTest(unittest.TestCase):
    def test_sensor_packet_roundtrip(self) -> None:
        packet = pb2.SensorPacket(
            sequence_id=7,
            timestamp_ns=123,
            instruction="test task",
            front_image=pb2.ImagePayload(data=b"\x00" * 12, width=2, height=2, encoding="rgb8"),
            top_image=pb2.ImagePayload(data=b"\x01" * 12, width=2, height=2, encoding="rgb8"),
            joint_names=["a", "b"],
            joint_positions=[1.0, 2.0],
        )
        loaded = pb2.SensorPacket.FromString(packet.SerializeToString())
        self.assertEqual(loaded.sequence_id, 7)
        self.assertEqual(list(loaded.joint_names), ["a", "b"])
        self.assertEqual(list(loaded.joint_positions), [1.0, 2.0])

    def test_action_packet_roundtrip(self) -> None:
        packet = pb2.ActionPacket(
            sequence_id=3,
            timestamp_ns=456,
            joint_names=["a", "b"],
            joint_targets=[0.1, 0.2],
        )
        loaded = pb2.ActionPacket.FromString(packet.SerializeToString())
        self.assertEqual(loaded.sequence_id, 3)
        self.assertEqual(list(loaded.joint_names), ["a", "b"])
        self.assertAlmostEqual(loaded.joint_targets[0], 0.1)
        self.assertAlmostEqual(loaded.joint_targets[1], 0.2)


if __name__ == "__main__":
    unittest.main()
