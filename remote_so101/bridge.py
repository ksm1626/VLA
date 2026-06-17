"""A6000-side gRPC bridge state shared by RemoteSO101Robot and the mock/SO101 gateway."""

from __future__ import annotations

import queue
import threading
import time
from concurrent import futures
from dataclasses import dataclass
from typing import Iterable

import grpc

from remote_so101.proto_modules import pb2, pb2_grpc


@dataclass(frozen=True)
class BridgeServer:
    """Handle returned by start_bridge_server."""

    server: grpc.Server
    state: "BridgeState"
    address: str

    def stop(self, grace: float = 0.0) -> None:
        """Stop the gRPC server."""
        self.server.stop(grace)


class BridgeState:
    """Thread-safe latest sensor packet and outgoing action queue."""

    def __init__(self, bridge_id: str = "default") -> None:
        self.bridge_id = bridge_id
        self._condition = threading.Condition()
        self._latest_sensor_packet: pb2.SensorPacket | None = None
        self._actions: queue.Queue[pb2.ActionPacket] = queue.Queue()
        self._action_sequence_id = 0

    def push_sensor_packet(self, packet: pb2.SensorPacket) -> tuple[bool, str]:
        """Store the latest sensor packet."""
        if len(packet.joint_names) != len(packet.joint_positions):
            return False, "joint_names and joint_positions length mismatch"
        if not packet.instruction:
            return False, "instruction is empty"
        if not packet.front_image.data:
            return False, "front_image is empty"
        if not packet.top_image.data:
            return False, "top_image is empty"

        with self._condition:
            self._latest_sensor_packet = packet
            self._condition.notify_all()
        return True, "accepted"

    def wait_for_sensor_packet(
        self,
        timeout_s: float,
        last_sequence_id: int | None = None,
    ) -> pb2.SensorPacket:
        """Wait until a sensor packet is available, preferably newer than last_sequence_id."""
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while True:
                packet = self._latest_sensor_packet
                if packet is not None and (
                    last_sequence_id is None or packet.sequence_id != last_sequence_id
                ):
                    return packet

                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(f"No sensor packet received within {timeout_s:.3f}s")
                self._condition.wait(timeout=remaining)

    @property
    def latest_sequence_id(self) -> int:
        """Return latest sensor sequence id, or zero if no packet has arrived."""
        with self._condition:
            if self._latest_sensor_packet is None:
                return 0
            return int(self._latest_sensor_packet.sequence_id)

    def enqueue_action(self, joint_names: Iterable[str], joint_targets: Iterable[float]) -> pb2.ActionPacket:
        """Queue an action packet for the SO101 gateway."""
        names = list(joint_names)
        targets = [float(value) for value in joint_targets]
        if len(names) != len(targets):
            raise ValueError("joint_names and joint_targets length mismatch")

        self._action_sequence_id += 1
        packet = pb2.ActionPacket(
            sequence_id=self._action_sequence_id,
            timestamp_ns=time.time_ns(),
            joint_names=names,
            joint_targets=targets,
        )
        self._actions.put(packet)
        return packet

    def get_action(self, timeout_s: float = 0.1) -> pb2.ActionPacket | None:
        """Return the next queued action, or None on timeout."""
        try:
            return self._actions.get(timeout=timeout_s)
        except queue.Empty:
            return None

    @property
    def queued_actions(self) -> int:
        """Return outgoing action queue size."""
        return self._actions.qsize()


class SO101RemoteBridgeServicer(pb2_grpc.SO101RemoteBridgeServicer):
    """gRPC service used by SO101 Gateway or mock gateway."""

    def __init__(self, state: BridgeState) -> None:
        self.state = state

    def PushSensorPacket(self, request, context):  # noqa: N802
        accepted, message = self.state.push_sensor_packet(request)
        return pb2.PushSensorReply(accepted=accepted, message=message)

    def StreamActions(self, request, context):  # noqa: N802
        while context.is_active():
            action = self.state.get_action(timeout_s=0.1)
            if action is not None:
                yield action

    def Heartbeat(self, request, context):  # noqa: N802
        return pb2.BridgeStatus(
            ready=True,
            message="ok",
            latest_sequence_id=self.state.latest_sequence_id,
            queued_actions=self.state.queued_actions,
        )


_STATES: dict[str, BridgeState] = {}


def get_bridge_state(bridge_id: str = "default") -> BridgeState:
    """Get or create a process-local bridge state."""
    if bridge_id not in _STATES:
        _STATES[bridge_id] = BridgeState(bridge_id=bridge_id)
    return _STATES[bridge_id]


def start_bridge_server(host: str, port: int, bridge_id: str = "default") -> BridgeServer:
    """Start the A6000 bridge gRPC server."""
    state = get_bridge_state(bridge_id)
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=4))
    pb2_grpc.add_SO101RemoteBridgeServicer_to_server(SO101RemoteBridgeServicer(state), server)
    address = f"{host}:{port}"
    bound_port = server.add_insecure_port(address)
    if bound_port == 0:
        raise RuntimeError(f"Failed to bind SO101 bridge server at {address}")
    server.start()
    return BridgeServer(server=server, state=state, address=address)
