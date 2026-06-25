#!/usr/bin/env python3
"""Run official LeRobot RobotClient with the A6000 RemoteSO101Robot adapter."""

from __future__ import annotations

import argparse
import logging
import os
import site
import sys
import threading
import time
from pathlib import Path

os.environ.setdefault("PYTHONNOUSERSITE", "1")

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


def _remove_user_site_from_sys_path() -> None:
    """Keep conda/venv imports isolated from ~/.local packages."""
    user_sites = site.getusersitepackages()
    if isinstance(user_sites, str):
        user_sites = [user_sites]
    resolved_user_sites = {str(Path(path).resolve()) for path in user_sites}
    sys.path[:] = [
        path
        for path in sys.path
        if str(Path(path).resolve()) not in resolved_user_sites
    ]


_remove_user_site_from_sys_path()

from lerobot.async_inference.configs import RobotClientConfig  # noqa: E402
from lerobot.async_inference.robot_client import RobotClient  # noqa: E402

from remote_so101.bridge import start_bridge_server  # noqa: E402
from remote_so101.config import DEFAULT_JOINT_NAMES, RemoteSO101Config  # noqa: E402
from remote_so101.config_loader import load_yaml, section, value  # noqa: E402


def _make_client_config(config: dict) -> RobotClientConfig:
    bridge = section(config, "bridge")
    robot = section(config, "robot")
    policy = section(config, "policy")
    runtime = section(config, "runtime")

    joint_names = robot.get("joint_names") or list(DEFAULT_JOINT_NAMES)
    unit_adapter = robot.get("unit_adapter") or {}
    remote_robot_config = RemoteSO101Config(
        id=str(robot.get("id", "remote_so101")),
        bridge_id=str(bridge.get("id", "default")),
        bridge_host=str(bridge.get("host", "127.0.0.1")),
        bridge_port=int(bridge.get("port", 49100)),
        sensor_timeout_s=float(robot.get("sensor_timeout_s", 2.0)),
        image_width=int(robot.get("image_width", 256)),
        image_height=int(robot.get("image_height", 256)),
        image_channels=int(robot.get("image_channels", 3)),
        front_camera_key=str(robot.get("front_camera_key", "camera1")),
        top_camera_key=str(robot.get("top_camera_key", "camera2")),
        joint_names=[str(name) for name in joint_names],
        arm_joint_names=[
            str(name) for name in unit_adapter.get("arm_joint_names", list(DEFAULT_JOINT_NAMES[:5]))
        ],
        gripper_joint_name=str(unit_adapter.get("gripper_joint_name", "gripper")),
        unit_adapter_enabled=bool(unit_adapter.get("enabled", True)),
        gripper_action_mode=str(unit_adapter.get("gripper_action_mode", "linear")),
        gripper_policy_min=float(unit_adapter.get("gripper_policy_min", 0.0)),
        gripper_policy_max=float(unit_adapter.get("gripper_policy_max", 33.0)),
        gripper_ros_closed=float(unit_adapter.get("gripper_ros_closed", 0.0)),
        gripper_ros_open=float(unit_adapter.get("gripper_ros_open", 0.8)),
        gripper_ros_min=float(unit_adapter.get("gripper_ros_min", -0.174533)),
        gripper_ros_max=float(unit_adapter.get("gripper_ros_max", 1.74533)),
    )

    return RobotClientConfig(
        policy_type=str(value(policy, "policy_type", "policy")),
        pretrained_name_or_path=str(value(policy, "pretrained_name_or_path", "policy")),
        robot=remote_robot_config,
        actions_per_chunk=int(value(policy, "actions_per_chunk", "policy")),
        task=str(value(runtime, "task", "runtime")),
        server_address=str(value(policy, "server_address", "policy")),
        policy_device=str(policy.get("policy_device", "cuda")),
        client_device=str(policy.get("client_device", "cpu")),
        chunk_size_threshold=float(policy.get("chunk_size_threshold", 0.6)),
        fps=int(runtime.get("fps", 5)),
        aggregate_fn_name=str(policy.get("aggregate_fn_name", "latest_only")),
        debug_visualize_queue_size=bool(runtime.get("debug_visualize_queue_size", False)),
    )


def _run_bounded_control_loop(
    client: RobotClient,
    task: str,
    duration_s: float,
    max_steps: int,
    stop_after_actions: int,
    verbose: bool,
) -> tuple[int, int]:
    deadline = time.monotonic() + duration_s
    steps = 0
    actions_sent = 0
    while client.running and time.monotonic() < deadline and steps < max_steps:
        loop_start = time.perf_counter()
        if client.actions_available():
            client.control_loop_action(verbose=verbose)
            actions_sent += 1
            if stop_after_actions > 0 and actions_sent >= stop_after_actions:
                break
        if client._ready_to_send_observation():  # noqa: SLF001 - official RobotClient control primitive
            client.control_loop_observation(task=task, verbose=verbose)
            steps += 1
        time.sleep(max(0, client.config.environment_dt - (time.perf_counter() - loop_start)))
    return steps, actions_sent


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/remote_so101.yaml")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--duration-s", type=float, default=None)
    parser.add_argument("--max-steps", type=int, default=None)
    parser.add_argument("--stop-after-actions", type=int, default=None)
    parser.add_argument("--verbose", action="store_true")
    return parser.parse_args()


def main() -> int:
    """Run the bounded official RobotClient loop."""
    args = parse_args()
    logging.basicConfig(level=logging.INFO)

    config = load_yaml(args.config)
    bridge_config = section(config, "bridge")
    runtime = section(config, "runtime")
    client_config = _make_client_config(config)

    duration_s = float(args.duration_s if args.duration_s is not None else runtime.get("duration_s", 20.0))
    max_steps = int(args.max_steps if args.max_steps is not None else runtime.get("max_steps", 20))
    stop_after_actions = int(
        args.stop_after_actions
        if args.stop_after_actions is not None
        else runtime.get("stop_after_actions", 0)
    )

    print(f"bridge={bridge_config.get('host', '127.0.0.1')}:{bridge_config.get('port', 49100)}")
    print(f"policy_server={client_config.server_address}")
    print(f"policy_type={client_config.policy_type}")
    print(f"checkpoint={client_config.pretrained_name_or_path}")
    print(f"robot_type={client_config.robot.type}")
    print(f"actions_per_chunk={client_config.actions_per_chunk}")
    if args.dry_run:
        return 0

    bridge_server = start_bridge_server(
        host=str(bridge_config.get("host", "127.0.0.1")),
        port=int(bridge_config.get("port", 49100)),
        bridge_id=str(bridge_config.get("id", "default")),
    )
    print(f"SO101 bridge server started at {bridge_server.address}")

    client = RobotClient(client_config)
    action_receiver_thread: threading.Thread | None = None
    try:
        if not client.start():
            return 1

        action_receiver_thread = threading.Thread(
            target=client.receive_actions,
            kwargs={"verbose": args.verbose},
            daemon=True,
        )
        action_receiver_thread.start()
        client.start_barrier.wait()

        observations, actions = _run_bounded_control_loop(
            client=client,
            task=client_config.task,
            duration_s=duration_s,
            max_steps=max_steps,
            stop_after_actions=stop_after_actions,
            verbose=args.verbose,
        )
        print(f"RobotClient bounded run complete: observations={observations} actions={actions}")
        return 0 if actions > 0 else 2
    finally:
        client.stop()
        if action_receiver_thread is not None:
            action_receiver_thread.join(timeout=3)
        bridge_server.stop()


if __name__ == "__main__":
    raise SystemExit(main())
