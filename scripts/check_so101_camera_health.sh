#!/usr/bin/env bash
set -euo pipefail

# Refresh the ROS graph and verify that the SO101 camera topics are actually
# publishing frames. `ros2 topic list` alone can show stale daemon cache.

echo "== Refresh ROS daemon =="
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true
sleep "${DISCOVERY_WAIT_S:-3}"

echo
echo "== ROS nodes =="
ros2 node list || true

echo
echo "== ROS topics =="
ros2 topic list || true

check_topic() {
  local topic="$1"

  echo
  echo "== ${topic} info =="
  ros2 topic info -v "${topic}" || true

  echo
  echo "== ${topic} hz =="
  timeout 5 ros2 topic hz "${topic}" || true
}

check_topic /arm/front_cam
check_topic /arm/top_cam
