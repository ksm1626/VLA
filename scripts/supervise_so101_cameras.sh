#!/usr/bin/env bash
set -euo pipefail

# Supervises the vendor SO101 camera executable without modifying vendor code.
# Start this after sourcing the ROS2/vendor workspace. It waits while the
# bringup-owned camera nodes are alive, then takes over and respawns them if
# they exit.

RESPAWN_DELAY_S="${RESPAWN_DELAY_S:-2}"
RESPAWN_MAX_DELAY_S="${RESPAWN_MAX_DELAY_S:-30}"
POLL_INTERVAL_S="${POLL_INTERVAL_S:-1}"

PIDS=()

cleanup() {
  trap - INT TERM EXIT
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
    fi
  done
  wait || true
}

node_exists() {
  local node_name="$1"
  ros2 node list 2>/dev/null | grep -Fxq "/${node_name}"
}

supervise_camera() {
  local camera_name="$1"
  local node_name="$2"
  local initial_delay_s="${3:-0}"
  local delay_s="${RESPAWN_DELAY_S}"

  sleep "${initial_delay_s}"

  while true; do
    while node_exists "${node_name}"; do
      echo "[${node_name}] already exists; waiting before taking over"
      sleep "${POLL_INTERVAL_S}"
    done

    echo "[${node_name}] starting camera_name=${camera_name}"
    set +e
    ros2 run physicai_arm camera_node \
      --ros-args \
      -r "__node:=${node_name}" \
      -p "camera_name:=${camera_name}"
    local rc=$?
    set -e

    echo "[${node_name}] exited rc=${rc}; respawning in ${delay_s}s"
    sleep "${delay_s}"
    if (( delay_s < RESPAWN_MAX_DELAY_S )); then
      delay_s=$(( delay_s * 2 ))
      if (( delay_s > RESPAWN_MAX_DELAY_S )); then
        delay_s="${RESPAWN_MAX_DELAY_S}"
      fi
    fi
  done
}

trap cleanup INT TERM EXIT

supervise_camera top arm_top_cam_publisher 0 &
PIDS+=("$!")

supervise_camera front arm_front_cam_publisher 4 &
PIDS+=("$!")

wait
