#!/usr/bin/env bash
set -euo pipefail

# Recover the Jetson Argus camera service after CSI/Argus errors.
# This does not modify vendor code. It stops VLA camera supervisors and vendor
# camera_node processes, restarts nvargus-daemon, then refreshes the ROS graph.

echo "== Stop camera supervisors and camera_node processes =="
pkill -f "supervise_so101_cameras.sh" 2>/dev/null || true
pkill -f "physicai_arm.*/camera_node" 2>/dev/null || true
pkill -f "ros2 run physicai_arm camera_node" 2>/dev/null || true
sleep 2

echo
echo "== Restart nvargus-daemon =="
sudo systemctl restart nvargus-daemon
sleep 3
systemctl --no-pager -l status nvargus-daemon | sed -n "1,80p"

echo
echo "== Refresh ROS daemon =="
ros2 daemon stop >/dev/null 2>&1 || true
sleep 1

echo
echo "== Camera devices =="
if command -v v4l2-ctl >/dev/null 2>&1; then
  v4l2-ctl --list-devices || true
fi
ls -l /dev/video* 2>/dev/null || true

echo
echo "Argus reset complete. Restart bringup or the VLA camera supervisor next."
