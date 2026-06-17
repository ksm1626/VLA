#!/usr/bin/env bash
set -euo pipefail

echo "== System Python / ROS2 Humble =="
if [[ -f /opt/ros/humble/setup.bash ]]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
  echo "ROS_DISTRO=${ROS_DISTRO:-<unset>}"
  command -v ros2 || true
  python3.10 - <<'PY'
import sys
print("python", sys.executable)
print("version", sys.version.split()[0])
try:
    import rclpy
    print("rclpy OK")
except Exception as exc:
    print(f"rclpy FAIL: {exc}")
PY
else
  echo "ROS2 Humble not found at /opt/ros/humble"
fi

echo
echo "== vla-lerobot =="
if command -v conda >/dev/null 2>&1 && conda env list | awk '{print $1}' | grep -qx "vla-lerobot"; then
  conda run -n vla-lerobot env PYTHONNOUSERSITE=1 python -c '
import importlib.metadata as metadata
import sys
print("python", sys.executable)
print("version", sys.version.split()[0])
try:
    print("lerobot Requires-Python", metadata.metadata("lerobot").get("Requires-Python"))
except Exception as exc:
    print(f"lerobot metadata FAIL: {exc}")
for module_name in ("lerobot", "lerobot.async_inference.robot_client", "torch", "rclpy"):
    try:
        __import__(module_name)
        print(f"{module_name} OK")
    except Exception as exc:
        print(f"{module_name} FAIL: {exc}")
'
else
  echo "conda env vla-lerobot not found"
fi

echo
echo "Expected split:"
echo "- vla-lerobot: Python 3.12, LeRobot/SmolVLA/PolicyServer"
echo "- ROS2 Humble: Python 3.10 rclpy"
