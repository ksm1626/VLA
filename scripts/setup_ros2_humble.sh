#!/usr/bin/env bash
set -euo pipefail

if [[ -r /etc/os-release ]]; then
  # shellcheck disable=SC1091
  . /etc/os-release
else
  echo "/etc/os-release not found" >&2
  exit 1
fi

if [[ "${VERSION_CODENAME:-}" != "jammy" ]]; then
  echo "ROS2 Humble deb packages target Ubuntu 22.04 Jammy; found ${PRETTY_NAME:-unknown}." >&2
  exit 1
fi

sudo apt update
sudo apt install -y locales software-properties-common curl gnupg
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
sudo add-apt-repository -y universe

ROS_APT_SOURCE_VERSION="$(
  curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F '"tag_name"' \
    | awk -F'"' '{print $4}'
)"

if [[ -z "${ROS_APT_SOURCE_VERSION}" ]]; then
  echo "Could not determine latest ros-apt-source release." >&2
  exit 1
fi

curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${VERSION_CODENAME}_all.deb"

sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
sudo apt install -y ros-humble-ros-base ros-dev-tools

echo "ROS2 Humble ros-base installed. Source it with:"
echo "source /opt/ros/humble/setup.bash"

