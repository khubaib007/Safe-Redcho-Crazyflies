#!/usr/bin/env bash
set -eo pipefail
case "${1:-help}" in
  offline)
    sudo apt update
    sudo apt install -y python3-pytest python3-numpy python3-scipy python3-yaml python3-matplotlib python3-pyqt5 python3-pyqtgraph
    ;;
  ros)
    [[ -f /opt/ros/jazzy/setup.bash ]] || { echo 'Install ROS Jazzy first; see docs/installation.md.' >&2; exit 2; }
    sudo apt update
    sudo apt install -y git build-essential cmake libusb-1.0-0-dev libboost-program-options-dev       python3-colcon-common-extensions python3-rosdep python3-pytest python3-numpy python3-scipy python3-yaml       python3-matplotlib python3-pyqt5 python3-pyqtgraph python3-venv python3-pip ros-jazzy-tf-transformations       ros-jazzy-rmw-fastrtps-cpp tcpdump usbutils
    root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
    /usr/bin/python3 -m venv --system-site-packages "$root/.venv"
    "$root/.venv/bin/python" -m pip install --no-deps rowan==1.3.2
    ;;
  *) echo 'Usage: bash scripts/setup.sh {offline|ros}'; exit 2 ;;
esac
