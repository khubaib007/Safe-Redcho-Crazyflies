#!/usr/bin/env bash
set -e
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONNOUSERSITE=1
if [[ -n "${AMENT_PREFIX_PATH:-}" && "$AMENT_PREFIX_PATH" == *"/home/"* ]]; then
  printf '%s\n' 'Start a clean shell as described in README.md before building.' >&2
  exit 1
fi
source /opt/ros/jazzy/setup.bash
sudo apt-get update
sudo apt-get install -y git build-essential cmake libusb-1.0-0-dev \
  libboost-program-options-dev python3-colcon-common-extensions python3-rosdep \
  python3-venv python3-pip python3-yaml python3-numpy python3-pytest \
  ros-jazzy-tf-transformations tcpdump usbutils
if [[ ! -f "$task_root/.venv/bin/activate" ]]; then
  /usr/bin/python3 -m venv --system-site-packages "$task_root/.venv"
fi
"$task_root/.venv/bin/python" -m pip install --no-deps 'rowan==1.3.2'
if [[ ! -e "$task_root/src/crazyswarm2" ]]; then
  git clone --branch main --recurse-submodules \
    https://github.com/IMRCLab/crazyswarm2.git "$task_root/src/crazyswarm2"
fi
if [[ ! -e "$task_root/src/motion_capture_tracking" ]]; then
  git clone --branch ros2 --recurse-submodules \
    https://github.com/IMRCLab/motion_capture_tracking.git "$task_root/src/motion_capture_tracking"
fi
# No automatic pull/rebase: later reruns preserve the first checked-out versions.
git -C "$task_root/src/crazyswarm2" submodule update --init --recursive
git -C "$task_root/src/motion_capture_tracking" submodule update --init --recursive
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update
cd "$task_root"
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
source "$task_root/scripts/environment.bash"
colcon build --symlink-install --packages-up-to safe_redcho_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
mkdir -p "$task_root/config/generated" "$task_root/results"
python3 "$task_root/scripts/record_versions.py"
python3 "$task_root/scripts/prepare_config.py"
printf '%s\n' 'Build complete. Follow README.md: pose reception first, then radio and hover.'
