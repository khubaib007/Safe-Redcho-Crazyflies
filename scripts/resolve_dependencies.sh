#!/usr/bin/env bash
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONNOUSERSITE=1
source /opt/ros/jazzy/setup.bash
cd "$root"
/usr/bin/python3 scripts/fetch_dependencies.py --check
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then sudo rosdep init; fi
rosdep update
mapfile -t packages < <(/usr/bin/python3 -c 'import json;print("\n".join(json.load(open("release.json"))["packages"]))')
mapfile -t paths < <(/usr/bin/colcon list --packages-up-to "${packages[@]}" crazyflie_server_cpp motion_capture_tracking crazyflie_description --paths-only)
rosdep install --from-paths "${paths[@]}" --ignore-src --rosdistro jazzy -y
