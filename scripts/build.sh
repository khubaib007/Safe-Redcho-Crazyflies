#!/usr/bin/env bash
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH VIRTUAL_ENV
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export PYTHONNOUSERSITE=1
source /opt/ros/jazzy/setup.bash
if [[ -n "${REDCHO_VENDOR_PREFIX:-}" ]]; then source "$REDCHO_VENDOR_PREFIX/local_setup.bash"; fi
cd "$root"
mapfile -t packages < <(/usr/bin/python3 -c 'import json;print("\n".join(json.load(open("release.json"))["packages"]))')
if [[ -n "${REDCHO_VENDOR_PREFIX:-}" ]]; then
  exec /usr/bin/colcon build --symlink-install --packages-select "${packages[@]}" --executor sequential "$@"
fi
/usr/bin/python3 scripts/fetch_dependencies.py --check
exec /usr/bin/colcon build --symlink-install --packages-up-to "${packages[@]}" crazyflie_server_cpp motion_capture_tracking crazyflie_description --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release "$@"
