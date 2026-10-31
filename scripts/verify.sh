#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/environment.bash"
cd "$SAFE_REDCHO_HW_ROOT"
/usr/bin/python3 scripts/verify.py
if [[ "${1:-}" == --ros ]]; then
  source scripts/ros_environment.bash
  /usr/bin/python3 - <<'PY'
from pathlib import Path
from ament_index_python.packages import get_package_prefix
from crazyflie_interfaces.msg import Position, FullState, Status
from crazyflie_interfaces.srv import Arm, Takeoff, Land, NotifySetpointsStop
from motion_capture_tracking_interfaces.msg import NamedPoseArray
import json,os
root=Path(os.environ['SAFE_REDCHO_HW_ROOT'])
for name in json.loads((root/'release.json').read_text())['packages']:
    if Path(get_package_prefix(name)).resolve()!=root/'install'/name:raise SystemExit('Wrong overlay: '+name)
for name,exe in [('crazyflie_server_cpp','crazyflie_server'),('motion_capture_tracking','motion_capture_tracking_node')]:
    target=Path(get_package_prefix(name))/'lib'/name/exe
    if not target.is_file():raise SystemExit('Missing executable '+str(target))
print('PASS: actual ROS interfaces and backend binaries; no ROS node/radio connection created')
PY
fi
