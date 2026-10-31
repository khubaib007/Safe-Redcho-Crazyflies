#!/usr/bin/env bash
# All runtime actions live inside the workspace. No Downloads runtime dependency.
set -eo pipefail
set +u
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$workspace/scripts/environment.bash"
export PYTHONNOUSERSITE=1
action="${1:-help}"
if [[ "$#" -gt 0 ]];then shift;fi
profile="$workspace/config/redcho_hardware.yaml"
case "$action" in
  mocap|backend|agents)
    exec ros2 launch safe_redcho_flight "${action}.launch.py" config_file:="$profile" "$@" ;;
  capture)
    exec ros2 run safe_redcho_bringup fleet_capture --config "$workspace/config/fleet_redcho.yaml" "$@" ;;
  health)
    exec python3 "$workspace/scripts/redcho_health_check.py" --config "$profile" "$@" ;;
  check|hover|fly)
    output="$workspace/results/redcho_${action}_$(date -u +%Y%m%dT%H%M%SZ)"
    flags=()
    if [[ "$action" == hover ]];then flags=(--fly --hover-only);fi
    if [[ "$action" == fly ]];then flags=(--fly);fi
    exec ros2 run safe_redcho_flight redcho_mission --config "$profile" --output "$output" "${flags[@]}" "$@" ;;
  emergency)
    exec ros2 service call /all/emergency std_srvs/srv/Empty '{}' ;;
  *) printf '%s\n' 'Actions: mocap | capture | backend | agents | health | check | hover | fly | emergency' ;;
esac
