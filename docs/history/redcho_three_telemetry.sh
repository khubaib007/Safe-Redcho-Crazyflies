#!/usr/bin/env bash
# Recorder wrapper only: no arming, flight services, setpoints or radio connections.
set -eo pipefail
set +u
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$workspace/scripts/environment.bash"
export PYTHONNOUSERSITE=1
action="${1:-help}"
if [[ "$#" -gt 0 ]]; then shift; fi
config="$workspace/config/telemetry_three.yaml"
case "$action" in
  all)
    exec ros2 launch safe_redcho_three_telemetry all.launch.py config_file:="$config" "$@" ;;
  flight|reference|redcho|control)
    exec ros2 run safe_redcho_three_telemetry "${action}_telemetry" --config "$config" "$@" ;;
  *) printf '%s\n' 'Actions: all | flight | reference | redcho | control' ;;
esac
