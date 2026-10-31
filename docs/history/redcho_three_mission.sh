#!/usr/bin/env bash
set -eo pipefail
# ROS-generated setup files are not nounset-compatible. Deliberately do not use -u.
set +u
mission_workspace="${SAFE_REDCHO_HW_ROOT:-$HOME/safe_redcho_hardware}"
if [[ ! -d "$mission_workspace" ]]; then
    echo "Workspace not found: $mission_workspace" >&2
    exit 2
fi
cd "$mission_workspace"
if [[ -f scripts/environment.bash ]]; then source scripts/environment.bash; fi
set +u
if [[ -f install/setup.bash ]]; then source install/setup.bash; fi
set +u
mission_profile="${SAFE_REDCHO_MISSION_CONFIG:-$mission_workspace/config/three_mission.yaml}"
mission_mode="${1:-help}"
case "$mission_mode" in
    mocap|backend|agents)
        exec ros2 launch safe_redcho_three_mission "${mission_mode}.launch.py" config_file:="$mission_profile"
        ;;
    doctor)
        exec ros2 run safe_redcho_three_mission mission_doctor --config "$mission_profile"
        ;;
    capture)
        exec ros2 run safe_redcho_three_mission mission_capture --config "$mission_profile"
        ;;
    check|acquire|fly)
        mission_run="$mission_workspace/results/three_mission_${mission_mode}_$(date -u +%Y%m%dT%H%M%S)_$$"
        mission_args=(--config "$mission_profile" --output "$mission_run")
        if [[ "$mission_mode" != check ]]; then mission_args+=(--fly); fi
        if [[ "$mission_mode" == acquire ]]; then mission_args+=(--acquire-only); fi
        exec ros2 run safe_redcho_three_mission mission_coordinator "${mission_args[@]}"
        ;;
    offline|preview)
        mission_run="$mission_workspace/results/three_mission_offline_$(date -u +%Y%m%dT%H%M%S)_$$"
        mission_args=(--config "$mission_profile" --output "$mission_run")
        if [[ "$mission_mode" == preview ]]; then mission_args+=(--captured); fi
        exec ros2 run safe_redcho_three_mission mission_preview "${mission_args[@]}"
        ;;
    observe)
        shift
        exec ros2 launch safe_redcho_three_mission observe.launch.py telemetry_config:="$mission_workspace/config/three_mission_telemetry.yaml" plot_config:="$mission_workspace/config/three_mission_liveplots.yaml" "$@"
        ;;
    *)
        echo 'Usage: bash +u scripts/redcho_three_mission.sh {doctor|mocap|capture|backend|agents|check|acquire|fly|offline|preview|observe}'
        echo 'check/offline never arm. acquire flies through formation acquisition, then lands. fly runs the complete mission.'
        ;;
esac
