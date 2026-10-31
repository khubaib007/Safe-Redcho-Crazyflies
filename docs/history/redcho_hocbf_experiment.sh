#!/usr/bin/env bash
set -eo pipefail
set +u
mission_workspace="${SAFE_REDCHO_HW_ROOT:-$HOME/safe_redcho_hardware}"
scenario="${1:-}"
mode="${2:-}"
case "$scenario" in obstacle|interagent|combined) ;; *) echo 'Usage: bash +u scripts/redcho_hocbf_experiment.sh {obstacle|interagent|combined} {mocap|capture|survey|backend|agents|check|preview|observe|acquire|fly|offline}'; exit 2;; esac
cd "$mission_workspace"
export PYTHONNOUSERSITE=1
source scripts/environment.bash
source install/setup.bash
base_profile="$mission_workspace/config/three_mission_${scenario}.yaml"
survey_profile="$mission_workspace/config/generated_hocbf/${scenario}.yaml"
case "$mode" in
  mocap)
    exec ros2 launch safe_redcho_three_mission survey_mocap.launch.py config_file:="$base_profile"
    ;;
  capture|survey)
    exec python3 -m safe_redcho_three_mission.survey --config "$base_profile" --output "$survey_profile"
    ;;
  offline)
    output="$mission_workspace/results/hocbf_${scenario}_$(date -u +%Y%m%dT%H%M%S)_$$"
    exec ros2 run safe_redcho_three_mission mission_preview --config "$base_profile" --starts "$mission_workspace/config/three_mission_${scenario}_fixture.yaml" --output "$output"
    ;;
  backend|agents|check|preview|acquire|fly)
    export SAFE_REDCHO_MISSION_CONFIG
    SAFE_REDCHO_MISSION_CONFIG="$(python3 -m safe_redcho_three_mission.survey --config "$base_profile" --output "$survey_profile" --resolve)"
    echo "Using surveyed profile: $SAFE_REDCHO_MISSION_CONFIG"
    shift 2
    exec bash +u scripts/redcho_three_mission.sh "$mode" "$@"
    ;;
  observe)
    shift 2
    exec bash +u scripts/redcho_three_mission.sh observe "$@"
    ;;
  *) echo 'Invalid mode'; exit 2;;
esac
