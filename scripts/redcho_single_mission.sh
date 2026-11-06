#!/usr/bin/env bash
set -eo pipefail
set +u
source "$(dirname -- "${BASH_SOURCE[0]}")/environment.bash"
cd "$SAFE_REDCHO_HW_ROOT"
case "${2:-help}" in
  init|offline|help) ;;
  *)
    [[ -f install/local_setup.bash ]] || { echo 'Build first: bash scripts/build.sh' >&2; exit 2; }
    source scripts/ros_environment.bash
    unset QT_QPA_PLATFORM
    ;;
esac
exec /usr/bin/python3 -s scripts/single_run.py "$@"
