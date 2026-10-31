#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/environment.bash"
cd "$SAFE_REDCHO_HW_ROOT"
case "${1:-help}" in offline|init|help) ;; *)
  [[ -f install/local_setup.bash ]] || { echo 'Build first: bash scripts/build.sh' >&2; exit 2; }
  source scripts/ros_environment.bash
  # Use the desktop binding selected by the user for live viewers.
  unset QT_QPA_PLATFORM
  ;;
esac
exec /usr/bin/python3 scripts/run.py "$@"
