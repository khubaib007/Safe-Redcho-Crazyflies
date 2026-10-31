#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/environment.bash"
cd "$SAFE_REDCHO_HW_ROOT"
exec /usr/bin/python3 scripts/test.py
