#!/usr/bin/env bash
export SAFE_REDCHO_HW_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONNOUSERSITE=1
export PYTHONDONTWRITEBYTECODE=1
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-offscreen}"
export MPLBACKEND=Agg
export MPLCONFIGDIR="${MPLCONFIGDIR:-$SAFE_REDCHO_HW_ROOT/results/.matplotlib}"
mkdir -p "$MPLCONFIGDIR"
for package in "$SAFE_REDCHO_HW_ROOT"/src/safe_redcho*; do
  if [[ -d "$package" ]]; then export PYTHONPATH="$package${PYTHONPATH:+:$PYTHONPATH}"; fi
done

if [[ -d "$SAFE_REDCHO_HW_ROOT/.venv/lib/python3.12/site-packages" ]]; then export PYTHONPATH="$SAFE_REDCHO_HW_ROOT/.venv/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}"; fi
