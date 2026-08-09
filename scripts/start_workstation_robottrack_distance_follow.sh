#!/usr/bin/env bash
set -euo pipefail
umask 077

# Dedicated opt-in fixed-distance profile.  The original RobotTrack launcher
# remains RGB-only; this wrapper adds aligned D435i depth and an initial metric
# setpoint while reusing the same persistent stack and command ownership path.

readonly ROOT="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
readonly PERSISTENT_LAUNCHER="$ROOT/scripts/start_workstation_persistent_voice_nav2.sh"

die() {
  local status="$1"
  shift
  printf '%s\n' "$*" >&2
  exit "$status"
}

[[ "$#" -eq 0 ]] || die 2 "this launcher accepts no arguments"
[[ -f "$PERSISTENT_LAUNCHER" && ! -L "$PERSISTENT_LAUNCHER" ]] \
  || die 2 "persistent full-stack launcher is missing or is a symlink"

export GO2_ROBOTTRACK_MODE=true
export GO2_ROBOTTRACK_DISTANCE_MODE=true
export GO2_ROBOTTRACK_LEAN_MODE=true
export ROBOTTRACK_TARGET_DISTANCE_M="${ROBOTTRACK_TARGET_DISTANCE_M:-5.0}"
export ROBOTTRACK_SERVER_URL="${ROBOTTRACK_SERVER_URL:-http://127.0.0.1:5801/eval_dual}"
export ROBOTTRACK_INSTRUCTION="${ROBOTTRACK_INSTRUCTION:-Follow the person ahead}"

exec bash "$PERSISTENT_LAUNCHER"
