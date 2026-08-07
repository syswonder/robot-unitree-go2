#!/usr/bin/env bash
set -euo pipefail
umask 077

# Thin opt-in wrapper for the existing persistent full-stack launcher.  This
# file selects the RobotTrack manifest only; all established map, localization,
# ClassicWalk, process ownership, and runtime behavior stay in that launcher.

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
export ROBOTTRACK_SERVER_URL="${ROBOTTRACK_SERVER_URL:-http://127.0.0.1:5801/eval_dual}"
export ROBOTTRACK_INSTRUCTION="${ROBOTTRACK_INSTRUCTION:-Follow the person ahead}"

exec bash "$PERSISTENT_LAUNCHER"
