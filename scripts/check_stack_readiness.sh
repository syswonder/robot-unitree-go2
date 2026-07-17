#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_ROOT="$(cd "$ROOT/../.." && pwd)"

echo "================================================================"
echo " ROBONIX GO2 READ-ONLY FULL-STACK READINESS GATE"
echo " No task, action goal, robot command, or ROS write is performed."
echo " Every ROS read is bounded by timeout; UNKNOWN means NOT READY."
echo "================================================================"

if [[ -f "$ROOT/.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "$ROOT/.env"
  set +a
fi

export GO2_MAP_ID="${GO2_MAP_ID:-lab_go2}"
export GO2_MAP_MODE="${GO2_MAP_MODE:-localization}"
export SEMANTIC_LANDMARKS_FILE="${SEMANTIC_LANDMARKS_FILE:-config/semantic_landmarks.yaml}"
export SEMANTIC_INTENT_PORT="${SEMANTIC_INTENT_PORT:-18080}"
export GO2_DASHBOARD_PORT="${GO2_DASHBOARD_PORT:-8092}"
export VLM_MODEL="${VLM_MODEL:-go2-semantic-router}"
export GO2_ALLOW_MOTION="${GO2_ALLOW_MOTION:-false}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

if [[ "$SEMANTIC_LANDMARKS_FILE" != /* ]]; then
  SEMANTIC_LANDMARKS_FILE="$ROOT/$SEMANTIC_LANDMARKS_FILE"
fi

export PATH="$WORKSPACE_ROOT/.tools/rbnx/bin:$HOME/.local/bin:$HOME/.cargo/bin:$PATH"
export ROS_LOG_DIR="$ROOT/logs/readiness/ros"
export RCUTILS_LOGGING_DIRECTORY="$ROS_LOG_DIR"
mkdir -p "$ROS_LOG_DIR"
chmod 700 "$ROOT/logs/readiness" "$ROS_LOG_DIR"

[[ -f /opt/ros/humble/setup.bash ]] || {
  echo "missing /opt/ros/humble/setup.bash" >&2
  exit 1
}
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
for overlay in \
  "$ROOT/rbnx-build/unitree_ros2/install/setup.bash" \
  "$ROOT/rbnx-boot/cache/service-map-rbnx/rbnx-build/codegen/ros2_idl/install/setup.bash"
do
  if [[ -f "$overlay" ]]; then
    # shellcheck disable=SC1090
    source "$overlay"
  fi
done
set -u

command -v timeout >/dev/null 2>&1 || {
  echo "timeout is required" >&2
  exit 1
}
command -v ros2 >/dev/null 2>&1 || {
  echo "ros2 is required" >&2
  exit 1
}
command -v rbnx >/dev/null 2>&1 || {
  echo "rbnx is required" >&2
  exit 1
}

exec python3 "$ROOT/scripts/stack_readiness.py" \
  --deploy-dir "$ROOT" \
  --landmarks-file "$SEMANTIC_LANDMARKS_FILE" \
  --map-id "$GO2_MAP_ID" \
  --map-mode "$GO2_MAP_MODE" \
  --model "$VLM_MODEL" \
  --semantic-port "$SEMANTIC_INTENT_PORT" \
  --dashboard-port "$GO2_DASHBOARD_PORT" \
  --allow-motion "$GO2_ALLOW_MOTION" \
  "$@"
