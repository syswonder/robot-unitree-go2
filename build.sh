#!/usr/bin/env bash
set -euo pipefail

DEPLOY_DIR="$(cd "$(dirname "$0")" && pwd)"
MANIFEST="${ROBONIX_MANIFEST:-$DEPLOY_DIR/robonix_manifest.yaml}"
export ROBONIX_DEPLOY_DIR="$DEPLOY_DIR"
export PATH="$HOME/.local/bin:$HOME/.cargo/bin:$PATH"
export UV_CACHE_DIR="${UV_CACHE_DIR:-$DEPLOY_DIR/.cache/uv}"
export PIP_CACHE_DIR="${PIP_CACHE_DIR:-$DEPLOY_DIR/.cache/pip}"

if [[ -f "$DEPLOY_DIR/.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "$DEPLOY_DIR/.env"
  set +a
fi

# Keep generated packages on the same loopback-only control-plane contract as
# runtime. The upstream compatibility gate separately requires Mapping to
# honor MAPPING_ENABLE_VIZ=false before it may inspect X11 or invoke xhost.
export ROBONIX_PROVIDER_BIND_HOST=127.0.0.1
export ROBONIX_ADVERTISE_HOST=127.0.0.1
export SPEECH_BIND_ADDR=127.0.0.1
export MAPPING_ENABLE_VIZ=false
unset DISPLAY

export GO2_MAP_ID="${GO2_MAP_ID:-lab_go2}"
export GO2_MAP_MODE="${GO2_MAP_MODE:-localization}"
export SEMANTIC_LANDMARKS_FILE="${SEMANTIC_LANDMARKS_FILE:-config/semantic_landmarks.yaml}"
export SPEECH_BACKEND="${SPEECH_BACKEND:-local}"
export GO2_DASHBOARD_PORT="${GO2_DASHBOARD_PORT:-8092}"
export GO2_ALLOW_MOTION="${GO2_ALLOW_MOTION:-false}"
export GO2_OPERATOR_PRESENT="${GO2_OPERATOR_PRESENT:-false}"
export GO2_SAFETY_ACK="${GO2_SAFETY_ACK:-}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export UNITREE_SDK2_DIR="${UNITREE_SDK2_DIR:-$DEPLOY_DIR/third_party/unitree_sdk2}"
export UNITREE_ROS2_SETUP="${UNITREE_ROS2_SETUP:-$DEPLOY_DIR/rbnx-build/unitree_ros2/install/setup.bash}"

[[ "$ROS_DOMAIN_ID" =~ ^[0-9]+$ ]] && (( ROS_DOMAIN_ID <= 232 )) || {
  echo "ROS_DOMAIN_ID must be an integer in 0..232" >&2
  exit 1
}

for command in bzip2 cmake colcon curl docker g++ git python3 rbnx tar uv; do
  command -v "$command" >/dev/null 2>&1 || {
    echo "missing dependency: $command" >&2
    exit 1
  }
done

[[ -f /opt/ros/humble/setup.bash ]] || {
  echo "missing /opt/ros/humble/setup.bash; ROS 2 Humble is required" >&2
  exit 1
}
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

[[ -f "$UNITREE_SDK2_DIR/CMakeLists.txt" ]] || {
  echo "missing official unitree_sdk2 submodule; run: git submodule update --init --recursive" >&2
  exit 1
}
[[ -f "$DEPLOY_DIR/third_party/unitree_ros2/cyclonedds_ws/src/unitree/unitree_go/package.xml" ]] || {
  echo "missing official unitree_ros2 submodule; run: git submodule update --init --recursive" >&2
  exit 1
}

"$DEPLOY_DIR/scripts/validate_offline.sh"

colcon build \
  --base-paths "$DEPLOY_DIR/third_party/unitree_ros2/cyclonedds_ws/src/unitree" \
  --build-base "$DEPLOY_DIR/rbnx-build/unitree_ros2/build" \
  --install-base "$DEPLOY_DIR/rbnx-build/unitree_ros2/install" \
  --log-base "$DEPLOY_DIR/rbnx-build/unitree_ros2/log" \
  --packages-select unitree_api unitree_go \
  --merge-install \
  --cmake-args -DBUILD_TESTING=OFF

set +u
# shellcheck disable=SC1090
source "$UNITREE_ROS2_SETUP"
set -u

rbnx build -f "$MANIFEST" "$@"
bash "$DEPLOY_DIR/scripts/verify_upstream_compatibility.sh"
