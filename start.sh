#!/usr/bin/env bash
set -euo pipefail

DEPLOY_DIR="$(cd "$(dirname "$0")" && pwd)"
MANIFEST="${ROBONIX_MANIFEST:-$DEPLOY_DIR/robonix_manifest.yaml}"
export ROBONIX_DEPLOY_DIR="$DEPLOY_DIR"
export PATH="$HOME/.local/bin:$HOME/.cargo/bin:$PATH"
WORKSPACE_ROOT="$(cd "$DEPLOY_DIR/../.." 2>/dev/null && pwd)"
WORKSPACE_TOOLS_BIN="$WORKSPACE_ROOT/.tools/rbnx/bin"
if ! command -v rbnx >/dev/null 2>&1 && [[ -x "$WORKSPACE_TOOLS_BIN/rbnx" ]]; then
  export PATH="$WORKSPACE_TOOLS_BIN:$PATH"
fi
if [[ -f "$DEPLOY_DIR/.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "$DEPLOY_DIR/.env"
  set +a
fi

# Local configuration cannot redirect rbnx to ~/.robonix or another checkout.
export ROBONIX_DEPLOY_DIR="$DEPLOY_DIR"
export ROBONIX_HOME="$WORKSPACE_ROOT/.tools/robonix-home"
command -v python3 >/dev/null 2>&1 || {
  echo "python3 is required to validate the workspace-local Robonix config" >&2
  exit 1
}
python3 "$DEPLOY_DIR/scripts/validate_robonix_home.py" \
  "$ROBONIX_HOME/config.yaml" \
  "$WORKSPACE_ROOT/upstream/robonix-go2-build"

# ModelScope must never fall back to ~/.cache or ~/.modelscope. The online
# FunASR model is an audited local artifact; runtime is not allowed to download
# a replacement or continue with a partial directory.
export MODELSCOPE_CACHE="$DEPLOY_DIR/.cache/modelscope"
export MODELSCOPE_CREDENTIALS_PATH="$DEPLOY_DIR/.cache/modelscope/credentials"
export GO2_FUNASR_MODEL_PATH="${GO2_FUNASR_MODEL_PATH:-$MODELSCOPE_CACHE/models/iic/speech_paraformer-large_asr_nat-zh-cn-16k-common-vocab8404-online}"
[[ "$GO2_FUNASR_MODEL_PATH" == /* ]] || {
  echo "GO2_FUNASR_MODEL_PATH must be an absolute path" >&2
  exit 1
}
for required_model_file in model.pt config.yaml; do
  [[ -s "$GO2_FUNASR_MODEL_PATH/$required_model_file" ]] || {
    echo "missing or empty audited FunASR artifact: $GO2_FUNASR_MODEL_PATH/$required_model_file" >&2
    echo "runtime download fallback is disabled; populate the workspace model cache first" >&2
    exit 1
  }
done

# The deployment owns every non-DDS bind address. Local configuration cannot
# widen the control, capability, audio, or UI surface onto a LAN interface.
export ROBONIX_PROVIDER_BIND_HOST=127.0.0.1
export ROBONIX_ADVERTISE_HOST=127.0.0.1
export SPEECH_BIND_ADDR=127.0.0.1
export MAPPING_ENABLE_VIZ=false
export MAPPING_WEBUI_HOST=127.0.0.1
unset DISPLAY

export GO2_MAP_ID="${GO2_MAP_ID:-lab_go2}"
export GO2_MAP_MODE="${GO2_MAP_MODE:-localization}"
export SEMANTIC_LANDMARKS_FILE="${SEMANTIC_LANDMARKS_FILE:-config/semantic_landmarks.yaml}"
export SPEECH_BACKEND="${SPEECH_BACKEND:-local}"
export GO2_DASHBOARD_PORT="${GO2_DASHBOARD_PORT:-8092}"
export GO2_RUNTIME_PLACEMENT="${GO2_RUNTIME_PLACEMENT:-workstation-local}"
export GO2_ALLOW_MOTION="${GO2_ALLOW_MOTION:-false}"
export GO2_OPERATOR_PRESENT="${GO2_OPERATOR_PRESENT:-false}"
export GO2_SAFETY_ACK="${GO2_SAFETY_ACK:-}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export UNITREE_SDK2_DIR="${UNITREE_SDK2_DIR:-$DEPLOY_DIR/third_party/unitree_sdk2}"
export UNITREE_ROS2_SETUP="${UNITREE_ROS2_SETUP:-$DEPLOY_DIR/rbnx-build/unitree_ros2/install/setup.bash}"

[[ "$ROS_DOMAIN_ID" =~ ^[0-9]+$ ]] && (( ROS_DOMAIN_ID <= 232 )) || {
  echo "ROS_DOMAIN_ID must be an integer in 0..232" >&2
  exit 1
}

case "$GO2_RUNTIME_PLACEMENT" in
  workstation-local)
    EXPECTED_SENSOR_SOURCE_MODE=local
    OWNERSHIP_PREFLIGHT=workstation-local
    START_UI_CLIENT_ONLY=false
    echo "Runtime placement: workstation owns chassis, sensors, description and UI"
    ;;
  workstation-full-nx-sensors)
    EXPECTED_SENSOR_SOURCE_MODE=external
    OWNERSHIP_PREFLIGHT=workstation-full-nx-sensors
    START_UI_CLIENT_ONLY=false
    echo "================================================================"
    echo " NX SENSOR OWNER - WORKSTATION SENSOR PUBLISHERS DISABLED"
    echo " Robonix will only wait for and register the standardized topics."
    echo "================================================================"
    ;;
  workstation-ui-nx-full)
    EXPECTED_SENSOR_SOURCE_MODE=external
    OWNERSHIP_PREFLIGHT=workstation-ui-nx-full
    START_UI_CLIENT_ONLY=true
    export GO2_ALLOW_MOTION=false
    export GO2_DASHBOARD_BROWSER_VOICE_ENABLED=0
    echo "================================================================"
    echo " NX FULL READ-ONLY OWNER - WORKSTATION UI/CLIENT ONLY"
    echo " No local chassis, sensor, description, Mapping or Nav2 provider starts."
    echo "================================================================"
    ;;
  *)
    echo "unsupported GO2_RUNTIME_PLACEMENT: $GO2_RUNTIME_PLACEMENT" >&2
    exit 1
    ;;
esac
if [[ -n "${GO2_SENSOR_SOURCE_MODE:-}" \
  && "$GO2_SENSOR_SOURCE_MODE" != "$EXPECTED_SENSOR_SOURCE_MODE" ]]; then
  echo "GO2_SENSOR_SOURCE_MODE conflicts with GO2_RUNTIME_PLACEMENT" >&2
  exit 1
fi
export GO2_SENSOR_SOURCE_MODE="$EXPECTED_SENSOR_SOURCE_MODE"

required_commands=(flock ip nmcli timeout)
if [[ "$START_UI_CLIENT_ONLY" == false ]]; then
  required_commands+=(docker git rbnx)
fi
for command in "${required_commands[@]}"; do
  command -v "$command" >/dev/null 2>&1 || {
    echo "$command is not installed or not on PATH" >&2
    exit 1
  }
done

[[ -n "${GO2_NETWORK_INTERFACE:-}" ]] || {
  echo "GO2_NETWORK_INTERFACE is unset; copy .env.example to .env and set the dedicated wired NIC" >&2
  exit 1
}
[[ "$GO2_NETWORK_INTERFACE" =~ ^[A-Za-z0-9_.:-]+$ ]] || {
  echo "GO2_NETWORK_INTERFACE contains unsupported characters" >&2
  exit 1
}
ip link show dev "$GO2_NETWORK_INTERFACE" >/dev/null 2>&1 || {
  echo "network interface does not exist: $GO2_NETWORK_INTERFACE" >&2
  exit 1
}
INTERFACE_SYSFS="$(readlink -f -- "/sys/class/net/$GO2_NETWORK_INTERFACE")"
[[ -n "$INTERFACE_SYSFS" && "$INTERFACE_SYSFS" != *"/virtual/"* ]] || {
  echo "GO2 network interface must be a physical device: $GO2_NETWORK_INTERFACE" >&2
  exit 1
}
[[ ! -d "/sys/class/net/$GO2_NETWORK_INTERFACE/wireless" ]] || {
  echo "GO2 network interface must not be Wi-Fi: $GO2_NETWORK_INTERFACE" >&2
  exit 1
}
[[ "$(cat -- "/sys/class/net/$GO2_NETWORK_INTERFACE/operstate")" == "up" ]] || {
  echo "GO2 network interface is not UP: $GO2_NETWORK_INTERFACE" >&2
  exit 1
}
mapfile -t GO2_IPV4_ADDRESSES < <(
  ip -o -4 addr show dev "$GO2_NETWORK_INTERFACE" \
    | awk '{print $4}'
)
if (( ${#GO2_IPV4_ADDRESSES[@]} != 1 )) \
  || [[ "${GO2_IPV4_ADDRESSES[0]:-}" != "192.168.123.99/24" ]]; then
  echo "GO2 interface must have exactly 192.168.123.99/24 and no other IPv4 address" >&2
  exit 1
fi
mapfile -t GO2_IPV6_ADDRESSES < <(
  ip -o -6 addr show dev "$GO2_NETWORK_INTERFACE" \
    | awk '{print $4}'
)
if (( ${#GO2_IPV6_ADDRESSES[@]} != 0 )); then
  echo "GO2 interface must have IPv6 disabled and no IPv6 addresses" >&2
  exit 1
fi
if ip -4 route show default dev "$GO2_NETWORK_INTERFACE" | grep -q .; then
  echo "GO2 interface must not have an IPv4 default route" >&2
  exit 1
fi
if ip -6 route show default dev "$GO2_NETWORK_INTERFACE" | grep -q .; then
  echo "GO2 interface must not have an IPv6 default route" >&2
  exit 1
fi
if ! GO2_NM_STATE="$(
  nmcli --get-values IP4.GATEWAY,IP4.DNS,IP6.GATEWAY,IP6.DNS \
    device show "$GO2_NETWORK_INTERFACE" 2>/dev/null \
    | sed '/^[[:space:]]*$/d'
)"; then
  echo "could not verify NetworkManager gateway/DNS state for the Go2 interface" >&2
  exit 1
fi
[[ -z "$GO2_NM_STATE" ]] || {
  echo "GO2 interface must not have any NetworkManager gateway or DNS entry" >&2
  exit 1
}

case "${GO2_ALLOW_MOTION,,}" in
  1|true|yes|on)
    [[ "${GO2_OPERATOR_PRESENT,,}" =~ ^(1|true|yes|on)$ ]] || {
      echo "motion requested but GO2_OPERATOR_PRESENT is not true" >&2
      exit 1
    }
    [[ "$GO2_SAFETY_ACK" == "I_UNDERSTAND_GO2_CAN_MOVE" ]] || {
      echo "motion requested but GO2_SAFETY_ACK does not match the required phrase" >&2
      exit 1
    }
    [[ -n "${GO2_ALLOWED_MODES:-}" ]] || {
      echo "motion requested but GO2_ALLOWED_MODES has not been recorded by the read-only audit" >&2
      exit 1
    }
    export GO2_ALLOWED_MODES
    echo "================================================================"
    echo " GO2 MOTION TRANSPORT ENABLED, BUT CHASSIS IS STILL DISARMED"
    echo " An operator must explicitly call the adapter arm service."
    echo "================================================================"
    ;;
  *)
    export GO2_ALLOW_MOTION=false
    echo "================================================================"
    echo " GO2 MOTION LOCKED - READ-ONLY STATE/SENSOR MODE"
    echo " Boot will not arm, stand, sit, or move the robot."
    echo "================================================================"
    ;;
esac

GO2_CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$GO2_NETWORK_INTERFACE\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>"
if [[ -n "${CYCLONEDDS_URI:-}" && "$CYCLONEDDS_URI" != "$GO2_CYCLONEDDS_URI" ]]; then
  echo "Ignoring inherited CYCLONEDDS_URI; this deployment owns the dedicated Go2 NIC binding" >&2
fi
export CYCLONEDDS_URI="$GO2_CYCLONEDDS_URI"

[[ -f /opt/ros/humble/setup.bash ]] || {
  echo "missing /opt/ros/humble/setup.bash" >&2
  exit 1
}
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [[ "$START_UI_CLIENT_ONLY" == true ]]; then
  : # The dashboard consumes only standard ROS interfaces.
elif [[ -f "$UNITREE_ROS2_SETUP" ]]; then
  # shellcheck disable=SC1090
  source "$UNITREE_ROS2_SETUP"
else
  echo "missing Unitree ROS 2 message overlay: $UNITREE_ROS2_SETUP; run ./build.sh first" >&2
  exit 1
fi
set -u

command -v ros2 >/dev/null 2>&1 || {
  echo "ros2 is not available after sourcing ROS 2 Humble" >&2
  exit 1
}
mkdir -p "$DEPLOY_DIR/logs" "$DEPLOY_DIR/rbnx-build/run"
chmod 700 "$DEPLOY_DIR/rbnx-build/run"
# shellcheck disable=SC1091
source "$DEPLOY_DIR/scripts/runtime_lease.sh"
go2_runtime_lease_acquire "$DEPLOY_DIR/rbnx-build/run" "$GO2_RUNTIME_PLACEMENT"
bash "$DEPLOY_DIR/scripts/check_runtime_ownership.sh" "$OWNERSHIP_PREFLIGHT" pre

if [[ "$START_UI_CLIENT_ONLY" == true ]]; then
  exec bash "$DEPLOY_DIR/scripts/start_ui_client_only.sh"
fi

bash "$DEPLOY_DIR/scripts/verify_upstream_compatibility.sh"

SEMANTIC_ROUTER_PID=""
SEMANTIC_ROUTER_START_TICKS=""
SEMANTIC_ROUTER_TOKEN=""
SEMANTIC_ROUTER_LEASE="$DEPLOY_DIR/rbnx-build/run/semantic-intent-router.lease"
SEMANTIC_ROUTER_LOCK="$DEPLOY_DIR/rbnx-build/run/semantic-intent-router.lock"
RBNX_BOOT_PID=""

process_is_same_and_live() {
  local pid="$1" expected_start="$2" stat rest
  local -a fields
  IFS= read -r stat < "/proc/${pid}/stat" 2>/dev/null || return 1
  rest="${stat#*) }"
  [[ "$rest" != "$stat" ]] || return 1
  read -r -a fields <<< "$rest"
  [[ "${fields[0]:-}" != Z && "${fields[19]:-}" == "$expected_start" ]]
}

remove_own_semantic_lease() {
  local recorded_token=""
  if [[ -r "$SEMANTIC_ROUTER_LEASE" ]]; then
    recorded_token="$(sed -n 's/^token=//p' "$SEMANTIC_ROUTER_LEASE" 2>/dev/null || true)"
  fi
  if [[ -n "$SEMANTIC_ROUTER_TOKEN" \
    && "$recorded_token" == "$SEMANTIC_ROUTER_TOKEN" ]]; then
    rm -f -- "$SEMANTIC_ROUTER_LEASE"
  fi
}

terminate_semantic_router() {
  if [[ -n "$SEMANTIC_ROUTER_PID" ]] \
    && process_is_same_and_live "$SEMANTIC_ROUTER_PID" "$SEMANTIC_ROUTER_START_TICKS"; then
    kill -TERM "$SEMANTIC_ROUTER_PID" 2>/dev/null || true
  fi
  if [[ -n "$SEMANTIC_ROUTER_PID" ]]; then
    wait "$SEMANTIC_ROUTER_PID" 2>/dev/null || true
  fi
  remove_own_semantic_lease
}

terminate_boot() {
  [[ -n "$RBNX_BOOT_PID" ]] || return 0
  trap - EXIT INT TERM
  if kill -0 "$RBNX_BOOT_PID" 2>/dev/null; then
    kill -TERM "$RBNX_BOOT_PID" 2>/dev/null || true
  fi
  wait "$RBNX_BOOT_PID" 2>/dev/null || true
}

cleanup_runtime() {
  trap - EXIT INT TERM
  terminate_boot
  terminate_semantic_router
}
trap cleanup_runtime EXIT INT TERM

start_semantic_router() {
  local owner_start temporary expected_vlm_url deadline stable
  export SEMANTIC_INTENT_HOST=127.0.0.1
  export SEMANTIC_INTENT_PORT="${SEMANTIC_INTENT_PORT:-18080}"
  [[ "$SEMANTIC_INTENT_PORT" =~ ^[0-9]+$ ]] \
    && (( SEMANTIC_INTENT_PORT >= 1 && SEMANTIC_INTENT_PORT <= 65535 )) || {
      echo "SEMANTIC_INTENT_PORT must be an integer in 1..65535" >&2
      return 75
    }
  expected_vlm_url="http://127.0.0.1:${SEMANTIC_INTENT_PORT}/v1"
  if [[ -n "${VLM_BASE_URL:-}" && "$VLM_BASE_URL" != "$expected_vlm_url" ]]; then
    echo "VLM_BASE_URL conflicts with the deployment-owned semantic router" >&2
    return 75
  fi
  export VLM_BASE_URL="$expected_vlm_url"
  export VLM_API_KEY=local-no-secret
  export VLM_MODEL=go2-semantic-router

  exec {SEMANTIC_ROUTER_LOCK_FD}>"$SEMANTIC_ROUTER_LOCK"
  if ! flock --exclusive --nonblock "$SEMANTIC_ROUTER_LOCK_FD"; then
    echo "another semantic intent router launcher already holds the runtime lock" >&2
    return 75
  fi
  SEMANTIC_ROUTER_TOKEN="$(go2_runtime_new_token)"
  owner_start="$(go2_runtime_process_start_ticks "$$")"
  bash "$DEPLOY_DIR/packages/semantic_intent_router/scripts/start.sh" &
  SEMANTIC_ROUTER_PID=$!
  SEMANTIC_ROUTER_START_TICKS="$(go2_runtime_process_start_ticks "$SEMANTIC_ROUTER_PID")" || {
    echo "semantic intent router exited before identity capture" >&2
    return 75
  }
  temporary="$(mktemp "${SEMANTIC_ROUTER_LEASE}.tmp.XXXXXX")"
  chmod 0600 "$temporary"
  {
    printf 'format=go2-semantic-router-lease-v1\n'
    printf 'token=%s\n' "$SEMANTIC_ROUTER_TOKEN"
    printf 'owner_pid=%s\n' "$$"
    printf 'owner_start_ticks=%s\n' "$owner_start"
    printf 'child_pid=%s\n' "$SEMANTIC_ROUTER_PID"
    printf 'child_start_ticks=%s\n' "$SEMANTIC_ROUTER_START_TICKS"
  } > "$temporary"
  mv -f -- "$temporary" "$SEMANTIC_ROUTER_LEASE"

  deadline=$((SECONDS + 15))
  stable=0
  while (( SECONDS <= deadline )); do
    if ! process_is_same_and_live "$SEMANTIC_ROUTER_PID" "$SEMANTIC_ROUTER_START_TICKS"; then
      echo "semantic intent router exited before becoming healthy" >&2
      return 75
    fi
    if python3 "$DEPLOY_DIR/scripts/check_semantic_intent_health.py" \
      "${expected_vlm_url}/models"; then
      stable=$((stable + 1))
      if (( stable >= 3 )); then
        echo "Semantic intent router health gate passed: ${expected_vlm_url}/models"
        return 0
      fi
    else
      stable=0
    fi
    sleep 0.2
  done
  echo "semantic intent router /v1/models health deadline expired" >&2
  return 75
}

start_semantic_router

rbnx boot --no-update-check -f "$MANIFEST" "$@" &
RBNX_BOOT_PID=$!
if ! bash "$DEPLOY_DIR/scripts/check_runtime_ownership.sh" "$OWNERSHIP_PREFLIGHT" post; then
  echo "post-start publisher ownership failed; stopping this boot process" >&2
  exit 74
fi
if ! process_is_same_and_live "$SEMANTIC_ROUTER_PID" "$SEMANTIC_ROUTER_START_TICKS"; then
  echo "semantic intent router exited during Robonix startup" >&2
  exit 75
fi
set +e
EXITED_RUNTIME_PID=""
wait -n -p EXITED_RUNTIME_PID "$RBNX_BOOT_PID" "$SEMANTIC_ROUTER_PID"
RUNTIME_STATUS=$?
set -e
trap - EXIT INT TERM
if [[ "$EXITED_RUNTIME_PID" == "$SEMANTIC_ROUTER_PID" ]]; then
  echo "semantic intent router exited unexpectedly; stopping Robonix boot" >&2
  terminate_boot
  remove_own_semantic_lease
  exit 75
fi
terminate_semantic_router
exit "$RUNTIME_STATUS"
