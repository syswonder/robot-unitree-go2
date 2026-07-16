#!/usr/bin/env bash
set -euo pipefail

DEPLOY_DIR="$(cd "$(dirname "$0")" && pwd)"
MANIFEST="${ROBONIX_MANIFEST:-$DEPLOY_DIR/robonix_manifest.yaml}"
export ROBONIX_DEPLOY_DIR="$DEPLOY_DIR"
export PATH="$HOME/.local/bin:$HOME/.cargo/bin:$PATH"

if [[ -f "$DEPLOY_DIR/.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "$DEPLOY_DIR/.env"
  set +a
fi

# The deployment owns every non-DDS bind address. Local configuration cannot
# widen the control, capability, audio, or UI surface onto a LAN interface.
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
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export UNITREE_SDK2_DIR="${UNITREE_SDK2_DIR:-$DEPLOY_DIR/third_party/unitree_sdk2}"
export UNITREE_ROS2_SETUP="${UNITREE_ROS2_SETUP:-$DEPLOY_DIR/rbnx-build/unitree_ros2/install/setup.bash}"

[[ "$ROS_DOMAIN_ID" =~ ^[0-9]+$ ]] && (( ROS_DOMAIN_ID <= 232 )) || {
  echo "ROS_DOMAIN_ID must be an integer in 0..232" >&2
  exit 1
}

for command in docker git ip nmcli rbnx; do
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
if [[ -f "$UNITREE_ROS2_SETUP" ]]; then
  # shellcheck disable=SC1090
  source "$UNITREE_ROS2_SETUP"
else
  echo "missing Unitree ROS 2 message overlay: $UNITREE_ROS2_SETUP; run ./build.sh first" >&2
  exit 1
fi
set -u

bash "$DEPLOY_DIR/scripts/verify_upstream_compatibility.sh"

mkdir -p "$DEPLOY_DIR/logs" "$DEPLOY_DIR/rbnx-build/run"
chmod 700 "$DEPLOY_DIR/rbnx-build/run"

exec rbnx boot --no-update-check -f "$MANIFEST" "$@"
