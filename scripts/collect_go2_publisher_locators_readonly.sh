#!/usr/bin/env bash
# Bounded publisher-locator evidence collection.  No ROS publishers are made.

set -euo pipefail
umask 077

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
INTERFACE="${1:-${GO2_INTERFACE:-}}"
OUTPUT_DIR="${2:-${PROJECT_ROOT}/logs/go2-readonly/${STAMP}-publisher-locators}"
CAPTURE_SECONDS="${3:-${GO2_LOCATOR_CAPTURE_SECONDS:-20}}"
CAPTURE_MODE="${GO2_LOCATOR_PACKET_CAPTURE:-NO}"
PCAP_PACKET_LIMIT="${GO2_LOCATOR_PACKET_LIMIT:-20000}"

cat <<'BANNER'
=============================================================================
  READ-ONLY / 只读取证：ROS writer GID 与 RTPS 数据包源地址
  默认只执行有界 topic info。只有显式设置
  GO2_LOCATOR_PACKET_CAPTURE=YES 才在指定专用网卡做有界被动抓包。
  不创建 publisher，不调用服务/动作/Unitree API，不修改网络或系统时钟。
=============================================================================
BANNER

for command in ros2 timeout realpath python3 ip awk; do
  command -v "${command}" >/dev/null 2>&1 || {
    printf '缺少依赖：%s。未执行取证。\n' "${command}" >&2
    exit 127
  }
done
for value in "${CAPTURE_SECONDS}" "${PCAP_PACKET_LIMIT}"; do
  if [[ ! "${value}" =~ ^[1-9][0-9]*$ ]]; then
    printf 'capture-seconds/packet-limit 必须是正整数，收到 %q。\n' \
      "${value}" >&2
    exit 2
  fi
done
if (( CAPTURE_SECONDS > 120 || PCAP_PACKET_LIMIT > 100000 )); then
  printf '拒绝无界抓包：capture-seconds<=120，packet-limit<=100000。\n' >&2
  exit 2
fi

OUTPUT_DIR="$(realpath -m -- "${OUTPUT_DIR}")"
case "${OUTPUT_DIR}" in
  "${PROJECT_ROOT}/logs/"*) ;;
  *)
    printf '输出目录必须位于 %s/logs/ 内。\n' "${PROJECT_ROOT}" >&2
    exit 2
    ;;
esac
if [[ -e "${OUTPUT_DIR}" ]]; then
  printf '拒绝覆盖已有取证目录：%s\n' "${OUTPUT_DIR}" >&2
  exit 2
fi
mkdir -m 0700 -p -- "${OUTPUT_DIR}"

topics=(
  /sportmodestate
  /lf/sportmodestate
  /utlidar/cloud
  /utlidar/imu
)
labels=(
  sport_primary
  sport_fallback
  mid360_cloud
  mid360_imu
)

{
  printf 'schema_version=1\n'
  printf 'mode=read-only\n'
  printf 'ros_topic_info_provides_ip=false\n'
  printf 'capture_enabled=%s\n' "${CAPTURE_MODE}"
  printf 'interface=%s\n' "${INTERFACE:-not-specified}"
  printf 'started_utc=%s\n' "${STAMP}"
} >"${OUTPUT_DIR}/metadata.txt"

for index in "${!topics[@]}"; do
  topic="${topics[index]}"
  label="${labels[index]}"
  set +e
  timeout --signal=INT --kill-after=2s 8s \
    ros2 topic info --verbose "${topic}" \
    >"${OUTPUT_DIR}/${label}.topic-info.txt" 2>&1
  status=$?
  set -e
  printf '%s topic-info status=%d\n' "${topic}" "${status}" \
    >>"${OUTPUT_DIR}/commands.status.txt"
done

cat >"${OUTPUT_DIR}/EVIDENCE_LIMITS.txt" <<'LIMITS'
ROS 2 publisher locator evidence limits
=======================================

`ros2 topic info --verbose` reports a DDS/RMW writer GID and QoS.  It does NOT
report or prove the publisher host IP address.  Do not infer an IP from node
name, topic name, interface selection, or participant discovery alone.

An IP conclusion requires a packet capture made in the same publisher session:
match the first 12 GID bytes to the RTPS header GUID prefix, then match GID
bytes 12..15 to the DATA/DATA_FRAG writerEntityId and read that packet's IPv4
source.  A GID may change after publisher/robot restart, so stale topic-info
must never be correlated with a later capture.

PCAP and topic samples are local field evidence under the ignored logs/
directory.  They may expose topology and sensor data; do not commit them.
LIMITS

if [[ "${CAPTURE_MODE}" != "YES" ]]; then
  printf '只完成 GID/QoS 取证。未抓包，因此没有形成 publisher IP 结论。\n'
  printf '证据目录：%s\n' "${OUTPUT_DIR}"
  exit 0
fi

if [[ -z "${INTERFACE}" || ! "${INTERFACE}" =~ ^[A-Za-z0-9_.:-]+$ ]]; then
  printf '抓包模式必须显式提供合法专用网卡名。\n' >&2
  exit 2
fi
if [[ ! -d "/sys/class/net/${INTERFACE}" ]]; then
  printf '网卡不存在：%s\n' "${INTERFACE}" >&2
  exit 2
fi
command -v tcpdump >/dev/null 2>&1 || {
  printf '缺少 tcpdump；未抓包，且不会自动安装。\n' >&2
  exit 127
}

interface_cidr="$({ ip -o -4 addr show dev "${INTERFACE}" || true; } \
  | awk '$4 == "192.168.123.99/24" || $4 == "192.168.123.18/24" {print $4}')"
if [[ -z "${interface_cidr}" ]]; then
  printf '拒绝抓包：%s 必须已由用户配置为 192.168.123.99/24 或 .18/24。\n' \
    "${INTERFACE}" >&2
  exit 2
fi
if ip -4 route show default dev "${INTERFACE}" | awk 'NF {found=1} END {exit !found}'; then
  printf '拒绝抓包：专用 Go2 网卡 %s 上不应存在默认路由。\n' \
    "${INTERFACE}" >&2
  exit 2
fi

pcap_path="${OUTPUT_DIR}/go2-rtps.pcap"
printf '开始 %s 秒、最多 %s 包的被动抓包；不会发送探测包。\n' \
  "${CAPTURE_SECONDS}" "${PCAP_PACKET_LIMIT}"
set +e
timeout --signal=INT --kill-after=3s "${CAPTURE_SECONDS}s" \
  tcpdump -i "${INTERFACE}" -nn -s 0 -c "${PCAP_PACKET_LIMIT}" \
    -Z "$(id -un)" -w "${pcap_path}" \
    'udp and net 192.168.123.0/24' \
    >"${OUTPUT_DIR}/tcpdump.stdout.txt" \
    2>"${OUTPUT_DIR}/tcpdump.stderr.txt" &
capture_pid=$!
set -e

cleanup_capture() {
  if kill -0 "${capture_pid}" 2>/dev/null; then
    kill -INT "${capture_pid}" 2>/dev/null || true
    wait "${capture_pid}" 2>/dev/null || true
  fi
}
trap cleanup_capture INT TERM EXIT

# Each echo creates only a short-lived subscription and is independently
# bounded.  Samples help ensure matching DATA packets occur during the PCAP.
for index in "${!topics[@]}"; do
  topic="${topics[index]}"
  label="${labels[index]}"
  set +e
  timeout --signal=INT --kill-after=2s 8s \
    ros2 topic echo "${topic}" --once --no-arr \
    >"${OUTPUT_DIR}/${label}.one-sample.txt" 2>&1
  status=$?
  set -e
  printf '%s topic-echo status=%d\n' "${topic}" "${status}" \
    >>"${OUTPUT_DIR}/commands.status.txt"
done

set +e
wait "${capture_pid}"
capture_status=$?
set -e
trap - INT TERM EXIT
printf 'tcpdump status=%d\n' "${capture_status}" \
  >>"${OUTPUT_DIR}/commands.status.txt"
if [[ ! -s "${pcap_path}" ]]; then
  printf '没有可分析 PCAP；检查 tcpdump.stderr.txt（脚本不会请求 sudo）。\n' >&2
  exit 1
fi
chmod 0600 "${pcap_path}"

extract_publisher_gids() {
  awk '
    /^[[:space:]]*Endpoint type:[[:space:]]*PUBLISHER[[:space:]]*$/ {
      publisher = 1
      next
    }
    publisher && /^[[:space:]]*GID:/ {
      sub(/^[[:space:]]*GID:[[:space:]]*/, "")
      print
      publisher = 0
    }
    /^[[:space:]]*Endpoint type:/ { publisher = 0 }
  ' "$1"
}

for index in "${!labels[@]}"; do
  label="${labels[index]}"
  info_path="${OUTPUT_DIR}/${label}.topic-info.txt"
  mapfile -t gids < <(extract_publisher_gids "${info_path}")
  if (( ${#gids[@]} != 1 )); then
    printf '%s: expected one publisher GID, observed %d; no IP conclusion\n' \
      "${label}" "${#gids[@]}" >>"${OUTPUT_DIR}/correlation.status.txt"
    continue
  fi
  set +e
  python3 "${SCRIPT_DIR}/correlate_rtps_writer_locator.py" \
    --pcap "${pcap_path}" \
    --writer-gid "${gids[0]}" \
    --output "${OUTPUT_DIR}/${label}.correlation.json"
  status=$?
  set -e
  printf '%s correlation status=%d\n' "${label}" "${status}" \
    >>"${OUTPUT_DIR}/correlation.status.txt"
done

printf '只读取证完成：%s\n' "${OUTPUT_DIR}"
printf '仅 correlation.json 的 writer_data_sources 可作为本次会话的 IP 证据。\n'
