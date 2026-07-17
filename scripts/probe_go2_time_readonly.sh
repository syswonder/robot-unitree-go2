#!/usr/bin/env bash
# Bounded wrapper for the subscription-only Go2 source-clock probe.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
OUTPUT_DIR="${1:-${PROJECT_ROOT}/logs/go2-readonly/${STAMP}-time-probe}"
DURATION_SECONDS="${2:-${GO2_TIME_PROBE_DURATION_SECONDS:-60}}"
MAX_SAMPLES="${GO2_TIME_PROBE_MAX_SAMPLES:-200000}"

cat <<'BANNER'
=======================================================================
  READ-ONLY / 只读取证：Go2 / NX / MID-360 源时间戳
  只创建四个 ROS 订阅；不创建 publisher，不调用 Unitree API，
  不执行 date/timedatectl/chrony，不请求任何调时权限。
=======================================================================
BANNER

for value in "${DURATION_SECONDS}" "${MAX_SAMPLES}"; do
  if [[ ! "${value}" =~ ^[1-9][0-9]*$ ]]; then
    printf 'duration/max-samples 必须是正整数，收到 %q。\n' "${value}" >&2
    exit 2
  fi
done
if (( DURATION_SECONDS > 86400 || MAX_SAMPLES > 10000000 )); then
  printf '拒绝无界取证：duration<=86400，max-samples<=10000000。\n' >&2
  exit 2
fi

for command in python3 realpath timeout; do
  command -v "${command}" >/dev/null 2>&1 || {
    printf '缺少依赖：%s。未执行取证。\n' "${command}" >&2
    exit 127
  }
done

OUTPUT_DIR="$(realpath -m -- "${OUTPUT_DIR}")"
case "${OUTPUT_DIR}" in
  "${PROJECT_ROOT}/logs/"*) ;;
  *)
    printf '输出目录必须位于 %s/logs/ 内。\n' "${PROJECT_ROOT}" >&2
    exit 2
    ;;
esac

outer_timeout=$((DURATION_SECONDS + 15))
set +e
timeout --signal=INT --kill-after=5s "${outer_timeout}s" \
  python3 "${SCRIPT_DIR}/probe_go2_time_readonly.py" \
    --output-dir "${OUTPUT_DIR}" \
    --duration-seconds "${DURATION_SECONDS}" \
    --max-samples "${MAX_SAMPLES}" \
    --primary-topic "${GO2_TIME_PRIMARY_TOPIC:-/sportmodestate}" \
    --fallback-topic "${GO2_TIME_FALLBACK_TOPIC:-/lf/sportmodestate}" \
    --cloud-topic "${GO2_TIME_CLOUD_TOPIC:-/utlidar/cloud}" \
    --imu-topic "${GO2_TIME_IMU_TOPIC:-/utlidar/imu}"
status=$?
set -e

if (( status == 124 || status == 137 )); then
  printf '外层 timeout 触发（状态 %d）；证据可能不完整：%s\n' \
    "${status}" "${OUTPUT_DIR}" >&2
  exit 1
fi
exit "${status}"
