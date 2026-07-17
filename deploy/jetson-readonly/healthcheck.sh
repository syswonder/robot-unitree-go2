#!/usr/bin/env bash
set -euo pipefail

readonly runtime_root="/tmp/robonix"
readonly pid_root="${runtime_root}/pids"

/opt/robonix/profile/validate-network.sh >/dev/null

required=(robot-state-publisher chassis-passive sensor-relay)
if [[ -f "${runtime_root}/camera-enabled" ]]; then
  required+=(camera-daemon camera-bridge)
fi

for name in "${required[@]}"; do
  pid_file="${pid_root}/${name}.pid"
  [[ -r "$pid_file" ]] || exit 20
  IFS= read -r pid < "$pid_file"
  [[ "$pid" =~ ^[1-9][0-9]*$ ]] || exit 21
  kill -0 "$pid" 2>/dev/null || exit 22
done

grep -Eq '^[[:space:]]*allow_motion:[[:space:]]*false[[:space:]]*$' \
  /opt/robonix/profile/config/chassis-passive.yaml
