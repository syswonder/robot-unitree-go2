#!/usr/bin/env bash
set -euo pipefail

# This is a graph-only ownership gate. It never creates a ROS publisher, calls
# a service/action, or changes networking. Every ROS CLI query is time-bounded.

if [[ "$#" -lt 1 || "$#" -gt 2 ]]; then
  echo "usage: $0 <workstation-local|workstation-full-nx-sensors|workstation-ui-nx-full|nx-full|nx-sensors-only> [pre|post]" >&2
  exit 2
fi

readonly profile="$1"
readonly phase="${2:-pre}"
if [[ "$phase" != pre && "$phase" != post ]]; then
  echo "ownership phase must be exactly pre or post" >&2
  exit 2
fi
readonly discovery_timeout_s="${GO2_OWNERSHIP_DISCOVERY_TIMEOUT_S:-20}"
readonly stability_samples="${GO2_OWNERSHIP_STABILITY_SAMPLES:-3}"
readonly query_timeout_s="${GO2_OWNERSHIP_QUERY_TIMEOUT_S:-4}"

for value_name in discovery_timeout_s stability_samples query_timeout_s; do
  value="${!value_name}"
  if [[ ! "$value" =~ ^[1-9][0-9]*$ ]]; then
    echo "${value_name} must be a positive integer" >&2
    exit 2
  fi
done
if (( discovery_timeout_s > 60 || stability_samples > 10 || query_timeout_s > 15 )); then
  echo "ownership timing bounds exceed their audited maxima" >&2
  exit 2
fi

declare -a topics=(
  /camera/color/image_raw
  /camera/color/camera_info
  /scanner/cloud
  /scanner/imu
  /odom
  /tf_static
)
declare -a expected

case "${profile}:${phase}" in
  workstation-local:pre|nx-full:pre|nx-sensors-only:pre)
    expected=(0 0 0 0 0 0)
    ;;
  workstation-full-nx-sensors:pre)
    # The NX owns standardized camera/lidar only. The workstation will start
    # the sole chassis/description publishers after this gate passes.
    expected=(1 1 1 1 0 0)
    ;;
  workstation-ui-nx-full:pre)
    # The NX owns every hardware-state publisher; this host starts only UI.
    expected=(1 1 1 1 1 1)
    ;;
  workstation-local:post|workstation-full-nx-sensors:post|workstation-ui-nx-full:post|nx-full:post)
    expected=(1 1 1 1 1 1)
    ;;
  nx-sensors-only:post)
    expected=(1 1 1 1 0 0)
    ;;
  *)
    echo "unknown runtime ownership profile: $profile" >&2
    exit 2
    ;;
esac

for command in grep ros2 sed timeout; do
  command -v "$command" >/dev/null 2>&1 || {
    echo "required read-only ownership command is missing: $command" >&2
    exit 3
  }
done

publisher_count() {
  local topic="$1"
  local output status count
  set +e
  output="$(
    LC_ALL=C timeout --signal=INT --kill-after=1s "${query_timeout_s}s" \
      ros2 topic info -v "$topic" 2>&1
  )"
  status=$?
  set -e
  if [[ "$status" -ne 0 ]]; then
    if grep -Fq "Unknown topic" <<<"$output"; then
      printf '0\n'
      return 0
    fi
    echo "could not query publisher ownership for ${topic}: ${output}" >&2
    return 4
  fi
  count="$(
    sed -n 's/^[[:space:]]*Publisher count:[[:space:]]*\([0-9][0-9]*\)[[:space:]]*$/\1/p' \
      <<<"$output"
  )"
  if [[ ! "$count" =~ ^[0-9]+$ ]]; then
    echo "ros2 topic info returned no unambiguous publisher count for ${topic}" >&2
    return 4
  fi
  printf '%s\n' "$count"
}

echo "================================================================"
echo " READ-ONLY ROS PUBLISHER OWNERSHIP ${phase^^}CHECK: ${profile}"
echo " No topic is published and no service, action or motion API is called."
echo "================================================================"

deadline=$((SECONDS + discovery_timeout_s))
stable=0
declare -a last_counts=(0 0 0 0 0 0)
while (( SECONDS <= deadline )); do
  missing=false
  for index in "${!topics[@]}"; do
    count="$(publisher_count "${topics[$index]}")"
    last_counts[$index]="$count"
    if (( count > expected[index] )); then
      echo "publisher ownership violation: ${topics[$index]} expected ${expected[$index]}, found ${count}" >&2
      exit 5
    fi
    if (( count < expected[index] )); then
      missing=true
    fi
  done

  if [[ "$missing" == false ]]; then
    stable=$((stable + 1))
    if (( stable >= stability_samples )); then
      for index in "${!topics[@]}"; do
        printf '  %s publishers=%s owner-profile=%s\n' \
          "${topics[$index]}" "${last_counts[$index]}" "$profile"
      done
      echo "READ-ONLY publisher ownership ${phase}check passed"
      exit 0
    fi
  else
    stable=0
  fi
  sleep 1
done

for index in "${!topics[@]}"; do
  printf '  %s expected=%s observed=%s\n' \
    "${topics[$index]}" "${expected[$index]}" "${last_counts[$index]}" >&2
done
echo "publisher ownership discovery deadline expired for profile: ${profile}" >&2
exit 6
