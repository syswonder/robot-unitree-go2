#!/usr/bin/env bash
set -euo pipefail

readonly script_root="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly image="${JETSON_READONLY_IMAGE:-robonix-go2-jetson-readonly:local}"
readonly container="robonix-go2-jetson-readonly"
camera=0

if [[ "$#" -gt 1 ]]; then
  echo "usage: $0 [--camera]" >&2
  exit 2
fi
if [[ "$#" -eq 1 ]]; then
  [[ "$1" == "--camera" ]] || { echo "unknown option: $1" >&2; exit 2; }
  camera=1
fi
if [[ ! "$image" =~ ^[a-zA-Z0-9][a-zA-Z0-9._/:@-]*$ ]]; then
  echo "invalid image reference" >&2
  exit 42
fi

"${script_root}/validate-network.sh"
if [[ "$(docker image inspect --format '{{.Architecture}}' "$image")" != "arm64" ]]; then
  echo "refusing to run a non-ARM64 image" >&2
  exit 43
fi
if docker container inspect "$container" >/dev/null 2>&1; then
  echo "container already exists: ${container}" >&2
  exit 45
fi

echo "Starting READ-ONLY profile; motion, control daemons and persistence are absent."
docker run \
  --detach \
  --rm \
  --name "$container" \
  --hostname "$container" \
  --runtime runc \
  --network host \
  --read-only \
  --tmpfs /tmp:rw,noexec,nosuid,nodev,size=128m,mode=1777 \
  --cap-drop ALL \
  --security-opt no-new-privileges=true \
  --pids-limit 192 \
  --memory 1536m \
  --cpus 2.0 \
  --ulimit nofile=1024:1024 \
  --restart no \
  --log-driver none \
  --stop-timeout 10 \
  --user 10001:10001 \
  --env "GO2_ENABLE_CAMERA=${camera}" \
  "$image"
