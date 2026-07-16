#!/usr/bin/env bash
# Static, read-only compatibility gate for safety-critical upstream packages.

set -euo pipefail

DEPLOY_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROBONIX_ROOT="$(rbnx path root)"
CACHE_ROOT="$DEPLOY_DIR/rbnx-boot/cache"
EVIDENCE_DIR="$DEPLOY_DIR/rbnx-build"
EVIDENCE_FILE="$EVIDENCE_DIR/upstream-lock.txt"

manifest_repo_dir() {
  local provider_name="$1"
  local url
  local trimmed

  url="$(awk -v wanted="$provider_name" '
    /^[[:space:]]{2}- name:[[:space:]]*/ {
      name = $0
      sub(/^[[:space:]]{2}- name:[[:space:]]*/, "", name)
      active = (name == wanted)
      next
    }
    active && /^[[:space:]]{4}url:[[:space:]]*/ {
      url = $0
      sub(/^[[:space:]]{4}url:[[:space:]]*/, "", url)
      gsub(/^["'"']|["'"']$/, "", url)
      print url
      exit
    }
  ' "$DEPLOY_DIR/robonix_manifest.yaml")"
  [[ -n "$url" ]] || {
    echo "manifest provider has no url: $provider_name" >&2
    exit 1
  }

  # Match rbnx deploy_repo_dir_name(): trailing slash and optional .git are
  # removed before the final URL/path component becomes the cache directory.
  trimmed="${url%/}"
  trimmed="${trimmed%.git}"
  trimmed="${trimmed##*/}"
  trimmed="${trimmed##*:}"
  [[ -n "$trimmed" ]] || {
    echo "cannot derive cache directory from manifest URL: $url" >&2
    exit 1
  }
  printf '%s\n' "$trimmed"
}

MAP_ROOT="$CACHE_ROOT/$(manifest_repo_dir mapping)"
NAV_ROOT="$CACHE_ROOT/$(manifest_repo_dir nav2)"

require_file() {
  local path="$1"
  [[ -f "$path" ]] || {
    echo "missing audited upstream file: $path" >&2
    exit 1
  }
}

require_text() {
  local path="$1"
  local expected="$2"
  local label="$3"
  require_file "$path"
  grep -Fq -- "$expected" "$path" || {
    echo "incompatible upstream: $label is absent from $path" >&2
    echo "Merge/update the linked upstream compatibility PR before deployment." >&2
    exit 1
  }
}

require_clean_git_checkout() {
  local label="$1"
  local path="$2"
  local dirty

  git -C "$path" rev-parse --is-inside-work-tree >/dev/null 2>&1 || {
    echo "audited upstream is not a git checkout: $label ($path)" >&2
    exit 1
  }
  dirty="$(git -C "$path" status --porcelain --untracked-files=normal)"
  [[ -z "$dirty" ]] || {
    echo "audited upstream checkout is dirty: $label ($path)" >&2
    printf '%s\n' "$dirty" >&2
    echo "Commit, discard, or separately audit those changes before deployment." >&2
    exit 1
  }
}

# Scene must discover this robot's canonical contracts, subscribe with the
# publisher-declared QoS, and carry the host's CycloneDDS NIC binding into its
# x86 container.
require_text "$ROBONIX_ROOT/pylib/robonix-api/robonix_api/capability.py" \
  'ROBONIX_PROVIDER_BIND_HOST' "Robonix provider bind-host control"
require_text "$ROBONIX_ROOT/pylib/robonix-api/robonix_api/capability.py" \
  'host=self._bind_host' "Robonix provider HTTP bind-host enforcement"
require_text "$ROBONIX_ROOT/system/scene/scene_service/service.py" \
  "robonix/primitive/lidar/lidar3d" "Scene canonical lidar3d contract"
require_text "$ROBONIX_ROOT/system/scene/scene_service/service.py" \
  "robonix/primitive/chassis/odom" "Scene chassis odometry contract"
require_text "$ROBONIX_ROOT/system/scene/scene_service/ingest/ros_subscribers.py" \
  "def topic_qos_policy" "Scene Atlas QoS policy"
require_text "$ROBONIX_ROOT/system/scene/docker/Dockerfile" \
  'ros-$ROS_DISTRO-rmw-cyclonedds-cpp' "Scene CycloneDDS RMW image dependency"
require_text "$ROBONIX_ROOT/system/scene/scripts/start.sh" \
  'CYCLONEDDS_URI="${CYCLONEDDS_URI:-}"' "Scene CycloneDDS URI forwarding"
require_text "$ROBONIX_ROOT/system/scene/scripts/start.sh" \
  'ROBONIX_PROVIDER_BIND_HOST="${ROBONIX_PROVIDER_BIND_HOST:-0.0.0.0}"' \
  "Scene provider bind-host forwarding"
require_text "$ROBONIX_ROOT/system/scene/scripts/start.sh" \
  'ROBONIX_ADVERTISE_HOST="${ROBONIX_ADVERTISE_HOST:-}"' \
  "Scene provider advertise-host forwarding"
require_text "$ROBONIX_ROOT/system/scene/scripts/start.sh" \
  'SCENE_HOST_DATA_DIR="${SCENE_DATA_DIR:-' \
  "Scene host persistence directory"
require_text "$ROBONIX_ROOT/system/scene/scripts/start.sh" \
  '-v "$SCENE_HOST_DATA_DIR":/data/robonix' \
  "Scene persistent data mount"

# Remote provider cache names are derived from the manifest repository URLs.
require_text "$MAP_ROOT/docker/Dockerfile" \
  "ros-humble-rmw-cyclonedds-cpp" "Mapping CycloneDDS RMW image dependency"
require_text "$MAP_ROOT/scripts/start.sh" \
  'CYCLONEDDS_URI="${CYCLONEDDS_URI:-}"' "Mapping CycloneDDS URI forwarding"
require_text "$MAP_ROOT/scripts/start.sh" \
  'ROBONIX_PROVIDER_BIND_HOST="${ROBONIX_PROVIDER_BIND_HOST:-0.0.0.0}"' \
  "Mapping provider bind-host forwarding"
require_text "$MAP_ROOT/scripts/start.sh" \
  'ROBONIX_ADVERTISE_HOST="${ROBONIX_ADVERTISE_HOST:-}"' \
  "Mapping provider advertise-host forwarding"
require_text "$MAP_ROOT/scripts/start.sh" \
  'MAPPING_ENABLE_VIZ="${MAPPING_ENABLE_VIZ:-false}"' \
  "Mapping visualization explicit opt-in"
require_text "$MAP_ROOT/scripts/start.sh" \
  'if [[ "$VIZ_ENABLED" == true ]]; then' \
  "Mapping X11 opt-in gate"
require_text "$MAP_ROOT/scripts/start.sh" \
  'XHOST_AUTHORIZED=true' "Mapping X11 authorization tracking"
require_text "$MAP_ROOT/scripts/start.sh" \
  'xhost -local:docker' "Mapping X11 authorization cleanup"

require_text "$NAV_ROOT/docker/Dockerfile" \
  "ros-humble-rmw-cyclonedds-cpp" "Navigation CycloneDDS RMW image dependency"
require_text "$NAV_ROOT/scripts/start.sh" \
  'CYCLONEDDS_URI="${CYCLONEDDS_URI:-}"' "Navigation CycloneDDS URI forwarding"
require_text "$NAV_ROOT/scripts/start.sh" \
  'ROBONIX_PROVIDER_BIND_HOST="${ROBONIX_PROVIDER_BIND_HOST:-0.0.0.0}"' \
  "Navigation provider bind-host forwarding"
require_text "$NAV_ROOT/scripts/start.sh" \
  'ROBONIX_ADVERTISE_HOST="${ROBONIX_ADVERTISE_HOST:-}"' \
  "Navigation provider advertise-host forwarding"
require_text "$NAV_ROOT/nav2_wrapper/atlas_bridge.py" \
  "cancel queued until goal acceptance" "Navigation pending-cancel latch"

require_clean_git_checkout "robonix" "$ROBONIX_ROOT"
require_clean_git_checkout "mapping" "$MAP_ROOT"
require_clean_git_checkout "navigation" "$NAV_ROOT"

mkdir -p "$EVIDENCE_DIR"
{
  printf 'Audited upstream compatibility evidence\n'
  printf 'Generated UTC: %s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  for entry in "robonix:$ROBONIX_ROOT" "mapping:$MAP_ROOT" "navigation:$NAV_ROOT"; do
    name="${entry%%:*}"
    path="${entry#*:}"
    revision="$(git -C "$path" rev-parse HEAD)"
    printf '%s %s\n' "$name" "$revision"
  done
} >"$EVIDENCE_FILE"

echo "[upstream-compat] PASS; revisions recorded in $EVIDENCE_FILE"
