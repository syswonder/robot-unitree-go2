#!/usr/bin/env bash
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"
rbnx codegen -p "$PKG" --mcp --ros2

# The skill consumes map/msg/MapLifecycle on the host. Mapping's x86 service
# builds its own copy inside Docker, which does not make the Python type
# importable here, so build and source this package-local canonical overlay.
[[ -f /opt/ros/humble/setup.bash ]] || {
  echo "missing /opt/ros/humble/setup.bash for MapLifecycle interface build" >&2
  exit 2
}
command -v colcon >/dev/null 2>&1 || {
  echo "colcon is required to build the MapLifecycle interface" >&2
  exit 2
}
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
IDL="$PKG/rbnx-build/codegen/ros2_idl"
[[ -d "$IDL/src/map" ]] || {
  echo "generated map/msg/MapLifecycle package is missing: $IDL/src/map" >&2
  exit 2
}
(cd "$IDL" && colcon build --packages-up-to map)
python3 -m unittest discover -s tests -p 'test_*.py'
