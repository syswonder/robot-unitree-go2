#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Build the existing Go2 daemon without building or running Unitree examples.
set -euo pipefail

PACKAGE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ROBOT_ROOT="${GO2_ROBOT_DIR:-$(cd -- "$PACKAGE_ROOT/../.." && pwd)}"
SDK2_DIR="${UNITREE_SDK2_DIR:-$ROBOT_ROOT/third_party/unitree_sdk2}"
BUILD_ROOT="$ROBOT_ROOT/packages/go2_chassis/rbnx-build/sdk"

if [[ ! -f "$ROBOT_ROOT/packages/go2_chassis/sdk_daemon/CMakeLists.txt" ]]; then
  echo "Set GO2_ROBOT_DIR to the robot-unitree-go2 checkout containing sport-action support." >&2
  exit 2
fi
if [[ ! -f "$SDK2_DIR/CMakeLists.txt" ]]; then
  echo "Official unitree_sdk2 checkout not found; initialize robot submodules or set UNITREE_SDK2_DIR." >&2
  exit 2
fi
cmake -S "$ROBOT_ROOT/packages/go2_chassis/sdk_daemon" -B "$BUILD_ROOT" \
  -DUNITREE_SDK2_DIR="$SDK2_DIR" -DBUILD_EXAMPLES=OFF \
  -DCMAKE_INSTALL_PREFIX="$BUILD_ROOT/install"
cmake --build "$BUILD_ROOT" --parallel --target go2_sport_daemon
cmake --install "$BUILD_ROOT"
test -x "$BUILD_ROOT/install/bin/go2_sport_daemon"
echo "Go2 sport daemon ready: $BUILD_ROOT/install/bin/go2_sport_daemon"
