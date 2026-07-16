#!/usr/bin/env bash
set -euo pipefail

DEPLOY_DIR="$(cd "$(dirname "$0")/.." && pwd)"
echo "[validate] OFFLINE ONLY - no ROS graph, network, or motion command is used"

find "$DEPLOY_DIR" -type f -name '*.sh' \
  -not -path '*/rbnx-build/*' \
  -not -path '*/third_party/*' -print0 \
  | xargs -0 -n1 bash -n

DEPLOY_DIR="$DEPLOY_DIR" python3 - <<'PY'
from pathlib import Path
import os
import xml.etree.ElementTree as ET
import yaml

root = Path(os.environ["DEPLOY_DIR"])
for path in sorted(root.rglob("*.yaml")):
    if "rbnx-build" not in path.parts and "third_party" not in path.parts:
        yaml.safe_load(path.read_text(encoding="utf-8"))
for path in sorted(root.rglob("*.yml")):
    if "rbnx-build" not in path.parts and "third_party" not in path.parts:
        yaml.safe_load(path.read_text(encoding="utf-8"))
for path in (root / "config" / "navigate.xml", root / "packages" / "go2_description" / "urdf" / "go2_robonix.urdf"):
    ET.parse(path)

urdf = ET.parse(root / "packages" / "go2_description" / "urdf" / "go2_robonix.urdf").getroot()
links = {node.attrib["name"] for node in urdf.findall("link")}
required = {"base_link", "base", "imu", "radar", "utlidar_lidar", "front_camera"}
missing = required - links
if missing:
    raise SystemExit(f"URDF missing required links: {sorted(missing)}")
PY

python3 -m unittest discover \
  -s "$DEPLOY_DIR/packages/semantic_navigation/tests" -p 'test_*.py'
python3 -m unittest discover \
  -s "$DEPLOY_DIR/packages/go2_chassis/tests" -p 'test_*.py'
PYTHONPATH="$DEPLOY_DIR/packages/go2_description${PYTHONPATH:+:$PYTHONPATH}" \
  python3 -m unittest discover \
    -s "$DEPLOY_DIR/packages/go2_description/tests" -p 'test_*.py'

command -v g++ >/dev/null 2>&1 || {
  echo "[validate] missing offline test compiler: g++" >&2
  exit 1
}
TEMP_DIR="$(mktemp -d)"
trap 'rm -rf -- "$TEMP_DIR"' EXIT
g++ -std=c++17 -Wall -Wextra -Wpedantic -Werror \
  -I"$DEPLOY_DIR/packages/go2_chassis/include" \
  "$DEPLOY_DIR/packages/go2_chassis/tests/protocol_guard_test.cpp" \
  -o "$TEMP_DIR/protocol_guard_test"
"$TEMP_DIR/protocol_guard_test"

bash "$DEPLOY_DIR/packages/go2_sensors/tests/run_offline_tests.sh"
PYTHONPATH="$DEPLOY_DIR/packages/go2_dashboard${PYTHONPATH:+:$PYTHONPATH}" \
  python3 -m unittest discover \
    -s "$DEPLOY_DIR/packages/go2_dashboard/tests" -p 'test_*.py'
python3 -m unittest discover -s "$DEPLOY_DIR/tests" -p 'test_*.py'

echo "[validate] PASS"
