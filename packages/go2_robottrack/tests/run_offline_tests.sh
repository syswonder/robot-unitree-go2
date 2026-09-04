#!/usr/bin/env bash
set -euo pipefail

ROOT="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

echo "[go2_robottrack/test] OFFLINE ONLY - no ROS graph, network, camera, or robot access"
PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH="${ROOT}${PYTHONPATH:+:${PYTHONPATH}}" \
  python3 -m unittest discover -s "${ROOT}/tests" -p 'test_*.py' -v
