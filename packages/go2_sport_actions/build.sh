#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# Use an already provisioned interpreter; never install dependencies here.
RBNX="${RBNX_BIN:-rbnx}"
PYTHON="${RBNX_CODEGEN_PYTHON:-python3}"
if [[ "$RBNX" == */* ]]; then
  export PATH="$(dirname -- "$RBNX"):$PATH"
fi
if "$RBNX" codegen --help | rg -q -- '--python'; then
  "$RBNX" codegen -p "$ROOT" --mcp --python "$PYTHON"
else
  PATH="$(dirname -- "$PYTHON"):$PATH" "$RBNX" codegen -p "$ROOT" --mcp \
    --out-dir rbnx-build/codegen
fi
