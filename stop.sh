#!/usr/bin/env bash
set -euo pipefail

DEPLOY_DIR="$(cd "$(dirname "$0")" && pwd)"
MANIFEST="${ROBONIX_MANIFEST:-$DEPLOY_DIR/robonix_manifest.yaml}"
export ROBONIX_DEPLOY_DIR="$DEPLOY_DIR"
export PATH="$HOME/.local/bin:$HOME/.cargo/bin:$PATH"

if command -v rbnx >/dev/null 2>&1 && [[ -f "$DEPLOY_DIR/rbnx-boot/state.json" ]]; then
  rbnx shutdown -f "$MANIFEST" "$@"
else
  echo "no Robonix boot state; nothing to stop"
fi
