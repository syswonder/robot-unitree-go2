#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Portable launcher; expects dependencies and configuration already provisioned.
set -euo pipefail
DEPLOY_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
export ROBONIX_DEPLOY_DIR="$DEPLOY_DIR"
export GO2_OPERATOR_PRESENT="${GO2_OPERATOR_PRESENT:-0}"
for setting in ROBONIX_AUDIO_BRIDGE_PATH ROBONIX_SPEECH_PATH FUNASR_MODEL_DIR GO2_SDK_SOCKET GO2_NETWORK_INTERFACE VLM_BASE_URL VLM_MODEL VLM_API_KEY; do
  if [[ -z "${!setting:-}" ]]; then
    echo "Set $setting before starting this deployment; see deploy/sport-actions/README.md." >&2
    exit 2
  fi
done
cd "$DEPLOY_DIR"
exec "${RBNX_BIN:-rbnx}" boot -f deploy/sport-actions/robonix_manifest.yaml
