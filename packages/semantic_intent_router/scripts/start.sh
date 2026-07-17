#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
HOST="${SEMANTIC_INTENT_HOST:-127.0.0.1}"
PORT="${SEMANTIC_INTENT_PORT:-18080}"
LANDMARKS="${SEMANTIC_LANDMARKS_FILE:-config/semantic_landmarks.yaml}"

if [[ "$LANDMARKS" != /* ]]; then
  LANDMARKS="$ROOT/$LANDMARKS"
fi

export PYTHONPATH="$ROOT/packages/semantic_intent_router:$ROOT/packages/semantic_navigation${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -m semantic_intent_router.server \
  --host "$HOST" \
  --port "$PORT" \
  --landmarks "$LANDMARKS"
