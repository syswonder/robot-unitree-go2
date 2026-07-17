#!/usr/bin/env bash
set -euo pipefail

readonly root="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

for script in \
  build.sh \
  entrypoint.sh \
  export.sh \
  healthcheck.sh \
  run.sh \
  stop.sh \
  validate-network.sh \
  verify-runtime.sh; do
  bash -n "${root}/${script}"
done
python3 "${root}/tests/test_profile.py"
