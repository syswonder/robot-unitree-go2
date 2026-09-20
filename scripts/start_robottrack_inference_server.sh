#!/usr/bin/env bash
# Start only the pinned official MiniCPM-RobotTrack HTTP inference server.

set -euo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
readonly WORKSPACE_ROOT="$(cd -- "${REPO_ROOT}/../.." && pwd)"
readonly DEFAULT_UPSTREAM_ROOT="${WORKSPACE_ROOT}/upstream/MiniCPM-Robot"
readonly UPSTREAM_CONFIG="${ROBOTTRACK_UPSTREAM_ROOT:-${DEFAULT_UPSTREAM_ROOT}}"
if [[ "${UPSTREAM_CONFIG}" = /* ]]; then
  readonly UPSTREAM_ROOT="${UPSTREAM_CONFIG}"
else
  readonly UPSTREAM_ROOT="${WORKSPACE_ROOT}/${UPSTREAM_CONFIG}"
fi
readonly TRACK_ROOT="${UPSTREAM_ROOT}/MiniCPM-RobotTrack"
readonly SAMPLE_ROOT="${TRACK_ROOT}/realworld/sample"
readonly SERVER_SCRIPT="${SAMPLE_ROOT}/http_minicpm_robot_track_server.py"
readonly MODEL_DIR="${ROBOTTRACK_MODEL_DIR:-${TRACK_ROOT}/minicpm_robot_track/checkpoints/MiniCPM-RobotTrack}"
readonly DINO_DIR="${TRACK_ROOT}/minicpm_robot_track/backbones/dino_local_hf"
readonly SIGLIP_DIR="${TRACK_ROOT}/minicpm_robot_track/backbones/siglip-so400m-patch14-384"
readonly VERIFY_SCRIPT="${REPO_ROOT}/scripts/verify_robottrack_assets.py"
readonly DEFAULT_DEPENDENCY_ROOT="${WORKSPACE_ROOT}/.tools/robottrack-python"
readonly DEPENDENCY_ROOT="${ROBOTTRACK_PYTHON_DEPS:-${DEFAULT_DEPENDENCY_ROOT}}"
readonly DEFAULT_SPEECH_PYTHON="${WORKSPACE_ROOT}/upstream/robonix/services/speech/rbnx-build/venv/bin/python"
readonly FALLBACK_SPEECH_PYTHON="${WORKSPACE_ROOT}/upstream/robonix-go2-build/services/speech/rbnx-build/venv/bin/python"

if [[ -n "${PYTHON:-}" ]]; then
  readonly PYTHON_BIN="${PYTHON}"
elif [[ -x "${DEFAULT_SPEECH_PYTHON}" ]]; then
  readonly PYTHON_BIN="${DEFAULT_SPEECH_PYTHON}"
else
  readonly PYTHON_BIN="${FALLBACK_SPEECH_PYTHON}"
fi

for required in \
  "${PYTHON_BIN}" \
  "${DEPENDENCY_ROOT}" \
  "${VERIFY_SCRIPT}" \
  "${SERVER_SCRIPT}" \
  "${MODEL_DIR}"; do
  if [[ ! -e "${required}" ]]; then
    echo "RobotTrack inference server not started: required local path is missing: ${required}" >&2
    exit 2
  fi
done
if [[ ! -x "${PYTHON_BIN}" ]]; then
  echo "RobotTrack inference server not started: PYTHON is not executable: ${PYTHON_BIN}" >&2
  exit 2
fi

export PYTHONPATH="${DEPENDENCY_ROOT}:${TRACK_ROOT}:${SAMPLE_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"

verify_status=0
"${PYTHON_BIN}" "${VERIFY_SCRIPT}" --upstream-root "${UPSTREAM_ROOT}" || verify_status=$?
if (( verify_status != 0 )); then
  missing_dino=()
  for required in \
    "${DINO_DIR}/model.safetensors" \
    "${DINO_DIR}/config.json" \
    "${DINO_DIR}/preprocessor_config.json" \
    "${DINO_DIR}/LICENSE.md"; do
    if [[ ! -s "${required}" ]]; then
      missing_dino+=("${required}")
    fi
  done
  if (( ${#missing_dino[@]} > 0 )); then
    echo "RobotTrack inference server not started: the pinned local DINOv3 asset is incomplete." >&2
    printf '  missing: %s\n' "${missing_dino[@]}" >&2
    echo "No download was attempted; install the licensed DINOv3 snapshot locally, then rerun." >&2
  else
    echo "RobotTrack inference server not started: pinned local asset verification failed." >&2
  fi
  exit "${verify_status}"
fi

export MINICPM_ROBOT_TRACK_ROOT="${TRACK_ROOT}"
export DINOV3_MODEL_PATH="${DINO_DIR}"
export SIGLIP_MODEL_PATH="${SIGLIP_DIR}"
export ROBOTTRACK_CAMERA_SOURCE="${ROBOTTRACK_CAMERA_SOURCE:-d435i}"
export TRANSFORMERS_OFFLINE=1
export HF_HUB_OFFLINE=1
export HF_HUB_DISABLE_TELEMETRY=1
export TOKENIZERS_PARALLELISM=false
export NO_PROXY="127.0.0.1,localhost${NO_PROXY:+,${NO_PROXY}}"
export no_proxy="127.0.0.1,localhost${no_proxy:+,${no_proxy}}"
export HF_HOME="${HF_HOME:-${WORKSPACE_ROOT}/.tools/robottrack-hf-cache}"
export HF_MODULES_CACHE="${HF_MODULES_CACHE:-${HF_HOME}/modules}"

readonly HOST="${ROBOTTRACK_SERVER_HOST:-127.0.0.1}"
readonly PORT="${ROBOTTRACK_SERVER_PORT:-5801}"
readonly DEVICE="${ROBOTTRACK_DEVICE:-cuda:0}"
readonly INSTRUCTION="${ROBOTTRACK_INSTRUCTION:-Follow the person ahead}"
readonly VELOCITY_CONTROLLER="${ROBOTTRACK_VELOCITY_CONTROLLER:-lookahead}"

echo "Starting pinned MiniCPM-RobotTrack inference only: host=${HOST}:${PORT} camera_source=${ROBOTTRACK_CAMERA_SOURCE} backends=torch/torch controller=${VELOCITY_CONTROLLER}" >&2
exec "${PYTHON_BIN}" -u "${SERVER_SCRIPT}" \
  --host "${HOST}" \
  --port "${PORT}" \
  --device "${DEVICE}" \
  --model_dir "${MODEL_DIR}" \
  --instruction "${INSTRUCTION}" \
  --vision_amp "${ROBOTTRACK_VISION_AMP:-bf16}" \
  --planner_amp "${ROBOTTRACK_PLANNER_AMP:-none}" \
  --dino_backend torch \
  --siglip_backend torch \
  --response-mode control \
  --timing-mode fast \
  --overlay-mode async \
  --velocity-controller "${VELOCITY_CONTROLLER}" \
  --controller-lookahead-idx "${ROBOTTRACK_CONTROLLER_LOOKAHEAD_IDX:-2}" \
  --controller-feedforward-points "${ROBOTTRACK_CONTROLLER_FEEDFORWARD_POINTS:-3}" \
  --controller-ff-blend "${ROBOTTRACK_CONTROLLER_FF_BLEND:-0.65}" \
  "$@"
