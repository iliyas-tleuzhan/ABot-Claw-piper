#!/usr/bin/env bash
set -euo pipefail

PORT="${LAP_PORT:-8016}"
GPU="${LAP_GPU_DEVICE:-1}"
LAP_REPO_URL="${LAP_REPO_URL:-https://github.com/lihzha/lap}"
LAP_COMMIT="${LAP_COMMIT:-3958d1466d5b92445b67de7d4202c19608ad4d56}"
LAP_SRC="${LAP_SRC:-/data/home/iliyas/abot-data/lap-src}"
LAP_CHECKPOINT_DIR="${LAP_CHECKPOINT_DIR:-/data/home/iliyas/abot-data/lap-checkpoints/LAP-3B}"
LAP_VENV="${LAP_VENV:-/data/home/iliyas/abot-data/lap-venv}"
HF_CACHE_DIR="${HF_CACHE_DIR:-/data/home/iliyas/abot-data/huggingface}"
OPENPI_CACHE_DIR="${OPENPI_CACHE_DIR:-/data/home/iliyas/.cache/openpi}"
LAP_LOG_DIR="${LAP_LOG_DIR:-/data/home/iliyas/abot-data/lap-logs}"
TOKENIZER_PATH="${TOKENIZER_PATH:-${OPENPI_CACHE_DIR}/big_vision/paligemma_tokenizer.model}"
PID_FILE="${LAP_LOG_DIR}/lap3b_${PORT}.pid"
LOG_FILE="${LAP_LOG_DIR}/lap3b_${PORT}.log"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

mkdir -p "${HF_CACHE_DIR}" "${OPENPI_CACHE_DIR}" "${LAP_CHECKPOINT_DIR}" "${LAP_LOG_DIR}"

export PATH="${HOME}/.local/bin:${PATH}"

if ! command -v uv >/dev/null 2>&1; then
  curl -LsSf https://astral.sh/uv/install.sh | sh
  export PATH="${HOME}/.local/bin:${PATH}"
fi

if [[ ! -d "${LAP_SRC}/.git" ]]; then
  git clone --recurse-submodules "${LAP_REPO_URL}" "${LAP_SRC}"
fi

git -C "${LAP_SRC}" fetch --all --tags
git -C "${LAP_SRC}" checkout "${LAP_COMMIT}"
git -C "${LAP_SRC}" submodule update --init --recursive

uv python install 3.11
UV_PROJECT_ENVIRONMENT="${LAP_VENV}" uv sync --directory "${LAP_SRC}" --python 3.11 --group cuda
uv pip install --python "${LAP_VENV}/bin/python" "protobuf==6.33.6"

if [[ ! -f "${TOKENIZER_PATH}" ]]; then
  mkdir -p "$(dirname "${TOKENIZER_PATH}")"
  curl -L --fail --retry 3 \
    https://storage.googleapis.com/big_vision/paligemma_tokenizer.model \
    -o "${TOKENIZER_PATH}"
fi

mkdir -p "${LAP_SRC}/checkpoints"
ln -sfn "${LAP_CHECKPOINT_DIR}" "${LAP_SRC}/checkpoints/lap"

if [[ -f "${PID_FILE}" ]]; then
  old_pid="$(cat "${PID_FILE}")"
  if [[ -n "${old_pid}" ]] && kill -0 "${old_pid}" 2>/dev/null; then
    echo "LAP-3B already running on pid ${old_pid}"
    exit 0
  fi
fi

pkill -f "scripts/serve_policy.py --env=LAP --port=${PORT}" || true

(
  cd "${LAP_SRC}"
  nohup env \
    CUDA_VISIBLE_DEVICES="${GPU}" \
    JAX_PLATFORMS=cuda \
    OPENPI_DATA_HOME="${OPENPI_CACHE_DIR}" \
    HF_HOME="${HF_CACHE_DIR}" \
    "${LAP_VENV}/bin/python" scripts/serve_policy.py --env=LAP --port="${PORT}" \
    >"${LOG_FILE}" 2>&1 &
  echo $! >"${PID_FILE}"
)

sleep 5
"${LAP_VENV}/bin/python" "${SCRIPT_DIR}/check_lap3b_service.py" --host 127.0.0.1 --port "${PORT}"
echo "LAP-3B running on ws://127.0.0.1:${PORT}"
