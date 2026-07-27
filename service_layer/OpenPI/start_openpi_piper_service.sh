#!/usr/bin/env bash
set -euo pipefail

PORT="${OPENPI_PORT:-8017}"
OPENPI_ROOT="${OPENPI_ROOT:-${HOME}/ABot-Claw-piper/openpi}"
OPENPI_COMMIT="${OPENPI_COMMIT:-15a9616a00943ada6c20a0f158e3adb39df2ccac}"
OPENPI_CONFIG="${OPENPI_CONFIG:-pi05_piper_single_arm}"
OPENPI_CHECKPOINT="${OPENPI_CHECKPOINT:-}"
LOG_DIR="${OPENPI_LOG_DIR:-${HOME}/abot-data/openpi-logs}"
PID_FILE="${LOG_DIR}/openpi_piper_${PORT}.pid"
LOG_FILE="${LOG_DIR}/openpi_piper_${PORT}.log"

if [[ -z "${OPENPI_CHECKPOINT}" ]]; then
  echo "ERROR: OPENPI_CHECKPOINT must point to a PiPER-fine-tuned checkpoint." >&2
  echo "Public pi05_base is shadow/protocol-only and is not accepted as physical PiPER-compatible." >&2
  exit 2
fi

mkdir -p "${LOG_DIR}"
if [[ ! -d "${OPENPI_ROOT}/.git" ]]; then
  git clone https://github.com/Physical-Intelligence/openpi.git "${OPENPI_ROOT}"
fi
git -C "${OPENPI_ROOT}" fetch origin main
git -C "${OPENPI_ROOT}" checkout "${OPENPI_COMMIT}"

cd "${OPENPI_ROOT}"
GIT_LFS_SKIP_SMUDGE=1 uv sync

if [[ -f "${PID_FILE}" ]] && kill -0 "$(cat "${PID_FILE}")" 2>/dev/null; then
  echo "OpenPI PiPER service already running on pid $(cat "${PID_FILE}")"
  exit 0
fi

(
  exec uv run scripts/serve_policy.py policy:checkpoint \
    --policy.config="${OPENPI_CONFIG}" \
    --policy.dir="${OPENPI_CHECKPOINT}" \
    --port="${PORT}"
) >"${LOG_FILE}" 2>&1 &
echo "$!" >"${PID_FILE}"
echo "OpenPI PiPER service starting on port ${PORT}; log: ${LOG_FILE}"

