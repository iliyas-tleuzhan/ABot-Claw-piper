#!/usr/bin/env bash
set -euo pipefail

SERVICE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAME="${VLAC_POLICY_CONTAINER:-abot-vlac-policy}"
IMAGE="${VLAC_POLICY_IMAGE:-abot-vlac:spm021}"
PORT="${VLAC_POLICY_PORT:-8016}"
MODEL_DIR="${VLAC_MODEL_DIR:-/data/home/iliyas/abot-data/vlac-model}"
HF_CACHE="${HF_HOME_HOST:-/data/home/iliyas/abot-data/huggingface}"
GPU="${VLAC_POLICY_GPU:-0}"

if docker ps --format '{{.Names}}' | grep -qx "$NAME"; then
  echo "$NAME is already running; not starting a duplicate."
  docker ps --filter "name=^/${NAME}$"
  exit 0
fi

if docker ps -a --format '{{.Names}}' | grep -qx "$NAME"; then
  echo "Starting existing stopped container: $NAME"
  docker start "$NAME"
  exit 0
fi

if ss -ltn | grep -q ":${PORT} "; then
  echo "ERROR: port ${PORT} is already listening; not starting ${NAME}." >&2
  exit 1
fi

cat <<START_MSG
Starting VLAC policy preview shadow service.
Container: $NAME
Image:     $IMAGE
GPU:       physical GPU $GPU
Port:      $PORT
Model:     $MODEL_DIR (read-only)
Cache:     $HF_CACHE
Execution: disabled, shadow-only
START_MSG

set -x
docker run -d \
  --name "$NAME" \
  --gpus "device=${GPU}" \
  --network host \
  -e PORT="$PORT" \
  -e VLAC_POLICY_DEVICE="cuda:0" \
  -e VLAC_MODEL_PATH=/model \
  -e VLAC_MODEL_TYPE=internvl2 \
  -e HF_HOME=/root/.cache/huggingface \
  -e PYTHONPYCACHEPREFIX=/tmp/pycache \
  -v "$HF_CACHE:/root/.cache/huggingface" \
  -v "$MODEL_DIR:/model:ro" \
  -v "$SERVICE_DIR:/workspace/VLAC:ro" \
  --workdir /workspace/VLAC \
  "$IMAGE" \
  python policy_main.py
set +x

echo "Health check command: curl http://127.0.0.1:${PORT}/health"
