#!/usr/bin/env bash
set -euo pipefail

SERVICE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAME="${OPENVLA_CONTAINER_NAME:-abot-openvla-shadow}"
IMAGE="${OPENVLA_IMAGE_NAME:-abot-openvla-shadow:local}"
PORT="${OPENVLA_PORT:-8018}"
MODEL_ID="${OPENVLA_MODEL_ID:-openvla/openvla-7b}"
MODEL_REVISION="${OPENVLA_MODEL_REVISION:-47a0ec7fc4ec123775a391911046cf33cf9ed83f}"
SOURCE_REVISION="${OPENVLA_SOURCE_REVISION:-c8f03f48af692657d3060c19588038c7220e9af9}"
HF_CACHE="${HF_HOME_HOST:-/data/home/iliyas/abot-data/huggingface}"
OPENVLA_CACHE_DIR="${OPENVLA_CACHE_DIR:-/data/home/iliyas/abot-data/openvla-huggingface}"
GPU="${OPENVLA_GPU:-1}"

if docker ps --format '{{.Names}}' | grep -qx "$NAME"; then
  echo "$NAME is already running."
  docker ps --filter "name=^/${NAME}$"
  exit 0
fi

if ss -ltn | grep -q ":${PORT} "; then
  echo "ERROR: port ${PORT} is already in use." >&2
  exit 1
fi

echo "Building $IMAGE from $SERVICE_DIR"
docker build -t "$IMAGE" "$SERVICE_DIR"

mkdir -p "$OPENVLA_CACHE_DIR"

if docker ps -a --format '{{.Names}}' | grep -qx "$NAME"; then
  echo "Removing stale stopped container $NAME before clean restart."
  docker rm "$NAME"
fi

cat <<EOF
Starting OpenVLA shadow service
  container: $NAME
  image:     $IMAGE
  port:      $PORT
  gpu:       physical GPU $GPU
  model:     $MODEL_ID@$MODEL_REVISION
  shared hf: $HF_CACHE
  ow cache:  $OPENVLA_CACHE_DIR
  source:    $SOURCE_REVISION
  shadow:    true
  execution: false
EOF

set -x
docker run -d \
  --name "$NAME" \
  --gpus "device=${GPU}" \
  --user "$(id -u):$(id -g)" \
  --restart on-failure:3 \
  -p "${PORT}:8018" \
  -e PORT=8018 \
  -e HF_HOME=/tmp/huggingface \
  -e HF_TOKEN_PATH=/tmp/huggingface/token \
  -e TRANSFORMERS_CACHE=/tmp/huggingface/hub \
  -e HF_HUB_CACHE=/tmp/huggingface/hub \
  -e OPENVLA_MODEL_ID="$MODEL_ID" \
  -e OPENVLA_MODEL_REVISION="$MODEL_REVISION" \
  -e OPENVLA_SOURCE_REVISION="$SOURCE_REVISION" \
  -e OPENVLA_DEVICE=cuda:0 \
  -v "$OPENVLA_CACHE_DIR:/tmp/huggingface" \
  "$IMAGE"
set +x

echo "Health check: curl http://127.0.0.1:${PORT}/health"
