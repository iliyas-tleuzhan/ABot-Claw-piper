#!/usr/bin/env bash
set -euo pipefail

ROOT="/data/home/iliyas/deploy/ABot-Claw-piper"
SERVICE_DIR="$ROOT/service_layer/SmolVLA"
IMAGE="abot-smolvla-shadow:local"
CONTAINER="abot-smolvla-shadow"
PORT="8018"
HF_CACHE="/data/home/iliyas/abot-data/huggingface"
MODEL_ID="lerobot/smolvla_base"
MODEL_REVISION="c83c3163b8ca9b7e67c509fffd9121e66cb96205"
LEROBOT_COMMIT="9c82c39c7b541e9c5bd8340abb7c9d8803c98744"

if docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
  echo "$CONTAINER already running"
  exit 0
fi
if docker ps -a --format '{{.Names}}' | grep -qx "$CONTAINER"; then
  docker start "$CONTAINER"
  exit 0
fi
if ss -ltn '( sport = :8018 )' | grep -q 8018; then
  echo "port 8018 already in use" >&2
  exit 1
fi
GPU=$(nvidia-smi --query-gpu=index,memory.used --format=csv,noheader,nounits | sort -t, -k2,2n | head -n1 | cut -d, -f1 | tr -d ' ')
echo "Selected physical GPU: $GPU"
echo "docker build -f $SERVICE_DIR/Dockerfile -t $IMAGE $ROOT"
docker build -f "$SERVICE_DIR/Dockerfile" -t "$IMAGE" "$ROOT"
echo "docker run -d --name $CONTAINER --gpus device=$GPU -p $PORT:$PORT ... $IMAGE"
docker run -d \
  --name "$CONTAINER" \
  --gpus "device=$GPU" \
  --user "$(id -u):$(id -g)" \
  --restart unless-stopped \
  -p "$PORT:$PORT" \
  -e SMOLVLA_SELECTED_PHYSICAL_GPU="$GPU" \
  -e SMOLVLA_DEVICE="cuda:0" \
  -e SMOLVLA_LOAD_MODEL="0" \
  -e SMOLVLA_MODEL_ID="$MODEL_ID" \
  -e SMOLVLA_MODEL_REVISION="$MODEL_REVISION" \
  -e SMOLVLA_LEROBOT_COMMIT="$LEROBOT_COMMIT" \
  -e SMOLVLA_PORT="$PORT" \
  -v "$HF_CACHE:/data/huggingface" \
  "$IMAGE"
