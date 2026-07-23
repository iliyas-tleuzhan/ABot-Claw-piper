#!/usr/bin/env bash
set -euo pipefail

NAME="${OPENVLA_CONTAINER_NAME:-abot-openvla-shadow}"

if docker ps -a --format '{{.Names}}' | grep -qx "$NAME"; then
  docker stop "$NAME" >/dev/null 2>&1 || true
  docker rm "$NAME"
else
  echo "$NAME does not exist."
fi
