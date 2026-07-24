#!/usr/bin/env bash
set -euo pipefail

NAME="${LAP_CONTAINER_NAME:-abot-lap3b}"

if docker ps -a --format '{{.Names}}' | grep -Fxq "${NAME}"; then
  docker rm -f "${NAME}"
else
  echo "Container ${NAME} not present."
fi
