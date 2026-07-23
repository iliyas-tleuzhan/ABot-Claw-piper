#!/usr/bin/env bash
set -euo pipefail
NAME="${VLAC_POLICY_CONTAINER:-abot-vlac-policy}"
if docker ps --format '{{.Names}}' | grep -qx "$NAME"; then
  docker stop "$NAME"
  echo "Stopped $NAME. Container, image, model and cache were preserved."
else
  echo "$NAME is not running. Nothing stopped."
fi
