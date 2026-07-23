#!/usr/bin/env bash
set -euo pipefail
CONTAINER="abot-smolvla-shadow"
if docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
  docker stop "$CONTAINER"
else
  echo "$CONTAINER is not running"
fi
