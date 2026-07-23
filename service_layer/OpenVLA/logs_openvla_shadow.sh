#!/usr/bin/env bash
set -euo pipefail

NAME="${OPENVLA_CONTAINER_NAME:-abot-openvla-shadow}"
docker logs -f "$NAME"
