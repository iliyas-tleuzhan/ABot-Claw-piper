#!/usr/bin/env bash
set -euo pipefail

PORT="${OPENVLA_PORT:-8018}"
curl -fsS "http://127.0.0.1:${PORT}/health"
