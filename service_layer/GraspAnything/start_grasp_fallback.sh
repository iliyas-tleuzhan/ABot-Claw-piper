#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"

export PORT="${PORT:-8015}"
export YOLO_URL="${YOLO_URL:-http://127.0.0.1:8013}"
export MIN_DEPTH_M="${MIN_DEPTH_M:-0.05}"
export MAX_DEPTH_M="${MAX_DEPTH_M:-1.50}"
export GRIPPER_MIN_WIDTH_M="${GRIPPER_MIN_WIDTH_M:-0.005}"
export GRIPPER_MAX_WIDTH_M="${GRIPPER_MAX_WIDTH_M:-0.060}"
export FALLBACK_APPROACH_OFFSET_M="${FALLBACK_APPROACH_OFFSET_M:-0.10}"

exec python3 grasp_service_depth_fallback.py
