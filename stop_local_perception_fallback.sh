#!/usr/bin/env bash
# Stop the local perception fallback tmux session.

set -Eeuo pipefail

SESSION="abot-local-perception"
CONTAINER="abot-piper-noetic"

if tmux has-session -t "${SESSION}" 2>/dev/null; then
    tmux kill-session -t "${SESSION}"
    echo "Stopped tmux session ${SESSION}."
else
    echo "No tmux session ${SESSION} is running."
fi

if [[ "$(docker inspect -f '{{.State.Status}}' "${CONTAINER}" 2>/dev/null || true)" == "running" ]]; then
    docker exec -i "${CONTAINER}" bash -lc '
for pid in $(pgrep -f "service_layer/YOLO/main.py|grasp_service_depth_fallback.py" || true); do
  kill "$pid" 2>/dev/null || true
done
' >/dev/null 2>&1 || true
fi

echo "Local perception fallback stop requested."
