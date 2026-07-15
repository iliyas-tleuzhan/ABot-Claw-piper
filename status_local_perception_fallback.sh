#!/usr/bin/env bash
# Print local CPU perception fallback status.

set -Eeuo pipefail

SESSION="abot-local-perception"
YOLO_URL="http://127.0.0.1:8013"
GRASP_URL="http://127.0.0.1:8015"

health_json() {
    curl -fsS --max-time 3 "$1/health" 2>/dev/null || true
}

health_label() {
    python3 - "$1" <<'PY' 2>/dev/null || true
import json, sys
try:
    data = json.loads(sys.argv[1])
except Exception:
    print("unavailable")
    raise SystemExit(0)
status = data.get("status")
if status == "ok":
    print("ok")
elif status == "degraded":
    print("degraded")
else:
    print("not ready")
PY
}

yolo_body="$(health_json "${YOLO_URL}")"
grasp_body="$(health_json "${GRASP_URL}")"

if tmux has-session -t "${SESSION}" 2>/dev/null; then
    echo "tmux session: running (${SESSION})"
else
    echo "tmux session: not running (${SESSION})"
fi

echo "YOLO backend: local CPU"
echo "YOLO URL: ${YOLO_URL}"
echo "YOLO health: $(health_label "${yolo_body}")"
echo "YOLO health JSON: ${yolo_body:-unreachable}"
echo "Grasp backend: local depth_fallback"
echo "Grasp URL: ${GRASP_URL}"
echo "Grasp health: $(health_label "${grasp_body}")"
echo "Grasp health JSON: ${grasp_body:-unreachable}"
