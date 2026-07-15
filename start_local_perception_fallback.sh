#!/usr/bin/env bash
# Start local CPU YOLO and depth-fallback grasp HTTP services.

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="${SCRIPT_DIR}"
CONTAINER="abot-piper-noetic"
CONTAINER_REPO="/root/ABot-Claw"
SESSION="abot-local-perception"
LOG_DIR="${REPO_DIR}/.abot_full_stack_logs"
YOLO_URL="http://127.0.0.1:8013"
GRASP_URL="http://127.0.0.1:8015"
CAMERA_FRAME_ID="${CAMERA_FRAME_ID:-table_camera_color_optical_frame}"

health_json() {
    curl -fsS --max-time 3 "$1/health" 2>/dev/null || true
}

health_ok() {
    local body
    body="$(health_json "$1")"
    python3 - "$body" <<'PY' 2>/dev/null
import json, sys
try:
    data = json.loads(sys.argv[1])
except Exception:
    raise SystemExit(1)
raise SystemExit(0 if data.get("status") == "ok" else 1)
PY
}

wait_health() {
    local name="$1"
    local url="$2"
    local deadline=$((SECONDS + 90))
    while (( SECONDS < deadline )); do
        if [[ -n "$(health_json "${url}")" ]]; then
            printf '%s health: %s\n' "${name}" "$(health_json "${url}")"
            return 0
        fi
        sleep 2
    done
    printf 'ERROR: %s did not publish /health at %s\n' "${name}" "${url}" >&2
    return 1
}

tmux_window_exists() {
    tmux list-windows -t "${SESSION}" -F '#W' 2>/dev/null | grep -Fxq "$1"
}

start_window_if_needed() {
    local name="$1"
    local command="$2"
    if tmux_window_exists "${name}"; then
        return 0
    fi
    if ! tmux has-session -t "${SESSION}" 2>/dev/null; then
        tmux new-session -d -s "${SESSION}" -n "${name}" "bash -lc $(printf '%q' "${command}")"
    else
        tmux new-window -t "${SESSION}" -n "${name}" "bash -lc $(printf '%q' "${command}")"
    fi
}

pick_model() {
    find "${REPO_DIR}/service_layer/YOLO" -maxdepth 2 -type f \( -name '*.pt' -o -name '*.pth' -o -name '*.onnx' \) \
        -printf '%s %p\n' | sort -n | awk 'NR==1 {print $2}'
}

docker_running() {
    [[ "$(docker inspect -f '{{.State.Status}}' "${CONTAINER}" 2>/dev/null || true)" == "running" ]]
}

missing_deps_host() {
    python3 - <<'PY'
missing = []
for name in ["torch", "fastapi", "uvicorn", "PIL", "pandas", "cv2", "requests"]:
    try:
        __import__(name)
    except Exception:
        missing.append(name)
print(" ".join(missing))
PY
}

missing_deps_container() {
    docker exec -i "${CONTAINER}" bash -lc 'python3 - <<'"'"'PY'"'"'
missing = []
for name in ["torch", "fastapi", "uvicorn", "PIL", "pandas", "cv2", "requests"]:
    try:
        __import__(name)
    except Exception:
        missing.append(name)
print(" ".join(missing))
PY' 2>/dev/null || echo "python3"
}

mkdir -p "${LOG_DIR}"
MODEL_PATH="$(pick_model)"
if [[ -z "${MODEL_PATH}" ]]; then
    echo "ERROR: no cached YOLO model found under service_layer/YOLO" >&2
    exit 1
fi

RUN_IN_DOCKER=false
if docker_running; then
    missing="$(missing_deps_container)"
    if [[ -z "${missing}" ]]; then
        RUN_IN_DOCKER=true
    else
        host_missing="$(missing_deps_host)"
        if [[ -n "${host_missing}" ]]; then
            echo "ERROR: local CPU YOLO dependencies are missing." >&2
            echo "  ${CONTAINER}: ${missing}" >&2
            echo "  host python3: ${host_missing}" >&2
            exit 1
        fi
    fi
else
    host_missing="$(missing_deps_host)"
    if [[ -n "${host_missing}" ]]; then
        echo "ERROR: local CPU YOLO dependencies are missing in host python3: ${host_missing}" >&2
        exit 1
    fi
fi

if [[ -n "$(health_json "${YOLO_URL}")" ]]; then
    echo "YOLO already responds at ${YOLO_URL}; not starting another service on port 8013."
else
    if [[ "${RUN_IN_DOCKER}" == "true" ]]; then
        container_model="${MODEL_PATH/${REPO_DIR}/${CONTAINER_REPO}}"
        yolo_cmd="mkdir -p '${CONTAINER_REPO}/.abot_full_stack_logs'; cd '${CONTAINER_REPO}/service_layer/YOLO'; export PORT=8013 DEVICE=cpu YOLO_DEVICE=cpu YOLO_MODEL_PATH='${container_model}'; exec python3 main.py >> '${CONTAINER_REPO}/.abot_full_stack_logs/local-yolo-cpu.log' 2>&1"
        start_window_if_needed "yolo-cpu" "docker exec -i ${CONTAINER} bash -lc $(printf '%q' "${yolo_cmd}")"
    else
        yolo_cmd="cd '${REPO_DIR}/service_layer/YOLO'; export PORT=8013 DEVICE=cpu YOLO_DEVICE=cpu YOLO_MODEL_PATH='${MODEL_PATH}'; exec python3 main.py >> '${LOG_DIR}/local-yolo-cpu.log' 2>&1"
        start_window_if_needed "yolo-cpu" "${yolo_cmd}"
    fi
fi

wait_health "YOLO" "${YOLO_URL}"

if [[ -n "$(health_json "${GRASP_URL}")" ]]; then
    echo "Grasp fallback already responds at ${GRASP_URL}; not starting another service on port 8015."
else
    if [[ "${RUN_IN_DOCKER}" == "true" ]]; then
        grasp_cmd="mkdir -p '${CONTAINER_REPO}/.abot_full_stack_logs'; cd '${CONTAINER_REPO}/service_layer/GraspAnything'; export PORT=8015 YOLO_URL='${YOLO_URL}' CAMERA_FRAME_ID='${CAMERA_FRAME_ID}'; exec python3 grasp_service_depth_fallback.py >> '${CONTAINER_REPO}/.abot_full_stack_logs/local-grasp-depth-fallback.log' 2>&1"
        start_window_if_needed "grasp-fallback" "docker exec -i ${CONTAINER} bash -lc $(printf '%q' "${grasp_cmd}")"
    else
        grasp_cmd="cd '${REPO_DIR}/service_layer/GraspAnything'; export PORT=8015 YOLO_URL='${YOLO_URL}' CAMERA_FRAME_ID='${CAMERA_FRAME_ID}'; exec python3 grasp_service_depth_fallback.py >> '${LOG_DIR}/local-grasp-depth-fallback.log' 2>&1"
        start_window_if_needed "grasp-fallback" "${grasp_cmd}"
    fi
fi

wait_health "Grasp fallback" "${GRASP_URL}"

echo "YOLO backend: local CPU"
echo "YOLO URL: ${YOLO_URL}"
echo "YOLO model: ${MODEL_PATH}"
echo "Grasp backend: local depth_fallback"
echo "Grasp URL: ${GRASP_URL}"
echo "Logs: ${LOG_DIR}"
