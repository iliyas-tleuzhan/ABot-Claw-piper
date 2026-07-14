#!/usr/bin/env bash
# Host-side ABot-Claw full-stack launcher.
# Starts infrastructure and read-only validation only. It never commands robot
# motion, leases, resets, gripper commands, calibration samples, or calibration
# computation.

set -Eeuo pipefail

MARKER_ID="6"
# MARKER_SIZE_M must equal the precisely measured outside width of the black
# square, in metres.
MARKER_SIZE_M="0.100"
RECTIFIED_IMAGE_TOPIC="/table_camera/color/image_rect_color"
RECTIFIED_CAMERA_INFO_TOPIC="/table_camera/color/camera_info"
IMAGE_PROC_NODE="/table_camera/color/image_proc"
EASY_HANDEYE_SYSTEM_CV_PATH="/usr/lib/python3/dist-packages"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="${SCRIPT_DIR}"
CONTAINER="abot-piper-noetic"
CONTAINER_REPO="/root/ABot-Claw"
AGENT_DIR="${CONTAINER_REPO}/robot_layer/arm_piper/agent_server"
ROS_WS="${AGENT_DIR}/robot_driver_ros"
EASY_HANDEYE_WS="/root/easy_handeye_ws"
SESSION="abot-full-stack"
LOWER_SESSION="abotclaw"
REMOTE_USER="iliyas"
REMOTE_HOST="192.168.1.104"
REMOTE_SSH_ALIAS="master"
REMOTE_REPO="/workspace/ABot-Claw-piper"
REMOTE_YOLO_CONTAINER="yolo-5090-torch"
REMOTE_YOLO_URL="http://${REMOTE_HOST}:8013"
REMOTE_GRASP_URL="http://${REMOTE_HOST}:8015"
AGENT_URL="http://localhost:8888"
LOG_DIR="${REPO_DIR}/.abot_full_stack_logs"
MODE="start"
X11_AVAILABLE=false
DOCKER_X11_ARGS=""

declare -A STATUS=()

stage="initializing"
trap 'echo "ERROR: failed during stage: ${stage}" >&2' ERR

usage() {
    cat <<'EOF'
Usage: ./start_abotclaw_full_stack.sh [--status]

  (no args)   Start/reuse the full ABot-Claw Piper stack.
  --status    Inspect and print status only. Does not start services or edit routes.
EOF
}

log() {
    printf '[%s] %s\n' "$(date '+%F %T')" "$*"
}

set_status() {
    STATUS["$1"]="$2"
}

require_cmd() {
    if ! command -v "$1" >/dev/null 2>&1; then
        echo "ERROR: required command not found: $1" >&2
        exit 1
    fi
}

run_quiet() {
    "$@" >/dev/null 2>&1
}

docker_running() {
    [[ "$(docker inspect -f '{{.State.Status}}' "${CONTAINER}" 2>/dev/null || true)" == "running" ]]
}

container_exec() {
    docker exec -i "${CONTAINER}" bash -lc "$1"
}

container_ros() {
    container_exec "source /opt/ros/noetic/setup.bash; source '${ROS_WS}/devel/setup.bash'; export ROS_MASTER_URI=http://localhost:11311; export ROS_HOSTNAME=localhost; $1"
}

tmux_window_exists() {
    tmux list-windows -t "${SESSION}" -F '#W' 2>/dev/null | grep -Fxq "$1"
}

tmux_start_window() {
    local name="$1"
    local command="$2"
    if tmux_window_exists "${name}"; then
        tmux kill-window -t "${SESSION}:${name}" 2>/dev/null || true
    fi
    tmux new-window -t "${SESSION}" -n "${name}" "bash -lc $(printf '%q' "${command}")"
}

tmux_start_window_if_missing() {
    local name="$1"
    local command="$2"
    if tmux_window_exists "${name}"; then
        log "Reusing tmux window ${SESSION}:${name}."
        return 0
    fi
    tmux new-window -t "${SESSION}" -n "${name}" "bash -lc $(printf '%q' "${command}")"
}

tmux_replace_window() {
    local name="$1"
    local command="$2"
    if tmux_window_exists "${name}"; then
        log "Replacing stale tmux window ${SESSION}:${name}."
        tmux kill-window -t "${SESSION}:${name}" 2>/dev/null || true
    fi
    tmux new-window -t "${SESSION}" -n "${name}" "bash -lc $(printf '%q' "${command}")"
}

ensure_full_tmux() {
    stage="tmux session setup"
    mkdir -p "${LOG_DIR}"
    if ! tmux has-session -t "${SESSION}" 2>/dev/null; then
        tmux new-session -d -s "${SESSION}" -n "status" "bash -lc 'cd $(printf '%q' "${REPO_DIR}"); echo ABot-Claw full stack status window; echo Logs: $(printf '%q' "${LOG_DIR}"); exec bash'"
    fi
}

prepare_x11() {
    stage="X11 preparation"
    X11_AVAILABLE=false
    DOCKER_X11_ARGS=""
    if [[ -z "${DISPLAY:-}" ]]; then
        log "DISPLAY is unset; GUI components will run only where supported without GUI."
        return 1
    fi

    if command -v xhost >/dev/null 2>&1; then
        log "Allowing narrow local root X access for Docker GUI apps: xhost +SI:localuser:root"
        xhost +SI:localuser:root >/dev/null 2>&1 || true
    fi

    DOCKER_X11_ARGS="-e DISPLAY=$(printf '%q' "${DISPLAY}") -e QT_X11_NO_MITSHM=1"
    local xauth_candidate=""
    if [[ -n "${XAUTHORITY:-}" && -r "${XAUTHORITY}" ]]; then
        xauth_candidate="${XAUTHORITY}"
    elif [[ -r "${HOME}/.Xauthority" ]]; then
        xauth_candidate="${HOME}/.Xauthority"
    fi
    if [[ -n "${xauth_candidate}" ]] && docker exec "${CONTAINER}" test -r "${xauth_candidate}" 2>/dev/null; then
        DOCKER_X11_ARGS="${DOCKER_X11_ARGS} -e XAUTHORITY=$(printf '%q' "${xauth_candidate}")"
    fi

    X11_AVAILABLE=true
    return 0
}

http_ok() {
    curl -fsS --max-time "${2:-3}" "$1" >/dev/null 2>&1
}

json_field() {
    python3 - "$1" "$2" <<'PY' 2>/dev/null || true
import json, sys
try:
    data=json.loads(sys.argv[1])
    cur=data
    for part in sys.argv[2].split("."):
        if isinstance(cur, dict):
            cur=cur.get(part)
        else:
            cur=None
            break
    if cur is not None:
        print(cur)
except Exception:
    pass
PY
}

wait_until() {
    local label="$1"
    local attempts="$2"
    local delay="$3"
    shift 3
    local i
    for ((i=1; i<=attempts; i++)); do
        if "$@"; then
            return 0
        fi
        sleep "${delay}"
    done
    log "WARNING: timeout waiting for ${label}"
    return 1
}

preflight() {
    stage="preflight"
    require_cmd docker
    require_cmd tmux
    require_cmd curl
    require_cmd ssh
    require_cmd ip
    if [[ ! -f "${REPO_DIR}/start_abotclaw_all.sh" || ! -d "${REPO_DIR}/robot_layer" ]]; then
        echo "ERROR: script must run from the ABot-Claw repository layout." >&2
        exit 1
    fi
    if ! docker container inspect "${CONTAINER}" >/dev/null 2>&1; then
        echo "ERROR: Docker container ${CONTAINER} does not exist. This launcher will not recreate it." >&2
        exit 1
    fi
    if [[ "${MODE}" == "status" && ! docker_running ]]; then
        return 0
    fi
    if ! docker_running; then
        log "Starting existing Docker container ${CONTAINER}."
        docker start "${CONTAINER}" >/dev/null
    fi
    container_exec "test -d '${AGENT_DIR}' && test -f '${AGENT_DIR}/server.py' && test -f '${ROS_WS}/devel/setup.bash'"
    container_exec "test -d '${EASY_HANDEYE_WS}'"
    if ! container_exec "source /opt/ros/noetic/setup.bash; rospack find aruco_ros >/dev/null"; then
        echo "ERROR: aruco_ros is not available in ${CONTAINER}." >&2
        exit 1
    fi
    if ! container_exec "dpkg -s ros-noetic-image-proc >/dev/null 2>&1"; then
        echo "ERROR: ros-noetic-image-proc is not installed in ${CONTAINER}." >&2
        echo "Install apt package: ros-noetic-image-proc" >&2
        exit 1
    fi
    if ! container_exec "source /opt/ros/noetic/setup.bash; rospack find image_proc >/dev/null"; then
        echo "ERROR: image_proc is not available in ${CONTAINER}." >&2
        echo "Install apt package: ros-noetic-image-proc" >&2
        exit 1
    fi
    if ! container_exec "source /opt/ros/noetic/setup.bash; source '${EASY_HANDEYE_WS}/devel/setup.bash'; rospack find easy_handeye >/dev/null"; then
        echo "ERROR: easy_handeye is not available in ${CONTAINER}." >&2
        exit 1
    fi
}

route_status() {
    local route
    route="$(ip route get "${REMOTE_HOST}" 2>/dev/null || true)"
    if [[ -z "${route}" ]]; then
        echo "unreachable"
    elif grep -q 'dev proton0' <<<"${route}"; then
        echo "bad: ${route}"
    else
        echo "ok: ${route}"
    fi
}

ensure_lan_route() {
    stage="LAN route check"
    local route rule
    route="$(ip route get "${REMOTE_HOST}" 2>/dev/null || true)"
    log "Route to ${REMOTE_HOST}: ${route:-unreachable}"
    if ! grep -q 'dev proton0' <<<"${route}"; then
        set_status "route to ${REMOTE_HOST}" "ok"
        return 0
    fi
    if ip rule show | grep -Eq "^[[:space:]]*100:.*to ${REMOTE_HOST}(/32)? .*lookup main"; then
        log "Priority-100 main-table route rule already exists."
    else
        log "Traffic to ${REMOTE_HOST} is routed through proton0."
        log "sudo may prompt because adding an ip rule requires root privileges; Proton VPN will not be disabled."
        sudo ip rule add priority 100 to "${REMOTE_HOST}/32" lookup main
    fi
    route="$(ip route get "${REMOTE_HOST}" 2>/dev/null || true)"
    log "Final route to ${REMOTE_HOST}: ${route:-unreachable}"
    if grep -q 'dev proton0' <<<"${route}"; then
        set_status "route to ${REMOTE_HOST}" "bad: still proton0"
        return 1
    fi
    set_status "route to ${REMOTE_HOST}" "ok"
}

ros_ready() {
    container_ros "rostopic list >/dev/null 2>&1"
}

topic_ready() {
    local topic="$1"
    container_ros "rostopic info '${topic}' >/dev/null 2>&1"
}

param_ready() {
    local param="$1"
    container_ros "rosparam get '${param}' >/dev/null 2>&1"
}

node_ready() {
    local node="$1"
    container_ros "rosnode list 2>/dev/null | grep -Fxq '${node}'"
}

rosnode_matches() {
    local pattern="$1"
    container_ros "rosnode list 2>/dev/null | grep -Eq '${pattern}'"
}

rosnode_matches_live() {
    local pattern="$1"
    container_ros "for node in \$(rosnode list 2>/dev/null | grep -E '${pattern}' || true); do timeout 2 rosnode ping -c 1 \"\$node\" 2>&1 | grep -q 'xmlrpc reply' && exit 0; done; exit 1"
}

tf_ready() {
    local parent="$1"
    local child="$2"
    container_ros "timeout 4 rosrun tf tf_echo '${parent}' '${child}' 2>&1 | grep -q 'Translation:'"
}

image_proc_ready() {
    rosnode_matches_live '^/table_camera/color/image_proc$' \
        && topic_ready "${RECTIFIED_IMAGE_TOPIC}" \
        && container_ros "timeout 6 rostopic echo -n 1 '${RECTIFIED_IMAGE_TOPIC}/header' >/dev/null 2>&1"
}

aruco_uses_rectified_image() {
    node_ready /aruco_simple \
        && container_ros "rosnode info /aruco_simple 2>/dev/null | grep -Fq '${RECTIFIED_IMAGE_TOPIC}'"
}

cleanup_aruco_keepalive() {
    container_exec "python3 - <<'PY'
import os
import signal

for pid in filter(str.isdigit, os.listdir('/proc')):
    try:
        raw = open(f'/proc/{pid}/cmdline', 'rb').read().split(b'\0')
    except OSError:
        continue
    args = [part.decode('utf-8', 'ignore') for part in raw if part]
    if len(args) >= 4 and args[0].endswith('python3') and args[1].endswith('/rostopic') and args[2:5] == ['hz', '/aruco_simple/pose']:
        os.kill(int(pid), signal.SIGTERM)
PY"
}

lower_stack_healthy() {
    local mgr
    mgr="$(container_ros "rosparam get /move_group/moveit_controller_manager 2>/dev/null || true" | tr -d '\r')"
    ros_ready \
        && topic_ready /joint_states \
        && topic_ready /joint_states_single \
        && topic_ready /end_pose \
        && node_ready /robot_state_publisher \
        && param_ready /robot_description \
        && node_ready /move_group \
        && [[ "${mgr}" == "moveit_simple_controller_manager/MoveItSimpleControllerManager" ]] \
        && topic_ready /table_camera/color/image_raw \
        && topic_ready /table_camera/aligned_depth_to_color/image_raw \
        && topic_ready /table_camera/color/camera_info \
        && tf_ready base_link gripper_base
}

start_lower_stack() {
    stage="lower Piper stack"
    if [[ "${MODE}" == "status" ]]; then
        return 0
    fi
    if lower_stack_healthy; then
        log "Lower Piper stack is already healthy; not restarting ${LOWER_SESSION}."
        return 0
    fi
    log "Starting lower Piper stack via start_abotclaw_all.sh --restart --no-attach."
    (cd "${REPO_DIR}" && ./start_abotclaw_all.sh --restart --no-attach)
    wait_until "ROS master" 60 1 ros_ready || true
    wait_until "/joint_states_single" 60 1 topic_ready /joint_states_single || true
    wait_until "/move_group" 90 1 node_ready /move_group || true
    wait_until "lower Piper stack readiness" 90 1 lower_stack_healthy || true
}

agent_healthy() {
    http_ok "${AGENT_URL}/health" 2
}

agent_server_process_running() {
    container_exec "pgrep -af 'python3 server.py --port 8888' >/dev/null 2>&1"
}

start_agent_server() {
    stage="official Piper Agent Server"
    if [[ "${MODE}" == "status" ]]; then
        return 0
    fi
    if agent_healthy; then
        log "Agent Server already healthy on port 8888."
        return 0
    fi
    local cmd
    cmd="mkdir -p $(printf '%q' "${LOG_DIR}"); exec docker exec -i ${CONTAINER} bash -lc 'source /opt/ros/noetic/setup.bash; source ${ROS_WS}/devel/setup.bash; export ROS_MASTER_URI=http://localhost:11311; export ROS_HOSTNAME=localhost; cd ${AGENT_DIR}; exec python3 server.py --port 8888 --no-reset-on-release' 2>&1 | tee -a $(printf '%q' "${LOG_DIR}/agent-server.log")"
    tmux_start_window "agent-server" "${cmd}"
    wait_until "Agent Server health" 60 1 agent_healthy || true
}

start_openclaw() {
    stage="OpenClaw gateway"
    if [[ "${MODE}" == "status" ]]; then
        return 0
    fi
    if ! command -v openclaw >/dev/null 2>&1; then
        log "WARNING: openclaw command not found; gateway not started."
        set_status "OpenClaw gateway" "not ready: openclaw command missing"
        return 0
    fi
    if openclaw gateway health >/dev/null 2>&1; then
        log "OpenClaw gateway is already healthy; reusing installed gateway service."
    else
        log "Starting installed OpenClaw gateway service once."
        openclaw gateway start 2>&1 | tee -a "${LOG_DIR}/openclaw.log" || true
    fi
    local cmd
    cmd="cd $(printf '%q' "${REPO_DIR}"); openclaw gateway status; echo; echo 'OpenClaw gateway is managed by the installed service. Logs:'; echo '/tmp/openclaw/openclaw-*.log'; exec bash"
    tmux_start_window_if_missing "openclaw" "${cmd}"
}

openclaw_readonly_smoke() {
    if ! command -v openclaw >/dev/null 2>&1; then
        return 1
    fi
    openclaw gateway health >/dev/null 2>&1 \
        && openclaw gateway probe >/dev/null 2>&1 \
        && grep -q "http://localhost:8888" "${REPO_DIR}/openclaw_layer/ROBOT.md" \
        && http_ok "${AGENT_URL}/docs/guide/html" 3 \
        && http_ok "${AGENT_URL}/code/sdk/markdown" 3 \
        && http_ok "${AGENT_URL}/state" 3 \
        && http_ok "${AGENT_URL}/cameras/table_camera/frame" 5
}

ssh_remote() {
    ssh -o BatchMode=yes -o ConnectTimeout=5 "${REMOTE_USER}@${REMOTE_HOST}" "$@"
}

ssh_remote_loose() {
    ssh -o BatchMode=yes -o ConnectTimeout=5 "${REMOTE_USER}@${REMOTE_HOST}" "$@" 2>/dev/null || true
}

remote_health_json() {
    curl -fsS --max-time 4 "$1/health" 2>/dev/null || true
}

yolo_healthy() {
    local body model device status
    body="$(remote_health_json "${REMOTE_YOLO_URL}")"
    status="$(json_field "${body}" status)"
    model="$(json_field "${body}" model_loaded)"
    device="$(json_field "${body}" device)"
    [[ "${status}" == "ok" && "${model}" == "True" || "${status}" == "ok" && "${model}" == "true" ]] && [[ "${device}" == cuda* ]]
}

start_remote_yolo() {
    stage="remote YOLO service"
    if ! ssh_remote "true"; then
        set_status "SSH connectivity" "not ready"
        return 0
    fi
    set_status "SSH connectivity" "ok"
    local exists running
    exists="$(ssh_remote_loose "docker inspect '${REMOTE_YOLO_CONTAINER}' >/dev/null 2>&1 && echo yes || echo no")"
    if [[ "${exists}" != "yes" ]]; then
        set_status "yolo-5090-torch state" "missing"
        return 0
    fi
    running="$(ssh_remote_loose "docker inspect -f '{{.State.Status}}' '${REMOTE_YOLO_CONTAINER}' 2>/dev/null")"
    if [[ "${running}" != "running" && "${MODE}" != "status" ]]; then
        ssh_remote "docker start '${REMOTE_YOLO_CONTAINER}' >/dev/null"
        running="running"
    fi
    set_status "yolo-5090-torch state" "${running}"
    if yolo_healthy; then
        return 0
    fi
    if [[ "${MODE}" == "status" ]]; then
        return 0
    fi
    log "YOLO health not ready; starting official service inside existing ${REMOTE_YOLO_CONTAINER} container."
    ssh_remote "docker exec '${REMOTE_YOLO_CONTAINER}' bash -lc 'cd ${REMOTE_REPO}/service_layer/YOLO && test -f yolov5l6.pt && if ! pgrep -af \"python.*main.py\" >/dev/null; then nohup env PORT=8013 DEVICE=auto python3 main.py > /tmp/yolo-8013.log 2>&1 & fi'"
    wait_until "YOLO health" 90 2 yolo_healthy || true
}

check_optional_remote_service() {
    local name="$1"
    local port="$2"
    local url="http://${REMOTE_HOST}:${port}"
    local body status
    body="$(remote_health_json "${url}")"
    status="$(json_field "${body}" status)"
    if [[ "${status}" == "ok" ]]; then
        set_status "${name} status" "healthy"
    else
        set_status "${name} status" "not ready"
    fi
}

check_grasp_assets() {
    local report
    report="$(ssh_remote_loose "docker exec '${REMOTE_YOLO_CONTAINER}' bash -lc '
cd ${REMOTE_REPO}/service_layer/GraspAnything 2>/dev/null || { echo repo_missing; exit 0; }
missing=()
find . -maxdepth 4 -iname \"*license*\" -type d | grep -q . || missing+=(license_folder)
find . -path \"*gsnet*\" -type f \\( -name \"*.so\" -o -name \"*.pyd\" \\) | grep -q . || missing+=(gsnet_extension)
find . -type f \\( -name \"checkpoint_detection.tar\" -o -name \"*.pth\" -o -name \"*.pt\" \\) | grep -q . || missing+=(checkpoint)
if [ \${#missing[@]} -eq 0 ]; then echo assets_ok; else printf \"%s \" \"\${missing[@]}\"; fi
'")"
    if [[ "${report}" == *assets_ok* ]]; then
        return 0
    fi
    set_status "GraspAnything status" "blocked/not ready: missing ${report:-asset check failed}"
    return 1
}

remote_grasp_health_json() {
    curl -fsS --max-time 4 "${REMOTE_GRASP_URL}/health" 2>/dev/null || true
}

remote_grasp_backend() {
    local body backend model_loaded
    body="$(remote_grasp_health_json)"
    backend="$(json_field "${body}" backend)"
    model_loaded="$(json_field "${body}" model_loaded)"
    if [[ "${backend}" == "depth_fallback" ]]; then
        echo "depth fallback"
    elif [[ "${model_loaded}" == "True" || "${model_loaded}" == "true" ]]; then
        echo "AnyGrasp"
    else
        echo ""
    fi
}

remote_grasp_healthy() {
    [[ -n "$(remote_grasp_backend)" ]]
}

remote_anygrasp_assets_ready() {
    ssh_remote "docker exec '${REMOTE_YOLO_CONTAINER}' bash -lc '
cd ${REMOTE_REPO}/service_layer/GraspAnything
find . -maxdepth 4 -iname \"*license*\" -type d | grep -q .
find . -type f \\( -name \"checkpoint_detection.tar\" -o -name \"*.pth\" -o -name \"*.pt\" \\) | grep -q .
'" >/dev/null 2>&1
}

remote_stop_grasp_processes() {
    local mode="$1"
    ssh_remote "docker exec '${REMOTE_YOLO_CONTAINER}' bash -lc 'python3 - \"${REMOTE_REPO}/service_layer/GraspAnything\" \"${mode}\" <<'\"'\"'PY'\"'\"'
import os
import signal
import sys

target_dir = os.path.realpath(sys.argv[1])
mode = sys.argv[2]
for pid in filter(str.isdigit, os.listdir(\"/proc\")):
    try:
        cwd = os.path.realpath(os.readlink(f\"/proc/{pid}/cwd\"))
        args = [part.decode(\"utf-8\", \"ignore\") for part in open(f\"/proc/{pid}/cmdline\", \"rb\").read().split(b\"\\0\") if part]
    except OSError:
        continue
    if cwd != target_dir or not args:
        continue
    cmd = \" \".join(args)
    is_anygrasp = \" main.py\" in f\" {cmd}\" or cmd.endswith(\"/main.py\")
    is_fallback = \"grasp_service_depth_fallback.py\" in cmd
    if (mode == \"fallback\" and is_anygrasp) or (mode == \"anygrasp\" and is_fallback) or (mode == \"all\" and (is_anygrasp or is_fallback)):
        os.kill(int(pid), signal.SIGTERM)
PY'"
}

start_remote_grasp_service() {
    stage="remote grasp service"
    if ! ssh_remote "docker inspect '${REMOTE_YOLO_CONTAINER}' >/dev/null 2>&1"; then
        set_status "GraspAnything status" "not ready: ${REMOTE_YOLO_CONTAINER} missing"
        return 0
    fi
    local running backend body
    running="$(ssh_remote_loose "docker inspect -f '{{.State.Status}}' '${REMOTE_YOLO_CONTAINER}' 2>/dev/null")"
    if [[ "${running}" != "running" && "${MODE}" != "status" ]]; then
        ssh_remote "docker start '${REMOTE_YOLO_CONTAINER}' >/dev/null"
    fi

    backend="$(remote_grasp_backend)"
    if [[ -n "${backend}" ]]; then
        set_status "Grasp backend" "${backend}"
        set_status "GraspAnything status" "healthy"
        return 0
    fi
    if [[ "${MODE}" == "status" ]]; then
        set_status "Grasp backend" "not ready"
        check_grasp_assets || true
        return 0
    fi

    if remote_anygrasp_assets_ready; then
        log "Starting official AnyGrasp service on ${REMOTE_HOST}:8015."
        remote_stop_grasp_processes all
        ssh_remote "docker exec '${REMOTE_YOLO_CONTAINER}' bash -lc '
cd ${REMOTE_REPO}/service_layer/GraspAnything
checkpoint=\"\$(find . -type f \\( -name checkpoint_detection.tar -o -name \"*.pth\" -o -name \"*.pt\" \\) | head -n 1)\"
nohup env PORT=8015 GRASP_CHECKPOINT_PATH=\"\$checkpoint\" DEVICE=auto python3 main.py > /tmp/graspanything-8015.log 2>&1 &
'"
    else
        log "AnyGrasp license/checkpoint not available; starting depth fallback on ${REMOTE_HOST}:8015."
        remote_stop_grasp_processes all
        ssh_remote "docker exec '${REMOTE_YOLO_CONTAINER}' bash -lc '
cd ${REMOTE_REPO}/service_layer/GraspAnything
nohup env PORT=8015 YOLO_URL=http://127.0.0.1:8013 python3 grasp_service_depth_fallback.py > /tmp/grasp-depth-fallback-8015.log 2>&1 &
'"
    fi

    wait_until "remote grasp service health" 30 2 remote_grasp_healthy >/dev/null || true
    body="$(remote_grasp_health_json)"
    backend="$(remote_grasp_backend)"
    if [[ -n "${backend}" ]]; then
        set_status "Grasp backend" "${backend}"
        set_status "GraspAnything status" "healthy"
    else
        set_status "Grasp backend" "not ready"
        set_status "GraspAnything status" "not ready: $(json_field "${body}" status)"
    fi
}

check_remote_services() {
    stage="remote services"
    set_status "route to ${REMOTE_HOST}" "$(route_status)"
    if ssh_remote "true"; then
        set_status "SSH connectivity" "ok"
    else
        set_status "SSH connectivity" "not ready"
        return 0
    fi
    start_remote_yolo
    local body
    body="$(remote_health_json "${REMOTE_YOLO_URL}")"
    set_status "YOLO health" "$(json_field "${body}" status)"
    set_status "YOLO model_loaded" "$(json_field "${body}" model_loaded)"
    set_status "YOLO CUDA device" "$(json_field "${body}" device)"
    if docker_running && container_exec "curl -fsS --max-time 4 '${REMOTE_YOLO_URL}/health' >/dev/null 2>&1"; then
        set_status "container -> YOLO" "ok"
    else
        set_status "container -> YOLO" "not ready"
    fi
    check_optional_remote_service "SpatialMemory" 8012
    check_optional_remote_service "VLAC" 8014
    start_remote_grasp_service
}

print_calibration_guidance() {
    cat <<EOF
WARNING: Calibration sample quality depends on the physical setup.
  - MARKER_SIZE_M must equal the measured outside black-square width.
  - The marker must be rigid and flat.
  - The marker must not be mounted to moving gripper fingers.
  - Wait for the arm and marker pose to settle before taking each sample.
EOF
}

start_image_proc() {
    stage="image_proc rectification"
    if [[ "${MODE}" == "status" ]]; then
        return 0
    fi
    local image_proc_cmd
    image_proc_cmd="mkdir -p $(printf '%q' "${LOG_DIR}"); exec docker exec -i ${CONTAINER} bash -lc 'source /opt/ros/noetic/setup.bash; source ${ROS_WS}/devel/setup.bash; export ROS_MASTER_URI=http://localhost:11311; export ROS_HOSTNAME=localhost; until timeout 3 rostopic echo -n 1 /table_camera/color/image_raw/header >/dev/null 2>&1 && timeout 3 rostopic echo -n 1 /table_camera/color/camera_info >/dev/null 2>&1; do sleep 1; done; exec rosrun image_proc image_proc __name:=image_proc __ns:=/table_camera/color' 2>&1 | tee -a $(printf '%q' "${LOG_DIR}/image-proc.log")"
    if image_proc_ready; then
        log "image_proc is already rectifying ${RECTIFIED_IMAGE_TOPIC}; reusing it."
    else
        tmux_replace_window "image-rectification" "${image_proc_cmd}"
    fi
    wait_until "rectified image topic ${RECTIFIED_IMAGE_TOPIC}" 30 1 image_proc_ready || true
    log "image_proc command: rosrun image_proc image_proc __name:=image_proc __ns:=/table_camera/color"
    log "Rectified image topic: ${RECTIFIED_IMAGE_TOPIC}"
    log "Matching CameraInfo topic from the same camera namespace: ${RECTIFIED_CAMERA_INFO_TOPIC}"
}

start_aruco() {
    stage="ArUco"
    if [[ "${MODE}" == "status" ]]; then
        return 0
    fi
    wait_until "rectified image topic ${RECTIFIED_IMAGE_TOPIC}" 30 1 image_proc_ready || true
    local aruco_cmd keepalive_cmd
    aruco_cmd="mkdir -p $(printf '%q' "${LOG_DIR}"); exec docker exec -i ${CONTAINER} bash -lc 'source /opt/ros/noetic/setup.bash; source ${ROS_WS}/devel/setup.bash; export ROS_MASTER_URI=http://localhost:11311; export ROS_HOSTNAME=localhost; rosrun aruco_ros single /image:=${RECTIFIED_IMAGE_TOPIC} /camera_info:=${RECTIFIED_CAMERA_INFO_TOPIC} _marker_id:=${MARKER_ID} _marker_size:=${MARKER_SIZE_M} _reference_frame:=table_camera_color_optical_frame _camera_frame:=table_camera_color_optical_frame _marker_frame:=aruco_marker_frame _image_is_rectified:=true' 2>&1 | tee -a $(printf '%q' "${LOG_DIR}/aruco.log")"
    keepalive_cmd="mkdir -p $(printf '%q' "${LOG_DIR}"); exec docker exec -i ${CONTAINER} bash -lc 'source /opt/ros/noetic/setup.bash; source ${ROS_WS}/devel/setup.bash; export ROS_MASTER_URI=http://localhost:11311; export ROS_HOSTNAME=localhost; while true; do rostopic hz /aruco_simple/pose || true; sleep 1; done' 2>&1 | tee -a $(printf '%q' "${LOG_DIR}/aruco-keepalive.log")"
    if ! aruco_uses_rectified_image; then
        tmux_replace_window "aruco" "${aruco_cmd}"
    else
        log "ArUco detector is already using ${RECTIFIED_IMAGE_TOPIC}; reusing it."
    fi
    cleanup_aruco_keepalive
    tmux_start_window "aruco-keepalive" "${keepalive_cmd}"
    log "ArUco detector is running. Place marker ID ${MARKER_ID} in view of the D555e."
}

handeye_backend_ready() {
    rosnode_matches_live '^/piper_d555_eye_on_base/easy_handeye_calibration_server$'
}

handeye_gui_ready() {
    rosnode_matches_live '^/piper_d555_eye_on_base/(.*rqt.*|rqt_gui_py_node_[0-9]+)$'
}

easy_handeye_opencv_preflight() {
    container_exec "source /opt/ros/noetic/setup.bash; source '${ROS_WS}/devel/setup.bash'; source '${EASY_HANDEYE_WS}/devel/setup.bash'; export PYTHONPATH='${EASY_HANDEYE_SYSTEM_CV_PATH}':\${PYTHONPATH:-}; python3 - <<'PY'
import cv2
print(cv2.__version__)
print(cv2.__file__)
print(hasattr(cv2, 'calibrateHandEye'))
raise SystemExit(0 if hasattr(cv2, 'calibrateHandEye') else 42)
PY"
}

start_handeye_gui_only() {
    if [[ "${X11_AVAILABLE}" != "true" ]]; then
        set_status "rqt_easy_handeye sampling GUI" "not started: DISPLAY unavailable"
        return 0
    fi
    if handeye_gui_ready; then
        return 0
    fi
    local cmd
    cmd="mkdir -p $(printf '%q' "${LOG_DIR}"); exec docker exec -i ${DOCKER_X11_ARGS} ${CONTAINER} bash -lc 'source /opt/ros/noetic/setup.bash; source ${ROS_WS}/devel/setup.bash; source ${EASY_HANDEYE_WS}/devel/setup.bash; export PYTHONPATH=${EASY_HANDEYE_SYSTEM_CV_PATH}:\${PYTHONPATH:-}; export ROS_MASTER_URI=http://localhost:11311; export ROS_HOSTNAME=localhost; export ROS_NAMESPACE=/piper_d555_eye_on_base; exec rosrun rqt_easy_handeye rqt_easy_handeye' 2>&1 | tee -a $(printf '%q' "${LOG_DIR}/handeye-gui.log")"
    tmux_replace_window "handeye-gui" "${cmd}"
}

start_handeye() {
    stage="Easy Handeye"
    if [[ "${MODE}" == "status" ]]; then
        return 0
    fi
    if handeye_backend_ready; then
        log "Easy Handeye backend is already running; reusing it to preserve any live samples."
        start_handeye_gui_only
        return 0
    fi
    if ! easy_handeye_opencv_preflight 2>&1 | tee -a "${LOG_DIR}/handeye-opencv-preflight.log"; then
        echo "ERROR: Easy Handeye OpenCV preflight failed; cv2.calibrateHandEye is unavailable." >&2
        exit 1
    fi
    local sampling_gui_arg="start_sampling_gui:=true"
    if [[ "${X11_AVAILABLE}" != "true" ]]; then
        sampling_gui_arg="start_sampling_gui:=false"
        set_status "rqt_easy_handeye sampling GUI" "not started: DISPLAY unavailable"
    fi
    local cmd
    cmd="mkdir -p $(printf '%q' "${LOG_DIR}"); exec docker exec -i ${DOCKER_X11_ARGS} ${CONTAINER} bash -lc 'source /opt/ros/noetic/setup.bash; source ${ROS_WS}/devel/setup.bash; source ${EASY_HANDEYE_WS}/devel/setup.bash; export PYTHONPATH=${EASY_HANDEYE_SYSTEM_CV_PATH}:\${PYTHONPATH:-}; export ROS_MASTER_URI=http://localhost:11311; export ROS_HOSTNAME=localhost; python3 - <<\"PY\"
import cv2
print(cv2.__version__)
print(cv2.__file__)
print(hasattr(cv2, \"calibrateHandEye\"))
raise SystemExit(0 if hasattr(cv2, \"calibrateHandEye\") else 42)
PY
exec roslaunch easy_handeye calibrate.launch eye_on_hand:=false namespace_prefix:=piper_d555 robot_base_frame:=base_link robot_effector_frame:=gripper_base tracking_base_frame:=table_camera_color_optical_frame tracking_marker_frame:=aruco_marker_frame freehand_robot_movement:=true start_rviz:=false publish_dummy:=true ${sampling_gui_arg}' 2>&1 | tee -a $(printf '%q' "${LOG_DIR}/handeye.log")"
    tmux_replace_window "handeye" "${cmd}"
}

rviz_ready() {
    rosnode_matches_live '^/rviz'
}

start_rviz() {
    stage="RViz"
    if [[ "${MODE}" == "status" ]]; then
        return 0
    fi
    if rviz_ready; then
        log "RViz is already running; reusing it."
        return 0
    fi
    if [[ "${X11_AVAILABLE}" != "true" ]]; then
        set_status "RViz" "not started: DISPLAY unset"
        return 0
    fi
    local cmd
    cmd="mkdir -p $(printf '%q' "${LOG_DIR}"); exec docker exec -i ${DOCKER_X11_ARGS} ${CONTAINER} bash -lc 'source /opt/ros/noetic/setup.bash; source ${ROS_WS}/devel/setup.bash; export ROS_MASTER_URI=http://localhost:11311; export ROS_HOSTNAME=localhost; roslaunch piper_with_gripper_moveit moveit_rviz.launch rviz_config:=${ROS_WS}/src/piper_ros/src/piper_moveit/piper_with_gripper_moveit/launch/moveit.rviz' 2>&1 | tee -a $(printf '%q' "${LOG_DIR}/rviz.log")"
    tmux_replace_window "rviz" "${cmd}"
}

update_local_status() {
    stage="local status"
    set_status "Docker container state" "$(docker inspect -f '{{.State.Status}}' "${CONTAINER}" 2>/dev/null || echo missing)"
    ros_ready && set_status "ROS master" "ok" || set_status "ROS master" "not ready"
    container_ros "rosnode list 2>/dev/null | grep -Eq '^/piper_ctrl_single_node(_|$)'" && set_status "Piper hardware node" "ok" || set_status "Piper hardware node" "not ready"
    topic_ready /joint_states && set_status "/joint_states" "ok" || set_status "/joint_states" "not ready"
    topic_ready /joint_states_single && set_status "/joint_states_single" "ok" || set_status "/joint_states_single" "not ready"
    topic_ready /end_pose && set_status "/end_pose" "ok" || set_status "/end_pose" "not ready"
    node_ready /robot_state_publisher && set_status "robot_state_publisher" "ok" || set_status "robot_state_publisher" "not ready"
    tf_ready base_link gripper_base && set_status "base_link -> gripper_base TF" "ok" || set_status "base_link -> gripper_base TF" "not ready"
    topic_ready /table_camera/color/image_raw && set_status "RealSense RGB" "ok" || set_status "RealSense RGB" "not ready"
    topic_ready /table_camera/aligned_depth_to_color/image_raw && set_status "RealSense aligned depth" "ok" || set_status "RealSense aligned depth" "not ready"
    topic_ready /table_camera/color/camera_info && set_status "camera_info" "ok" || set_status "camera_info" "not ready"
    image_proc_ready && set_status "image_proc rectification" "ok: ${RECTIFIED_IMAGE_TOPIC}" || set_status "image_proc rectification" "not ready"
    node_ready /move_group && set_status "move_group" "ok" || set_status "move_group" "not ready"
    local mgr
    mgr="$(container_ros "rosparam get /move_group/moveit_controller_manager 2>/dev/null || true" | tr -d '\r')"
    set_status "MoveIt controller manager" "${mgr:-not ready}"
    agent_healthy && set_status "Agent Server port 8888" "healthy" || set_status "Agent Server port 8888" "not ready"
    validate_agent_state
    if command -v openclaw >/dev/null 2>&1 && openclaw gateway health >/dev/null 2>&1; then
        set_status "OpenClaw gateway" "running"
    else
        : "${STATUS["OpenClaw gateway"]:=not ready}"
    fi
    openclaw_readonly_smoke && set_status "OpenClaw read-only smoke" "ok" || set_status "OpenClaw read-only smoke" "not ready"
    node_ready /aruco_simple && set_status "ArUco node" "running" || set_status "ArUco node" "not ready"
    topic_ready /aruco_simple/pose && set_status "ArUco pose topic" "ok" || set_status "ArUco pose topic" "waiting for marker/subscriber"
    tf_ready table_camera_color_optical_frame aruco_marker_frame && set_status "marker TF" "ok" || set_status "marker TF" "waiting for marker"
    handeye_backend_ready && set_status "Easy Handeye backend" "running" || set_status "Easy Handeye backend" "not ready"
    handeye_gui_ready && set_status "rqt_easy_handeye sampling GUI" "running" || : "${STATUS["rqt_easy_handeye sampling GUI"]:=not running}"
    rviz_ready && set_status "RViz" "running" || : "${STATUS["RViz"]:=not ready}"
}

validate_agent_state() {
    local body verdict
    body="$(curl -fsS --max-time 4 "${AGENT_URL}/state" 2>/dev/null || true)"
    if [[ -z "${body}" ]]; then
        set_status "Agent Server real robot state" "not ready"
        return 0
    fi
    verdict="$(python3 - <<'PY' "${body}" 2>/dev/null || true
import json, sys
try:
    s=json.loads(sys.argv[1])
    arm = s.get("arm", {})
    robot = s.get("robot", {})
    joints = s.get("joints") or s.get("joint_positions") or arm.get("joint_positions") or robot.get("joint_positions")
    ee = s.get("end_pose") or s.get("ee_pose") or s.get("end_effector") or arm.get("end_pose") or robot.get("end_pose")
    grip = s.get("gripper") or s.get("gripper_position") or robot.get("gripper_position")
    cams = s.get("cameras") or {}
    ok_j = isinstance(joints, list) and len(joints) >= 6
    ok_e = bool(ee)
    ok_g = grip is not None
    ok_c = "table_camera" in str(cams) or "cam_low" in str(cams)
    print("ok" if ok_j and ok_e and ok_g and ok_c else f"partial joints={ok_j} end_pose={ok_e} gripper={ok_g} camera={ok_c}")
except Exception as e:
    print(f"invalid: {e}")
PY
)"
    set_status "Agent Server real robot state" "${verdict:-not ready}"
}

print_summary() {
    cat <<EOF

ABot-Claw full-stack status
===========================
Local:
  Docker container state:              ${STATUS["Docker container state"]:-unknown}
  ROS master:                          ${STATUS["ROS master"]:-unknown}
  Piper hardware node:                 ${STATUS["Piper hardware node"]:-unknown}
  /joint_states:                       ${STATUS["/joint_states"]:-unknown}
  /joint_states_single:                ${STATUS["/joint_states_single"]:-unknown}
  /end_pose:                           ${STATUS["/end_pose"]:-unknown}
  robot_state_publisher:               ${STATUS["robot_state_publisher"]:-unknown}
  base_link -> gripper_base TF:        ${STATUS["base_link -> gripper_base TF"]:-unknown}
  RealSense RGB:                       ${STATUS["RealSense RGB"]:-unknown}
  RealSense aligned depth:             ${STATUS["RealSense aligned depth"]:-unknown}
  camera_info:                         ${STATUS["camera_info"]:-unknown}
  image_proc rectification:            ${STATUS["image_proc rectification"]:-unknown}
  move_group:                          ${STATUS["move_group"]:-unknown}
  MoveIt controller manager:           ${STATUS["MoveIt controller manager"]:-unknown}
  Agent Server port 8888:              ${STATUS["Agent Server port 8888"]:-unknown}
  Agent Server real robot state:       ${STATUS["Agent Server real robot state"]:-unknown}
  OpenClaw gateway:                    ${STATUS["OpenClaw gateway"]:-unknown}
  OpenClaw read-only smoke:            ${STATUS["OpenClaw read-only smoke"]:-unknown}
  ArUco node:                          ${STATUS["ArUco node"]:-unknown}
  ArUco pose topic:                    ${STATUS["ArUco pose topic"]:-unknown}
  marker TF:                           ${STATUS["marker TF"]:-unknown}
  Easy Handeye backend:                ${STATUS["Easy Handeye backend"]:-unknown}
  rqt_easy_handeye sampling GUI:       ${STATUS["rqt_easy_handeye sampling GUI"]:-unknown}
  RViz:                                ${STATUS["RViz"]:-unknown}

Remote:
  route to ${REMOTE_HOST}:             ${STATUS["route to ${REMOTE_HOST}"]:-unknown}
  SSH connectivity:                    ${STATUS["SSH connectivity"]:-unknown}
  yolo-5090-torch state:               ${STATUS["yolo-5090-torch state"]:-unknown}
  YOLO health:                         ${STATUS["YOLO health"]:-unknown}
  YOLO model_loaded:                   ${STATUS["YOLO model_loaded"]:-unknown}
  YOLO CUDA device:                    ${STATUS["YOLO CUDA device"]:-unknown}
  SpatialMemory status:                ${STATUS["SpatialMemory status"]:-unknown}
  VLAC status:                         ${STATUS["VLAC status"]:-unknown}
  GraspAnything status:                ${STATUS["GraspAnything status"]:-unknown}
  Grasp backend:                       ${STATUS["Grasp backend"]:-unknown}

Attach to the full-stack tmux session:
  tmux attach -t ${SESSION}

Detach safely:
  Ctrl+B, release, then D

No robot movement was commanded by this launcher.
EOF
}

main() {
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --status) MODE="status" ;;
            -h|--help) usage; exit 0 ;;
            *) echo "ERROR: unknown argument: $1" >&2; usage >&2; exit 2 ;;
        esac
        shift
    done

    preflight
    if [[ "${MODE}" != "status" ]]; then
        ensure_lan_route || true
        ensure_full_tmux
        prepare_x11 || true
    fi
    start_lower_stack
    start_agent_server
    start_openclaw
    check_remote_services
    start_image_proc
    print_calibration_guidance
    start_aruco
    start_handeye
    start_rviz
    update_local_status
    print_summary
}

main "$@"
