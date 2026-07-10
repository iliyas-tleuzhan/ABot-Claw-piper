#!/usr/bin/env bash
# Host-side ABot-Claw/Piper/RealSense infrastructure launcher.
# Requires Docker and tmux, and assumes the ROS1 container is abot-piper-noetic.
# It only starts infrastructure; it never moves the robot or runs calibration/pick-place.

set -euo pipefail

SESSION="abotclaw"
CONTAINER="abot-piper-noetic"
CONTAINER_REPO="/root/ABot-Claw"
STACK_DIR="${CONTAINER_REPO}/robot_layer/arm_piper/agent_server"
ROS_WS="${STACK_DIR}/robot_driver_ros"
USE_FAKE_DEPTH=false

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
    cat <<'EOF'
Usage: ./start_abotclaw_all.sh [--restart] [--use-fake-depth]
       ./start_abotclaw_all.sh [--status|--stop]

  (no option)  Start the infrastructure if needed, then attach to tmux.
  --restart     Kill the existing abotclaw tmux session and start it fresh.
  --use-fake-depth  Use the raw ROS RealSense publisher plus fake aligned-depth bridge.
  --status      Show Docker, tmux, and ROS quick status without starting anything.
  --stop        Stop only the abotclaw tmux session; Docker continues running.
EOF
}

require_command() {
    if ! command -v "$1" >/dev/null 2>&1; then
        echo "ERROR: Required command not found: $1" >&2
        exit 1
    fi
}

docker_shell() {
    local command="$1"
    printf 'docker exec -it %q bash -lc %q' "${CONTAINER}" "${command}"
}

container_exists() {
    docker container inspect "${CONTAINER}" >/dev/null 2>&1
}

container_state() {
    docker inspect -f '{{.State.Status}}' "${CONTAINER}"
}

check_container() {
    if ! container_exists; then
        echo "ERROR: Docker container ${CONTAINER} does not exist." >&2
        echo "Create it with ${CONTAINER_REPO} mounted before running this script." >&2
        exit 1
    fi
}

check_repo_root() {
    if [[ ! -f "${SCRIPT_DIR}/README.md" || ! -d "${SCRIPT_DIR}/robot_layer" ]]; then
        echo "ERROR: start_abotclaw_all.sh must live at the ABot-Claw repository root." >&2
        exit 1
    fi
}

ensure_container_running() {
    local state
    state="$(container_state)"
    if [[ "${state}" != "running" ]]; then
        echo "Starting Docker container ${CONTAINER} (${state})..."
        docker start "${CONTAINER}" >/dev/null
    fi

    if ! docker exec "${CONTAINER}" test -d "${STACK_DIR}"; then
        echo "ERROR: ${CONTAINER} cannot see ${STACK_DIR}." >&2
        echo "Mount this repository at ${CONTAINER_REPO} inside the container." >&2
        exit 1
    fi
}

show_status() {
    check_container
    echo "Docker container ${CONTAINER}: $(container_state)"
    if tmux has-session -t "${SESSION}" 2>/dev/null; then
        echo "tmux session ${SESSION}: running"
    else
        echo "tmux session ${SESSION}: not running"
    fi

    if [[ "$(container_state)" == "running" ]]; then
        echo "ROS quick status:"
        docker exec "${CONTAINER}" bash -lc '
            source /opt/ros/noetic/setup.bash
            source /root/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros/devel/setup.bash
            export ROS_MASTER_URI=http://localhost:11311
            export ROS_HOSTNAME=localhost
            rostopic list 2>/dev/null | head -n 10 || true
        '
    fi
}

ROS_ENV=$(cat <<EOF
source /opt/ros/noetic/setup.bash
source ${ROS_WS}/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
EOF
)

roscore_cmd=$(cat <<EOF
${ROS_ENV}
roscore
EOF
)

piper_driver_cmd=$(cat <<EOF
${ROS_ENV}
cd ${ROS_WS}

ip link set can0 down 2>/dev/null || true
ip link set can0 type can bitrate 1000000
ip link set can0 txqueuelen 1000
ip link set can0 up

ip link show can0

until rostopic list >/dev/null 2>&1; do sleep 1; done
roslaunch piper start_single_piper.launch
EOF
)

raw_realsense_cmd=$(cat <<EOF
${ROS_ENV}
cd ${ROS_WS}

export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:\$LD_LIBRARY_PATH

pkill -f '[r]ealsense_d555_py_publisher.py' || true
pkill -f '[r]ealsense2_camera' || true
pkill -f '[f]ake_aligned_depth_from_raw.py' || true

until rostopic list >/dev/null 2>&1; do sleep 1; done
roslaunch realsense2_camera rs_camera.launch \\
  camera:=table_camera \\
  serial_no:=352222303634 \\
  align_depth:=false \\
  enable_color:=true \\
  enable_depth:=true \\
  enable_infra1:=false \\
  enable_infra2:=false \\
  enable_confidence:=false \\
  color_width:=640 \\
  color_height:=480 \\
  depth_width:=640 \\
  depth_height:=360 \\
  color_fps:=30 \\
  depth_fps:=30
EOF
)

real_aligned_realsense_cmd=$(cat <<EOF
${ROS_ENV}
cd ${STACK_DIR}

pkill -f '[r]ealsense_d555_py_publisher.py' || true
pkill -f '[r]ealsense2_camera' || true
pkill -f '[f]ake_aligned_depth_from_raw.py' || true

until rostopic list >/dev/null 2>&1; do sleep 1; done
./start_realsense_d555_py.sh
EOF
)

fake_aligned_depth_cmd=$(cat <<EOF
${ROS_ENV}
cd ${STACK_DIR}
echo "Waiting 8 seconds for the raw RealSense topics..."
sleep 8
if [[ -f ./fake_aligned_depth_from_raw.py ]]; then
  ./fake_aligned_depth_from_raw.py
else
  echo "WARNING: fake_aligned_depth_from_raw.py is missing; aligned-depth relay was not started."
  exec bash
fi
EOF
)

moveit_cmd=$(cat <<EOF
${ROS_ENV}
cd ${ROS_WS}
echo "WARNING: 'Fake execution of trajectory' means MoveIt is planning only, not commanding real hardware."
until rostopic list >/dev/null 2>&1; do sleep 1; done
roslaunch piper_with_gripper_moveit demo.launch rviz:=false
EOF
)

health_checks_cmd=$(cat <<EOF
${ROS_ENV}
echo "Waiting 12 seconds for infrastructure startup..."
sleep 12
set +e
rostopic list | head
rostopic echo -n 1 /joint_states_single
rostopic echo -n 1 /end_pose

check_topic_publishers() {
  local topic="\$1"
  local max_publishers="\$2"
  local info publishers
  info="\$(rostopic info "\$topic" 2>&1)"
  echo "\$info"
  publishers="\$(printf '%s\n' "\$info" | awk '/^Publishers:/{in_publishers=1; next} /^Subscribers:/{in_publishers=0} in_publishers && /\*/{count++} END{print count+0}')"
  if [[ "\$publishers" -eq 0 ]]; then
    echo "WARNING: \$topic has zero publishers."
  elif [[ "\$max_publishers" -gt 0 && "\$publishers" -gt "\$max_publishers" ]]; then
    echo "WARNING: \$topic has \$publishers publishers; expected at most \$max_publishers."
  fi
}

check_topic_publishers /table_camera/color/image_raw 0
check_topic_publishers /table_camera/aligned_depth_to_color/image_raw 1
check_topic_publishers /table_camera/color/camera_info 0
timeout 8 rostopic hz /table_camera/color/image_raw
timeout 8 rostopic hz /table_camera/aligned_depth_to_color/image_raw
timeout 8 rostopic echo -n 1 /table_camera/color/camera_info
rosservice list | grep joint_moveit_ctrl || true
cat <<'NEXT_STEPS'

ABot-Claw infrastructure started.
Next manual commands:
cd /root/ABot-Claw/robot_layer/arm_piper/agent_server
python3 calibrate_camera_to_base_points.py ...
python3 today_red_to_purple_pick_place.py --calibration camera_to_base.yaml
NEXT_STEPS
echo "WARNING: Motion scripts must be run manually after reviewing these health checks."
exec bash
EOF
)

main() {
    local action="start"
    local restart=false
    while [[ $# -gt 0 ]]; do
        case "$1" in
            -h|--help)
                usage
                exit 0
                ;;
            --restart)
                restart=true
                ;;
            --use-fake-depth)
                USE_FAKE_DEPTH=true
                ;;
            --stop)
                action="stop"
                ;;
            --status)
                action="status"
                ;;
            *)
                echo "ERROR: Unknown option: $1" >&2
                usage >&2
                exit 2
                ;;
        esac
        shift
    done

    if [[ "${action}" != "start" && ( "${restart}" == true || "${USE_FAKE_DEPTH}" == true ) ]]; then
        echo "ERROR: --restart and --use-fake-depth can only be used when starting." >&2
        exit 2
    fi

    if [[ "${action}" == "stop" ]]; then
        require_command tmux
        if tmux has-session -t "${SESSION}" 2>/dev/null; then
            tmux kill-session -t "${SESSION}"
            echo "Stopped tmux session ${SESSION}. Docker container remains running."
        else
            echo "tmux session ${SESSION} is not running."
        fi
        exit 0
    fi

    if [[ "${action}" == "status" ]]; then
        require_command docker
        require_command tmux
        show_status
        exit 0
    fi

    require_command docker
    require_command tmux
    check_repo_root
    check_container
    ensure_container_running

    if tmux has-session -t "${SESSION}" 2>/dev/null; then
        if [[ "${restart}" == true ]]; then
            echo "Restarting tmux session ${SESSION}..."
            tmux kill-session -t "${SESSION}"
        else
            echo "tmux session ${SESSION} already exists; attaching without starting duplicate services."
            exec tmux attach-session -t "${SESSION}"
        fi
    fi

    echo "WARNING: This starts infrastructure only. It does not move the robot."
    echo "WARNING: MoveIt 'Fake execution of trajectory' means planning only, not real hardware control."
    if [[ "${USE_FAKE_DEPTH}" == true ]]; then
        echo "WARNING: Using fake aligned depth fallback, not RealSense hardware alignment."
    else
        echo "Using the RealSense Python publisher with hardware depth-to-color alignment."
    fi

    tmux new-session -d -s "${SESSION}" -n "roscore" "$(docker_shell "${roscore_cmd}")"
    tmux new-window -t "${SESSION}" -n "piper_driver" "$(docker_shell "${piper_driver_cmd}")"
    if [[ "${USE_FAKE_DEPTH}" == true ]]; then
        tmux new-window -t "${SESSION}" -n "realsense_raw" "$(docker_shell "${raw_realsense_cmd}")"
        tmux new-window -t "${SESSION}" -n "fake_aligned_depth" "$(docker_shell "${fake_aligned_depth_cmd}")"
    else
        tmux new-window -t "${SESSION}" -n "realsense" "$(docker_shell "${real_aligned_realsense_cmd}")"
    fi
    tmux new-window -t "${SESSION}" -n "moveit_services" "$(docker_shell "${moveit_cmd}")"
    tmux new-window -t "${SESSION}" -n "health_checks" "$(docker_shell "${health_checks_cmd}")"
    tmux select-window -t "${SESSION}:roscore"

    echo "Started tmux session ${SESSION}. Attach with: tmux attach -t ${SESSION}"
    exec tmux attach-session -t "${SESSION}"
}

main "$@"
