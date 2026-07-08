#!/usr/bin/env bash
set -eo pipefail

SESSION="piper_language_stack"
CONTAINER="abot-piper-noetic"
STACK_DIR="/root/ABot-Claw/robot_layer/arm_piper/agent_server"
ROS_WS="${STACK_DIR}/robot_driver_ros"
ATTACH="${PIPER_TMUX_ATTACH:-1}"

if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux is not installed. Install it with: sudo apt install tmux"
  exit 1
fi

if ! docker container inspect "${CONTAINER}" >/dev/null 2>&1; then
  echo "Container ${CONTAINER} does not exist."
  echo "Create it with the repo mounted at:"
  echo "  -v /home/dase-hw101/ABot-Claw:/root/ABot-Claw"
  exit 1
fi

container_state="$(docker inspect -f '{{.State.Status}}' "${CONTAINER}")"
if [[ "${container_state}" != "running" ]]; then
  echo "Starting stopped container ${CONTAINER} (${container_state})..."
  docker start "${CONTAINER}" >/dev/null
fi

if ! docker exec "${CONTAINER}" test -d "${STACK_DIR}" >/dev/null 2>&1; then
  echo "Container ${CONTAINER} cannot see ${STACK_DIR}"
  echo "The repo was moved to ~/ABot-Claw; recreate or remount the container with:"
  echo "  -v /home/dase-hw101/ABot-Claw:/root/ABot-Claw"
  exit 1
fi

if tmux has-session -t "${SESSION}" 2>/dev/null; then
  if [[ "${ATTACH}" == "0" ]]; then
    echo "tmux session ${SESSION} is already running"
    exit 0
  fi
  exec tmux attach-session -t "${SESSION}"
fi

docker_shell() {
  local command="$1"
  printf 'docker exec -it %q bash -lc %q' "${CONTAINER}" "${command}"
}

piper_driver_cmd=$(cat <<EOF
cd ${ROS_WS}
source /opt/ros/noetic/setup.bash
source devel/setup.bash
cd src/piper_ros
bash can_activate.sh can0 1000000
cd ${ROS_WS}
roslaunch piper start_single_piper.launch can_port:=can0 auto_enable:=true
exec bash
EOF
)

moveit_cmd=$(cat <<EOF
cd ${ROS_WS}
source /opt/ros/noetic/setup.bash
source devel/setup.bash
echo "Waiting for ROS master from Piper driver..."
until rostopic list >/dev/null 2>&1; do sleep 1; done
echo "Waiting for live /joint_states_single messages..."
until timeout 3 rostopic echo -n 1 /joint_states_single >/dev/null 2>&1; do sleep 1; done
roslaunch piper_with_gripper_moveit demo.launch use_rviz:=false
exec bash
EOF
)

action_server_cmd=$(cat <<EOF
cd ${STACK_DIR}
source /opt/ros/noetic/setup.bash
source robot_driver_ros/devel/setup.bash
echo "Waiting for ROS master from Piper driver..."
until rostopic list >/dev/null 2>&1; do sleep 1; done
echo "Waiting for live /joint_states_single messages..."
until timeout 3 rostopic echo -n 1 /joint_states_single >/dev/null 2>&1; do sleep 1; done
./start_piper_language_action_server.sh
exec bash
EOF
)

tests_cmd=$(cat <<EOF
cd ${STACK_DIR}
cat <<'COMMANDS'
Helpful Piper language action checks:
curl http://localhost:8891/health
curl http://localhost:8891/state
./test_piper_language_action_server.sh
./test_piper_language_action_server.sh --gripper
./test_piper_language_action_server.sh --move
COMMANDS
exec bash
EOF
)

tmux new-session -d -s "${SESSION}" -n "piper_driver" "$(docker_shell "${piper_driver_cmd}")"

tmux new-window -t "${SESSION}" -n "moveit" "$(docker_shell "${moveit_cmd}")"

tmux new-window -t "${SESSION}" -n "action_server" "$(docker_shell "${action_server_cmd}")"

tmux new-window -t "${SESSION}" -n "tests" "$(docker_shell "${tests_cmd}")"

tmux select-window -t "${SESSION}:piper_driver"
if [[ "${ATTACH}" == "0" ]]; then
  echo "Started tmux session ${SESSION}"
  echo "Attach with: tmux attach -t ${SESSION}"
  exit 0
fi
exec tmux attach-session -t "${SESSION}"
