#!/usr/bin/env bash
set -eo pipefail

SESSION="piper_language_stack"
CONTAINER="abot-piper-noetic"
STACK_DIR="/root/Iliyas/abot/ABot-Claw/robot_layer/arm_piper/agent_server"
ROS_WS="${STACK_DIR}/robot_driver_ros"

if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux is not installed. Install it with: sudo apt install tmux"
  exit 1
fi

if tmux has-session -t "${SESSION}" 2>/dev/null; then
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
echo "Waiting for /joint_states_single..."
until rostopic list 2>/dev/null | grep -qx "/joint_states_single"; do sleep 1; done
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
echo "Waiting for /joint_states_single..."
until rostopic list 2>/dev/null | grep -qx "/joint_states_single"; do sleep 1; done
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
exec tmux attach-session -t "${SESSION}"
