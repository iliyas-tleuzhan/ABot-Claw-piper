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

piper_driver_cmd=$(cat <<EOF
cd ${ROS_WS}
source /opt/ros/noetic/setup.bash
source devel/setup.bash
cd src/piper_ros
bash can_activate.sh can0 1000000
cd ${ROS_WS}
roslaunch piper start_single_piper.launch can_port:=can0 auto_enable:=true
EOF
)

moveit_cmd=$(cat <<EOF
cd ${ROS_WS}
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch piper_with_gripper_moveit demo.launch use_rviz:=false
EOF
)

action_server_cmd=$(cat <<EOF
cd ${STACK_DIR}
./start_piper_language_action_server.sh
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

tmux new-session -d -s "${SESSION}" -n "piper_driver" "docker exec -it ${CONTAINER} bash"
tmux send-keys -t "${SESSION}:piper_driver" "${piper_driver_cmd}" C-m

tmux new-window -t "${SESSION}" -n "moveit" "docker exec -it ${CONTAINER} bash"
tmux send-keys -t "${SESSION}:moveit" "${moveit_cmd}" C-m

tmux new-window -t "${SESSION}" -n "action_server" "docker exec -it ${CONTAINER} bash"
tmux send-keys -t "${SESSION}:action_server" "${action_server_cmd}" C-m

tmux new-window -t "${SESSION}" -n "tests" "docker exec -it ${CONTAINER} bash"
tmux send-keys -t "${SESSION}:tests" "${tests_cmd}" C-m

tmux select-window -t "${SESSION}:piper_driver"
exec tmux attach-session -t "${SESSION}"
