#!/usr/bin/env bash
set -eo pipefail

cat <<'EOF'
Docker path prerequisite:

The container must mount the moved repo as:
  /home/dase-hw101/ABot-Claw:/root/ABot-Claw

Terminal 1 Piper driver:

docker exec -it abot-piper-noetic bash
cd /root/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros
source /opt/ros/noetic/setup.bash
source devel/setup.bash
cd src/piper_ros
bash can_activate.sh can0 1000000
cd /root/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros
roslaunch piper start_single_piper.launch can_port:=can0 auto_enable:=true

Terminal 2 MoveIt:

docker exec -it abot-piper-noetic bash
cd /root/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch piper_with_gripper_moveit demo.launch use_rviz:=false

Terminal 3 action server:

docker exec -it abot-piper-noetic bash
cd /root/ABot-Claw/robot_layer/arm_piper/agent_server
./start_piper_language_action_server.sh

Terminal 4 curl/OpenClaw tests:

docker exec -it abot-piper-noetic bash
cd /root/ABot-Claw/robot_layer/arm_piper/agent_server
curl http://localhost:8891/health
curl http://localhost:8891/state
./test_piper_language_action_server.sh
./test_piper_language_action_server.sh --gripper
./test_piper_language_action_server.sh --move

Host OpenClaw language tests:

cd ~/ABot-Claw/robot_layer/arm_piper/agent_server
openclaw agent --agent main --message "Move the Piper arm up."
openclaw agent --agent main --message "Move the Piper arm down."
openclaw agent --agent main --message "Open the Piper gripper."
openclaw agent --agent main --message "Close the Piper gripper."
EOF
