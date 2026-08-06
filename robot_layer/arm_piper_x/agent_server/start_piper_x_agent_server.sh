#!/usr/bin/env bash
set -eo pipefail

cd /home/dase-hw101/ABot-Claw
source /opt/ros/jazzy/setup.bash
source /home/dase-hw101/agx_arm_ws/install/setup.bash
if [ -f /home/dase-hw101/ros2_ws/install/setup.bash ]; then
  source /home/dase-hw101/ros2_ws/install/setup.bash
fi

exec python3 robot_layer/arm_piper_x/agent_server/server.py --host 127.0.0.1 --port 8893
