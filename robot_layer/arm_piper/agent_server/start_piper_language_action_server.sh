#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PORT="${PIPER_LANGUAGE_ACTION_PORT:-8891}"

# ROS setup files can reference these variables; avoid set -u because those
# setup files are not strict-mode clean.
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-localhost}"

source /opt/ros/noetic/setup.bash
source "${SCRIPT_DIR}/robot_driver_ros/devel/setup.bash"

cd "${SCRIPT_DIR}"
echo "Starting piper_language_action_server_v1 on port ${PORT}"
exec python3 piper_language_action_server.py
