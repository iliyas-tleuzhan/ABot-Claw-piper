#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash

: "${ROS_MASTER_URI:?Set ROS_MASTER_URI, for example http://192.168.1.154:11311}"
: "${ROS_IP:?Set ROS_IP to the 5090 host address, for example 192.168.1.104}"

echo "ROS_MASTER_URI=${ROS_MASTER_URI}"
echo "ROS_IP=${ROS_IP}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "${SCRIPT_DIR}/yolo_stream_viewer_5090.py" "$@"
