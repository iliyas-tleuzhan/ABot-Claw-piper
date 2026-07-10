#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS="${SCRIPT_DIR}/robot_driver_ros"

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-localhost}"

source /opt/ros/noetic/setup.bash
source "${ROS_WS}/devel/setup.bash"

CAMERA_NAME="${REALSENSE_CAMERA_NAME:-table_camera}"
SERIAL="${REALSENSE_SERIAL:-352222303634}"
WIDTH="${REALSENSE_COLOR_WIDTH:-640}"
HEIGHT="${REALSENSE_COLOR_HEIGHT:-360}"
FPS="${REALSENSE_FPS:-30}"

echo "Starting pyrealsense2 D555 publisher as /${CAMERA_NAME}"
exec python3 "${SCRIPT_DIR}/realsense_d555_py_publisher.py" \
  --camera "${CAMERA_NAME}" \
  --serial "${SERIAL}" \
  --width "${WIDTH}" \
  --height "${HEIGHT}" \
  --fps "${FPS}"
