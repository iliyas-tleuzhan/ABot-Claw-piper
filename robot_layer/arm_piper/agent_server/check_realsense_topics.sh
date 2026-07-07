#!/usr/bin/env bash
set -eo pipefail

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-localhost}"

source /opt/ros/noetic/setup.bash
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/robot_driver_ros/devel/setup.bash"

CAMERA_NAME="${REALSENSE_CAMERA_NAME:-wrist_camera}"
COLOR_TOPIC="/${CAMERA_NAME}/color/image_raw"
DEPTH_TOPIC="/${CAMERA_NAME}/aligned_depth_to_color/image_raw"
INFO_TOPIC="/${CAMERA_NAME}/color/camera_info"

check_topic() {
  local topic="$1"
  local label="$2"
  if timeout 5 rostopic echo -n 1 "${topic}" >/dev/null 2>&1; then
    echo "PASS ${label}: ${topic}"
  else
    echo "FAIL ${label}: no message on ${topic}"
    return 1
  fi
}

check_topic "${COLOR_TOPIC}" "color image"
check_topic "${DEPTH_TOPIC}" "aligned depth"
check_topic "${INFO_TOPIC}" "camera info"
