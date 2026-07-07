#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS="${SCRIPT_DIR}/robot_driver_ros"

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-localhost}"

source /opt/ros/noetic/setup.bash
source "${ROS_WS}/devel/setup.bash"

CAMERA_NAME="${REALSENSE_CAMERA_NAME:-wrist_camera}"
DEVICE_TYPE="${REALSENSE_DEVICE_TYPE:-d555}"
COLOR_WIDTH="${REALSENSE_COLOR_WIDTH:-640}"
COLOR_HEIGHT="${REALSENSE_COLOR_HEIGHT:-480}"
DEPTH_WIDTH="${REALSENSE_DEPTH_WIDTH:-640}"
DEPTH_HEIGHT="${REALSENSE_DEPTH_HEIGHT:-480}"
FPS="${REALSENSE_FPS:-30}"

echo "Starting RealSense ${DEVICE_TYPE} as /${CAMERA_NAME}"
echo "Expected topics:"
echo "  /${CAMERA_NAME}/color/image_raw"
echo "  /${CAMERA_NAME}/aligned_depth_to_color/image_raw"
echo "  /${CAMERA_NAME}/color/camera_info"

exec roslaunch realsense2_camera rs_camera.launch \
  camera:="${CAMERA_NAME}" \
  tf_prefix:="${CAMERA_NAME}" \
  device_type:="${DEVICE_TYPE}" \
  enable_color:=true \
  enable_depth:=true \
  align_depth:=true \
  enable_sync:=true \
  color_width:="${COLOR_WIDTH}" \
  color_height:="${COLOR_HEIGHT}" \
  depth_width:="${DEPTH_WIDTH}" \
  depth_height:="${DEPTH_HEIGHT}" \
  color_fps:="${FPS}" \
  depth_fps:="${FPS}" \
  enable_infra:=false \
  enable_infra1:=false \
  enable_infra2:=false \
  enable_pointcloud:=false
