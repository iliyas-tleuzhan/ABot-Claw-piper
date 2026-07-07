#!/usr/bin/env python3
"""Publish D555 RGB, aligned depth, and camera info to ROS1 using pyrealsense2."""

from __future__ import annotations

import argparse
import time

import numpy as np
import pyrealsense2 as rs
import rospy
from sensor_msgs.msg import CameraInfo, Image


def make_camera_info(width: int, height: int, intrinsics: rs.intrinsics, frame_id: str) -> CameraInfo:
    msg = CameraInfo()
    msg.width = width
    msg.height = height
    msg.distortion_model = "plumb_bob"
    msg.D = list(intrinsics.coeffs[:5])
    msg.K = [
        intrinsics.fx, 0.0, intrinsics.ppx,
        0.0, intrinsics.fy, intrinsics.ppy,
        0.0, 0.0, 1.0,
    ]
    msg.R = [
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
    ]
    msg.P = [
        intrinsics.fx, 0.0, intrinsics.ppx, 0.0,
        0.0, intrinsics.fy, intrinsics.ppy, 0.0,
        0.0, 0.0, 1.0, 0.0,
    ]
    msg.header.frame_id = frame_id
    return msg


def make_image_msg(array: np.ndarray, encoding: str, stamp: rospy.Time, frame_id: str) -> Image:
    arr = np.ascontiguousarray(array)
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = int(arr.shape[0])
    msg.width = int(arr.shape[1])
    msg.encoding = encoding
    msg.is_bigendian = False
    if encoding == "bgr8":
        msg.step = int(msg.width * 3)
    elif encoding == "16UC1":
        msg.step = int(msg.width * 2)
    else:
        raise ValueError(f"Unsupported encoding: {encoding}")
    msg.data = arr.tobytes()
    return msg


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--camera", default="table_camera")
    p.add_argument("--serial", default="")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=360)
    p.add_argument("--fps", type=int, default=30)
    return p.parse_args()


def main() -> int:
    args = parse_args()
    rospy.init_node("realsense_d555_py_publisher", anonymous=True)

    color_topic = f"/{args.camera}/color/image_raw"
    depth_topic = f"/{args.camera}/aligned_depth_to_color/image_raw"
    info_topic = f"/{args.camera}/color/camera_info"
    frame_id = f"{args.camera}_color_optical_frame"

    color_pub = rospy.Publisher(color_topic, Image, queue_size=1)
    depth_pub = rospy.Publisher(depth_topic, Image, queue_size=1)
    info_pub = rospy.Publisher(info_topic, CameraInfo, queue_size=1)

    pipeline = rs.pipeline()
    config = rs.config()
    if args.serial:
        config.enable_device(args.serial)
    config.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)
    config.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, args.fps)

    profile = pipeline.start(config)
    align = rs.align(rs.stream.color)
    color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
    intrinsics = color_stream.get_intrinsics()
    camera_info = make_camera_info(args.width, args.height, intrinsics, frame_id)

    device = profile.get_device()
    rospy.loginfo("Started %s serial=%s", device.get_info(rs.camera_info.name), device.get_info(rs.camera_info.serial_number))
    rospy.loginfo("Publishing %s", color_topic)
    rospy.loginfo("Publishing %s", depth_topic)
    rospy.loginfo("Publishing %s", info_topic)

    try:
        while not rospy.is_shutdown():
            frames = pipeline.wait_for_frames(timeout_ms=5000)
            aligned = align.process(frames)
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            stamp = rospy.Time.now()
            color = np.asanyarray(color_frame.get_data())
            depth = np.asanyarray(depth_frame.get_data())

            color_msg = make_image_msg(color, "bgr8", stamp, frame_id)
            depth_msg = make_image_msg(depth, "16UC1", stamp, frame_id)

            camera_info.header.stamp = stamp
            camera_info.header.frame_id = frame_id

            color_pub.publish(color_msg)
            depth_pub.publish(depth_msg)
            info_pub.publish(camera_info)
    finally:
        pipeline.stop()
        time.sleep(0.2)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
