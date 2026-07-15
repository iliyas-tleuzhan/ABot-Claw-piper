#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo

latest_color_info = None
pub_img = None
pub_info = None

def color_info_cb(msg):
    global latest_color_info
    latest_color_info = msg

def depth_cb(msg):
    global latest_color_info, pub_img, pub_info

    if latest_color_info is None:
        return

    raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)

    target_w = latest_color_info.width
    target_h = latest_color_info.height

    resized = cv2.resize(raw, (target_w, target_h), interpolation=cv2.INTER_NEAREST)

    out = Image()
    out.header = latest_color_info.header
    out.height = target_h
    out.width = target_w
    out.encoding = msg.encoding if msg.encoding else "16UC1"
    out.is_bigendian = msg.is_bigendian
    out.step = target_w * 2
    out.data = resized.astype(np.uint16).tobytes()

    pub_img.publish(out)

    info = CameraInfo()
    info = latest_color_info
    info.header = out.header
    pub_info.publish(info)

def main():
    global pub_img, pub_info

    rospy.init_node("fake_aligned_depth_from_raw")

    pub_img = rospy.Publisher(
        "/table_camera/aligned_depth_to_color/image_raw",
        Image,
        queue_size=1
    )

    pub_info = rospy.Publisher(
        "/table_camera/aligned_depth_to_color/camera_info",
        CameraInfo,
        queue_size=1
    )

    rospy.Subscriber(
        "/table_camera/color/camera_info",
        CameraInfo,
        color_info_cb,
        queue_size=1
    )

    rospy.Subscriber(
        "/table_camera/depth/image_rect_raw",
        Image,
        depth_cb,
        queue_size=1
    )

    rospy.loginfo("Publishing fake aligned depth from raw depth.")
    rospy.spin()

if __name__ == "__main__":
    main()
