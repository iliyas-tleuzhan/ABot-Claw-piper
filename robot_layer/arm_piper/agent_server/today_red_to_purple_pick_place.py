#!/usr/bin/env python3
"""Today-only red-object to purple-file Piper pick/place demo.

This deliberately uses simple HSV color segmentation and a local
camera_to_base.yaml calibration. It does not use YOLO, GraspAnything,
ABot /code/execute, old port 8890, or /end_pose topic control.
"""

from __future__ import annotations

import argparse
import math
import os
import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import cv2
import numpy as np
import rospy
import yaml
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf.transformations import quaternion_from_euler, quaternion_matrix

from robot_sdk.piper_sdk import PiperRobotEnv


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CALIBRATION = os.path.join(SCRIPT_DIR, "camera_to_base.yaml")
DEFAULT_COLOR_TOPIC = "/table_camera/color/image_raw"
DEFAULT_DEPTH_TOPIC = "/table_camera/aligned_depth_to_color/image_raw"
DEFAULT_INFO_TOPIC = "/table_camera/color/camera_info"


@dataclass
class Detection:
    name: str
    pixel: Tuple[int, int]
    area_px: float
    depth_m: float
    camera_xyz_m: np.ndarray
    base_xyz_m: Optional[np.ndarray]


class RgbdReader:
    def __init__(self, color_topic: str, depth_topic: str, info_topic: str, timeout: float):
        self.bridge = CvBridge()
        self.color_topic = color_topic
        self.depth_topic = depth_topic
        self.info_topic = info_topic
        self.timeout = timeout
        self.color: Optional[np.ndarray] = None
        self.depth: Optional[np.ndarray] = None
        self.K: Optional[np.ndarray] = None

    def start(self) -> None:
        rospy.Subscriber(self.color_topic, Image, self._on_color, queue_size=1)
        rospy.Subscriber(self.depth_topic, Image, self._on_depth, queue_size=1)
        rospy.Subscriber(self.info_topic, CameraInfo, self._on_info, queue_size=1)
        deadline = time.time() + self.timeout
        while time.time() < deadline and not rospy.is_shutdown():
            if self.color is not None and self.depth is not None and self.K is not None:
                return
            time.sleep(0.05)
        raise RuntimeError(
            "Timed out waiting for RealSense topics: "
            f"{self.color_topic}, {self.depth_topic}, {self.info_topic}"
        )

    def _on_color(self, msg: Image) -> None:
        self.color = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def _on_depth(self, msg: Image) -> None:
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        arr = np.asarray(depth)
        if arr.dtype == np.uint16:
            self.depth = arr.astype(np.float32) / 1000.0
        else:
            self.depth = arr.astype(np.float32)

    def _on_info(self, msg: CameraInfo) -> None:
        self.K = np.asarray(msg.K, dtype=np.float64).reshape(3, 3)


def load_camera_to_base(path: str) -> np.ndarray:
    if not os.path.exists(path):
        raise FileNotFoundError(
            f"Missing required calibration: {path}\n"
            "Create this from a real camera-to-base calibration before using --execute."
        )
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    if "camera_to_base" in data:
        T = np.asarray(data["camera_to_base"], dtype=np.float64)
    elif "matrix" in data:
        T = np.asarray(data["matrix"], dtype=np.float64)
    elif "translation" in data and "quaternion_xyzw" in data:
        T = quaternion_matrix(data["quaternion_xyzw"])
        T[:3, 3] = np.asarray(data["translation"], dtype=np.float64)
    else:
        raise ValueError(
            "Calibration must contain camera_to_base: [[4x4]], matrix: [[4x4]], "
            "or translation + quaternion_xyzw."
        )
    T = T.reshape(4, 4)
    if not np.all(np.isfinite(T)):
        raise ValueError("Calibration contains non-finite values")
    return T


def make_mask(hsv: np.ndarray, name: str) -> np.ndarray:
    if name == "red":
        m1 = cv2.inRange(hsv, np.array([0, 80, 50]), np.array([12, 255, 255]))
        m2 = cv2.inRange(hsv, np.array([170, 80, 50]), np.array([179, 255, 255]))
        mask = cv2.bitwise_or(m1, m2)
    elif name == "purple":
        mask = cv2.inRange(hsv, np.array([125, 45, 35]), np.array([165, 255, 255]))
    elif name == "blue":
        mask = cv2.inRange(hsv, np.array([90, 70, 40]), np.array([130, 255, 255]))
    else:
        raise ValueError(name)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def robust_depth(depth_m: np.ndarray, u: int, v: int, radius: int = 5) -> float:
    h, w = depth_m.shape[:2]
    x1, x2 = max(0, u - radius), min(w, u + radius + 1)
    y1, y2 = max(0, v - radius), min(h, v + radius + 1)
    patch = depth_m[y1:y2, x1:x2]
    valid = patch[np.isfinite(patch) & (patch > 0.05) & (patch < 2.0)]
    if valid.size == 0:
        raise RuntimeError(f"No valid depth near pixel ({u}, {v})")
    return float(np.median(valid))


def pixel_to_camera(K: np.ndarray, u: int, v: int, z: float) -> np.ndarray:
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    return np.array([(u - cx) * z / fx, (v - cy) * z / fy, z], dtype=np.float64)


def transform_point(T: np.ndarray, p: np.ndarray) -> np.ndarray:
    ph = np.ones(4, dtype=np.float64)
    ph[:3] = p
    return (T @ ph)[:3]


def detect_color_object(
    name: str,
    color_bgr: np.ndarray,
    depth_m: np.ndarray,
    K: np.ndarray,
    T_camera_to_base: Optional[np.ndarray],
    min_area: float,
) -> Detection:
    hsv = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2HSV)
    mask = make_mask(hsv, name)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        raise RuntimeError(f"No {name} object found")
    contour = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(contour))
    if area < min_area:
        raise RuntimeError(f"{name} object area too small: {area:.1f}px")
    moments = cv2.moments(contour)
    if moments["m00"] == 0:
        raise RuntimeError(f"{name} contour has zero moment")
    u = int(moments["m10"] / moments["m00"])
    v = int(moments["m01"] / moments["m00"])
    z = robust_depth(depth_m, u, v)
    p_cam = pixel_to_camera(K, u, v, z)
    p_base = None if T_camera_to_base is None else transform_point(T_camera_to_base, p_cam)
    return Detection(name, (u, v), area, z, p_cam, p_base)


def save_debug_image(path: str, color_bgr: np.ndarray, detections: Dict[str, Detection]) -> None:
    img = color_bgr.copy()
    colors = {"red": (0, 0, 255), "purple": (255, 0, 255)}
    for det in detections.values():
        cv2.circle(img, det.pixel, 8, colors.get(det.name, (255, 255, 255)), 2)
        cv2.putText(img, det.name, (det.pixel[0] + 10, det.pixel[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, colors.get(det.name, (255, 255, 255)), 2)
    cv2.imwrite(path, img)


def move_pick_place(args: argparse.Namespace, red: Detection, purple: Detection) -> None:
    if red.base_xyz_m is None or purple.base_xyz_m is None:
        raise RuntimeError("Refusing to move without calibrated base coordinates")
    robot = PiperRobotEnv(init_ros_node=False)
    q = quaternion_from_euler(math.pi, 0.0, 0.0)
    orientation = [float(q[0]), float(q[1]), float(q[2]), float(q[3])]

    pick = red.base_xyz_m.copy()
    place = purple.base_xyz_m.copy()
    pick[2] += args.pick_z_offset
    place[2] += args.place_z_offset

    pre_pick = pick.copy()
    pre_pick[2] += args.approach_height
    pre_place = place.copy()
    pre_place[2] += args.approach_height

    speed = args.speed
    accel = args.accel
    print("Moving to pre-pick", pre_pick.tolist())
    robot.move_to_pose(pre_pick.tolist() + orientation, max_velocity=speed, max_acceleration=accel)
    print("Moving to pick", pick.tolist())
    robot.move_to_pose(pick.tolist() + orientation, max_velocity=speed, max_acceleration=accel)
    print("Lifting")
    robot.move_to_pose(pre_pick.tolist() + orientation, max_velocity=speed, max_acceleration=accel)
    print("Moving to pre-place", pre_place.tolist())
    robot.move_to_pose(pre_place.tolist() + orientation, max_velocity=speed, max_acceleration=accel)
    print("Moving to place", place.tolist())
    robot.move_to_pose(place.tolist() + orientation, max_velocity=speed, max_acceleration=accel)
    print("Retreating")
    robot.move_to_pose(pre_place.tolist() + orientation, max_velocity=speed, max_acceleration=accel)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Detect red object and purple file; optionally execute pick/place.")
    p.add_argument("--execute", action="store_true", help="Actually move the robot. Omitted means detect-only.")
    p.add_argument("--calibration", default=DEFAULT_CALIBRATION)
    p.add_argument("--color-topic", default=DEFAULT_COLOR_TOPIC)
    p.add_argument("--depth-topic", default=DEFAULT_DEPTH_TOPIC)
    p.add_argument("--camera-info-topic", default=DEFAULT_INFO_TOPIC)
    p.add_argument("--timeout", type=float, default=15.0)
    p.add_argument("--min-red-area", type=float, default=300.0)
    p.add_argument("--min-purple-area", type=float, default=600.0)
    p.add_argument("--debug-image", default=os.path.join(SCRIPT_DIR, "today_pick_place_debug.jpg"))
    p.add_argument("--approach-height", type=float, default=0.08)
    p.add_argument("--pick-z-offset", type=float, default=0.02)
    p.add_argument("--place-z-offset", type=float, default=0.04)
    p.add_argument("--speed", type=float, default=0.05)
    p.add_argument("--accel", type=float, default=0.05)
    return p.parse_args()


def main() -> int:
    args = parse_args()
    if not rospy.get_node_uri():
        rospy.init_node("today_red_to_purple_pick_place", anonymous=True, disable_signals=True)

    if args.execute:
        T = load_camera_to_base(args.calibration)
    elif os.path.exists(args.calibration):
        T = load_camera_to_base(args.calibration)
    else:
        T = None
        print(f"Detect-only mode: no calibration loaded ({args.calibration} missing)")
    reader = RgbdReader(args.color_topic, args.depth_topic, args.camera_info_topic, args.timeout)
    reader.start()
    assert reader.color is not None and reader.depth is not None and reader.K is not None

    red = detect_color_object("red", reader.color, reader.depth, reader.K, T, args.min_red_area)
    purple = detect_color_object("purple", reader.color, reader.depth, reader.K, T, args.min_purple_area)
    detections = {"red": red, "purple": purple}
    save_debug_image(args.debug_image, reader.color, detections)

    for det in detections.values():
        base = None if det.base_xyz_m is None else det.base_xyz_m.tolist()
        print(
            f"{det.name}: pixel={det.pixel} area={det.area_px:.1f} "
            f"depth={det.depth_m:.3f} camera={det.camera_xyz_m.tolist()} "
            f"base={base}"
        )
    print(f"Wrote debug image: {args.debug_image}")

    if not args.execute:
        print("Detect-only mode. Re-run with --execute after checking calibration and debug image.")
        return 0

    move_pick_place(args, red, purple)
    print("Pick/place sequence complete")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
