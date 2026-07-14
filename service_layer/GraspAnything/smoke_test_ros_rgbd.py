#!/usr/bin/env python3
"""Read-only ROS RGB-D smoke test for /grasp/detect.

Captures one color image, one aligned depth image, and one CameraInfo message,
sends them to the GraspAnything-compatible HTTP service, and prints returned
camera-frame grasp candidates. It does not command robot motion.
"""

from __future__ import annotations

import argparse
import base64
import json
from typing import Any

import cv2
import numpy as np
import requests
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image


def _encode_png(arr: np.ndarray) -> str:
    ok, buf = cv2.imencode(".png", arr)
    if not ok:
        raise RuntimeError("cv2.imencode(.png) failed")
    return base64.b64encode(buf).decode("utf-8")


def _depth_to_u16(depth: np.ndarray) -> np.ndarray:
    arr = np.asarray(depth)
    if arr.dtype == np.uint16:
        return arr
    if np.issubdtype(arr.dtype, np.floating):
        arr = np.where(np.isfinite(arr), arr, 0.0)
        arr = arr * 1000.0
    return np.clip(arr.astype(np.float32), 0, 65535).astype(np.uint16)


def main() -> None:
    parser = argparse.ArgumentParser(description="Read-only RGB-D grasp fallback smoke test")
    parser.add_argument("--url", default="http://127.0.0.1:8015/grasp/detect")
    parser.add_argument("--object-name", required=True)
    parser.add_argument("--top-k", type=int, default=3)
    parser.add_argument("--color-topic", default="/table_camera/color/image_raw")
    parser.add_argument("--depth-topic", default="/table_camera/aligned_depth_to_color/image_raw")
    parser.add_argument("--camera-info-topic", default="/table_camera/color/camera_info")
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()

    rospy.init_node("grasp_depth_fallback_smoke_test", anonymous=True, disable_signals=True)
    bridge = CvBridge()

    color_msg = rospy.wait_for_message(args.color_topic, Image, timeout=args.timeout)
    depth_msg = rospy.wait_for_message(args.depth_topic, Image, timeout=args.timeout)
    info_msg = rospy.wait_for_message(args.camera_info_topic, CameraInfo, timeout=args.timeout)

    color_bgr = bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
    depth = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
    depth_u16 = _depth_to_u16(depth)
    k = np.array(info_msg.K, dtype=float).reshape(3, 3).tolist()

    payload: dict[str, Any] = {
        "color_image": _encode_png(color_bgr),
        "depth_image": _encode_png(depth_u16),
        "camera_intrinsics": k,
        "object_name": args.object_name,
        "top_k": args.top_k,
    }
    response = requests.post(args.url, json=payload, timeout=max(30.0, args.timeout))
    response.raise_for_status()
    data = response.json()

    print(json.dumps({
        "backend": data.get("backend"),
        "frame_id": info_msg.header.frame_id,
        "target": data.get("target"),
        "count": data.get("count"),
        "latency_ms": data.get("latency_ms"),
    }, indent=2))
    for result in data.get("results", []):
        print(f"result label={result.get('label')} confidence={result.get('confidence')} xyxy={result.get('xyxy')}")
        for i, grasp in enumerate(result.get("grasps", [])):
            print(json.dumps({
                "index": i,
                "score": grasp.get("score"),
                "width": grasp.get("width"),
                "translation_camera": grasp.get("translation_camera"),
                "translation_camera_retreat": grasp.get("translation_camera_retreat"),
                "quaternion_camera_xyzw": grasp.get("quaternion_camera_xyzw"),
            }, indent=2))


if __name__ == "__main__":
    main()
