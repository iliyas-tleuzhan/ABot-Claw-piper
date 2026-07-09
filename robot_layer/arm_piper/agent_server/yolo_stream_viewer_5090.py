#!/usr/bin/env python3
"""Continuously send a ROS camera stream to the local 5090 YOLO service."""

import argparse
import base64
import json
import os
import subprocess
import sys
import tempfile
import threading
import time
from typing import Any, Dict, Optional

import cv2
import requests
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


WINDOW_NAME = "5090 YOLO detections"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="View YOLO detections produced by the 5090 HTTP service."
    )
    parser.add_argument(
        "--color-topic", default="/table_camera/color/image_raw"
    )
    parser.add_argument(
        "--server-url", default="http://127.0.0.1:8013/detect"
    )
    parser.add_argument("--rate", type=float, default=2.0, help="Maximum requests/second")
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--iou", type=float, default=0.45)
    parser.add_argument("--no-window", action="store_true")
    parser.add_argument("--save-path", default="/tmp/yolo_stream_latest.jpg")
    parser.add_argument("--json-path", default="/tmp/yolo_stream_latest.json")
    args = parser.parse_args(rospy.myargv()[1:])

    if args.rate <= 0:
        parser.error("--rate must be greater than zero")
    if not 0.0 <= args.conf <= 1.0:
        parser.error("--conf must be between 0 and 1")
    if not 0.0 <= args.iou <= 1.0:
        parser.error("--iou must be between 0 and 1")
    return args


class LatestFrame:
    def __init__(self) -> None:
        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._frame = None
        self._sequence = 0

    def callback(self, message: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(5.0, "Could not convert camera frame: %s", exc)
            return

        with self._lock:
            self._frame = frame
            self._sequence += 1

    def get(self):
        with self._lock:
            if self._frame is None:
                return None, self._sequence
            return self._frame.copy(), self._sequence


def atomic_write_bytes(path: str, data: bytes) -> None:
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    file_descriptor, temporary_path = tempfile.mkstemp(
        dir=directory, prefix=".yolo_stream_"
    )
    try:
        with os.fdopen(file_descriptor, "wb") as output:
            output.write(data)
        os.replace(temporary_path, path)
    except Exception:
        try:
            os.unlink(temporary_path)
        except OSError:
            pass
        raise


def draw_detections(frame, result: Dict[str, Any]):
    annotated = frame.copy()
    height, width = annotated.shape[:2]

    for detection in result.get("detections", []):
        x1 = max(0, min(width - 1, int(round(float(detection["x1"])))))
        y1 = max(0, min(height - 1, int(round(float(detection["y1"])))))
        x2 = max(0, min(width - 1, int(round(float(detection["x2"])))))
        y2 = max(0, min(height - 1, int(round(float(detection["y2"])))))
        class_name = str(detection.get("class_name", detection.get("class_id", "?")))
        confidence = float(detection.get("confidence", 0.0))
        label = "{} {:.2f}".format(class_name, confidence)

        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
        (text_width, text_height), baseline = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1
        )
        label_top = max(0, y1 - text_height - baseline - 4)
        cv2.rectangle(
            annotated,
            (x1, label_top),
            (min(width - 1, x1 + text_width + 4), y1),
            (0, 255, 0),
            -1,
        )
        cv2.putText(
            annotated,
            label,
            (x1 + 2, max(text_height, y1 - baseline - 2)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 0, 0),
            1,
            cv2.LINE_AA,
        )
    return annotated


def detection_summary(result: Dict[str, Any]) -> str:
    detections = result.get("detections", [])
    if not detections:
        return "no detections"
    return ", ".join(
        "{} {:.2f}".format(
            item.get("class_name", item.get("class_id", "?")),
            float(item.get("confidence", 0.0)),
        )
        for item in detections
    )


def disable_window(save_path: str, reason: Optional[Exception] = None) -> None:
    detail = " ({})".format(reason) if reason else ""
    rospy.logwarn(
        "OpenCV window unavailable%s; continuing in save-only mode. "
        "Copy or open %s to view the latest image.",
        detail,
        save_path,
    )
    try:
        cv2.destroyAllWindows()
    except cv2.error:
        pass


def display_is_usable() -> bool:
    if not os.environ.get("DISPLAY"):
        return False

    # Qt-backed OpenCV can abort, rather than raise cv2.error, on a bad Docker
    # display. Probe it in a child process so the stream process remains alive.
    probe = (
        "import cv2; "
        "cv2.namedWindow('opencv_display_probe'); "
        "cv2.destroyAllWindows()"
    )
    try:
        completed = subprocess.run(
            [sys.executable, "-c", probe],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=5.0,
            check=False,
        )
        return completed.returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False


def main() -> None:
    args = parse_args()
    rospy.init_node("yolo_stream_viewer_5090", anonymous=False)

    latest = LatestFrame()
    rospy.Subscriber(
        args.color_topic, Image, latest.callback, queue_size=1, buff_size=2**24
    )

    window_enabled = not args.no_window
    if window_enabled and not display_is_usable():
        disable_window(args.save_path)
        window_enabled = False
    elif window_enabled:
        try:
            cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        except cv2.error as exc:
            disable_window(args.save_path, exc)
            window_enabled = False

    rospy.loginfo(
        "Streaming %s to %s at %.2f Hz (conf=%.2f, iou=%.2f)",
        args.color_topic,
        args.server_url,
        args.rate,
        args.conf,
        args.iou,
    )
    session = requests.Session()
    loop_rate = rospy.Rate(args.rate)
    last_sequence = -1

    while not rospy.is_shutdown():
        frame, sequence = latest.get()
        if frame is None:
            rospy.loginfo_throttle(5.0, "Waiting for frames on %s", args.color_topic)
            loop_rate.sleep()
            continue
        if sequence == last_sequence:
            loop_rate.sleep()
            continue
        last_sequence = sequence

        try:
            encoded_ok, encoded = cv2.imencode(".jpg", frame)
            if not encoded_ok:
                raise RuntimeError("OpenCV JPEG encoding failed")
            payload = {
                "image": base64.b64encode(encoded.tobytes()).decode("ascii"),
                "conf_thres": args.conf,
                "iou_thres": args.iou,
            }
            started = time.monotonic()
            response = session.post(args.server_url, json=payload, timeout=30.0)
            response.raise_for_status()
            result = response.json()
            annotated = draw_detections(frame, result)

            saved_ok, saved_image = cv2.imencode(".jpg", annotated)
            if not saved_ok:
                raise RuntimeError("OpenCV annotated JPEG encoding failed")
            atomic_write_bytes(args.save_path, saved_image.tobytes())
            atomic_write_bytes(
                args.json_path,
                (json.dumps(result, indent=2, sort_keys=True) + "\n").encode("utf-8"),
            )

            elapsed_ms = (time.monotonic() - started) * 1000.0
            rospy.loginfo(
                "frame=%d count=%d inference_http=%.0fms: %s",
                sequence,
                len(result.get("detections", [])),
                elapsed_ms,
                detection_summary(result),
            )

            if window_enabled:
                try:
                    cv2.imshow(WINDOW_NAME, annotated)
                    if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                        rospy.signal_shutdown("viewer window closed by user")
                except cv2.error as exc:
                    disable_window(args.save_path, exc)
                    window_enabled = False
        except (
            requests.RequestException,
            cv2.error,
            OSError,
            ValueError,
            KeyError,
            TypeError,
            RuntimeError,
        ) as exc:
            rospy.logerr_throttle(5.0, "YOLO stream request failed: %s", exc)

        try:
            loop_rate.sleep()
        except rospy.ROSInterruptException:
            break

    session.close()
    if window_enabled:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
