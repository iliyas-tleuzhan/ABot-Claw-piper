#!/usr/bin/env python3
"""Detect colored targets and validate a camera-to-base calibration without motion."""

from __future__ import annotations

import argparse

import numpy as np
import rospy

from today_red_to_purple_pick_place import (
    DEFAULT_CALIBRATION,
    DEFAULT_COLOR_TOPIC,
    DEFAULT_DEPTH_TOPIC,
    DEFAULT_INFO_TOPIC,
    RgbdReader,
    detect_color_object,
    load_camera_to_base,
)


SANITY_MIN = np.array([-0.20, -0.60, -0.20])
SANITY_MAX = np.array([0.80, 0.60, 0.80])


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--calibration", default=DEFAULT_CALIBRATION)
    parser.add_argument("--color-topic", default=DEFAULT_COLOR_TOPIC)
    parser.add_argument("--depth-topic", default=DEFAULT_DEPTH_TOPIC)
    parser.add_argument("--camera-info-topic", default=DEFAULT_INFO_TOPIC)
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--min-red-area", type=float, default=300.0)
    parser.add_argument("--min-purple-area", type=float, default=600.0)
    parser.add_argument("--watch", action="store_true", help="Print updated target coordinates at about 2 Hz")
    return parser.parse_args()


def print_detection(name: str, detection) -> None:
    base_xyz = detection.base_xyz_m
    assert base_xyz is not None
    print(f"{name}:")
    print(f"  pixel: {list(detection.pixel)}")
    print(f"  depth_m: {detection.depth_m:.4f}")
    print(f"  camera_xyz_m: {detection.camera_xyz_m.tolist()}")
    print(f"  base_xyz_m: {base_xyz.tolist()}")
    if np.any(base_xyz < SANITY_MIN) or np.any(base_xyz > SANITY_MAX):
        print("  !!! WARNING: BASE COORDINATE OUTSIDE BROAD SANITY BOUNDS !!!")
        print("  warning_bounds: X [-0.20, 0.80], Y [-0.60, 0.60], Z [-0.20, 0.80]")


def print_current_detections(args: argparse.Namespace, reader: RgbdReader, transform: np.ndarray) -> None:
    assert reader.color is not None and reader.depth is not None and reader.K is not None
    for name, min_area in (("red", args.min_red_area), ("purple", args.min_purple_area)):
        try:
            detection = detect_color_object(name, reader.color, reader.depth, reader.K, transform, min_area)
            print_detection(name, detection)
        except RuntimeError as exc:
            print(f"{name}: not detected ({exc})")


def main() -> int:
    args = parse_args()
    print("=" * 72)
    print("NO ROBOT MOTION IS PERFORMED BY THIS SCRIPT")
    print("=" * 72)
    transform = load_camera_to_base(args.calibration)
    if not rospy.get_node_uri():
        rospy.init_node("validate_manual_calibration", anonymous=True, disable_signals=True)

    reader = RgbdReader(args.color_topic, args.depth_topic, args.camera_info_topic, args.timeout)
    reader.start()
    print_current_detections(args, reader, transform)
    if not args.watch:
        return 0

    print("Watching at about 2 Hz. Press Ctrl-C to stop.")
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        print("-" * 72)
        print_current_detections(args, reader, transform)
        rate.sleep()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
