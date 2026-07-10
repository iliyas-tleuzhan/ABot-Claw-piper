#!/usr/bin/env python3
"""Create camera_to_base.yaml from colored target samples.

For each sample, place a colored target at a known base-frame coordinate and pass
that coordinate with --sample. The script observes the target through the D555,
records its camera-frame 3D point, fits a rigid camera->base transform, and
writes camera_to_base.yaml.
"""

from __future__ import annotations

import argparse
import os
import time
from typing import List, Tuple

import cv2
import numpy as np
import rospy
import yaml

from today_red_to_purple_pick_place import (
    DEFAULT_COLOR_TOPIC,
    DEFAULT_DEPTH_TOPIC,
    DEFAULT_INFO_TOPIC,
    RgbdReader,
    detect_color_object,
)


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_OUTPUT = os.path.join(SCRIPT_DIR, "camera_to_base.yaml")


def parse_sample(value: str) -> Tuple[str, np.ndarray]:
    parts = value.split(",")
    if len(parts) == 3:
        name = f"sample_{time.time_ns()}"
        xyz = parts
    elif len(parts) == 4:
        name = parts[0]
        xyz = parts[1:]
    else:
        raise argparse.ArgumentTypeError(
            "sample must be X,Y,Z or NAME,X,Y,Z, values in meters in base_link"
        )
    try:
        return name, np.asarray([float(v) for v in xyz], dtype=np.float64)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc


def fit_rigid_transform(camera_points: np.ndarray, base_points: np.ndarray) -> np.ndarray:
    if camera_points.shape != base_points.shape or camera_points.shape[1] != 3:
        raise ValueError("camera/base point arrays must both be Nx3")
    if camera_points.shape[0] < 3:
        raise ValueError("need at least 3 non-collinear samples")

    camera_centroid = camera_points.mean(axis=0)
    base_centroid = base_points.mean(axis=0)
    camera_centered = camera_points - camera_centroid
    base_centered = base_points - base_centroid

    if np.linalg.matrix_rank(camera_centered) < 2 or np.linalg.matrix_rank(base_centered) < 2:
        raise ValueError("samples are degenerate; use non-collinear table points")

    H = camera_centered.T @ base_centered
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1.0
        R = Vt.T @ U.T
    t = base_centroid - R @ camera_centroid

    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def format_matrix(T: np.ndarray) -> List[List[float]]:
    return [[float(f"{value:.9f}") for value in row] for row in T.tolist()]


def save_sample_debug_image(
    path: str, color_bgr: np.ndarray, pixel: Tuple[int, int], sample_name: str, target_color: str
) -> None:
    image = color_bgr.copy()
    colors = {"red": (0, 0, 255), "blue": (255, 0, 0), "purple": (255, 0, 255)}
    color = colors[target_color]
    cv2.drawMarker(image, pixel, color, cv2.MARKER_CROSS, 20, 2)
    cv2.circle(image, pixel, 8, color, 2)
    cv2.putText(
        image,
        f"{sample_name} ({target_color})",
        (pixel[0] + 12, max(24, pixel[1] - 12)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        color,
        2,
        cv2.LINE_AA,
    )
    if not cv2.imwrite(path, image):
        raise RuntimeError(f"Failed to write debug image: {path}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Fit camera_to_base.yaml from colored target detections and known base coordinates."
    )
    parser.add_argument(
        "--sample",
        action="append",
        type=parse_sample,
        required=True,
        help="Known base point as X,Y,Z or NAME,X,Y,Z in meters. Repeat at least 3 times.",
    )
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    parser.add_argument("--color-topic", default=DEFAULT_COLOR_TOPIC)
    parser.add_argument("--depth-topic", default=DEFAULT_DEPTH_TOPIC)
    parser.add_argument("--camera-info-topic", default=DEFAULT_INFO_TOPIC)
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--target-color", choices=("red", "blue", "purple"), default="red")
    parser.add_argument("--min-target-area", dest="min_target_area", type=float, default=300.0)
    parser.add_argument("--min-red-area", dest="min_target_area", type=float, help=argparse.SUPPRESS)
    args = parser.parse_args()

    if len(args.sample) < 3:
        raise SystemExit("Need at least 3 --sample points")

    if not rospy.get_node_uri():
        rospy.init_node("calibrate_camera_to_base_points", anonymous=True, disable_signals=True)

    camera_points = []
    base_points = []
    observed = []
    debug_dir = os.path.dirname(os.path.abspath(args.output))
    for index, (name, base_xyz) in enumerate(args.sample, start=1):
        input(
            f"Place {args.target_color} target at {name} base={base_xyz.tolist()} "
            f"({index}/{len(args.sample)}), then press Enter..."
        )
        reader = RgbdReader(args.color_topic, args.depth_topic, args.camera_info_topic, args.timeout)
        reader.start()
        assert reader.color is not None and reader.depth is not None and reader.K is not None
        det = detect_color_object(
            args.target_color,
            reader.color,
            reader.depth,
            reader.K,
            None,
            args.min_target_area,
        )
        debug_path = os.path.join(debug_dir, f"calibration_debug_{name}.jpg")
        save_sample_debug_image(debug_path, reader.color, det.pixel, name, args.target_color)
        camera_points.append(det.camera_xyz_m)
        base_points.append(base_xyz)
        observed.append(
            {
                "name": name,
                "pixel": [int(det.pixel[0]), int(det.pixel[1])],
                "area_px": float(det.area_px),
                "depth_m": float(det.depth_m),
                "camera_xyz_m": [float(v) for v in det.camera_xyz_m.tolist()],
                "base_xyz_m": [float(v) for v in base_xyz.tolist()],
                "debug_image": debug_path,
            }
        )
        print(
            f"  camera={det.camera_xyz_m.tolist()} pixel={det.pixel} "
            f"area={det.area_px:.1f} debug={debug_path}"
        )

    camera_arr = np.vstack(camera_points)
    base_arr = np.vstack(base_points)
    T = fit_rigid_transform(camera_arr, base_arr)
    residuals = (T[:3, :3] @ camera_arr.T).T + T[:3, 3] - base_arr
    errors = np.linalg.norm(residuals, axis=1)

    payload = {
        "camera_to_base": format_matrix(T),
        "calibration": {
            "method": "target_color_point_correspondences",
            "target_color": args.target_color,
            "samples": observed,
            "residuals_m": [float(v) for v in errors.tolist()],
            "rms_error_m": float(np.sqrt(np.mean(errors ** 2))),
            "max_error_m": float(np.max(errors)),
        },
    }
    with open(args.output, "w", encoding="utf-8") as f:
        yaml.safe_dump(payload, f, sort_keys=False)

    print("Calibration summary")
    print(f"Output path: {os.path.abspath(args.output)}")
    print(f"RMS error: {payload['calibration']['rms_error_m']:.4f} m")
    print(f"Max error: {payload['calibration']['max_error_m']:.4f} m")
    for sample, error in zip(observed, errors):
        print(
            f"  {sample['name']}: pixel={sample['pixel']} depth={sample['depth_m']:.4f} m "
            f"camera_xyz={sample['camera_xyz_m']} base_xyz={sample['base_xyz_m']} "
            f"residual={error:.6f} m"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
