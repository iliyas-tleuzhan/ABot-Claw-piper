#!/usr/bin/env python3
"""Create camera_to_base.yaml from colored target samples.

For each sample, place a red target at a known base-frame coordinate and pass
that coordinate with --sample. The script observes the target through the D555,
records its camera-frame 3D point, fits a rigid camera->base transform, and
writes camera_to_base.yaml.
"""

from __future__ import annotations

import argparse
import os
import time
from typing import List, Tuple

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


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Fit camera_to_base.yaml from red target detections and known base coordinates."
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
    parser.add_argument("--min-red-area", type=float, default=300.0)
    args = parser.parse_args()

    if len(args.sample) < 3:
        raise SystemExit("Need at least 3 --sample points")

    if not rospy.get_node_uri():
        rospy.init_node("calibrate_camera_to_base_points", anonymous=True, disable_signals=True)

    camera_points = []
    base_points = []
    observed = []
    for index, (name, base_xyz) in enumerate(args.sample, start=1):
        input(
            f"Place red target at {name} base={base_xyz.tolist()} "
            f"({index}/{len(args.sample)}), then press Enter..."
        )
        reader = RgbdReader(args.color_topic, args.depth_topic, args.camera_info_topic, args.timeout)
        reader.start()
        assert reader.color is not None and reader.depth is not None and reader.K is not None
        det = detect_color_object(
            "red",
            reader.color,
            reader.depth,
            reader.K,
            None,
            args.min_red_area,
        )
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
            }
        )
        print(f"  camera={det.camera_xyz_m.tolist()} pixel={det.pixel} area={det.area_px:.1f}")

    camera_arr = np.vstack(camera_points)
    base_arr = np.vstack(base_points)
    T = fit_rigid_transform(camera_arr, base_arr)
    residuals = (T[:3, :3] @ camera_arr.T).T + T[:3, 3] - base_arr
    errors = np.linalg.norm(residuals, axis=1)

    payload = {
        "camera_to_base": format_matrix(T),
        "calibration": {
            "method": "red_target_point_correspondences",
            "samples": observed,
            "residuals_m": [float(v) for v in errors.tolist()],
            "rms_error_m": float(np.sqrt(np.mean(errors ** 2))),
            "max_error_m": float(np.max(errors)),
        },
    }
    with open(args.output, "w", encoding="utf-8") as f:
        yaml.safe_dump(payload, f, sort_keys=False)

    print(f"Wrote {args.output}")
    print(f"RMS error: {payload['calibration']['rms_error_m']:.4f} m")
    print(f"Max error: {payload['calibration']['max_error_m']:.4f} m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
