#!/usr/bin/env python3
"""Generate a rough camera-to-base transform from a measured camera pose."""

from __future__ import annotations

import argparse
import math
import os

import numpy as np
import yaml


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def normalize(vector: np.ndarray, label: str) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm < 1e-12:
        raise ValueError(f"{label} is undefined; provide a different horizontal aim point")
    return vector / norm


def build_transform(
    camera_position: np.ndarray, aim_point: np.ndarray, tilt_deg: float, roll_deg: float
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    horizontal_heading = normalize(
        np.array([aim_point[0] - camera_position[0], aim_point[1] - camera_position[1], 0.0]),
        "camera horizontal heading",
    )
    theta = math.radians(tilt_deg)
    camera_z = math.sin(theta) * horizontal_heading + np.array([0.0, 0.0, -math.cos(theta)])
    camera_z = normalize(camera_z, "camera optical axis")

    world_up = np.array([0.0, 0.0, 1.0])
    camera_x_zero_roll = normalize(np.cross(camera_z, world_up), "camera X axis")
    camera_y_zero_roll = np.cross(camera_z, camera_x_zero_roll)

    roll = math.radians(roll_deg)
    camera_x = math.cos(roll) * camera_x_zero_roll + math.sin(roll) * camera_y_zero_roll
    camera_y = -math.sin(roll) * camera_x_zero_roll + math.cos(roll) * camera_y_zero_roll

    rotation = np.column_stack((camera_x, camera_y, camera_z))
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = camera_position
    return transform, camera_x, camera_y, camera_z


def validate_rotation(rotation: np.ndarray) -> tuple[float, float]:
    if not np.all(np.isfinite(rotation)):
        raise ValueError("Generated rotation contains NaN or infinite values")
    orthonormality_error = float(np.linalg.norm(rotation.T @ rotation - np.eye(3), ord=np.inf))
    determinant = float(np.linalg.det(rotation))
    if orthonormality_error > 1e-9:
        raise ValueError(f"Generated rotation is not orthonormal (error={orthonormality_error})")
    if not math.isclose(determinant, 1.0, abs_tol=1e-9):
        raise ValueError(f"Generated rotation determinant is not +1 (det={determinant})")
    return determinant, orthonormality_error


def matrix_rows(transform: np.ndarray) -> list[list[float]]:
    return [[float(f"{value:.9f}") for value in row] for row in transform.tolist()]


def format_matrix_yaml(transform: np.ndarray) -> str:
    rows = [", ".join(f"{value:.9f}" for value in row) for row in transform]
    return "camera_to_base:\n" + "\n".join(f"  - [{row}]" for row in rows) + "\n"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--xc", type=float, required=True, help="Camera X in Piper base coordinates, meters")
    parser.add_argument("--yc", type=float, required=True, help="Camera Y in Piper base coordinates, meters")
    parser.add_argument("--zc", type=float, required=True, help="Camera Z in Piper base coordinates, meters")
    parser.add_argument("--tilt-deg", type=float, required=True, help="Tilt from straight-down, degrees")
    parser.add_argument("--aim-x", type=float, default=0.0)
    parser.add_argument("--aim-y", type=float, default=0.0)
    parser.add_argument("--aim-z", type=float, default=0.0)
    parser.add_argument("--roll-deg", type=float, default=0.0)
    parser.add_argument("--output", default=os.path.join(SCRIPT_DIR, "calibration_1.yaml"))
    parser.add_argument("--print-matrix", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    camera_position = np.array([args.xc, args.yc, args.zc], dtype=np.float64)
    aim_point = np.array([args.aim_x, args.aim_y, args.aim_z], dtype=np.float64)
    if not np.all(np.isfinite(camera_position)) or not np.all(np.isfinite(aim_point)):
        raise SystemExit("Camera position and aim point must contain only finite values")
    if args.xc == 0.0 and args.yc == 0.0 and args.aim_x == 0.0 and args.aim_y == 0.0:
        raise SystemExit("xc and yc cannot both be zero when aiming at the base origin")

    try:
        transform, camera_x, camera_y, camera_z = build_transform(
            camera_position, aim_point, args.tilt_deg, args.roll_deg
        )
        determinant, orthonormality_error = validate_rotation(transform[:3, :3])
    except ValueError as exc:
        raise SystemExit(f"ERROR: {exc}") from exc

    calibration_name = os.path.splitext(os.path.basename(args.output))[0]
    payload = {
        "calibration": {
            "name": calibration_name,
            "method": "rough_manual_camera_pose",
            "camera_position_base_m": {"x": args.xc, "y": args.yc, "z": args.zc},
            "aim_point_base_m": {"x": args.aim_x, "y": args.aim_y, "z": args.aim_z},
            "tilt_from_down_deg": args.tilt_deg,
            "roll_deg": args.roll_deg,
            "assumptions": [
                "camera horizontal heading points toward the configured aim point",
                "zero camera roll unless --roll-deg is provided",
                "measurements are rough ruler and phone-angle measurements",
            ],
            "warnings": [
                "use detect-only first",
                "validate coordinate directions before hover or motion",
                "do not use for precise grasping without further verification",
            ],
        },
    }
    with open(args.output, "w", encoding="utf-8") as output_file:
        output_file.write(format_matrix_yaml(transform))
        yaml.safe_dump(payload, output_file, sort_keys=False)

    if args.print_matrix:
        print(f"Camera position (base m): {camera_position.tolist()}")
        print(f"Aim point (base m): {aim_point.tolist()}")
        print(f"Tilt from down (deg): {args.tilt_deg}")
        print(f"Roll (deg): {args.roll_deg}")
        print(f"Camera X axis in base: {camera_x.tolist()}")
        print(f"Camera Y axis in base: {camera_y.tolist()}")
        print(f"Camera Z axis in base: {camera_z.tolist()}")
        print("T_camera_to_base:")
        for row in matrix_rows(transform):
            print(f"  {row}")
        print(f"Determinant: {determinant:.12f}")
        print(f"Orthonormality error: {orthonormality_error:.12e}")
    print(f"Wrote rough manual calibration: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
