#!/usr/bin/env python3
"""Run reusable Piper manipulation tasks through the Agent Server lease path."""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from typing import Any, Dict, Optional
from urllib import error, request

import yaml


TERMINAL_STATUSES = {"completed", "failed", "timeout", "stopped", "idle"}
VALIDATED_REGION_SCHEMA_VERSION = 1
HEARTBEAT_INTERVAL_S = 20.0


def compute_gripper_plan_values(
    detected_width: float,
    gripper_min: float,
    gripper_max: float,
    close_width_override: Optional[float],
    execute: bool,
) -> Dict[str, Any]:
    detected = float(detected_width)
    gripper_min = float(gripper_min)
    gripper_max = float(gripper_max)
    open_width = gripper_max
    saturated = detected >= gripper_max - 0.001

    if close_width_override is None:
        source = "automatic"
        close_width = None if saturated else max(gripper_min, detected - 0.005)
    else:
        source = "explicit"
        close_width = float(close_width_override)

    if close_width is not None:
        if not (gripper_min <= close_width <= gripper_max):
            raise ValueError(
                "close_width %.4f outside gripper range %.4f..%.4f"
                % (close_width, gripper_min, gripper_max)
            )
        if not close_width < open_width:
            raise ValueError(
                "close_width %.4f must be smaller than open_width %.4f"
                % (close_width, open_width)
            )

    if execute and saturated and close_width_override is None:
        raise ValueError(
            "Detected grasp width %.4f m is saturated at gripper max %.4f m; "
            "execute mode requires explicit --close-width"
            % (detected, gripper_max)
        )

    return {
        "detected_width_m": detected,
        "estimate_saturated": saturated,
        "open_width_m": open_width,
        "close_width_m": close_width,
        "close_width_source": source,
        "grip_margin_m": None if close_width is None else detected - close_width,
    }


def http_json(
    method: str,
    url: str,
    payload: Optional[Dict[str, Any]] = None,
    headers: Optional[Dict[str, str]] = None,
):
    data = None if payload is None else json.dumps(payload).encode("utf-8")
    req_headers = {"Content-Type": "application/json"}
    if headers:
        req_headers.update(headers)
    req = request.Request(url, data=data, headers=req_headers, method=method)
    try:
        with request.urlopen(req, timeout=10) as resp:
            body = resp.read().decode("utf-8")
    except error.HTTPError as exc:
        detail = exc.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"{method} {url} failed: HTTP {exc.code}: {detail}") from exc
    except error.URLError as exc:
        raise RuntimeError(f"{method} {url} failed: {exc}") from exc
    return json.loads(body) if body else {}


def _require_vec(name: str, value: Any, length: int) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f"{name} must be a {length}-element list")
    out = [float(v) for v in value]
    return out


def load_validated_region_config(path: str, enabled: bool) -> Dict[str, Any]:
    if not enabled:
        return {
            "schema_version": VALIDATED_REGION_SCHEMA_VERSION,
            "source_path": path,
            "embedded": False,
            "regions": [],
            "region_count": 0,
            "voxel_count": 0,
            "status": "disabled",
        }
    try:
        with open(path, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
    except Exception as exc:
        raise RuntimeError(
            "VALIDATED_GRASP_REGION_CONFIG_ERROR_JSON "
            + json.dumps(
                {
                    "source_path": path,
                    "embedded": False,
                    "error": str(exc),
                    "region_count": 0,
                    "voxel_count": 0,
                },
                sort_keys=True,
            )
        ) from exc
    if not isinstance(data, dict):
        raise RuntimeError("Validated grasp region config must be a mapping: " + path)
    schema_version = int(data.get("schema_version", VALIDATED_REGION_SCHEMA_VERSION))
    if schema_version != VALIDATED_REGION_SCHEMA_VERSION:
        raise RuntimeError(
            "Unsupported validated grasp region schema_version %s in %s"
            % (schema_version, path)
        )
    regions = data.get("regions")
    if not isinstance(regions, list) or not regions:
        raise RuntimeError("Validated grasp region config has no regions: " + path)
    voxel_count = 0
    for r_index, region in enumerate(regions):
        if not isinstance(region, dict):
            raise RuntimeError(f"Region {r_index} is not a mapping")
        if not region.get("name"):
            raise RuntimeError(f"Region {r_index} missing name")
        voxels = region.get("validated_voxels")
        if not isinstance(voxels, list):
            raise RuntimeError(f"Region {region.get('name')} missing validated_voxels list")
        for v_index, voxel in enumerate(voxels):
            prefix = f"Region {region.get('name')} voxel {v_index}"
            if not isinstance(voxel, dict):
                raise RuntimeError(prefix + " is not a mapping")
            for key in (
                "voxel_id",
                "usage",
                "source_surface_xyz",
                "source_surface_bounds",
                "tcp_hover_xyz",
                "tcp_quaternion",
                "preferred_ik_seed",
                "actual_ik_solution",
                "validated_hover_height_m",
                "descend_fraction",
                "lift_fraction",
            ):
                if key not in voxel:
                    raise RuntimeError(prefix + " missing " + key)
            bounds = voxel["source_surface_bounds"]
            if not isinstance(bounds, dict):
                raise RuntimeError(prefix + " source_surface_bounds must be a mapping")
            _require_vec(prefix + ".source_surface_xyz", voxel["source_surface_xyz"], 3)
            _require_vec(prefix + ".source_surface_bounds.min", bounds.get("min"), 3)
            _require_vec(prefix + ".source_surface_bounds.max", bounds.get("max"), 3)
            _require_vec(prefix + ".tcp_hover_xyz", voxel["tcp_hover_xyz"], 3)
            _require_vec(prefix + ".tcp_quaternion", voxel["tcp_quaternion"], 4)
            _require_vec(prefix + ".preferred_ik_seed", voxel["preferred_ik_seed"], 6)
            _require_vec(prefix + ".actual_ik_solution", voxel["actual_ik_solution"], 6)
            voxel_count += 1
    return {
        **data,
        "schema_version": schema_version,
        "source_path": path,
        "embedded": True,
        "region_count": len(regions),
        "voxel_count": voxel_count,
        "status": "ok",
    }


def build_robot_code(args: argparse.Namespace) -> str:
    validated_region_data = load_validated_region_config(
        args.grasp_region_config,
        enabled=args.grasp_region == "auto",
    )
    cfg = {
        "task": args.task,
        "source": args.source,
        "destination": args.destination,
        "plan_only": args.plan_only,
        "perception_only": args.perception_only,
        "destination_perception_only": args.destination_perception_only,
        "execute": args.execute,
        "source_provider": args.source_provider,
        "source_width": args.source_width,
        "close_width": args.close_width,
        "source_xyz": [args.source_x, args.source_y, args.source_z],
        "grasp_region": args.grasp_region,
        "grasp_region_config": args.grasp_region_config,
        "validated_grasp_region_data": validated_region_data,
        "max_grasp_tilt": args.max_grasp_tilt,
        "visualize_candidates": args.visualize_candidates,
        "verbose_diagnostics": args.verbose_diagnostics,
        "explore_unvalidated_candidates": args.explore_unvalidated_candidates,
        "exploratory_planning_budget_s": args.exploratory_planning_budget,
        "aruco_frame": args.aruco_frame,
        "aruco_offset_xyz": [args.aruco_offset_x, args.aruco_offset_y, args.aruco_offset_z],
        "top_k": args.top_k,
        "transit_z": args.transit_z,
        "hover_height": args.hover_height,
        "grasp_z_offset": args.grasp_z_offset,
        "place_z_offset": args.place_z_offset,
        "velocity_scaling": 1.0,
        "acceleration_scaling": 1.0,
        "hardware_speed_percent": 100,
        "workspace": {
            "x": [args.x_min, args.x_max],
            "y": [args.y_min, args.y_max],
            "z": [args.z_min, args.z_max],
        },
    }
    return f"""
import json
import math
import os
import time
import xml.etree.ElementTree as ET
import atexit
import copy

import cv2
import numpy as np
import rospy
import tf2_ros
import yaml
from geometry_msgs.msg import Point, Pose, PoseStamped
from sensor_msgs.msg import CameraInfo, Image as RosImage
from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import DisplayTrajectory, MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionIK, GetPositionIKRequest, GetStateValidity, GetStateValidityRequest
from visualization_msgs.msg import Marker

CONFIG = {repr(cfg)}
TCP_LINK = "gripper_tcp"
LINK6_TO_TCP_TRANSLATION = [0.0, 0.0, 0.1358]
LINK6_TO_TCP_QUAT = [0.0, 0.0, 0.0, 1.0]
# Current TCP depth is the modeled midpoint of the two finger joint origins.
# It has not yet been physically measured at the real fingertip contact depth.
TCP_MODELED_APPROACH_DEPTH_M = 0.1358
TCP_LOCAL_APPROACH_AXIS = [0.0, 0.0, 1.0]
TCP_LOCAL_CLOSING_AXIS = [0.0, 1.0, 0.0]
TARGET_APPROACH_AXIS = [0.0, 0.0, -1.0]
TARGET_CLOSING_AXIS = [0.0, 1.0, 0.0]
CLEAR_DISPLAY_ON_EXIT = True


def section(title):
    print("\\n=== " + title + " ===", flush=True)


def clear_display_trajectory():
    try:
        pub = rospy.Publisher("/move_group/display_planned_path", DisplayTrajectory, queue_size=1, latch=True)
        deadline = time.time() + 1.0
        while pub.get_num_connections() == 0 and time.time() < deadline and not rospy.is_shutdown():
            time.sleep(0.05)
        pub.publish(DisplayTrajectory())
        print("DISPLAY_TRAJECTORY_CLEARED", flush=True)
    except Exception as exc:
        print("DISPLAY_TRAJECTORY_CLEAR_FAILED " + str(exc), flush=True)


def clear_display_trajectory_on_exit():
    if CLEAR_DISPLAY_ON_EXIT:
        clear_display_trajectory()


def publish_rviz_preview(planner, ordered_trajectories):
    global CLEAR_DISPLAY_ON_EXIT
    try:
        msg = DisplayTrajectory()
        msg.model_id = "piper"
        msg.trajectory_start = planner.get_current_state()
        msg.trajectory = [trajectory for trajectory in ordered_trajectories if trajectory is not None]
        pub = rospy.Publisher("/move_group/display_planned_path", DisplayTrajectory, queue_size=1, latch=True)
        deadline = time.time() + 1.0
        while pub.get_num_connections() == 0 and time.time() < deadline and not rospy.is_shutdown():
            time.sleep(0.05)
        pub.publish(msg)
        CLEAR_DISPLAY_ON_EXIT = False
        print("RVIZ_PREVIEW_DISPLAYED_JSON " + json.dumps({{
            "topic": "/move_group/display_planned_path",
            "trajectory_count": len(msg.trajectory),
            "loop_animation_expected": False,
            "cleared_at_next_request": True,
        }}, sort_keys=True), flush=True)
    except Exception as exc:
        print("RVIZ_PREVIEW_DISPLAY_FAILED " + str(exc), flush=True)


atexit.register(clear_display_trajectory_on_exit)


def finite_vec(name, values, n):
    if values is None or len(values) != n:
        raise RuntimeError(name + " missing or wrong length: " + repr(values))
    out = [float(v) for v in values]
    for value in out:
        if not math.isfinite(value):
            raise RuntimeError(name + " contains non-finite value: " + repr(values))
    return out


def unit_vec(name, values):
    vec = np.asarray(finite_vec(name, values, 3), dtype=float)
    norm = float(np.linalg.norm(vec))
    if norm <= 1e-9:
        raise RuntimeError(name + " has zero length")
    return vec / norm


def angle_to_down_deg(vector):
    vec = unit_vec("approach_axis", vector)
    down = np.array([0.0, 0.0, -1.0], dtype=float)
    dot = max(-1.0, min(1.0, float(np.dot(vec, down))))
    return math.degrees(math.acos(dot))


def rot_to_quat_xyzw(rot):
    m = np.asarray(rot, dtype=float).reshape(3, 3)
    trace = float(np.trace(m))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m[2, 1] - m[1, 2]) / s
        qy = (m[0, 2] - m[2, 0]) / s
        qz = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        qw = (m[2, 1] - m[1, 2]) / s
        qx = 0.25 * s
        qy = (m[0, 1] + m[1, 0]) / s
        qz = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        qw = (m[0, 2] - m[2, 0]) / s
        qx = (m[0, 1] + m[1, 0]) / s
        qy = 0.25 * s
        qz = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        qw = (m[1, 0] - m[0, 1]) / s
        qx = (m[0, 2] + m[2, 0]) / s
        qy = (m[1, 2] + m[2, 1]) / s
        qz = 0.25 * s
    quat = np.array([qx, qy, qz, qw], dtype=float)
    quat /= np.linalg.norm(quat)
    return [float(x) for x in quat.tolist()]


def quat_to_rot_xyzw(q):
    qx, qy, qz, qw = [float(x) for x in q]
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n <= 1e-9:
        raise RuntimeError("zero quaternion")
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ], dtype=float)


def compute_tcp_orientation(approach_target_axis, closing_target_axis):
    approach_local = unit_vec("tcp_local_approach_axis", TCP_LOCAL_APPROACH_AXIS)
    closing_local = unit_vec("tcp_local_closing_axis", TCP_LOCAL_CLOSING_AXIS)
    approach_target = unit_vec("target_approach_axis", approach_target_axis)
    closing_target = unit_vec("target_closing_axis", closing_target_axis)
    if abs(float(np.dot(approach_local, closing_local))) > 1e-6:
        raise RuntimeError("TCP approach and closing axes are not orthogonal")
    if abs(float(np.dot(approach_target, closing_target))) > 1e-6:
        raise RuntimeError("Target approach and closing axes are not orthogonal")
    x_local = np.cross(closing_local, approach_local)
    x_local /= np.linalg.norm(x_local)
    x_target = np.cross(closing_target, approach_target)
    x_target /= np.linalg.norm(x_target)
    local_basis = np.column_stack([x_local, closing_local, approach_local])
    target_basis = np.column_stack([x_target, closing_target, approach_target])
    rot = target_basis.dot(local_basis.T)
    quat = rot_to_quat_xyzw(rot)
    target_approach = rot.dot(approach_local)
    target_closing = rot.dot(closing_local)
    down = np.array([0.0, 0.0, -1.0], dtype=float)
    dot = max(-1.0, min(1.0, float(np.dot(target_approach, down))))
    angle = math.degrees(math.acos(dot))
    return quat, [float(x) for x in target_approach.tolist()], [float(x) for x in target_closing.tolist()], angle


def compute_top_down_tcp_orientation(closing_target_axis=None):
    return compute_tcp_orientation(TARGET_APPROACH_AXIS, closing_target_axis or TARGET_CLOSING_AXIS)


def top_down_orientation_candidates(current_tcp_pose):
    candidates = []
    exact_yaw_degrees = [0.0, 45.0, 90.0, 135.0, 180.0, -45.0, -90.0, -135.0]
    max_tilt = float(CONFIG.get("max_grasp_tilt", 25.0))
    tilt_degrees = [value for value in [5.0, 10.0, 15.0, 20.0, 25.0] if value <= max_tilt + 1e-9]

    for yaw_deg in exact_yaw_degrees:
        yaw = math.radians(yaw_deg)
        closing = [math.cos(yaw), math.sin(yaw), 0.0]
        quat, approach_axis, closing_axis, angle = compute_top_down_tcp_orientation(closing)
        candidates.append({{
            "phase": "exact_top_down",
            "yaw_rad": float(yaw),
            "yaw_deg": float(yaw_deg),
            "tilt_deg": 0.0,
            "closing_axis": [float(x) for x in closing],
            "quat": quat,
            "approach_axis": approach_axis,
            "target_closing_axis": closing_axis,
            "approach_angle_deg": angle,
        }})

    for yaw_deg in [0.0]:
        yaw = math.radians(yaw_deg)
        closing = [math.cos(yaw), math.sin(yaw), 0.0]
        closing_vec = unit_vec("candidate_closing_axis", closing)
        lateral_vec = np.cross(closing_vec, np.array([0.0, 0.0, -1.0], dtype=float))
        lateral_vec /= np.linalg.norm(lateral_vec)
        for tilt_deg in tilt_degrees:
            tilt_rad = math.radians(abs(tilt_deg))
            tilt_sign = 1.0 if tilt_deg >= 0.0 else -1.0
            approach = np.array([0.0, 0.0, -math.cos(tilt_rad)], dtype=float) + lateral_vec * (tilt_sign * math.sin(tilt_rad))
            approach /= np.linalg.norm(approach)
            adjusted_closing = closing_vec - approach * float(np.dot(closing_vec, approach))
            adjusted_closing /= np.linalg.norm(adjusted_closing)
            quat, approach_axis, closing_axis, angle = compute_tcp_orientation(approach, adjusted_closing)
            candidates.append({{
                "phase": "tilted",
                "yaw_rad": float(yaw),
                "yaw_deg": float(yaw_deg),
                "tilt_deg": float(tilt_deg),
                "closing_axis": [float(x) for x in closing],
                "quat": quat,
                "approach_axis": approach_axis,
                "target_closing_axis": closing_axis,
                "approach_angle_deg": angle,
            }})
    return candidates


def publish_tcp_marker(frame_id, pose):
    pub = rospy.Publisher("/abot_piper/gripper_tcp_approach_marker", Marker, queue_size=1, latch=True)
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "abot_piper"
    marker.id = 1
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.scale.x = 0.012
    marker.scale.y = 0.024
    marker.scale.z = 0.024
    marker.color.r = 0.0
    marker.color.g = 0.7
    marker.color.b = 1.0
    marker.color.a = 1.0
    rot = quat_to_rot_xyzw([
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ])
    start = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)
    approach = rot.dot(unit_vec("tcp_local_approach_axis", TCP_LOCAL_APPROACH_AXIS))
    end = start + approach * 0.08
    p0 = Point()
    p0.x, p0.y, p0.z = [float(x) for x in start.tolist()]
    p1 = Point()
    p1.x, p1.y, p1.z = [float(x) for x in end.tolist()]
    marker.points = [p0, p1]
    pub.publish(marker)


def assert_workspace(name, xyz):
    ws = CONFIG["workspace"]
    if not (ws["x"][0] <= xyz[0] <= ws["x"][1]):
        raise RuntimeError("%s x %.4f outside workspace %s" % (name, xyz[0], ws["x"]))
    if not (ws["y"][0] <= xyz[1] <= ws["y"][1]):
        raise RuntimeError("%s y %.4f outside workspace %s" % (name, xyz[1], ws["y"]))
    if not (ws["z"][0] <= xyz[2] <= ws["z"][1]):
        raise RuntimeError("%s z %.4f outside workspace %s" % (name, xyz[2], ws["z"]))


def pose_msg(xyz, quat):
    pose = Pose()
    pose.position.x = xyz[0]
    pose.position.y = xyz[1]
    pose.position.z = xyz[2]
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    return pose


def pose_to_dict(pose):
    return {{
        "position": [pose.position.x, pose.position.y, pose.position.z],
        "orientation_quat": [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ],
    }}


def pose_from_current(current):
    xyz = finite_vec("current_end_effector.position", current.get("position"), 3)
    quat = finite_vec("current_end_effector.orientation_quat", current.get("orientation_quat"), 4)
    return pose_msg(xyz, quat)


def start_state_from_trajectory(trajectory):
    points = getattr(getattr(trajectory, "joint_trajectory", None), "points", [])
    joint_names = list(getattr(getattr(trajectory, "joint_trajectory", None), "joint_names", []))
    if not points or not joint_names:
        return None
    state = RobotState()
    state.joint_state.name = joint_names
    state.joint_state.position = list(points[-1].positions)
    return state


def trajectory_point_count(trajectory):
    return len(getattr(getattr(trajectory, "joint_trajectory", None), "points", []))


MOVEIT_ERROR_NAMES = {{
    MoveItErrorCodes.SUCCESS: "SUCCESS",
    MoveItErrorCodes.FAILURE: "FAILURE",
    MoveItErrorCodes.PLANNING_FAILED: "PLANNING_FAILED",
    MoveItErrorCodes.INVALID_MOTION_PLAN: "INVALID_MOTION_PLAN",
    MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
    MoveItErrorCodes.CONTROL_FAILED: "CONTROL_FAILED",
    MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA: "UNABLE_TO_AQUIRE_SENSOR_DATA",
    MoveItErrorCodes.TIMED_OUT: "TIMED_OUT",
    MoveItErrorCodes.PREEMPTED: "PREEMPTED",
    MoveItErrorCodes.START_STATE_IN_COLLISION: "START_STATE_IN_COLLISION",
    MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
    MoveItErrorCodes.GOAL_IN_COLLISION: "GOAL_IN_COLLISION",
    MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS: "GOAL_VIOLATES_PATH_CONSTRAINTS",
    MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED: "GOAL_CONSTRAINTS_VIOLATED",
    MoveItErrorCodes.INVALID_GROUP_NAME: "INVALID_GROUP_NAME",
    MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS: "INVALID_GOAL_CONSTRAINTS",
    MoveItErrorCodes.INVALID_ROBOT_STATE: "INVALID_ROBOT_STATE",
    MoveItErrorCodes.INVALID_LINK_NAME: "INVALID_LINK_NAME",
    MoveItErrorCodes.INVALID_OBJECT_NAME: "INVALID_OBJECT_NAME",
    MoveItErrorCodes.FRAME_TRANSFORM_FAILURE: "FRAME_TRANSFORM_FAILURE",
    MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE: "COLLISION_CHECKING_UNAVAILABLE",
    MoveItErrorCodes.ROBOT_STATE_STALE: "ROBOT_STATE_STALE",
    MoveItErrorCodes.SENSOR_INFO_STALE: "SENSOR_INFO_STALE",
    MoveItErrorCodes.NO_IK_SOLUTION: "NO_IK_SOLUTION",
}}


def moveit_error_summary(error_code):
    value = getattr(error_code, "val", None)
    if value is None:
        return str(error_code)
    return "%s(%s)" % (MOVEIT_ERROR_NAMES.get(value, "UNKNOWN"), value)


def trajectory_joint_summary(trajectory):
    jt = getattr(trajectory, "joint_trajectory", None)
    points = list(getattr(jt, "points", []))
    if not points:
        return {{"first_joint_positions": [], "final_joint_positions": []}}
    return {{
        "first_joint_positions": [float(x) for x in points[0].positions],
        "final_joint_positions": [float(x) for x in points[-1].positions],
    }}


def check_current_state_validity(planner):
    try:
        rospy.wait_for_service("/check_state_validity", timeout=2.0)
        service = rospy.ServiceProxy("/check_state_validity", GetStateValidity)
        req = GetStateValidityRequest()
        req.robot_state = planner.get_current_state()
        req.group_name = "arm"
        resp = service(req)
        return {{
            "available": True,
            "valid": bool(resp.valid),
            "contacts": len(getattr(resp, "contacts", [])),
        }}
    except Exception as exc:
        return {{"available": False, "valid": None, "error": str(exc)}}


def joint_limits_from_robot_description(joint_names):
    limits = {{}}
    try:
        root = ET.fromstring(rospy.get_param("/robot_description"))
        wanted = set(joint_names)
        for joint in root.findall("joint"):
            name = joint.attrib.get("name")
            if name not in wanted:
                continue
            limit = joint.find("limit")
            if limit is None:
                continue
            lower = float(limit.attrib.get("lower", "-3.141592653589793"))
            upper = float(limit.attrib.get("upper", "3.141592653589793"))
            limits[name] = (lower, upper)
    except Exception as exc:
        print("JOINT_LIMIT_PARSE_WARNING " + str(exc))
    return limits


def clamp_joint(name, value, limits):
    if name not in limits:
        return float(value)
    lower, upper = limits[name]
    return float(max(lower, min(upper, value)))


def ik_seed_states(planner, preferred_seed_joint_state=None):
    current_state = planner.get_current_state()
    joint_names = list(planner.get_active_joints())
    current_values = [float(x) for x in planner.get_current_joint_values()]
    limits = joint_limits_from_robot_description(joint_names)
    offsets = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.20, 0.0, 0.0, 0.0, 0.0, 0.0],
        [-0.20, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.10, -0.10, 0.0, 0.0, 0.0],
    ]
    seeds = []
    if preferred_seed_joint_state is not None:
        state = RobotState()
        state.is_diff = True
        state.joint_state.name = joint_names
        state.joint_state.position = [
            clamp_joint(joint_name, preferred_seed_joint_state[index], limits)
            for index, joint_name in enumerate(joint_names)
        ]
        seeds.append({{
            "index": 0,
            "label": "validated_preferred_seed",
            "state": state,
            "joint_positions": [float(x) for x in state.joint_state.position],
        }})
    for index, offset in enumerate(offsets):
        state = RobotState()
        state.is_diff = True
        state.joint_state.name = joint_names
        positions = []
        for joint_index, joint_name in enumerate(joint_names):
            delta = offset[joint_index] if joint_index < len(offset) else 0.0
            base = current_values[joint_index] if joint_index < len(current_values) else 0.0
            positions.append(clamp_joint(joint_name, base + delta, limits))
        state.joint_state.position = positions
        seeds.append({{
            "index": len(seeds),
            "label": "current" if index == 0 else "offset_%d" % index,
            "state": state,
            "joint_positions": [float(x) for x in positions],
        }})
    return seeds


def solution_joint_limit_distances(solution, active_joint_names, limits):
    name_to_position = dict(zip(solution.joint_state.name, solution.joint_state.position))
    distances = []
    positions = []
    for name in active_joint_names:
        if name not in name_to_position:
            continue
        position = float(name_to_position[name])
        positions.append(position)
        if name in limits:
            lower, upper = limits[name]
            distances.append(min(position - lower, upper - position))
    return {{
        "joint_names": list(active_joint_names),
        "joint_positions": positions,
        "min_joint_limit_distance": float(min(distances)) if distances else None,
    }}


def check_pose_ik(planner, pose, preferred_seed_joint_state=None):
    try:
        rospy.wait_for_service("/compute_ik", timeout=5.0)
        service = rospy.ServiceProxy("/compute_ik", GetPositionIK)
        results = []
        active_joint_names = list(planner.get_active_joints())
        limits = joint_limits_from_robot_description(active_joint_names)
        for seed in ik_seed_states(planner, preferred_seed_joint_state):
            req = GetPositionIKRequest()
            req.ik_request.group_name = "arm"
            req.ik_request.ik_link_name = TCP_LINK
            req.ik_request.robot_state = seed["state"]
            req.ik_request.avoid_collisions = True
            req.ik_request.timeout = rospy.Duration(2.0)
            stamped = PoseStamped()
            stamped.header.frame_id = planner.get_planning_frame()
            stamped.header.stamp = rospy.Time.now()
            stamped.pose = pose
            req.ik_request.pose_stamped = stamped
            resp = service(req)
            success = resp.error_code.val == MoveItErrorCodes.SUCCESS
            solution = solution_joint_limit_distances(resp.solution, active_joint_names, limits) if success else {{}}
            results.append({{
                "seed_index": seed["index"],
                "seed_label": seed["label"],
                "success": success,
                "error": moveit_error_summary(resp.error_code),
                **solution,
            }})
        success = any(item["success"] for item in results)
        first_error = next((item["error"] for item in results if item["success"]), None)
        if first_error is None and results:
            first_error = results[0]["error"]
        return {{
            "available": True,
            "success": success,
            "error": first_error,
            "seeds": results,
            "solution": next((item for item in results if item.get("success")), None),
        }}
    except Exception as exc:
        return {{"available": False, "success": False, "error": str(exc), "seeds": []}}


def safe_planning_time(value):
    try:
        out = float(value)
    except Exception:
        return None
    if not math.isfinite(out) or out < 0.0 or out > 3600.0:
        return None
    # MoveIt can leave this double uninitialized on some failure paths.
    if abs(out) < 1e-9:
        return None
    return out


def summarize_plan(name, result, raise_on_failure=True, planner_wall_time_s=None):
    if isinstance(result, tuple):
        success = bool(result[0])
        trajectory = result[1]
        planning_time = safe_planning_time(result[2]) if len(result) > 2 else None
        error_code = moveit_error_summary(result[3]) if len(result) > 3 else ""
    else:
        trajectory = result
        points = getattr(getattr(trajectory, "joint_trajectory", None), "points", [])
        success = len(points) > 0
        planning_time = None
        error_code = ""
    wall_time = float(planner_wall_time_s) if planner_wall_time_s is not None else None
    points = getattr(getattr(trajectory, "joint_trajectory", None), "points", [])
    count = len(points)
    print("PLAN %s success=%s points=%d planner_reported_time=%s wall_time=%s error=%s" % (
        name, success, count, str(planning_time), str(wall_time), error_code
    ))
    if raise_on_failure and (not success or count == 0):
        raise RuntimeError("MoveIt planning failed for %s: error=%s points=%d" % (name, error_code, count))
    summary = {{
        "name": name,
        "type": "pose_target",
        "success": success,
        "points": count,
        "planner_reported_time_s": planning_time,
        "planner_wall_time_s": wall_time,
        "error": error_code,
    }}
    summary.update(trajectory_joint_summary(trajectory))
    return trajectory, summary


def retime_trajectory(planner, start_state, trajectory):
    return planner.retime_trajectory(
        start_state,
        trajectory,
        velocity_scaling_factor=float(CONFIG["velocity_scaling"]),
        acceleration_scaling_factor=float(CONFIG["acceleration_scaling"]),
    )


def plan_cartesian(planner, name, waypoints, start_state=None, raise_on_failure=True):
    if start_state is not None:
        planner.set_start_state(start_state)
    for index, waypoint in enumerate(waypoints):
        assert_workspace("%s waypoint %d" % (name, index), [
            waypoint.position.x,
            waypoint.position.y,
            waypoint.position.z,
        ])
    trajectory, fraction = planner.compute_cartesian_path(
        waypoints,
        0.01,
        0.0,
    )
    retime_start = start_state if start_state is not None else planner.get_current_state()
    if fraction >= 0.999:
        trajectory = retime_trajectory(planner, retime_start, trajectory)
    point_count = trajectory_point_count(trajectory)
    diag = {{
        "name": name,
        "type": "cartesian",
        "success": bool(fraction >= 0.999 and point_count > 0),
        "fraction": float(fraction),
        "points": point_count,
        "waypoints": [pose_to_dict(pose) for pose in waypoints],
        "final_requested_pose": pose_to_dict(waypoints[-1]),
    }}
    diag.update(trajectory_joint_summary(trajectory))
    print("TRAJECTORY_DIAGNOSTIC_JSON " + json.dumps(diag, sort_keys=True))
    if raise_on_failure and fraction < 0.999:
        raise RuntimeError("Cartesian path %s incomplete: fraction=%.6f" % (name, fraction))
    if raise_on_failure and point_count == 0:
        raise RuntimeError("Cartesian path %s produced no trajectory points" % name)
    final_state = start_state_from_trajectory(trajectory)
    return trajectory, diag, final_state


def reverse_trajectory_for_lift(name, descend_trajectory, final_requested_pose):
    jt = getattr(descend_trajectory, "joint_trajectory", None)
    points = list(getattr(jt, "points", []))
    if len(points) < 2:
        raise RuntimeError("%s cannot reverse descend trajectory with %d points" % (name, len(points)))
    trajectory = copy.deepcopy(descend_trajectory)
    total = points[-1].time_from_start if hasattr(points[-1], "time_from_start") else rospy.Duration(0.0)
    reversed_points = []
    for point in reversed(points):
        new_point = copy.deepcopy(point)
        try:
            new_point.time_from_start = total - point.time_from_start
        except Exception:
            pass
        if getattr(new_point, "velocities", None):
            new_point.velocities = [-float(v) for v in new_point.velocities]
        reversed_points.append(new_point)
    trajectory.joint_trajectory.points = reversed_points
    diag = {{
        "name": name,
        "type": "cartesian_reverse",
        "success": True,
        "fraction": 1.0,
        "points": trajectory_point_count(trajectory),
        "source": "reverse_of_source_descend",
        "final_requested_pose": pose_to_dict(final_requested_pose),
    }}
    diag.update(trajectory_joint_summary(trajectory))
    print("TRAJECTORY_DIAGNOSTIC_JSON " + json.dumps(diag, sort_keys=True))
    final_state = start_state_from_trajectory(trajectory)
    return trajectory, diag, final_state


def plan_target(planner, name, xyz, quat, start_state=None, raise_on_failure=True):
    assert_workspace(name, xyz)
    if start_state is not None:
        planner.set_start_state(start_state)
    planner.clear_pose_targets()
    requested_pose = pose_msg(xyz, quat)
    planner.set_pose_target(requested_pose)
    plan_started = time.time()
    result = planner.plan()
    wall_time = time.time() - plan_started
    trajectory, summary = summarize_plan(
        name,
        result,
        raise_on_failure=raise_on_failure,
        planner_wall_time_s=wall_time,
    )
    summary["final_requested_pose"] = pose_to_dict(requested_pose)
    print("TRAJECTORY_DIAGNOSTIC_JSON " + json.dumps(summary, sort_keys=True))
    state = start_state_from_trajectory(trajectory)
    if state is not None:
        planner.set_start_state(state)
    return trajectory, summary


def quat_angle_deg(a, b):
    qa = np.asarray(a, dtype=float)
    qb = np.asarray(b, dtype=float)
    qa /= np.linalg.norm(qa)
    qb /= np.linalg.norm(qb)
    dot = abs(float(np.dot(qa, qb)))
    dot = max(-1.0, min(1.0, dot))
    return math.degrees(2.0 * math.acos(dot))


def fk_pose_for_state(planner, state):
    rospy.wait_for_service("/compute_fk", timeout=5.0)
    service = rospy.ServiceProxy("/compute_fk", GetPositionFK)
    req = GetPositionFKRequest()
    req.header.frame_id = planner.get_planning_frame()
    req.fk_link_names = [TCP_LINK]
    req.robot_state = state
    resp = service(req)
    if resp.error_code.val != MoveItErrorCodes.SUCCESS or not resp.pose_stamped:
        raise RuntimeError("FK failed for final trajectory state: " + moveit_error_summary(resp.error_code))
    return resp.pose_stamped[0].pose


def verify_final_plan_state(planner, trajectory, target_pose, ik_solution):
    final_state = start_state_from_trajectory(trajectory)
    if final_state is None:
        return {{"success": False, "error": "missing final trajectory state"}}
    fk_pose = fk_pose_for_state(planner, final_state)
    target_xyz = np.asarray([target_pose.position.x, target_pose.position.y, target_pose.position.z], dtype=float)
    actual_xyz = np.asarray([fk_pose.position.x, fk_pose.position.y, fk_pose.position.z], dtype=float)
    target_quat = [target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w]
    actual_quat = [fk_pose.orientation.x, fk_pose.orientation.y, fk_pose.orientation.z, fk_pose.orientation.w]
    try:
        rospy.wait_for_service("/check_state_validity", timeout=2.0)
        service = rospy.ServiceProxy("/check_state_validity", GetStateValidity)
        req = GetStateValidityRequest()
        req.robot_state = final_state
        req.group_name = "arm"
        resp = service(req)
        validity = {{"available": True, "valid": bool(resp.valid), "contacts": len(getattr(resp, "contacts", []))}}
    except Exception as exc:
        validity = {{"available": False, "valid": None, "error": str(exc)}}
    position_error = float(np.linalg.norm(actual_xyz - target_xyz))
    orientation_error = quat_angle_deg(target_quat, actual_quat)
    return {{
        "success": bool(position_error <= 0.005 and orientation_error <= 5.0 and validity.get("valid") is not False),
        "target_tcp_link": TCP_LINK,
        "position_error_m": position_error,
        "orientation_error_deg": orientation_error,
        "fk_pose": pose_to_dict(fk_pose),
        "state_validity": validity,
        "ik_solution": ik_solution,
    }}


def plan_joint_solution(planner, name, pose, ik_result, raise_on_failure=True):
    solution = ik_result.get("solution") or {{}}
    joints = solution.get("joint_positions")
    if not joints:
        return None, {{
            "success": False,
            "error": "missing IK joint solution",
            "points": 0,
        }}
    planner.clear_pose_targets()
    planner.set_joint_value_target([float(x) for x in joints])
    plan_started = time.time()
    result = planner.plan()
    wall_time = time.time() - plan_started
    trajectory, summary = summarize_plan(
        name,
        result,
        raise_on_failure=raise_on_failure,
        planner_wall_time_s=wall_time,
    )
    summary["final_requested_pose"] = pose_to_dict(pose)
    summary["target_source"] = "validated_ik_joint_solution"
    verification = verify_final_plan_state(planner, trajectory, pose, solution)
    summary["final_state_verification"] = verification
    print("SELECTED_IK_SOLUTION_JSON " + json.dumps(verification, sort_keys=True), flush=True)
    print("TRAJECTORY_DIAGNOSTIC_JSON " + json.dumps(summary, sort_keys=True))
    if raise_on_failure and not verification.get("success"):
        raise RuntimeError("Final planned TCP state failed validation")
    return trajectory, summary


def tcp_height_plan(source_z, hover_height, minimum_link6_transit_z, candidate):
    rot = quat_to_rot_xyzw(candidate["quat"])
    world_link6_to_tcp = rot.dot(np.asarray(LINK6_TO_TCP_TRANSLATION, dtype=float))
    tcp_z_from_object_clearance = float(source_z) + float(hover_height)
    tcp_z_from_link6_clearance = float(minimum_link6_transit_z) + float(world_link6_to_tcp[2])
    if candidate.get("phase") == "validated_region":
        selected_tcp_hover_z = tcp_z_from_object_clearance
    else:
        selected_tcp_hover_z = max(tcp_z_from_object_clearance, tcp_z_from_link6_clearance)
    return {{
        "source_surface_z": float(source_z),
        "hover_height": float(hover_height),
        "minimum_link6_transit_z": float(minimum_link6_transit_z),
        "tcp_z_from_object_clearance": tcp_z_from_object_clearance,
        "tcp_z_from_link6_clearance": tcp_z_from_link6_clearance,
        "selected_tcp_hover_z": selected_tcp_hover_z,
        "validated_region_tcp_height": bool(candidate.get("phase") == "validated_region"),
        "world_link6_to_tcp_translation": [float(x) for x in world_link6_to_tcp.tolist()],
    }}


def reachability_diagnostic(tcp_xyz, candidate, world_link6_to_tcp):
    tcp = np.asarray(tcp_xyz, dtype=float)
    offset = np.asarray(world_link6_to_tcp, dtype=float)
    link6 = tcp - offset
    return {{
        "tcp_hover_xyz": [float(x) for x in tcp.tolist()],
        "target_tcp_quaternion": candidate["quat"],
        "world_link6_to_tcp_translation": [float(x) for x in offset.tolist()],
        "implied_link6_xyz": [float(x) for x in link6.tolist()],
        "base_to_tcp_distance_m": float(np.linalg.norm(tcp)),
        "base_to_link6_distance_m": float(np.linalg.norm(link6)),
        "yaw_rad": float(candidate["yaw_rad"]),
        "tilt_deg": float(candidate.get("tilt_deg", 0.0)),
    }}


def load_validated_grasp_regions():
    if CONFIG.get("grasp_region") != "auto":
        return []
    data = CONFIG.get("validated_grasp_region_data") or {{}}
    status = {{
        "host_source_path": CONFIG.get("grasp_region_config"),
        "embedded": bool(data.get("embedded")),
        "status": data.get("status"),
        "schema_version": data.get("schema_version"),
        "region_count": int(data.get("region_count") or 0),
        "voxel_count": int(data.get("voxel_count") or 0),
    }}
    print("VALIDATED_GRASP_REGION_CONFIG_JSON " + json.dumps(status, sort_keys=True), flush=True)
    if not data.get("embedded") or data.get("status") != "ok":
        print("VALIDATED_GRASP_REGION_CONFIG_ERROR_JSON " + json.dumps(status, sort_keys=True), flush=True)
        raise RuntimeError("Validated grasp region configuration is unavailable or invalid")
    regions = data.get("regions") or []
    if not isinstance(regions, list):
        status["error"] = "regions is not a list"
        print("VALIDATED_GRASP_REGION_CONFIG_ERROR_JSON " + json.dumps(status, sort_keys=True), flush=True)
        raise RuntimeError("Validated grasp region configuration has invalid regions list")
    if not regions:
        status["error"] = "no regions"
        print("VALIDATED_GRASP_REGION_CONFIG_ERROR_JSON " + json.dumps(status, sort_keys=True), flush=True)
        raise RuntimeError("Validated grasp region configuration has no regions")
    return regions


def vec_distance(a, b):
    aa = np.asarray(a, dtype=float)
    bb = np.asarray(b, dtype=float)
    return float(np.linalg.norm(aa - bb))


def region_for_hover_height(region):
    surfaces = region.get("cup_surface_regions") or {{}}
    if not surfaces:
        return None, None
    hover = float(CONFIG.get("hover_height", 0.10))
    best_key = min(surfaces.keys(), key=lambda key: abs(float(key) - hover))
    return str(best_key), surfaces[best_key]


def point_in_box(point, min_xyz, max_xyz):
    return all(float(min_xyz[i]) <= float(point[i]) <= float(max_xyz[i]) for i in range(3))


def voxel_source_comparison_geometry(voxel):
    bounds = voxel.get("source_surface_bounds") or {{}}
    surface_min = finite_vec("voxel.source_surface_bounds.min", bounds.get("min"), 3)
    surface_max = finite_vec("voxel.source_surface_bounds.max", bounds.get("max"), 3)
    surface_center = finite_vec("voxel.source_surface_xyz", voxel.get("source_surface_xyz"), 3)
    grasp_depth = voxel.get("grasp_depth_m")
    if grasp_depth is None:
        comparison_center = list(surface_center)
        comparison_min = list(surface_min)
        comparison_max = list(surface_max)
        semantics = "legacy_surface_bounds_without_grasp_depth"
    else:
        dz = float(grasp_depth)
        comparison_center = [surface_center[0], surface_center[1], surface_center[2] + dz]
        comparison_min = [surface_min[0], surface_min[1], surface_min[2] + dz]
        comparison_max = [surface_max[0], surface_max[1], surface_max[2] + dz]
        semantics = "source_surface_plus_grasp_depth_matches_perception_grasp_center"
    return {{
        "surface_center": surface_center,
        "surface_min": surface_min,
        "surface_max": surface_max,
        "comparison_center": comparison_center,
        "comparison_min": comparison_min,
        "comparison_max": comparison_max,
        "grasp_depth_m": None if grasp_depth is None else float(grasp_depth),
        "comparison_semantics": semantics,
    }}


def select_validated_voxel(region, point, usage):
    voxels = region.get("validated_voxels") or []
    rows = []
    for voxel in voxels:
        if usage not in (voxel.get("usage") or []):
            continue
        geometry = voxel_source_comparison_geometry(voxel)
        min_xyz = geometry["comparison_min"]
        max_xyz = geometry["comparison_max"]
        center = geometry["comparison_center"]
        inside = point_in_box(point, min_xyz, max_xyz)
        signed_displacement = []
        for axis in range(3):
            value = float(point[axis])
            if value < min_xyz[axis]:
                signed_displacement.append(value - min_xyz[axis])
            elif value > max_xyz[axis]:
                signed_displacement.append(value - max_xyz[axis])
            else:
                signed_displacement.append(0.0)
        rows.append({{
            "voxel": voxel,
            "inside": inside,
            "displacement": vec_distance(point, center),
            "signed_displacement_to_bounds": signed_displacement,
            "surface_center": geometry["surface_center"],
            "surface_min": geometry["surface_min"],
            "surface_max": geometry["surface_max"],
            "comparison_center": center,
            "comparison_min": min_xyz,
            "comparison_max": max_xyz,
            "grasp_depth_m": geometry["grasp_depth_m"],
            "comparison_semantics": geometry["comparison_semantics"],
        }})
    rows.sort(key=lambda row: (not row["inside"], row["displacement"]))
    return rows[0] if rows else None


def select_validated_grasp_region(source_base):
    regions = load_validated_grasp_regions()
    if not regions:
        selection = {{
            "enabled": CONFIG.get("grasp_region") == "auto",
            "detected_source_xyz": [float(x) for x in source_base],
            "selected_region": None,
            "source_inside_region": False,
            "nearest_region_displacement": None,
            "reason": "no validated regions loaded",
        }}
        print("VALIDATED_GRASP_REGION_SELECTION_JSON " + json.dumps(selection, sort_keys=True))
        return None, selection

    rows = []
    point = [float(x) for x in source_base]
    for region in regions:
        hover_key, surface = region_for_hover_height(region)
        if not surface:
            continue
        min_xyz = finite_vec("region.min", surface.get("min"), 3)
        max_xyz = finite_vec("region.max", surface.get("max"), 3)
        center = finite_vec("region.center", surface.get("center"), 3)
        inside = point_in_box(point, min_xyz, max_xyz)
        displacement = vec_distance(point, center)
        rows.append({{
            "region": region,
            "hover_height_key": hover_key,
            "inside": inside,
            "displacement": displacement,
            "surface_center": center,
            "surface_min": min_xyz,
            "surface_max": max_xyz,
        }})
    if not rows:
        raise RuntimeError("Validated grasp region file contains no usable cup surface regions")
    rows.sort(key=lambda row: (not row["inside"], row["displacement"]))
    selected = rows[0]
    region = selected["region"]
    voxel_row = select_validated_voxel(region, point, "source_pick")
    voxel = voxel_row["voxel"] if voxel_row else None
    voxel_inside = bool(voxel_row and voxel_row["inside"])
    selected_source = voxel if voxel_inside else region
    quat_key = "tcp_quaternion" if voxel_inside else "representative_tcp_quaternion"
    quat = finite_vec("region.tcp_quaternion", selected_source.get(quat_key), 4)
    rot = quat_to_rot_xyzw(quat)
    approach = rot.dot(unit_vec("tcp_local_approach_axis", TCP_LOCAL_APPROACH_AXIS))
    closing = rot.dot(unit_vec("tcp_local_closing_axis", TCP_LOCAL_CLOSING_AXIS))
    approach_angle = angle_to_down_deg(approach)
    allowed = region.get("allowed_approach_angle_deg") or [0.0, float(CONFIG.get("max_grasp_tilt", 25.0))]
    max_allowed = min(float(allowed[-1]), float(CONFIG.get("max_grasp_tilt", 25.0)))
    selection = {{
        "enabled": True,
        "detected_source_xyz": point,
        "selected_region": region.get("name"),
        "selected_voxel": voxel.get("voxel_id") if voxel else None,
        "source_inside_region": bool(selected["inside"]),
        "source_inside_validated_voxel": voxel_inside,
        "region_execution_validated": bool(region.get("execution_validated", False)),
        "validation_scope": region.get("validation_scope"),
        "nearest_region_displacement": float(selected["displacement"]),
        "nearest_voxel_displacement": float(voxel_row["displacement"]) if voxel_row else None,
        "nearest_voxel": voxel.get("voxel_id") if voxel else None,
        "nearest_voxel_bounds": {{
            "min": voxel_row["surface_min"],
            "max": voxel_row["surface_max"],
        }} if voxel_row else None,
        "nearest_voxel_comparison_bounds": {{
            "min": voxel_row["comparison_min"],
            "max": voxel_row["comparison_max"],
        }} if voxel_row else None,
        "nearest_voxel_center": voxel_row["surface_center"] if voxel_row else None,
        "nearest_voxel_comparison_center": voxel_row["comparison_center"] if voxel_row else None,
        "nearest_voxel_grasp_depth_m": voxel_row["grasp_depth_m"] if voxel_row else None,
        "nearest_voxel_comparison_semantics": voxel_row["comparison_semantics"] if voxel_row else None,
        "nearest_voxel_signed_displacement_to_bounds": voxel_row["signed_displacement_to_bounds"] if voxel_row else None,
        "hover_height_key": selected["hover_height_key"],
        "selected_tcp_orientation": quat,
        "approach_angle": float(approach_angle),
        "selected_seed_joint_state": selected_source.get("preferred_ik_seed") or region.get("representative_joint_state"),
        "selected_ik_solution": selected_source.get("actual_ik_solution"),
        "region_surface_center": selected["surface_center"],
        "region_surface_min": selected["surface_min"],
        "region_surface_max": selected["surface_max"],
        "validated_hover_height_m": selected_source.get("validated_hover_height_m"),
        "transit_result": None,
        "descend_fraction": None,
        "lift_fraction": None,
    }}
    requested_hover = float(CONFIG.get("hover_height", 0.10))
    validated_hover = selected_source.get("validated_hover_height_m")
    if validated_hover is not None and abs(float(validated_hover) - requested_hover) > 1e-6:
        selection["hover_height_mismatch"] = {{
            "requested": requested_hover,
            "validated": float(validated_hover),
        }}
        if CONFIG.get("execute"):
            print("VALIDATED_GRASP_REGION_SELECTION_JSON " + json.dumps(selection, sort_keys=True), flush=True)
            raise RuntimeError("Execute refused: requested hover height does not match validated voxel height")
    if CONFIG.get("execute") and not voxel_inside:
        print("VALIDATED_GRASP_REGION_SELECTION_JSON " + json.dumps(selection, sort_keys=True))
        raise RuntimeError("Execute refused: detected source is outside all individually validated grasp voxels")
    if approach_angle > max_allowed + 1e-6:
        print("VALIDATED_GRASP_REGION_SELECTION_JSON " + json.dumps(selection, sort_keys=True))
        raise RuntimeError("Validated region approach angle %.3f exceeds max %.3f" % (approach_angle, max_allowed))
    print("VALIDATED_GRASP_REGION_SELECTION_JSON " + json.dumps(selection, sort_keys=True))
    if CONFIG.get("grasp_region") == "auto" and not voxel_inside and not CONFIG.get("explore_unvalidated_candidates"):
        print("SOURCE_OUTSIDE_VALIDATED_VOXEL_JSON " + json.dumps(selection, sort_keys=True), flush=True)
        raise RuntimeError("Detected source is outside all individually validated grasp voxels; rerun with --explore-unvalidated-candidates for diagnostics only")
    candidate = {{
        "phase": "validated_region",
        "region_name": region.get("name"),
        "voxel_id": voxel.get("voxel_id") if voxel else None,
        "yaw_rad": math.radians(float(region.get("closing_axis_yaw_range_deg", [0.0, 0.0])[0])),
        "yaw_deg": float(region.get("closing_axis_yaw_range_deg", [0.0, 0.0])[0]),
        "tilt_deg": float(approach_angle),
        "closing_axis": [float(x) for x in closing.tolist()],
        "quat": quat,
        "approach_axis": [float(x) for x in approach.tolist()],
        "target_closing_axis": [float(x) for x in closing.tolist()],
        "approach_angle_deg": float(approach_angle),
        "seed_joint_state": selected_source.get("preferred_ik_seed") or region.get("representative_joint_state"),
        "preferred_ik_solution": selected_source.get("actual_ik_solution"),
        "validated_region_selection": selection,
    }}
    return candidate, selection


def plan_transit_to_source_hover(planner, source_base, current_tcp_pose):
    rows = []
    hover_height = float(CONFIG["hover_height"])
    minimum_link6_transit_z = float(CONFIG["transit_z"])
    candidates = top_down_orientation_candidates(current_tcp_pose)
    validated_candidate, validated_selection = select_validated_grasp_region(source_base)
    if validated_candidate is not None and (
        validated_selection.get("source_inside_validated_voxel")
        or CONFIG.get("explore_unvalidated_candidates")
    ):
        candidates = [validated_candidate] + candidates
    exact_candidates = [candidate for candidate in candidates if candidate["phase"] == "exact_top_down"]
    tilted_candidates = [candidate for candidate in candidates if candidate["phase"] == "tilted"]
    validated_candidates = [candidate for candidate in candidates if candidate["phase"] == "validated_region"]
    control_candidate = (validated_candidates or exact_candidates)[0]
    control_height = tcp_height_plan(source_base[2], hover_height, minimum_link6_transit_z, control_candidate)
    control_xyz = [source_base[0], source_base[1], control_height["selected_tcp_hover_z"]]
    current_orientation_quat = [
        current_tcp_pose.orientation.x,
        current_tcp_pose.orientation.y,
        current_tcp_pose.orientation.z,
        current_tcp_pose.orientation.w,
    ]
    current_orientation_ik = check_pose_ik(planner, pose_msg(control_xyz, current_orientation_quat))
    exact_top_down_ik = check_pose_ik(planner, pose_msg(control_xyz, control_candidate["quat"]))
    current_orientation_report = {{
        "success": bool(current_orientation_ik.get("success")),
        "error": current_orientation_ik.get("error"),
    }}
    exact_top_down_report = {{
        "success": bool(exact_top_down_ik.get("success")),
        "error": exact_top_down_ik.get("error"),
    }}
    if CONFIG.get("verbose_diagnostics"):
        current_orientation_report["seeds"] = current_orientation_ik.get("seeds", [])
        exact_top_down_report["seeds"] = exact_top_down_ik.get("seeds", [])
    print("TCP_IK_CONTROL_JSON " + json.dumps({{
        "tcp_xyz": control_xyz,
        "current_orientation": current_orientation_report,
        "exact_top_down_yaw0": exact_top_down_report,
    }}, sort_keys=True))

    if CONFIG.get("grasp_region") == "auto" and not CONFIG.get("explore_unvalidated_candidates"):
        phase_sequence = [("validated_region", validated_candidates)]
    else:
        phase_sequence = [
            ("validated_region", validated_candidates),
            ("exact_top_down", exact_candidates),
            ("tilted", tilted_candidates),
        ]
    if CONFIG.get("execute") and validated_candidates:
        phase_sequence = [("validated_region", validated_candidates)]
    exploratory_started = time.time()
    exploratory_budget = float(CONFIG.get("exploratory_planning_budget_s", 60.0))
    for phase_name, phase_candidates in phase_sequence:
        phase_had_ik_success = False
        if phase_name == "tilted":
            print("TCP_TILTED_CANDIDATES_START no exact top-down candidate produced a valid planned trajectory")
        for candidate in phase_candidates:
            if phase_name != "validated_region" and CONFIG.get("explore_unvalidated_candidates"):
                elapsed = time.time() - exploratory_started
                if elapsed >= exploratory_budget:
                    print("EXPLORATORY_PLANNING_BUDGET_JSON " + json.dumps({{
                        "budget_s": exploratory_budget,
                        "elapsed_s": elapsed,
                        "stopped": True,
                        "attempted_candidates": len(rows),
                    }}, sort_keys=True), flush=True)
                    break
            index = len(rows)
            quat = candidate["quat"]
            height_plan = tcp_height_plan(source_base[2], hover_height, minimum_link6_transit_z, candidate)
            xyz = [source_base[0], source_base[1], height_plan["selected_tcp_hover_z"]]
            hover_pose = pose_msg(xyz, quat)
            assert_workspace("source_hover candidate %d" % index, xyz)
            reachability = reachability_diagnostic(xyz, candidate, height_plan["world_link6_to_tcp_translation"])
            if CONFIG.get("verbose_diagnostics"):
                print("TCP_HEIGHT_PLAN_JSON " + json.dumps(height_plan, sort_keys=True))
                print("TCP_REACHABILITY_CANDIDATE_JSON " + json.dumps(reachability, sort_keys=True))
            ik_result = check_pose_ik(planner, hover_pose, candidate.get("seed_joint_state"))
            ik_report = {{
                "index": index,
                "phase": phase_name,
                "yaw_rad": float(candidate["yaw_rad"]),
                "yaw_deg": float(candidate["yaw_deg"]),
                "tilt_deg": float(candidate.get("tilt_deg", 0.0)),
                "success": bool(ik_result.get("success")),
                "error": ik_result.get("error"),
                "available": bool(ik_result.get("available")),
            }}
            if CONFIG.get("verbose_diagnostics"):
                ik_report["seeds"] = ik_result.get("seeds", [])
            print("TCP_IK_CANDIDATE_JSON " + json.dumps(ik_report, sort_keys=True))
            phase_had_ik_success = phase_had_ik_success or bool(ik_result.get("success"))
            planner.set_start_state_to_current_state()
            if candidate.get("phase") == "validated_region" and ik_result.get("success"):
                trajectory, summary = plan_joint_solution(
                    planner,
                    "transit_to_source_hover_yaw_%d" % index,
                    hover_pose,
                    ik_result,
                    raise_on_failure=False,
                )
            else:
                trajectory, summary = plan_target(
                    planner,
                    "transit_to_source_hover_yaw_%d" % index,
                    xyz,
                    quat,
                    raise_on_failure=False,
                )
            row = {{
                "index": index,
                "phase": phase_name,
                "yaw_deg": float(candidate["yaw_deg"]),
                "tilt_deg": float(candidate.get("tilt_deg", 0.0)),
                "ik_success": bool(ik_result.get("success")),
                "ik_error": ik_result.get("error"),
                "planner_success": bool(summary.get("success")),
                "planner_error": summary.get("error"),
                "trajectory_points": int(summary.get("points", 0)),
            }}
            rows.append(row)
            print("TCP_CANDIDATE_RESULT_JSON " + json.dumps(row, sort_keys=True))
            if summary.get("success") and summary.get("points", 0) > 0:
                summary["name"] = "transit_to_source_hover"
                summary["selected_yaw_candidate"] = candidate
                summary["height_plan"] = height_plan
                summary["reachability"] = reachability
                summary["ik"] = ik_result
                if candidate.get("validated_region_selection"):
                    updated_selection = dict(candidate["validated_region_selection"])
                    updated_selection["transit_result"] = {{
                        "success": bool(summary.get("success")),
                        "points": int(summary.get("points", 0)),
                        "error": summary.get("error"),
                    }}
                    print("VALIDATED_GRASP_REGION_SELECTION_JSON " + json.dumps(updated_selection, sort_keys=True))
                print("SELECTED_TCP_YAW_CANDIDATE_JSON " + json.dumps(candidate, sort_keys=True))
                print("SELECTED_TCP_HEIGHT_PLAN_JSON " + json.dumps(height_plan, sort_keys=True))
                return trajectory, summary, candidate, hover_pose
        if phase_name == "exact_top_down":
            print("TCP_EXACT_TOP_DOWN_SUMMARY_JSON " + json.dumps({{
                "candidate_count": len([row for row in rows if row["phase"] == "exact_top_down"]),
                "ik_success_count": sum(1 for row in rows if row["phase"] == "exact_top_down" and row["ik_success"]),
                "planner_success_count": sum(1 for row in rows if row["phase"] == "exact_top_down" and row["planner_success"]),
            }}, sort_keys=True))
            if any(row["planner_success"] for row in rows if row["phase"] == "exact_top_down"):
                break
            if not phase_had_ik_success:
                print("TCP_EXACT_TOP_DOWN_NO_IK_SUCCESS testing bounded tilted candidates up to 20 degrees")
    print("TCP_YAW_CANDIDATE_SUMMARY_JSON " + json.dumps({{
        "candidate_count": len(rows),
        "planner_success_count": sum(1 for row in rows if row["planner_success"]),
        "ik_success_count": sum(1 for row in rows if row["ik_success"]),
        "rows": rows,
    }}, sort_keys=True))
    raise RuntimeError("MoveIt planning failed for transit_to_source_hover for all tested TCP candidates")


def execute_trajectory(planner, name, trajectory):
    print("EXEC_STEP " + name)
    ok = bool(planner.execute(trajectory, wait=True))
    print("EXEC_RESULT %s %s" % (name, ok))
    planner.stop()
    planner.clear_pose_targets()
    if not ok:
        raise RuntimeError("Execution failed at " + name)


def compute_gripper_plan(detected_width, close_width_override):
    detected = float(detected_width)
    gripper_min = float(env.GRIPPER_MIN)
    gripper_max = float(env.GRIPPER_MAX)
    open_width = gripper_max
    saturated = detected >= gripper_max - 0.001
    if close_width_override is None:
        source = "automatic"
        close_width = None if saturated else max(gripper_min, detected - 0.005)
    else:
        source = "explicit"
        close_width = float(close_width_override)

    if close_width is not None:
        if not (gripper_min <= close_width <= gripper_max):
            raise RuntimeError(
                "close_width %.4f outside gripper range %.4f..%.4f"
                % (close_width, gripper_min, gripper_max)
            )
        if not close_width < open_width:
            raise RuntimeError(
                "close_width %.4f must be smaller than open_width %.4f"
                % (close_width, open_width)
            )

    if CONFIG["execute"] and saturated and close_width_override is None:
        raise RuntimeError(
            "Detected grasp width %.4f m is saturated at gripper max %.4f m; "
            "execute mode requires explicit --close-width"
            % (detected, gripper_max)
        )

    return {{
        "detected_width_m": detected,
        "estimate_saturated": saturated,
        "open_width_m": open_width,
        "close_width_m": close_width,
        "close_width_source": source,
        "grip_margin_m": None if close_width is None else detected - close_width,
    }}


def depth_to_m(depth_value):
    value = float(depth_value)
    if value <= 0.0:
        return None
    return value / 1000.0 if value > 20.0 else value


def deproject(u, v, depth_m, K):
    fx, fy = float(K[0, 0]), float(K[1, 1])
    cx, cy = float(K[0, 2]), float(K[1, 2])
    return [(float(u) - cx) * depth_m / fx, (float(v) - cy) * depth_m / fy, depth_m]


def transform_point(target_frame, source_frame, point_xyz):
    if grasp._tf_buffer is None:
        raise RuntimeError("TF buffer is not initialized")
    tf = grasp._tf_buffer.lookup_transform(
        target_frame,
        source_frame,
        rospy.Time(0),
        rospy.Duration(1.0),
    )
    t = tf.transform.translation
    q = tf.transform.rotation
    rot = grasp._quat_to_rot_xyzw([q.x, q.y, q.z, q.w])
    point = np.asarray(point_xyz, dtype=float).reshape(3)
    return [float(x) for x in (rot.dot(point) + np.array([t.x, t.y, t.z], dtype=float)).tolist()]


def base_to_camera_point(base_xyz):
    return transform_point(grasp._camera_frame_id, grasp._base_frame_id, base_xyz)


def topic_publishers(topic):
    try:
        _code, _message, state = rospy.get_master().getSystemState()
        for name, nodes in state[0]:
            if name == topic:
                return list(nodes)
    except Exception:
        pass
    return []


def wait_fresh_message(topic, msg_type, timeout_s=5.0, max_age_s=2.0):
    publishers = topic_publishers(topic)
    try:
        msg = rospy.wait_for_message(topic, msg_type, timeout=timeout_s)
    except Exception as exc:
        return {{
            "ok": False,
            "topic": topic,
            "error": str(exc),
            "publishers": publishers,
            "publisher_count": len(publishers),
            "age_s": None,
        }}
    stamp = getattr(getattr(msg, "header", None), "stamp", None)
    stamp_s = float(stamp.to_sec()) if stamp is not None else 0.0
    age = float((rospy.Time.now() - stamp).to_sec()) if stamp_s > 0.0 else None
    ok = bool(stamp_s > 0.0 and age is not None and age <= max_age_s)
    return {{
        "ok": ok,
        "topic": topic,
        "publishers": publishers,
        "publisher_count": len(publishers),
        "stamp": stamp_s,
        "age_s": age,
        "error": None if ok else ("missing or zero timestamp" if stamp_s <= 0.0 else "stale image"),
    }}


def check_camera_tf(camera_frame, base_frame, timeout_s=5.0):
    buffer = tf2_ros.Buffer()
    _listener = tf2_ros.TransformListener(buffer)
    deadline = time.time() + timeout_s
    last_error = None
    while time.time() < deadline and not rospy.is_shutdown():
        try:
            tf = buffer.lookup_transform(base_frame, camera_frame, rospy.Time(0), rospy.Duration(0.25))
            stamp = float(tf.header.stamp.to_sec())
            age = float((rospy.Time.now() - tf.header.stamp).to_sec()) if stamp > 0.0 else None
            return {{
                "ok": True,
                "source_frame": camera_frame,
                "target_frame": base_frame,
                "stamp": stamp,
                "age_s": age,
                "error": None,
            }}
        except Exception as exc:
            last_error = str(exc)
            time.sleep(0.05)
    return {{
        "ok": False,
        "source_frame": camera_frame,
        "target_frame": base_frame,
        "error": last_error,
    }}


def perception_preflight():
    section("PERCEPTION PREFLIGHT")
    checks = {{
        "color": wait_fresh_message(grasp._image_topic, RosImage),
        "depth": wait_fresh_message(grasp._depth_topic, RosImage),
        "camera_info": wait_fresh_message(grasp._camera_info_topic, CameraInfo),
        "tf": check_camera_tf(grasp._camera_frame_id, grasp._base_frame_id),
        "configured_topics": {{
            "color": grasp._image_topic,
            "depth": grasp._depth_topic,
            "camera_info": grasp._camera_info_topic,
            "camera_frame": grasp._camera_frame_id,
            "base_frame": grasp._base_frame_id,
        }},
        "tmux_camera_window_status": "not_checked_from_submitted_code",
    }}
    print("PERCEPTION_PREFLIGHT_JSON " + json.dumps(checks, sort_keys=True), flush=True)
    failed = [name for name, value in checks.items() if isinstance(value, dict) and value.get("ok") is False]
    if failed:
        raise RuntimeError("PERCEPTION_PREFLIGHT_FAILED " + json.dumps({{
            "failed": failed,
            "checks": checks,
        }}, sort_keys=True))
    grasp.start()
    return checks


def normalize_source(provider, label, camera_xyz, base_xyz, width_m, confidence, metadata):
    camera = finite_vec("source.camera_xyz", camera_xyz, 3)
    base = finite_vec("source.base_xyz", base_xyz, 3)
    width = float(width_m)
    if not (0.0 < width <= float(env.GRIPPER_MAX) + 1e-6):
        raise RuntimeError(
            "Invalid source width %.4f m; expected 0.0 < width <= %.4f m"
            % (width, float(env.GRIPPER_MAX))
        )
    assert_workspace("source_base", base)
    return {{
        "provider": provider,
        "label": label,
        "camera_xyz": camera,
        "base_xyz": base,
        "width_m": width,
        "confidence": float(confidence),
        "metadata": metadata or {{}},
    }}


def source_from_perception(source):
    perception_preflight()
    results = grasp.get_grasp_pose(source, top_k=CONFIG["top_k"])
    instances = [r for r in results if r.get("grasps")]
    if not instances:
        raise RuntimeError("No grasp returned for source " + source)
    for inst in instances:
        for candidate in inst.get("grasps", []):
            width = float(candidate.get("width", 0.0))
            if 0.0 < width <= float(env.GRIPPER_MAX) + 1e-6:
                return normalize_source(
                    "perception",
                    inst.get("label") or source,
                    candidate.get("translation_camera"),
                    candidate.get("translation_base"),
                    width,
                    inst.get("confidence", 0.0),
                    {{
                        "instance": inst,
                        "selected_grasp": candidate,
                        "coordinate_semantics": {{
                            "selected_base_field": "selected_grasp.translation_base",
                            "meaning": "AnyGrasp geometric grasp center transformed to base_link; not the retreat pose and not a table surface point",
                            "source": "GraspSDK.get_grasp_pose",
                        }},
                    }},
                )
    raise RuntimeError(
        "No source grasp with valid width 0.0 < width <= %.4f m for %s"
        % (float(env.GRIPPER_MAX), source)
    )


def source_from_manual():
    xyz = CONFIG["source_xyz"]
    if any(value is None for value in xyz):
        raise RuntimeError("Manual source provider requires --source-x --source-y --source-z")
    if CONFIG["source_width"] is None:
        raise RuntimeError("Manual source provider requires --source-width")
    base_xyz = finite_vec("manual_source.base_xyz", xyz, 3)
    return normalize_source(
        "manual",
        CONFIG["source"] or "manual-source",
        base_xyz,
        base_xyz,
        CONFIG["source_width"],
        1.0,
        {{"coordinate_frame": grasp._base_frame_id, "camera_xyz_note": "manual provider uses base_link coordinates"}},
    )


def source_from_aruco():
    if CONFIG["source_width"] is None:
        raise RuntimeError("ArUco source provider requires --source-width")
    marker_frame = CONFIG["aruco_frame"]
    offset = finite_vec("aruco_offset_xyz", CONFIG["aruco_offset_xyz"], 3)
    try:
        base_xyz = transform_point(grasp._base_frame_id, marker_frame, offset)
    except Exception as exc:
        raise RuntimeError("ArUco source provider failed to read TF for %s: %s" % (marker_frame, exc))
    camera_xyz = base_to_camera_point(base_xyz)
    return normalize_source(
        "aruco",
        CONFIG["source"] or marker_frame,
        camera_xyz,
        base_xyz,
        CONFIG["source_width"],
        1.0,
        {{"aruco_frame": marker_frame, "marker_to_object_offset_xyz": offset}},
    )


def select_source():
    provider = CONFIG["source_provider"]
    if provider == "perception":
        return source_from_perception(CONFIG["source"])
    if provider == "manual":
        return source_from_manual()
    if provider == "aruco":
        return source_from_aruco()
    raise RuntimeError("Unknown source provider: " + repr(provider))


class DestinationDetector:
    def detect(self, description):
        raise NotImplementedError


class PurpleRegionDetector(DestinationDetector):
    def detect(self, description):
        text = (description or "").lower()
        if "purple" not in text:
            raise RuntimeError("No destination detector available for: " + repr(description))

        img = grasp._read_color()
        depth = grasp._read_depth_u16()
        K = grasp._read_camera_K()
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower = np.array([120, 35, 35], dtype=np.uint8)
        upper = np.array([165, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [c for c in contours if cv2.contourArea(c) >= 400.0]
        if not contours:
            raise RuntimeError("Destination perception failed: no purple region found")
        contour = max(contours, key=cv2.contourArea)
        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            raise RuntimeError("Destination perception failed: purple region has zero moment")
        u = moments["m10"] / moments["m00"]
        v = moments["m01"] / moments["m00"]
        region_depth = depth[mask > 0]
        valid_depth = region_depth[region_depth > 0]
        if valid_depth.size == 0:
            raise RuntimeError("Destination perception failed: purple region has no valid depth")
        depth_m = depth_to_m(float(np.median(valid_depth)))
        if depth_m is None:
            raise RuntimeError("Destination perception failed: invalid purple region depth")
        camera_xyz = deproject(u, v, depth_m, K)
        base_xyz = [float(x) for x in grasp._cam_to_base_point(camera_xyz).tolist()]
        x, y, w, h = cv2.boundingRect(contour)
        return {{
            "kind": "color_region",
            "description": description,
            "color": "purple",
            "pixel": [float(u), float(v)],
            "xywh": [int(x), int(y), int(w), int(h)],
            "area_px": float(cv2.contourArea(contour)),
            "depth_m": float(depth_m),
            "camera_xyz": [float(x) for x in camera_xyz],
            "base_xyz": base_xyz,
        }}


def detect_destination(description):
    return PurpleRegionDetector().detect(description)


def visual_verify(source_base, destination_base):
    try:
        dets = yolo.segment_3d(CONFIG["source"])
    except Exception as exc:
        return {{"status": "uncertain", "reason": "verification perception failed: %s" % exc}}
    if not dets:
        return {{"status": "uncertain", "reason": "source not detected after execution"}}

    def dist(a, b):
        return math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))

    nearest_original = min(dist(d["position_base"], source_base) for d in dets)
    out = {{"nearest_original_m": nearest_original, "detections": dets}}
    if destination_base is not None:
        nearest_destination = min(dist(d["position_base"], destination_base) for d in dets)
        out["nearest_destination_m"] = nearest_destination
        if nearest_destination <= 0.10 and nearest_original > 0.08:
            out["status"] = "success"
        elif nearest_original <= 0.08:
            out["status"] = "failed"
        else:
            out["status"] = "uncertain"
    else:
        out["status"] = "uncertain"
    return out


clear_display_trajectory()
task_status = {{
    key: CONFIG.get(key)
    for key in (
        "task",
        "source",
        "destination",
        "plan_only",
        "perception_only",
        "execute",
        "source_provider",
        "grasp_region",
        "hover_height",
        "explore_unvalidated_candidates",
        "exploratory_planning_budget_s",
    )
}}
region_data = CONFIG.get("validated_grasp_region_data") or {{}}
task_status["validated_region_config"] = {{
    "embedded": bool(region_data.get("embedded")),
    "schema_version": region_data.get("schema_version"),
    "region_count": int(region_data.get("region_count") or 0),
    "voxel_count": int(region_data.get("voxel_count") or 0),
    "source_path": region_data.get("source_path"),
}}
if CONFIG.get("verbose_diagnostics"):
    print("PIPER_MANIPULATION_TASK_VERBOSE_JSON " + json.dumps(CONFIG, sort_keys=True))
else:
    print("PIPER_MANIPULATION_TASK_JSON " + json.dumps(task_status, sort_keys=True))
print("MOTION_PROFILE_JSON " + json.dumps({{
    "velocity_scaling": CONFIG["velocity_scaling"],
    "acceleration_scaling": CONFIG["acceleration_scaling"],
    "hardware_speed_percent": CONFIG["hardware_speed_percent"],
}}, sort_keys=True))

if not hasattr(env, "_ensure_piper_enabled"):
    raise RuntimeError("PiperRobotEnv missing enable preflight helper")

if CONFIG["task"] == "place" and CONFIG["execute"]:
    raise RuntimeError("PLACE_EXECUTION_DISABLED destination has no fully validated place region")

if CONFIG["destination_perception_only"]:
    if not CONFIG["destination"]:
        raise RuntimeError("--destination-perception-only requires --destination")
    destination = detect_destination(CONFIG["destination"])
    print("DESTINATION_DETECTION_JSON " + json.dumps(destination, sort_keys=True))
    print("destination_pixel " + json.dumps(destination["pixel"]))
    print("destination_camera_xyz " + json.dumps(destination["camera_xyz"]))
    print("destination_base_xyz " + json.dumps(destination["base_xyz"]))
    print("DESTINATION_PERCEPTION_ONLY_COMPLETE no planning or robot movement commanded")
    raise SystemExit(0)

current_pose = env.get_robot_end_pose()
if current_pose is None:
    raise RuntimeError("No current end-effector pose from /end_pose")

section("SOURCE DETECTION")
source_detection = select_source()
source_camera = source_detection["camera_xyz"]
source_base = source_detection["base_xyz"]
width = float(source_detection["width_m"])
selected_grasp = (source_detection.get("metadata") or {{}}).get("selected_grasp") or {{}}
source_summary = {{
    "label": source_detection.get("label"),
    "confidence": source_detection.get("confidence"),
    "selected_camera_xyz": source_camera,
    "selected_base_xyz": source_base,
    "selected_width": width,
    "selected_score": selected_grasp.get("score"),
    "candidate_count": len(((source_detection.get("metadata") or {{}}).get("instance") or {{}}).get("grasps", [])),
    "provider": source_detection.get("provider"),
}}
print("SOURCE_DETECTION_SUMMARY_JSON " + json.dumps(source_summary, sort_keys=True), flush=True)
print("SOURCE_COORDINATE_SEMANTICS_JSON " + json.dumps({{
    "provider": source_detection.get("provider"),
    "selected_source_base_field": "selected_grasp.translation_base" if source_detection.get("provider") == "perception" else "provider_base_xyz",
    "selected_source_meaning": ((source_detection.get("metadata") or {{}}).get("coordinate_semantics") or {{}}).get("meaning", "provider-specific base_link point"),
    "validated_voxel_field": "source_surface_xyz",
    "validated_voxel_comparison_semantics": "The stored voxel is compared against the normalized source_base_xyz used by the planner. For perception this is GraspSDK selected_grasp.translation_base, the AnyGrasp geometric grasp center in base_link.",
    "semantically_equivalent_for_comparison": source_detection.get("provider") in ("perception", "manual", "aruco"),
}}, sort_keys=True), flush=True)
gripper_plan = compute_gripper_plan(width, CONFIG["close_width"])
open_width = float(gripper_plan["open_width_m"])
close_width = gripper_plan["close_width_m"]
print("GRIPPER_PLAN_JSON " + json.dumps(gripper_plan, sort_keys=True))
if gripper_plan["estimate_saturated"] and gripper_plan["close_width_source"] == "automatic":
    print(
        "WARNING_GRIPPER_WIDTH_SATURATED detected_width_m=%.4f is at/near gripper max; "
        "plan/perception may continue but execute requires --close-width"
        % width
    )

destination = None
destination_base = None
if CONFIG["destination"]:
    destination = detect_destination(CONFIG["destination"])
    destination_base = finite_vec("destination.base_xyz", destination["base_xyz"], 3)

if CONFIG.get("verbose_diagnostics"):
    print("SOURCE_DETECTION_JSON " + json.dumps({{
        **source_detection,
        "source_camera_xyz": source_camera,
        "source_base_xyz": source_base,
    }}, sort_keys=True))
print("source_camera_xyz " + json.dumps(source_camera))
print("source_base_xyz " + json.dumps(source_base))
if destination is not None:
    print("DESTINATION_DETECTION_JSON " + json.dumps(destination, sort_keys=True))
    print("destination_camera_xyz " + json.dumps(destination["camera_xyz"]))
    print("destination_base_xyz " + json.dumps(destination["base_xyz"]))

if CONFIG["perception_only"]:
    print("PERCEPTION_ONLY_COMPLETE")
    raise SystemExit(0)

section("MOTION PLANNING")
grasp_z = source_base[2] + float(CONFIG["grasp_z_offset"])

planner = MoveGroupCommander("arm")
planner.set_end_effector_link(TCP_LINK)
selected_tcp_link = planner.get_end_effector_link()
if selected_tcp_link != TCP_LINK:
    raise RuntimeError("MoveIt selected end effector link %s, expected %s" % (selected_tcp_link, TCP_LINK))

planner.set_max_velocity_scaling_factor(float(CONFIG["velocity_scaling"]))
planner.set_max_acceleration_scaling_factor(float(CONFIG["acceleration_scaling"]))
planner.set_start_state_to_current_state()
planner.set_planning_time(10.0)
planner.set_num_planning_attempts(20)
planner.set_goal_position_tolerance(0.005)
planner.set_goal_orientation_tolerance(math.radians(5.0))

plans = {{}}
summaries = []

current_tcp_pose = planner.get_current_pose().pose
print("START_POSE_JSON " + json.dumps(pose_to_dict(pose_from_current(current_pose)), sort_keys=True))
print("CURRENT_TCP_POSE_JSON " + json.dumps(pose_to_dict(current_tcp_pose), sort_keys=True))

transit_trajectory, transit_summary, yaw_candidate, source_hover_pose = plan_transit_to_source_hover(
    planner,
    source_base,
    current_tcp_pose,
)
target_tcp_quat = yaw_candidate["quat"]
target_approach_axis = yaw_candidate["approach_axis"]
target_closing_axis = yaw_candidate["target_closing_axis"]
approach_angle_deg = yaw_candidate["approach_angle_deg"]
if approach_angle_deg > float(CONFIG.get("max_grasp_tilt", 25.0)):
    raise RuntimeError(
        "TCP approach axis is %.3f deg from planning-frame -Z, above max %.3f"
        % (approach_angle_deg, float(CONFIG.get("max_grasp_tilt", 25.0)))
    )
source_grasp_pose = pose_msg([source_base[0], source_base[1], grasp_z], target_tcp_quat)
for name, pose in [
    ("source_hover", source_hover_pose),
    ("source_grasp", source_grasp_pose),
    ("source_hover_after_grasp", source_hover_pose),
]:
    assert_workspace(name, [pose.position.x, pose.position.y, pose.position.z])

print("TARGET_TCP_QUATERNION_XYZW " + json.dumps(target_tcp_quat))
print("REQUESTED_TCP_SOURCE_HOVER_POSE_JSON " + json.dumps(pose_to_dict(source_hover_pose), sort_keys=True))
print("REQUESTED_TCP_SOURCE_GRASP_POSE_JSON " + json.dumps(pose_to_dict(source_grasp_pose), sort_keys=True))
publish_tcp_marker(planner.get_planning_frame(), source_hover_pose)
print("TOOL_FRAME_DIAGNOSTICS_JSON " + json.dumps({{
    "planning_frame": planner.get_planning_frame(),
    "move_group": "arm",
    "selected_tcp_link": selected_tcp_link,
    "link6_to_tcp_translation": LINK6_TO_TCP_TRANSLATION,
    "link6_to_tcp_quaternion": LINK6_TO_TCP_QUAT,
    "tcp_local_approach_axis": TCP_LOCAL_APPROACH_AXIS,
    "tcp_local_closing_axis": TCP_LOCAL_CLOSING_AXIS,
    "target_tcp_quaternion": target_tcp_quat,
    "target_approach_axis_in_planning_frame": target_approach_axis,
    "target_closing_axis_in_planning_frame": target_closing_axis,
    "approach_angle_to_down_deg": approach_angle_deg,
}}, sort_keys=True))
planner_diagnostics = {{
    "planning_frame": planner.get_planning_frame(),
    "pose_reference_frame": planner.get_pose_reference_frame(),
    "end_effector_link": selected_tcp_link,
    "planner_current_pose": pose_to_dict(planner.get_current_pose().pose),
    "current_joint_values": [float(x) for x in planner.get_current_joint_values()],
    "current_robot_state_satisfies_bounds": check_current_state_validity(planner),
    "source_hover_pose": pose_to_dict(source_hover_pose),
    "source_grasp_pose": pose_to_dict(source_grasp_pose),
}}
print("PLANNER_DIAGNOSTICS_JSON " + json.dumps(planner_diagnostics, sort_keys=True))

transit_final_state = start_state_from_trajectory(transit_trajectory)
plans["transit_to_source_hover"] = transit_trajectory
summaries.append(transit_summary)

source_descend_trajectory, source_descend_summary, source_descend_final_state = plan_cartesian(
    planner,
    "source_descend",
    [source_grasp_pose],
    transit_final_state,
)
plans["source_descend"] = source_descend_trajectory
summaries.append(source_descend_summary)

source_lift_trajectory, source_lift_summary, source_lift_final_state = plan_cartesian(
    planner,
    "source_lift",
    [source_hover_pose],
    source_descend_final_state,
    raise_on_failure=False,
)
if not source_lift_summary.get("success"):
    print("SOURCE_LIFT_REVERSE_FALLBACK_JSON " + json.dumps({{
        "reason": "direct_compute_cartesian_path_incomplete",
        "direct_fraction": source_lift_summary.get("fraction"),
        "direct_points": source_lift_summary.get("points"),
        "fallback": "reverse_of_validated_source_descend",
    }}, sort_keys=True))
    source_lift_trajectory, source_lift_summary, source_lift_final_state = reverse_trajectory_for_lift(
        "source_lift",
        source_descend_trajectory,
        source_hover_pose,
    )
plans["source_lift"] = source_lift_trajectory
summaries.append(source_lift_summary)
if yaw_candidate.get("validated_region_selection"):
    validated_result = dict(yaw_candidate["validated_region_selection"])
    validated_result["transit_result"] = {{
        "success": bool(transit_summary.get("success")),
        "points": int(transit_summary.get("points", 0)),
        "error": transit_summary.get("error"),
    }}
    validated_result["descend_fraction"] = float(source_descend_summary.get("fraction", 0.0))
    validated_result["lift_fraction"] = float(source_lift_summary.get("fraction", 0.0))
    print("VALIDATED_GRASP_REGION_SELECTION_JSON " + json.dumps(validated_result, sort_keys=True))

transport_name = None
destination_descend_name = None
destination_rise_name = None
if CONFIG["task"] == "place":
    if destination_base is None:
        raise RuntimeError("Place task requires a detected destination")
    dest_height_plan = tcp_height_plan(
        destination_base[2],
        float(CONFIG["hover_height"]),
        float(CONFIG["transit_z"]),
        yaw_candidate,
    )
    print("DESTINATION_TCP_HEIGHT_PLAN_JSON " + json.dumps(dest_height_plan, sort_keys=True))
    dest_hover_z = dest_height_plan["selected_tcp_hover_z"]
    place_z = destination_base[2] + float(CONFIG["place_z_offset"])
    destination_hover_pose = pose_msg([destination_base[0], destination_base[1], dest_hover_z], target_tcp_quat)
    destination_place_pose = pose_msg([destination_base[0], destination_base[1], place_z], target_tcp_quat)
    print("REQUESTED_TCP_DESTINATION_HOVER_POSE_JSON " + json.dumps(pose_to_dict(destination_hover_pose), sort_keys=True))
    print("REQUESTED_TCP_DESTINATION_PLACE_POSE_JSON " + json.dumps(pose_to_dict(destination_place_pose), sort_keys=True))
    for name, pose in [
        ("destination_hover", destination_hover_pose),
        ("destination_place", destination_place_pose),
        ("destination_hover_after_release", destination_hover_pose),
    ]:
        assert_workspace(name, [pose.position.x, pose.position.y, pose.position.z])

    transport_trajectory, transport_summary = plan_target(
        planner,
        "transport_to_destination_hover",
        [destination_hover_pose.position.x, destination_hover_pose.position.y, destination_hover_pose.position.z],
        target_tcp_quat,
        source_lift_final_state,
    )
    transport_final_state = start_state_from_trajectory(transport_trajectory)
    plans["transport_to_destination_hover"] = transport_trajectory
    summaries.append({{
        "name": "transport_to_destination_hover",
        "type": "planned_pose_target",
        "points": trajectory_point_count(transport_trajectory),
        "final_pose": pose_to_dict(destination_hover_pose),
        **transport_summary,
    }})

    destination_descend_trajectory, destination_descend_summary, destination_descend_state = plan_cartesian(
        planner,
        "destination_descend",
        [destination_place_pose],
        transport_final_state,
    )
    plans["destination_descend"] = destination_descend_trajectory
    summaries.append(destination_descend_summary)

    destination_rise_trajectory, destination_rise_summary, _destination_rise_state = plan_cartesian(
        planner,
        "destination_rise",
        [destination_hover_pose],
        destination_descend_state,
    )
    plans["destination_rise"] = destination_rise_trajectory
    summaries.append(destination_rise_summary)

print("MOVEIT_PLAN_JSON " + json.dumps(summaries, sort_keys=True))

if CONFIG["plan_only"]:
    preview_order = [
        "transit_to_source_hover",
        "source_descend",
        "source_lift",
        "transport_to_destination_hover",
        "destination_descend",
        "destination_rise",
    ]
    publish_rviz_preview(planner, [plans[name] for name in preview_order if name in plans])
    print("PLAN_ONLY_COMPLETE no robot movement commanded")
else:
    section("EXECUTION")
    env._ensure_piper_enabled()
    print("EXECUTION_START")
    print("EXEC_STEP open_gripper")
    result = env.set_gripper(open_width)
    print("EXEC_RESULT open_gripper %s" % result)
    if not result.get("success"):
        raise RuntimeError("open_gripper failed")

    execute_trajectory(planner, "transit_to_source_hover", plans["transit_to_source_hover"])
    execute_trajectory(planner, "source_descend", plans["source_descend"])

    print("EXEC_STEP close_gripper")
    result = env.set_gripper(close_width)
    print("EXEC_RESULT close_gripper %s" % result)
    if not result.get("success"):
        raise RuntimeError("close_gripper failed")

    execute_trajectory(planner, "source_lift", plans["source_lift"])

    if CONFIG["task"] == "place":
        execute_trajectory(planner, "transport_to_destination_hover", plans["transport_to_destination_hover"])
        execute_trajectory(planner, "destination_descend", plans["destination_descend"])
        print("EXEC_STEP open_gripper_release")
        result = env.set_gripper(open_width)
        print("EXEC_RESULT open_gripper_release %s" % result)
        if not result.get("success"):
            raise RuntimeError("open_gripper_release failed")
        execute_trajectory(planner, "destination_rise", plans["destination_rise"])
        print("VISUAL_VERIFICATION_JSON " + json.dumps(visual_verify(source_base, destination_base), sort_keys=True))

    print("EXECUTION_COMPLETE")
"""


def acquire_lease(agent_url: str, holder: str) -> str:
    data = http_json(
        "POST",
        agent_url.rstrip("/") + "/lease/acquire",
        {"holder": holder, "rewind_on_release": False},
    )
    lease_id = data.get("lease_id")
    if not lease_id:
        raise RuntimeError(f"Could not acquire lease: {data}")
    return lease_id


def lease_status(agent_url: str) -> Dict[str, Any]:
    return http_json("GET", agent_url.rstrip("/") + "/lease/status")


def extend_lease(agent_url: str, lease_id: str) -> Dict[str, Any]:
    return http_json("POST", agent_url.rstrip("/") + "/lease/extend", {"lease_id": lease_id})


def release_lease(agent_url: str, lease_id: str) -> None:
    try:
        data = http_json("POST", agent_url.rstrip("/") + "/lease/release", {"lease_id": lease_id})
        print("LEASE_RELEASE " + json.dumps(data, sort_keys=True))
    except Exception as exc:
        print(f"LEASE_RELEASE_FAILED {exc}", file=sys.stderr)


def render_stream_chunk(chunk: str) -> str:
    if not chunk:
        return ""
    # The full result JSON remains unchanged on disk. This is only terminal rendering.
    return chunk.replace("\\n", "\n")


def submit_and_stream(agent_url: str, lease_id: str, code: str, timeout: float, verbose_result: bool = False) -> int:
    base = agent_url.rstrip("/")
    status_before = lease_status(agent_url)
    lease_config = status_before.get("config") or {}
    max_duration = float(lease_config.get("max_duration_s", 0.0) or 0.0)
    idle_timeout = float(lease_config.get("idle_timeout_s", 0.0) or 0.0)
    duration_preflight = {
        "requested_timeout_s": float(timeout),
        "lease_max_duration_s": max_duration,
        "lease_idle_timeout_s": idle_timeout,
        "heartbeat_interval_s": HEARTBEAT_INTERVAL_S,
    }
    print("LEASE_DURATION_PREFLIGHT_JSON " + json.dumps(duration_preflight, sort_keys=True), flush=True)
    if max_duration > 0.0 and float(timeout) > max_duration:
        raise RuntimeError(
            "Requested execution timeout %.1fs exceeds lease max duration %.1fs"
            % (float(timeout), max_duration)
        )
    validation = http_json("POST", base + "/code/validate", {"code": code})
    if not validation.get("valid"):
        raise RuntimeError("Code validation failed: " + json.dumps(validation, indent=2))
    started = http_json(
        "POST",
        base + "/code/execute",
        {"code": code, "timeout": timeout},
        headers={"X-Lease-Id": lease_id},
    )
    if not started.get("success"):
        raise RuntimeError("Code execution was not accepted: " + json.dumps(started, indent=2))
    print("EXECUTION_ID " + started.get("execution_id", ""))

    stdout_offset = 0
    stderr_offset = 0
    status = "running"
    last_heartbeat = 0.0
    while status not in TERMINAL_STATUSES:
        time.sleep(1.0)
        now = time.time()
        if now - last_heartbeat >= HEARTBEAT_INTERVAL_S:
            heartbeat = extend_lease(agent_url, lease_id)
            print("LEASE_HEARTBEAT_JSON " + json.dumps(heartbeat, sort_keys=True), flush=True)
            if heartbeat.get("status") == "not_found":
                raise RuntimeError("Lease heartbeat failed: lease not found")
            last_heartbeat = now
        state = http_json(
            "GET",
            f"{base}/code/status?stdout_offset={stdout_offset}&stderr_offset={stderr_offset}",
        )
        stdout_offset = int(state.get("stdout_offset", stdout_offset))
        stderr_offset = int(state.get("stderr_offset", stderr_offset))
        if state.get("stdout"):
            print(render_stream_chunk(str(state["stdout"])), end="")
        if state.get("stderr"):
            print(render_stream_chunk(str(state["stderr"])), end="", file=sys.stderr)
        status = str(state.get("status", "unknown"))
    if status != "completed":
        final = http_json("GET", base + "/code/result")
        result = final.get("result") or {}
        result_status = str(result.get("status") or status)
        print("FINAL_STATUS " + result_status)
        if status == "idle" and result_status == "completed":
            return 0
        result_path = save_full_result(final, result.get("execution_id") or started.get("execution_id", "unknown"))
        print_failure_summary(final, result_path)
        if verbose_result:
            print("FINAL_RESULT " + json.dumps(final, indent=2, sort_keys=True))
        return 1
    print("FINAL_STATUS " + status)
    return 0


def save_full_result(final: Dict[str, Any], execution_id: str) -> str:
    log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs", "code_results")
    os.makedirs(log_dir, exist_ok=True)
    path = os.path.join(log_dir, f"{int(time.time())}_{execution_id}.json")
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(final, handle, indent=2, sort_keys=True)
    return path


def print_failure_summary(final: Dict[str, Any], result_path: str) -> None:
    result = final.get("result") or {}
    stderr = str(result.get("stderr") or "")
    traceback_lines = [line for line in stderr.splitlines() if line.strip()]
    exception_type = None
    exception_message = result.get("error") or ""
    for line in reversed(traceback_lines):
        if ":" in line and not line.startswith(" "):
            exception_type, exception_message = line.split(":", 1)
            exception_message = exception_message.strip()
            break
    summary = {
        "status": result.get("status"),
        "exit_code": result.get("exit_code"),
        "execution_id": result.get("execution_id"),
        "duration": result.get("duration"),
        "exception_type": exception_type,
        "exception_message": exception_message,
        "last_traceback_lines": traceback_lines[-8:],
    }
    print("FINAL_ERROR_SUMMARY_JSON")
    print(json.dumps(summary, indent=2, sort_keys=True))
    print("FULL_RESULT_SAVED " + result_path)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--agent-url", default="http://127.0.0.1:8888")
    parser.add_argument("--holder", default="piper-manipulation")
    parser.add_argument("--task", choices=("pick", "place"), required=True)
    parser.add_argument("--source")
    parser.add_argument("--destination")
    parser.add_argument(
        "--source-provider",
        choices=("perception", "aruco", "manual"),
        default="perception",
    )
    parser.add_argument("--source-width", type=float)
    parser.add_argument("--source-x", type=float)
    parser.add_argument("--source-y", type=float)
    parser.add_argument("--source-z", type=float)
    parser.add_argument("--close-width", type=float, help="Explicit total physical gripper opening in meters")
    parser.add_argument("--grasp-region", choices=("auto", "off"), default="auto")
    parser.add_argument(
        "--grasp-region-config",
        default="/home/dase-hw101/ABot-Claw/robot_layer/arm_piper/agent_server/config/piper_validated_grasp_regions.yaml",
    )
    parser.add_argument("--max-grasp-tilt", type=float, default=25.0)
    parser.add_argument("--visualize-candidates", action="store_true")
    parser.add_argument("--verbose-diagnostics", action="store_true")
    parser.add_argument(
        "--explore-unvalidated-candidates",
        action="store_true",
        help="Plan-only diagnostic search outside validated voxels; never weakens execute gating",
    )
    parser.add_argument("--exploratory-planning-budget", type=float, default=60.0)
    parser.add_argument("--aruco-frame", default="aruco_marker_frame")
    parser.add_argument("--aruco-offset-x", type=float, default=0.0)
    parser.add_argument("--aruco-offset-y", type=float, default=0.0)
    parser.add_argument("--aruco-offset-z", type=float, default=0.0)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--plan-only", action="store_true")
    mode.add_argument("--execute", action="store_true")
    mode.add_argument("--perception-only", action="store_true")
    mode.add_argument("--destination-perception-only", action="store_true")
    parser.add_argument("--top-k", type=int, default=5)
    parser.add_argument("--transit-z", type=float, default=0.32)
    parser.add_argument("--hover-height", type=float, default=0.10)
    parser.add_argument("--grasp-z-offset", type=float, default=0.035)
    parser.add_argument("--place-z-offset", type=float, default=0.045)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--verbose-result", action="store_true")
    parser.add_argument("--x-min", type=float, default=0.05)
    parser.add_argument("--x-max", type=float, default=0.60)
    parser.add_argument("--y-min", type=float, default=-0.35)
    parser.add_argument("--y-max", type=float, default=0.35)
    parser.add_argument("--z-min", type=float, default=0.02)
    parser.add_argument("--z-max", type=float, default=0.55)
    args = parser.parse_args()
    if args.task == "place" and not args.destination:
        parser.error("--task place requires --destination")
    if not args.destination_perception_only and not args.source:
        parser.error("--source is required unless --destination-perception-only is used")
    if args.source_provider == "manual" and not args.destination_perception_only:
        missing = [
            name
            for name, value in (
                ("--source-x", args.source_x),
                ("--source-y", args.source_y),
                ("--source-z", args.source_z),
                ("--source-width", args.source_width),
            )
            if value is None
        ]
        if missing:
            parser.error("--source-provider manual requires " + " ".join(missing))
    if args.source_provider == "aruco" and not args.destination_perception_only and args.source_width is None:
        parser.error("--source-provider aruco requires --source-width")
    if args.source_width is not None and args.source_width <= 0.0:
        parser.error("--source-width must be greater than 0")
    if args.close_width is not None and args.close_width <= 0.0:
        parser.error("--close-width must be greater than 0")
    if args.max_grasp_tilt <= 0.0 or args.max_grasp_tilt > 30.0:
        parser.error("--max-grasp-tilt must be in (0, 30]")
    if args.exploratory_planning_budget <= 0.0:
        parser.error("--exploratory-planning-budget must be greater than 0")
    if args.execute and args.explore_unvalidated_candidates:
        parser.error("--explore-unvalidated-candidates is only allowed for plan-only diagnostics")
    if args.execute:
        args.plan_only = False
    return args


def main() -> int:
    args = parse_args()
    code = build_robot_code(args)
    lease_id = ""
    try:
        lease_id = acquire_lease(args.agent_url, args.holder)
        print("LEASE_ACQUIRED " + lease_id)
        return submit_and_stream(args.agent_url, lease_id, code, args.timeout, args.verbose_result)
    finally:
        if lease_id:
            release_lease(args.agent_url, lease_id)


if __name__ == "__main__":
    raise SystemExit(main())
