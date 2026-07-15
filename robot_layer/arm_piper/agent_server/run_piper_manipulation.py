#!/usr/bin/env python3
"""Run reusable Piper manipulation tasks through the Agent Server lease path."""

from __future__ import annotations

import argparse
import json
import sys
import time
from typing import Any, Dict, Optional
from urllib import error, request


TERMINAL_STATUSES = {"completed", "failed", "timeout", "stopped", "idle"}


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


def build_robot_code(args: argparse.Namespace) -> str:
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
import time

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Point, Pose
from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest
from visualization_msgs.msg import Marker

CONFIG = {repr(cfg)}
TCP_LINK = "gripper_tcp"
LINK6_TO_TCP_TRANSLATION = [0.0, 0.0, 0.1358]
LINK6_TO_TCP_QUAT = [0.0, 0.0, 0.0, 1.0]
TCP_LOCAL_APPROACH_AXIS = [0.0, 0.0, 1.0]
TCP_LOCAL_CLOSING_AXIS = [0.0, 1.0, 0.0]
TARGET_APPROACH_AXIS = [0.0, 0.0, -1.0]
TARGET_CLOSING_AXIS = [0.0, 1.0, 0.0]


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


def compute_top_down_tcp_orientation(closing_target_axis=None):
    approach_local = unit_vec("tcp_local_approach_axis", TCP_LOCAL_APPROACH_AXIS)
    closing_local = unit_vec("tcp_local_closing_axis", TCP_LOCAL_CLOSING_AXIS)
    approach_target = unit_vec("target_approach_axis", TARGET_APPROACH_AXIS)
    closing_target = unit_vec("target_closing_axis", closing_target_axis or TARGET_CLOSING_AXIS)
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


def top_down_orientation_candidates(current_tcp_pose):
    rot = quat_to_rot_xyzw([
        current_tcp_pose.orientation.x,
        current_tcp_pose.orientation.y,
        current_tcp_pose.orientation.z,
        current_tcp_pose.orientation.w,
    ])
    current_closing = rot.dot(unit_vec("tcp_local_closing_axis", TCP_LOCAL_CLOSING_AXIS))
    projected = np.array([current_closing[0], current_closing[1], 0.0], dtype=float)
    if float(np.linalg.norm(projected)) > 1e-6:
        base_yaw = math.atan2(float(projected[1]), float(projected[0]))
    else:
        base = unit_vec("target_closing_axis", TARGET_CLOSING_AXIS)
        base_yaw = math.atan2(float(base[1]), float(base[0]))
    offsets = [0.0, math.pi / 4.0, -math.pi / 4.0, math.pi / 2.0, -math.pi / 2.0, math.pi, 3.0 * math.pi / 4.0, -3.0 * math.pi / 4.0]
    seen = set()
    candidates = []
    for offset in offsets:
        yaw = math.atan2(math.sin(base_yaw + offset), math.cos(base_yaw + offset))
        key = round(yaw, 6)
        if key in seen:
            continue
        seen.add(key)
        closing = [math.cos(yaw), math.sin(yaw), 0.0]
        quat, approach_axis, closing_axis, angle = compute_top_down_tcp_orientation(closing)
        candidates.append({{
            "yaw_rad": float(yaw),
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


def summarize_plan(name, result, raise_on_failure=True):
    if isinstance(result, tuple):
        success = bool(result[0])
        trajectory = result[1]
        planning_time = float(result[2]) if len(result) > 2 else 0.0
        error_code = moveit_error_summary(result[3]) if len(result) > 3 else ""
    else:
        trajectory = result
        points = getattr(getattr(trajectory, "joint_trajectory", None), "points", [])
        success = len(points) > 0
        planning_time = 0.0
        error_code = ""
    points = getattr(getattr(trajectory, "joint_trajectory", None), "points", [])
    count = len(points)
    print("PLAN %s success=%s points=%d planning_time=%.3f error=%s" % (
        name, success, count, planning_time, error_code
    ))
    if raise_on_failure and (not success or count == 0):
        raise RuntimeError("MoveIt planning failed for %s: error=%s points=%d" % (name, error_code, count))
    summary = {{
        "name": name,
        "type": "pose_target",
        "success": success,
        "points": count,
        "planning_time": planning_time,
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


def plan_cartesian(planner, name, waypoints, start_state=None):
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
    if fraction < 0.999:
        raise RuntimeError("Cartesian path %s incomplete: fraction=%.6f" % (name, fraction))
    if point_count == 0:
        raise RuntimeError("Cartesian path %s produced no trajectory points" % name)
    final_state = start_state_from_trajectory(trajectory)
    return trajectory, diag, final_state


def plan_target(planner, name, xyz, quat, start_state=None, raise_on_failure=True):
    assert_workspace(name, xyz)
    if start_state is not None:
        planner.set_start_state(start_state)
    planner.clear_pose_targets()
    requested_pose = pose_msg(xyz, quat)
    planner.set_pose_target(requested_pose)
    trajectory, summary = summarize_plan(name, planner.plan(), raise_on_failure=raise_on_failure)
    summary["final_requested_pose"] = pose_to_dict(requested_pose)
    print("TRAJECTORY_DIAGNOSTIC_JSON " + json.dumps(summary, sort_keys=True))
    state = start_state_from_trajectory(trajectory)
    if state is not None:
        planner.set_start_state(state)
    return trajectory, summary


def plan_transit_to_source_hover(planner, xyz, current_tcp_pose):
    failures = []
    for index, candidate in enumerate(top_down_orientation_candidates(current_tcp_pose)):
        quat = candidate["quat"]
        hover_pose = pose_msg(xyz, quat)
        assert_workspace("source_hover candidate %d" % index, xyz)
        planner.set_start_state_to_current_state()
        trajectory, summary = plan_target(
            planner,
            "transit_to_source_hover_yaw_%d" % index,
            xyz,
            quat,
            raise_on_failure=False,
        )
        if summary.get("success") and summary.get("points", 0) > 0:
            summary["name"] = "transit_to_source_hover"
            summary["selected_yaw_candidate"] = candidate
            print("SELECTED_TCP_YAW_CANDIDATE_JSON " + json.dumps(candidate, sort_keys=True))
            return trajectory, summary, candidate, hover_pose
        failures.append({{"candidate": candidate, "summary": summary}})
    print("TCP_YAW_CANDIDATE_FAILURES_JSON " + json.dumps(failures, sort_keys=True))
    raise RuntimeError("MoveIt planning failed for transit_to_source_hover for all top-down TCP yaw candidates")


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
                    {{"instance": inst, "selected_grasp": candidate}},
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
    camera_xyz = base_to_camera_point(base_xyz)
    return normalize_source(
        "manual",
        CONFIG["source"] or "manual-source",
        camera_xyz,
        base_xyz,
        CONFIG["source_width"],
        1.0,
        {{"coordinate_frame": grasp._base_frame_id}},
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


print("PIPER_MANIPULATION_TASK " + json.dumps(CONFIG, sort_keys=True))
print("MOTION_PROFILE_JSON " + json.dumps({{
    "velocity_scaling": CONFIG["velocity_scaling"],
    "acceleration_scaling": CONFIG["acceleration_scaling"],
    "hardware_speed_percent": CONFIG["hardware_speed_percent"],
}}, sort_keys=True))

if not hasattr(env, "_ensure_piper_enabled"):
    raise RuntimeError("PiperRobotEnv missing enable preflight helper")

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
current_xyz = finite_vec("current_end_effector.position", current_pose.get("position"), 3)

source_detection = select_source()
source_camera = source_detection["camera_xyz"]
source_base = source_detection["base_xyz"]
width = float(source_detection["width_m"])
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

transit_z = max(float(CONFIG["transit_z"]), current_xyz[2], source_base[2] + float(CONFIG["hover_height"]))
hover_z = max(transit_z, source_base[2] + float(CONFIG["hover_height"]))
grasp_z = source_base[2] + float(CONFIG["grasp_z_offset"])

planner = MoveGroupCommander("arm")
planner.set_end_effector_link(TCP_LINK)
selected_tcp_link = planner.get_end_effector_link()
if selected_tcp_link != TCP_LINK:
    raise RuntimeError("MoveIt selected end effector link %s, expected %s" % (selected_tcp_link, TCP_LINK))

planner.set_max_velocity_scaling_factor(float(CONFIG["velocity_scaling"]))
planner.set_max_acceleration_scaling_factor(float(CONFIG["acceleration_scaling"]))
planner.set_start_state_to_current_state()

plans = {{}}
summaries = []

current_tcp_pose = planner.get_current_pose().pose
print("START_POSE_JSON " + json.dumps(pose_to_dict(pose_from_current(current_pose)), sort_keys=True))
print("CURRENT_TCP_POSE_JSON " + json.dumps(pose_to_dict(current_tcp_pose), sort_keys=True))

transit_trajectory, transit_summary, yaw_candidate, source_hover_pose = plan_transit_to_source_hover(
    planner,
    [source_base[0], source_base[1], hover_z],
    current_tcp_pose,
)
target_tcp_quat = yaw_candidate["quat"]
target_approach_axis = yaw_candidate["approach_axis"]
target_closing_axis = yaw_candidate["target_closing_axis"]
approach_angle_deg = yaw_candidate["approach_angle_deg"]
if approach_angle_deg > 5.0:
    raise RuntimeError("TCP approach axis is %.3f deg from planning-frame -Z" % approach_angle_deg)
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
)
plans["source_lift"] = source_lift_trajectory
summaries.append(source_lift_summary)

transport_name = None
destination_descend_name = None
destination_rise_name = None
if CONFIG["task"] == "place":
    if destination_base is None:
        raise RuntimeError("Place task requires a detected destination")
    dest_hover_z = max(transit_z, destination_base[2] + float(CONFIG["hover_height"]))
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
    print("PLAN_ONLY_COMPLETE no robot movement commanded")
else:
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


def release_lease(agent_url: str, lease_id: str) -> None:
    try:
        data = http_json("POST", agent_url.rstrip("/") + "/lease/release", {"lease_id": lease_id})
        print("LEASE_RELEASE " + json.dumps(data, sort_keys=True))
    except Exception as exc:
        print(f"LEASE_RELEASE_FAILED {exc}", file=sys.stderr)


def submit_and_stream(agent_url: str, lease_id: str, code: str, timeout: float) -> int:
    base = agent_url.rstrip("/")
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
    while status not in TERMINAL_STATUSES:
        time.sleep(1.0)
        state = http_json(
            "GET",
            f"{base}/code/status?stdout_offset={stdout_offset}&stderr_offset={stderr_offset}",
        )
        stdout_offset = int(state.get("stdout_offset", stdout_offset))
        stderr_offset = int(state.get("stderr_offset", stderr_offset))
        if state.get("stdout"):
            print(state["stdout"], end="")
        if state.get("stderr"):
            print(state["stderr"], end="", file=sys.stderr)
        status = str(state.get("status", "unknown"))
    if status != "completed":
        final = http_json("GET", base + "/code/result")
        result = final.get("result") or {}
        result_status = str(result.get("status") or status)
        print("FINAL_STATUS " + result_status)
        if status == "idle" and result_status == "completed":
            return 0
        print("FINAL_RESULT " + json.dumps(final, indent=2, sort_keys=True))
        return 1
    print("FINAL_STATUS " + status)
    return 0


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
        return submit_and_stream(args.agent_url, lease_id, code, args.timeout)
    finally:
        if lease_id:
            release_lease(args.agent_url, lease_id)


if __name__ == "__main__":
    raise SystemExit(main())
