#!/usr/bin/env python3
"""Read-only Piper gripper_tcp IK and collision diagnostics.

This script only calls MoveIt state, IK, and validity services. It never sends
trajectory, gripper, enable, or stop commands.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import xml.etree.ElementTree as ET
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import moveit_commander
import numpy as np
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest


TCP_LINK = "gripper_tcp"
GROUP_NAME = "arm"
LINK6_TO_TCP_TRANSLATION = [0.0, 0.0, 0.1358]
TCP_LOCAL_APPROACH_AXIS = [0.0, 0.0, 1.0]
TCP_LOCAL_CLOSING_AXIS = [0.0, 1.0, 0.0]
TARGET_APPROACH_AXIS = [0.0, 0.0, -1.0]

DEFAULT_SOURCE_BASE_XYZ = [0.2288215, -0.0172091, 0.0897315]
DEFAULT_HOVER_HEIGHT = 0.10
DEFAULT_MINIMUM_LINK6_TRANSIT_Z = 0.32


MOVEIT_ERROR_NAMES = {
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
}


def moveit_error_summary(error_code: MoveItErrorCodes) -> str:
    value = getattr(error_code, "val", None)
    if value is None:
        return str(error_code)
    return "%s(%s)" % (MOVEIT_ERROR_NAMES.get(value, "UNKNOWN"), value)


def finite_vec(name: str, values: Sequence[float], n: int) -> List[float]:
    if values is None or len(values) != n:
        raise RuntimeError("%s missing or wrong length: %r" % (name, values))
    out = [float(v) for v in values]
    for value in out:
        if not math.isfinite(value):
            raise RuntimeError("%s contains non-finite value: %r" % (name, values))
    return out


def unit_vec(name: str, values: Sequence[float]) -> np.ndarray:
    vec = np.asarray(finite_vec(name, values, 3), dtype=float)
    norm = float(np.linalg.norm(vec))
    if norm <= 1e-9:
        raise RuntimeError(name + " has zero length")
    return vec / norm


def rot_to_quat_xyzw(rot: np.ndarray) -> List[float]:
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


def quat_to_rot_xyzw(q: Sequence[float]) -> np.ndarray:
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


def compute_tcp_orientation(approach_target_axis: Sequence[float], closing_target_axis: Sequence[float]) -> List[float]:
    approach_local = unit_vec("tcp_local_approach_axis", TCP_LOCAL_APPROACH_AXIS)
    closing_local = unit_vec("tcp_local_closing_axis", TCP_LOCAL_CLOSING_AXIS)
    approach_target = unit_vec("target_approach_axis", approach_target_axis)
    closing_target = unit_vec("target_closing_axis", closing_target_axis)
    x_local = np.cross(closing_local, approach_local)
    x_local /= np.linalg.norm(x_local)
    x_target = np.cross(closing_target, approach_target)
    x_target /= np.linalg.norm(x_target)
    local_basis = np.column_stack([x_local, closing_local, approach_local])
    target_basis = np.column_stack([x_target, closing_target, approach_target])
    return rot_to_quat_xyzw(target_basis.dot(local_basis.T))


def top_down_quat_for_yaw(yaw_deg: float) -> List[float]:
    yaw = math.radians(float(yaw_deg))
    closing = [math.cos(yaw), math.sin(yaw), 0.0]
    return compute_tcp_orientation(TARGET_APPROACH_AXIS, closing)


def pose_msg(xyz: Sequence[float], quat: Sequence[float]) -> Pose:
    pose = Pose()
    pose.position.x = float(xyz[0])
    pose.position.y = float(xyz[1])
    pose.position.z = float(xyz[2])
    pose.orientation.x = float(quat[0])
    pose.orientation.y = float(quat[1])
    pose.orientation.z = float(quat[2])
    pose.orientation.w = float(quat[3])
    return pose


def pose_to_dict(pose: Pose) -> Dict[str, List[float]]:
    return {
        "position": [float(pose.position.x), float(pose.position.y), float(pose.position.z)],
        "orientation_quat": [
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        ],
    }


def robot_state_with_group_joints(
    base_state: RobotState,
    joint_names: Sequence[str],
    joint_positions: Sequence[float],
) -> RobotState:
    state = RobotState()
    state.is_diff = False
    state.joint_state = base_state.joint_state
    name_to_index = {name: index for index, name in enumerate(state.joint_state.name)}
    positions = list(state.joint_state.position)
    for name, position in zip(joint_names, joint_positions):
        if name in name_to_index:
            positions[name_to_index[name]] = float(position)
        else:
            state.joint_state.name.append(name)
            positions.append(float(position))
    state.joint_state.position = positions
    return state


def joint_limits_from_robot_description(joint_names: Sequence[str]) -> Dict[str, Tuple[float, float]]:
    limits: Dict[str, Tuple[float, float]] = {}
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
    return limits


def clamp_joint(name: str, value: float, limits: Dict[str, Tuple[float, float]]) -> float:
    if name not in limits:
        return float(value)
    lower, upper = limits[name]
    return float(max(lower, min(upper, value)))


def seed_states(planner, base_state: RobotState, include_offsets: bool) -> List[Dict[str, object]]:
    joint_names = list(planner.get_active_joints())
    current_values = [float(x) for x in planner.get_current_joint_values()]
    limits = joint_limits_from_robot_description(joint_names)
    offsets = [[0.0] * len(joint_names)]
    if include_offsets:
        offsets.extend([
            [0.20, 0.0, 0.0, 0.0, 0.0, 0.0],
            [-0.20, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.10, -0.10, 0.0, 0.0, 0.0],
            [0.0, -0.10, 0.10, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.35],
            [0.0, 0.0, 0.0, 0.0, 0.0, -0.35],
        ])
    seeds = []
    for index, offset in enumerate(offsets):
        positions = []
        for joint_index, joint_name in enumerate(joint_names):
            delta = offset[joint_index] if joint_index < len(offset) else 0.0
            positions.append(clamp_joint(joint_name, current_values[joint_index] + delta, limits))
        seeds.append({
            "index": index,
            "label": "current" if index == 0 else "offset_%d" % index,
            "state": robot_state_with_group_joints(base_state, joint_names, positions),
            "joint_positions": positions,
        })
    return seeds


def solution_for_group(solution: RobotState, joint_names: Sequence[str]) -> List[float]:
    name_to_position = dict(zip(solution.joint_state.name, solution.joint_state.position))
    return [float(name_to_position[name]) for name in joint_names if name in name_to_position]


def joint_delta(a: Sequence[float], b: Sequence[float]) -> Optional[float]:
    if len(a) != len(b):
        return None
    return float(math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b))))


def compute_ik(
    pose: Pose,
    planner,
    base_state: RobotState,
    avoid_collisions: bool,
    include_offsets: bool = True,
    timeout_s: float = 2.0,
) -> Dict[str, object]:
    service = rospy.ServiceProxy("/compute_ik", GetPositionIK)
    planning_frame = planner.get_planning_frame()
    joint_names = list(planner.get_active_joints())
    current_joints = [float(x) for x in planner.get_current_joint_values()]
    seed_results = []
    for seed in seed_states(planner, base_state, include_offsets):
        req = GetPositionIKRequest()
        req.ik_request.group_name = GROUP_NAME
        req.ik_request.ik_link_name = TCP_LINK
        req.ik_request.robot_state = seed["state"]
        req.ik_request.avoid_collisions = bool(avoid_collisions)
        req.ik_request.timeout = rospy.Duration(float(timeout_s))
        stamped = PoseStamped()
        stamped.header.frame_id = planning_frame
        stamped.header.stamp = rospy.Time.now()
        stamped.pose = pose
        req.ik_request.pose_stamped = stamped
        resp = service(req)
        success = resp.error_code.val == MoveItErrorCodes.SUCCESS
        solution = solution_for_group(resp.solution, joint_names) if success else []
        item = {
            "seed_index": seed["index"],
            "seed_label": seed["label"],
            "success": success,
            "error": moveit_error_summary(resp.error_code),
            "solution": solution,
            "solution_delta_from_current": joint_delta(current_joints, solution) if success else None,
        }
        seed_results.append(item)
        if success:
            return {
                "success": True,
                "error": item["error"],
                "solution": solution,
                "solution_delta_from_current": item["solution_delta_from_current"],
                "seed_results": seed_results,
            }
    first_error = seed_results[0]["error"] if seed_results else "NO_SEEDS"
    return {
        "success": False,
        "error": first_error,
        "solution": [],
        "solution_delta_from_current": None,
        "seed_results": seed_results,
    }


def classify_contact(link1: str, link2: str) -> str:
    pair = " ".join([link1, link2]).lower()
    if "table" in pair or "camera" in pair or "collision_object" in pair:
        return "table/environment collision"
    if "gripper" in pair or "finger" in pair or "link7" in pair or "link8" in pair:
        return "gripper/finger collision"
    if "base" in pair or "dummy" in pair:
        return "base/arm collision"
    return "self collision"


def check_state_validity(
    robot_state: RobotState,
    group_name: str = GROUP_NAME,
    contacts: bool = True,
) -> Dict[str, object]:
    service = rospy.ServiceProxy("/check_state_validity", GetStateValidity)
    req = GetStateValidityRequest()
    req.robot_state = robot_state
    req.group_name = group_name
    if hasattr(req, "contacts"):
        req.contacts = contacts
    resp = service(req)
    contact_rows = []
    for contact in getattr(resp, "contacts", []):
        link1 = str(getattr(contact, "contact_body_1", ""))
        link2 = str(getattr(contact, "contact_body_2", ""))
        contact_rows.append({
            "link1": link1,
            "link2": link2,
            "classification": classify_contact(link1, link2),
        })
    return {
        "valid": bool(resp.valid),
        "contact_count": len(contact_rows),
        "contacts": contact_rows,
    }


def state_from_solution(planner, base_state: RobotState, solution: Sequence[float]) -> RobotState:
    return robot_state_with_group_joints(base_state, list(planner.get_active_joints()), solution)


def tcp_hover_xyz(source_xyz: Sequence[float], hover_height: float, minimum_link6_transit_z: float) -> Dict[str, object]:
    quat = top_down_quat_for_yaw(0.0)
    rot = quat_to_rot_xyzw(quat)
    world_link6_to_tcp = rot.dot(np.asarray(LINK6_TO_TCP_TRANSLATION, dtype=float))
    object_clearance_z = float(source_xyz[2]) + float(hover_height)
    link6_clearance_z = float(minimum_link6_transit_z) + float(world_link6_to_tcp[2])
    selected_z = max(object_clearance_z, link6_clearance_z)
    return {
        "source_surface_z": float(source_xyz[2]),
        "hover_height": float(hover_height),
        "minimum_link6_transit_z": float(minimum_link6_transit_z),
        "tcp_z_from_object_clearance": object_clearance_z,
        "tcp_z_from_link6_clearance": link6_clearance_z,
        "selected_tcp_hover_z": selected_z,
        "world_link6_to_tcp_translation": [float(x) for x in world_link6_to_tcp.tolist()],
        "tcp_xyz": [float(source_xyz[0]), float(source_xyz[1]), selected_z],
    }


def compact_target_row(
    tcp_xyz: Sequence[float],
    orientation_type: str,
    yaw_deg: Optional[float],
    avoid_collisions: bool,
    result: Dict[str, object],
) -> Dict[str, object]:
    return {
        "tcp_xyz": [float(x) for x in tcp_xyz],
        "orientation_type": orientation_type,
        "yaw_deg": yaw_deg,
        "avoid_collisions": bool(avoid_collisions),
        "success": bool(result["success"]),
        "error": result["error"],
        "joint_solution": result["solution"],
    }


def print_json(prefix: str, payload: Dict[str, object]) -> None:
    print(prefix + " " + json.dumps(payload, sort_keys=True), flush=True)


def feasibility_map(planner, base_state: RobotState) -> Dict[str, object]:
    xs = [0.22, 0.26, 0.30, 0.34, 0.38, 0.42]
    ys = [-0.15, 0.0, 0.15]
    zs = [0.18, 0.22, 0.26]
    yaws = [0.0, 45.0, 90.0, 135.0]
    off_successes = []
    on_successes = []
    print("FEASIBILITY_MAP_COLLISION_OFF", flush=True)
    for z in zs:
        print("z=%.2f" % z, flush=True)
        for y in ys:
            cells = []
            for x in xs:
                yaw_hits = []
                first_solution = None
                for yaw in yaws:
                    pose = pose_msg([x, y, z], top_down_quat_for_yaw(yaw))
                    result = compute_ik(
                        pose,
                        planner,
                        base_state,
                        avoid_collisions=False,
                        include_offsets=False,
                        timeout_s=0.3,
                    )
                    if result["success"]:
                        yaw_hits.append(int(yaw))
                        if first_solution is None:
                            first_solution = result["solution"]
                if yaw_hits:
                    off_successes.append({"xyz": [x, y, z], "yaws": yaw_hits, "solution": first_solution})
                    cells.append("+%s" % "/".join(str(v) for v in yaw_hits))
                else:
                    cells.append("--")
            print("  y=% .2f  %s" % (y, " ".join("%8s" % cell for cell in cells)), flush=True)
    print("FEASIBILITY_MAP_COLLISION_ON_FOR_OFF_SUCCESSES", flush=True)
    for item in off_successes:
        valid_yaws = []
        for yaw in item["yaws"]:
            pose = pose_msg(item["xyz"], top_down_quat_for_yaw(yaw))
            result = compute_ik(
                pose,
                planner,
                base_state,
                avoid_collisions=True,
                include_offsets=False,
                timeout_s=0.3,
            )
            if result["success"]:
                valid_yaws.append(yaw)
        if valid_yaws:
            on_successes.append({"xyz": item["xyz"], "yaws": valid_yaws})
        print("  xyz=%s collision_on_yaws=%s" % (
            ["%.2f" % v for v in item["xyz"]],
            valid_yaws if valid_yaws else "none",
        ), flush=True)
    return {
        "collision_off_success_count": len(off_successes),
        "collision_on_success_count": len(on_successes),
        "collision_off_successes": off_successes,
        "collision_on_successes": on_successes,
    }


def recommended_region(feasibility: Dict[str, object]) -> str:
    successes = feasibility.get("collision_on_successes", [])
    if not successes:
        successes = feasibility.get("collision_off_successes", [])
    if not successes:
        return "No tested top-down TCP hover region was IK-feasible."
    xs = [float(item["xyz"][0]) for item in successes]
    ys = [float(item["xyz"][1]) for item in successes]
    zs = [float(item["xyz"][2]) for item in successes]
    return "x %.2f..%.2f, y %.2f..%.2f, tcp_hover_z %.2f..%.2f" % (
        min(xs),
        max(xs),
        min(ys),
        max(ys),
        min(zs),
        max(zs),
    )


def conclusion(
    self_off: Dict[str, object],
    self_on: Dict[str, object],
    target_current_off: Dict[str, object],
    target_topdown_off: Dict[str, object],
    target_topdown_on: Dict[str, object],
    feasibility: Optional[Dict[str, object]],
) -> str:
    if not self_off["success"]:
        return "A. IK chain/request is broken"
    if self_off["success"] and not self_on["success"]:
        return "D. collision checking is the blocker"
    if target_current_off["success"] and not target_topdown_off["success"]:
        return "C. top-down orientation is infeasible"
    if target_topdown_off["success"] and not target_topdown_on["success"]:
        return "D. collision checking is the blocker"
    if not target_current_off["success"] and not target_topdown_off["success"]:
        if feasibility and feasibility.get("collision_off_success_count", 0) > 0:
            return "B. target XYZ is kinematically infeasible"
        return "B. target XYZ is kinematically infeasible"
    if feasibility and feasibility.get("collision_off_success_count", 0) > 0 and feasibility.get("collision_on_success_count", 0) == 0:
        return "E. combination of target placement and collision constraints"
    return "E. combination of target placement and collision constraints"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-x", type=float, default=DEFAULT_SOURCE_BASE_XYZ[0])
    parser.add_argument("--source-y", type=float, default=DEFAULT_SOURCE_BASE_XYZ[1])
    parser.add_argument("--source-z", type=float, default=DEFAULT_SOURCE_BASE_XYZ[2])
    parser.add_argument("--hover-height", type=float, default=DEFAULT_HOVER_HEIGHT)
    parser.add_argument("--minimum-link6-transit-z", type=float, default=DEFAULT_MINIMUM_LINK6_TRANSIT_Z)
    args = parser.parse_args()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("diagnose_piper_tcp_ik", anonymous=True, disable_signals=True)
    rospy.wait_for_service("/compute_ik", timeout=10.0)
    rospy.wait_for_service("/check_state_validity", timeout=10.0)

    planner = moveit_commander.MoveGroupCommander(GROUP_NAME)
    planner.set_end_effector_link(TCP_LINK)
    base_state = planner.get_current_state()
    current_joints = [float(x) for x in planner.get_current_joint_values()]
    current_pose = planner.get_current_pose(TCP_LINK).pose
    current_quat = pose_to_dict(current_pose)["orientation_quat"]

    print_json("DIAGNOSTIC_CONTEXT_JSON", {
        "planning_frame": planner.get_planning_frame(),
        "pose_reference_frame": planner.get_pose_reference_frame(),
        "selected_tcp_link": planner.get_end_effector_link(),
        "active_joints": list(planner.get_active_joints()),
    })

    self_collision_off = compute_ik(current_pose, planner, base_state, avoid_collisions=False, include_offsets=False)
    self_collision_on = compute_ik(current_pose, planner, base_state, avoid_collisions=True, include_offsets=False)
    print_json("IK_SELF_TEST_JSON", {
        "current_joint_positions": current_joints,
        "current_tcp_pose": pose_to_dict(current_pose),
        "collision_off": {
            "success": self_collision_off["success"],
            "error": self_collision_off["error"],
            "solution": self_collision_off["solution"],
            "solution_delta_from_current": self_collision_off["solution_delta_from_current"],
        },
        "collision_on": {
            "success": self_collision_on["success"],
            "error": self_collision_on["error"],
            "solution": self_collision_on["solution"],
            "solution_delta_from_current": self_collision_on["solution_delta_from_current"],
        },
    })

    current_validity = check_state_validity(base_state)
    print_json("CURRENT_STATE_VALIDITY_JSON", current_validity)

    source_xyz = [args.source_x, args.source_y, args.source_z]
    hover = tcp_hover_xyz(source_xyz, args.hover_height, args.minimum_link6_transit_z)
    print_json("TARGET_TCP_HEIGHT_JSON", hover)
    target_xyz = hover["tcp_xyz"]

    target_current_off = compute_ik(
        pose_msg(target_xyz, current_quat),
        planner,
        base_state,
        avoid_collisions=False,
        include_offsets=True,
    )
    target_current_on = compute_ik(
        pose_msg(target_xyz, current_quat),
        planner,
        base_state,
        avoid_collisions=True,
        include_offsets=True,
    )
    for avoid, result in [(False, target_current_off), (True, target_current_on)]:
        print_json("TARGET_IK_COMPARISON_JSON", compact_target_row(
            target_xyz,
            "current_tcp_orientation",
            None,
            avoid,
            result,
        ))

    topdown_yaw0_pose = pose_msg(target_xyz, top_down_quat_for_yaw(0.0))
    target_topdown_off = compute_ik(topdown_yaw0_pose, planner, base_state, avoid_collisions=False, include_offsets=True)
    target_topdown_on = compute_ik(topdown_yaw0_pose, planner, base_state, avoid_collisions=True, include_offsets=True)
    for avoid, result in [(False, target_topdown_off), (True, target_topdown_on)]:
        print_json("TARGET_IK_COMPARISON_JSON", compact_target_row(
            target_xyz,
            "exact_top_down",
            0.0,
            avoid,
            result,
        ))

    if target_topdown_off["success"] and not target_topdown_on["success"]:
        validity = check_state_validity(state_from_solution(planner, base_state, target_topdown_off["solution"]))
        print_json("COLLISION_OFF_SOLUTION_VALIDITY_JSON", validity)

    for yaw in [45.0, 90.0, 135.0, 180.0, -45.0, -90.0, -135.0]:
        pose = pose_msg(target_xyz, top_down_quat_for_yaw(yaw))
        for avoid in [False, True]:
            result = compute_ik(pose, planner, base_state, avoid_collisions=avoid, include_offsets=True)
            print_json("TARGET_IK_COMPARISON_JSON", compact_target_row(
                target_xyz,
                "exact_top_down",
                yaw,
                avoid,
                result,
            ))
            if not avoid and result["success"]:
                on_result = compute_ik(pose, planner, base_state, avoid_collisions=True, include_offsets=True)
                if not on_result["success"]:
                    validity = check_state_validity(state_from_solution(planner, base_state, result["solution"]))
                    print_json("COLLISION_OFF_SOLUTION_VALIDITY_JSON", {
                        "yaw_deg": yaw,
                        **validity,
                    })

    feasibility = None
    if self_collision_off["success"]:
        feasibility = feasibility_map(planner, base_state)
        print_json("FEASIBILITY_MAP_SUMMARY_JSON", {
            "collision_off_success_count": feasibility["collision_off_success_count"],
            "collision_on_success_count": feasibility["collision_on_success_count"],
            "recommended_region": recommended_region(feasibility),
        })
    else:
        print("FEASIBILITY_MAP_SKIPPED FK_TO_IK_SELF_TEST_FAILED", flush=True)

    final_conclusion = conclusion(
        self_collision_off,
        self_collision_on,
        target_current_off,
        target_topdown_off,
        target_topdown_on,
        feasibility,
    )
    print_json("DIAGNOSTIC_CONCLUSION_JSON", {
        "conclusion": final_conclusion,
        "recommended_cup_placement_region": recommended_region(feasibility) if feasibility else "not evaluated",
        "hardware_commanded": False,
    })
    print("NO_HARDWARE_COMMANDS_ISSUED", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
