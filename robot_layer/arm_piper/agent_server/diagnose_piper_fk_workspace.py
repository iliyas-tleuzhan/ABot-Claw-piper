#!/usr/bin/env python3
"""Read-only Piper joint-space FK workspace diagnostic.

This script samples joint states, checks them with MoveIt, computes FK for
gripper_tcp, and uses known-valid FK states as IK seeds. It never sends
trajectory, gripper, enable, or hardware commands.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import sys
import time
import xml.etree.ElementTree as ET
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import moveit_commander
import numpy as np
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest
from std_msgs.msg import Header


GROUP_NAME = "arm"
TCP_LINK = "gripper_tcp"
TCP_LOCAL_APPROACH_AXIS = [0.0, 0.0, 1.0]
TCP_LOCAL_CLOSING_AXIS = [0.0, 1.0, 0.0]
TARGET_DOWN_AXIS = np.array([0.0, 0.0, -1.0], dtype=float)

DEFAULT_CUP_SURFACE_XYZ = [0.2288215, -0.0172091, 0.0897315]
DEFAULT_CUP_HOVER_XYZ = [0.2288215, -0.0172091, 0.1897315]
TABLETOP_BOUNDS = {
    "x": (0.15, 0.55),
    "y": (-0.30, 0.30),
    "z": (0.14, 0.35),
}

MOVEIT_ERROR_NAMES = {
    MoveItErrorCodes.SUCCESS: "SUCCESS",
    MoveItErrorCodes.FAILURE: "FAILURE",
    MoveItErrorCodes.PLANNING_FAILED: "PLANNING_FAILED",
    MoveItErrorCodes.INVALID_MOTION_PLAN: "INVALID_MOTION_PLAN",
    MoveItErrorCodes.TIMED_OUT: "TIMED_OUT",
    MoveItErrorCodes.START_STATE_IN_COLLISION: "START_STATE_IN_COLLISION",
    MoveItErrorCodes.GOAL_IN_COLLISION: "GOAL_IN_COLLISION",
    MoveItErrorCodes.INVALID_GROUP_NAME: "INVALID_GROUP_NAME",
    MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS: "INVALID_GOAL_CONSTRAINTS",
    MoveItErrorCodes.INVALID_ROBOT_STATE: "INVALID_ROBOT_STATE",
    MoveItErrorCodes.INVALID_LINK_NAME: "INVALID_LINK_NAME",
    MoveItErrorCodes.FRAME_TRANSFORM_FAILURE: "FRAME_TRANSFORM_FAILURE",
    MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE: "COLLISION_CHECKING_UNAVAILABLE",
    MoveItErrorCodes.NO_IK_SOLUTION: "NO_IK_SOLUTION",
}


def print_json(prefix: str, payload: Dict[str, object]) -> None:
    print(prefix + " " + json.dumps(payload, sort_keys=True), flush=True)


def moveit_error_summary(error_code: MoveItErrorCodes) -> str:
    value = getattr(error_code, "val", None)
    if value is None:
        return str(error_code)
    return "%s(%s)" % (MOVEIT_ERROR_NAMES.get(value, "UNKNOWN"), value)


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


def pose_from_xyz_quat(xyz: Sequence[float], quat: Sequence[float]) -> Pose:
    pose = Pose()
    pose.position.x = float(xyz[0])
    pose.position.y = float(xyz[1])
    pose.position.z = float(xyz[2])
    pose.orientation.x = float(quat[0])
    pose.orientation.y = float(quat[1])
    pose.orientation.z = float(quat[2])
    pose.orientation.w = float(quat[3])
    return pose


def unit(values: Sequence[float]) -> np.ndarray:
    vec = np.asarray(values, dtype=float)
    norm = float(np.linalg.norm(vec))
    if norm <= 1e-9:
        raise RuntimeError("zero vector")
    return vec / norm


def orientation_with_approach_blend(current_quat: Sequence[float], blend_to_down: float) -> List[float]:
    rot = quat_to_rot_xyzw(current_quat)
    current_approach = unit(rot.dot(unit(TCP_LOCAL_APPROACH_AXIS)))
    closing = unit(rot.dot(unit(TCP_LOCAL_CLOSING_AXIS)))
    target_approach = unit((1.0 - blend_to_down) * current_approach + blend_to_down * TARGET_DOWN_AXIS)
    adjusted_closing = closing - target_approach * float(np.dot(closing, target_approach))
    if float(np.linalg.norm(adjusted_closing)) <= 1e-6:
        adjusted_closing = np.array([0.0, 1.0, 0.0], dtype=float)
    adjusted_closing = unit(adjusted_closing)
    x_target = unit(np.cross(adjusted_closing, target_approach))
    local_approach = unit(TCP_LOCAL_APPROACH_AXIS)
    local_closing = unit(TCP_LOCAL_CLOSING_AXIS)
    x_local = unit(np.cross(local_closing, local_approach))
    local_basis = np.column_stack([x_local, local_closing, local_approach])
    target_basis = np.column_stack([x_target, adjusted_closing, target_approach])
    return rot_to_quat_xyzw(target_basis.dot(local_basis.T))


def joint_limits_from_robot_description(joint_names: Sequence[str]) -> Dict[str, Tuple[float, float]]:
    root = ET.fromstring(rospy.get_param("/robot_description"))
    limits: Dict[str, Tuple[float, float]] = {}
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
    missing = [name for name in joint_names if name not in limits]
    if missing:
        raise RuntimeError("missing URDF limits for " + ", ".join(missing))
    return limits


def inside_limits(joints: Sequence[float], joint_names: Sequence[str], limits: Dict[str, Tuple[float, float]]) -> bool:
    return all(limits[name][0] <= float(value) <= limits[name][1] for name, value in zip(joint_names, joints))


def joint_limit_margin(joints: Sequence[float], joint_names: Sequence[str], limits: Dict[str, Tuple[float, float]]) -> float:
    margins = []
    for name, value in zip(joint_names, joints):
        lower, upper = limits[name]
        margins.append(min(float(value) - lower, upper - float(value)))
    return float(min(margins)) if margins else 0.0


def joint_distance(a: Sequence[float], b: Sequence[float]) -> float:
    return float(math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b))))


def robot_state_with_joints(base_state: RobotState, joint_names: Sequence[str], joints: Sequence[float]) -> RobotState:
    state = copy.deepcopy(base_state)
    name_to_index = {name: index for index, name in enumerate(state.joint_state.name)}
    positions = list(state.joint_state.position)
    for name, value in zip(joint_names, joints):
        if name in name_to_index:
            positions[name_to_index[name]] = float(value)
        else:
            state.joint_state.name.append(name)
            positions.append(float(value))
    state.joint_state.position = positions
    state.is_diff = False
    return state


def halton(index: int, base: int) -> float:
    result = 0.0
    f = 1.0 / float(base)
    i = index
    while i > 0:
        result += f * (i % base)
        i //= base
        f /= float(base)
    return result


def halton_joint_samples(
    count: int,
    joint_names: Sequence[str],
    limits: Dict[str, Tuple[float, float]],
) -> Iterable[List[float]]:
    primes = [2, 3, 5, 7, 11, 13]
    for index in range(1, count + 1):
        joints = []
        for dim, name in enumerate(joint_names):
            lower, upper = limits[name]
            value = lower + halton(index, primes[dim]) * (upper - lower)
            joints.append(float(value))
        yield joints


def structured_samples(
    current: Sequence[float],
    joint_names: Sequence[str],
    limits: Dict[str, Tuple[float, float]],
    named_states: Sequence[Sequence[float]],
) -> Iterable[List[float]]:
    yielded = set()

    def emit(values: Sequence[float]):
        key = tuple(round(float(v), 8) for v in values)
        if key in yielded:
            return None
        yielded.add(key)
        return [float(v) for v in values]

    midpoint = [(limits[name][0] + limits[name][1]) * 0.5 for name in joint_names]
    for values in [current, midpoint, *named_states]:
        sample = emit(values)
        if sample is not None:
            yield sample

    fractions = [0.2, 0.5, 0.8]
    for joint_index, name in enumerate(joint_names):
        for fraction in fractions:
            values = list(midpoint)
            lower, upper = limits[name]
            values[joint_index] = lower + fraction * (upper - lower)
            sample = emit(values)
            if sample is not None:
                yield sample

    for j1_fraction in [0.25, 0.5, 0.75]:
        for j2_fraction in [0.25, 0.5, 0.75]:
            values = list(midpoint)
            for joint_index, fraction in [(0, j1_fraction), (1, j2_fraction)]:
                name = joint_names[joint_index]
                lower, upper = limits[name]
                values[joint_index] = lower + fraction * (upper - lower)
            sample = emit(values)
            if sample is not None:
                yield sample


class MoveItServices:
    def __init__(self, planning_frame: str):
        self.planning_frame = planning_frame
        self.validity = rospy.ServiceProxy("/check_state_validity", GetStateValidity, persistent=True)
        self.fk = rospy.ServiceProxy("/compute_fk", GetPositionFK, persistent=True)
        self.ik = rospy.ServiceProxy("/compute_ik", GetPositionIK, persistent=True)

    def check_state(self, state: RobotState) -> bool:
        req = GetStateValidityRequest()
        req.robot_state = state
        req.group_name = GROUP_NAME
        resp = self.validity(req)
        return bool(resp.valid)

    def compute_fk(self, state: RobotState) -> Optional[Pose]:
        req = GetPositionFKRequest()
        req.header = Header()
        req.header.frame_id = self.planning_frame
        req.header.stamp = rospy.Time.now()
        req.fk_link_names = [TCP_LINK]
        req.robot_state = state
        resp = self.fk(req)
        if resp.error_code.val != MoveItErrorCodes.SUCCESS or not resp.pose_stamped:
            return None
        return resp.pose_stamped[0].pose

    def compute_ik(self, pose: Pose, seed_state: RobotState, avoid_collisions: bool = True) -> Dict[str, object]:
        req = GetPositionIKRequest()
        req.ik_request.group_name = GROUP_NAME
        req.ik_request.ik_link_name = TCP_LINK
        req.ik_request.robot_state = seed_state
        req.ik_request.avoid_collisions = bool(avoid_collisions)
        req.ik_request.timeout = rospy.Duration(1.0)
        stamped = PoseStamped()
        stamped.header.frame_id = self.planning_frame
        stamped.header.stamp = rospy.Time.now()
        stamped.pose = pose
        req.ik_request.pose_stamped = stamped
        resp = self.ik(req)
        return {
            "success": resp.error_code.val == MoveItErrorCodes.SUCCESS,
            "error": moveit_error_summary(resp.error_code),
            "solution_state": resp.solution,
        }


def tabletop_inside(xyz: Sequence[float]) -> bool:
    return (
        TABLETOP_BOUNDS["x"][0] <= xyz[0] <= TABLETOP_BOUNDS["x"][1]
        and TABLETOP_BOUNDS["y"][0] <= xyz[1] <= TABLETOP_BOUNDS["y"][1]
        and TABLETOP_BOUNDS["z"][0] <= xyz[2] <= TABLETOP_BOUNDS["z"][1]
    )


def candidate_from_fk(
    pose: Pose,
    joints: Sequence[float],
    current_joints: Sequence[float],
    joint_names: Sequence[str],
    limits: Dict[str, Tuple[float, float]],
) -> Dict[str, object]:
    pose_dict = pose_to_dict(pose)
    xyz = pose_dict["position"]
    quat = pose_dict["orientation_quat"]
    rot = quat_to_rot_xyzw(quat)
    approach = rot.dot(unit(TCP_LOCAL_APPROACH_AXIS))
    closing = rot.dot(unit(TCP_LOCAL_CLOSING_AXIS))
    dot = max(-1.0, min(1.0, float(np.dot(approach, TARGET_DOWN_AXIS))))
    angle = math.degrees(math.acos(dot))
    return {
        "joint_positions": [float(x) for x in joints],
        "tcp_xyz": [float(x) for x in xyz],
        "tcp_quaternion": [float(x) for x in quat],
        "approach_axis": [float(x) for x in approach.tolist()],
        "approach_angle_to_down_deg": float(angle),
        "closing_axis": [float(x) for x in closing.tolist()],
        "minimum_joint_limit_margin_rad": joint_limit_margin(joints, joint_names, limits),
        "joint_distance_from_current": joint_distance(joints, current_joints),
        "inside_tabletop_workspace": tabletop_inside(xyz),
    }


def solution_joints(solution_state: RobotState, joint_names: Sequence[str]) -> List[float]:
    positions = dict(zip(solution_state.joint_state.name, solution_state.joint_state.position))
    return [float(positions[name]) for name in joint_names if name in positions]


def nearest_candidate(candidates: Sequence[Dict[str, object]], target_xyz: Sequence[float], max_angle: Optional[float]) -> Optional[Dict[str, object]]:
    filtered = []
    for candidate in candidates:
        if max_angle is not None and candidate["approach_angle_to_down_deg"] > max_angle:
            continue
        xyz = candidate["tcp_xyz"]
        dist = float(np.linalg.norm(np.asarray(xyz, dtype=float) - np.asarray(target_xyz, dtype=float)))
        filtered.append((dist, candidate))
    if not filtered:
        return None
    filtered.sort(key=lambda item: item[0])
    candidate = dict(filtered[0][1])
    candidate["position_distance_to_cup_hover_m"] = filtered[0][0]
    candidate["collision_state"] = "collision_free"
    return candidate


def rank_key(candidate: Dict[str, object]) -> Tuple[float, int, float, float]:
    return (
        float(candidate["approach_angle_to_down_deg"]),
        0 if candidate["inside_tabletop_workspace"] else 1,
        -float(candidate["minimum_joint_limit_margin_rad"]),
        float(candidate["joint_distance_from_current"]),
    )


def run_fk_seeded_ik_tests(
    services: MoveItServices,
    base_state: RobotState,
    best_candidates: Sequence[Dict[str, object]],
    cup_hover_xyz: Sequence[float],
    joint_names: Sequence[str],
) -> None:
    for candidate in best_candidates[:5]:
        seed_joints = candidate["joint_positions"]
        seed_state = robot_state_with_joints(base_state, joint_names, seed_joints)
        seed_xyz = np.asarray(candidate["tcp_xyz"], dtype=float)
        target_xyz = np.asarray(cup_hover_xyz, dtype=float)
        delta = target_xyz - seed_xyz
        distance = float(np.linalg.norm(delta))
        if distance <= 1e-9:
            directions = [0.0]
        else:
            max_steps = min(10, int(math.floor(distance / 0.01)))
            directions = [min(distance, i * 0.01) for i in range(0, max_steps + 1)]
            if distance not in directions:
                directions.append(distance)
        direction = delta / distance if distance > 1e-9 else np.zeros(3)
        for shift in directions:
            xyz = seed_xyz + direction * shift
            pose = pose_from_xyz_quat(xyz, candidate["tcp_quaternion"])
            result = services.compute_ik(pose, seed_state, avoid_collisions=True)
            row = {
                "seed_joint_positions": seed_joints,
                "requested_tcp_pose": pose_to_dict(pose),
                "position_shift_from_seed_fk_m": float(shift),
                "success": bool(result["success"]),
                "error": result["error"],
                "avoid_collisions": True,
            }
            if result["success"]:
                row["joint_solution"] = solution_joints(result["solution_state"], joint_names)
            print_json("FK_SEEDED_IK_TEST_JSON", row)
            if not result["success"] and shift > 0.0:
                break
        for blend in [0.25, 0.5, 0.75, 1.0]:
            pose = pose_from_xyz_quat(seed_xyz, orientation_with_approach_blend(candidate["tcp_quaternion"], blend))
            result = services.compute_ik(pose, seed_state, avoid_collisions=True)
            row = {
                "seed_joint_positions": seed_joints,
                "requested_tcp_pose": pose_to_dict(pose),
                "position_shift_from_seed_fk_m": 0.0,
                "downward_orientation_blend": blend,
                "success": bool(result["success"]),
                "error": result["error"],
                "avoid_collisions": True,
            }
            if result["success"]:
                row["joint_solution"] = solution_joints(result["solution_state"], joint_names)
            print_json("FK_SEEDED_IK_TEST_JSON", row)
            if not result["success"]:
                break


def conclusion(candidates: Sequence[Dict[str, object]], workspace_candidates: Sequence[Dict[str, object]]) -> str:
    if not candidates:
        return "E. The TCP axis convention is inconsistent with the robot's physically reachable gripper orientations."
    min_angle = min(float(c["approach_angle_to_down_deg"]) for c in candidates)
    exact = [c for c in candidates if c["approach_angle_to_down_deg"] <= 5.0]
    exact_workspace = [c for c in workspace_candidates if c["approach_angle_to_down_deg"] <= 5.0]
    if exact_workspace:
        return "B. Exact top-down exists, but only in a specific XYZ region."
    if exact:
        return "B. Exact top-down exists, but only in a specific XYZ region."
    if min_angle <= 45.0:
        return "C. Near-top-down is possible, with a minimum feasible approach angle of %.3f deg." % min_angle
    return "A. Exact top-down orientation is absent from the Piper model's reachable joint space."


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--halton-samples", type=int, default=20000)
    parser.add_argument("--progress-every", type=int, default=100)
    parser.add_argument("--max-runtime-s", type=float, default=0.0, help="Optional bounded diagnostic run; 0 means complete all samples")
    parser.add_argument("--cup-x", type=float, default=DEFAULT_CUP_SURFACE_XYZ[0])
    parser.add_argument("--cup-y", type=float, default=DEFAULT_CUP_SURFACE_XYZ[1])
    parser.add_argument("--cup-z", type=float, default=DEFAULT_CUP_SURFACE_XYZ[2])
    parser.add_argument("--cup-hover-z", type=float, default=DEFAULT_CUP_HOVER_XYZ[2])
    args = parser.parse_args()

    if args.halton_samples < 20000:
        raise RuntimeError("--halton-samples must be at least 20000")

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("diagnose_piper_fk_workspace", anonymous=True, disable_signals=True)
    rospy.wait_for_service("/check_state_validity", timeout=10.0)
    rospy.wait_for_service("/compute_fk", timeout=10.0)
    rospy.wait_for_service("/compute_ik", timeout=10.0)

    planner = moveit_commander.MoveGroupCommander(GROUP_NAME)
    planner.set_end_effector_link(TCP_LINK)
    planning_frame = planner.get_planning_frame()
    services = MoveItServices(planning_frame)
    base_state = planner.get_current_state()
    joint_names = list(planner.get_active_joints())
    current_joints = [float(x) for x in planner.get_current_joint_values()]
    limits = joint_limits_from_robot_description(joint_names)
    named_states = []
    named_state_names = []
    for name in planner.get_named_targets():
        values = planner.get_named_target_values(name)
        if values:
            named_states.append([float(values[joint]) for joint in joint_names])
            named_state_names.append(name)

    print_json("FK_WORKSPACE_CONTEXT_JSON", {
        "planning_frame": planning_frame,
        "pose_reference_frame": planner.get_pose_reference_frame(),
        "selected_tcp_link": planner.get_end_effector_link(),
        "joint_names": joint_names,
        "joint_limits": {name: list(limits[name]) for name in joint_names},
        "named_states": named_state_names,
        "halton_samples": args.halton_samples,
    })

    samples = list(structured_samples(current_joints, joint_names, limits, named_states))
    samples.extend(halton_joint_samples(args.halton_samples, joint_names, limits))

    valid_joint_sample_count = 0
    collision_free_count = 0
    fk_success_count = 0
    candidates: List[Dict[str, object]] = []
    workspace_candidates: List[Dict[str, object]] = []
    start = time.time()
    processed_count = 0
    partial_stop_reason = None
    for index, joints in enumerate(samples, start=1):
        processed_count = index
        if index <= 5:
            print_json("FK_WORKSPACE_EARLY_SAMPLE_JSON", {
                "index": index,
                "stage": "start",
                "joint_positions": [float(x) for x in joints],
            })
        if not inside_limits(joints, joint_names, limits):
            continue
        valid_joint_sample_count += 1
        state = robot_state_with_joints(base_state, joint_names, joints)
        if index <= 5:
            print_json("FK_WORKSPACE_EARLY_SAMPLE_JSON", {"index": index, "stage": "before_validity"})
        if not services.check_state(state):
            if index <= 5:
                print_json("FK_WORKSPACE_EARLY_SAMPLE_JSON", {"index": index, "stage": "collision_rejected"})
            continue
        collision_free_count += 1
        if index <= 5:
            print_json("FK_WORKSPACE_EARLY_SAMPLE_JSON", {"index": index, "stage": "before_fk"})
        pose = services.compute_fk(state)
        if pose is None:
            if index <= 5:
                print_json("FK_WORKSPACE_EARLY_SAMPLE_JSON", {"index": index, "stage": "fk_failed"})
            continue
        fk_success_count += 1
        if index <= 5:
            print_json("FK_WORKSPACE_EARLY_SAMPLE_JSON", {"index": index, "stage": "fk_success"})
        candidate = candidate_from_fk(pose, joints, current_joints, joint_names, limits)
        candidates.append(candidate)
        if candidate["inside_tabletop_workspace"]:
            workspace_candidates.append(candidate)
        if args.progress_every > 0 and index % args.progress_every == 0:
            print_json("FK_WORKSPACE_PROGRESS_JSON", {
                "processed": index,
                "valid_joint_sample_count": valid_joint_sample_count,
                "collision_free_count": collision_free_count,
                "fk_success_count": fk_success_count,
                "workspace_candidate_count": len(workspace_candidates),
                "elapsed_s": round(time.time() - start, 3),
            })
        if args.max_runtime_s > 0.0 and time.time() - start >= args.max_runtime_s:
            partial_stop_reason = "max_runtime_s %.3f reached" % args.max_runtime_s
            print_json("FK_WORKSPACE_PARTIAL_STOP_JSON", {
                "processed": index,
                "requested_sample_count": len(samples),
                "reason": partial_stop_reason,
                "elapsed_s": round(time.time() - start, 3),
            })
            break

    thresholds = [5, 10, 20, 30, 45]
    min_angle = min((float(c["approach_angle_to_down_deg"]) for c in candidates), default=None)
    summary = {
        "sample_count": len(samples),
        "processed_sample_count": processed_count,
        "partial_stop_reason": partial_stop_reason,
        "valid_joint_sample_count": valid_joint_sample_count,
        "collision_free_count": collision_free_count,
        "workspace_candidate_count": len(workspace_candidates),
        "minimum_downward_approach_angle_deg": min_angle,
        "near_vertical_counts": {
            "%d_deg" % threshold: sum(
                1 for c in workspace_candidates if c["approach_angle_to_down_deg"] <= threshold
            )
            for threshold in thresholds
        },
    }
    print_json("FK_WORKSPACE_SUMMARY_JSON", summary)

    ranked = sorted(candidates, key=rank_key)
    for rank, candidate in enumerate(ranked[:20], start=1):
        row = dict(candidate)
        row["rank"] = rank
        print_json("FK_REACHABLE_TCP_CANDIDATE_JSON", row)

    cup_hover = [args.cup_x, args.cup_y, args.cup_hover_z]
    print_json("CUP_TARGET_JSON", {
        "cup_surface_xyz": [args.cup_x, args.cup_y, args.cup_z],
        "requested_tcp_hover_xyz": cup_hover,
    })
    nearest_rows = {}
    for label, threshold in [
        ("unrestricted_orientation", None),
        ("45_deg", 45.0),
        ("30_deg", 30.0),
        ("20_deg", 20.0),
        ("10_deg", 10.0),
    ]:
        nearest = nearest_candidate(candidates, cup_hover, threshold)
        nearest_rows[label] = nearest
        print_json("FK_NEAREST_CUP_HOVER_JSON", {
            "filter": label,
            "result": nearest,
        })

    fk_seed_candidates = []
    for value in nearest_rows.values():
        if value is not None:
            fk_seed_candidates.append(value)
    fk_seed_candidates.extend(ranked[:5])
    dedup = []
    seen = set()
    for candidate in fk_seed_candidates:
        key = tuple(round(float(x), 5) for x in candidate["joint_positions"])
        if key in seen:
            continue
        seen.add(key)
        dedup.append(candidate)
    run_fk_seeded_ik_tests(services, base_state, dedup[:5], cup_hover, joint_names)

    recommended_pool = workspace_candidates if workspace_candidates else candidates
    if recommended_pool:
        xs = [float(c["tcp_xyz"][0]) for c in recommended_pool]
        ys = [float(c["tcp_xyz"][1]) for c in recommended_pool]
        zs = [float(c["tcp_xyz"][2]) for c in recommended_pool]
        recommended_region = "x %.3f..%.3f, y %.3f..%.3f, tcp_z %.3f..%.3f" % (
            min(xs), max(xs), min(ys), max(ys), min(zs), max(zs)
        )
        known_valid = min(recommended_pool, key=rank_key)
    else:
        recommended_region = "No collision-free FK TCP poses found."
        known_valid = None

    print_json("FK_WORKSPACE_CONCLUSION_JSON", {
        "conclusion": conclusion(candidates, workspace_candidates),
        "recommended_cup_region": recommended_region,
        "known_valid_tcp_pose_and_joint_state": known_valid,
        "hardware_commanded": False,
    })
    print("NO_HARDWARE_COMMANDS_ISSUED", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
