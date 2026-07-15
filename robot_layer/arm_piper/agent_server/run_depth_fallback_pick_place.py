#!/usr/bin/env python3
"""Run a depth-fallback grasp pick/place through Agent Server code execution."""

from __future__ import annotations

import argparse
import json
import sys
import time
from typing import Any, Dict, Optional
from urllib import error, request


TERMINAL_STATUSES = {"completed", "failed", "timeout", "stopped", "idle"}


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
        "object_name": args.object_name,
        "top_k": args.top_k,
        "plan_only": args.plan_only,
        "pregrasp_height": args.pregrasp_height,
        "grasp_z_offset": args.grasp_z_offset,
        "lift_height": args.lift_height,
        "place": [args.place_x, args.place_y, args.place_z],
        "velocity": args.velocity,
        "acceleration": args.acceleration,
        "staging_joints": [args.stage_j1, args.stage_j2, args.stage_j3, args.stage_j4, args.stage_j5, args.stage_j6],
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

from geometry_msgs.msg import Pose
from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import RobotState

CONFIG = {repr(cfg)}


def finite_vec(name, values, n):
    if values is None or len(values) != n:
        raise RuntimeError(name + " missing or wrong length: " + repr(values))
    out = [float(v) for v in values]
    for value in out:
        if not math.isfinite(value):
            raise RuntimeError(name + " contains non-finite value: " + repr(values))
    return out


def clamp_gripper(value):
    return max(float(env.GRIPPER_MIN), min(float(env.GRIPPER_MAX), float(value)))


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


def start_state_from_trajectory(trajectory):
    points = getattr(getattr(trajectory, "joint_trajectory", None), "points", [])
    joint_names = list(getattr(getattr(trajectory, "joint_trajectory", None), "joint_names", []))
    if not points or not joint_names:
        return None
    state = RobotState()
    state.joint_state.name = joint_names
    state.joint_state.position = list(points[-1].positions)
    return state


def summarize_plan(name, result):
    if isinstance(result, tuple):
        success = bool(result[0])
        trajectory = result[1]
        planning_time = float(result[2]) if len(result) > 2 else 0.0
        error_code = str(result[3]) if len(result) > 3 else ""
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
    if not success or count == 0:
        raise RuntimeError("MoveIt planning failed for " + name)
    return trajectory, {{"name": name, "success": success, "points": count, "planning_time": planning_time, "error": error_code}}


def set_next_start_state(planner, trajectory):
    state = start_state_from_trajectory(trajectory)
    if state is not None:
        planner.set_start_state(state)


def plan_joints(planner, name, joints):
    planner.clear_pose_targets()
    planner.set_joint_value_target([float(v) for v in joints])
    trajectory, summary = summarize_plan(name, planner.plan())
    set_next_start_state(planner, trajectory)
    return summary


def plan_target(planner, name, xyz, quat):
    planner.clear_pose_targets()
    planner.set_pose_target(pose_msg(xyz, quat))
    trajectory, summary = summarize_plan(name, planner.plan())
    set_next_start_state(planner, trajectory)
    return summary


print("Depth fallback pick/place task")
print("mode=%s object=%s top_k=%d" % (
    "plan-only" if CONFIG["plan_only"] else "execute",
    CONFIG["object_name"],
    CONFIG["top_k"],
))
print("grasp_url=%s" % getattr(grasp, "_grasp_url", "(unknown)"))

results = grasp.get_grasp_pose(CONFIG["object_name"], top_k=CONFIG["top_k"])
instances = [r for r in results if r.get("grasps")]
if not instances:
    raise RuntimeError("No grasp returned for object " + CONFIG["object_name"])

selected = None
for inst in instances:
    for candidate in inst.get("grasps", []):
        width = float(candidate.get("width", 0.0))
        if width <= 0.0 or width <= float(env.GRIPPER_MAX) + 1e-6:
            selected = (inst, candidate)
            break
    if selected is not None:
        break
if selected is None:
    raise RuntimeError("No candidate compatible with gripper max %.4f m" % float(env.GRIPPER_MAX))

inst, best = selected
camera_grasp = finite_vec("translation_camera", best.get("translation_camera"), 3)
camera_retreat = finite_vec("translation_camera_retreat", best.get("translation_camera_retreat"), 3)
base_grasp = finite_vec("translation_base", best.get("translation_base"), 3)
base_retreat = finite_vec("translation_base_retreat", best.get("translation_base_retreat"), 3)
quat_sdk = finite_vec("quaternion_base", best.get("quaternion_base"), 4)
# Existing Piper pipeline top-down orientation used by today_red_to_purple_pick_place.py.
quat = [0.0, 0.7071067811865476, 0.0, 0.7071067811865476]
width = float(best.get("width", 0.0))
score = float(best.get("score", 0.0))
if base_grasp[2] <= 0.0 or camera_grasp[2] <= 0.0:
    raise RuntimeError("Invalid depth-derived grasp z: camera=%s base=%s" % (camera_grasp, base_grasp))

grasp_xyz = [base_grasp[0], base_grasp[1], base_grasp[2] + float(CONFIG["grasp_z_offset"])]
pre = [grasp_xyz[0], grasp_xyz[1], grasp_xyz[2] + float(CONFIG["pregrasp_height"])]
grasp_pose = [grasp_xyz[0], grasp_xyz[1], grasp_xyz[2]]
lift = [grasp_xyz[0], grasp_xyz[1], grasp_xyz[2] + float(CONFIG["lift_height"])]
place = [float(v) for v in CONFIG["place"]]
pre_place = [place[0], place[1], place[2] + float(CONFIG["pregrasp_height"])]
retreat = [pre_place[0], pre_place[1], pre_place[2]]

sequence = [
    ("pre_grasp", pre, quat),
    ("grasp", grasp_pose, quat),
    ("lift", lift, quat),
    ("pre_place", pre_place, quat),
    ("place", place, quat),
    ("retreat", retreat, quat),
]

for name, xyz, _quat in sequence:
    assert_workspace(name, xyz)

open_width = clamp_gripper(env.GRIPPER_MAX)
close_width = clamp_gripper(max(env.GRIPPER_MIN, min(env.GRIPPER_MAX, width * 0.35)))

summary = {{
    "label": inst.get("label"),
    "confidence": inst.get("confidence"),
    "xyxy": inst.get("xyxy"),
    "score": score,
    "width_m": width,
    "camera_frame": getattr(grasp, "_camera_frame_id", "table_camera_color_optical_frame"),
    "base_frame": getattr(grasp, "_base_frame_id", "base_link"),
    "camera_grasp": camera_grasp,
    "camera_retreat": camera_retreat,
    "base_grasp": base_grasp,
    "base_retreat_from_sdk": base_retreat,
    "quaternion_base_from_sdk": quat_sdk,
    "quaternion_base": quat,
    "open_width_m": open_width,
    "close_width_m": close_width,
    "sequence": [
        {{"name": name, "xyz": xyz, "quat": q}} for name, xyz, q in sequence
    ],
}}
print("SELECTED_GRASP_JSON " + json.dumps(summary, sort_keys=True))

planner = MoveGroupCommander("arm")
planner.set_max_velocity_scaling_factor(float(CONFIG["velocity"]))
planner.set_max_acceleration_scaling_factor(float(CONFIG["acceleration"]))
planner.set_start_state_to_current_state()
plan_results = []
plan_results.append(plan_joints(planner, "stage", CONFIG["staging_joints"]))
for name, xyz, q in sequence:
    plan_results.append(plan_target(planner, name, xyz, q))
print("MOVEIT_PLAN_JSON " + json.dumps(plan_results, sort_keys=True))

if CONFIG["plan_only"]:
    print("PLAN_ONLY_COMPLETE no robot movement commanded")
else:
    print("EXECUTION_START")
    commands = [
        ("open_gripper", None, None),
        ("move_stage", CONFIG["staging_joints"], None),
        ("move_pre_grasp", pre, quat),
        ("move_grasp", grasp_pose, quat),
        ("close_gripper", None, None),
        ("move_lift", lift, quat),
        ("move_pre_place", pre_place, quat),
        ("move_place", place, quat),
        ("open_gripper", None, None),
        ("move_retreat", retreat, quat),
    ]
    for name, xyz, q in commands:
        print("EXEC_STEP " + name)
        if name == "open_gripper":
            result = env.set_gripper(open_width, max_velocity=CONFIG["velocity"], max_acceleration=CONFIG["acceleration"])
        elif name == "close_gripper":
            result = env.set_gripper(close_width, max_velocity=CONFIG["velocity"], max_acceleration=CONFIG["acceleration"])
        elif name == "move_stage":
            result = env.move_joints(xyz, max_velocity=CONFIG["velocity"], max_acceleration=CONFIG["acceleration"])
        else:
            result = env.move_to_pose(xyz + q, max_velocity=CONFIG["velocity"], max_acceleration=CONFIG["acceleration"])
        print("EXEC_RESULT %s %s" % (name, result))
        if not result.get("success"):
            raise RuntimeError("Execution failed at " + name)
        time.sleep(0.5)
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
    print("FINAL_STATUS " + status)
    if status != "completed":
        final = http_json("GET", base + "/code/result")
        result = final.get("result") or {}
        if status == "idle" and result.get("status") == "completed":
            return 0
        print("FINAL_RESULT " + json.dumps(final, indent=2, sort_keys=True))
        return 1
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--agent-url", default="http://127.0.0.1:8888")
    parser.add_argument("--holder", default="depth-fallback-pick-place")
    parser.add_argument("--object-name", default="cup")
    parser.add_argument("--top-k", type=int, default=5)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--plan-only", action="store_true")
    mode.add_argument("--execute", action="store_true")
    parser.add_argument("--pregrasp-height", type=float, default=0.10)
    parser.add_argument("--grasp-z-offset", type=float, default=0.04)
    parser.add_argument("--lift-height", type=float, default=0.10)
    parser.add_argument("--place-x", type=float, default=0.30)
    parser.add_argument("--place-y", type=float, default=0.18)
    parser.add_argument("--place-z", type=float, default=0.18)
    parser.add_argument("--stage-j1", type=float, default=0.0)
    parser.add_argument("--stage-j2", type=float, default=0.55)
    parser.add_argument("--stage-j3", type=float, default=-0.75)
    parser.add_argument("--stage-j4", type=float, default=0.0)
    parser.add_argument("--stage-j5", type=float, default=0.65)
    parser.add_argument("--stage-j6", type=float, default=0.0)
    parser.add_argument("--velocity", type=float, default=0.05)
    parser.add_argument("--acceleration", type=float, default=0.05)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--x-min", type=float, default=0.05)
    parser.add_argument("--x-max", type=float, default=0.60)
    parser.add_argument("--y-min", type=float, default=-0.35)
    parser.add_argument("--y-max", type=float, default=0.35)
    parser.add_argument("--z-min", type=float, default=0.02)
    parser.add_argument("--z-max", type=float, default=0.55)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.execute:
        args.plan_only = False
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
