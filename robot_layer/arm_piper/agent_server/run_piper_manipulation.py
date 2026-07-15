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
from geometry_msgs.msg import Pose
from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import RobotState

CONFIG = {repr(cfg)}
DOWNWARD_QUAT = [0.0, 0.7071067811865476, 0.0, 0.7071067811865476]


def finite_vec(name, values, n):
    if values is None or len(values) != n:
        raise RuntimeError(name + " missing or wrong length: " + repr(values))
    out = [float(v) for v in values]
    for value in out:
        if not math.isfinite(value):
            raise RuntimeError(name + " contains non-finite value: " + repr(values))
    return out


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
    if fraction < 0.999:
        raise RuntimeError("Cartesian path %s incomplete: fraction=%.6f" % (name, fraction))
    retime_start = start_state if start_state is not None else planner.get_current_state()
    trajectory = retime_trajectory(planner, retime_start, trajectory)
    point_count = trajectory_point_count(trajectory)
    if point_count == 0:
        raise RuntimeError("Cartesian path %s produced no trajectory points" % name)
    final_state = start_state_from_trajectory(trajectory)
    print("CARTESIAN_PLAN_JSON " + json.dumps({{
        "name": name,
        "fraction": float(fraction),
        "points": point_count,
        "waypoints": [pose_to_dict(pose) for pose in waypoints],
        "final_pose": pose_to_dict(waypoints[-1]),
    }}, sort_keys=True))
    return trajectory, {{
        "name": name,
        "type": "cartesian",
        "fraction": float(fraction),
        "points": point_count,
        "waypoints": [pose_to_dict(pose) for pose in waypoints],
        "final_pose": pose_to_dict(waypoints[-1]),
    }}, final_state


def plan_target(planner, name, xyz, quat):
    assert_workspace(name, xyz)
    planner.clear_pose_targets()
    planner.set_pose_target(pose_msg(xyz, quat))
    trajectory, summary = summarize_plan(name, planner.plan())
    state = start_state_from_trajectory(trajectory)
    if state is not None:
        planner.set_start_state(state)
    return trajectory, summary


def execute_trajectory(planner, name, trajectory):
    print("EXEC_STEP " + name)
    ok = bool(planner.execute(trajectory, wait=True))
    print("EXEC_RESULT %s %s" % (name, ok))
    planner.stop()
    planner.clear_pose_targets()
    if not ok:
        raise RuntimeError("Execution failed at " + name)


def select_source_grasp(source):
    results = grasp.get_grasp_pose(source, top_k=CONFIG["top_k"])
    instances = [r for r in results if r.get("grasps")]
    if not instances:
        raise RuntimeError("No grasp returned for source " + source)
    for inst in instances:
        for candidate in inst.get("grasps", []):
            width = float(candidate.get("width", 0.0))
            if 0.0 < width <= float(env.GRIPPER_MAX) + 1e-6:
                source_camera = finite_vec("source.translation_camera", candidate.get("translation_camera"), 3)
                source_base = finite_vec("source.translation_base", candidate.get("translation_base"), 3)
                return inst, candidate, source_camera, source_base
    raise RuntimeError(
        "No source grasp with valid width 0.0 < width <= %.4f m for %s"
        % (float(env.GRIPPER_MAX), source)
    )


def depth_to_m(depth_value):
    value = float(depth_value)
    if value <= 0.0:
        return None
    return value / 1000.0 if value > 20.0 else value


def deproject(u, v, depth_m, K):
    fx, fy = float(K[0, 0]), float(K[1, 1])
    cx, cy = float(K[0, 2]), float(K[1, 2])
    return [(float(u) - cx) * depth_m / fx, (float(v) - cy) * depth_m / fy, depth_m]


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

current_pose = env.get_robot_end_pose()
if current_pose is None:
    raise RuntimeError("No current end-effector pose from /end_pose")
current_xyz = finite_vec("current_end_effector.position", current_pose.get("position"), 3)

inst, best, source_camera, source_base = select_source_grasp(CONFIG["source"])
width = float(best.get("width", 0.0))
open_width = float(env.GRIPPER_MAX)
close_width = max(float(env.GRIPPER_MIN), min(float(env.GRIPPER_MAX), width * 0.35))

destination = None
destination_base = None
if CONFIG["destination"]:
    destination = detect_destination(CONFIG["destination"])
    destination_base = finite_vec("destination.base_xyz", destination["base_xyz"], 3)

print("SOURCE_DETECTION_JSON " + json.dumps({{
    "label": inst.get("label"),
    "confidence": inst.get("confidence"),
    "xyxy": inst.get("xyxy"),
    "width_m": width,
    "source_camera_xyz": source_camera,
    "source_base_xyz": source_base,
    "selected_grasp": best,
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

raised_current_pose = pose_msg([current_xyz[0], current_xyz[1], transit_z], DOWNWARD_QUAT)
source_hover_pose = pose_msg([source_base[0], source_base[1], hover_z], DOWNWARD_QUAT)
source_grasp_pose = pose_msg([source_base[0], source_base[1], grasp_z], DOWNWARD_QUAT)

for name, pose in [
    ("raised_current", raised_current_pose),
    ("source_hover", source_hover_pose),
    ("source_grasp", source_grasp_pose),
    ("source_hover_after_grasp", source_hover_pose),
]:
    assert_workspace(name, [pose.position.x, pose.position.y, pose.position.z])

planner = MoveGroupCommander("arm")
planner.set_max_velocity_scaling_factor(float(CONFIG["velocity_scaling"]))
planner.set_max_acceleration_scaling_factor(float(CONFIG["acceleration_scaling"]))
planner.set_start_state_to_current_state()

print("START_POSE_JSON " + json.dumps(pose_to_dict(pose_from_current(current_pose)), sort_keys=True))
print("DOWNWARD_TOOL_QUATERNION_XYZW " + json.dumps(DOWNWARD_QUAT))

plans = {{}}
summaries = []

approach_trajectory, approach_summary, approach_final_state = plan_cartesian(
    planner,
    "approach",
    [raised_current_pose, source_hover_pose, source_grasp_pose],
)
plans["approach"] = approach_trajectory
summaries.append(approach_summary)

lift_trajectory, lift_summary, lift_final_state = plan_cartesian(
    planner,
    "lift",
    [source_hover_pose],
    approach_final_state,
)
plans["lift"] = lift_trajectory
summaries.append(lift_summary)

transport_name = None
destination_descend_name = None
destination_rise_name = None
if CONFIG["task"] == "place":
    if destination_base is None:
        raise RuntimeError("Place task requires a detected destination")
    dest_hover_z = max(transit_z, destination_base[2] + float(CONFIG["hover_height"]))
    place_z = destination_base[2] + float(CONFIG["place_z_offset"])
    destination_hover_pose = pose_msg([destination_base[0], destination_base[1], dest_hover_z], DOWNWARD_QUAT)
    destination_place_pose = pose_msg([destination_base[0], destination_base[1], place_z], DOWNWARD_QUAT)
    for name, pose in [
        ("destination_hover", destination_hover_pose),
        ("destination_place", destination_place_pose),
        ("destination_hover_after_release", destination_hover_pose),
    ]:
        assert_workspace(name, [pose.position.x, pose.position.y, pose.position.z])

    planner.set_start_state(lift_final_state)
    transport_trajectory, transport_summary = plan_target(
        planner,
        "transport_to_destination_hover",
        [destination_hover_pose.position.x, destination_hover_pose.position.y, destination_hover_pose.position.z],
        DOWNWARD_QUAT,
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

    execute_trajectory(planner, "approach", plans["approach"])

    print("EXEC_STEP close_gripper")
    result = env.set_gripper(close_width)
    print("EXEC_RESULT close_gripper %s" % result)
    if not result.get("success"):
        raise RuntimeError("close_gripper failed")

    execute_trajectory(planner, "lift", plans["lift"])

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
    parser.add_argument("--holder", default="piper-manipulation")
    parser.add_argument("--task", choices=("pick", "place"), required=True)
    parser.add_argument("--source", required=True)
    parser.add_argument("--destination")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--plan-only", action="store_true")
    mode.add_argument("--execute", action="store_true")
    mode.add_argument("--perception-only", action="store_true")
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
