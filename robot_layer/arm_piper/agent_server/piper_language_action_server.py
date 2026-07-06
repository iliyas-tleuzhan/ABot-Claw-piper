#!/usr/bin/env python3
"""Small language-action HTTP layer for safe Piper arm nudges.

This server intentionally avoids ABot /code/execute, camera pipelines, and
/end_pose. It reads live joint state from /joint_states_single and sends small
joint-space commands through PiperRobotEnv.
"""

import os
import threading
import time
from typing import Any, Dict, Optional

import rospy
import uvicorn
from fastapi import FastAPI
from pydantic import BaseModel
from sensor_msgs.msg import JointState

from robot_sdk.piper_sdk import PiperRobotEnv


SERVER_MARKER = "piper_language_action_server_v1"
SERVER_REVISION = "ros_state_cache_20260706"
DEFAULT_PORT = 8891
JOINT_STATE_TOPIC = os.environ.get("PIPER_JOINT_STATE_TOPIC", "/joint_states_single")
MAX_JOINT_STEP_RAD = 0.08
DEFAULT_JOINT_STEP_RAD = 0.04
DEFAULT_SPEED = 0.05
DEFAULT_ACCELERATION = 0.05
DEFAULT_GRIPPER_OPEN = 0.04
DEFAULT_GRIPPER_CLOSED = 0.005


app = FastAPI(title="Piper Language Action Server")
robot: Optional[PiperRobotEnv] = None
robot_init_lock = threading.Lock()
robot_lock = threading.Lock()
ros_init_lock = threading.Lock()
joint_state_lock = threading.Lock()
joint_state_event = threading.Event()
joint_state_subscriber: Optional[rospy.Subscriber] = None
latest_joint_state: Optional[JointState] = None
latest_joint_state_time: Optional[float] = None


class MoveRequest(BaseModel):
    joint_step: float = DEFAULT_JOINT_STEP_RAD
    speed: float = DEFAULT_SPEED
    accel: float = DEFAULT_ACCELERATION


class GripperRequest(BaseModel):
    position: float
    speed: float = DEFAULT_SPEED
    accel: float = DEFAULT_ACCELERATION


def base_response(**extra: Any) -> Dict[str, Any]:
    return {"server": SERVER_MARKER, "server_revision": SERVER_REVISION, **extra}


def ensure_ros_node() -> None:
    if rospy.get_node_uri():
        return
    with ros_init_lock:
        if not rospy.get_node_uri():
            rospy.init_node(
                "piper_language_action_server",
                anonymous=True,
                disable_signals=True,
            )


def get_robot() -> PiperRobotEnv:
    global robot
    if robot is None:
        with robot_init_lock:
            if robot is None:
                ensure_ros_node()
                robot = PiperRobotEnv(init_ros_node=True)
    return robot


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


def clamp_joint_step(joint_step: float) -> float:
    return clamp(abs(float(joint_step)), 0.0, MAX_JOINT_STEP_RAD)


def joint_state_callback(msg: JointState) -> None:
    global latest_joint_state, latest_joint_state_time
    with joint_state_lock:
        latest_joint_state = msg
        latest_joint_state_time = time.time()
    joint_state_event.set()


def start_joint_state_subscriber() -> None:
    global joint_state_subscriber
    ensure_ros_node()
    if joint_state_subscriber is None:
        with joint_state_lock:
            if joint_state_subscriber is None:
                joint_state_subscriber = rospy.Subscriber(
                    JOINT_STATE_TOPIC,
                    JointState,
                    joint_state_callback,
                    queue_size=1,
                    tcp_nodelay=True,
                )


def joint_state_to_payload(msg: JointState, source: str) -> Dict[str, Any]:
    names = list(msg.name)
    positions = [float(position) for position in msg.position]
    joint_values = dict(zip(names, positions))

    if len(positions) < 6:
        raise RuntimeError(
            f"{JOINT_STATE_TOPIC} returned {len(positions)} positions, expected at least 6"
        )

    q = [
        float(joint_values.get("joint1", positions[0])),
        float(joint_values.get("joint2", positions[1])),
        float(joint_values.get("joint3", positions[2])),
        float(joint_values.get("joint4", positions[3])),
        float(joint_values.get("joint5", positions[4])),
        float(joint_values.get("joint6", positions[5])),
    ]
    gripper = float(
        joint_values.get(
            "gripper",
            joint_values.get(
                "gripper_joint",
                joint_values.get("joint7", positions[6] if len(positions) > 6 else 0.0),
            ),
        )
    )

    return {
        "joint_positions": q,
        "gripper_position": gripper,
        "ros_joint_names": names,
        "ros_joint_positions": positions,
        "joint_state_topic": JOINT_STATE_TOPIC,
        "joint_state_source": source,
        "joint_state_age_s": (
            None if latest_joint_state_time is None else time.time() - latest_joint_state_time
        ),
    }


def read_live_joints(timeout: float = 5.0) -> Dict[str, Any]:
    start_joint_state_subscriber()
    with joint_state_lock:
        if latest_joint_state is not None and latest_joint_state_time is not None:
            if time.time() - latest_joint_state_time <= timeout:
                return joint_state_to_payload(latest_joint_state, "subscriber_cache")

    joint_state_event.clear()
    if joint_state_event.wait(timeout):
        with joint_state_lock:
            if latest_joint_state is not None:
                return joint_state_to_payload(latest_joint_state, "subscriber_cache")

    msg = rospy.wait_for_message(JOINT_STATE_TOPIC, JointState, timeout=timeout)
    return joint_state_to_payload(msg, "wait_for_message")


def ros_diagnostics() -> Dict[str, Any]:
    diagnostics: Dict[str, Any] = {
        "joint_state_topic": JOINT_STATE_TOPIC,
        "ros_master_uri": os.environ.get("ROS_MASTER_URI"),
        "ros_hostname": os.environ.get("ROS_HOSTNAME"),
        "ros_ip": os.environ.get("ROS_IP"),
        "rospy_node_uri": rospy.get_node_uri(),
    }
    try:
        diagnostics["published_topics"] = rospy.get_published_topics("/")
    except Exception as exc:
        diagnostics["published_topics_error"] = repr(exc)
    return diagnostics


def success_from_result(result: Any) -> bool:
    if isinstance(result, dict):
        return bool(result.get("success", False))
    return bool(result)


def move_joint2_joint3(direction: str, req: MoveRequest) -> Dict[str, Any]:
    try:
        r = get_robot()
    except Exception as exc:
        return base_response(
            success=False,
            action=direction,
            error=f"PiperRobotEnv initialization failed: {repr(exc)}",
            ros_diagnostics=ros_diagnostics(),
        )
    joint_step = clamp_joint_step(req.joint_step)

    with robot_lock:
        try:
            live_state = read_live_joints()
        except Exception as exc:
            return base_response(
                success=False,
                action=direction,
                ros_read_error=repr(exc),
                ros_diagnostics=ros_diagnostics(),
            )

        before_joints = list(live_state["joint_positions"])
        target_joints = before_joints[:]
        if direction == "move_up":
            target_joints[1] += joint_step
            target_joints[2] -= joint_step
            action_detail = "joint2_plus_joint3_minus"
        elif direction == "move_down":
            target_joints[1] -= joint_step
            target_joints[2] += joint_step
            action_detail = "joint2_minus_joint3_plus"
        else:
            return base_response(success=False, action=direction, error="unknown direction")

        gripper_position = live_state["gripper_position"]
        try:
            result = r.move_joints(
                target_joints,
                gripper=gripper_position,
                max_velocity=req.speed,
                max_acceleration=req.accel,
            )
        except Exception as exc:
            return base_response(
                success=False,
                action=direction,
                action_detail=action_detail,
                joint_step=joint_step,
                before_joints=before_joints,
                target_joints=target_joints,
                gripper_position=gripper_position,
                ros_joint_names=live_state["ros_joint_names"],
                ros_joint_positions=live_state["ros_joint_positions"],
                error=f"move_joints failed: {repr(exc)}",
            )

    return base_response(
        success=success_from_result(result),
        action=direction,
        action_detail=action_detail,
        joint_step=joint_step,
        before_joints=before_joints,
        target_joints=target_joints,
        gripper_position=gripper_position,
        ros_joint_names=live_state["ros_joint_names"],
        ros_joint_positions=live_state["ros_joint_positions"],
        result=result,
    )


@app.get("/health")
def health() -> Dict[str, Any]:
    return base_response(
        success=True,
        status="ok",
        robot_initialized=robot is not None,
        joint_state_topic=JOINT_STATE_TOPIC,
        ros_node_initialized=bool(rospy.get_node_uri()),
    )


@app.get("/state")
def state() -> Dict[str, Any]:
    try:
        live_state = read_live_joints()
    except Exception as exc:
        return base_response(
            success=False,
            ros_read_error=repr(exc),
            ros_diagnostics=ros_diagnostics(),
        )

    return base_response(success=True, **live_state, ros_read_error=None)


@app.post("/move_up")
def move_up(req: MoveRequest = MoveRequest()) -> Dict[str, Any]:
    return move_joint2_joint3("move_up", req)


@app.post("/move_down")
def move_down(req: MoveRequest = MoveRequest()) -> Dict[str, Any]:
    return move_joint2_joint3("move_down", req)


@app.post("/open_gripper")
def open_gripper() -> Dict[str, Any]:
    return set_gripper(GripperRequest(position=DEFAULT_GRIPPER_OPEN))


@app.post("/close_gripper")
def close_gripper() -> Dict[str, Any]:
    return set_gripper(GripperRequest(position=DEFAULT_GRIPPER_CLOSED))


@app.post("/set_gripper")
def set_gripper(req: GripperRequest) -> Dict[str, Any]:
    try:
        r = get_robot()
    except Exception as exc:
        return base_response(
            success=False,
            action="set_gripper",
            error=f"PiperRobotEnv initialization failed: {repr(exc)}",
            ros_diagnostics=ros_diagnostics(),
        )
    position = clamp(float(req.position), r.GRIPPER_MIN, r.GRIPPER_MAX)
    with robot_lock:
        try:
            result = r.set_gripper(
                position,
                max_velocity=req.speed,
                max_acceleration=req.accel,
            )
        except Exception as exc:
            return base_response(
                success=False,
                action="set_gripper",
                position=position,
                error=f"set_gripper failed: {repr(exc)}",
            )

    return base_response(
        success=success_from_result(result),
        action="set_gripper",
        position=position,
        result=result,
    )


if __name__ == "__main__":
    port = int(os.environ.get("PIPER_LANGUAGE_ACTION_PORT", DEFAULT_PORT))
    print(f"Starting {SERVER_MARKER} on port {port}", flush=True)
    uvicorn.run(app, host="0.0.0.0", port=port)
