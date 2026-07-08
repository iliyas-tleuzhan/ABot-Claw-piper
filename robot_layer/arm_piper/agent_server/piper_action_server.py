#!/usr/bin/env python3
import threading
from typing import Optional

from fastapi import FastAPI
from pydantic import BaseModel
import uvicorn
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

from robot_sdk.piper_sdk import PiperRobotEnv

app = FastAPI(title="Piper Safe Action Server")

robot = None
lock = threading.Lock()


class MoveRequest(BaseModel):
    distance: float = 0.03
    speed: float = 0.05
    accel: float = 0.05


class GripperRequest(BaseModel):
    position: float
    speed: float = 0.05
    accel: float = 0.05


def get_robot():
    global robot
    if robot is None:
        robot = PiperRobotEnv(init_ros_node=True)
    return robot


def clamp_distance(d: float) -> float:
    # Hard safety limit: max 5 cm per command.
    if d > 0.05:
        return 0.05
    if d < -0.05:
        return -0.05
    return d



def current_end_pose(r, timeout=2.0):
    """Return [x,y,z,qx,qy,qz,qw] from SDK cache or /end_pose topic."""
    try:
        pose = r.get_robot_end_pose()
        if pose:
            x, y, z = pose["position"]
            qx, qy, qz, qw = pose["orientation_quat"]
            return [x, y, z, qx, qy, qz, qw]
    except Exception as e:
        print(f"get_robot_end_pose failed: {e}", flush=True)

    msg = rospy.wait_for_message("/end_pose", PoseStamped, timeout=timeout)
    return [
        msg.pose.position.x,
        msg.pose.position.y,
        msg.pose.position.z,
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w,
    ]



def read_live_joints(timeout=3.0):
    """
    Read live Piper joint state directly from /joint_states_single.
    Returns:
      q = [joint1..joint6]
      gripper = gripper position
      names = raw joint names
      positions = raw positions
    """
    msg = rospy.wait_for_message("/joint_states_single", JointState, timeout=timeout)

    names = list(msg.name)
    positions = list(msg.position)

    joint_values = {}
    for name, pos in zip(names, positions):
        joint_values[name] = pos

    q = [
        joint_values.get("joint1", positions[0]),
        joint_values.get("joint2", positions[1]),
        joint_values.get("joint3", positions[2]),
        joint_values.get("joint4", positions[3]),
        joint_values.get("joint5", positions[4]),
        joint_values.get("joint6", positions[5]),
    ]

    gripper = joint_values.get("gripper", positions[6] if len(positions) > 6 else 0.04)

    return q, gripper, names, positions


@app.get("/health")
def health():
    return {"status": "ok", "robot_initialized": robot is not None}


@app.get("/state")
def state():
    r = get_robot()
    s = r.get_robot_state()
    try:
        p = current_end_pose(r, timeout=1.0)
    except Exception:
        p = None
    return {
        "joint_positions": s["joint_positions"].tolist(),
        "joint_velocities": s["joint_velocities"].tolist(),
        "gripper_position": float(s["gripper_position"][0]),
        "end_pose": p,
    }





@app.post("/move_up")
def move_up(req: MoveRequest):
    """
    Safe joint-based arm-up action.
    Empirically/visually best on this Piper:
    joint2_plus + joint3_minus raises the arm cleanly.
    """
    r = get_robot()

    joint_step = abs(req.distance) * 2.0

    with lock:
        try:
            q, gripper, names, positions = read_live_joints(timeout=3.0)
        except Exception as e:
            return {"success": False, "error": f"Could not read /joint_states_single: {repr(e)}"}

        before = q[:]

        # Best observed upward motion:
        q[1] += joint_step
        q[2] -= joint_step

        try:
            result = r.move_joints(
                q,
                gripper=gripper,
                max_velocity=req.speed,
                max_acceleration=req.accel,
            )
        except Exception as e:
            return {
                "success": False,
                "error": f"move_joints failed: {repr(e)}",
                "before_joints": before,
                "target_joints": q,
            }

    return {
        "success": bool(result.get("success", False)) if isinstance(result, dict) else bool(result),
        "action": "move_up_joint2_plus_joint3_minus",
        "joint_step": joint_step,
        "before_joints": before,
        "target_joints": q,
        "gripper": gripper,
        "result": result,
    }


@app.post("/move_down")
def move_down(req: MoveRequest):
    """
    Reverse of move_up:
    joint2_minus + joint3_plus lowers/returns the arm.
    """
    r = get_robot()
    joint_step = abs(req.distance) * 2.0

    with lock:
        try:
            q, gripper, names, positions = read_live_joints(timeout=3.0)
        except Exception as e:
            return {"success": False, "error": f"Could not read /joint_states_single: {repr(e)}"}

        before = q[:]

        # Reverse of upward motion:
        q[1] -= joint_step
        q[2] += joint_step

        try:
            result = r.move_joints(
                q,
                gripper=gripper,
                max_velocity=req.speed,
                max_acceleration=req.accel,
            )
        except Exception as e:
            return {
                "success": False,
                "error": f"move_joints failed: {repr(e)}",
                "before_joints": before,
                "target_joints": q,
            }

    return {
        "success": bool(result.get("success", False)) if isinstance(result, dict) else bool(result),
        "action": "move_down_joint2_minus_joint3_plus",
        "joint_step": joint_step,
        "before_joints": before,
        "target_joints": q,
        "gripper": gripper,
        "result": result,
    }


@app.post("/open_gripper")
def open_gripper():
    r = get_robot()
    with lock:
        result = r.set_gripper(0.04, max_velocity=0.05, max_acceleration=0.05)
    return {"success": bool(result.get("success")), "action": "open_gripper", "result": result}


@app.post("/close_gripper")
def close_gripper():
    r = get_robot()
    with lock:
        result = r.set_gripper(0.005, max_velocity=0.05, max_acceleration=0.05)
    return {"success": bool(result.get("success")), "action": "close_gripper", "result": result}


@app.post("/set_gripper")
def set_gripper(req: GripperRequest):
    r = get_robot()
    pos = max(0.0, min(0.06, req.position))
    with lock:
        result = r.set_gripper(pos, max_velocity=req.speed, max_acceleration=req.accel)
    return {"success": bool(result.get("success")), "action": "set_gripper", "position": pos, "result": result}


@app.post("/reset")
def reset():
    r = get_robot()
    with lock:
        result = r.reset(max_velocity=0.05, max_acceleration=0.05)
    return {"success": bool(result.get("success")), "action": "reset", "result": result}


if __name__ == "__main__":
    print("Starting Piper Safe Action Server on port 8890", flush=True)
    uvicorn.run(app, host="0.0.0.0", port=8890)
