---
name: abotclaw-piper-x-manipulation
description: Route PiPER-X marker, home-pose, and gripper commands through the PiPER-X Agent Server on 127.0.0.1:8893. Use when the user asks the PiPER-X arm to approach/touch ArUco marker 6, press the marked location, go home, save the current pose as home, open the gripper, or close the gripper.
---

# AbotClaw PiPER-X Manipulation

Use this skill for the AgileX PiPER-X arm. This is not the regular Piper
workcell stack.

## Active PiPER-X Contract

- Robot: AgileX PiPER-X
- Driver: ROS 2 Jazzy `agx_arm_ros`
- Arm launch argument: `arm_type:=piper_x`
- End effector: `effector_type:=agx_gripper`
- Firmware argument: `fw_version:=v189`
- CAN: `can0` at 1 Mbps
- MoveIt group: `arm`
- MoveIt tip/TCP: `tcp_link`
- TCP offset: `[0.0, 0.0, 0.1425, 0.0, 0.0, 0.0]`
- Joint feedback: `/feedback/joint_states`
- Gripper command: `/control/joint_states`
- Gripper joint: `gripper`
- Gripper width range: `[0.0, 0.1] m`
- Gripper effort range: `[0.5, 3.0] N`
- Wrist camera: Intel RealSense D435i
- Point cloud: `/camera/camera/depth/color/points`
- ArUco pose: `/aruco_single/pose`
- Marker: ID `6`, size `0.10 m`
- PiPER-X Agent Server: `http://127.0.0.1:8893`
- Low-level marker bridge: `http://127.0.0.1:8892`

The Agent Server is under
`robot_layer/arm_piper_x/agent_server`. It wraps the lower-level ROS 2 package
`piper_x_aruco_wall_approach`, which uses ArUco, RealSense depth, hand-eye TF,
and MoveIt to approach a wall marker.

## Required Health Check

Always call health first:

```bash
cd /home/dase-hw101/ABot-Claw &&
python3 robot_layer/arm_piper_x/agent_server/run_piper_x_agent_task.py health
```

For `approach` and `touch`, require:

- `ros_ok: true`
- `marker_pose_available: true`
- `point_cloud_available: true`
- `moveit_available: true`
- `marker_task_service_available: true`
- `joint_state_available: true`
- `execution_allowed: true` for physical execution
- a valid `/lease/acquire` lease for physical execution through the Agent Server.
  The `run_piper_x_agent_task.py` helper acquires and releases this lease
  automatically when `--execute` is used without `--lease-id`.

For `go home`, marker and point-cloud readiness are not required. Require:

- `ros_ok: true`
- `home_action_available: true`
- `joint_state_available: true`
- `execution_allowed: true` for physical execution

For `save current pose as home`, require fresh joint state. This command does
not move the robot.

For `open gripper` and `close gripper`, marker, point-cloud, and camera
readiness are not required. Require:

- `ros_ok: true`
- `gripper_control.supported: true`
- an active ROS 2 driver subscriber on `/control/joint_states` for physical
  execution
- `execution_allowed: true` for physical execution
- a valid `/lease/acquire` lease for physical execution through the Agent
  Server

## Routing

For "approach the marker", "point at the marker", or "move to the marker":

```bash
cd /home/dase-hw101/ABot-Claw &&
python3 robot_layer/arm_piper_x/agent_server/run_piper_x_agent_task.py \
  approach \
  --execute
```

For "touch the marker" or "press the marked location", use the geometric touch
flow. It moves to pre-touch, performs a slow final approach, retracts, then
returns to saved home:

```bash
cd /home/dase-hw101/ABot-Claw &&
python3 robot_layer/arm_piper_x/agent_server/run_piper_x_agent_task.py \
  touch \
  --execute \
  --retract \
  --return-home-after
```

For "go home":

```bash
cd /home/dase-hw101/ABot-Claw &&
python3 robot_layer/arm_piper_x/agent_server/run_piper_x_agent_task.py \
  home \
  --execute
```

For "save current pose as home" or "remember current pose as home":

```bash
cd /home/dase-hw101/ABot-Claw &&
python3 robot_layer/arm_piper_x/agent_server/run_piper_x_agent_task.py \
  save-home
```

For "open the gripper", "release the gripper", or "open the claw":

```bash
cd /home/dase-hw101/ABot-Claw &&
python3 robot_layer/arm_piper_x/agent_server/run_piper_x_agent_task.py \
  open-gripper \
  --execute
```

For "close the gripper", "close the claw", or "shut the gripper":

```bash
cd /home/dase-hw101/ABot-Claw &&
python3 robot_layer/arm_piper_x/agent_server/run_piper_x_agent_task.py \
  close-gripper \
  --execute
```

## Parser

Use `scripts/piper_x_marker_task.py` to classify language into one of:

- `approach`
- `touch`
- `home`
- `save-home`
- `open-gripper`
- `close-gripper`

Example:

```bash
python3 /home/dase-hw101/.openclaw/workspace/skills/abotclaw-piper-x-manipulation/scripts/piper_x_marker_task.py \
  "touch the marker"
```

## Safety Rules

- Do not use `robot_layer/arm_piper` for PiPER-X.
- Do not call the regular Piper Agent Server at `127.0.0.1:8888` for PiPER-X.
- Use `127.0.0.1:8893` for PiPER-X Agent Server commands.
- Treat `127.0.0.1:8892` as the lower-level marker/home bridge, not the OpenClaw-facing Agent Server.
- Do not import or call regular Piper `piper_sdk` from this skill.
- Do not generate arbitrary joint, CAN, gripper, or MoveIt code.
- For gripper commands, use only `/tools/open-gripper` and
  `/tools/close-gripper`; do not publish ROS messages directly from OpenClaw.
- Do not retry physical commands automatically.
- Do not call `execute=true` when `/health` reports `execution_allowed=false`.
- Report the exact failed `stage` and `message` returned by the API.
- Never claim force-confirmed contact. Report:

```json
{
  "contact_confirmed": false,
  "completion_type": "geometric_surface_approach"
}
```
