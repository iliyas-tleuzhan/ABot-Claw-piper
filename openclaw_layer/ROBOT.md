---

## summary: "AbotClaw robot fleet reference"
read_when:
  - Bootstrapping a workspace manually

# ROBOT.md - About the Robot Fleet

This workspace is built for a **multi-robot fleet**, not a single embodiment.

## Fleet Connection Placeholders

Fill only the **Base URL** column. The guide and SDK URLs are derived from it.


| Robot       | Base URL           | Getting-started guide              | SDK reference                        |
| ----------- | ------------------ | ---------------------------------- | ------------------------------------ |
| Piper       | `http://localhost:8888` | `http://localhost:8888/docs/guide/html` | `http://localhost:8888/code/sdk/markdown` |
| PiPER-X     | `http://127.0.0.1:8893` | `robot_layer/arm_piper_x/agent_server/README_AGENT_SERVER.md` | `robot_layer/arm_piper_x/agent_server/config/piper_x_robot_contract.yaml` |
| Unitree G1  | `<G1_BASE_URL>`    | `<G1_BASE_URL>/docs/guide/html`    | `<G1_BASE_URL>/code/sdk/markdown`    |
| Unitree Go2 | `<GO2_BASE_URL>`   | `<GO2_BASE_URL>/docs/guide/html`   | `<GO2_BASE_URL>/code/sdk/markdown`   |


### Recommended placeholders

- `PIPER_BASE_URL=http://localhost:8888`
- `PIPER_X_AGENT_URL=http://127.0.0.1:8893`
- `PIPER_X_MARKER_API_URL=http://127.0.0.1:8892`
- `G1_BASE_URL=http://<G1_HOST>:<G1_PORT>`
- `GO2_BASE_URL=http://<GO2_HOST>:<GO2_PORT>`

## Fleet Overview

### 1. Piper

- Role: fixed-base manipulation
- Strengths: stable tabletop reach, repeatable grasping, station-based operation
- Best for: picking, placing, sorting, pressing, tool interaction near a fixed workcell
- Limits: no mobility, workspace constrained by mounting position and arm reach

### 1B. PiPER-X

- Role: wrist-camera marker approach and geometric touch on the AgileX PiPER-X arm
- Current stack: ROS 2 Jazzy `agx_arm_ros`, MoveIt, RealSense D435i, ArUco ID 6, PiPER-X Agent Server on `127.0.0.1:8893`, and lower-level marker bridge on `127.0.0.1:8892`
- Best for: `approach the marker`, `touch the marker`, `go home`, and `save current pose as home`
- Limits: current ABot-Claw PiPER-X support is not generic tabletop pick/place; contact is geometric only and not force-confirmed
- Do not use the regular Piper Agent Server on `127.0.0.1:8888` for PiPER-X

### 2. Unitree G1

- Role: humanoid interaction and whole-body task execution
- Strengths: human-scale reach, upright embodiment, richer interaction possibilities
- Best for: tasks designed around human environments, upper-body interaction, demonstrations, teleop-assisted sequences
- Limits: balance, whole-body safety, more complex motion planning, higher execution risk than a fixed arm

### 3. Unitree Go2

- Role: mobile scouting and environmental coverage
- Strengths: locomotion, patrol, following, mobile sensing, inspection from place to place
- Best for: navigation, scene checks, route traversal, remote observation, bringing perception closer to a target area
- Limits: not a primary precision manipulator, mobility safety and terrain constraints must be considered

## Operating Principle

Treat these robots as complementary:

- Use **Go2** to go somewhere, inspect, or gather context
- Use **G1** when a task needs human-like embodiment or richer interaction
- Use **Piper** when the task can be brought to a stable manipulation station
- Use **PiPER-X** when the task is the wrist-camera ArUco marker approach/touch/home demo

## Important Differences from Single-Robot Setups

- Do not assume one shared SDK shape across all robots
- Do not assume one common camera arrangement
- Do not assume one common coordinate frame
- Do not assume one shared safety envelope
- Do not assume regular Piper and PiPER-X share the same runtime. Regular Piper uses the Agent Server path; PiPER-X uses the ROS 2 `agx_arm_ros` path documented under `robot_layer/arm_piper_x`.

## SDK Discovery Rule

Do not duplicate SDK query logic here.

For any robot-facing coding task:

1. Fill in the correct base URL above
2. Use `abotclaw-sdk-discovery` to find the real SDK/docs/examples
3. Only then write code

## Required Local Notes

Fill these in before serious deployment:

- Control/API endpoint for Piper: `http://localhost:8888`
- Control/API endpoint for PiPER-X Agent Server: `http://127.0.0.1:8893`
- Low-level PiPER-X marker/home bridge: `http://127.0.0.1:8892`
- Control/API endpoint for Unitree G1:
- Control/API endpoint for Unitree Go2:
- Auth method / API key details:
- Camera list for each robot:
- E-stop / recovery procedure for each robot:
- Teleoperation fallback path:

## Current PiPER-X Local Notes

- Arm: AgileX PiPER-X
- End effector: AgileX parallel gripper
- ROS: Jazzy
- Driver: `agx_arm_ctrl`, `arm_type:=piper_x`, `effector_type:=agx_gripper`, `fw_version:=v189`
- MoveIt: `agx_arm_moveit`, group `arm`, tip `tcp_link`
- TCP offset: `[0.0, 0.0, 0.1425, 0.0, 0.0, 0.0]`
- Feedback topic: `/feedback/joint_states`
- Camera: wrist RealSense D435i
- ArUco: ID `6`, size `0.10 m`
- Point cloud: `/camera/camera/depth/color/points`
- Skill: `abotclaw-piper-x-manipulation` or short alias `piper-touch-marker`

## Skill Design Reminder

Every robot-facing skill should state:

1. Which robot it targets
2. What assumptions it makes about sensors and actuators
3. What safety checks should happen before execution
4. Whether it can run unattended or needs supervision
