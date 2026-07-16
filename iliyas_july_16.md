# ABot-Claw Piper Notes - July 16

## Baseline

- Parent repository: `/home/dase-hw101/ABot-Claw`
- Parent HEAD during this handoff: `cd2d38d allow small validated voxel perception tolerance`
- Nested Piper ROS repository: `robot_layer/arm_piper/agent_server/robot_driver_ros/src/piper_ros`
- Nested HEAD after committing today: `d539c15 align Piper joint5 limit with hardware`

## Read-only physical state snapshot

Captured without commanding the arm, gripper, reset, or execute.

`/joint_states_single`:

```json
{
  "joint_positions": [-0.572459748, 1.407224924, -1.038057552, -0.123451188, 0.84132412, -0.723960888],
  "joint_velocities": [0.0, 0.0, 0.0, 0.001, -0.205, 0.0],
  "gripper_position_m": 0.0522
}
```

`/end_pose`:

```json
{
  "position": [0.229709, -0.15809, 0.277671],
  "orientation_quat": [-0.10349832999841875, 0.9682196422755155, -0.1766468304704209, 0.1436478934205649]
}
```

`/arm_status`:

```json
{
  "ctrl_mode": 1,
  "arm_status": 0,
  "mode_feedback": 1,
  "teach_status": 2,
  "motion_status": 0,
  "err_code": 0,
  "joint_limit_flags": false,
  "communication_error_flags": false
}
```

Interpretation: the physical arm is not at the original start pose. It is still near the last planned hover/transit region. The gripper feedback is about 52 mm open, so the previous explicit `--close-width 0.052` command was too loose for gripping the cup.

## What worked today

- The full stack was restarted and reached a usable state:
  - ROS master
  - Piper driver
  - RealSense RGB and aligned depth
  - saved hand-eye TF publisher
  - MoveIt
  - RViz non-looping preview
  - Agent Server on port 8888
  - OpenClaw gateway
  - remote YOLO and Grasp services
- The Agent Server executor was confirmed as `lazy-perception-v2`.
- Perception preflight succeeded.
- Cup detection succeeded.
- RViz plan previews displayed accepted trajectory sequences.
- MoveIt planning succeeded for a reachable near-top-down FK workspace candidate.
- The successful execute attempt earlier in the day completed:
  - open gripper
  - transit to hover
  - live vertical descend
  - close gripper
  - live vertical lift

## Problems observed

- The robot was not physically at the assumed start pose for later commands.
- The explicit `--close-width 0.052` left the gripper around 52 mm open and did not grip the cup firmly.
- The planned grasp was too high because the code added `--grasp-z-offset 0.035` to `source_base_z`.
- Perception `source_base_xyz` is the AnyGrasp geometric grasp center, not the table surface. Adding 35 mm made the TCP target stay above the intended grasp point.
- `/end_pose` is the Piper wrist/end-pose topic, not `gripper_tcp`. The manipulation code must keep printing both `/end_pose` and MoveIt `gripper_tcp` diagnostics.

## Fixes made in the working tree

- `run_piper_manipulation.py`
  - Default `--grasp-z-offset` changed from `0.035` to `0.0`.
  - The generated code now prints `CURRENT_PHYSICAL_STATE_JSON` before perception/planning.
  - The generated code now prints `SOURCE_GRASP_HEIGHT_SEMANTICS_JSON`.
  - Saturated cup width can use an object profile close width of `0.035 m` when no explicit `--close-width` is supplied.
  - Explicit `--close-width` still overrides the profile.
  - Terminal `\\n` rendering was fixed earlier so streamed logs are readable.
- `piper_trajectory_bridge.py`
  - The bridge now keeps publishing the accepted final arm target while waiting for `/joint_states_single` feedback.
  - This fixed the case where a previously accepted trajectory reached the physical target only after the final target was held.
- Nested `piper_ros`
  - Joint 5 upper limit is reduced in the working tree to match the observed physical limit more closely.
  - MoveIt joint limits are updated in the working tree for joint 5.
- `start_abotclaw_all.sh`
  - Now delegates to the full-stack launcher by default.
  - Old lower-stack-only behavior remains available as hidden `--lower-only`.
- `start_abotclaw_full_stack.sh`
  - Added `--restart` and `--stop`.
  - `--restart` is intended to kill/restart local tmux sessions, local Docker ROS/Agent/vision processes, the local container, remote YOLO/Grasp processes/container, and OpenClaw gateway before starting the stack fresh.

## Safer next pickup command

Do not use `--close-width 0.052` for cup pickup. Either omit `--close-width` and let the cup profile use 35 mm, or set it explicitly:

```bash
./robot_layer/arm_piper/agent_server/run_piper_manipulation.py \
  --task pick \
  --source cup \
  --source-provider perception \
  --grasp-region auto \
  --hover-height 0.07 \
  --grasp-z-offset 0.0 \
  --close-width 0.035 \
  --execute \
  --timeout 300
```

Plan-only/RViz preview:

```bash
./robot_layer/arm_piper/agent_server/run_piper_manipulation.py \
  --task pick \
  --source cup \
  --source-provider perception \
  --grasp-region auto \
  --hover-height 0.07 \
  --grasp-z-offset 0.0 \
  --close-width 0.035 \
  --plan-only \
  --timeout 300
```

## Startup command for tomorrow

```bash
./start_abotclaw_all.sh --restart
```

Status-only:

```bash
./start_abotclaw_all.sh --status
```

Stop:

```bash
./start_abotclaw_all.sh --stop
```

## Important safety note

Do not run repeated physical execute attempts from one prompt. If one attempt fails or leaves the arm away from the expected start pose, inspect `/joint_states_single`, `/end_pose`, `/arm_status`, and RViz before another physical run.
