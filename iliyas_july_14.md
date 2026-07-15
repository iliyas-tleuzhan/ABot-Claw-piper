# ABot-Claw Piper Progress - July 14

This document records the current progress on the ABot-Claw Piper autonomous pick-and-place pipeline.

## Current Goal

Unblock the real Piper pick-and-place pipeline using the license-free depth fallback GraspAnything backend, while preserving the existing ABot-Claw architecture:

- Agent Server lease/code execution path
- PiperRobotEnv public API
- MoveIt movement path
- official Piper ROS driver
- existing gripper controller
- existing GraspSDK camera-to-base TF logic
- depth fallback GraspAnything service on port 8015

No calibration, YOLO, camera publisher, Piper driver, or perception architecture was replaced during the current fixes.

## Working Stack

The following components were already running and healthy during this work:

- Agent Server: `http://127.0.0.1:8888`
- depth fallback GraspAnything: `http://192.168.1.104:8015/grasp/detect`
- YOLO service: `http://192.168.1.104:8013`
- RealSense D555 camera topics
- saved Easy Handeye transform: `base_link <- table_camera_color_optical_frame`
- MoveIt
- Piper driver
- gripper service
- RViz

## Files Changed In This Session

- `robot_layer/arm_piper/agent_server/robot_sdk/piper_sdk.py`
- `robot_layer/arm_piper/agent_server/piper_trajectory_bridge.py`
- `robot_layer/arm_piper/agent_server/run_depth_fallback_pick_place.py`

The repository also contains unrelated dirty/untracked files. Those were not reset or cleaned.

## Gripper Width Convention Fix

Root cause:

- `PiperRobotEnv.set_gripper(position)` exposes `position` as total physical jaw opening in metres:
  - `0.0` = closed
  - `0.06` = fully open
- MoveIt controls the Piper gripper with `joint7`, which is one finger's prismatic displacement.
- The MoveIt `joint7` URDF limit is approximately:
  - lower: `0.0`
  - upper: `0.035`
- Passing total jaw opening directly to `joint7` caused MoveIt gripper execution failures.

Implemented conversion:

```text
moveit_finger_joint_m = total_opening_width_m / 2.0
total_opening_width_m = moveit_finger_joint_m * 2.0
```

Where implemented:

- `piper_sdk.py`
  - `PiperRobotEnv.set_gripper(position)` still accepts total physical opening.
  - At the MoveIt boundary, total jaw opening is converted to `joint7`.
  - logging now reports:
    - requested total opening
    - clamped total opening
    - converted `moveit_finger_joint_m`
    - service response status
    - service error code
- `piper_trajectory_bridge.py`
  - MoveIt `joint7` trajectory points are converted back to physical total jaw opening before calling `/gripper_srv`.
  - live feedback named `gripper` is treated as physical total jaw opening and divided by 2 for MoveIt feedback units.
Validation:

- `open_gripper` now succeeds with:

```text
set_gripper total_opening_width_m=0.060000
clamped_total_opening_width_m=0.060000
moveit_finger_joint_m=0.030000
status=True
error_code=0
```

- The physical gripper opened.

## Piper Enable Fix

Root cause:

- The official Piper driver ignores arm joint commands unless its internal enable flag is true.
- The gripper service can still accept commands, so the gripper opened while the arm did not move.

Implemented fix:

- `PiperRobotEnv` now calls official `/enable_srv` before arm MoveIt commands.
- The task logs:

```text
Piper enable preflight: /enable_srv enable_response=True
```

Validation:

- After enabling, arm trajectories produced changing `/joint_states_single` feedback.
- A full execute run completed once after this fix.

## Trajectory Bridge Speed Adapter

Root cause of shakiness:

- The official Piper driver does not use normal 6-joint trajectory velocities as expected.
- In `piper_ctrl_single_node.py`, arm speed is taken from `JointState.velocity[6]` as a global hardware speed percentage.
- When the bridge published only six velocity values, the driver fell back to its internal default of about `50%`, causing aggressive/shaky motion despite the pick script using low MoveIt scaling.

Implemented fix:

- `piper_trajectory_bridge.py` now sends a seventh velocity slot for arm commands:

```text
command.velocity = [0, 0, 0, 0, 0, 0, speed_percent]
```

- Current default speed adapter:

```text
arm_min_speed_percent = 10.0
arm_max_speed_percent = 20.0
arm_speed_reference_rad_s = 3.0
```

- The bridge still keeps strict arm feedback enabled.
- Arm final feedback timeout was increased:

```text
feedback_timeout_s = 45.0
```

Reason:

- `3%` speed was too slow and caused `GOAL_TOLERANCE_VIOLATED` before the arm could reach the target.
- `10-20%` is intended to reduce shakiness without timing out on larger moves.

## Planning Failure Fix

Observed failure:

```text
arm/arm: Unable to sample any valid states for goal tree
No solution found after 5.006s
Unable to solve the planning problem
```

Cause:

- A previous aborted slow move left the arm near a low/home-like posture:

```text
end_pose z ~= 0.210 m
joint2/joint3 near lower bounds
```

- The pick script immediately planned to a far top-down pre-grasp pose.
- The plan-validation loop also planned every pose from the current robot state instead of rolling the start state forward through the previously planned trajectory.
- That made later target checks unreliable and sometimes impossible from the real current posture.

Implemented fix in `run_depth_fallback_pick_place.py`:

1. Added a conservative joint-space staging move before pre-grasp:

```text
stage_joints = [0.0, 0.55, -0.75, 0.0, 0.65, 0.0]
```

2. Added CLI overrides:

```text
--stage-j1
--stage-j2
--stage-j3
--stage-j4
--stage-j5
--stage-j6
```

3. Plan-only validation now rolls the start state forward:

- plan `stage` from current state
- set the next planning start state to the final point of `stage`
- plan `pre_grasp` from the staged state
- continue updating the start state through `grasp`, `lift`, `pre_place`, `place`, and `retreat`

4. Execute mode now performs:

```text
open_gripper
move_stage
move_pre_grasp
move_grasp
close_gripper
move_lift
move_pre_place
move_place
open_gripper
move_retreat
```

## Plan-Only Validation Result

Command run:

```bash
./robot_layer/arm_piper/agent_server/run_depth_fallback_pick_place.py \
  --agent-url http://127.0.0.1:8888 \
  --object-name cup \
  --top-k 5 \
  --plan-only \
  --pregrasp-height 0.10 \
  --grasp-z-offset 0.04 \
  --lift-height 0.10 \
  --velocity 0.03 \
  --acceleration 0.03 \
  --timeout 240
```

Result:

```text
PLAN stage success=True points=23
PLAN pre_grasp success=True points=53
PLAN grasp success=True points=11
PLAN lift success=True points=11
PLAN pre_place success=True points=50
PLAN place success=True points=13
PLAN retreat success=True points=13
PLAN_ONLY_COMPLETE no robot movement commanded
```

No robot movement was commanded during this validation.

## Current Recommended Execute Command

Run this next for the physical test:

```bash
./robot_layer/arm_piper/agent_server/run_depth_fallback_pick_place.py \
  --agent-url http://127.0.0.1:8888 \
  --object-name cup \
  --top-k 5 \
  --execute \
  --pregrasp-height 0.10 \
  --grasp-z-offset 0.04 \
  --lift-height 0.10 \
  --velocity 0.03 \
  --acceleration 0.03 \
  --timeout 240
```

Expected behavior:

1. Acquire Agent Server lease.
2. Detect cup with depth fallback service.
3. Transform grasp from `table_camera_color_optical_frame` to `base_link`.
4. Plan all stages.
5. Open gripper.
6. Move to staging joint posture.
7. Move to pre-grasp.
8. Descend.
9. Close gripper.
10. Lift.
11. Move to place.
12. Open gripper.
13. Retreat.

## Important Notes

- No commit or push has been performed.
- No calibration files were intentionally modified for these fixes.
- No robot driver, camera publisher, YOLO service, or depth fallback perception code was changed during the shakiness/planning fixes.
- The current `run_depth_fallback_pick_place.py` file is untracked in Git.
- The current `piper_trajectory_bridge.py` and `piper_sdk.py` files are modified.

## If The Next Run Is Still Shaky

Do not reduce speed below `10%` again without increasing timeouts and checking feedback. The `3%` test showed the robot may not reach target before strict feedback timeout.

Next likely improvement:

- reduce waypoint chatter in `piper_trajectory_bridge.py`
- avoid publishing every tiny MoveIt trajectory point to the Piper driver
- publish a decimated/smoothed command stream while preserving strict final feedback

This should be done in the bridge adapter layer, not by changing perception or calibration.
