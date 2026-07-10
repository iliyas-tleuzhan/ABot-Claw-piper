# Piper Language Action Server

This is a no-camera language/action control layer for AgileX Piper. It only maps simple language-style actions to a clean HTTP server that reads `/joint_states_single` and sends safe MoveIt joint/gripper commands through `PiperRobotEnv`.

It does not use camera, YOLO, GraspAnything, pick-and-place, `/end_pose`, or ABot `/code/execute`.

## Restarting After Closing All Tabs

Docker path prerequisite after the repo move:

```text
/home/dase-hw101/ABot-Claw:/root/ABot-Claw
```

Option A: tmux startup

```bash
cd ~/ABot-Claw/robot_layer/arm_piper/agent_server
./start_piper_language_stack_tmux.sh
```

Option B: print manual commands

```bash
./print_piper_language_startup_commands.sh
```

First tests after startup:

```bash
curl http://localhost:8891/health
curl http://localhost:8891/state
```

Then safe optional tests:

```bash
./test_piper_language_action_server.sh --gripper
./test_piper_language_action_server.sh --move
```

OpenClaw language tests:

```bash
openclaw agent --agent main --message "Move the Piper arm up."
openclaw agent --agent main --message "Move the Piper arm down."
openclaw agent --agent main --message "Open the Piper gripper."
openclaw agent --agent main --message "Close the Piper gripper."
```

## RealSense Color Pick-Place Demo

Start the D555 top-down camera in another container terminal. It publishes under `/table_camera` by default:

```bash
cd /root/ABot-Claw/robot_layer/arm_piper/agent_server
./start_realsense_d555_py.sh
```

This Python publisher uses RealSense depth-to-color alignment. `start_abotclaw_all.sh` uses it by default; `--use-fake-depth` is only a fallback for the raw ROS RealSense publisher plus bridge.

Check RGB, aligned depth, and camera info:

```bash
./check_realsense_topics.sh
```

Run color detection only:

```bash
./today_red_to_purple_pick_place.py
```

The movement path is deliberately gated. Before robot motion, create a real calibration file:

```bash
cp camera_to_base.example.yaml camera_to_base.yaml
```

Replace the example identity matrix with a measured `camera_to_base` transform. Then execute the today-only demo:

```bash
./today_red_to_purple_pick_place.py --execute
```

Do not use `camera_to_base.example.yaml` for movement.

## Rough Manual Camera Calibration

The current calibration workflow uses a rough external-camera pose for detect-only and hover validation. It is not suitable for precise grasping. Generate the initial transform from the measured camera position and tilt:

```bash
cd /root/ABot-Claw/robot_layer/arm_piper/agent_server

source /opt/ros/noetic/setup.bash
source robot_driver_ros/devel/setup.bash

python3 generate_manual_camera_calibration.py \
  --xc 0.35 \
  --yc 0.21 \
  --zc 0.46 \
  --tilt-deg 45.8 \
  --output calibration_1.yaml \
  --print-matrix
```

Validate only the transformed target coordinates before any hover or movement:

```bash
python3 validate_manual_calibration.py \
  --calibration calibration_1.yaml \
  --watch
```

Then use detect-only pick/place output:

```bash
python3 today_red_to_purple_pick_place.py \
  --calibration calibration_1.yaml
```

Before any movement, verify all of the following:

1. Moving the red cup farther from the base changes base X in the expected direction.
2. Moving it left/right changes base Y in the expected direction.
3. Table targets produce plausible and similar base Z values.
4. The result is only a rough transform from ruler and phone-angle measurements.
5. ArUco eye-to-hand calibration remains the intended accurate solution.

`calibrate_camera_to_base_points.py` is retained for point-pair calibration, but it is not the current chosen workflow.

## Blue Tape Point Calibration (Retained)

Use the manual point-pair calibration script to create `camera_to_base.yaml` from blue tape markers:

```bash
cd /root/ABot-Claw/robot_layer/arm_piper/agent_server

source /opt/ros/noetic/setup.bash
source robot_driver_ros/devel/setup.bash

python3 calibrate_camera_to_base_points.py \
  --target-color blue \
  --sample p1,0.370,-0.070,0.000 \
  --sample p2,0.370,0.150,0.000 \
  --sample p3,0.210,0.150,0.000 \
  --sample p4,0.205,-0.055,0.000 \
  --sample p5,0.300,0.020,0.000 \
  --output camera_to_base.yaml \
  --min-target-area 100
```

## Required Terminals

1. Piper ROS driver:

   ```bash
   docker exec -it abot-piper-noetic bash
   cd /root/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros/src/piper_ros
   source /opt/ros/noetic/setup.bash
   source ../../devel/setup.bash
   roslaunch piper start_single_piper.launch can_port:=can0 auto_enable:=true
   ```

2. MoveIt:

   ```bash
   docker exec -it abot-piper-noetic bash
   cd /root/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros
   source /opt/ros/noetic/setup.bash
   source devel/setup.bash
   roslaunch piper_with_gripper_moveit demo.launch use_rviz:=false
   ```

3. Language action server:

   ```bash
   docker exec -it abot-piper-noetic bash
   cd /root/ABot-Claw/robot_layer/arm_piper/agent_server
   ./start_piper_language_action_server.sh
   ```

   The server prints `Starting piper_language_action_server_v1 on port 8891`.

4. Test/OpenClaw terminal:

   ```bash
   docker exec -it abot-piper-noetic bash
   ```

## Curl Tests

Health, first:

```bash
curl http://localhost:8891/health
```

Confirm the response includes:

```json
{"server":"piper_language_action_server_v1","server_revision":"ros_state_cache_20260706"}
```

State, second:

```bash
curl http://localhost:8891/state
```

This must show live `/joint_states_single` values in `joint_positions`, `gripper_position`, `ros_joint_names`, and `ros_joint_positions`. Do not trust the old reset/default pose unless the robot is actually there.

Open gripper:

```bash
curl -X POST http://localhost:8891/open_gripper
```

Close gripper:

```bash
curl -X POST http://localhost:8891/close_gripper
```

Move arm up using the calibrated safe joint nudge:

```bash
curl -X POST http://localhost:8891/move_up \
  -H 'Content-Type: application/json' \
  -d '{"joint_step":0.08,"speed":0.05,"accel":0.05}'
```

Move arm down using the reverse nudge:

```bash
curl -X POST http://localhost:8891/move_down \
  -H 'Content-Type: application/json' \
  -d '{"joint_step":0.08,"speed":0.05,"accel":0.05}'
```

Optional direct gripper position:

```bash
curl -X POST http://localhost:8891/set_gripper \
  -H 'Content-Type: application/json' \
  -d '{"position":0.03,"speed":0.05,"accel":0.05}'
```

`joint_step` is used as requested, without an application-level cap. `move_up` applies `q[1] += abs(joint_step)` and `q[2] -= abs(joint_step)`; `move_down` applies the reverse. MoveIt, controller, and hardware joint limits may still reject unreachable or unsafe targets.

## Working Daily Startup

Terminal 1:

```bash
docker exec -it abot-piper-noetic bash
cd /root/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros
source /opt/ros/noetic/setup.bash
source devel/setup.bash
cd src/piper_ros
bash can_activate.sh can0 1000000
cd /root/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros
roslaunch piper start_single_piper.launch can_port:=can0 auto_enable:=true
```

Terminal 2:

```bash
docker exec -it abot-piper-noetic bash
cd /root/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch piper_with_gripper_moveit demo.launch use_rviz:=false
```

Terminal 3:

```bash
docker exec -it abot-piper-noetic bash
cd /root/ABot-Claw/robot_layer/arm_piper/agent_server
./start_piper_language_action_server.sh
```

Terminal 4:

```bash
curl http://localhost:8891/health
curl http://localhost:8891/state
./test_piper_language_action_server.sh
./test_piper_language_action_server.sh --gripper
./test_piper_language_action_server.sh --move
```

OpenClaw host test:

```bash
cd ~/ABot-Claw/robot_layer/arm_piper/agent_server
./test_openclaw_piper_action.sh
openclaw agent --agent main --message "Move the Piper arm up."
```
