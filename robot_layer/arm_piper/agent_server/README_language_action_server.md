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
