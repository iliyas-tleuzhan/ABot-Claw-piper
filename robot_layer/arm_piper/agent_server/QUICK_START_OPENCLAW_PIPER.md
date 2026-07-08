# OpenClaw Piper Quick Start

Use this after rebooting or closing all terminals.

## 1. Start the robot stack

```bash
cd ~/ABot-Claw/robot_layer/arm_piper/agent_server
./start_piper_language_stack_tmux.sh
```

This starts Piper ROS driver, MoveIt, the language action server, and a test pane in `tmux`.

Detach from tmux without stopping it:

```text
Ctrl-b then d
```

Reattach:

```bash
tmux attach -t piper_language_stack
```

## 2. Start the D555 table camera

If the D555 was plugged in after the Docker container started, restart the container first so it can see the camera device:

```bash
tmux kill-session -t piper_language_stack
docker restart abot-piper-noetic
cd ~/ABot-Claw/robot_layer/arm_piper/agent_server
PIPER_TMUX_ATTACH=0 ./start_piper_language_stack_tmux.sh
```

Start RealSense in its own tmux window:

```bash
tmux new-window -t piper_language_stack -n realsense \
  "docker exec -it abot-piper-noetic bash -lc 'cd /root/ABot-Claw/robot_layer/arm_piper/agent_server && ./start_realsense_d555_py.sh'"
```

Check that the table camera topics exist:

```bash
docker exec abot-piper-noetic bash -lc '
source /opt/ros/noetic/setup.bash
source /root/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros/devel/setup.bash
rostopic list | grep table_camera
'
```

You should see:

```text
/table_camera/color/image_raw
/table_camera/aligned_depth_to_color/image_raw
/table_camera/color/camera_info
```

## 3. Verify the robot server

```bash
curl --noproxy '*' http://localhost:8891/health
curl --noproxy '*' http://localhost:8891/state
openclaw status
```

`/state` should show fresh `joint_positions` from `/joint_states_single`.

If `can0` is down, activate it:

```bash
docker exec abot-piper-noetic bash -lc '
cd /root/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros/src/piper_ros
bash can_activate.sh can0 1000000
'
```

## 4. Calibrate camera to Piper base

Do this before any camera-based pick/place motion. The file `camera_to_base.yaml` must be measured; do not use `camera_to_base.example.yaml` for motion.

Place a red target at several known points on the table. Measure each point in Piper `base_link`, in meters:

- `x`: forward from robot base
- `y`: left/right from robot base
- `z`: height above robot base, usually tabletop height or target center height

Run the calibrator with at least 3 non-collinear points; 4 to 8 is better:

```bash
docker exec -it abot-piper-noetic bash -lc '
cd /root/ABot-Claw/robot_layer/arm_piper/agent_server
source /opt/ros/noetic/setup.bash
source robot_driver_ros/devel/setup.bash
./calibrate_camera_to_base_points.py \
  --sample p1,0.25,-0.10,0.02 \
  --sample p2,0.25,0.10,0.02 \
  --sample p3,0.40,-0.10,0.02 \
  --sample p4,0.40,0.10,0.02
'
```

Replace the sample coordinates with your measured coordinates. For each sample, move the red target to that measured table point, then press Enter.

The script writes:

```text
/root/ABot-Claw/robot_layer/arm_piper/agent_server/camera_to_base.yaml
```

Check the reported RMS and max error. Recalibrate if the error is more than a few centimeters.

## 5. Test object detection only

Put the red cup and purple paper on the table, then run:

```bash
docker exec -it abot-piper-noetic bash -lc '
cd /root/ABot-Claw/robot_layer/arm_piper/agent_server
source /opt/ros/noetic/setup.bash
source robot_driver_ros/devel/setup.bash
./today_red_to_purple_pick_place.py
'
```

Check that the printed `base=[x,y,z]` values look physically sane. Also inspect:

```text
/root/ABot-Claw/robot_layer/arm_piper/agent_server/today_pick_place_debug.jpg
```

## 6. Execute red-cup to purple-paper pick/place

Only run this after all of these are true:

- D555 topics are publishing.
- `camera_to_base.yaml` exists and has low calibration error.
- `/state` shows fresh live joint state.
- Piper driver is enabled and has no CAN send failures.
- The detect-only `base=[x,y,z]` coordinates look sane.

Then execute:

```bash
docker exec -it abot-piper-noetic bash -lc '
cd /root/ABot-Claw/robot_layer/arm_piper/agent_server
source /opt/ros/noetic/setup.bash
source robot_driver_ros/devel/setup.bash
./today_red_to_purple_pick_place.py --execute
'
```

## 7. Tell OpenClaw what to do

```bash
openclaw agent --agent main --message "Move the Piper arm up."
openclaw agent --agent main --message "Move the Piper arm down."
openclaw agent --agent main --message "Open the Piper gripper."
openclaw agent --agent main --message "Close the Piper gripper."
```

Arm up/down uses the requested `joint_step` without an application-level cap. The examples below use `0.08` radians with `speed: 0.05`, `accel: 0.05`.

## 8. Direct fallback commands

Use these if OpenClaw model streaming is down but the robot server is alive.

```bash
curl -X POST http://localhost:8891/move_up \
  -H 'Content-Type: application/json' \
  -d '{"joint_step":0.08,"speed":0.05,"accel":0.05}'

curl -X POST http://localhost:8891/move_down \
  -H 'Content-Type: application/json' \
  -d '{"joint_step":0.08,"speed":0.05,"accel":0.05}'

curl -X POST http://localhost:8891/open_gripper
curl -X POST http://localhost:8891/close_gripper
```

## 9. If OpenClaw stream fails

Clear broken proxy variables and restart the gateway:

```bash
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY all_proxy
systemctl --user unset-environment http_proxy https_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY all_proxy
openclaw gateway restart
```

Then retry:

```bash
openclaw agent --agent main --message "Move the Piper arm up."
```
