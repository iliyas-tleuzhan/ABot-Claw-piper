# ABot-Claw Piper Pick-and-Place Runbook

# Overview:

## **What this document is for**

This document is the updated restart-and-reproduce guide for the ABot-Claw Piper pick-and-place work.

It combines:

```Plain Text
ABot-Claw + Piper control
ABot-Claw + YOLO / RealSense streaming
Color-based red cup + purple paper detection
Official AgileX eye-to-hand calibration plan
Tomorrow's real camera_to_base calibration workflow
```

The goal is to replicate the setup tomorrow and then replace the missing calibration file with a real one.

Current practical target:

```Plain Text
RealSense D555 sees red cup + purple paper
  -> script finds pixel center + depth
  -> camera XYZ
  -> real camera_to_base.yaml
  -> Piper base XYZ
  -> Piper moves above object/target
  -> pick/place sequence
```

Important current status:

```Plain Text
RealSense D555: working
/table_camera color image: working
/table_camera aligned depth: working through pyrealsense2 publisher
/table_camera camera_info: working
Red object detection: working
Purple target detection: working
Debug image: working
YOLO on 5090 GPU: working
Continuous YOLO stream: working
Browser MJPEG stream: working
Piper movement server: working through 8891 stack
Main missing piece: real camera_to_base.yaml
```

Do **not** run real execute mode until `camera_to_base.yaml` is real and detect-only base coordinates are reasonable.

---

# System Architecture

## **Piper / robot side**

Current practical Piper path:

```Plain Text
User / script
  -> ABot-Claw Piper script
  -> ROS Noetic inside abot-piper-noetic
  -> Piper driver / MoveIt / Piper service layer
  -> CAN can0
  -> AgileX Piper arm
```

Important machine/container:

```Plain Text
Laptop host: HKU-CPS
Laptop IP on HKU-CPS WiFi: 192.168.1.154
Docker container: abot-piper-noetic
Repo path on host: ~/ABot-Claw
Repo path in Docker: /root/ABot-Claw
Main directory: /root/ABot-Claw/robot_layer/arm_piper/agent_server
ROS distro: Noetic
CAN interface: can0
CAN bitrate: 1000000
```

## **RealSense / camera side**

The RealSense D555 is physically connected to the laptop, not the 5090.

Current camera topics:

```Plain Text
/table_camera/color/image_raw
/table_camera/aligned_depth_to_color/image_raw
/table_camera/color/camera_info
```

Current RealSense start script:

```Plain Text
/root/ABot-Claw/robot_layer/arm_piper/agent_server/start_realsense_d555_py.sh
```

This starts:

```Plain Text
/root/ABot-Claw/robot_layer/arm_piper/agent_server/realsense_d555_py_publisher.py
```

The publisher uses `pyrealsense2`, publishes color, aligned depth, and camera info, and names the camera:

```Plain Text
table_camera
```

## **5090 YOLO side**

The 5090 is used for GPU YOLO inference and live debug viewing.

Current 5090 setup:

```Plain Text
5090 host: master
5090 IP on HKU-CPS WiFi: 192.168.1.104
Repo path on 5090 host: ~/ABot-Claw-piper
Repo path inside 5090 containers: /workspace/ABot-Claw-piper
```

There are two 5090 containers:

```Plain Text
abot-yolo-5090:
  ROS Noetic container
  Reads laptop ROS topics
  Runs stream viewer/client

 yolo-5090-torch:
  PyTorch CUDA container
  Runs YOLO FastAPI service on 5090 GPU
  Service URL: http://127.0.0.1:8013
```

Full YOLO stream architecture:

```Plain Text
Laptop RealSense D555
  -> laptop ROS master at http://192.168.1.154:11311
  -> /table_camera/color/image_raw
  -> 5090 ROS container abot-yolo-5090
  -> sends frame to http://127.0.0.1:8013/detect
  -> 5090 PyTorch container yolo-5090-torch
  -> YOLO on 5090 GPU
  -> annotated frame saved at /tmp/yolo_stream_latest.jpg
  -> browser stream on http://192.168.1.104:8090
```

---

# What We Learned Today

## **1. Detection works, calibration is missing**

The color-based script worked in detect-only mode:

```Bash
python3 today_red_to_purple_pick_place.py
```

Observed output:

```Plain Text
Detect-only mode: no calibration loaded (/root/ABot-Claw/robot_layer/arm_piper/agent_server/camera_to_base.yaml missing)
red: pixel=(220, 250) area=1945.5 depth=0.553 camera=[-0.1625165957269299, 0.11784116058412103, 0.5529999732971191] base=None
purple: pixel=(251, 165) area=4814.0 depth=0.751 camera=[-0.14777713682819532, -0.03993016098196177, 0.7509999871253967] base=None
Wrote debug image: /root/ABot-Claw/robot_layer/arm_piper/agent_server/today_pick_place_debug.jpg
Detect-only mode. Re-run with --execute after checking calibration and debug image.
```

This means:

```Plain Text
Red/purple image detection works.
Depth lookup works.
Camera XYZ calculation works.
Base XYZ is missing because camera_to_base.yaml does not exist yet.
```

## **2. The robot does not know object coordinates automatically**

The camera sees objects in camera coordinates:

```Plain Text
red pixel=(220, 250)
depth=0.553
camera=[-0.1625, 0.1178, 0.553]
```

But Piper moves in robot base coordinates:

```Plain Text
base x = forward/back from Piper base
base y = left/right from Piper base
base z = up/down from Piper base
```

So the required missing file is:

```Plain Text
camera_to_base.yaml
```

It must convert:

```Plain Text
camera XYZ -> Piper base XYZ
```

## **3. Manual marker calibration is possible but not ideal**

We considered using the Piper tip and red marker/cup method.

Problem:

```Plain Text
If the gripper hovers above the marker, /end_pose Z is hover height, not table height.
If different parts of the gripper touch different points, calibration becomes inconsistent.
If gripper rotation changes, the reference point changes.
```

If doing manual calibration, the correct rule would be:

```Plain Text
Use the same physical reference point every time.
Use the same wrist orientation every time.
Use the same low hover height every time.
```

For the gripper reference, use the front/tip/center point indicated by the yellow arrow in the photo, not the blue-arrow body/flange area.

But this method is not the official best method.

## **4. Official AgileX hand-eye calibration is the better method**

The mentor pointed out AgileX has an official Piper hand-eye calibration example.

For our setup, this is:

```Plain Text
eye-to-hand / eye outside hand
```

Meaning:

```Plain Text
Camera is fixed outside the robot.
ArUco marker/board is attached rigidly to the Piper end-effector.
Piper moves through different poses.
Camera sees marker pose.
Piper publishes /end_pose.
Calibration solves robot base <-> camera transform.
```

This should replace the red cup manual calibration idea.

## **5. We need to print an ArUco marker**

Use a black/white ArUco marker.

Practical rule:

```Plain Text
Print it on paper.
Tape/glue the paper to stiff cardboard/foam board/plastic.
Attach the stiff marker board rigidly to the Piper gripper/end-effector.
Do not use loose floppy paper.
Do not let it bend, wobble, or slide.
```

The marker must face the RealSense camera enough to be detected during robot motion.

Need to know:

```Plain Text
marker_id
marker_size in meters
```

Example:

```Plain Text
If printed marker width is 70 mm, marker_size = 0.07
```

## **6. YOLO works, but current pick/place should still use color first**

YOLO on the 5090 works.

However:

```Plain Text
YOLO can detect cup.
YOLO will probably not reliably detect purple paper/file because COCO does not have a clean purple paper class.
```

For first physical pick/place, use current color-based `today_red_to_purple_pick_place.py` first.

YOLO is useful for:

```Plain Text
live visualization
future object detection
GPU inference experiments
checking camera stream remotely
```

---

# Quick Start: Current Laptop Camera + Detection

## **Step 1: Make sure laptop is on correct WiFi**

Laptop must be on:

```Plain Text
HKU-CPS
```

Check route to the 5090:

```Bash
ip route get 192.168.1.104
```

Good output:

```Plain Text
192.168.1.104 dev wlp0s20f3 src 192.168.1.154 uid 1000
```

This means:

```Plain Text
Laptop IP = 192.168.1.154
5090 IP   = 192.168.1.104
```

Bad output example:

```Plain Text
192.168.1.104 via 172.29.0.1 dev wlp0s20f3 src 172.29.24.220
```

Fix:

```Plain Text
Connect laptop to HKU-CPS.
Disable VPN / Proton / Kill Switch if route is hijacked.
Run ip route get again.
```

## **Step 2: Start ROS master on laptop**

If `roscore` is already running at the correct IP, skip this.

Start:

```Bash
export LAPTOP_IP=192.168.1.154

docker exec -it \
  -e ROS_MASTER_URI=http://$LAPTOP_IP:11311 \
  -e ROS_IP=$LAPTOP_IP \
  -e ROS_HOSTNAME=$LAPTOP_IP \
  abot-piper-noetic bash -lc '
source /opt/ros/noetic/setup.bash
roscore
'
```

If you see:

```Plain Text
roscore cannot run as another roscore/master is already running
The ROS_MASTER_URI is http://192.168.1.154:11311/
```

that is okay. It means ROS master is already running at the correct IP.

## **Step 3: Start RealSense D555 publisher**

```Bash
export LAPTOP_IP=192.168.1.154

docker exec -it \
  -e ROS_MASTER_URI=http://$LAPTOP_IP:11311 \
  -e ROS_IP=$LAPTOP_IP \
  -e ROS_HOSTNAME=$LAPTOP_IP \
  abot-piper-noetic bash -lc '
cd /root/ABot-Claw/robot_layer/arm_piper/agent_server
./start_realsense_d555_py.sh
'
```

Expected output:

```Plain Text
Starting pyrealsense2 D555 publisher as /table_camera
```

Leave this terminal running.

## **Step 4: Check camera topics**

In a new laptop terminal:

```Bash
export LAPTOP_IP=192.168.1.154

docker exec -it \
  -e ROS_MASTER_URI=http://$LAPTOP_IP:11311 \
  -e ROS_IP=$LAPTOP_IP \
  -e ROS_HOSTNAME=$LAPTOP_IP \
  abot-piper-noetic bash -lc '
cd /root/ABot-Claw/robot_layer/arm_piper/agent_server
./check_realsense_topics.sh
'
```

Expected:

```Plain Text
PASS color image
PASS aligned depth
PASS camera info
```

## **Step 5: Enter Docker for detection-only**

```Bash
docker exec -it \
  -e ROS_MASTER_URI=http://192.168.1.154:11311 \
  -e ROS_IP=192.168.1.154 \
  -e ROS_HOSTNAME=192.168.1.154 \
  abot-piper-noetic bash
```

Inside Docker:

```Bash
cd /root/ABot-Claw/robot_layer/arm_piper/agent_server
source /opt/ros/noetic/setup.bash
source robot_driver_ros/devel/setup.bash
```

Run detect-only:

```Bash
python3 today_red_to_purple_pick_place.py
```

Expected before calibration:

```Plain Text
base=None
camera_to_base.yaml missing
```

Expected after calibration:

```Plain Text
red ... base=[x, y, z]
purple ... base=[x, y, z]
```

## **Step 6: View debug image**

From laptop host:

```Bash
docker cp abot-piper-noetic:/root/ABot-Claw/robot_layer/arm_piper/agent_server/today_pick_place_debug.jpg ~/today_pick_place_debug.jpg
xdg-open ~/today_pick_place_debug.jpg
```

Check:

```Plain Text
Red box is actually around red cup/object.
Purple box is actually around purple paper/place target.
```

---

# Quick Start: 5090 YOLO Stream

This is for live viewing / YOLO debugging. It is not required for the current color-based pick/place script, but it is useful.

## **Step 1: Start or enter 5090 ROS container**

On 5090 host:

```Bash
docker ps -a
```

If `abot-yolo-5090` is stopped:

```Bash
docker start abot-yolo-5090
docker exec -it abot-yolo-5090 bash
```

If already running:

```Bash
docker exec -it abot-yolo-5090 bash
```

Inside:

```Bash
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://192.168.1.154:11311
export ROS_IP=192.168.1.104

cd /workspace/ABot-Claw-piper
git pull
```

If `git pull` gives dubious ownership:

```Bash
git config --global --add safe.directory /workspace/ABot-Claw-piper
git pull
```

Check laptop camera topics:

```Bash
rostopic list | grep table_camera
```

Expected:

```Plain Text
/table_camera/aligned_depth_to_color/image_raw
/table_camera/color/camera_info
/table_camera/color/image_raw
```

## **Step 2: Start or create PyTorch YOLO container**

On 5090 host, in another terminal:

If `yolo-5090-torch` does not exist:

```Bash
docker run -it \
  --name yolo-5090-torch \
  --gpus all \
  --net=host \
  -v ~/ABot-Claw-piper:/workspace/ABot-Claw-piper \
  pytorch/pytorch:2.7.0-cuda12.8-cudnn9-runtime \
  bash
```

If it already exists:

```Bash
docker start yolo-5090-torch
docker exec -it yolo-5090-torch bash
```

Inside PyTorch container:

```Bash
cd /workspace/ABot-Claw-piper
```

If `git` is missing:

```Bash
apt update
apt install -y git
```

Pull code:

```Bash
git config --global --add safe.directory /workspace/ABot-Claw-piper
git pull
```

Install dependencies if this is a new container:

```Bash
apt update
apt install -y git libxcb1 libgl1 libglib2.0-0 libsm6 libxext6 libxrender1

pip install ultralytics
pip install matplotlib scipy seaborn pandas tqdm thop pycocotools fastapi uvicorn pydantic python-multipart pillow requests opencv-python pyyaml
```

Check CUDA:

```Bash
python - <<'PY'
import torch
print(torch.__version__)
print("cuda:", torch.cuda.is_available())
print(torch.cuda.get_device_name(0) if torch.cuda.is_available() else "NO CUDA")
PY
```

Start YOLO server:

```Bash
cd /workspace/ABot-Claw-piper/service_layer/YOLO
PORT=8013 DEVICE=cuda python main.py
```

Leave it running.

Health check from another 5090 terminal/container:

```Bash
curl http://127.0.0.1:8013/health
```

Good result:

```Plain Text
status: ok
model_loaded: true
device: cuda:0
```

## **Step 3: Start continuous YOLO stream viewer**

Inside `abot-yolo-5090` ROS container:

```Bash
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://192.168.1.154:11311
export ROS_IP=192.168.1.104

cd /workspace/ABot-Claw-piper/robot_layer/arm_piper/agent_server
./start_yolo_stream_viewer_5090.sh --rate 5 --conf 0.25 --no-window
```

This continuously writes:

```Plain Text
/tmp/yolo_stream_latest.jpg
/tmp/yolo_stream_latest.json
```

## **Step 4: Serve live browser stream**

Inside another `abot-yolo-5090` terminal:

```Bash
docker exec -it abot-yolo-5090 bash
```

Inside container:

```Bash
cat > /tmp/serve_yolo_mjpeg.py <<'PY'
#!/usr/bin/env python3
from http.server import BaseHTTPRequestHandler, HTTPServer
import os
import time

IMG_PATH = "/tmp/yolo_stream_latest.jpg"
HOST = "0.0.0.0"
PORT = 8090
FPS = 10

class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path in ["/", "/index.html"]:
            html = f"""
            <html>
            <head>
              <title>ABot-Claw YOLO Stream</title>
              <style>
                body {{ background: #111; color: white; font-family: Arial, sans-serif; text-align: center; }}
                img {{ max-width: 95vw; max-height: 85vh; border: 2px solid #444; }}
              </style>
            </head>
            <body>
              <h2>ABot-Claw YOLO Stream</h2>
              <p>Source: {IMG_PATH}</p>
              <img src="/stream">
            </body>
            </html>
            """
            data = html.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
            return

        if self.path != "/stream":
            self.send_error(404)
            return

        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()

        last_mtime = 0
        delay = 1.0 / FPS

        while True:
            try:
                if not os.path.exists(IMG_PATH):
                    time.sleep(delay)
                    continue

                mtime = os.path.getmtime(IMG_PATH)
                if mtime == last_mtime:
                    time.sleep(delay)
                    continue

                last_mtime = mtime

                with open(IMG_PATH, "rb") as f:
                    frame = f.read()

                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode("utf-8"))
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
                time.sleep(delay)

            except (BrokenPipeError, ConnectionResetError):
                break
            except Exception as e:
                print("stream error:", e)
                time.sleep(1)

print(f"Serving YOLO stream at http://{HOST}:{PORT}")
print(f"Open from laptop: http://192.168.1.104:{PORT}")
HTTPServer((HOST, PORT), Handler).serve_forever()
PY

python3 /tmp/serve_yolo_mjpeg.py
```

Open on laptop browser:

```Plain Text
http://192.168.1.104:8090
```

---

# Official AgileX Eye-to-Hand Calibration Plan

## **What calibration we need**

Our RealSense is outside the robot.

So the correct mode is:

```Plain Text
eye_to_hand
```

Meaning:

```Plain Text
Camera is fixed outside.
ArUco marker/board is fixed on Piper end-effector.
Move Piper to different poses.
Camera observes marker.
Piper publishes /end_pose.
Calibration solves camera/base relationship.
```

This is better than red cup manual calibration.

## **Physical setup**

Print an ArUco marker.

Mounting:

```Plain Text
Print ArUco on paper.
Glue/tape paper to stiff cardboard/foam board/plastic.
Attach board rigidly to Piper gripper/end-effector.
Marker must face RealSense camera.
Marker must not bend, slide, wobble, or rotate relative to gripper.
```

Do not hold marker by hand.

Do not use loose paper.

## **Marker requirements**

Need to know:

```Plain Text
marker_id
marker_size in meters
```

Example:

```Plain Text
Printed marker width = 70 mm
marker_size = 0.07
```

The marker size must be the actual black square marker size, not the whole paper size.

## **ROS2 Humble workspace for AgileX calibration**

AgileX hand-eye tutorial uses:

```Plain Text
Ubuntu 22.04
ROS2 Humble
aruco_ros
piper_ros humble branch
handeye_calibration_ros humble branch
```

Create workspace:

```Bash
mkdir -p ~/handeye/src
cd ~/handeye/src

git clone -b humble-devel https://github.com/pal-robotics/aruco_ros.git
git clone -b humble https://github.com/agilexrobotics/piper_ros.git
git clone -b humble https://github.com/agilexrobotics/handeye_calibration_ros.git
```

Build:

```Bash
cd ~/handeye
colcon build
```

Source:

```Bash
source ~/handeye/install/setup.bash
```

Important: AgileX README showed `source ~/handeye/src/install/setup.sh`, but normal ROS2 workspace install path is usually:

```Plain Text
~/handeye/install/setup.bash
```

Check which one exists.

## **Run ArUco detector**

Example:

```Bash
source ~/handeye/install/setup.bash

ros2 launch aruco_ros single.launch.py eye:=left marker_id:=582 marker_size:=0.0677
```

Replace:

```Plain Text
marker_id:=582
marker_size:=0.0677
```

with your printed marker's actual ID and size.

## **Run RealSense camera in ROS2**

AgileX example remaps RealSense topics to what `aruco_ros` expects:

```Bash
ros2 run realsense2_camera realsense2_camera_node --ros-args \
  -p rgb_camera.color_profile:=640x480x60 \
  --remap /camera/camera/color/image_raw:=/stereo/left/image_rect_color \
  --remap /camera/camera/color/camera_info:=/stereo/left/camera_info
```

If topic names differ on our machine, inspect with:

```Bash
ros2 topic list
```

Then adjust remaps.

## **Run Piper ROS2 driver**

```Bash
bash ~/handeye/src/piper_ros/can_activate.sh
source ~/handeye/install/setup.bash
ros2 launch piper start_single_piper.launch.py can_port:=can0
```

Check:

```Bash
ros2 topic echo /end_pose
```

## **Run hand-eye calibration**

For our outside camera setup, use:

```Bash
ros2 run handeye_calibration_ros handeye_calibration --ros-args \
  -p piper_topic:=/end_pose \
  -p marker_topic:=/aruco_single/pose \
  -p mode:=eye_to_hand
```

Do not use `eye_in_hand` for this setup.

`eye_in_hand` is for camera mounted on robot hand.

## **Move Piper through calibration poses**

During calibration:

```Plain Text
Camera stays still.
ArUco marker stays fixed to Piper end-effector.
Move Piper to many different visible poses.
Marker should be visible in camera for each sample.
Use varied positions and rotations.
Do not move the camera.
Do not move the marker mount.
```

Useful debug commands:

```Bash
ros2 run image_view image_view --ros-args --remap /image:=/aruco_single/result
ros2 topic echo /aruco_single/pose
ros2 topic echo /end_pose
```

## **After calibration**

The result should be a transform involving robot base and camera.

For our current ROS1 pick/place script, we need:

```Plain Text
camera_to_base.yaml
```

Required meaning:

```Plain Text
[base_x, base_y, base_z, 1]^T = camera_to_base × [camera_x, camera_y, camera_z, 1]^T
```

If AgileX package outputs `T_base_cam`, check convention carefully.

If `T_base_cam` means camera pose in base frame / camera-to-base transform, use it directly as `camera_to_base`.

If it maps base points into camera frame, invert it before saving.

Do not guess. Verify by testing one object with detect-only and checking base coordinates.

---

# camera_to_base.yaml Format

The active pick/place script expects a real calibration file at:

```Plain Text
/root/ABot-Claw/robot_layer/arm_piper/agent_server/camera_to_base.yaml
```

Expected format:

```YAML
camera_to_base:
  - [r11, r12, r13, tx]
  - [r21, r22, r23, ty]
  - [r31, r32, r33, tz]
  - [0.0, 0.0, 0.0, 1.0]
```

Meaning:

```Plain Text
base_point = camera_to_base × camera_point
```

Projection before transform:

```Plain Text
camera_x = (u - cx) * depth / fx
camera_y = (v - cy) * depth / fy
camera_z = depth
```

Camera intrinsics come from:

```Plain Text
/table_camera/color/camera_info
```

Depth comes from:

```Plain Text
/table_camera/aligned_depth_to_color/image_raw
```

## **Sanity checks after real calibration**

Run:

```Bash
cd /root/ABot-Claw/robot_layer/arm_piper/agent_server
python3 today_red_to_purple_pick_place.py
```

Good output should look like:

```Plain Text
red ... base=[0.20 to 0.55, -0.30 to 0.30, reasonable table/object height]
purple ... base=[0.20 to 0.55, -0.30 to 0.30, reasonable table/object height]
```

Bad signs:

```Plain Text
base=None
base x negative
base x > 0.7
base y outside +/-0.35
base z around 0.6 to 0.9 for tabletop object
large jumps between runs while object is not moving
```

If bad, do not execute. Fix/invert calibration.

---

# Piper Movement / Pick-Place Startup

Use this only after RealSense and calibration are ready.

## **Start Piper stack**

On laptop host:

```Bash
cd ~/ABot-Claw/robot_layer/arm_piper/agent_server
./start_piper_language_stack_tmux.sh
```

Check:

```Bash
curl http://localhost:8891/health
curl http://localhost:8891/state
```

Test movement:

```Bash
./test_piper_language_action_server.sh --move
```

Test gripper only if needed:

```Bash
./test_piper_language_action_server.sh --gripper
```

Current note:

```Plain Text
Codex was asked to remove gripper commands from the current pick/place path.
The pick/place script should not open/close the gripper for now.
The script should also not use artificial joint movement limits.
Physical Piper limits are enough for this stage.
```

## **Run detect-only before execute**

Inside laptop Docker:

```Bash
cd /root/ABot-Claw/robot_layer/arm_piper/agent_server
source /opt/ros/noetic/setup.bash
source robot_driver_ros/devel/setup.bash
python3 today_red_to_purple_pick_place.py
```

Check:

```Plain Text
red detected correctly
purple detected correctly
base coordinates plausible
debug image correct
```

Only then consider execute.

## **First motion should be hover-only if available**

Before real pick/place, the safer test is:

```Plain Text
move above red object
pause
move above purple target
pause
return/stop
```

No descend. No gripper movement.

If the current script does not have hover-only mode, add it before full execute.

## **Execute mode**

Only after calibration is verified:

```Bash
python3 today_red_to_purple_pick_place.py --execute
```

Watch the robot and keep power cutoff nearby.

Expected high-level sequence:

```Plain Text
move to pre-pick
move to pick / approach
lift
move to pre-place
move to place / approach
retreat
```

Because gripper movement is removed, this is currently mostly an arm path test unless the gripper is manually prepared.

---

# Troubleshooting

## **Problem 1: RealSense says no device connected**

Symptom:

```Plain Text
RuntimeError: No device connected
```

Fix on laptop host:

```Bash
lsusb | grep -i -E "intel|realsense|8086"
ls /dev/video*
```

If host does not see it:

```Plain Text
Unplug/replug D555.
Use good USB 3 port/cable.
```

Check Docker sees it:

```Bash
docker exec -it abot-piper-noetic bash -lc '
lsusb | grep -i -E "intel|realsense|8086" || true
ls /dev/video* 2>/dev/null || true
'
```

If needed:

```Bash
docker restart abot-piper-noetic
```

## **Problem 2: RealSense device busy**

Symptom:

```Plain Text
xioctl(VIDIOC_S_FMT) failed, errno=16
Device or resource busy
```

Fix:

```Bash
pkill -f realsense-viewer || true
pkill -f realsense || true
pkill -f pyrealsense || true
pkill -f realsense2_camera || true

docker exec -it abot-piper-noetic bash -lc '
pkill -f realsense || true
pkill -f pyrealsense || true
pkill -f realsense2_camera || true
pkill -f rs_rgbd || true
pkill -f rs_camera || true
'
```

If still busy, unplug/replug camera.

## **Problem 3: ROS master already running**

Symptom:

```Plain Text
roscore cannot run as another roscore/master is already running
```

If URI is correct:

```Plain Text
http://192.168.1.154:11311
```

then skip roscore and continue.

To force reset:

```Bash
docker exec -it abot-piper-noetic bash -lc '
pkill -f roscore || true
pkill -f rosmaster || true
pkill -f rosout || true
pkill -f roslaunch || true
'
```

## **Problem 4: Typed `>` into Docker command**

Bad:

```Bash
docker exec -it > -e ROS_MASTER_URI=...
```

Fix:

```Plain Text
Do not type the > symbols.
They are shell continuation prompts.
```

Correct:

```Bash
docker exec -it \
  -e ROS_MASTER_URI=http://192.168.1.154:11311 \
  -e ROS_IP=192.168.1.154 \
  -e ROS_HOSTNAME=192.168.1.154 \
  abot-piper-noetic bash -lc '
cd /root/ABot-Claw/robot_layer/arm_piper/agent_server
./start_realsense_d555_py.sh
'
```

## **Problem 5: Git dubious ownership in Docker**

Symptom:

```Plain Text
fatal: detected dubious ownership in repository
```

Fix:

```Bash
git config --global --add safe.directory /root/ABot-Claw
cd /root/ABot-Claw
git pull
```

For 5090 container:

```Bash
git config --global --add safe.directory /workspace/ABot-Claw-piper
cd /workspace/ABot-Claw-piper
git pull
```

## **Problem 6: 5090 PyTorch container has no git**

Symptom:

```Plain Text
bash: git: command not found
```

Fix inside PyTorch container:

```Bash
apt update
apt install -y git
```

Or pull on 5090 host:

```Bash
cd ~/ABot-Claw-piper
git pull
```

The container sees the updated files through the mounted volume.

## **Problem 7: PyTorch install fails in ROS container**

Symptom:

```Plain Text
ERROR: Could not find a version that satisfies the requirement torch
```

Cause:

```Plain Text
ROS Noetic container uses Python 3.8.
Modern PyTorch CUDA wheels need newer Python.
```

Fix:

```Plain Text
Do not install PyTorch in abot-yolo-5090.
Use yolo-5090-torch for CUDA/YOLO.
```

## **Problem 8: Missing ultralytics**

Symptom:

```Plain Text
ModuleNotFoundError: No module named 'ultralytics'
```

Fix in PyTorch container:

```Bash
pip install ultralytics
```

## **Problem 9: Missing libxcb**

Symptom:

```Plain Text
ImportError: libxcb.so.1: cannot open shared object file
```

Fix:

```Bash
apt update
apt install -y libxcb1 libgl1 libglib2.0-0 libsm6 libxext6 libxrender1
```

Alternative:

```Bash
pip uninstall -y opencv-python opencv-contrib-python
pip install opencv-python-headless
```

## **Problem 10: YOLO wrong path**

Wrong:

```Plain Text
service_layer/Services/YOLO/yolo_http_server_5090.py
```

Correct:

```Plain Text
service_layer/YOLO/main.py
```

Start:

```Bash
cd /workspace/ABot-Claw-piper/service_layer/YOLO
PORT=8013 DEVICE=cuda python main.py
```

## **Problem 11: 5090 ROS container cannot see camera topics**

Inside `abot-yolo-5090`:

```Bash
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://192.168.1.154:11311
export ROS_IP=192.168.1.104
rostopic list | grep table_camera
```

If no topics:

```Plain Text
Check laptop roscore.
Check laptop RealSense publisher.
Check both machines are on HKU-CPS.
Check ping both ways.
Check VPN is off.
Check ROS_MASTER_URI uses laptop IP.
Check ROS_IP uses current machine IP.
```

## **Problem 12: Browser stream does not open**

Check on laptop:

```Bash
curl http://192.168.1.104:8090
```

If it fails:

```Plain Text
MJPEG server not running.
Port blocked.
Wrong 5090 IP.
Container not using host network.
```

You can still manually copy image:

```Bash
docker cp abot-yolo-5090:/tmp/yolo_stream_latest.jpg ~/yolo_stream_latest.jpg
```

## **Problem 13: base=None in pick/place script**

Symptom:

```Plain Text
base=None
camera_to_base.yaml missing
```

Fix:

```Plain Text
Create real camera_to_base.yaml from AgileX hand-eye calibration.
Place it in /root/ABot-Claw/robot_layer/arm_piper/agent_server/camera_to_base.yaml.
```

## **Problem 14: base coordinates insane**

Bad examples:

```Plain Text
base x negative
base x > 0.7
base y outside +/-0.35
base z = 0.6 to 0.9 for tabletop cup
```

Fix:

```Plain Text
Check transform direction.
Maybe invert T_base_cam.
Check marker_size.
Check camera_info.
Check marker mount rigidity.
Rerun calibration.
```

---

# Tomorrow Replication Checklist

Do this order.

## **A. Camera and detection**

```Plain Text
1. Laptop on HKU-CPS.
2. Confirm laptop IP 192.168.1.154.
3. Start/confirm roscore.
4. Start RealSense D555 pyrealsense2 publisher.
5. Run check_realsense_topics.sh.
6. Run today_red_to_purple_pick_place.py detect-only.
7. Check debug image.
```

## **B. 5090 stream if needed**

```Plain Text
1. Start yolo-5090-torch.
2. Start YOLO service on port 8013.
3. Start abot-yolo-5090 ROS container.
4. Start stream viewer.
5. Start MJPEG server on 8090.
6. Open http://192.168.1.104:8090.
```

## **C. Official calibration**

```Plain Text
1. Print ArUco marker.
2. Mount on stiff board.
3. Attach rigidly to Piper end-effector.
4. RealSense fixed outside robot.
5. Set up ROS2 Humble handeye workspace.
6. Run RealSense ROS2 node.
7. Run aruco_ros with correct marker_id and marker_size.
8. Run Piper ROS2 driver.
9. Run handeye_calibration_ros with mode:=eye_to_hand.
10. Save output transform.
11. Convert to camera_to_base.yaml.
12. Copy into /root/ABot-Claw/robot_layer/arm_piper/agent_server/camera_to_base.yaml.
```

## **D. Pick/place**

```Plain Text
1. Start Piper stack.
2. Test 8891 health/state.
3. Test small motion.
4. Run detect-only with real camera_to_base.yaml.
5. Check base coordinates.
6. Run hover-only if available.
7. Only then run --execute.
```

---

# Final Short Version

Laptop camera:

```Bash
export LAPTOP_IP=192.168.1.154

docker exec -it \
  -e ROS_MASTER_URI=http://$LAPTOP_IP:11311 \
  -e ROS_IP=$LAPTOP_IP \
  -e ROS_HOSTNAME=$LAPTOP_IP \
  abot-piper-noetic bash -lc '
cd /root/ABot-Claw/robot_layer/arm_piper/agent_server
./start_realsense_d555_py.sh
'
```

Laptop detect-only:

```Bash
docker exec -it \
  -e ROS_MASTER_URI=http://192.168.1.154:11311 \
  -e ROS_IP=192.168.1.154 \
  -e ROS_HOSTNAME=192.168.1.154 \
  abot-piper-noetic bash
```

Inside:

```Bash
cd /root/ABot-Claw/robot_layer/arm_piper/agent_server
source /opt/ros/noetic/setup.bash
source robot_driver_ros/devel/setup.bash
python3 today_red_to_purple_pick_place.py
```

5090 YOLO server:

```Bash
docker start yolo-5090-torch
docker exec -it yolo-5090-torch bash
cd /workspace/ABot-Claw-piper/service_layer/YOLO
PORT=8013 DEVICE=cuda python main.py
```

5090 stream viewer:

```Bash
docker start abot-yolo-5090
docker exec -it abot-yolo-5090 bash
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://192.168.1.154:11311
export ROS_IP=192.168.1.104
cd /workspace/ABot-Claw-piper/robot_layer/arm_piper/agent_server
./start_yolo_stream_viewer_5090.sh --rate 5 --conf 0.25 --no-window
```

Open browser:

```Plain Text
http://192.168.1.104:8090
```

Real calibration file location:

```Plain Text
/root/ABot-Claw/robot_layer/arm_piper/agent_server/camera_to_base.yaml
```

Only execute after detect-only prints plausible base coordinates:

```Bash
python3 today_red_to_purple_pick_place.py --execute
```

---

# Critical Safety Rule

Do not run physical execute mode until all of these are true:

```Plain Text
RealSense topics pass.
Red/purple detection is correct in debug image.
camera_to_base.yaml is real, not example/fake.
Detect-only base coordinates are plausible.
Piper small movement test works.
Workspace is clear.
Power cutoff is nearby.
First test is hover/small motion, not full aggressive pick/place.
```

---

# Current Infrastructure Update

This section supersedes the older `start_piper_language_stack_tmux.sh` / port
8891 startup advice above for physical Piper work. That older script launches
`piper_with_gripper_moveit/demo.launch`, whose default controller manager is
fake. A fake MoveIt controller can report successful execution without sending
a CAN command to the Piper arm.

## One-command startup

Use the host-side launcher from the repository root:

```bash
cd /root/ABot-Claw
./start_abotclaw_all.sh --restart
```

Attach to the running infrastructure:

```bash
tmux attach -t abotclaw
```

Useful lifecycle commands:

```bash
./start_abotclaw_all.sh --status
./start_abotclaw_all.sh --stop
```

The default stack starts:

```Plain Text
roscore
Piper CAN driver on can0 at 1000000 bit/s
joint-state relay (/joint_states_single -> /joint_states)
RealSense D555 Python publisher with real depth-to-color alignment
Piper FollowJointTrajectory bridge
MoveIt with the real simple controller manager
joint_moveit_ctrl services
health checks
```

It starts infrastructure only. It does not run a motion script. The default
camera publisher is `start_realsense_d555_py.sh`, which uses
`realsense_d555_py_publisher.py` and real `rs.align(rs.stream.color)` alignment.
Use `--use-fake-depth` only as an explicit fallback.

The hardware trajectory path is:

```Plain Text
today_red_to_purple_pick_place.py
  -> joint_moveit_ctrl_endpose
  -> MoveIt simple controller manager
  -> /arm_controllers/follow_joint_trajectory
  -> piper_trajectory_bridge.py
  -> /piper_joint_commands
  -> Piper ROS driver
  -> can0
  -> Piper arm
```

The trajectory bridge waits for live `/joint_states_single` feedback before it
reports a stage complete. MoveIt is configured with a longer real-hardware
execution watchdog and start-state tolerance so slow Piper feedback is not
cancelled as a fake/simulation trajectory.

## RealSense health checks

After startup, the following topics must have one active publisher from the
RealSense Python publisher:

```Plain Text
/table_camera/color/image_raw
/table_camera/aligned_depth_to_color/image_raw
/table_camera/color/camera_info
```

Run the stack health window or check manually inside the container:

```bash
source /opt/ros/noetic/setup.bash
source /root/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros/devel/setup.bash
rostopic info /table_camera/color/image_raw
rostopic info /table_camera/aligned_depth_to_color/image_raw
rostopic info /table_camera/color/camera_info
```

## RViz visualization

RViz is installed in the Noetic image, but the existing `abot-piper-noetic`
container was not created with the host X11 socket mounted. Do not pass `-v` to
`docker exec`; volume mounts are only valid for `docker run`.

Start the ABot-Claw stack first so `roscore` is available, then from a desktop
terminal run:

```bash
xhost +si:localuser:root

docker run --rm -it \
  --name abot-piper-rviz \
  --network host \
  --privileged \
  -e DISPLAY="$DISPLAY" \
  -e ROS_MASTER_URI=http://localhost:11311 \
  -e ROS_HOSTNAME=localhost \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/dase-hw101/ABot-Claw:/root/ABot-Claw \
  abot-piper-noetic:pre-remount-20260707-162935 \
  bash -lc '
source /opt/ros/noetic/setup.bash
source /root/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros/devel/setup.bash
rviz
'
```

If RViz says it cannot connect to the ROS master, the Docker container may be
running but `roscore` is not. Start `./start_abotclaw_all.sh --restart` first.

## Rough manual calibration status

`generate_manual_camera_calibration.py` can generate a valid rigid 4x4 matrix
from camera position, tilt, aim point, and roll. The current `calibration_1.yaml`
was generated with rough external measurements:

```bash
cd /root/ABot-Claw/robot_layer/arm_piper/agent_server

source /opt/ros/noetic/setup.bash
source robot_driver_ros/devel/setup.bash

python3 generate_manual_camera_calibration.py \
  --xc 0.35 \
  --yc 0.21 \
  --zc 0.46 \
  --tilt-deg 45.8 \
  --aim-x 0.16 \
  --aim-y 0.47 \
  --aim-z 0.00 \
  --output calibration_1.yaml \
  --print-matrix
```

This transform is not yet valid for real object pickup. Current detect-only
output mapped the purple object to an unreachable base-frame location, so normal
calibrated `--execute` must remain disabled until a measured calibration is
available. The failed blue-tape artifact has a large RMS error and must not be
used as a replacement.

For normal-mode coordinate validation only:

```bash
python3 validate_manual_calibration.py \
  --calibration calibration_1.yaml \
  --watch
```

The intended accurate solution remains an ArUco eye-to-hand calibration or a
fresh set of reliable point-pair correspondences.

## Safe fake-target pipeline test

When real calibration is unavailable, use the explicit pipeline test mode. It
still reads the real RealSense topics, detects red and purple, prints their real
pixel/depth/camera XYZ values, and writes the debug image. It bypasses
camera-to-base coordinates only for robot motion.

Default fake base-frame targets:

```Plain Text
fake pick:  [0.30, 0.00, 0.25]
fake place: [0.30, 0.10, 0.25]
approach:   0.05 m
```

The generated path is deliberately in open air and never lowers below Z=0.25 m:

```Plain Text
pre-pick:  [0.30, 0.00, 0.30]
pick:      [0.30, 0.00, 0.25]
lift:      [0.30, 0.00, 0.30]
pre-place: [0.30, 0.10, 0.30]
place:     [0.30, 0.10, 0.25]
retreat:   [0.30, 0.10, 0.30]
```

No-motion test. This performs real perception and plans all six fake targets,
but sends no robot command:

```bash
cd /root/ABot-Claw/robot_layer/arm_piper/agent_server
python3 today_red_to_purple_pick_place.py \
  --pipeline-test
```

Slow physical pipeline test. The script plans all six stages before the first
motion, checks for the real controller manager, never operates the gripper, and
stops at the first stage that fails:

```bash
python3 today_red_to_purple_pick_place.py \
  --pipeline-test \
  --execute \
  --speed 0.02 \
  --accel 0.02
```

This tests perception -> task construction -> planning -> real trajectory
execution. It is not an object pickup and does not validate camera calibration.
