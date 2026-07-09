# Continuous YOLO stream viewer on the 5090

This pipeline keeps camera capture on the laptop and YOLO inference on the
5090 GPU. The 5090 ROS container subscribes to the laptop's ROS image topic and
sends rate-limited frames to the YOLO HTTP service in the 5090 PyTorch
container. It does not run YOLO locally.

## Laptop (`192.168.1.154`)

Set the laptop address:

```bash
export LAPTOP_IP=192.168.1.154
```

Start `roscore` if it is not already running:

```bash
roscore
```

Start the existing RealSense D555 publisher in `abot-piper-noetic`. It must
publish:

```text
/table_camera/color/image_raw
/table_camera/aligned_depth_to_color/image_raw
/table_camera/color/camera_info
```

## 5090 PyTorch container

Start the YOLO service on the 5090 GPU and verify it:

```bash
cd /workspace/ABot-Claw-piper/service_layer/YOLO
PORT=8013 DEVICE=cuda python main.py
```

In another shell in the same container:

```bash
curl http://127.0.0.1:8013/health
```

The health response should report a CUDA device and `"model_loaded": true`.

## 5090 ROS container

```bash
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://192.168.1.154:11311
export ROS_IP=192.168.1.104
rostopic list | grep table_camera
cd /workspace/ABot-Claw-piper/robot_layer/arm_piper/agent_server
./start_yolo_stream_viewer_5090.sh
```

The default request rate is 2 Hz, confidence threshold is 0.25, and IoU
threshold is 0.45. Press `q` or Escape in the OpenCV window to stop. In a
headless container the viewer automatically continues in save-only mode.

To explicitly disable the window or change the rate:

```bash
./start_yolo_stream_viewer_5090.sh --no-window --rate 1
```

The latest results are continuously replaced at:

```text
/tmp/yolo_stream_latest.jpg
/tmp/yolo_stream_latest.json
```

Copy the latest annotated frame from the 5090 ROS container:

```bash
docker cp abot-yolo-5090:/tmp/yolo_stream_latest.jpg ~/yolo_stream_latest.jpg
```
