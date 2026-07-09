# Fake calibration for pipeline loading tests

Warning: `fake_calibration.yaml` is not a real calibration. Never use it with
`--execute` or for real pick-and-place motion.

The pick-and-place entry point `today_red_to_purple_pick_place.py` normally
loads `camera_to_base.yaml` from this directory. That production-default file
did not exist when this test fixture was added. Its format is defined by
`camera_to_base.example.yaml`, which was copied unchanged to
`camera_to_base.example.yaml.backup_original`.

The fake file uses a near-identity rotation (0.003 radians around Z) and a
small translation of `[0.01, -0.02, 0.03]` metres:

```yaml
camera_to_base:
  - [0.9999955, -0.0029999955, 0.0, 0.01]
  - [0.0029999955, 0.9999955, 0.0, -0.02]
  - [0.0, 0.0, 1.0, 0.03]
  - [0.0, 0.0, 0.0, 1.0]
```

Validate config loading without starting ROS, camera capture, or robot motion:

```bash
docker exec abot-piper-noetic bash -lc \
  'source /opt/ros/noetic/setup.bash; source /root/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros/devel/setup.bash; export PIPER_ENV_SOURCED=1; cd /root/ABot-Claw/robot_layer/arm_piper/agent_server; python3 -c '"'"'from today_red_to_purple_pick_place import load_camera_to_base; print(load_camera_to_base("fake_calibration.yaml"))'"'"
```

For a camera-connected detection-only test, omit `--execute`:

```bash
./today_red_to_purple_pick_place.py --calibration fake_calibration.yaml
```

Restore the unchanged example from its backup with:

```bash
cp camera_to_base.example.yaml.backup_original camera_to_base.example.yaml
```

If a real `camera_to_base.yaml` is later installed, keep it separate and pass
the fake fixture only through `--calibration fake_calibration.yaml`.
