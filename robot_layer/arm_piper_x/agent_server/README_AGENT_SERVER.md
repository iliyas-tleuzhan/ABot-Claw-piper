# PiPER-X Agent Server

This is the ABot-Claw-facing server for PiPER-X.

It listens on `http://127.0.0.1:8893` and wraps the ROS 2 PiPER-X marker/home
bridge on `http://127.0.0.1:8892`.

## Ports

- `8893`: PiPER-X Agent Server for ABot-Claw and OpenClaw
- `8892`: low-level ROS 2 marker/home bridge
- `8888`: old regular Piper Agent Server, not used for PiPER-X

## Start

```bash
cd /home/dase-hw101/ABot-Claw
robot_layer/arm_piper_x/agent_server/start_piper_x_agent_server.sh
```

Physical execution through this server is blocked unless:

```bash
export PIPER_X_AGENT_ALLOW_EXECUTION=1
```

The low-level `8892` bridge must also allow execution.

## Install User Service

```bash
cd /home/dase-hw101/ABot-Claw
deployment/scripts/install_piper_x_agent_server_service.sh
systemctl --user start piper-x-agent-server
systemctl --user status piper-x-agent-server
```

Logs:

```bash
journalctl --user -u piper-x-agent-server -f
```

## Health

```bash
curl -s http://127.0.0.1:8893/health | python3 -m json.tool
```

## Lease

Physical execution requires a lease:

```bash
curl -sS -X POST http://127.0.0.1:8893/lease/acquire \
  -H 'Content-Type: application/json' \
  -d '{"holder":"openclaw","duration_s":300}' | python3 -m json.tool
```

Use the returned `lease_id` in execute requests.
The helper script `run_piper_x_agent_task.py` acquires and releases a temporary
lease automatically when `--execute` is used without `--lease-id`.

## Marker Tools

Plan-only approach:

```bash
curl -sS -X POST http://127.0.0.1:8893/tools/approach-marker \
  -H 'Content-Type: application/json' \
  -d '{"execute":false,"pre_clearance_m":0.05}' | python3 -m json.tool
```

Physical geometric touch, retract, and home:

```bash
curl -sS -X POST http://127.0.0.1:8893/tools/touch-marker \
  -H 'Content-Type: application/json' \
  -d '{"execute":true,"lease_id":"<LEASE_ID>","pre_clearance_m":0.05,"final_clearance_m":0.005,"retract_after":true,"retract_distance_m":0.05,"final_velocity_scaling":0.05,"return_home_after":true}' \
  | python3 -m json.tool
```

## Saved Pose Tools

Plan-only home:

```bash
python3 robot_layer/arm_piper_x/agent_server/run_piper_x_agent_task.py \
  home \
  --plan-only
```

Save current pose as previous without moving:

```bash
python3 robot_layer/arm_piper_x/agent_server/run_piper_x_agent_task.py \
  save-previous
```

Plan-only return to previous:

```bash
python3 robot_layer/arm_piper_x/agent_server/run_piper_x_agent_task.py \
  previous \
  --plan-only
```

Physical return to previous, after both execution gates are intentionally
enabled:

```bash
python3 robot_layer/arm_piper_x/agent_server/run_piper_x_agent_task.py \
  previous \
  --execute
```

The previous pose is stored by the low-level bridge as a saved six-joint pose.
Physical marker and home commands automatically update it before motion.

## Gripper Tools

The PiPER-X gripper uses the AgileX ROS 2 `agx_arm_ros` control topic:

- topic: `/control/joint_states`
- type: `sensor_msgs/msg/JointState`
- joint name: `gripper`
- position: total opening width in metres, range `[0.0, 0.1]`
- effort: command force in newtons, range `[0.5, 3.0]`

These tools do not require the wrist camera, ArUco, or point cloud. Physical
execution still requires `PIPER_X_AGENT_ALLOW_EXECUTION=1`, an Agent Server
lease, and an active ROS 2 driver subscriber on `/control/joint_states`.

Plan-only open:

```bash
python3 robot_layer/arm_piper_x/agent_server/run_piper_x_agent_task.py \
  open-gripper \
  --plan-only
```

Plan-only close:

```bash
python3 robot_layer/arm_piper_x/agent_server/run_piper_x_agent_task.py \
  close-gripper \
  --plan-only
```

Physical open, after the driver is running and execution is intentionally
enabled:

```bash
python3 robot_layer/arm_piper_x/agent_server/run_piper_x_agent_task.py \
  open-gripper \
  --execute
```

Physical close:

```bash
python3 robot_layer/arm_piper_x/agent_server/run_piper_x_agent_task.py \
  close-gripper \
  --execute
```

## Current Limits

The server intentionally does not expose `/code/execute`.

Generic pose endpoints still fail closed until the real PiPER-X MoveIt
pose-command contract is implemented. Use `GET /state` to inspect discovered
ROS topics, services, and actions.
