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

## Current Limits

The server intentionally does not expose `/code/execute`.

The gripper endpoints and generic pose endpoints exist but fail closed until
the real PiPER-X ROS 2 gripper and MoveIt pose-command contract is verified.
Use `GET /state` to inspect discovered ROS topics, services, and actions.
