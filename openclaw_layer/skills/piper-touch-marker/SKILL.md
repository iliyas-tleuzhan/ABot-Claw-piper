---
name: piper-touch-marker
description: Use the PiPER-X ROS 2 Agent Server for requests such as "touch the marker", "touch ArUco marker 6", "move the Piper arm to the marker", "approach the marker", "point at the marker", "press the marked location", "go home", "return the Piper arm home", "open the gripper", or "close the gripper".
---

# PiPER Touch Marker

This is the short OpenClaw-facing skill for the current PiPER-X marker demo.
For full robot details, use `abotclaw-piper-x-manipulation`.

Default PiPER-X Agent Server:

```text
http://127.0.0.1:8893
```

The lower-level ROS 2 marker bridge remains on `http://127.0.0.1:8892`, but
OpenClaw should call the Agent Server on `8893`.

## Required Health Check

Always call:

```bash
curl -sS http://127.0.0.1:8893/health
```

Physical execution requires `execution_allowed: true` and a lease from
`POST /lease/acquire`.

For OpenClaw shell execution, prefer:

```bash
python3 /home/dase-hw101/ABot-Claw/robot_layer/arm_piper_x/agent_server/run_piper_x_agent_task.py touch --execute --retract --return-home-after
```

That helper acquires and releases a temporary lease automatically.

## Commands

Approach marker:

```bash
curl -sS -X POST http://127.0.0.1:8893/tools/approach-marker \
  -H 'Content-Type: application/json' \
  -d '{"execute":true,"lease_id":"<LEASE_ID>","pre_clearance_m":0.05,"final_clearance_m":0.005,"retract_after":false,"retract_distance_m":0.05,"final_velocity_scaling":0.05,"return_home_after":false,"home_duration_s":6.0}'
```

Touch marker, retract, then home:

```bash
curl -sS -X POST http://127.0.0.1:8893/tools/touch-marker \
  -H 'Content-Type: application/json' \
  -d '{"execute":true,"lease_id":"<LEASE_ID>","pre_clearance_m":0.05,"final_clearance_m":0.005,"retract_after":true,"retract_distance_m":0.05,"final_velocity_scaling":0.05,"return_home_after":true,"home_duration_s":6.0}'
```

Go home:

```bash
curl -sS -X POST http://127.0.0.1:8893/tools/go-home \
  -H 'Content-Type: application/json' \
  -d '{"execute":true,"lease_id":"<LEASE_ID>","duration_s":6.0}'
```

Save current pose as home:

```bash
curl -sS -X POST http://127.0.0.1:8893/tools/save-home \
  -H 'Content-Type: application/json' \
  -d '{"pose_name":"home"}'
```

Open gripper:

```bash
curl -sS -X POST http://127.0.0.1:8893/tools/open-gripper \
  -H 'Content-Type: application/json' \
  -d '{"execute":true,"lease_id":"<LEASE_ID>","width_m":0.10,"effort_n":1.0}'
```

Close gripper:

```bash
curl -sS -X POST http://127.0.0.1:8893/tools/close-gripper \
  -H 'Content-Type: application/json' \
  -d '{"execute":true,"lease_id":"<LEASE_ID>","width_m":0.0,"effort_n":1.0}'
```

## Safety

This skill targets PiPER-X through the Agent Server on `8893`, not the regular
Piper Agent Server on `8888`. The Agent Server wraps the lower-level ROS 2
marker bridge on `8892`.

Gripper commands use the AgileX ROS 2 command topic `/control/joint_states`,
message type `sensor_msgs/msg/JointState`, joint name `gripper`, and command
opening width in metres. They do not require the wrist camera.

"Touch" is geometric only:

```json
{
  "contact_confirmed": false,
  "completion_type": "geometric_surface_approach"
}
```
