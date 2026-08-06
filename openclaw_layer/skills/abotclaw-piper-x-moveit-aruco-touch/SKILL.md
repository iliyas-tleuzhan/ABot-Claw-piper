---
name: abotclaw-piper-x-moveit-aruco-touch
description: Legacy alias for the current ROS 2 PiPER-X ArUco marker approach API. Use piper-touch-marker behavior for "touch marker", "approach marker", "go home", "save current pose as home", "go previous", and "save current pose as previous".
---

# Legacy Alias

This skill used to describe the old ROS 1 `piper-pipeline-testbed` MoveIt touch flow.

That path is superseded for the current demo. Do not call the old
`piper-pipeline-testbed` commands from this skill.

Use the active `piper-touch-marker` skill contract instead. OpenClaw should
call the PiPER-X Agent Server on `8893`; the lower-level ROS 2 marker bridge
remains on `8892`.

- `approach the marker` -> `POST http://127.0.0.1:8893/tools/approach-marker`
- `touch the marker` -> `POST http://127.0.0.1:8893/tools/touch-marker`
- `go home` -> `POST http://127.0.0.1:8893/tools/go-home`
- `save current pose as home` -> `POST http://127.0.0.1:8893/tools/save-home`
- `go previous` -> `POST http://127.0.0.1:8893/tools/go-previous`
- `save current pose as previous` -> `POST http://127.0.0.1:8893/tools/save-previous`

Always call:

```bash
curl -sS http://127.0.0.1:8893/health
```

before a physical command.

Never generate arbitrary MoveIt, joint, CAN, gripper, or shell movement code
when the local PiPER marker API is available.
