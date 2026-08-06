# SERVICE.md

ABotClaw shared service registry.

All services in this file are assumed to be reachable by both OpenClaw and the robots.
Update this file first when the service host or port changes.

## Shared Service Host

- `SERVICE_HOST=192.168.1.104`

## Service Table

| Service | Purpose | IP / Host | Port | Base URL | Main Endpoint |
|---|---|---|---|---|---|
| SpatialMemory | Robot memory write / query / retrieval | `192.168.1.104` | `8012` | `http://192.168.1.104:8012` | `/health`, `/query/*`, `/memory/*` |
| YOLO | Object detection service | `192.168.1.104` | `8013` | `http://192.168.1.104:8013` | `/health`, `/detect` |
| VLAC | Task progress / completion critic | `192.168.1.104` | `8014` | `http://192.168.1.104:8014` | `/health`, `/critic` |
| GraspAnything | Grasp proposal / grasp detection service | `192.168.1.104` | `8015` | `http://192.168.1.104:8015` | `/health`, `/grasp/detect` |
| LAP-3B | Action-policy websocket service for PiPER proof-of-concept action generation | `192.168.1.104` | `8016` | `ws://192.168.1.104:8016` | websocket metadata frame, websocket action inference |
| PiPER-X Agent Server | OpenClaw-facing PiPER-X marker/home/previous/gripper Agent Server with lease, state, and safe tool endpoints | `127.0.0.1` | `8893` | `http://127.0.0.1:8893` | `/health`, `/state`, `/lease/acquire`, `/tools/approach-marker`, `/tools/touch-marker`, `/tools/go-home`, `/tools/save-home`, `/tools/go-previous`, `/tools/save-previous`, `/tools/open-gripper`, `/tools/close-gripper` |
| PiPER-X Marker/Home ROS2 | Local PiPER-X ArUco wall approach, geometric touch, home-pose, and previous-pose bridge | `127.0.0.1` | `8892` | `http://127.0.0.1:8892` | `/health`, `/tools/piper/approach-marker`, `/tools/piper/touch-marker`, `/tools/piper/go-home`, `/tools/piper/save-home`, `/tools/piper/go-previous`, `/tools/piper/save-previous` |

## Notes

- Use `base64` for image transfer unless a service explicitly documents another format.
- For long flows, call `/health` first.
- For full request/response details, check each service's own API / agent docs.
- PiPER-X Marker/Home ROS2 is local to the ROS 2 robot host and is disabled for physical execution unless `PIPER_TOUCH_ALLOW_EXECUTION=1` or the full-stack launch receives `execute_allowed:=true`.
- OpenClaw should call PiPER-X through the Agent Server on `8893`. The `8892` bridge is the lower-level ROS 2 implementation detail.
- Do not send PiPER-X commands to the regular Piper Agent Server on `127.0.0.1:8888`.
