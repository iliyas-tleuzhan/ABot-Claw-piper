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

## Notes

- Use `base64` for image transfer unless a service explicitly documents another format.
- For long flows, call `/health` first.
- For full request/response details, check each service's own API / agent docs.
