# OpenVLA Shadow Service

This service evaluates the official `openvla/openvla-7b` checkpoint in shadow mode only.

- Container: `abot-openvla-shadow`
- Port: `8018`
- Model: `openvla/openvla-7b`
- Checkpoint revision: `47a0ec7fc4ec123775a391911046cf33cf9ed83f`
- Official source revision: `c8f03f48af692657d3060c19588038c7220e9af9`

Safety boundary:

- No execution endpoint exists.
- No PiPER, MoveIt, CAN, ROS motion, or 8891 backend is imported.
- Every response forces `execution_allowed: false`.
- PiPER state is metadata only and is never fed into the model.

Endpoints:

- `GET /health`
- `GET /model-info`
- `POST /compatibility-check`
- `POST /action-preview`

Use `start_openvla_shadow.sh` to build and start the isolated service without touching the running VLAC services.
