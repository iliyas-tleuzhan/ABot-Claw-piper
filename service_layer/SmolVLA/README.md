# SmolVLA Shadow Service

This is a separate shadow-only compatibility and inference boundary for official SmolVLA.

Safety properties:

- `execution_allowed` is always `false`.
- No motion backend is imported.
- No execution endpoint exists.
- No MoveIt, 8891, CAN, or PiPER command path is exposed.
- The existing VLAC services on ports 8014 and 8016 are not reused or modified.

Endpoints:

- `GET /health`
- `GET /model-info`
- `POST /compatibility-check`
- `POST /action-preview`

The current expected result for PiPER is incompatibility with the released base checkpoint. The service returns a structured incompatibility report and refuses forced inference.
