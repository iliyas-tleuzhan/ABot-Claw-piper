# LAP-3B Service

This directory wraps the official `lihzha/lap` websocket policy server for ABot-Claw deployment.

- Official repository: `https://github.com/lihzha/lap`
- Official checkpoint: `lihzha/LAP-3B`
- Default runtime: host-side official `uv` environment
- Default process name: `scripts/serve_policy.py --env=LAP --port=8016`
- Default port: `8016`

The service runs the official websocket policy server:

```bash
JAX_PLATFORMS=cuda python scripts/serve_policy.py --env=LAP --port=8016
```

The wrapper scripts:

1. ensure the pinned official LAP checkout exists,
2. ensure the checkpoint exists in persistent storage,
3. create the pinned `uv` Python 3.11 environment,
4. apply the required protobuf runtime fix for current upstream startup,
5. cache the PaLI-Gemma tokenizer asset locally,
6. start the LAP websocket server on one selected GPU,
5. verify websocket reachability without sending any robot command.

This service does not contain PiPER execution logic.
