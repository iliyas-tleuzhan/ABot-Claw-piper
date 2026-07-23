# OpenVLA Compatibility Audit

Date: 2026-07-23

Official artifacts audited:

- Model: `openvla/openvla-7b`
- Checkpoint revision: `47a0ec7fc4ec123775a391911046cf33cf9ed83f`
- Repository: `https://github.com/openvla/openvla`
- Source revision: `c8f03f48af692657d3060c19588038c7220e9af9`

Findings:

1. The official inference path accepts one RGB image and one natural-language instruction.
2. The official deploy path does not require a robot state vector.
3. The official checkpoint exposes a 7D action through `predict_action`.
4. Action channel order is documented as `(x, y, z, roll, pitch, yaw, gripper)`.
5. The model card describes these as 7-DoF end-effector deltas.
6. `bridge_orig` normalization statistics are embedded directly in the checkpoint `config.json`.
7. The official custom code unnormalizes channels 0..5 with `q01/q99`; channel 6 is left normalized because `mask[6]` is false.
8. This is sufficient for shadow inference.
9. This is not sufficient for PiPER execution because the BridgeData frame, rotation convention, and gripper semantics are not PiPER-verified.

Decision:

- Shadow inference: allowed
- Physical execution: rejected

Runtime verification on 2026-07-23:

- The official checkpoint downloaded and loaded successfully on the RTX 5090 server.
- The first load required:
  - a dedicated writable cache directory under `/data/home/iliyas/abot-data/openvla-huggingface`
  - `accelerate`
  - running the container as the remote user instead of root
- After load, the service reported `model_loaded: true` and served `GET /health`, `GET /model-info`, `POST /compatibility-check`, and `POST /action-preview`.
