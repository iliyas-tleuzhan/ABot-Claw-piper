# OpenVLA Shadow Results

Date: 2026-07-23

Checkpoint and runtime:

- Model: `openvla/openvla-7b`
- Revision: `47a0ec7fc4ec123775a391911046cf33cf9ed83f`
- Source revision: `c8f03f48af692657d3060c19588038c7220e9af9`
- Service port: `8018`
- GPU: physical GPU 1 (`cuda:0` inside the container)
- Health after load:
  - `model_loaded: true`
  - `allocated_vram_mb: 14428.7`
  - `reserved_vram_mb: 14470.0`

Saved-image bakeoff:

- Image source: one RealSense RGB frame captured from the live `table_camera` stream.
- Important limitation: the image shows the PiPER workspace and a marked target card, not a clearly visible physical button.
- Repetitions: 5 per instruction.
- Sampling: deterministic (`do_sample=false` through the official path).
- Result: all 5 repetitions for every instruction were bit-identical.

Behavior summary:

- Contradictory prompts did change the returned 7D action.
- The output shape remained valid for every prompt.
- The gripper channel did not separate `Open the gripper.` from `Close the gripper.` in the saved-image bakeoff.
- The action frame, rotation convention, and Bridge-to-PiPER mapping remain unverified.

Live-image follow-up:

- A second fresh RealSense frame and matching live PiPER state were captured after restoring the D555 publisher.
- Live prompts returned valid 7D outputs with `execution_allowed: false`.
- Open/close again did not produce a useful gripper distinction.

Classification:

- Outcome B: technically runnable but behaviorally failed as a zero-shot PiPER execution policy.

Execution stays disabled for every result.
