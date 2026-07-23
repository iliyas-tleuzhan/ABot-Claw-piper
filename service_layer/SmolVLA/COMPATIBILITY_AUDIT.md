# SmolVLA Compatibility Audit

Official sources audited:

- LeRobot commit `9c82c39c7b541e9c5bd8340abb7c9d8803c98744`
- Model `lerobot/smolvla_base` revision `c83c3163b8ca9b7e67c509fffd9121e66cb96205`
- `src/lerobot/policies/smolvla/configuration_smolvla.py`
- `src/lerobot/policies/smolvla/modeling_smolvla.py`
- `src/lerobot/policies/smolvla/processor_smolvla.py`
- `examples/tutorial/smolvla/using_smolvla_example.py`
- checkpoint files `config.json`, `policy_preprocessor.json`, `policy_postprocessor.json`

Key findings:

1. The released checkpoint declares `observation.state` shape `[6]`.
2. The released checkpoint declares `action` shape `[6]`.
3. The released checkpoint declares three camera feature keys: `observation.images.camera1`, `camera2`, `camera3`.
4. The bundled normalizer and unnormalizer are checkpoint-specific and tied to the pretrained schema.
5. `chunk_size` and `n_action_steps` are both `50`.
6. The official example requires camera keys that match the training-time keys and builds dataset features from the target robot embodiment.
7. The base checkpoint is intended as a starting point for fine-tuning, not as a proven zero-shot PiPER policy.

PiPER blockers:

- PiPER proposes a 7D joint-plus-gripper state, but the base checkpoint expects 6D state.
- PiPER proposes a 7D joint-plus-gripper action, but the base checkpoint expects 6D action.
- PiPER currently has one required external camera and one optional wrist camera, but the checkpoint expects three camera inputs and `empty_cameras=0`.
- The checkpoint normalization statistics cannot be reused honestly for PiPER's schema.
- Action semantics are embodiment-dependent continuous actions, not validated PiPER joint targets.

Conclusion:

`lerobot/smolvla_base` is not genuinely compatible with the current PiPER schema without embodiment-specific dataset work and fine-tuning. Forced inference is refused.
