# Action Contract

The service returns the official OpenVLA Bridge-style 7D action without mapping it to PiPER commands.

Returned channels:

1. `x`
2. `y`
3. `z`
4. `roll`
5. `pitch`
6. `yaw`
7. `gripper`

Important semantics:

- `action_semantics`: `openvla_bridge_7d_end_effector_action`
- `relative_vs_absolute`: relative end-effector delta according to the official model card
- `coordinate_frame`: unverified for PiPER
- `rotation_convention`: unresolved in the official source
- `gripper_semantics`: unverified for PiPER
- `normalization_key`: `bridge_orig`
- `normalization_piper_specific`: `false`
- `execution_allowed`: `false`
