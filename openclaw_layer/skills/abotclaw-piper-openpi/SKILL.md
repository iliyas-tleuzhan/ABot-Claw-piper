# ABot-Claw PiPER OpenPI Manipulation

Default PiPER manipulation uses OpenClaw semantic phases followed by a
PiPER-fine-tuned OpenPI pi0.5 policy that emits direct absolute joint chunks.

Allowed operations:

- plan_manipulation_task
- start_manipulation_task
- execute_manipulation_phase
- get_manipulation_status
- confirm_phase
- abort_manipulation
- recover_manipulation

Do not call LAP, MoveIt, IK, Cartesian conversion, `MoveGroupCommander`,
`FollowJointTrajectory`, or unrestricted shell joint-command tools for the
default manipulation path.

Physical execution is forbidden unless the active policy response says:

- `action_semantics: absolute_piper_joint_targets`
- `joint_names: [joint1, joint2, joint3, joint4, joint5, joint6, gripper]`
- `piper_compatible: true`
- complete PiPER normalization/checkpoint metadata

Public OpenPI checkpoints are shadow/protocol-only.
