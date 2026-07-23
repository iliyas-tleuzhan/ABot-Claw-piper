# Recommended PiPER LeRobot Dataset Schema

Recommended initial PiPER schema for LeRobot fine-tuning:

- `observation.images.camera1`: external RealSense RGB image
- `observation.images.camera2`: wrist RGB image when available
- `observation.state`: 7D vector `[joint1, joint2, joint3, joint4, joint5, joint6, gripper]`
- `task`: natural-language instruction
- `action`: 7D vector `[target_joint1, target_joint2, target_joint3, target_joint4, target_joint5, target_joint6, target_gripper]`

Required metadata:

- timestamps
- episode index
- frame index
- success/failure label
- exact unit documentation for gripper observation and action
- camera source metadata
- commanded action semantics

Collection recommendation:

- start with a small validation dataset to prove feature schemas and normalization
- then collect roughly 50 successful tabletop episodes as the first serious fine-tuning dataset
