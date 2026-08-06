# ABot-Claw PiPER-X Robot Layer

This folder is the ABot-Claw-side contract for the AgileX PiPER-X arm.

It is intentionally separate from `robot_layer/arm_piper`, which is the older
regular-Piper Agent Server path on `127.0.0.1:8888`.

## Current Supported PiPER-X Capability

The active PiPER-X path is the ROS 2 Jazzy MoveIt wall-marker approach stack
wrapped by a PiPER-X Agent Server:

- OpenClaw-facing Agent Server: `http://127.0.0.1:8893`
- low-level marker/home bridge: `http://127.0.0.1:8892`
- health: `GET /health`
- state: `GET /state`
- lease: `POST /lease/acquire`, `POST /lease/release`
- approach marker: `POST /tools/approach-marker`
- geometric touch marker: `POST /tools/touch-marker`
- go to saved home pose: `POST /tools/go-home`
- save current pose as home: `POST /tools/save-home`
- go to saved previous pose: `POST /tools/go-previous`
- save current pose as previous: `POST /tools/save-previous`
- open AgileX gripper: `POST /tools/open-gripper`
- close AgileX gripper: `POST /tools/close-gripper`

The Agent Server reads ROS 2 feedback topics for state and calls the low-level
marker/home bridge for the already validated marker demo tools. It does not
talk directly to CAN or expose arbitrary joint commands.

## Hardware Contract

- robot model: AgileX PiPER-X
- ROS driver: `agx_arm_ros`
- arm launch argument: `arm_type:=piper_x`
- end effector: AgileX parallel gripper
- effector launch argument: `effector_type:=agx_gripper`
- firmware launch argument: `fw_version:=v189`
- CAN interface: `can0`, 1 Mbps, configured before ROS launch
- MoveIt planning group: `arm`
- MoveIt TCP/tip link: `tcp_link`
- TCP offset: `[0.0, 0.0, 0.1425, 0.0, 0.0, 0.0]`
- arm feedback topic: `/feedback/joint_states`
- TCP feedback topic: `/feedback/tcp_pose`
- gripper command topic: `/control/joint_states`
- gripper command type: `sensor_msgs/msg/JointState`
- gripper joint name: `gripper`
- gripper opening width range: `[0.0, 0.1] m`
- gripper effort range: `[0.5, 3.0] N`
- wrist camera: Intel RealSense D435i
- color image: `/camera/camera/color/image_raw`
- camera info: `/camera/camera/color/camera_info`
- point cloud: `/camera/camera/depth/color/points`
- ArUco pose: `/aruco_single/pose`
- marker: ID `6`, size `0.10 m`
- hand-eye calibration file:
  `/home/dase-hw101/handeye/config/piper_x_d435i_eye_in_hand.json`

## Official Source Boundary

Use the installed AgileX ROS 2 stack as the source of truth:

- driver and MoveIt launch files:
  `/home/dase-hw101/agx_arm_ws/src/agx_arm_ros`
- PiPER-X URDF and meshes:
  `/home/dase-hw101/agx_arm_ws/src/agx_arm_ros/src/agx_arm_description/agx_arm_urdf/piper_x`
- MoveIt joint limits and controllers:
  `/home/dase-hw101/agx_arm_ws/src/agx_arm_ros/src/agx_arm_moveit/config`

The public AgileX organization documents that `agx_arm_ros` is the ROS 2 driver
for Piper-family arms including PiPER-X, and that `agx_arm_urdf` owns the URDF,
Xacro, and mesh resources.

## Current Generality Boundary

This folder does not claim PiPER-X general tabletop pick/place support yet.
Regular tabletop pick/place is still handled by `robot_layer/arm_piper`.

The Agent Server now supports the verified AgileX gripper width command
interface. Generic Cartesian/joint pose endpoints still fail closed until the
real PiPER-X MoveIt pose-command contract and workspace validation are
implemented.

Current PiPER-X skills are:

- approach ArUco marker
- geometric touch of ArUco marker
- save current pose as home
- go to saved home pose
- save current pose as previous
- go to saved previous pose
- open gripper
- close gripper

There is no force sensor in this path. "Touch" means a geometric approach to a
small clearance from the fitted wall surface, not force-confirmed contact.
"Close gripper" means command the configured opening width, not confirmed
object grasp.

The previous pose is a six-joint snapshot. Physical marker and home motions
save the current pose as previous before sending the new trajectory. The
operator can also call `save-previous` manually from a known safe pose.

## Quick Checks

```bash
python3 robot_layer/arm_piper_x/agent_server/run_piper_x_agent_task.py health
python3 robot_layer/arm_piper_x/agent_server/run_piper_x_agent_task.py state
python3 robot_layer/arm_piper_x/agent_server/run_piper_x_agent_task.py approach --plan-only
python3 robot_layer/arm_piper_x/agent_server/run_piper_x_agent_task.py touch --plan-only --retract --return-home-after
python3 robot_layer/arm_piper_x/agent_server/run_piper_x_agent_task.py save-home
python3 robot_layer/arm_piper_x/agent_server/run_piper_x_agent_task.py home --plan-only
python3 robot_layer/arm_piper_x/agent_server/run_piper_x_agent_task.py save-previous
python3 robot_layer/arm_piper_x/agent_server/run_piper_x_agent_task.py previous --plan-only
python3 robot_layer/arm_piper_x/agent_server/run_piper_x_agent_task.py open-gripper --plan-only
python3 robot_layer/arm_piper_x/agent_server/run_piper_x_agent_task.py close-gripper --plan-only
```

Use `--execute` only when the ROS 2 stack health reports
`execution_allowed: true` and the operator is ready to supervise the arm.
