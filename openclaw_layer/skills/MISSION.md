# MISSION.md - Robot-Type Routing Rules

Use this file as the quick routing rule sheet when choosing which robot should own a task.

## Robot Routing

- Piper: regular fixed-base workcell manipulation tasks
- PiPER-X: ROS 2 wrist-camera ArUco marker approach/touch/home tasks
- Unitree G1: humanoid, whole-body, or human-environment interaction tasks
- Unitree Go2: mobility, patrol, inspection, following, scene scouting, and remote perception tasks

## Rule

Classify the task first, then choose Piper, PiPER-X, G1, Go2, or a staged multi-robot workflow before writing or running robot control code.

Use PiPER-X, not regular Piper, for:

- approach the marker
- touch the ArUco marker
- go home
- save current pose as home
