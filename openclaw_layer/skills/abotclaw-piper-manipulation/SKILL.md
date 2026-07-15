---
name: abotclaw-piper-manipulation
description: Parse natural-language tabletop manipulation requests and route them to Piper through the existing Agent Server manipulation runner. Use when the user asks to pick, grab, move, or place a graspable object in the Piper workcell.
---

# AbotClaw Piper Manipulation

Use Piper for tabletop pick and place requests. Keep the existing architecture:

- OpenClaw workspace and skills
- Agent Server at `http://127.0.0.1:8888`
- lease plus `/code/execute`
- `PiperRobotEnv`
- `GraspSDK`
- YOLO and live RealSense RGB-D
- MoveIt and the Piper trajectory bridge

## Parse

Use `scripts/piper_manipulation_task.py` to parse the user message. It returns:

- `action`
- `source`
- optional `destination`
- selected robot `Piper`
- direct command

The parser handles phrasing variations such as:

- `Pick up the cup.`
- `Grab the bottle.`
- `Pick up the cup and place it on the purple paper.`
- `Move the bottle onto the tray.`
- `Put the red cup on the blue area.`

Do not require an exact phrase match.

## Execute

For pickup-only:

```bash
cd /home/dase-hw101/ABot-Claw &&
python3 robot_layer/arm_piper/agent_server/run_piper_manipulation.py \
  --task pick \
  --source cup \
  --plan-only
```

For pick and place:

```bash
cd /home/dase-hw101/ABot-Claw &&
python3 robot_layer/arm_piper/agent_server/run_piper_manipulation.py \
  --task place \
  --source cup \
  --destination "purple paper" \
  --plan-only
```

Use `--execute` only after the matching plan-only run succeeds. Do not silently
replace a named destination with default base coordinates; destination perception
must succeed or the task fails.

## Result

Report:

- parsed task
- selected robot
- selected source detection
- selected destination detection when present
- source and destination camera/base coordinates
- grasp/release results
- final status
- visual verification status when a place task is executed
