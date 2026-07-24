---
name: abotclaw-piper-lap3b
description: Route a human tabletop instruction to the PiPER LAP-3B action-preview or one bounded execution step through the existing PiPER testbed runner. Use when the operator explicitly asks for a LAP-3B proof-of-concept action on PiPER.
---

# AbotClaw PiPER LAP-3B

Use this skill for the LAP-3B proof of concept:

- instruction
- live RealSense image
- live PiPER state
- remote LAP-3B websocket inference
- PiPER action adapter
- one bounded preview or one bounded execution step

Keep the existing architecture:

- OpenClaw workspace and skills
- local OpenClaw gateway
- local ABot-Claw Agent Server
- `~/piper-pipeline-testbed`
- `./tools/run_in_noetic_container.sh`
- `piper-on-bunker/scripts/run_lap_piper_action.py`

## Parse

Use `scripts/lap3b_piper_task.py` to turn the message into:

- `instruction`
- `mode`: `shadow` or `execute`
- `max_actions`
- direct command

Default to `shadow`.

Only use `execute` when the operator explicitly asks to execute one LAP-3B action on the real PiPER.

## Execute

Shadow:

```bash
cd /home/dase-hw101/piper-pipeline-testbed &&
./tools/run_in_noetic_container.sh \
  python3 piper-on-bunker/scripts/run_lap_piper_action.py \
    --instruction "Move the gripper toward the red cup." \
    --max-actions 1
```

Execute one bounded action:

```bash
cd /home/dase-hw101/piper-pipeline-testbed &&
./tools/run_in_noetic_container.sh \
  python3 piper-on-bunker/scripts/run_lap_piper_action.py \
    --instruction "Move the gripper toward the red cup." \
    --execute \
    --max-actions 1
```

Do not turn this into an autonomous loop. One action only for the first physical test.

## Result

Report:

- instruction
- mode
- live-observation capture status
- raw LAP action
- transformed translation
- current pose
- proposed target pose
- MoveIt request preview
- whether a motion service would be called
- final status
