---
name: abotclaw-run-robot-task
description: Execute or plan a real task on Piper, PiPER-X, Unitree G1, or Unitree Go2. Use when the user asks the robot fleet to do something in the physical world, including observing, moving, manipulating, inspecting, or multi-robot task execution.
---

# AbotClaw Run Robot Task

Follow this order. Do not jump straight into code.

## Step 1: Classify the task

Decide whether the request is mainly:

- perception
- manipulation
- locomotion
- human interaction
- multi-stage coordination

## Step 2: Choose the robot

Pick Piper, PiPER-X, G1, Go2, or a staged combination.

Use PiPER-X when the command is specifically one of:

- approach the marker
- touch or press ArUco marker 6 / the marked wall location
- go home
- save current pose as home

Route those through `abotclaw-piper-x-manipulation` / `piper-touch-marker`, not
the regular Piper Agent Server.

## Step 3: Discover real usage first

Before new code, use SDK discovery:

- find the actual SDK / API docs
- find examples
- identify the real calling pattern

If the interface is not yet known, stop and discover it first.

## Step 4: Check reusable skills

Look for existing local skills or reusable code paths before building from scratch.

## Step 5: Execute cautiously on hardware

Because there is no simulator safety layer here:

- prefer minimal smoke tests first
- make preconditions explicit
- avoid large first-run motions
- keep rollback / stop strategy in mind

## Rule

The first successful robot action should usually be a small probe that confirms the API and embodiment assumptions, not a heroic end-to-end attempt.
