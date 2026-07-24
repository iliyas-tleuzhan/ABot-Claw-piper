# AGENTS.md

This file applies to the entire `ABot-Claw-piper` repository.

## Purpose

This repository owns ABot-Claw-side services and deployment work for the PiPER and broader mobile-manipulation stack:

- ABot-Claw services and agent integration
- remote AI/model services
- model-serving APIs
- critic, perception, and spatial-memory services
- deployment scripts and service documentation

It does not own PiPER mission supervision, PiPER hardware orchestration, or the Bunker controller implementation. Treat those as external integrations with explicit contracts.

## Physical and Runtime Safety

Default to no physical execution.

Never send PiPER, Bunker, MoveIt, CAN, `piper_sdk`, joint, gripper, Cartesian, or `8891` movement commands merely because a model or service produced an action.

Do not add execution endpoints to shadow-model services. Do not silently change hardware-activation settings or assume permission for real-robot tests.

Do not stop unrelated services during model experiments. Keep existing healthy services in place unless the current task explicitly requires a restart.

## Responsibility Split

This repository owns:

- service APIs and deployment scripts
- service-specific docs, provenance, and compatibility audits
- remote inference containers and pinned revisions
- high-level ABot-Claw integration around those services

The `piper-pipeline-testbed` repository owns PiPER-side state machines, safety validation, shadow clients, and hardware adapters. Keep that boundary explicit.

The Bunker navigation controller is a collaborator-owned external contract. Do not invent or silently implement it here.

## Shadow Model Rules

All unvalidated model integrations must remain shadow-only.

Every shadow response must enforce `execution_allowed: false`.

Separate clearly:

- request success
- model-load success
- compatibility result
- inference success
- parser reliability
- output-shape validity
- model confidence
- behavioral success
- execution permission

Do not use ambiguous generic `success: true` fields without defining what succeeded. Parser reliability is not model confidence.

Shadow services must not import or call robot execution backends, MoveIt motion services, PiPER command APIs, ROS command publishers, CAN transmission, `8891` movement endpoints, or generic physical executors.

## Compatibility Before Implementation

Before downloading, containerizing, or implementing a new VLA checkpoint, perform a pre-download static compatibility audit.

Verify from the official source and checkpoint configuration:

1. camera count and exact input keys
2. image preprocessing and normalization
3. required state dimension and semantics
4. output action dimension and semantics
5. relative vs absolute action meaning
6. joint-space vs Cartesian action space
7. rotation representation and units
8. gripper representation and units
9. required normalization statistics and dataset metadata
10. action chunk length
11. GPU/VRAM and disk requirements
12. zero-shot vs fine-tuning expectations
13. whether missing inputs or dimension adaptation are legitimate

Do not force compatibility by padding or truncating undocumented dimensions, inventing camera keys, borrowing statistics from an unrelated robot, or treating any arbitrary 7D output as a PiPER command.

## Deployment Rules

Use the clean deployment checkout rather than the known dirty historical checkout.

Always verify:

- current `HEAD`
- `origin/main`
- working-tree cleanliness
- ownership
- which source tree a running service uses

Keep model services isolated by separate container, image, port, health endpoint, and pinned source/model revisions. Use intentional cache mounts only. Do not use privileged mode, Docker socket mounts, robot-device mounts, or implicit host access.

## Git and Filesystem Safety

Always inspect `git status` before editing. Preserve unrelated modified and untracked files.

Do not use `git reset --hard`, `git clean`, broad `git restore`, force push, recursive ownership changes across repositories, Docker prune commands, or broad Hugging Face cache deletion.

Use normal forward commits to `main` unless the user explicitly requests another workflow.

Before committing:

1. review the diff
2. run relevant tests/checks
3. confirm no unrelated files are staged
4. confirm no secrets, tokens, generated logs, model weights, or caches are staged
5. report the exact tests actually run

## Documentation

Keep detailed model results in the service docs, not here.

Preserve durable provenance and safety material for each service:

- source revision and checkpoint revision
- input/output contract
- normalization source
- unresolved semantics
- compatibility conclusion
- raw shadow results
- behavioral conclusion
- safety boundary
- exact tests performed
- whether live hardware was used
- whether any physical movement occurred

Known research conclusions belong in the existing documentation, including:

- VLAC-2B was technically runnable but rejected as a zero-shot PiPER policy.
- SmolVLA’s audited base checkpoint was statically incompatible with the proposed PiPER schema and was removed.
- OpenVLA-7B was technically runnable in shadow mode but rejected as a zero-shot PiPER policy.
- Future VLA work should prioritize PiPER-specific data collection and embodiment adaptation/fine-tuning.
- X-VLA is a fine-tuning candidate, not an assumed zero-shot controller.
