# AGENTS.md

This file applies to `service_layer/` and inherits the repository root `AGENTS.md`.

## Local Scope

Keep `service_layer/` focused on isolated service implementations, deployment scripts, and service-specific documentation.

Each service should expose only the APIs it actually supports. Do not return false success for unsupported routes or silently reinterpret inputs.

## Service Isolation

For model experiments, use a separate container, image, and port. Do not reuse or mutate unrelated live services in place.

Do not stop or restart unrelated services during an experiment. Keep critic, perception, memory, and shadow-model services isolated from each other.

## Shadow API Rules

Shadow-only services must:

- force `execution_allowed: false` on every response path
- expose no execution or forwarding endpoint
- reject arbitrary remote image fetching by default
- preserve raw outputs and provenance metadata
- keep parser reliability separate from model confidence
- keep behavioral conclusions separate from inference success

If compatibility is not established, refuse inference with a structured incompatibility response rather than forcing inputs to fit.

## Provenance and Documentation

For each model service, maintain concise docs covering:

- model/checkpoint identity and revision
- source revision
- dependency/runtime versions
- input contract
- output contract
- normalization source
- unresolved semantics
- shadow-only safety boundary
- exact tests performed
- behavioral conclusion

## Physical VLA Test Handoff

service_layer may support audits, compatibility checks, and shadow inference, but it must stop at a manual physical-test handoff. If a service result is mature enough to justify one bounded hardware check, report one exact user-entered command plus the expected movement, required initial pose, tested model action, completion signal, stop command, and log path.

Do not execute motion during the audit, do not add repeated confirmation flags, and do not let model output choose speed, acceleration, limits, or duration. If frames, normalization, transforms, units, gripper semantics, or command mapping remain unresolved, report the exact calibration or validation step instead of executable motion.

## Deployment Hygiene

Pin important dependency versions. Prefer intentional cache mounts and conservative restart behavior. Do not mount robot devices, CAN devices, ROS sockets, or the Docker socket into model-service containers.
