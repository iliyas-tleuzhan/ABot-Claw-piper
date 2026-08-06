"""Configuration for the PiPER-X Agent Server."""

from __future__ import annotations

import os
from dataclasses import dataclass, field


def _truthy_env(name: str) -> bool:
    return os.environ.get(name, "").strip().lower() in {"1", "true"}


@dataclass(frozen=True)
class PiperXAgentConfig:
    host: str = "127.0.0.1"
    port: int = 8893
    marker_api_url: str = "http://127.0.0.1:8892"
    joint_state_topic: str = "/feedback/joint_states"
    tcp_pose_topic: str = "/feedback/tcp_pose"
    gripper_control_topic: str = "/control/joint_states"
    gripper_joint_name: str = "gripper"
    gripper_open_width_m: float = 0.10
    gripper_close_width_m: float = 0.0
    gripper_default_effort_n: float = 1.0
    trajectory_action: str = "/arm_controller/follow_joint_trajectory"
    marker_task_service: str = "/run_marker_task"
    state_timeout_s: float = 1.0
    request_timeout_s: float = 180.0
    lease_max_duration_s: float = 300.0
    expected_joints: list[str] = field(
        default_factory=lambda: ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    )
    marker_id: int = 6
    marker_size_m: float = 0.10

    @property
    def execution_allowed(self) -> bool:
        return _truthy_env("PIPER_X_AGENT_ALLOW_EXECUTION")


def config_from_env(host: str | None = None, port: int | None = None) -> PiperXAgentConfig:
    return PiperXAgentConfig(
        host=host or os.environ.get("PIPER_X_AGENT_HOST", "127.0.0.1"),
        port=port or int(os.environ.get("PIPER_X_AGENT_PORT", "8893")),
        marker_api_url=os.environ.get("PIPER_X_MARKER_API_URL", "http://127.0.0.1:8892").rstrip("/"),
        joint_state_topic=os.environ.get("PIPER_X_JOINT_STATE_TOPIC", "/feedback/joint_states"),
        tcp_pose_topic=os.environ.get("PIPER_X_TCP_POSE_TOPIC", "/feedback/tcp_pose"),
        gripper_control_topic=os.environ.get("PIPER_X_GRIPPER_CONTROL_TOPIC", "/control/joint_states"),
        gripper_joint_name=os.environ.get("PIPER_X_GRIPPER_JOINT_NAME", "gripper"),
        gripper_open_width_m=float(os.environ.get("PIPER_X_GRIPPER_OPEN_WIDTH_M", "0.10")),
        gripper_close_width_m=float(os.environ.get("PIPER_X_GRIPPER_CLOSE_WIDTH_M", "0.0")),
        gripper_default_effort_n=float(os.environ.get("PIPER_X_GRIPPER_DEFAULT_EFFORT_N", "1.0")),
        trajectory_action=os.environ.get(
            "PIPER_X_TRAJECTORY_ACTION",
            "/arm_controller/follow_joint_trajectory",
        ),
        marker_task_service=os.environ.get("PIPER_X_MARKER_TASK_SERVICE", "/run_marker_task"),
        state_timeout_s=float(os.environ.get("PIPER_X_AGENT_STATE_TIMEOUT_S", "1.0")),
        request_timeout_s=float(os.environ.get("PIPER_X_AGENT_REQUEST_TIMEOUT_S", "180.0")),
    )
