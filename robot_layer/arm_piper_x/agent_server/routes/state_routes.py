"""State and health routes for PiPER-X Agent Server."""

from __future__ import annotations

from fastapi import APIRouter


def create_router(cfg, state_monitor, sdk, lease_mgr) -> APIRouter:
    router = APIRouter(tags=["state"])

    @router.get("/health")
    def health():
        marker_health = sdk.health()
        state = state_monitor.state()
        graph = state_monitor.graph()
        return {
            "status": "ready"
            if marker_health.get("ros_ok") and state.get("joint_state_fresh") and graph.get("trajectory_action_available")
            else "not_ready",
            "agent": "piper_x_agent_server",
            "port": cfg.port,
            "execution_allowed": cfg.execution_allowed,
            "marker_api_url": cfg.marker_api_url,
            "marker_api_health": marker_health,
            "ros": {
                "joint_state_topic": cfg.joint_state_topic,
                "tcp_pose_topic": cfg.tcp_pose_topic,
                "trajectory_action": cfg.trajectory_action,
                "marker_task_service": cfg.marker_task_service,
                "joint_state_available": state.get("joint_state_fresh", False),
                "tcp_pose_available": state.get("tcp_pose_fresh", False),
                "trajectory_action_available": graph.get("trajectory_action_available", False),
                "marker_task_service_seen": graph.get("marker_task_service_seen", False),
            },
            "marker": {
                "configured_marker_id": cfg.marker_id,
                "configured_marker_size_m": cfg.marker_size_m,
            },
            "lease": lease_mgr.status(),
            "gripper_control": {
                "supported": False,
                "reason": "No verified PiPER-X gripper command interface has been selected yet.",
                "discovered_topics": graph.get("topics", []),
                "discovered_services": graph.get("services", []),
            },
        }

    @router.get("/state")
    def state():
        return {
            "robot": "PiPER-X",
            "state": state_monitor.state(),
            "ros_graph": state_monitor.graph(),
            "lease": lease_mgr.status(),
            "marker_api_health": sdk.health(),
        }

    return router
