#!/usr/bin/env python3
"""PiPER-X Agent Server backed by the ROS 2 MoveIt marker/home stack."""

from __future__ import annotations

import argparse
import os
import sys

import uvicorn
from fastapi import FastAPI

_SERVER_DIR = os.path.dirname(os.path.abspath(__file__))
if _SERVER_DIR not in sys.path:
    sys.path.insert(0, _SERVER_DIR)

from config import config_from_env
from lease import LeaseManager
from robot_sdk import PiperXAgentSdk
from state import PiperXStateMonitor


def build_app(cfg=None, sdk=None, state_monitor=None, lease_mgr=None) -> FastAPI:
    cfg = cfg or config_from_env()
    sdk = sdk or PiperXAgentSdk(cfg.marker_api_url, timeout_s=cfg.request_timeout_s)
    lease_mgr = lease_mgr or LeaseManager(max_duration_s=cfg.lease_max_duration_s)
    state_monitor = state_monitor or PiperXStateMonitor(
        joint_state_topic=cfg.joint_state_topic,
        tcp_pose_topic=cfg.tcp_pose_topic,
        gripper_control_topic=cfg.gripper_control_topic,
        trajectory_action=cfg.trajectory_action,
        marker_task_service=cfg.marker_task_service,
        expected_joints=cfg.expected_joints,
        timeout_s=cfg.state_timeout_s,
    )

    app = FastAPI(title="PiPER-X Agent Server", version="0.1.0")
    app.state.cfg = cfg
    app.state.sdk = sdk
    app.state.state_monitor = state_monitor
    app.state.lease_mgr = lease_mgr

    @app.get("/", include_in_schema=False)
    def root():
        return {
            "status": "ok",
            "agent": "piper_x_agent_server",
            "docs": "/docs",
            "health": "/health",
        }

    from routes.lease_routes import create_router as lease_router
    from routes.state_routes import create_router as state_router
    from routes.tool_routes import create_router as tool_router

    app.include_router(state_router(cfg, state_monitor, sdk, lease_mgr))
    app.include_router(lease_router(lease_mgr))
    app.include_router(tool_router(cfg, sdk, lease_mgr, state_monitor))

    @app.on_event("startup")
    def startup():
        state_monitor.start()

    @app.on_event("shutdown")
    def shutdown():
        state_monitor.stop()

    return app


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=os.environ.get("PIPER_X_AGENT_HOST", "127.0.0.1"))
    parser.add_argument("--port", type=int, default=int(os.environ.get("PIPER_X_AGENT_PORT", "8893")))
    args = parser.parse_args()
    cfg = config_from_env(host=args.host, port=args.port)
    uvicorn.run(build_app(cfg), host=cfg.host, port=cfg.port, log_level="info")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
