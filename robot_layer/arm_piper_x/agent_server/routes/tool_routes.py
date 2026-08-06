"""Tool routes for PiPER-X Agent Server."""

from __future__ import annotations

from typing import Any, Dict, Optional

from fastapi import APIRouter, HTTPException, status
from pydantic import BaseModel, Field


class MarkerTaskRequest(BaseModel):
    execute: bool = False
    lease_id: Optional[str] = None
    pre_clearance_m: float = Field(default=0.05)
    final_clearance_m: float = Field(default=0.005)
    retract_after: bool = False
    retract_distance_m: float = Field(default=0.05)
    final_velocity_scaling: float = Field(default=0.05)
    return_home_after: bool = False
    home_duration_s: float = Field(default=6.0)


class HomeRequest(BaseModel):
    execute: bool = False
    lease_id: Optional[str] = None
    duration_s: float = Field(default=6.0)


class SaveHomeRequest(BaseModel):
    pose_name: str = Field(default="home")


class GripperRequest(BaseModel):
    execute: bool = False
    lease_id: Optional[str] = None


class PoseRequest(BaseModel):
    execute: bool = False
    lease_id: Optional[str] = None
    frame_id: str = Field(default="base_link")
    position_m: list[float]
    orientation_xyzw: list[float]
    velocity_scaling: float = Field(default=0.10)


class RelativeMoveRequest(BaseModel):
    execute: bool = False
    lease_id: Optional[str] = None
    frame_id: str = Field(default="base_link")
    translation_m: list[float]
    velocity_scaling: float = Field(default=0.10)


def _normalize_marker_api_response(status_code: int, result: Dict[str, Any]) -> Dict[str, Any]:
    if status_code >= 400:
        detail = result.get("detail", result)
        if isinstance(detail, dict):
            return detail
        return {"success": False, "stage": "marker_api", "message": str(detail)}
    return result


def create_router(cfg, sdk, lease_mgr) -> APIRouter:
    router = APIRouter(prefix="/tools", tags=["tools"])

    def require_execution_allowed(execute: bool, lease_id: str | None) -> None:
        if not execute:
            return
        if not cfg.execution_allowed:
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail={
                    "success": False,
                    "stage": "execution_gate",
                    "message": "PiPER-X Agent Server execution is disabled; set PIPER_X_AGENT_ALLOW_EXECUTION=1",
                },
            )
        ok, detail = lease_mgr.require(lease_id)
        if not ok:
            raise HTTPException(status_code=status.HTTP_409_CONFLICT, detail={"success": False, **detail})

    def proxy_marker_task(endpoint: str, request: MarkerTaskRequest) -> Dict[str, Any]:
        require_execution_allowed(request.execute, request.lease_id)
        payload = request.model_dump(exclude={"lease_id"})
        if endpoint == "approach-marker":
            status_code, result = sdk.approach_marker(payload)
        elif endpoint == "touch-marker":
            status_code, result = sdk.touch_marker(payload)
        else:
            raise AssertionError(endpoint)
        normalized = _normalize_marker_api_response(status_code, result)
        if not normalized.get("success", False):
            raise HTTPException(status_code=422 if status_code < 400 else status_code, detail=normalized)
        return normalized

    @router.post("/approach-marker")
    def approach_marker(req: MarkerTaskRequest):
        return proxy_marker_task("approach-marker", req)

    @router.post("/touch-marker")
    def touch_marker(req: MarkerTaskRequest):
        return proxy_marker_task("touch-marker", req)

    @router.post("/go-home")
    def go_home(req: HomeRequest):
        require_execution_allowed(req.execute, req.lease_id)
        status_code, result = sdk.go_home(req.model_dump(exclude={"lease_id"}))
        normalized = _normalize_marker_api_response(status_code, result)
        if not normalized.get("success", False):
            raise HTTPException(status_code=422 if status_code < 400 else status_code, detail=normalized)
        return normalized

    @router.post("/save-home")
    def save_home(req: SaveHomeRequest):
        status_code, result = sdk.save_home(req.model_dump())
        normalized = _normalize_marker_api_response(status_code, result)
        if not normalized.get("success", False):
            raise HTTPException(status_code=422 if status_code < 400 else status_code, detail=normalized)
        return normalized

    @router.post("/open-gripper")
    def open_gripper(req: GripperRequest):
        require_execution_allowed(req.execute, req.lease_id)
        raise HTTPException(
            status_code=501,
            detail={
                "success": False,
                "stage": "gripper_interface",
                "message": "PiPER-X gripper command endpoint is not enabled because no verified ROS 2 gripper command topic/service was found.",
            },
        )

    @router.post("/close-gripper")
    def close_gripper(req: GripperRequest):
        require_execution_allowed(req.execute, req.lease_id)
        raise HTTPException(
            status_code=501,
            detail={
                "success": False,
                "stage": "gripper_interface",
                "message": "PiPER-X gripper command endpoint is not enabled because no verified ROS 2 gripper command topic/service was found.",
            },
        )

    @router.post("/plan-to-pose")
    def plan_to_pose(req: PoseRequest):
        ok, message = sdk.validate_pose_payload(req.model_dump())
        if not ok:
            raise HTTPException(status_code=400, detail={"success": False, "stage": "request_validation", "message": message})
        return {
            "success": False,
            "stage": "moveit_pose_api",
            "message": "Generic PiPER-X plan-to-pose is not implemented yet; use approach-marker/touch-marker until MoveIt pose validation is added.",
            "execution_attempted": False,
        }

    @router.post("/move-to-pose")
    def move_to_pose(req: PoseRequest):
        require_execution_allowed(req.execute, req.lease_id)
        ok, message = sdk.validate_pose_payload(req.model_dump())
        if not ok:
            raise HTTPException(status_code=400, detail={"success": False, "stage": "request_validation", "message": message})
        raise HTTPException(
            status_code=501,
            detail={
                "success": False,
                "stage": "moveit_pose_api",
                "message": "Generic PiPER-X move-to-pose is not implemented yet; arbitrary pose execution remains blocked.",
            },
        )

    @router.post("/move-relative")
    def move_relative(req: RelativeMoveRequest):
        require_execution_allowed(req.execute, req.lease_id)
        if not isinstance(req.translation_m, list) or len(req.translation_m) != 3:
            raise HTTPException(
                status_code=400,
                detail={"success": False, "stage": "request_validation", "message": "translation_m must contain three values"},
            )
        raise HTTPException(
            status_code=501,
            detail={
                "success": False,
                "stage": "moveit_relative_api",
                "message": "Generic PiPER-X move-relative is not implemented yet; arbitrary relative execution remains blocked.",
            },
        )

    return router
