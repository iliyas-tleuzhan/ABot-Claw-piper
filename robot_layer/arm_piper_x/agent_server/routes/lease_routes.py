"""Lease routes for PiPER-X Agent Server."""

from __future__ import annotations

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel


class AcquireLeaseRequest(BaseModel):
    holder: str
    duration_s: float | None = None


class ReleaseLeaseRequest(BaseModel):
    lease_id: str


def create_router(lease_mgr) -> APIRouter:
    router = APIRouter(prefix="/lease", tags=["lease"])

    @router.post("/acquire")
    def acquire(req: AcquireLeaseRequest):
        result = lease_mgr.acquire(req.holder, req.duration_s)
        if not result.get("success", False):
            status_code = 409 if result.get("status") == "busy" else 400
            raise HTTPException(status_code=status_code, detail=result)
        return result

    @router.post("/release")
    def release(req: ReleaseLeaseRequest):
        result = lease_mgr.release(req.lease_id)
        if not result.get("success", False):
            raise HTTPException(status_code=404, detail=result)
        return result

    @router.get("/status")
    def status():
        return lease_mgr.status()

    return router
