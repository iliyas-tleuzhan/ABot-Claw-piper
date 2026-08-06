"""In-memory lease for PiPER-X physical command ownership."""

from __future__ import annotations

import time
import uuid
from dataclasses import dataclass
from threading import Lock
from typing import Optional


@dataclass
class Lease:
    lease_id: str
    holder: str
    granted_at: float
    expires_at: float


class LeaseManager:
    def __init__(self, max_duration_s: float = 300.0):
        self._max_duration_s = float(max_duration_s)
        self._lock = Lock()
        self._current: Optional[Lease] = None

    def _expire_if_needed(self) -> None:
        if self._current and time.time() >= self._current.expires_at:
            self._current = None

    def acquire(self, holder: str, duration_s: float | None = None) -> dict:
        holder = holder.strip()
        if not holder:
            return {"success": False, "status": "invalid_request", "message": "holder is required"}
        requested = self._max_duration_s if duration_s is None else float(duration_s)
        if requested <= 0.0 or requested > self._max_duration_s:
            return {
                "success": False,
                "status": "invalid_request",
                "message": f"duration_s must be in (0, {self._max_duration_s}]",
            }
        with self._lock:
            self._expire_if_needed()
            if self._current is not None and self._current.holder != holder:
                return {
                    "success": False,
                    "status": "busy",
                    "holder": self._current.holder,
                    "expires_at": self._current.expires_at,
                }
            if self._current is not None and self._current.holder == holder:
                return {
                    "success": True,
                    "status": "already_held",
                    "lease_id": self._current.lease_id,
                    "holder": self._current.holder,
                    "expires_at": self._current.expires_at,
                }
            lease = Lease(
                lease_id=str(uuid.uuid4()),
                holder=holder,
                granted_at=time.time(),
                expires_at=time.time() + requested,
            )
            self._current = lease
            return {
                "success": True,
                "status": "granted",
                "lease_id": lease.lease_id,
                "holder": lease.holder,
                "expires_at": lease.expires_at,
                "max_duration_s": self._max_duration_s,
            }

    def release(self, lease_id: str) -> dict:
        with self._lock:
            self._expire_if_needed()
            if self._current is None:
                return {"success": False, "status": "not_found"}
            if self._current.lease_id != lease_id:
                return {"success": False, "status": "not_owner", "holder": self._current.holder}
            holder = self._current.holder
            self._current = None
            return {"success": True, "status": "released", "holder": holder}

    def require(self, lease_id: str | None) -> tuple[bool, dict | None]:
        with self._lock:
            self._expire_if_needed()
            if self._current is None:
                return False, {"stage": "lease", "message": "no active PiPER-X lease"}
            if not lease_id or lease_id != self._current.lease_id:
                return False, {"stage": "lease", "message": "valid lease_id is required"}
            return True, None

    def status(self) -> dict:
        with self._lock:
            self._expire_if_needed()
            if self._current is None:
                return {"held": False}
            return {
                "held": True,
                "lease_id": self._current.lease_id,
                "holder": self._current.holder,
                "granted_at": self._current.granted_at,
                "expires_at": self._current.expires_at,
                "remaining_s": max(0.0, self._current.expires_at - time.time()),
            }
