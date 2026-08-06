"""Restricted PiPER-X SDK backed by the ROS 2 marker/home HTTP bridge."""

from __future__ import annotations

import json
import math
import os
from typing import Any, Dict, Optional
from urllib import error, request


class PiperXAgentSdk:
    def __init__(self, base_url: str = "http://127.0.0.1:8892", timeout_s: float = 180.0):
        self.base_url = base_url.rstrip("/")
        self.timeout_s = float(timeout_s)
        self.token = os.environ.get("PIPER_TOUCH_API_TOKEN", "").strip() or None

    def _request(self, method: str, path: str, payload: Optional[Dict[str, Any]] = None) -> tuple[int, Dict[str, Any]]:
        body = None if payload is None else json.dumps(payload).encode("utf-8")
        headers = {"Accept": "application/json"}
        if payload is not None:
            headers["Content-Type"] = "application/json"
        if self.token:
            headers["Authorization"] = f"Bearer {self.token}"
        req = request.Request(f"{self.base_url}{path}", data=body, headers=headers, method=method)
        try:
            with request.urlopen(req, timeout=self.timeout_s) as response:
                raw = response.read().decode("utf-8", errors="replace")
                return response.status, json.loads(raw) if raw else {}
        except error.HTTPError as exc:
            raw = exc.read().decode("utf-8", errors="replace")
            try:
                parsed = json.loads(raw) if raw else {}
            except json.JSONDecodeError:
                parsed = {"detail": raw}
            return exc.code, parsed
        except error.URLError as exc:
            return 503, {"success": False, "stage": "marker_api_connection", "message": str(exc)}

    def health(self) -> dict:
        return self._request("GET", "/health")[1]

    def approach_marker(self, payload: dict) -> tuple[int, dict]:
        return self._request("POST", "/tools/piper/approach-marker", payload)

    def touch_marker(self, payload: dict) -> tuple[int, dict]:
        return self._request("POST", "/tools/piper/touch-marker", payload)

    def go_home(self, payload: dict) -> tuple[int, dict]:
        return self._request("POST", "/tools/piper/go-home", payload)

    def save_home(self, payload: dict) -> tuple[int, dict]:
        return self._request("POST", "/tools/piper/save-home", payload)

    @staticmethod
    def validate_pose_payload(payload: dict) -> tuple[bool, str]:
        position = payload.get("position_m")
        orientation = payload.get("orientation_xyzw")
        if not isinstance(position, list) or len(position) != 3:
            return False, "position_m must be a list of three metres values"
        if not isinstance(orientation, list) or len(orientation) != 4:
            return False, "orientation_xyzw must be a list of four quaternion values"
        values = [float(value) for value in position + orientation]
        if not all(math.isfinite(value) for value in values):
            return False, "pose values must be finite"
        return True, "ok"
