"""HTTP service selection helpers for Piper perception SDKs."""

from __future__ import annotations

import os
from typing import Optional, Tuple
from urllib.parse import urlparse, urlunparse

import requests


def _health_url(service_url: str) -> str:
    parsed = urlparse(service_url)
    path = parsed.path.rstrip("/")
    if path.endswith("/grasp/detect"):
        path = path[: -len("/grasp/detect")]
    elif path.endswith("/detect"):
        path = path[: -len("/detect")]
    return urlunparse(parsed._replace(path=path + "/health", params="", query="", fragment=""))


def service_health_status(service_url: str, timeout: float = 1.0) -> Tuple[str, dict]:
    try:
        resp = requests.get(_health_url(service_url), timeout=timeout)
        resp.raise_for_status()
        body = resp.json()
    except Exception as exc:
        return "unavailable", {"error": str(exc)}
    status = str(body.get("status", "")).lower()
    if status == "ok":
        return "ok", body
    if status == "degraded":
        return "degraded", body
    return "unavailable", body


def select_service_url(
    *,
    service_name: str,
    explicit_url: Optional[str],
    env_var: str,
    configured_url: str,
    local_url: str,
    timeout: float = 1.0,
) -> Tuple[str, str, str]:
    """Return selected URL, backend label, and health status."""
    if explicit_url:
        return explicit_url, "explicit", "not_checked"

    env_url = os.getenv(env_var)
    if env_url:
        status, _body = service_health_status(env_url, timeout=timeout)
        print(f"{service_name}: selected {env_var}={env_url} health={status}")
        return env_url, "env", status

    remote_status, _remote_body = service_health_status(configured_url, timeout=timeout)
    if remote_status == "ok":
        print(f"{service_name}: selected remote {configured_url} health=ok")
        return configured_url, "remote", remote_status

    local_status, _local_body = service_health_status(local_url, timeout=timeout)
    if local_status == "ok":
        print(
            f"{service_name}: selected local {local_url} "
            f"because configured service health={remote_status}"
        )
        return local_url, "local", local_status

    print(
        f"{service_name}: using configured {configured_url}; "
        f"configured health={remote_status}, local health={local_status}"
    )
    return configured_url, "configured", remote_status
