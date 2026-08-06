#!/usr/bin/env python3
"""Call the PiPER-X Agent Server on port 8893."""

from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Any, Dict, Optional
from urllib import error, request


DEFAULT_BASE_URL = "http://127.0.0.1:8893"


def request_json(method: str, url: str, payload: Optional[Dict[str, Any]] = None) -> tuple[int, Dict[str, Any]]:
    data = None if payload is None else json.dumps(payload).encode("utf-8")
    headers = {"Accept": "application/json"}
    if payload is not None:
        headers["Content-Type"] = "application/json"
    req = request.Request(url, data=data, headers=headers, method=method)
    try:
        with request.urlopen(req, timeout=180.0) as resp:
            body = resp.read().decode("utf-8")
            return resp.status, json.loads(body) if body else {}
    except error.HTTPError as exc:
        body = exc.read().decode("utf-8", errors="replace")
        try:
            parsed = json.loads(body)
        except json.JSONDecodeError:
            parsed = {"detail": body}
        return exc.code, parsed
    except error.URLError as exc:
        return 503, {"success": False, "stage": "agent_connection", "message": str(exc)}


def print_result(status_code: int, body: Dict[str, Any]) -> int:
    print(json.dumps(body, indent=2, sort_keys=True))
    if status_code >= 400 or body.get("success") is False:
        return 1
    return 0


def acquire_lease(base_url: str, holder: str = "openclaw-agent-client") -> str:
    status_code, body = request_json(
        "POST",
        f"{base_url}/lease/acquire",
        {"holder": holder, "duration_s": 300.0},
    )
    if status_code >= 400 or not body.get("success", False):
        print(json.dumps(body, indent=2, sort_keys=True))
        raise SystemExit(1)
    return str(body["lease_id"])


def release_lease(base_url: str, lease_id: str) -> None:
    request_json("POST", f"{base_url}/lease/release", {"lease_id": lease_id})


def marker_payload(args: argparse.Namespace) -> Dict[str, Any]:
    return {
        "execute": args.execute,
        "lease_id": args.lease_id,
        "pre_clearance_m": args.pre_clearance,
        "final_clearance_m": args.final_clearance,
        "retract_after": args.retract,
        "retract_distance_m": args.retract_distance,
        "final_velocity_scaling": args.final_velocity_scaling,
        "return_home_after": args.return_home_after,
        "home_duration_s": args.home_duration,
    }


def command_needs_lease(args: argparse.Namespace) -> bool:
    return bool(getattr(args, "execute", False)) and args.command in {
        "approach",
        "touch",
        "home",
        "previous",
        "open-gripper",
        "close-gripper",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-url", default=os.environ.get("PIPER_X_AGENT_URL", DEFAULT_BASE_URL))
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("health")
    subparsers.add_parser("state")
    acquire = subparsers.add_parser("acquire-lease")
    acquire.add_argument("--holder", default="openclaw")
    acquire.add_argument("--duration-s", type=float, default=300.0)
    release = subparsers.add_parser("release-lease")
    release.add_argument("--lease-id", required=True)

    for name in ("approach", "touch"):
        task = subparsers.add_parser(name)
        task.add_argument("--execute", action="store_true")
        task.add_argument("--plan-only", action="store_true")
        task.add_argument("--lease-id")
        task.add_argument("--pre-clearance", type=float, default=0.05)
        task.add_argument("--final-clearance", type=float, default=0.005)
        task.add_argument("--retract", action="store_true")
        task.add_argument("--retract-distance", type=float, default=0.05)
        task.add_argument("--final-velocity-scaling", type=float, default=0.05)
        task.add_argument("--return-home-after", action="store_true")
        task.add_argument("--home-duration", type=float, default=6.0)

    home = subparsers.add_parser("home")
    home.add_argument("--execute", action="store_true")
    home.add_argument("--plan-only", action="store_true")
    home.add_argument("--lease-id")
    home.add_argument("--duration", type=float, default=6.0)
    previous = subparsers.add_parser("previous")
    previous.add_argument("--execute", action="store_true")
    previous.add_argument("--plan-only", action="store_true")
    previous.add_argument("--lease-id")
    previous.add_argument("--duration", type=float, default=6.0)
    save_home = subparsers.add_parser("save-home")
    save_home.add_argument("--pose-name", default="home")
    subparsers.add_parser("save-previous")
    for name in ("open-gripper", "close-gripper"):
        gripper = subparsers.add_parser(name)
        gripper.add_argument("--execute", action="store_true")
        gripper.add_argument("--plan-only", action="store_true")
        gripper.add_argument("--lease-id")
        gripper.add_argument("--width-m", type=float)
        gripper.add_argument("--effort-n", type=float)

    args = parser.parse_args()
    base_url = args.base_url.rstrip("/")
    if getattr(args, "plan_only", False):
        args.execute = False

    owned_lease_id = None
    if command_needs_lease(args) and not getattr(args, "lease_id", None):
        owned_lease_id = acquire_lease(base_url)
        args.lease_id = owned_lease_id

    try:
        if args.command == "health":
            return print_result(*request_json("GET", f"{base_url}/health"))
        if args.command == "state":
            return print_result(*request_json("GET", f"{base_url}/state"))
        if args.command == "acquire-lease":
            return print_result(
                *request_json(
                    "POST",
                    f"{base_url}/lease/acquire",
                    {"holder": args.holder, "duration_s": args.duration_s},
                )
            )
        if args.command == "release-lease":
            return print_result(
                *request_json("POST", f"{base_url}/lease/release", {"lease_id": args.lease_id})
            )
        if args.command == "home":
            return print_result(
                *request_json(
                    "POST",
                    f"{base_url}/tools/go-home",
                    {"execute": args.execute, "lease_id": args.lease_id, "duration_s": args.duration},
                )
            )
        if args.command == "previous":
            return print_result(
                *request_json(
                    "POST",
                    f"{base_url}/tools/go-previous",
                    {"execute": args.execute, "lease_id": args.lease_id, "duration_s": args.duration},
                )
            )
        if args.command == "save-home":
            return print_result(
                *request_json("POST", f"{base_url}/tools/save-home", {"pose_name": args.pose_name})
            )
        if args.command == "save-previous":
            return print_result(*request_json("POST", f"{base_url}/tools/save-previous", {}))
        if args.command in {"open-gripper", "close-gripper"}:
            payload = {
                "execute": args.execute,
                "lease_id": args.lease_id,
                "width_m": args.width_m,
                "effort_n": args.effort_n,
            }
            return print_result(*request_json("POST", f"{base_url}/tools/{args.command}", payload))
        endpoint = "approach-marker" if args.command == "approach" else "touch-marker"
        return print_result(*request_json("POST", f"{base_url}/tools/{endpoint}", marker_payload(args)))
    finally:
        if owned_lease_id:
            release_lease(base_url, owned_lease_id)


if __name__ == "__main__":
    raise SystemExit(main())
