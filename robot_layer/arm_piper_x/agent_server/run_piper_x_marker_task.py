#!/usr/bin/env python3
"""Call the local PiPER-X marker/home HTTP API from ABot-Claw."""

from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Any, Dict, Optional
from urllib import error, request


DEFAULT_BASE_URL = "http://127.0.0.1:8892"


def request_json(
    method: str,
    url: str,
    payload: Optional[Dict[str, Any]] = None,
    token: Optional[str] = None,
) -> tuple[int, Dict[str, Any]]:
    data = None if payload is None else json.dumps(payload).encode("utf-8")
    headers = {"Accept": "application/json"}
    if payload is not None:
        headers["Content-Type"] = "application/json"
    if token:
        headers["Authorization"] = f"Bearer {token}"
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
        return 503, {
            "success": False,
            "stage": "api_connection",
            "message": str(exc),
            "contact_confirmed": False,
            "completion_type": "geometric_surface_approach",
        }


def print_result(status: int, body: Dict[str, Any]) -> int:
    print(json.dumps(body, indent=2, sort_keys=True))
    if status >= 400 or body.get("success") is False:
        return 1
    return 0


def task_payload(args: argparse.Namespace) -> Dict[str, Any]:
    return {
        "execute": args.execute,
        "pre_clearance_m": args.pre_clearance,
        "final_clearance_m": args.final_clearance,
        "retract_after": args.retract,
        "retract_distance_m": args.retract_distance,
        "final_velocity_scaling": args.final_velocity_scaling,
        "return_home_after": args.return_home_after,
        "home_duration_s": args.home_duration,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--base-url",
        default=os.environ.get("PIPER_X_MARKER_API_URL", DEFAULT_BASE_URL),
    )
    parser.add_argument("--token", default=os.environ.get("PIPER_TOUCH_API_TOKEN"))
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("health")

    for name in ("approach", "touch"):
        task = subparsers.add_parser(name)
        task.add_argument("--execute", action="store_true")
        task.add_argument("--plan-only", action="store_true")
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
    home.add_argument("--duration", type=float, default=6.0)

    save_home = subparsers.add_parser("save-home")
    save_home.add_argument("--pose-name", default="home")

    args = parser.parse_args()
    base_url = args.base_url.rstrip("/")

    if getattr(args, "plan_only", False):
        args.execute = False

    if args.command == "health":
        return print_result(
            *request_json("GET", f"{base_url}/health", token=args.token)
        )

    if args.command == "home":
        return print_result(
            *request_json(
                "POST",
                f"{base_url}/tools/piper/go-home",
                payload={"execute": args.execute, "duration_s": args.duration},
                token=args.token,
            )
        )

    if args.command == "save-home":
        return print_result(
            *request_json(
                "POST",
                f"{base_url}/tools/piper/save-home",
                payload={"pose_name": args.pose_name},
                token=args.token,
            )
        )

    endpoint = "approach-marker" if args.command == "approach" else "touch-marker"
    return print_result(
        *request_json(
            "POST",
            f"{base_url}/tools/piper/{endpoint}",
            payload=task_payload(args),
            token=args.token,
        )
    )


if __name__ == "__main__":
    raise SystemExit(main())
