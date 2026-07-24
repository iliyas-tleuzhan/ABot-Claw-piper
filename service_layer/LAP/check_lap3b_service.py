#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path


def _ensure_runtime_python() -> None:
    lap_venv = Path(os.environ.get("LAP_VENV", "/data/home/iliyas/abot-data/lap-venv"))
    target_python = lap_venv / "bin" / "python"
    if not target_python.exists():
        return
    if Path(sys.executable).resolve() == target_python.resolve():
        return
    os.execv(str(target_python), [str(target_python), __file__, *sys.argv[1:]])


def _extend_import_path() -> None:
    script_dir = Path(__file__).resolve().parent
    configured_src = Path(
        os.environ.get("LAP_SRC", "/data/home/iliyas/abot-data/lap-src")
    )
    candidates = (
        configured_src / "third_party" / "openpi" / "packages" / "openpi-client" / "src",
        script_dir / "lap-src" / "third_party" / "openpi" / "packages" / "openpi-client" / "src",
    )
    for candidate in candidates:
        if candidate.exists():
            sys.path.insert(0, str(candidate))
            return


def main() -> int:
    _ensure_runtime_python()

    parser = argparse.ArgumentParser(description="Check LAP websocket policy reachability.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8016)
    args = parser.parse_args()

    _extend_import_path()

    try:
        from openpi_client import websocket_client_policy
    except Exception as exc:
        print(json.dumps({"ok": False, "error": f"openpi_client import failed: {exc!r}"}))
        return 2

    try:
        client = websocket_client_policy.WebsocketClientPolicy(args.host, args.port)
        metadata = client.get_server_metadata()
    except Exception as exc:
        print(json.dumps({"ok": False, "error": repr(exc), "host": args.host, "port": args.port}))
        return 1

    print(json.dumps({"ok": True, "host": args.host, "port": args.port, "metadata": metadata}, default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
