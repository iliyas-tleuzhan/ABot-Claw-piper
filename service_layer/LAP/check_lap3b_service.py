#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys


def main() -> int:
    parser = argparse.ArgumentParser(description="Check LAP websocket policy reachability.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8016)
    args = parser.parse_args()

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
