#!/usr/bin/env python3
from __future__ import annotations

import argparse
import urllib.request


def main() -> int:
    parser = argparse.ArgumentParser(description="Check OpenPI websocket server health endpoint.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8017)
    args = parser.parse_args()
    url = f"http://{args.host}:{args.port}/healthz"
    with urllib.request.urlopen(url, timeout=3.0) as response:
        body = response.read().decode("utf-8", "replace")
    print(f"{url}: {body.strip()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

