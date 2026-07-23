#!/usr/bin/env bash
set -euo pipefail
curl -fsS http://127.0.0.1:8018/health && echo
curl -fsS http://127.0.0.1:8018/model-info && echo
