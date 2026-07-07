#!/usr/bin/env bash
set -eo pipefail

CONTAINER="abot-piper-noetic"
PORT="8891"
OLD_PORT="8890"
CONTAINER_REPO="/root/ABot-Claw"

if command -v docker >/dev/null 2>&1; then
  echo "PASS Docker command exists"
else
  echo "FAIL Docker command not found"
  exit 1
fi

if docker container inspect "${CONTAINER}" >/dev/null 2>&1; then
  state="$(docker inspect -f '{{.State.Status}}' "${CONTAINER}")"
  echo "PASS container ${CONTAINER} exists (${state})"
  if [[ "${state}" != "running" ]]; then
    echo "Container is stopped. Start it with:"
    echo "docker start ${CONTAINER}"
  elif docker exec "${CONTAINER}" test -d "${CONTAINER_REPO}" >/dev/null 2>&1; then
    echo "PASS container repo path exists: ${CONTAINER_REPO}"
  else
    echo "WARNING: container cannot see ${CONTAINER_REPO}"
    echo "The repo now lives at ~/ABot-Claw. Recreate or remount ${CONTAINER} with:"
    echo "  -v /home/dase-hw101/ABot-Claw:/root/ABot-Claw"
  fi
else
  echo "FAIL container ${CONTAINER} does not exist"
  exit 1
fi

if curl --noproxy '*' -fsS --max-time 2 "http://localhost:${PORT}/health" >/dev/null 2>&1; then
  echo "PASS port ${PORT} responds to /health"
else
  echo "INFO port ${PORT} is not responding to /health"
fi

if command -v ss >/dev/null 2>&1; then
  if ss -ltn | awk '{print $4}' | grep -Eq "(^|:)${OLD_PORT}$"; then
    echo "WARNING: old port 8890 is active; do not use it"
  else
    echo "PASS old port ${OLD_PORT} is not listening"
  fi
elif command -v lsof >/dev/null 2>&1; then
  if lsof -nP -iTCP:"${OLD_PORT}" -sTCP:LISTEN >/dev/null 2>&1; then
    echo "WARNING: old port 8890 is active; do not use it"
  else
    echo "PASS old port ${OLD_PORT} is not listening"
  fi
else
  echo "INFO cannot check old port ${OLD_PORT}; install ss or lsof"
fi
