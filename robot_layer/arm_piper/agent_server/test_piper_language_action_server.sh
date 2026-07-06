#!/usr/bin/env bash
set -eo pipefail

BASE_URL="${PIPER_LANGUAGE_ACTION_URL:-http://localhost:8891}"
MARKER="piper_language_action_server_v1"
RUN_GRIPPER=0
RUN_MOVE=0

for arg in "$@"; do
  case "${arg}" in
    --gripper)
      RUN_GRIPPER=1
      ;;
    --move)
      RUN_MOVE=1
      ;;
    -h|--help)
      echo "Usage: $0 [--gripper] [--move]"
      exit 0
      ;;
    *)
      echo "FAIL unknown argument: ${arg}" >&2
      exit 1
      ;;
  esac
done

print_json() {
  if command -v jq >/dev/null 2>&1; then
    printf '%s\n' "$1" | jq .
  else
    printf '%s\n' "$1"
  fi
}

require_contains() {
  local name="$1"
  local response="$2"
  local expected="$3"
  if [[ "${response}" == *"${expected}"* ]]; then
    echo "PASS ${name}"
  else
    echo "FAIL ${name}: expected response to contain ${expected}" >&2
    print_json "${response}" >&2
    exit 1
  fi
}

post_json() {
  local path="$1"
  local data="$2"
  curl -fsS -X POST "${BASE_URL}${path}" \
    -H 'Content-Type: application/json' \
    -d "${data}"
}

echo "Testing Piper language action server at ${BASE_URL}"

health="$(curl -fsS "${BASE_URL}/health")"
print_json "${health}"
require_contains "health marker" "${health}" "${MARKER}"

state="$(curl -fsS "${BASE_URL}/state")"
print_json "${state}"
require_contains "state marker" "${state}" "${MARKER}"
require_contains "state joint_positions" "${state}" "joint_positions"
require_contains "state gripper_position" "${state}" "gripper_position"
require_contains "state ros_joint_names" "${state}" "ros_joint_names"
require_contains "state ros_joint_positions" "${state}" "ros_joint_positions"

if [[ "${RUN_GRIPPER}" -eq 1 ]]; then
  open_response="$(curl -fsS -X POST "${BASE_URL}/open_gripper")"
  print_json "${open_response}"
  require_contains "open gripper" "${open_response}" '"success":true'

  close_response="$(curl -fsS -X POST "${BASE_URL}/close_gripper")"
  print_json "${close_response}"
  require_contains "close gripper" "${close_response}" '"success":true'
else
  echo "SKIP gripper tests; pass --gripper to run them."
fi

if [[ "${RUN_MOVE}" -eq 1 ]]; then
  up_response="$(post_json "/move_up" '{"joint_step":0.04,"speed":0.05,"accel":0.05}')"
  print_json "${up_response}"
  require_contains "move up" "${up_response}" '"success":true'

  sleep 1

  down_response="$(post_json "/move_down" '{"joint_step":0.04,"speed":0.05,"accel":0.05}')"
  print_json "${down_response}"
  require_contains "move down" "${down_response}" '"success":true'
else
  echo "SKIP movement tests; pass --move to run safe arm nudges."
fi

echo "PASS Piper language action server checks complete."
