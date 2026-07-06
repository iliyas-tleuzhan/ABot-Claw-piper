#!/usr/bin/env bash
set -eo pipefail

SKILL_FILE="${HOME}/.openclaw/workspace/skills/PIPER_ACTION/SKILL.md"
RUN_OPENCLAW=0

for arg in "$@"; do
  case "${arg}" in
    --run)
      RUN_OPENCLAW=1
      ;;
    -h|--help)
      echo "Usage: $0 [--run]"
      exit 0
      ;;
    *)
      echo "FAIL unknown argument: ${arg}" >&2
      exit 1
      ;;
  esac
done

echo "Checking OpenClaw CLI..."
openclaw --version
echo "PASS openclaw --version"

if [[ -f "${SKILL_FILE}" ]]; then
  echo "PASS skill file exists: ${SKILL_FILE}"
else
  echo "FAIL missing skill file: ${SKILL_FILE}" >&2
  exit 1
fi

if grep -q "localhost:8891" "${SKILL_FILE}"; then
  echo "PASS skill file uses localhost:8891"
else
  echo "FAIL skill file does not contain localhost:8891" >&2
  exit 1
fi

if grep -q "8890" "${SKILL_FILE}"; then
  echo "FAIL skill file still references 8890" >&2
  exit 1
else
  echo "PASS skill file does not reference 8890"
fi

echo "Restarting OpenClaw gateway..."
openclaw gateway restart
echo "PASS openclaw gateway restart"

cat <<'EOF'

Suggested manual commands:
openclaw agent --message "Move the Piper arm up."
openclaw agent --message "Move the Piper arm down."
openclaw agent --message "Open the Piper gripper."
openclaw agent --message "Close the Piper gripper."
EOF

if [[ "${RUN_OPENCLAW}" -eq 1 ]]; then
  echo "Running one safe OpenClaw command..."
  openclaw agent --message "Move the Piper arm up."
else
  echo "SKIP OpenClaw robot command; pass --run to execute one safe command."
fi
