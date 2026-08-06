#!/usr/bin/env bash
set -euo pipefail

repo_root="/home/dase-hw101/ABot-Claw"
unit_name="piper-x-agent-server.service"
source_unit="${repo_root}/deployment/systemd/${unit_name}"
target_dir="${HOME}/.config/systemd/user"
target_unit="${target_dir}/${unit_name}"

mkdir -p "${target_dir}"

if [ -f "${target_unit}" ]; then
  cp "${target_unit}" "${target_unit}.bak.$(date +%Y%m%d_%H%M%S)"
fi

cp "${source_unit}" "${target_unit}"
systemctl --user daemon-reload
systemctl --user enable "${unit_name}"

cat <<EOF
Installed ${target_unit}

Start:
  systemctl --user start ${unit_name}

Status:
  systemctl --user status ${unit_name}

Logs:
  journalctl --user -u ${unit_name} -f

Physical execution remains disabled unless the operator exports:
  systemctl --user set-environment PIPER_X_AGENT_ALLOW_EXECUTION=1

The lower-level marker bridge on 8892 must also be running and execution-enabled
for physical marker/home commands.
EOF
