#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import re
import shlex
from dataclasses import dataclass


DEFAULT_PIPELINE_ROOT = "/home/dase-hw101/piper-pipeline-testbed"


def _clean_instruction(text: str) -> str:
    text = text.strip()
    text = re.sub(r"\s+", " ", text)
    text = re.sub(r"\busing lap-?3b\b", "", text, flags=re.IGNORECASE)
    text = re.sub(r"\bwith lap-?3b\b", "", text, flags=re.IGNORECASE)
    text = re.sub(r"\bin shadow mode\b", "", text, flags=re.IGNORECASE)
    text = re.sub(r"\bin execute mode\b", "", text, flags=re.IGNORECASE)
    text = re.sub(r"\bexecute one action\b", "", text, flags=re.IGNORECASE)
    text = re.sub(r"\bshadow\b", "", text, flags=re.IGNORECASE)
    text = re.sub(r"\s+", " ", text).strip(" .")
    return text


@dataclass
class LapTask:
    instruction: str
    mode: str
    max_actions: int

    def command(self, pipeline_root: str = DEFAULT_PIPELINE_ROOT) -> list[str]:
        cmd = [
            "./tools/run_in_noetic_container.sh",
            "python3",
            "piper-on-bunker/scripts/run_lap_piper_action.py",
            "--instruction",
            self.instruction,
            "--max-actions",
            str(self.max_actions),
        ]
        if self.mode == "execute":
            cmd.append("--execute")
        shell = "cd " + shlex.quote(pipeline_root) + " && " + " ".join(shlex.quote(part) for part in cmd)
        return ["bash", "-lc", shell]

    def to_dict(self, pipeline_root: str = DEFAULT_PIPELINE_ROOT) -> dict:
        command = self.command(pipeline_root)
        return {
            "instruction": self.instruction,
            "mode": self.mode,
            "max_actions": self.max_actions,
            "command": command,
            "command_text": command[-1],
        }


def parse_task(message: str, *, default_max_actions: int = 1) -> LapTask:
    execute = bool(re.search(r"\bexecute\b", message, flags=re.IGNORECASE))
    instruction = _clean_instruction(message)
    return LapTask(
        instruction=instruction or "Move the gripper toward the target.",
        mode="execute" if execute else "shadow",
        max_actions=default_max_actions,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("message")
    parser.add_argument("--pipeline-root", default=DEFAULT_PIPELINE_ROOT)
    parser.add_argument("--default-max-actions", type=int, default=1)
    args = parser.parse_args()
    task = parse_task(args.message, default_max_actions=args.default_max_actions)
    print(json.dumps(task.to_dict(args.pipeline_root), sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
