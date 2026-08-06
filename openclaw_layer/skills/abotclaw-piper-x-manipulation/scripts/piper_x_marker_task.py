#!/usr/bin/env python3
"""Parse OpenClaw text into a bounded PiPER-X Agent Server command."""

from __future__ import annotations

import argparse
import json
import re
import shlex
from dataclasses import dataclass


DEFAULT_REPO_ROOT = "/home/dase-hw101/ABot-Claw"


@dataclass
class PiperXMarkerTask:
    action: str
    selected_robot: str = "PiPER-X"

    def runner_args(self, execute: bool = True) -> list[str]:
        base = ["python3", "robot_layer/arm_piper_x/agent_server/run_piper_x_agent_task.py"]
        if self.action == "approach":
            return base + ["approach", "--execute" if execute else "--plan-only"]
        if self.action == "touch":
            args = base + [
                "touch",
                "--retract",
                "--return-home-after",
                "--execute" if execute else "--plan-only",
            ]
            return args
        if self.action == "home":
            return base + ["home", "--execute" if execute else "--plan-only"]
        if self.action == "save-home":
            return base + ["save-home"]
        if self.action == "previous":
            return base + ["previous", "--execute" if execute else "--plan-only"]
        if self.action == "save-previous":
            return base + ["save-previous"]
        if self.action == "open-gripper":
            return base + ["open-gripper", "--execute" if execute else "--plan-only"]
        if self.action == "close-gripper":
            return base + ["close-gripper", "--execute" if execute else "--plan-only"]
        return base + ["health"]

    def shell_command(self, execute: bool = True, repo_root: str = DEFAULT_REPO_ROOT) -> str:
        runner = " ".join(shlex.quote(part) for part in self.runner_args(execute))
        return "cd " + shlex.quote(repo_root) + " && " + runner

    def to_dict(self, execute: bool = True, repo_root: str = DEFAULT_REPO_ROOT) -> dict:
        return {
            "action": self.action,
            "selected_robot": self.selected_robot,
            "command_text": self.shell_command(execute=execute, repo_root=repo_root),
            "contact_confirmed": False,
            "completion_type": (
                "gripper_width_command"
                if self.action in {"open-gripper", "close-gripper"}
                else (
                    "saved_pose_command"
                    if self.action in {"home", "previous", "save-home", "save-previous"}
                    else "geometric_surface_approach"
                )
            ),
        }


def parse_task(message: str) -> PiperXMarkerTask:
    text = re.sub(r"[^a-z0-9\s-]", " ", message.lower())
    text = re.sub(r"\s+", " ", text).strip()

    if re.search(r"\b(save|remember|update|set)\b.*\b(previous|last)\b.*\b(pose|position)\b", text):
        return PiperXMarkerTask("save-previous")
    if re.search(r"\b(save|remember|update|set)\b.*\b(home|home pose)\b", text):
        return PiperXMarkerTask("save-home")
    if re.search(r"\b(go|return|move)\b.*\bhome\b", text):
        return PiperXMarkerTask("home")
    if re.search(r"\b(go|return|move)\b.*\b(previous|last)\b.*\b(pose|position|place|spot)\b", text):
        return PiperXMarkerTask("previous")
    if re.search(r"\b(go|return|move)\b.*\bback\b", text):
        return PiperXMarkerTask("previous")
    if re.search(r"\b(open|release)\b.*\b(gripper|claw|hand)\b", text):
        return PiperXMarkerTask("open-gripper")
    if re.search(r"\b(close|shut)\b.*\b(gripper|claw|hand)\b", text):
        return PiperXMarkerTask("close-gripper")
    if re.search(r"\b(touch|press|tap|contact)\b", text):
        return PiperXMarkerTask("touch")
    if re.search(r"\b(approach|point|move)\b.*\b(marker|aruco|marked)\b", text):
        return PiperXMarkerTask("approach")
    raise ValueError("No PiPER-X marker/home action found")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("message")
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--repo-root", default=DEFAULT_REPO_ROOT)
    args = parser.parse_args()
    task = parse_task(args.message)
    print(
        json.dumps(
            task.to_dict(execute=not args.plan_only, repo_root=args.repo_root),
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
