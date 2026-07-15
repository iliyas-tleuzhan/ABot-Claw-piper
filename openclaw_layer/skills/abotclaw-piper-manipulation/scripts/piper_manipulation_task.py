#!/usr/bin/env python3
"""Parse OpenClaw tabletop manipulation text into a Piper runner command."""

from __future__ import annotations

import argparse
import json
import re
import shlex
from dataclasses import dataclass
from typing import Optional


DEFAULT_REPO_ROOT = "/home/dase-hw101/ABot-Claw"

STOP_WORDS = {
    "the",
    "a",
    "an",
    "please",
    "that",
    "this",
}

SOURCE_VERBS = (
    "pick up",
    "pick",
    "grab",
    "move",
    "put",
    "place",
)

DESTINATION_MARKERS = (
    " on top of ",
    " onto ",
    " on ",
    " to ",
    " into ",
)


@dataclass
class ManipulationTask:
    action: str
    source: str
    source_description: str
    destination: Optional[str]
    selected_robot: str = "Piper"

    def runner_args(self, execute: bool = False) -> list[str]:
        args = [
            "python3",
            "robot_layer/arm_piper/agent_server/run_piper_manipulation.py",
            "--task",
            "place" if self.destination else "pick",
            "--source",
            self.source,
        ]
        if self.destination:
            args.extend(["--destination", self.destination])
        args.append("--execute" if execute else "--plan-only")
        return args

    def shell_command(self, execute: bool = False, repo_root: str = DEFAULT_REPO_ROOT) -> str:
        runner = " ".join(shlex.quote(part) for part in self.runner_args(execute))
        return "cd " + shlex.quote(repo_root) + " && " + runner

    def command(self, execute: bool = False, repo_root: str = DEFAULT_REPO_ROOT) -> list[str]:
        return ["bash", "-lc", self.shell_command(execute, repo_root)]

    def to_dict(self, execute: bool = False, repo_root: str = DEFAULT_REPO_ROOT) -> dict:
        cmd = self.command(execute, repo_root)
        return {
            "action": self.action,
            "source": self.source,
            "source_description": self.source_description,
            "destination": self.destination,
            "selected_robot": self.selected_robot,
            "command": cmd,
            "command_text": self.shell_command(execute, repo_root),
        }


def clean_phrase(text: str) -> str:
    text = re.sub(r"[^a-z0-9\s-]", " ", text.lower())
    text = re.sub(r"\s+", " ", text).strip()
    return text


def strip_stop_words(text: str) -> str:
    words = [word for word in text.split() if word not in STOP_WORDS]
    return " ".join(words).strip()


def split_source_destination(text: str) -> tuple[str, Optional[str]]:
    for marker in DESTINATION_MARKERS:
        if marker in text:
            left, right = text.split(marker, 1)
            destination = strip_stop_words(right)
            return left.strip(), destination or None
    return text, None


def strip_source_verb(text: str) -> tuple[str, str]:
    for verb in SOURCE_VERBS:
        if text.startswith(verb + " "):
            return verb, text[len(verb):].strip()
    match = re.search(r"\b(" + "|".join(re.escape(v) for v in SOURCE_VERBS) + r")\b\s+(.+)", text)
    if match:
        return match.group(1), match.group(2).strip()
    raise ValueError("No manipulation action found")


def normalize_source(text: str) -> str:
    source = strip_stop_words(text)
    source = re.sub(r"\band\s+(place|put|move)\s+it\b.*$", "", source).strip()
    if not source:
        raise ValueError("No source object found")
    known_objects = (
        "cup",
        "bottle",
        "bowl",
        "can",
        "book",
        "cell phone",
        "remote",
    )
    for name in known_objects:
        if re.search(r"\b" + re.escape(name) + r"\b", source):
            return name
    return source


def parse_task(message: str) -> ManipulationTask:
    text = clean_phrase(message)
    verb, remainder = strip_source_verb(text)
    source_text, destination = split_source_destination(remainder)
    source = normalize_source(source_text)
    action = "place" if destination else "pick"
    return ManipulationTask(
        action=action,
        source=source,
        source_description=re.sub(
            r"\band\s+(place|put|move)\s+it\b.*$",
            "",
            strip_stop_words(source_text),
        ).strip(),
        destination=destination,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("message")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--repo-root", default=DEFAULT_REPO_ROOT)
    args = parser.parse_args()
    task = parse_task(args.message)
    print(json.dumps(task.to_dict(execute=args.execute, repo_root=args.repo_root), sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
