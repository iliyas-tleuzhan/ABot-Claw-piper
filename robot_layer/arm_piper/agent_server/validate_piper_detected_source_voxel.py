#!/usr/bin/env python3
"""Validate the currently detected Piper source as a proposed grasp voxel.

This is a read-only/plan-only helper. It submits the existing manipulation
planner through the Agent Server lease path, never uses --execute, and never
commands the arm or gripper.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from typing import Any


def parse_prefixed_json(line: str, prefix: str) -> Any | None:
    if not line.startswith(prefix + " "):
        return None
    try:
        return json.loads(line[len(prefix) + 1 :])
    except json.JSONDecodeError:
        return None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", default="cup")
    parser.add_argument("--agent-url", default="http://127.0.0.1:8888")
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--exploratory-planning-budget", type=float, default=60.0)
    parser.add_argument("--hover-height", type=float, default=0.07)
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    runner = os.path.join(script_dir, "run_piper_manipulation.py")
    cmd = [
        sys.executable,
        runner,
        "--task",
        "pick",
        "--source",
        args.source,
        "--source-provider",
        "perception",
        "--plan-only",
        "--hover-height",
        str(args.hover_height),
        "--agent-url",
        args.agent_url,
        "--timeout",
        str(args.timeout),
        "--explore-unvalidated-candidates",
        "--exploratory-planning-budget",
        str(args.exploratory_planning_budget),
    ]

    print("CURRENT_SOURCE_VALIDATOR_COMMAND_JSON " + json.dumps(cmd), flush=True)
    proc = subprocess.run(cmd, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    print(proc.stdout, end="")

    source_summary = None
    selected = None
    validation = None
    plan_complete = "PLAN_ONLY_COMPLETE" in proc.stdout
    for line in proc.stdout.splitlines():
        source_summary = parse_prefixed_json(line, "SOURCE_DETECTION_SUMMARY_JSON") or source_summary
        selected = parse_prefixed_json(line, "SELECTED_TCP_YAW_CANDIDATE_JSON") or selected
        validation = parse_prefixed_json(line, "VALIDATED_GRASP_REGION_SELECTION_JSON") or validation

    success = bool(
        plan_complete
        and validation
        and validation.get("transit_result", {}).get("success")
        and float(validation.get("descend_fraction") or 0.0) >= 0.999
        and float(validation.get("lift_fraction") or 0.0) >= 0.999
    )
    result = {
        "plan_only_complete": plan_complete,
        "transit_success": bool(validation and validation.get("transit_result", {}).get("success")),
        "descend_fraction": None if not validation else validation.get("descend_fraction"),
        "lift_fraction": None if not validation else validation.get("lift_fraction"),
        "source_summary": source_summary,
        "selected_candidate_phase": None if not selected else selected.get("phase"),
        "validated": success,
    }
    print("CURRENT_SOURCE_VALIDATOR_RESULT_JSON " + json.dumps(result, sort_keys=True), flush=True)

    if success and source_summary and selected:
        xyz = source_summary["selected_base_xyz"]
        hover_height = float(args.hover_height)
        voxel = {
            "voxel_id": "REVIEW_new_detected_source_voxel",
            "usage": ["source_pick"],
            "execution_validated": False,
            "validation_scope": "individual_voxel_proposed_for_review",
            "source_surface_xyz": xyz,
            "source_surface_bounds": {
                "min": [float(x) - 0.005 for x in xyz],
                "max": [float(x) + 0.005 for x in xyz],
            },
            "tcp_hover_xyz": [float(xyz[0]), float(xyz[1]), float(xyz[2]) + hover_height],
            "tcp_quaternion": selected.get("quat"),
            "approach_angle_deg": selected.get("approach_angle_deg"),
            "preferred_ik_seed": selected.get("seed_joint_state"),
            "actual_ik_solution": selected.get("preferred_ik_solution"),
            "validated_hover_height_m": hover_height,
            "descend_fraction": validation.get("descend_fraction"),
            "lift_fraction": validation.get("lift_fraction"),
        }
        print("PROPOSED_VALIDATED_VOXEL_YAML_JSON " + json.dumps(voxel, sort_keys=True), flush=True)
    return proc.returncode


if __name__ == "__main__":
    raise SystemExit(main())
