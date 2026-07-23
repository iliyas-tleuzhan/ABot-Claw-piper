"""VLAC action-preview service.

Shadow-only service for VLA action generation. It never executes actions and
never exposes a robot command endpoint.
"""

from __future__ import annotations

import base64
import io
import json
import math
import os
import re
import threading
import time
from contextlib import asynccontextmanager
from typing import Any, Dict, List, Optional
from urllib.parse import urlparse

import requests
import torch
import uvicorn
from fastapi import FastAPI, HTTPException, Request
from PIL import Image
from pydantic import BaseModel, Field, model_validator

from evo_vlac import GAC_model

SERVICE_VERSION = "0.1.0"
MODEL_NAME = "InternRobotics/VLAC"
MODEL_PATH = os.getenv("VLAC_MODEL_PATH", "/model")
MODEL_TYPE = os.getenv("VLAC_MODEL_TYPE", "internvl2")
DEVICE_MAP = os.getenv("VLAC_POLICY_DEVICE", os.getenv("VLAC_DEVICE", "cuda:0"))
MAX_IMAGE_BYTES = int(os.getenv("VLAC_POLICY_MAX_IMAGE_BYTES", "8000000"))
MAX_REQUEST_BYTES = int(os.getenv("VLAC_POLICY_MAX_REQUEST_BYTES", "26000000"))
ALLOW_REMOTE_IMAGE_URLS = os.getenv("VLAC_POLICY_ALLOW_REMOTE_IMAGE_URLS", "0") == "1"
ALLOWED_REMOTE_IMAGE_HOSTS = {host.strip() for host in os.getenv("VLAC_POLICY_ALLOWED_IMAGE_HOSTS", "").split(",") if host.strip()}
ACTION_SEMANTICS = "unknown_songling_convention"
GRIPPER_PLACEHOLDER = 0.0

POLICY: Optional[GAC_model] = None
INFER_LOCK = threading.Lock()


def _finite(value: float) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("non-finite value")
    return value


class EndEffectorState(BaseModel):
    x_m: float
    y_m: float
    z_m: float
    roll_rad: float
    pitch_rad: float
    yaw_rad: float
    gripper_m: float = 0.0

    @model_validator(mode="after")
    def finite_values(self):
        for name, value in self.model_dump().items():
            if not math.isfinite(float(value)):
                raise ValueError(f"non-finite end_effector_state field: {name}")
        return self


class ActionPreviewRequest(BaseModel):
    images: List[str] = Field(..., min_length=1, max_length=3)
    task_description: str
    end_effector_state: EndEffectorState
    history: List[Dict[str, Any]] = Field(default_factory=list)

    @model_validator(mode="after")
    def validate_instruction(self):
        if not self.task_description.strip():
            raise ValueError("task_description cannot be empty")
        return self


class ParsedAction(BaseModel):
    raw_translation_m: List[Optional[float]] = Field(default_factory=lambda: [None, None, None])
    raw_rotation_rad: List[Optional[float]] = Field(default_factory=lambda: [None, None, None])
    gripper_command_raw: Optional[float] = None
    action_semantics: str = ACTION_SEMANTICS
    parse_reliability: str = "failed"
    model_confidence: Optional[float] = None
    execution_allowed: bool = False


def _normalize_image_input(image_input: str) -> Image.Image:
    payload = image_input.strip()
    try:
        if payload.startswith(("http://", "https://")):
            if not ALLOW_REMOTE_IMAGE_URLS:
                raise ValueError("remote image URLs are disabled by default")
            host = urlparse(payload).hostname
            if not host or host not in ALLOWED_REMOTE_IMAGE_HOSTS:
                raise ValueError("remote image host is not allowlisted")
            response = requests.get(payload, timeout=15)
            response.raise_for_status()
            data = response.content
        else:
            if payload.startswith("data:image"):
                payload = payload.split(",", 1)[1]
            data = base64.b64decode(payload, validate=True)
        if len(data) > MAX_IMAGE_BYTES:
            raise ValueError(f"image exceeds limit: {len(data)} > {MAX_IMAGE_BYTES}")
        image = Image.open(io.BytesIO(data)).convert("RGB")
        if image.width < 8 or image.height < 8:
            raise ValueError("image is too small")
        return image
    except Exception as exc:
        raise HTTPException(status_code=400, detail=f"Invalid image payload: {exc}")


def _si_to_legacy_model_units(state: EndEffectorState) -> List[float]:
    # The bundled vla_example.py describes input state as XYZ in 0.001 mm and
    # RPY in 0.001 degrees. GAC_model.format_state then integer-divides by
    # 1000, producing mm/degrees in the prompt. PiPER gripper units are not
    # verified for VLAC, so the service passes an explicit zero placeholder
    # into format_state instead of inventing a conversion.
    return [
        _finite(state.x_m) * 1_000_000.0,
        _finite(state.y_m) * 1_000_000.0,
        _finite(state.z_m) * 1_000_000.0,
        math.degrees(_finite(state.roll_rad)) * 1000.0,
        math.degrees(_finite(state.pitch_rad)) * 1000.0,
        math.degrees(_finite(state.yaw_rad)) * 1000.0,
        GRIPPER_PLACEHOLDER,
    ]


def _parse_raw_actions(raw: Any) -> tuple[List[ParsedAction], str, List[str]]:
    warnings: List[str] = []
    text = raw if isinstance(raw, str) else json.dumps(raw, sort_keys=True)
    pattern = re.compile(
        r"x:\s*([-+0-9.eE]+)\s*mm,\s*y:\s*([-+0-9.eE]+)\s*mm,\s*z:\s*([-+0-9.eE]+)\s*mm,\s*"
        r"roll:\s*([-+0-9.eE]+)\s*degrees,\s*pitch:\s*([-+0-9.eE]+)\s*degrees,\s*yaw:\s*([-+0-9.eE]+)\s*degrees,\s*open:\s*([-+0-9.eE]+)"
    )
    actions: List[ParsedAction] = []
    for match in pattern.finditer(text):
        vals = [float(item) for item in match.groups()]
        if all(math.isfinite(v) for v in vals):
            actions.append(
                ParsedAction(
                    raw_translation_m=[vals[0] / 1000.0, vals[1] / 1000.0, vals[2] / 1000.0],
                    raw_rotation_rad=[math.radians(vals[3]), math.radians(vals[4]), math.radians(vals[5])],
                    gripper_command_raw=vals[6],
                    action_semantics=ACTION_SEMANTICS,
                    parse_reliability="exact_grammar_match",
                    model_confidence=None,
                    execution_allowed=False,
                )
            )
    if actions:
        return actions, "exact_grammar_match", warnings
    nums = [float(v) for v in re.findall(r"[-+]?(?:\d+\.\d+|\d+)(?:[eE][-+]?\d+)?", text)]
    if len(nums) >= 7:
        warnings.append("Parsed numeric action without verified surrounding grammar. Values are preserved as raw Songling-format fields only.")
        return [
            ParsedAction(
                raw_translation_m=[nums[0] / 1000.0, nums[1] / 1000.0, nums[2] / 1000.0],
                raw_rotation_rad=[math.radians(nums[3]), math.radians(nums[4]), math.radians(nums[5])],
                gripper_command_raw=nums[6],
                action_semantics=ACTION_SEMANTICS,
                parse_reliability="assumed_numeric_layout",
                model_confidence=None,
                execution_allowed=False,
            )
        ], "assumed_numeric_layout", warnings
    warnings.append("Could not parse VLAC output into a verified action grammar.")
    return [], "failed", warnings


def _gpu_info() -> Dict[str, Any]:
    info: Dict[str, Any] = {"device": DEVICE_MAP, "cuda_available": torch.cuda.is_available()}
    if torch.cuda.is_available():
        try:
            idx = int(DEVICE_MAP.split(":", 1)[1]) if DEVICE_MAP.startswith("cuda:") else torch.cuda.current_device()
            info.update(
                {
                    "gpu_index": idx,
                    "gpu_name": torch.cuda.get_device_name(idx),
                    "memory_allocated_mb": round(torch.cuda.memory_allocated(idx) / 1024 / 1024, 1),
                    "memory_reserved_mb": round(torch.cuda.memory_reserved(idx) / 1024 / 1024, 1),
                }
            )
        except Exception as exc:
            info["gpu_error"] = repr(exc)
    return info


@asynccontextmanager
async def lifespan(app: FastAPI):
    global POLICY
    policy = GAC_model(tag="Policy")
    policy.init_model(model_path=MODEL_PATH, model_type=MODEL_TYPE, device_map=DEVICE_MAP)
    policy.temperature = 0.5
    policy.top_k = 1
    policy.set_config()
    policy.set_system_prompt()
    POLICY = policy
    yield
    POLICY = None


app = FastAPI(title="ABot-Claw VLAC Policy Action Preview", version=SERVICE_VERSION, lifespan=lifespan)


@app.middleware("http")
async def request_size_guard(request: Request, call_next):
    length = request.headers.get("content-length")
    if length and int(length) > MAX_REQUEST_BYTES:
        raise HTTPException(status_code=413, detail="request too large")
    return await call_next(request)


@app.get("/health")
def health() -> Dict[str, Any]:
    return {
        "status": "ok",
        "service": "vlac_policy_action_preview",
        "version": SERVICE_VERSION,
        "model_loaded": POLICY is not None,
        "model_path": MODEL_PATH,
        "model_type": MODEL_TYPE,
        "shadow_mode": True,
        "execution_allowed": False,
        "execution_disabled": True,
        "inference_interruption_supported": False,
        "gpu": _gpu_info(),
    }


@app.get("/model-info")
def model_info() -> Dict[str, Any]:
    return {
        "checkpoint": MODEL_PATH,
        "model": MODEL_NAME,
        "parameter_count": "unknown",
        "gpu": _gpu_info(),
        "expected_camera_count": "1-3",
        "expected_state_representation": "7D end-effector state [x, y, z, roll, pitch, yaw, gripper]",
        "expected_action_representation": "raw Songling-format action text {x mm, y mm, z mm, roll degrees, pitch degrees, yaw degrees, open}",
        "expected_units": {
            "example_input": "XYZ in 0.001 mm, RPY in 0.001 degrees before GAC_model.format_state",
            "prompt_state_after_format_state": "XYZ in mm, RPY in degrees, gripper placeholder 0.0 with gripper_format=False",
            "action_output": "raw mm/degrees/open values according to prompt; relative-vs-absolute semantics are not proven",
        },
        "action_semantics": ACTION_SEMANTICS,
        "model_confidence": None,
        "remote_image_urls_enabled": ALLOW_REMOTE_IMAGE_URLS,
        "inference_interruption_supported": False,
        "shadow_only": True,
        "execution_allowed": False,
    }


@app.post("/action-preview")
def action_preview(req: ActionPreviewRequest) -> Dict[str, Any]:
    if POLICY is None:
        raise HTTPException(status_code=503, detail="Policy model not initialized")
    start_t = time.time()
    warnings: List[str] = [
        "SHADOW ONLY: model output is not executable",
        "Action semantics remain unknown_songling_convention and are not proven to be relative PiPER deltas",
        "VLAC gripper input units are unverified; format_state receives a zero placeholder",
        "Synchronous policy inference cannot currently be interrupted safely once dispatched",
    ]
    images = [_normalize_image_input(image) for image in req.images]
    legacy_units = _si_to_legacy_model_units(req.end_effector_state)
    formatted_state = POLICY.format_state(legacy_units, gripper_format=False)
    query = POLICY.get_action_prompt(
        task=req.task_description,
        view_num=len(images),
        position_output=False,
        simple=False,
        state=formatted_state,
        think=False,
    )
    try:
        with INFER_LOCK:
            infer_requests = POLICY.get_infer_requests(prompt=query, images=images)
            response_list, model_infer_time = POLICY.chat(infer_requests)
            answers_list, _ = POLICY.results_format(response_list, infer_requests, rich=True)
    except Exception as exc:
        raise HTTPException(status_code=500, detail=f"Policy inference failed: {exc}")
    raw_model_output = answers_list[0] if answers_list else ""
    parsed, parse_reliability, parse_warnings = _parse_raw_actions(raw_model_output)
    warnings.extend(parse_warnings)
    return {
        "success": parse_reliability != "failed",
        "model": MODEL_NAME,
        "mode": "shadow_only",
        "execution_allowed": False,
        "task_description": req.task_description,
        "input_state_si": req.end_effector_state.model_dump(),
        "input_state_model_units": legacy_units,
        "input_state_model_units_note": "Gripper input uses an explicit zero placeholder because VLAC gripper units are unverified",
        "formatted_state_prompt_units": formatted_state,
        "formatted_state_prompt_units_note": "Values were passed to POLICY.format_state(..., gripper_format=False)",
        "prompt": query,
        "raw_model_output": raw_model_output,
        "all_raw_model_outputs": answers_list,
        "parsed_actions": [item.model_dump() for item in parsed],
        "parse_reliability": parse_reliability,
        "model_confidence": None,
        "action_semantics": ACTION_SEMANTICS,
        "warnings": warnings,
        "latency_ms": (time.time() - start_t) * 1000.0,
        "model_infer_time_s": model_infer_time,
    }


if __name__ == "__main__":
    port = int(os.getenv("PORT", "8016"))
    uvicorn.run("policy_main:app", host="0.0.0.0", port=port, reload=False)
