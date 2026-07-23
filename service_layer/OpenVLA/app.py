from __future__ import annotations

import base64
import hashlib
import io
import json
import math
import os
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

import numpy as np
import torch
import uvicorn
from fastapi import FastAPI, HTTPException, Request
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse
from PIL import Image
from pydantic import BaseModel, Field, ValidationError, field_validator, model_validator
from transformers import AutoModelForVision2Seq, AutoProcessor


SERVICE_VERSION = "0.1.0"
SERVICE_NAME = "openvla_shadow"
MODEL_ID = os.getenv("OPENVLA_MODEL_ID", "openvla/openvla-7b")
MODEL_REVISION = os.getenv("OPENVLA_MODEL_REVISION", "47a0ec7fc4ec123775a391911046cf33cf9ed83f")
SOURCE_REVISION = os.getenv("OPENVLA_SOURCE_REVISION", "c8f03f48af692657d3060c19588038c7220e9af9")
PORT = int(os.getenv("PORT", "8018"))
HF_HOME = os.getenv("HF_HOME", "")
DEVICE = os.getenv("OPENVLA_DEVICE", "cuda:0" if torch.cuda.is_available() else "cpu")
TORCH_DTYPE = torch.bfloat16 if torch.cuda.is_available() else torch.float32
UNNORM_KEY = os.getenv("OPENVLA_UNNORM_KEY", "bridge_orig")
MAX_REQUEST_BYTES = int(os.getenv("OPENVLA_MAX_REQUEST_BYTES", "12000000"))
MAX_IMAGE_BYTES = int(os.getenv("OPENVLA_MAX_IMAGE_BYTES", "8000000"))
MAX_IMAGE_PIXELS = int(os.getenv("OPENVLA_MAX_IMAGE_PIXELS", str(4096 * 4096)))
IMAGE_MEANS = [[0.485, 0.456, 0.406], [0.5, 0.5, 0.5]]
IMAGE_STDS = [[0.229, 0.224, 0.225], [0.5, 0.5, 0.5]]
SUPPORTED_IMAGE_MIMES = {"image/jpeg", "image/png"}
MOTION_MODULE_TOKENS = ("moveit", "rospy", "piper_sdk", "socketcan", "candump", "8891")
INFERENCE_LOCK = threading.Lock()


def _gpu_index(device: str) -> Optional[int]:
    if not torch.cuda.is_available():
        return None
    if device.startswith("cuda:"):
        return int(device.split(":", 1)[1])
    return torch.cuda.current_device()


def _gpu_metrics(device: str) -> Dict[str, Any]:
    result: Dict[str, Any] = {"device": device, "cuda_available": torch.cuda.is_available()}
    idx = _gpu_index(device)
    if idx is None:
        return result
    result.update(
        {
            "gpu_index": idx,
            "gpu_name": torch.cuda.get_device_name(idx),
            "memory_allocated_mb": round(torch.cuda.memory_allocated(idx) / 1024 / 1024, 1),
            "memory_reserved_mb": round(torch.cuda.memory_reserved(idx) / 1024 / 1024, 1),
        }
    )
    return result


def _openvla_prompt(instruction: str, model_id: str) -> str:
    normalized = instruction.strip().lower()
    if "v01" in model_id:
        return (
            "A chat between a curious user and an artificial intelligence assistant. "
            "The assistant gives helpful, detailed, and polite answers to the user's questions. "
            f"USER: What action should the robot take to {normalized}? ASSISTANT:"
        )
    return f"In: What action should the robot take to {normalized}?\nOut:"


def _sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _parse_data_image(image_data: str) -> tuple[bytes, str]:
    if image_data.startswith(("http://", "https://")):
        raise HTTPException(status_code=400, detail=_error_payload("remote image URLs are disabled", 400))
    if image_data.startswith("data:"):
        header, encoded = image_data.split(",", 1)
        mime = header.split(":", 1)[1].split(";", 1)[0]
    else:
        mime = "image/jpeg"
        encoded = image_data
    if mime not in SUPPORTED_IMAGE_MIMES:
        raise HTTPException(status_code=400, detail=_error_payload(f"unsupported image mime: {mime}", 400))
    try:
        decoded = base64.b64decode(encoded, validate=True)
    except Exception:
        raise HTTPException(status_code=400, detail=_error_payload("malformed base64 image", 400))
    if not decoded:
        raise HTTPException(status_code=400, detail=_error_payload("empty image payload", 400))
    if len(decoded) > MAX_IMAGE_BYTES:
        raise HTTPException(status_code=413, detail=_error_payload("image exceeds size limit", 413))
    return decoded, mime


def _decode_image(image_data: str) -> tuple[Image.Image, Dict[str, Any]]:
    decoded, mime = _parse_data_image(image_data)
    try:
        image = Image.open(io.BytesIO(decoded)).convert("RGB")
    except Exception:
        raise HTTPException(status_code=400, detail=_error_payload("image decode failed", 400))
    if image.width * image.height > MAX_IMAGE_PIXELS:
        raise HTTPException(status_code=413, detail=_error_payload("image exceeds pixel limit", 413))
    if min(image.width, image.height) < 8:
        raise HTTPException(status_code=400, detail=_error_payload("image is too small", 400))
    return image, {
        "mime": mime,
        "sha256": _sha256_bytes(decoded),
        "width": image.width,
        "height": image.height,
        "mode": image.mode,
        "byte_count": len(decoded),
    }


def _finite_number(value: float) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("non-finite value")
    return value


def _error_payload(message: str, status_code: int) -> Dict[str, Any]:
    return {
        "request_success": False,
        "service": SERVICE_NAME,
        "status_code": status_code,
        "error": message,
        "execution_allowed": False,
    }


class PiperStateMetadata(BaseModel):
    joint_names: Optional[List[str]] = None
    joint_positions_rad: Optional[List[float]] = None
    gripper_raw: Optional[float] = None
    timestamp_s: Optional[float] = None

    @field_validator("joint_positions_rad")
    @classmethod
    def _validate_joints(cls, value: Optional[List[float]]) -> Optional[List[float]]:
        if value is None:
            return value
        for item in value:
            _finite_number(item)
        return value


class EndPoseMetadata(BaseModel):
    position_m: Optional[List[float]] = None
    quaternion_xyzw: Optional[List[float]] = None
    timestamp_s: Optional[float] = None

    @field_validator("position_m", "quaternion_xyzw")
    @classmethod
    def _validate_vector(cls, value: Optional[List[float]]) -> Optional[List[float]]:
        if value is None:
            return value
        for item in value:
            _finite_number(item)
        return value


class ActionPreviewRequest(BaseModel):
    image: str
    instruction: str
    source_timestamp_s: Optional[float] = None
    camera_metadata: Dict[str, Any] = Field(default_factory=dict)
    piper_state_metadata: Optional[PiperStateMetadata] = None
    end_pose_metadata: Optional[EndPoseMetadata] = None
    execution_allowed: bool = False

    @model_validator(mode="after")
    def _validate(self) -> "ActionPreviewRequest":
        if not self.instruction.strip():
            raise ValueError("instruction must be non-empty")
        if self.source_timestamp_s is not None:
            _finite_number(self.source_timestamp_s)
        return self


class CompatibilityRequest(BaseModel):
    camera_count: int
    camera_source: str
    language_instruction: bool
    available_state: List[str]
    desired_action_dimension: int
    desired_future_executor: str
    execution_allowed: bool = False


@dataclass
class OpenVLAModelInfo:
    processor: Any
    model: Any
    action_stats: Dict[str, Any]
    action_dim: int


class OpenVLAShadowRuntime:
    def __init__(self) -> None:
        self.model_info: Optional[OpenVLAModelInfo] = None

    def load(self) -> None:
        if self.model_info is not None:
            return
        processor = AutoProcessor.from_pretrained(
            MODEL_ID,
            revision=MODEL_REVISION,
            trust_remote_code=True,
        )
        model = AutoModelForVision2Seq.from_pretrained(
            MODEL_ID,
            revision=MODEL_REVISION,
            torch_dtype=TORCH_DTYPE,
            low_cpu_mem_usage=True,
            trust_remote_code=True,
        ).to(DEVICE)
        action_stats = model.get_action_stats(UNNORM_KEY)
        action_dim = model.get_action_dim(UNNORM_KEY)
        self.model_info = OpenVLAModelInfo(
            processor=processor,
            model=model,
            action_stats=action_stats,
            action_dim=action_dim,
        )

    @property
    def loaded(self) -> bool:
        return self.model_info is not None

    def _require_loaded(self) -> OpenVLAModelInfo:
        if self.model_info is None:
            raise RuntimeError("model not loaded")
        return self.model_info

    def model_contract(self) -> Dict[str, Any]:
        info = self._require_loaded()
        return {
            "model_id": MODEL_ID,
            "model_revision": MODEL_REVISION,
            "source_revision": SOURCE_REVISION,
            "expected_image_count": 1,
            "image_preprocessing": {
                "input_image_count": 1,
                "input_color_mode": "RGB",
                "image_resize_strategy": "resize-naive",
                "processor_image_size": [224, 224],
                "fused_backbone_branches": 2,
                "branch_input_sizes": [[3, 224, 224], [3, 224, 224]],
                "image_mean": IMAGE_MEANS,
                "image_std": IMAGE_STDS,
                "letterbox": False,
            },
            "language_prompt_format": _openvla_prompt("<instruction>", MODEL_ID),
            "robot_state_input": {
                "accepted_by_model": False,
                "used_for_inference": False,
                "metadata_only": True,
            },
            "action_dimension": info.action_dim,
            "action_channels": ["x", "y", "z", "roll", "pitch", "yaw", "gripper"],
            "action_semantics": "openvla_bridge_7d_end_effector_action",
            "relative_vs_absolute": "relative_end_effector_delta_from_official_model_card",
            "rotation_convention": "unresolved_in_official_source",
            "gripper_convention": "unresolved_for_piper; bridge_orig channel is left normalized because mask[6] is false",
            "normalization_keys": [UNNORM_KEY],
            "selected_unnorm_key": UNNORM_KEY,
            "normalization_statistics_provenance": "embedded in checkpoint config.json norm_stats",
            "bridge_orig_statistics": info.action_stats,
            "coordinate_frame": "bridge_dataset_specific_unverified_for_piper",
            "model_confidence_available": False,
            "execution_allowed": False,
        }

    def compatibility_check(self, request: CompatibilityRequest) -> Dict[str, Any]:
        info = self._require_loaded()
        input_ok = request.camera_count == 1 and request.language_instruction
        output_dim_ok = request.desired_action_dimension == info.action_dim
        shadow_allowed = input_ok and output_dim_ok and UNNORM_KEY == "bridge_orig"
        blockers: List[str] = []
        warnings: List[str] = []
        assumptions: List[str] = []
        if request.camera_count != 1:
            blockers.append("Official OpenVLA-7B path accepts exactly one RGB image.")
        if not request.language_instruction:
            blockers.append("Official OpenVLA inference requires a natural-language instruction.")
        if request.desired_action_dimension != info.action_dim:
            blockers.append(f"Desired action dimension {request.desired_action_dimension} does not match checkpoint action dimension {info.action_dim}.")
        warnings.extend(
            [
                "BridgeData normalization statistics are present, but they are not PiPER-specific.",
                "The official output is a 7D Bridge-style end-effector action. PiPER frame semantics remain unverified.",
                "PiPER state is not consumed by the official checkpoint and remains metadata-only in this service.",
            ]
        )
        assumptions.append("Shadow inference is allowed because the audited schema matches the official one-image, one-instruction, 7D action path.")
        physical_blockers = [
            "Coordinate frame is not verified for PiPER.",
            "Rotation convention is not verified for PiPER.",
            "Gripper semantics are not verified for PiPER.",
            "bridge_orig normalization statistics come from BridgeData, not PiPER demonstrations.",
            f"Desired future executor `{request.desired_future_executor}` is not validated against OpenVLA Bridge action semantics.",
        ]
        return {
            "request_success": True,
            "input_schema_compatible": input_ok,
            "output_dimension_compatible": output_dim_ok,
            "output_semantics_compatible": False,
            "normalization_available": UNNORM_KEY == "bridge_orig",
            "normalization_piper_specific": False,
            "coordinate_frame_verified_for_piper": False,
            "gripper_verified_for_piper": False,
            "shadow_inference_allowed": shadow_allowed,
            "physical_execution_compatible": False,
            "blockers": blockers,
            "warnings": warnings,
            "assumptions": assumptions,
            "physical_execution_blockers": physical_blockers,
            "execution_allowed": False,
        }

    def preview(self, request: ActionPreviewRequest) -> Dict[str, Any]:
        info = self._require_loaded()
        image, image_metadata = _decode_image(request.image)
        prompt = _openvla_prompt(request.instruction, MODEL_ID)
        started = time.time()
        with INFERENCE_LOCK, torch.inference_mode():
            inputs = info.processor(prompt, image).to(DEVICE, dtype=TORCH_DTYPE)
            input_ids = inputs["input_ids"]
            if not torch.all(input_ids[:, -1] == 29871):
                suffix = torch.tensor([[29871]], dtype=torch.long, device=input_ids.device)
                input_ids = torch.cat((input_ids, suffix), dim=1)
                inputs["input_ids"] = input_ids
                inputs["attention_mask"] = torch.cat(
                    (
                        inputs["attention_mask"],
                        torch.ones((inputs["attention_mask"].shape[0], 1), dtype=inputs["attention_mask"].dtype, device=inputs["attention_mask"].device),
                    ),
                    dim=1,
                )
            generated_ids = info.model.generate(
                **inputs,
                max_new_tokens=info.action_dim,
                do_sample=False,
            )
            predicted_action_token_ids = generated_ids[0, -info.action_dim :].detach().cpu().numpy()
            discretized_actions = info.model.vocab_size - predicted_action_token_ids
            discretized_actions = np.clip(discretized_actions - 1, a_min=0, a_max=info.model.bin_centers.shape[0] - 1)
            normalized_actions = info.model.bin_centers[discretized_actions]
            stats = info.action_stats
            mask = np.array(stats.get("mask", np.ones_like(stats["q01"], dtype=bool)), dtype=bool)
            action_high = np.array(stats["q99"], dtype=np.float64)
            action_low = np.array(stats["q01"], dtype=np.float64)
            unnormalized = np.where(
                mask,
                0.5 * (normalized_actions + 1.0) * (action_high - action_low) + action_low,
                normalized_actions,
            )
            generated_text = info.processor.batch_decode(generated_ids.detach().cpu(), skip_special_tokens=False)
        latency_ms = round((time.time() - started) * 1000.0, 1)
        return {
            "request_success": True,
            "compatibility_verified": True,
            "inference_attempted": True,
            "inference_success": True,
            "output_shape_valid": len(unnormalized.tolist()) == 7,
            "language_conditioning_tested": False,
            "behavioral_success": None,
            "model_confidence": None,
            "instruction": request.instruction,
            "prompt_passed_to_processor": prompt,
            "image_metadata": image_metadata,
            "processor_configuration": {
                "model_id": MODEL_ID,
                "model_revision": MODEL_REVISION,
                "source_revision": SOURCE_REVISION,
                "image_resize_strategy": "resize-naive",
                "processor_image_size": [224, 224],
                "fused_backbone_branches": 2,
            },
            "generated_token_ids": generated_ids[0].detach().cpu().tolist(),
            "predicted_action_token_ids": predicted_action_token_ids.tolist(),
            "raw_model_generation": generated_text,
            "raw_normalized_action": normalized_actions.tolist(),
            "bridge_unnormalized_action": unnormalized.tolist(),
            "action_channels": ["x", "y", "z", "roll", "pitch", "yaw", "gripper"],
            "action_semantics": "openvla_bridge_7d_end_effector_action",
            "relative_vs_absolute": "relative_end_effector_delta_from_official_model_card",
            "coordinate_frame": "unverified_for_piper",
            "rotation_convention": "unknown",
            "gripper_semantics": "unverified_for_piper",
            "normalization_key": UNNORM_KEY,
            "normalization_statistics_revision": MODEL_REVISION,
            "normalization_piper_specific": False,
            "piper_state_metadata": request.piper_state_metadata.model_dump(mode="json") if request.piper_state_metadata else None,
            "piper_state_model_input": False,
            "end_pose_metadata": request.end_pose_metadata.model_dump(mode="json") if request.end_pose_metadata else None,
            "latency_ms": latency_ms,
            "gpu": _gpu_metrics(DEVICE),
            "warnings": [
                "Output is shadow-only and not connected to any executor.",
                "BridgeData action frame is not verified for PiPER.",
                "Gripper semantics are not verified for PiPER.",
            ],
            "execution_allowed": False,
        }


def create_app(runtime: OpenVLAShadowRuntime, load_on_startup: bool = True) -> FastAPI:
    app = FastAPI(title="ABot-Claw OpenVLA Shadow Service", version=SERVICE_VERSION)
    app.state.runtime = runtime

    @app.exception_handler(HTTPException)
    async def _http_exception_handler(_: Request, exc: HTTPException) -> JSONResponse:
        detail = exc.detail if isinstance(exc.detail, dict) else _error_payload(str(exc.detail), exc.status_code)
        detail["execution_allowed"] = False
        return JSONResponse(status_code=exc.status_code, content=detail)

    @app.exception_handler(ValidationError)
    async def _validation_exception_handler(_: Request, exc: ValidationError) -> JSONResponse:
        return JSONResponse(status_code=422, content=_error_payload("validation error", 422))

    @app.exception_handler(RequestValidationError)
    async def _request_validation_exception_handler(_: Request, exc: RequestValidationError) -> JSONResponse:
        return JSONResponse(status_code=422, content=_error_payload("validation error", 422))

    @app.exception_handler(Exception)
    async def _unhandled_exception_handler(_: Request, exc: Exception) -> JSONResponse:
        return JSONResponse(status_code=500, content=_error_payload(f"internal error: {exc.__class__.__name__}", 500))

    @app.on_event("startup")
    def _startup() -> None:
        if load_on_startup:
            app.state.runtime.load()

    @app.middleware("http")
    async def _request_size_guard(request: Request, call_next):
        header = request.headers.get("content-length")
        if header is not None and int(header) > MAX_REQUEST_BYTES:
            return JSONResponse(status_code=413, content=_error_payload("request too large", 413))
        response = await call_next(request)
        return response

    @app.get("/health")
    def health() -> Dict[str, Any]:
        runtime_obj: OpenVLAShadowRuntime = app.state.runtime
        return {
            "request_success": True,
            "service": SERVICE_NAME,
            "version": SERVICE_VERSION,
            "model_id": MODEL_ID,
            "model_revision": MODEL_REVISION,
            "source_revision": SOURCE_REVISION,
            "model_loaded": runtime_obj.loaded,
            "selected_gpu": _gpu_metrics(DEVICE),
            "python_version": os.sys.version.split()[0],
            "torch_version": torch.__version__,
            "cuda_version": torch.version.cuda,
            "transformers_version": __import__("transformers").__version__,
            "allocated_vram_mb": _gpu_metrics(DEVICE).get("memory_allocated_mb"),
            "reserved_vram_mb": _gpu_metrics(DEVICE).get("memory_reserved_mb"),
            "hf_home": HF_HOME,
            "shadow_only": True,
            "execution_allowed": False,
        }

    @app.get("/model-info")
    def model_info() -> Dict[str, Any]:
        return app.state.runtime.model_contract()

    @app.post("/compatibility-check")
    def compatibility_check(request: CompatibilityRequest) -> Dict[str, Any]:
        return app.state.runtime.compatibility_check(request)

    @app.post("/action-preview")
    def action_preview(request: ActionPreviewRequest) -> Dict[str, Any]:
        return app.state.runtime.preview(request)

    return app


runtime = OpenVLAShadowRuntime()
app = create_app(runtime, load_on_startup=os.getenv("OPENVLA_SKIP_MODEL_LOAD", "0") != "1")


if __name__ == "__main__":
    uvicorn.run("app:app", host="0.0.0.0", port=PORT, reload=False)
