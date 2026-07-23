from __future__ import annotations

import base64
import io
import json
import math
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

import torch
import uvicorn
from fastapi import FastAPI, HTTPException, Request
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse
from huggingface_hub import HfApi, hf_hub_download
from PIL import Image
from pydantic import BaseModel, ConfigDict, Field, model_validator

SERVICE_NAME = "smolvla_shadow"
SERVICE_VERSION = "0.1.0"
MODEL_ID = os.getenv("SMOLVLA_MODEL_ID", "lerobot/smolvla_base")
MODEL_REVISION = os.getenv("SMOLVLA_MODEL_REVISION", "c83c3163b8ca9b7e67c509fffd9121e66cb96205")
LEROBOT_COMMIT = os.getenv("SMOLVLA_LEROBOT_COMMIT", "9c82c39c7b541e9c5bd8340abb7c9d8803c98744")
HOST = os.getenv("SMOLVLA_HOST", "0.0.0.0")
PORT = int(os.getenv("SMOLVLA_PORT", "8018"))
DEVICE = os.getenv("SMOLVLA_DEVICE", "cuda:0" if torch.cuda.is_available() else "cpu")
SELECTED_PHYSICAL_GPU = os.getenv("SMOLVLA_SELECTED_PHYSICAL_GPU", "unknown")
LOAD_MODEL = os.getenv("SMOLVLA_LOAD_MODEL", "1") == "1"
MAX_IMAGE_BYTES = int(os.getenv("SMOLVLA_MAX_IMAGE_BYTES", "8000000"))
MAX_REQUEST_BYTES = int(os.getenv("SMOLVLA_MAX_REQUEST_BYTES", "16000000"))

PIPER_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"]
PIPER_ACTION_NAMES = [
    "target_joint1",
    "target_joint2",
    "target_joint3",
    "target_joint4",
    "target_joint5",
    "target_joint6",
    "target_gripper",
]


class ImageSpec(BaseModel):
    type: str = "RGB image"
    source: str
    required: bool = True


class ImagesSpec(BaseModel):
    external_camera: ImageSpec
    wrist_camera: Optional[ImageSpec] = None


class ObservationSpec(BaseModel):
    images: ImagesSpec
    state_names: List[str] = Field(..., min_length=1)
    state_dimension: int
    joint_units: str
    gripper_units: str
    state_order: str


class TaskSpec(BaseModel):
    type: str = "natural-language instruction"


class ActionSpec(BaseModel):
    names: List[str] = Field(..., min_length=1)
    dimension: int
    semantics: str
    joint_units: str
    gripper_units: str
    execution_allowed: bool = False


class ProposedSchema(BaseModel):
    observation: ObservationSpec
    task: TaskSpec
    action: ActionSpec


class CompatibilityCheckRequest(BaseModel):
    model_config = ConfigDict(populate_by_name=True)
    feature_schema: ProposedSchema = Field(alias="schema")


class PiPERState(BaseModel):
    joint1: float
    joint2: float
    joint3: float
    joint4: float
    joint5: float
    joint6: float
    gripper: float

    @model_validator(mode="after")
    def validate_finite(self):
        for key, value in self.model_dump().items():
            if not math.isfinite(float(value)):
                raise ValueError(f"non-finite state field: {key}")
        return self

    def as_list(self) -> List[float]:
        return [
            float(self.joint1),
            float(self.joint2),
            float(self.joint3),
            float(self.joint4),
            float(self.joint5),
            float(self.joint6),
            float(self.gripper),
        ]


class ActionPreviewRequest(BaseModel):
    images: List[str] = Field(..., min_length=1, max_length=2)
    joint_state: PiPERState
    task_description: str
    timestamps: Dict[str, float] = Field(default_factory=dict)
    source_metadata: Dict[str, Any] = Field(default_factory=dict)

    @model_validator(mode="after")
    def validate_instruction(self):
        if not self.task_description.strip():
            raise ValueError("task_description must be non-empty")
        return self


@dataclass
class ModelMetadata:
    config: Dict[str, Any]
    preprocessor: Dict[str, Any]
    postprocessor: Dict[str, Any]
    hub_info: Dict[str, Any]
    expected_camera_keys: List[str]
    expected_state_dimension: int
    expected_action_dimension: int
    action_chunk_length: int
    normalization_source: Dict[str, Any]


@dataclass
class RuntimeState:
    metadata: Optional[ModelMetadata] = None
    model: Any = None
    model_loaded: bool = False
    load_error: Optional[str] = None
    parameter_count: Optional[int] = None
    selected_physical_gpu: str = SELECTED_PHYSICAL_GPU
    startup_latency_s: float = 0.0


def piper_proposed_schema() -> ProposedSchema:
    return ProposedSchema(
        observation=ObservationSpec(
            images=ImagesSpec(
                external_camera=ImageSpec(source="RealSense", required=True),
                wrist_camera=ImageSpec(source="PiPER wrist camera", required=False),
            ),
            state_names=PIPER_JOINT_NAMES,
            state_dimension=7,
            joint_units="radians",
            gripper_units="unverified",
            state_order="exactly as listed",
        ),
        task=TaskSpec(),
        action=ActionSpec(
            names=PIPER_ACTION_NAMES,
            dimension=7,
            semantics="proposed absolute joint targets",
            joint_units="radians",
            gripper_units="unverified",
            execution_allowed=False,
        ),
    )


def piper_dataset_schema() -> Dict[str, Any]:
    return {
        "observation.images.camera1": {
            "source": "external RealSense RGB",
            "required": True,
            "notes": "Primary tabletop view.",
        },
        "observation.images.camera2": {
            "source": "wrist RGB camera",
            "required": False,
            "notes": "Recommended for final embodiment even if absent in the first tabletop audit.",
        },
        "observation.state": {
            "shape": [7],
            "names": PIPER_JOINT_NAMES,
            "joint_units": "radians",
            "gripper_units": "record raw units and document them explicitly",
        },
        "action": {
            "shape": [7],
            "names": PIPER_ACTION_NAMES,
            "semantics": "embodiment-specific commanded joint targets or another explicitly documented PiPER action representation",
            "joint_units": "radians",
            "gripper_units": "unverified until PiPER command units are audited",
        },
        "task": {
            "type": "natural-language instruction",
            "key": "task",
        },
        "required_episode_metadata": [
            "timestamp",
            "episode_index",
            "frame_index",
            "success",
            "failure_reason",
            "camera_source_metadata",
        ],
    }


def _read_hub_json(filename: str) -> Dict[str, Any]:
    path = hf_hub_download(repo_id=MODEL_ID, revision=MODEL_REVISION, filename=filename)
    return json.loads(Path(path).read_text(encoding="utf-8"))


def baked_metadata() -> ModelMetadata:
    config = {
        "type": "smolvla",
        "input_features": {
            "observation.state": {"type": "STATE", "shape": [6]},
            "observation.images.camera1": {"type": "VISUAL", "shape": [3, 256, 256]},
            "observation.images.camera2": {"type": "VISUAL", "shape": [3, 256, 256]},
            "observation.images.camera3": {"type": "VISUAL", "shape": [3, 256, 256]},
        },
        "output_features": {"action": {"type": "ACTION", "shape": [6]}},
        "chunk_size": 50,
        "n_action_steps": 50,
        "normalization_mapping": {"VISUAL": "IDENTITY", "STATE": "MEAN_STD", "ACTION": "MEAN_STD"},
        "max_state_dim": 32,
        "max_action_dim": 32,
        "empty_cameras": 0,
        "vlm_model_name": "HuggingFaceTB/SmolVLM2-500M-Video-Instruct",
        "tokenizer_max_length": 48,
    }
    preprocessor = {
        "name": "policy_preprocessor",
        "steps": [
            {"registry_name": "rename_observations_processor", "config": {"rename_map": {}}},
            {"registry_name": "to_batch_processor", "config": {}},
            {"registry_name": "smolvla_new_line_processor", "config": {}},
            {
                "registry_name": "tokenizer_processor",
                "config": {
                    "max_length": 48,
                    "task_key": "task",
                    "padding_side": "right",
                    "padding": "max_length",
                    "truncation": True,
                    "tokenizer_name": "HuggingFaceTB/SmolVLM2-500M-Video-Instruct",
                },
            },
            {"registry_name": "device_processor", "config": {"device": "cuda", "float_dtype": None}},
            {
                "registry_name": "normalizer_processor",
                "config": {
                    "eps": 1e-08,
                    "features": {
                        "observation.state": {"type": "STATE", "shape": [6]},
                        "observation.image": {"type": "VISUAL", "shape": [3, 256, 256]},
                        "observation.image2": {"type": "VISUAL", "shape": [3, 256, 256]},
                        "observation.image3": {"type": "VISUAL", "shape": [3, 256, 256]},
                        "action": {"type": "ACTION", "shape": [6]},
                    },
                    "norm_map": {"VISUAL": "IDENTITY", "STATE": "MEAN_STD", "ACTION": "MEAN_STD"},
                },
                "state_file": "policy_preprocessor_step_5_normalizer_processor.safetensors",
            },
        ],
    }
    postprocessor = {
        "name": "policy_postprocessor",
        "steps": [
            {
                "registry_name": "unnormalizer_processor",
                "config": {
                    "eps": 1e-08,
                    "features": {"action": {"type": "ACTION", "shape": [6]}},
                    "norm_map": {"VISUAL": "IDENTITY", "STATE": "MEAN_STD", "ACTION": "MEAN_STD"},
                },
                "state_file": "policy_postprocessor_step_0_unnormalizer_processor.safetensors",
            },
            {"registry_name": "device_processor", "config": {"device": "cpu", "float_dtype": None}},
        ],
    }
    return ModelMetadata(
        config=config,
        preprocessor=preprocessor,
        postprocessor=postprocessor,
        hub_info={"sha": MODEL_REVISION, "metadata_source": "baked_fallback"},
        expected_camera_keys=["observation.images.camera1", "observation.images.camera2", "observation.images.camera3"],
        expected_state_dimension=6,
        expected_action_dimension=6,
        action_chunk_length=50,
        normalization_source={
            "preprocessor_state_file": "policy_preprocessor_step_5_normalizer_processor.safetensors",
            "postprocessor_state_file": "policy_postprocessor_step_0_unnormalizer_processor.safetensors",
            "mapping": {"VISUAL": "IDENTITY", "STATE": "MEAN_STD", "ACTION": "MEAN_STD"},
        },
    )


def load_metadata() -> ModelMetadata:
    try:
        config = _read_hub_json("config.json")
        preprocessor = _read_hub_json("policy_preprocessor.json")
        postprocessor = _read_hub_json("policy_postprocessor.json")
        hub_info_obj = HfApi().model_info(MODEL_ID, revision=MODEL_REVISION)
        if hasattr(hub_info_obj, "model_dump"):
            hub_info = hub_info_obj.model_dump()
        else:
            hub_info = dict(getattr(hub_info_obj, "__dict__", {}))
        expected_camera_keys = [key for key in config.get("input_features", {}) if key.startswith("observation.images.")]
        state_shape = config.get("input_features", {}).get("observation.state", {}).get("shape", [])
        action_shape = config.get("output_features", {}).get("action", {}).get("shape", [])
        normalization_source = {
            "preprocessor_state_file": preprocessor.get("steps", [{}])[-1].get("state_file"),
            "postprocessor_state_file": postprocessor.get("steps", [{}])[0].get("state_file"),
            "mapping": config.get("normalization_mapping", {}),
        }
        return ModelMetadata(
            config=config,
            preprocessor=preprocessor,
            postprocessor=postprocessor,
            hub_info=hub_info,
            expected_camera_keys=expected_camera_keys,
            expected_state_dimension=int(state_shape[0]) if state_shape else 0,
            expected_action_dimension=int(action_shape[0]) if action_shape else 0,
            action_chunk_length=int(config.get("chunk_size", 0)),
            normalization_source=normalization_source,
        )
    except Exception:
        return baked_metadata()


def _gpu_info() -> Dict[str, Any]:
    info: Dict[str, Any] = {
        "device": DEVICE,
        "selected_physical_gpu": SELECTED_PHYSICAL_GPU,
        "cuda_available": torch.cuda.is_available(),
    }
    if torch.cuda.is_available():
        try:
            idx = int(DEVICE.split(":", 1)[1]) if DEVICE.startswith("cuda:") else torch.cuda.current_device()
            info.update(
                {
                    "container_gpu_index": idx,
                    "gpu_name": torch.cuda.get_device_name(idx),
                    "memory_allocated_mb": round(torch.cuda.memory_allocated(idx) / 1024 / 1024, 1),
                    "memory_reserved_mb": round(torch.cuda.memory_reserved(idx) / 1024 / 1024, 1),
                }
            )
        except Exception as exc:
            info["gpu_error"] = str(exc)
    return info


def _decode_image(image_input: str) -> Dict[str, Any]:
    if image_input.startswith(("http://", "https://")):
        raise HTTPException(status_code=400, detail="remote image URLs are disabled")
    payload = image_input.split(",", 1)[1] if image_input.startswith("data:image") and "," in image_input else image_input
    try:
        decoded = base64.b64decode(payload, validate=True)
    except Exception as exc:
        raise HTTPException(status_code=400, detail=f"invalid image payload: {exc}") from exc
    if len(decoded) > MAX_IMAGE_BYTES:
        raise HTTPException(status_code=413, detail="image exceeds size limit")
    try:
        image = Image.open(io.BytesIO(decoded)).convert("RGB")
    except Exception as exc:
        raise HTTPException(status_code=400, detail=f"invalid image bytes: {exc}") from exc
    return {"width": image.width, "height": image.height, "mode": image.mode, "bytes": len(decoded)}


def build_compatibility_report(proposed: ProposedSchema, metadata: ModelMetadata) -> Dict[str, Any]:
    blockers: List[str] = []
    warnings: List[str] = []
    assumptions: List[str] = []

    proposed_camera_count = 1 + int(proposed.observation.images.wrist_camera is not None)
    checkpoint_camera_count = len(metadata.expected_camera_keys)
    camera_ok = proposed_camera_count == checkpoint_camera_count
    if not camera_ok:
        blockers.append(
            f"Checkpoint expects {checkpoint_camera_count} camera keys {metadata.expected_camera_keys}, but PiPER proposes {proposed_camera_count}."
        )

    state_ok = proposed.observation.state_dimension == metadata.expected_state_dimension
    if not state_ok:
        blockers.append(
            f"Checkpoint observation.state dimension is {metadata.expected_state_dimension}, but PiPER proposes {proposed.observation.state_dimension}."
        )

    action_ok = proposed.action.dimension == metadata.expected_action_dimension
    if not action_ok:
        blockers.append(
            f"Checkpoint action dimension is {metadata.expected_action_dimension}, but PiPER proposes {proposed.action.dimension}."
        )

    if metadata.config.get("empty_cameras", 0) == 0 and proposed_camera_count < checkpoint_camera_count:
        blockers.append("Checkpoint config declares empty_cameras=0, so missing pretrained camera inputs cannot be represented honestly.")

    normalization_ok = camera_ok and state_ok and action_ok
    if not normalization_ok:
        blockers.append("Bundled normalization statistics are checkpoint-specific and do not cover PiPER's proposed camera/state/action schema.")

    warnings.append("smolvla_base outputs embodiment-dependent continuous action chunks, not validated PiPER joint targets.")
    warnings.append("PiPER gripper units remain unverified and must be documented during data collection.")
    assumptions.append("Language input field `task` is compatible with PiPER task text.")

    compatible = not blockers
    return {
        "compatible": compatible,
        "observation_compatibility": compatible and camera_ok and state_ok,
        "camera_compatibility": camera_ok,
        "state_dimension_compatibility": state_ok,
        "action_dimension_compatibility": action_ok,
        "language_compatibility": True,
        "normalization_compatibility": normalization_ok,
        "missing_statistics": [] if normalization_ok else [
            "PiPER-specific observation.state normalization statistics",
            "PiPER-specific action normalization statistics",
            "PiPER camera feature normalization metadata",
        ],
        "missing_metadata": [] if compatible else [
            "PiPER embodiment dataset metadata",
            "PiPER action semantics metadata",
        ],
        "blockers": blockers,
        "warnings": warnings,
        "assumptions": assumptions,
        "fine_tuning_required": not compatible,
        "exact_recommended_dataset_schema": piper_dataset_schema(),
        "checkpoint_contract": {
            "input_features": metadata.config.get("input_features", {}),
            "output_features": metadata.config.get("output_features", {}),
            "expected_camera_keys": metadata.expected_camera_keys,
            "chunk_size": metadata.action_chunk_length,
            "normalization_source": metadata.normalization_source,
        },
        "execution_allowed": False,
    }


def _make_error(status_code: int, message: str) -> JSONResponse:
    return JSONResponse(
        status_code=status_code,
        content={"request_success": False, "error": message, "execution_allowed": False},
    )


def create_runtime(load_model: bool = LOAD_MODEL) -> RuntimeState:
    runtime = RuntimeState()
    start = time.monotonic()
    try:
        runtime.metadata = load_metadata()
        if load_model:
            from lerobot.policies.smolvla import SmolVLAPolicy

            if torch.cuda.is_available() and DEVICE.startswith("cuda:"):
                torch.cuda.set_device(int(DEVICE.split(":", 1)[1]))
            runtime.model = SmolVLAPolicy.from_pretrained(MODEL_ID, revision=MODEL_REVISION)
            runtime.model_loaded = True
            runtime.parameter_count = int(sum(parameter.numel() for parameter in runtime.model.parameters()))
    except Exception as exc:
        runtime.load_error = str(exc)
    runtime.startup_latency_s = round(time.monotonic() - start, 3)
    return runtime


def create_app(runtime: Optional[RuntimeState] = None) -> FastAPI:
    runtime = runtime or create_runtime()
    app = FastAPI(title="ABot-Claw SmolVLA Shadow Service", version=SERVICE_VERSION)

    @app.middleware("http")
    async def guard_request_size(request: Request, call_next):
        length = request.headers.get("content-length")
        if length and int(length) > MAX_REQUEST_BYTES:
            return _make_error(413, "request too large")
        return await call_next(request)

    @app.exception_handler(HTTPException)
    async def http_exc_handler(request: Request, exc: HTTPException):
        detail = exc.detail if isinstance(exc.detail, str) else "request rejected"
        return _make_error(exc.status_code, detail)

    @app.exception_handler(RequestValidationError)
    async def validation_exc_handler(request: Request, exc: RequestValidationError):
        return _make_error(422, "request validation failed")

    @app.exception_handler(Exception)
    async def generic_exc_handler(request: Request, exc: Exception):
        return _make_error(500, "internal service error")

    @app.get("/health")
    def health() -> Dict[str, Any]:
        return {
            "service": SERVICE_NAME,
            "status": "ok",
            "model_id": MODEL_ID,
            "model_revision": MODEL_REVISION,
            "lerobot_commit": LEROBOT_COMMIT,
            "python_version": os.sys.version.split()[0],
            "pytorch_version": torch.__version__,
            "cuda_version": torch.version.cuda,
            "selected_gpu": runtime.selected_physical_gpu,
            "gpu": _gpu_info(),
            "model_loaded": runtime.model_loaded,
            "model_load_error": runtime.load_error,
            "parameter_count": runtime.parameter_count,
            "startup_latency_s": runtime.startup_latency_s,
            "shadow_only": True,
            "execution_allowed": False,
        }

    @app.get("/model-info")
    def model_info() -> Dict[str, Any]:
        metadata = runtime.metadata
        if metadata is None:
            raise HTTPException(status_code=503, detail="metadata unavailable")
        return {
            "model_id": MODEL_ID,
            "model_revision": MODEL_REVISION,
            "architecture": metadata.config.get("type"),
            "parameter_count": runtime.parameter_count,
            "checkpoint_revision": metadata.hub_info.get("sha", MODEL_REVISION),
            "expected_observation_features": metadata.config.get("input_features", {}),
            "expected_camera_keys": metadata.expected_camera_keys,
            "expected_camera_count": len(metadata.expected_camera_keys),
            "expected_state_dimension": metadata.expected_state_dimension,
            "expected_action_dimension": metadata.expected_action_dimension,
            "action_chunk_length": metadata.action_chunk_length,
            "normalization_requirements": metadata.normalization_source,
            "action_semantics": "embodiment-dependent continuous action chunk; not validated PiPER joint targets",
            "task_language_requirements": {
                "field": "task",
                "tokenizer": metadata.config.get("vlm_model_name"),
                "max_length": metadata.config.get("tokenizer_max_length"),
            },
            "preprocessing_requirements": metadata.preprocessor,
            "postprocessing_requirements": metadata.postprocessor,
            "model_confidence_available": False,
            "shadow_only": True,
            "execution_allowed": False,
        }

    @app.post("/compatibility-check")
    def compatibility_check(request: CompatibilityCheckRequest) -> Dict[str, Any]:
        metadata = runtime.metadata
        if metadata is None:
            raise HTTPException(status_code=503, detail="metadata unavailable")
        report = build_compatibility_report(request.feature_schema, metadata)
        report.update(
            {
                "request_success": True,
                "compatibility_verified": True,
                "model_loaded": runtime.model_loaded,
                "execution_allowed": False,
            }
        )
        return report

    @app.post("/action-preview")
    def action_preview(request: ActionPreviewRequest) -> Dict[str, Any]:
        metadata = runtime.metadata
        if metadata is None:
            raise HTTPException(status_code=503, detail="metadata unavailable")
        image_metadata = [_decode_image(item) for item in request.images]
        compatibility = build_compatibility_report(piper_proposed_schema(), metadata)
        return {
            "request_success": True,
            "model_loaded": runtime.model_loaded,
            "compatibility_verified": True,
            "inference_attempted": False,
            "inference_success": False,
            "output_shape_valid": None,
            "model_confidence": None,
            "behavioral_success": None,
            "compatible": compatibility["compatible"],
            "blockers": compatibility["blockers"],
            "warnings": compatibility["warnings"],
            "assumptions": compatibility["assumptions"],
            "action_semantics": "unknown for PiPER; checkpoint semantics remain embodiment-dependent",
            "normalization_source": metadata.normalization_source,
            "raw_model_output": None,
            "raw_action_tensor": None,
            "processed_action_tensor": None,
            "action_shape": None,
            "action_chunk_length": metadata.action_chunk_length,
            "input_feature_names": list(metadata.config.get("input_features", {}).keys()),
            "input_state": request.joint_state.model_dump(),
            "image_metadata": image_metadata,
            "task_description": request.task_description.strip(),
            "latency_ms": 0.0,
            "execution_allowed": False,
        }

    return app


app = create_app()


if __name__ == "__main__":
    uvicorn.run("app:app", host=HOST, port=PORT)
