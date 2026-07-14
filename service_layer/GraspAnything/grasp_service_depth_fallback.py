"""License-free depth fallback for the GraspAnything HTTP API.

This backend preserves the GraspAnything routes and response shape while
generating simple top-down parallel-jaw grasp candidates from YOLO detections
and aligned depth. It does not send robot commands.
"""

from __future__ import annotations

import base64
import logging
import math
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import requests
import uvicorn
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel, Field


SERVICE_VERSION = "0.1.0-depth-fallback"
BACKEND_NAME = "depth_fallback"

YOLO_URL = os.getenv("YOLO_URL", "http://127.0.0.1:8013").rstrip("/")
MIN_DEPTH_M = float(os.getenv("MIN_DEPTH_M", "0.05"))
MAX_DEPTH_M = float(os.getenv("MAX_DEPTH_M", "1.50"))
GRIPPER_MIN_WIDTH_M = float(os.getenv("GRIPPER_MIN_WIDTH_M", "0.005"))
GRIPPER_MAX_WIDTH_M = float(os.getenv("GRIPPER_MAX_WIDTH_M", "0.060"))
FALLBACK_APPROACH_OFFSET_M = float(os.getenv("FALLBACK_APPROACH_OFFSET_M", "0.10"))
YOLO_CONF_THRES = float(os.getenv("YOLO_CONF_THRES", "0.25"))
YOLO_IOU_THRES = float(os.getenv("YOLO_IOU_THRES", "0.45"))
MIN_POINTS = int(os.getenv("FALLBACK_MIN_POINTS", "80"))

LOG_LEVEL = os.getenv("LOG_LEVEL", "INFO").upper()
logging.basicConfig(
    level=getattr(logging, LOG_LEVEL, logging.INFO),
    format="%(asctime)s %(levelname)s %(name)s: %(message)s",
)
logger = logging.getLogger("grasp_depth_fallback")


class GraspRequest(BaseModel):
    color_image: str = Field(..., description="RGB image input: base64/data-uri/path/url")
    depth_image: str = Field(..., description="Depth image input: base64/data-uri/path/url")
    camera_intrinsics: List[List[float]] = Field(..., description="3x3 camera intrinsics matrix K")
    object_name: str = Field(..., description="Target object class name")
    top_k: int = Field(5, ge=1, le=20, description="Max grasp candidates per detected instance")


class GraspResponse(BaseModel):
    frame_id: str
    target: str
    top_k: int
    count: int
    results: list[dict[str, Any]]
    latency_ms: float
    backend: str = BACKEND_NAME


@dataclass(frozen=True)
class Detection:
    label: str
    confidence: float
    xyxy: Tuple[int, int, int, int]


def _load_image_bytes(image_input: str) -> bytes:
    payload = image_input.strip()
    if not payload:
        raise ValueError("Empty image payload")
    if payload.startswith(("http://", "https://")):
        response = requests.get(payload, timeout=20)
        response.raise_for_status()
        return response.content
    if payload.startswith("data:image"):
        payload = payload.split(",", 1)[1]
    try:
        return base64.b64decode(payload, validate=True)
    except Exception:
        path = Path(payload)
        if path.exists():
            return path.read_bytes()
        raise ValueError("Invalid image payload: not valid base64 and path not found")


def _decode_color_bgr(image_input: str) -> np.ndarray:
    raw = _load_image_bytes(image_input)
    arr = np.frombuffer(raw, dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError("Failed to decode color image")
    return img


def _decode_depth_m(image_input: str) -> np.ndarray:
    raw = _load_image_bytes(image_input)
    arr = np.frombuffer(raw, dtype=np.uint8)
    depth = cv2.imdecode(arr, cv2.IMREAD_UNCHANGED)
    if depth is None:
        raise ValueError("Failed to decode depth image")
    if depth.ndim != 2:
        raise ValueError(f"Depth image must be single-channel, got shape {depth.shape}")

    if depth.dtype == np.uint16:
        return depth.astype(np.float32) / 1000.0
    if np.issubdtype(depth.dtype, np.floating):
        depth_f = depth.astype(np.float32, copy=False)
        depth_f = np.where(np.isfinite(depth_f), depth_f, 0.0)
        if depth_f.size and float(np.nanmax(depth_f)) > 20.0:
            depth_f = depth_f / 1000.0
        return depth_f
    if np.issubdtype(depth.dtype, np.integer):
        return depth.astype(np.float32) / 1000.0
    raise ValueError(f"Unsupported depth image dtype: {depth.dtype}")


def _parse_intrinsics(k: List[List[float]]) -> np.ndarray:
    matrix = np.array(k, dtype=np.float64)
    if matrix.shape != (3, 3):
        raise ValueError(f"camera_intrinsics must be 3x3, got {matrix.shape}")
    if not np.all(np.isfinite(matrix)):
        raise ValueError("camera_intrinsics contains non-finite values")
    if matrix[0, 0] <= 0 or matrix[1, 1] <= 0:
        raise ValueError("camera_intrinsics fx/fy must be positive")
    return matrix


def _rot_to_quat_xyzw(r: np.ndarray) -> List[float]:
    m00, m01, m02 = r[0, 0], r[0, 1], r[0, 2]
    m10, m11, m12 = r[1, 0], r[1, 1], r[1, 2]
    m20, m21, m22 = r[2, 0], r[2, 1], r[2, 2]
    tr = m00 + m11 + m22
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m21 - m12) / s
        qy = (m02 - m20) / s
        qz = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / s
        qx = 0.25 * s
        qy = (m01 + m10) / s
        qz = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / s
        qx = (m01 + m10) / s
        qy = 0.25 * s
        qz = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / s
        qx = (m02 + m20) / s
        qy = (m12 + m21) / s
        qz = 0.25 * s
    q = np.array([qx, qy, qz, qw], dtype=np.float64)
    n = np.linalg.norm(q)
    if n > 0:
        q /= n
    return [float(v) for v in q]


def _encode_color_for_yolo(color_bgr: np.ndarray) -> str:
    ok, buf = cv2.imencode(".png", color_bgr)
    if not ok:
        raise RuntimeError("Failed to encode color image for YOLO")
    return base64.b64encode(buf).decode("utf-8")


def _yolo_detect(color_bgr: np.ndarray, object_name: str) -> List[Detection]:
    url = YOLO_URL if YOLO_URL.endswith("/detect") else YOLO_URL + "/detect"
    payload = {
        "image": _encode_color_for_yolo(color_bgr),
        "conf_thres": YOLO_CONF_THRES,
        "iou_thres": YOLO_IOU_THRES,
    }
    logger.info("YOLO detection request url=%s target=%s", url, object_name)
    response = requests.post(url, json=payload, timeout=20)
    response.raise_for_status()
    data = response.json()
    detections = []
    target = object_name.strip().lower()
    for item in data.get("detections", []) or []:
        label = str(item.get("class_name", item.get("name", "")))
        if label.lower() != target:
            continue
        xyxy = (
            int(round(float(item.get("x1", item.get("xmin"))))),
            int(round(float(item.get("y1", item.get("ymin"))))),
            int(round(float(item.get("x2", item.get("xmax"))))),
            int(round(float(item.get("y2", item.get("ymax"))))),
        )
        detections.append(
            Detection(
                label=label,
                confidence=float(item.get("confidence", 0.0)),
                xyxy=xyxy,
            )
        )
    detections.sort(key=lambda d: d.confidence, reverse=True)
    logger.info("YOLO detection target=%s count=%d boxes=%s", object_name, len(detections), [d.xyxy for d in detections])
    return detections


def _yolo_health_url() -> str:
    if YOLO_URL.endswith("/detect"):
        return YOLO_URL.rsplit("/", 1)[0] + "/health"
    return YOLO_URL + "/health"


def _object_points(depth_m: np.ndarray, k: np.ndarray, xyxy: Tuple[int, int, int, int]) -> Tuple[np.ndarray, np.ndarray]:
    h, w = depth_m.shape[:2]
    x1, y1, x2, y2 = xyxy
    x1, x2 = max(0, min(x1, w - 1)), max(0, min(x2, w))
    y1, y2 = max(0, min(y1, h - 1)), max(0, min(y2, h))
    if x2 <= x1 + 2 or y2 <= y1 + 2:
        raise ValueError(f"empty bbox after clipping: {xyxy}")

    # Use a lightly shrunken box to reduce table/background depth leakage.
    sx = max(1, int(0.05 * (x2 - x1)))
    sy = max(1, int(0.05 * (y2 - y1)))
    x1s, x2s = x1 + sx, x2 - sx
    y1s, y2s = y1 + sy, y2 - sy

    roi = depth_m[y1s:y2s, x1s:x2s].astype(np.float32, copy=False)
    valid = np.isfinite(roi) & (roi >= MIN_DEPTH_M) & (roi <= MAX_DEPTH_M)
    if int(np.count_nonzero(valid)) < MIN_POINTS:
        return np.empty((0, 3), dtype=np.float32), np.empty((0, 2), dtype=np.float32)

    vals = roi[valid]
    lo, hi = np.percentile(vals, [10.0, 90.0])
    median_z = float(np.median(vals))
    band = max(0.03, 2.5 * float(np.median(np.abs(vals - median_z))))
    valid &= (roi >= max(lo, median_z - band)) & (roi <= min(hi, median_z + band))

    ys, xs = np.nonzero(valid)
    if len(xs) < MIN_POINTS:
        return np.empty((0, 3), dtype=np.float32), np.empty((0, 2), dtype=np.float32)

    u = xs.astype(np.float32) + float(x1s)
    v = ys.astype(np.float32) + float(y1s)
    z = roi[ys, xs].astype(np.float32)
    fx, fy = float(k[0, 0]), float(k[1, 1])
    cx, cy = float(k[0, 2]), float(k[1, 2])
    x = (u - cx) / fx * z
    y = (v - cy) / fy * z
    points = np.stack([x, y, z], axis=1).astype(np.float32)
    pixels = np.stack([u, v], axis=1).astype(np.float32)
    return points, pixels


def _estimate_geometry(points: np.ndarray) -> Tuple[np.ndarray, float, float, float, np.ndarray]:
    center = np.median(points, axis=0).astype(np.float64)
    xy = points[:, :2].astype(np.float64)
    xy_centered = xy - np.median(xy, axis=0)
    cov = np.cov(xy_centered.T)
    eigvals, eigvecs = np.linalg.eigh(cov)
    order = np.argsort(eigvals)[::-1]
    eigvecs = eigvecs[:, order]
    principal = eigvecs[:, 0]
    yaw = math.atan2(float(principal[1]), float(principal[0]))

    long_proj = xy_centered @ eigvecs[:, 0]
    short_proj = xy_centered @ eigvecs[:, 1]
    long_extent = float(np.percentile(long_proj, 95) - np.percentile(long_proj, 5))
    short_extent = float(np.percentile(short_proj, 95) - np.percentile(short_proj, 5))
    width = max(GRIPPER_MIN_WIDTH_M, min(GRIPPER_MAX_WIDTH_M, short_extent * 1.15))
    return center, yaw, width, long_extent, eigvecs


def _rotation_from_yaw(yaw: float) -> np.ndarray:
    # Camera optical frame: x right, y down, z forward. Approach along +Z;
    # retreat is toward the camera by subtracting this approach vector.
    closing = np.array([math.cos(yaw + math.pi / 2.0), math.sin(yaw + math.pi / 2.0), 0.0])
    lateral = np.array([-math.sin(yaw + math.pi / 2.0), math.cos(yaw + math.pi / 2.0), 0.0])
    approach = np.array([0.0, 0.0, 1.0])
    r = np.stack([closing, lateral, approach], axis=1).astype(np.float64)
    if np.linalg.det(r) < 0:
        r[:, 1] *= -1.0
    return r


def _candidate_score(
    offset_norm: float,
    point_count: int,
    detection_conf: float,
    width: float,
    valid_ratio: float,
) -> float:
    width_mid = 0.5 * (GRIPPER_MIN_WIDTH_M + GRIPPER_MAX_WIDTH_M)
    width_span = max(1e-6, GRIPPER_MAX_WIDTH_M - GRIPPER_MIN_WIDTH_M)
    width_score = max(0.0, 1.0 - abs(width - width_mid) / width_span)
    support_score = min(1.0, math.log10(max(point_count, 1)) / 4.0)
    centrality_score = max(0.0, 1.0 - offset_norm)
    score = 0.35 * detection_conf + 0.25 * valid_ratio + 0.20 * support_score + 0.15 * centrality_score + 0.05 * width_score
    return float(max(0.0, min(1.0, score)))


def _make_grasps(det: Detection, points: np.ndarray, top_k: int) -> List[Dict[str, Any]]:
    if len(points) < MIN_POINTS:
        logger.info("reject bbox=%s reason=too_few_valid_depth_points count=%d", det.xyxy, len(points))
        return []

    center, yaw, width, long_extent, eigvecs = _estimate_geometry(points)
    logger.info("valid depth point count=%d", len(points))
    logger.info("estimated object center camera_xyz=%s", [round(float(v), 5) for v in center])
    logger.info("PCA orientation yaw_rad=%.5f yaw_deg=%.2f long_extent=%.4f", yaw, math.degrees(yaw), long_extent)
    logger.info("estimated grasp width=%.4f gripper_range=[%.4f, %.4f]", width, GRIPPER_MIN_WIDTH_M, GRIPPER_MAX_WIDTH_M)

    if width > GRIPPER_MAX_WIDTH_M:
        logger.info("reject bbox=%s reason=estimated_width_exceeds_gripper width=%.4f", det.xyxy, width)
        return []

    offsets = [
        (0.0, 0.0, 0.0),
        (0.010, 0.0, 0.0),
        (-0.010, 0.0, 0.0),
        (0.0, 0.010, 0.0),
        (0.0, -0.010, 0.0),
        (0.0, 0.0, math.radians(10.0)),
        (0.0, 0.0, math.radians(-10.0)),
    ]
    point_count = int(len(points))
    valid_ratio = min(1.0, point_count / max(1.0, (det.xyxy[2] - det.xyxy[0]) * (det.xyxy[3] - det.xyxy[1]) * 0.25))

    candidates: List[Dict[str, Any]] = []
    for dx, dy, dyaw in offsets:
        t = center.copy()
        t[0] += dx
        t[1] += dy
        r = _rotation_from_yaw(yaw + dyaw)
        retreat = t - r[:, 2] * FALLBACK_APPROACH_OFFSET_M
        offset_norm = min(1.0, math.hypot(dx, dy) / 0.025 + abs(dyaw) / math.radians(30.0))
        score = _candidate_score(offset_norm, point_count, det.confidence, width, valid_ratio)
        candidates.append(
            {
                "score": score,
                "width": float(width),
                "translation_camera": [float(v) for v in t.tolist()],
                "translation_camera_retreat": [float(v) for v in retreat.tolist()],
                "quaternion_camera_xyzw": _rot_to_quat_xyzw(r),
                "rotation_camera": r.tolist(),
                "backend": BACKEND_NAME,
                "point_count": point_count,
            }
        )

    candidates.sort(key=lambda g: g["score"], reverse=True)
    selected = candidates[:top_k]
    logger.info("returned candidates=%s", [{"score": round(g["score"], 3), "width": round(g["width"], 4), "t": [round(v, 4) for v in g["translation_camera"]]} for g in selected])
    return selected


app = FastAPI(
    title="ABot-Claw GraspAnything Depth Fallback",
    version=SERVICE_VERSION,
    description="License-free YOLO + aligned-depth fallback preserving the GraspAnything API.",
)


@app.get("/health")
def health() -> Dict[str, Any]:
    yolo_status: Dict[str, Any]
    try:
        response = requests.get(_yolo_health_url(), timeout=3)
        yolo_status = response.json()
        yolo_ok = response.ok and yolo_status.get("status") == "ok"
    except Exception as exc:
        yolo_status = {"error": str(exc)}
        yolo_ok = False
    return {
        "status": "ok" if yolo_ok else "degraded",
        "backend": BACKEND_NAME,
        "version": SERVICE_VERSION,
        "model_loaded": True,
        "yolo_url": YOLO_URL,
        "yolo_ok": yolo_ok,
        "yolo": yolo_status,
        "min_depth_m": MIN_DEPTH_M,
        "max_depth_m": MAX_DEPTH_M,
        "gripper_min_width_m": GRIPPER_MIN_WIDTH_M,
        "gripper_max_width_m": GRIPPER_MAX_WIDTH_M,
        "approach_offset_m": FALLBACK_APPROACH_OFFSET_M,
    }


@app.post("/grasp/detect", response_model=GraspResponse)
def grasp_detect(req: GraspRequest) -> GraspResponse:
    if not req.object_name.strip():
        raise HTTPException(status_code=400, detail="object_name cannot be empty")

    start_t = time.time()
    try:
        color_bgr = _decode_color_bgr(req.color_image)
        depth_m = _decode_depth_m(req.depth_image)
        k = _parse_intrinsics(req.camera_intrinsics)
        if color_bgr.shape[:2] != depth_m.shape[:2]:
            raise ValueError(f"color/depth resolution mismatch: color={color_bgr.shape[:2]}, depth={depth_m.shape[:2]}")
    except Exception as exc:
        raise HTTPException(status_code=400, detail=f"Invalid input payload: {exc}")

    try:
        detections = _yolo_detect(color_bgr, req.object_name)
    except Exception as exc:
        logger.exception("YOLO detection failed")
        raise HTTPException(status_code=502, detail=f"YOLO detection failed: {exc}")

    results: List[Dict[str, Any]] = []
    for det in detections:
        try:
            points, _pixels = _object_points(depth_m, k, det.xyxy)
            grasps = _make_grasps(det, points, req.top_k)
        except Exception as exc:
            logger.info("reject bbox=%s reason=%s", det.xyxy, exc)
            grasps = []
        results.append(
            {
                "label": det.label,
                "confidence": det.confidence,
                "xyxy": list(det.xyxy),
                "backend": BACKEND_NAME,
                "grasps": grasps,
            }
        )

    latency_ms = (time.time() - start_t) * 1000.0
    return GraspResponse(
        frame_id=os.getenv("CAMERA_FRAME_ID", "camera_frame"),
        target=req.object_name,
        top_k=req.top_k,
        count=len(results),
        results=results,
        latency_ms=latency_ms,
        backend=BACKEND_NAME,
    )


if __name__ == "__main__":
    port = int(os.getenv("PORT", "8015"))
    uvicorn.run("grasp_service_depth_fallback:app", host="0.0.0.0", port=port, reload=False)
