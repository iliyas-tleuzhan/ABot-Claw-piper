import base64
import inspect
import io

from fastapi.testclient import TestClient
from PIL import Image

import app as service_app


def _png_8px() -> str:
    buf = io.BytesIO()
    Image.new("RGB", (8, 8), color=(32, 64, 128)).save(buf, format="PNG")
    return base64.b64encode(buf.getvalue()).decode("ascii")


PNG_8PX = _png_8px()


class FakeRuntime(service_app.OpenVLAShadowRuntime):
    def __init__(self) -> None:
        super().__init__()
        self.model_info = service_app.OpenVLAModelInfo(
            processor=type("P", (), {"batch_decode": staticmethod(lambda ids, skip_special_tokens=False: ["decoded"])})(),
            model=type(
                "M",
                (),
                {
                    "vocab_size": 32000,
                    "bin_centers": service_app.np.linspace(-1.0, 1.0, 256),
                    "generate": lambda self, **kwargs: service_app.torch.tensor([[1, 2, 3, 4, 5, 6, 7]]),
                },
            )(),
            action_stats={
                "mask": [True, True, True, True, True, True, False],
                "q01": [-0.1, -0.1, -0.1, -0.1, -0.1, -0.1, 0.0],
                "q99": [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 1.0],
            },
            action_dim=7,
        )

    def preview(self, request: service_app.ActionPreviewRequest):
        image, meta = service_app._decode_image(request.image)
        assert image.width == 8
        return {
            "request_success": True,
            "compatibility_verified": True,
            "inference_attempted": True,
            "inference_success": True,
            "output_shape_valid": True,
            "language_conditioning_tested": False,
            "behavioral_success": None,
            "model_confidence": None,
            "inference_attempted": True,
            "inference_success": True,
            "bridge_unnormalized_action": [0.0] * 7,
            "raw_normalized_action": [0.0] * 7,
            "action_channels": ["x", "y", "z", "roll", "pitch", "yaw", "gripper"],
            "action_semantics": "openvla_bridge_7d_end_effector_action",
            "coordinate_frame": "unverified_for_piper",
            "rotation_convention": "unknown",
            "gripper_semantics": "unverified_for_piper",
            "normalization_key": "bridge_orig",
            "normalization_piper_specific": False,
            "piper_state_model_input": False,
            "image_metadata": meta,
            "execution_allowed": False,
        }


def _client():
    return TestClient(service_app.create_app(FakeRuntime(), load_on_startup=False))


def test_health_response():
    response = _client().get("/health")
    assert response.status_code == 200
    data = response.json()
    assert data["service"] == "openvla_shadow"
    assert data["execution_allowed"] is False


def test_model_info_contains_provenance():
    data = _client().get("/model-info").json()
    assert data["model_id"] == "openvla/openvla-7b"
    assert data["selected_unnorm_key"] == "bridge_orig"
    assert data["execution_allowed"] is False


def test_compatibility_distinguishes_shadow_and_physical():
    response = _client().post(
        "/compatibility-check",
        json={
            "camera_count": 1,
            "camera_source": "external_realsense",
            "language_instruction": True,
            "available_state": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"],
            "desired_action_dimension": 7,
            "desired_future_executor": "piper_moveit_end_pose",
            "execution_allowed": False,
        },
    )
    data = response.json()
    assert data["shadow_inference_allowed"] is True
    assert data["physical_execution_compatible"] is False
    assert data["execution_allowed"] is False


def test_action_preview_single_image_only():
    response = _client().post(
        "/action-preview",
        json={
            "image": "data:image/png;base64," + PNG_8PX,
            "instruction": "Move toward the marked button",
            "execution_allowed": False,
        },
    )
    data = response.json()
    assert data["request_success"] is True
    assert data["output_shape_valid"] is True
    assert data["piper_state_model_input"] is False
    assert data["execution_allowed"] is False


def test_remote_image_rejected():
    response = _client().post(
        "/action-preview",
        json={"image": "https://example.com/test.jpg", "instruction": "Move left", "execution_allowed": False},
    )
    assert response.status_code == 400
    assert response.json()["execution_allowed"] is False


def test_missing_instruction_rejected():
    response = _client().post(
        "/action-preview",
        json={"image": "data:image/png;base64," + PNG_8PX, "instruction": " ", "execution_allowed": False},
    )
    assert response.status_code == 422
    assert response.json()["execution_allowed"] is False


def test_wrong_action_dimension_rejected_in_compatibility():
    data = _client().post(
        "/compatibility-check",
        json={
            "camera_count": 1,
            "camera_source": "external_realsense",
            "language_instruction": True,
            "available_state": [],
            "desired_action_dimension": 6,
            "desired_future_executor": "piper_moveit_end_pose",
            "execution_allowed": False,
        },
    ).json()
    assert data["output_dimension_compatible"] is False
    assert data["shadow_inference_allowed"] is False


def test_model_confidence_remains_null():
    data = _client().post(
        "/action-preview",
        json={"image": "data:image/png;base64," + PNG_8PX, "instruction": "Move left", "execution_allowed": False},
    ).json()
    assert data["model_confidence"] is None
    assert data["behavioral_success"] is None


def test_no_execution_endpoint_exists():
    response = _client().post("/execute", json={})
    assert response.status_code == 404


def test_source_imports_no_motion_backend():
    source = inspect.getsource(service_app)
    import_lines = [line.strip() for line in source.splitlines() if line.strip().startswith(("import ", "from "))]
    joined = "\n".join(import_lines).lower()
    for token in ("moveit", "rospy", "piper_sdk", "socketcan"):
        assert token not in joined
