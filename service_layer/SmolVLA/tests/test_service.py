import base64
import io

from fastapi.testclient import TestClient
from PIL import Image

from app import RuntimeState, build_compatibility_report, create_app, piper_proposed_schema


class Meta:
    config = {
        "type": "smolvla",
        "input_features": {
            "observation.state": {"shape": [6]},
            "observation.images.camera1": {"shape": [3, 256, 256]},
            "observation.images.camera2": {"shape": [3, 256, 256]},
            "observation.images.camera3": {"shape": [3, 256, 256]},
        },
        "output_features": {"action": {"shape": [6]}},
        "chunk_size": 50,
        "normalization_mapping": {"STATE": "MEAN_STD", "ACTION": "MEAN_STD", "VISUAL": "IDENTITY"},
        "vlm_model_name": "HuggingFaceTB/SmolVLM2-500M-Video-Instruct",
        "tokenizer_max_length": 48,
        "empty_cameras": 0,
    }
    preprocessor = {"name": "policy_preprocessor"}
    postprocessor = {"name": "policy_postprocessor"}
    hub_info = {"sha": "rev123"}
    expected_camera_keys = [
        "observation.images.camera1",
        "observation.images.camera2",
        "observation.images.camera3",
    ]
    expected_state_dimension = 6
    expected_action_dimension = 6
    action_chunk_length = 50
    normalization_source = {"mapping": {"STATE": "MEAN_STD"}}


def fake_runtime():
    return RuntimeState(metadata=Meta(), model_loaded=False, load_error=None, parameter_count=None)


def png_data_uri():
    image = Image.new("RGB", (8, 8), (255, 0, 0))
    buffer = io.BytesIO()
    image.save(buffer, format="PNG")
    return "data:image/png;base64," + base64.b64encode(buffer.getvalue()).decode("ascii")


def test_health_response():
    client = TestClient(create_app(fake_runtime()))
    response = client.get("/health")
    assert response.status_code == 200
    assert response.json()["execution_allowed"] is False


def test_model_info_reports_contract():
    client = TestClient(create_app(fake_runtime()))
    body = client.get("/model-info").json()
    assert body["expected_state_dimension"] == 6
    assert body["expected_action_dimension"] == 6
    assert body["execution_allowed"] is False


def test_compatibility_rejects_piper_schema():
    report = build_compatibility_report(piper_proposed_schema(), fake_runtime().metadata)
    assert report["compatible"] is False
    assert report["camera_compatibility"] is False
    assert report["state_dimension_compatibility"] is False
    assert report["action_dimension_compatibility"] is False
    assert report["execution_allowed"] is False


def test_action_preview_refuses_inference_but_accepts_request():
    client = TestClient(create_app(fake_runtime()))
    response = client.post(
        "/action-preview",
        json={
            "images": [png_data_uri()],
            "joint_state": {
                "joint1": 0.1,
                "joint2": 0.2,
                "joint3": 0.3,
                "joint4": 0.4,
                "joint5": 0.5,
                "joint6": 0.6,
                "gripper": 0.0,
            },
            "task_description": "Move toward the button",
        },
    )
    body = response.json()
    assert response.status_code == 200
    assert body["compatible"] is False
    assert body["inference_attempted"] is False
    assert body["execution_allowed"] is False


def test_remote_image_url_rejected():
    client = TestClient(create_app(fake_runtime()))
    response = client.post(
        "/action-preview",
        json={
            "images": ["https://example.com/test.jpg"],
            "joint_state": {
                "joint1": 0.1,
                "joint2": 0.2,
                "joint3": 0.3,
                "joint4": 0.4,
                "joint5": 0.5,
                "joint6": 0.6,
                "gripper": 0.0,
            },
            "task_description": "Move toward the button",
        },
    )
    assert response.status_code == 400
    assert response.json()["execution_allowed"] is False


def test_no_execution_endpoint_exists():
    client = TestClient(create_app(fake_runtime()))
    assert client.post("/execute", json={}).status_code == 404


def test_validation_error_does_not_leak_stack_trace():
    client = TestClient(create_app(fake_runtime()))
    response = client.post("/action-preview", json={"images": [], "task_description": "x", "joint_state": {}})
    body = response.json()
    assert response.status_code == 422
    assert body["request_success"] is False
    assert body["execution_allowed"] is False
    assert "traceback" not in str(body).lower()
