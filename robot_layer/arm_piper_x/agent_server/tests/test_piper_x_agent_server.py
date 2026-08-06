import os
import unittest
from unittest.mock import patch

from fastapi.testclient import TestClient

from robot_layer.arm_piper_x.agent_server.config import PiperXAgentConfig
from robot_layer.arm_piper_x.agent_server.lease import LeaseManager
from robot_layer.arm_piper_x.agent_server.server import build_app


class FakeSdk:
    def __init__(self):
        self.calls = []

    def health(self):
        return {
            "status": "ready",
            "ros_ok": True,
            "marker_pose_available": True,
            "point_cloud_available": True,
            "moveit_available": True,
            "marker_task_service_available": True,
            "home_action_available": True,
            "joint_state_available": True,
            "execution_allowed": True,
        }

    def approach_marker(self, payload):
        self.calls.append(("approach", payload))
        return 200, {"success": True, "stage": "complete", "message": "approach ok"}

    def touch_marker(self, payload):
        self.calls.append(("touch", payload))
        return 200, {
            "success": True,
            "stage": "complete",
            "message": "touch ok",
            "contact_confirmed": False,
            "completion_type": "geometric_surface_approach",
        }

    def go_home(self, payload):
        self.calls.append(("home", payload))
        return 200, {"success": True, "stage": "complete", "message": "home ok"}

    def save_home(self, payload):
        self.calls.append(("save-home", payload))
        return 200, {"success": True, "stage": "complete", "message": "saved"}

    def validate_pose_payload(self, payload):
        return True, "ok"


class FakeStateMonitor:
    def __init__(self, gripper_subscribers=True):
        self.gripper_commands = []
        self.gripper_subscribers = gripper_subscribers

    def start(self):
        pass

    def stop(self):
        pass

    def state(self):
        return {
            "joint_state_fresh": True,
            "tcp_pose_fresh": True,
            "joint_state": {"positions_rad": {f"joint{i}": 0.0 for i in range(1, 7)}},
        }

    def graph(self):
        return {
            "trajectory_action_available": True,
            "marker_task_service_seen": True,
            "topics": [],
            "services": [],
            "actions": ["/arm_controller/follow_joint_trajectory"],
            "gripper_control_topic_seen": True,
            "gripper_control_subscribers": 1 if self.gripper_subscribers else 0,
        }

    def command_gripper(self, joint_name, width_m, effort_n):
        if not self.gripper_subscribers:
            raise RuntimeError("no subscribers are connected to /control/joint_states")
        self.gripper_commands.append((joint_name, width_m, effort_n))
        return {
            "success": True,
            "stage": "complete",
            "message": "published",
            "joint_name": joint_name,
            "target_width_m": width_m,
            "effort_n": effort_n,
        }


def make_client(execution_allowed=False, gripper_subscribers=True):
    cfg = PiperXAgentConfig()
    sdk = FakeSdk()
    state_monitor = FakeStateMonitor(gripper_subscribers=gripper_subscribers)
    app = build_app(cfg=cfg, sdk=sdk, state_monitor=state_monitor, lease_mgr=LeaseManager())
    env_value = "1" if execution_allowed else ""
    return TestClient(app), sdk, env_value, state_monitor


class PiperXAgentServerTest(unittest.TestCase):
    def test_health_reports_agent(self):
        client, _sdk, _env, _state = make_client()
        result = client.get("/health")
        self.assertEqual(result.status_code, 200)
        self.assertEqual(result.json()["agent"], "piper_x_agent_server")

    def test_plan_only_approach_does_not_require_lease(self):
        client, sdk, _env, _state = make_client()
        result = client.post("/tools/approach-marker", json={"execute": False})
        self.assertEqual(result.status_code, 200)
        self.assertEqual(sdk.calls[0][0], "approach")
        self.assertFalse(sdk.calls[0][1]["execute"])

    def test_execute_blocked_without_agent_gate(self):
        client, _sdk, _env, _state = make_client()
        result = client.post("/tools/touch-marker", json={"execute": True})
        self.assertEqual(result.status_code, 403)
        self.assertEqual(result.json()["detail"]["stage"], "execution_gate")

    def test_execute_requires_lease_when_gate_enabled(self):
        client, _sdk, _env, _state = make_client(execution_allowed=True)
        with patch.dict(os.environ, {"PIPER_X_AGENT_ALLOW_EXECUTION": "1"}):
            result = client.post("/tools/touch-marker", json={"execute": True})
        self.assertEqual(result.status_code, 409)
        self.assertEqual(result.json()["detail"]["stage"], "lease")

    def test_execute_with_lease_proxies_touch(self):
        client, sdk, _env, _state = make_client(execution_allowed=True)
        with patch.dict(os.environ, {"PIPER_X_AGENT_ALLOW_EXECUTION": "1"}):
            lease = client.post("/lease/acquire", json={"holder": "test"}).json()["lease_id"]
            result = client.post(
                "/tools/touch-marker",
                json={"execute": True, "lease_id": lease, "retract_after": True, "return_home_after": True},
            )
        self.assertEqual(result.status_code, 200)
        self.assertEqual(sdk.calls[0][0], "touch")
        self.assertEqual(sdk.calls[0][1]["lease_id"] if "lease_id" in sdk.calls[0][1] else None, None)

    def test_save_home_proxies_without_execution_lease(self):
        client, sdk, _env, _state = make_client()
        result = client.post("/tools/save-home", json={"pose_name": "home"})
        self.assertEqual(result.status_code, 200)
        self.assertEqual(sdk.calls[0][0], "save-home")

    def test_open_gripper_plan_only_reports_command_without_publish(self):
        client, _sdk, _env, state = make_client()
        result = client.post("/tools/open-gripper", json={"execute": False})
        self.assertEqual(result.status_code, 200)
        self.assertEqual(result.json()["target_width_m"], 0.10)
        self.assertFalse(result.json()["execution_attempted"])
        self.assertEqual(state.gripper_commands, [])

    def test_close_gripper_execute_publishes_with_lease(self):
        client, _sdk, _env, state = make_client(execution_allowed=True)
        with patch.dict(os.environ, {"PIPER_X_AGENT_ALLOW_EXECUTION": "1"}):
            lease = client.post("/lease/acquire", json={"holder": "test"}).json()["lease_id"]
            result = client.post("/tools/close-gripper", json={"execute": True, "lease_id": lease})
        self.assertEqual(result.status_code, 200)
        self.assertEqual(state.gripper_commands, [("gripper", 0.0, 1.0)])

    def test_gripper_invalid_width_rejected(self):
        client, _sdk, _env, _state = make_client()
        result = client.post("/tools/open-gripper", json={"execute": False, "width_m": 0.2})
        self.assertEqual(result.status_code, 400)
        self.assertEqual(result.json()["detail"]["stage"], "request_validation")

    def test_gripper_execute_requires_gate(self):
        client, _sdk, _env, _state = make_client()
        result = client.post("/tools/open-gripper", json={"execute": True})
        self.assertEqual(result.status_code, 403)
        self.assertEqual(result.json()["detail"]["stage"], "execution_gate")

    def test_gripper_execute_requires_ros_driver_subscriber(self):
        client, _sdk, _env, _state = make_client(execution_allowed=True, gripper_subscribers=False)
        with patch.dict(os.environ, {"PIPER_X_AGENT_ALLOW_EXECUTION": "1"}):
            lease = client.post("/lease/acquire", json={"holder": "test"}).json()["lease_id"]
            result = client.post("/tools/open-gripper", json={"execute": True, "lease_id": lease})
        self.assertEqual(result.status_code, 502)
        self.assertEqual(result.json()["detail"]["stage"], "gripper_execution")
        self.assertIn("/control/joint_states", result.json()["detail"]["message"])

    def test_generic_move_to_pose_fails_closed(self):
        client, _sdk, _env, _state = make_client()
        result = client.post(
            "/tools/move-to-pose",
            json={
                "execute": False,
                "position_m": [0.1, 0.0, 0.2],
                "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
            },
        )
        self.assertEqual(result.status_code, 501)


if __name__ == "__main__":
    unittest.main()
