from __future__ import annotations

import importlib.util
import sys
import types
from pathlib import Path


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "joint_moveit_ctrl_server.py"
)


class _FakePose:
    def __init__(self):
        self.position = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.orientation = types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)


class _FakeResponse:
    def __init__(self, status=False, error_code=-1):
        self.status = status
        self.error_code = error_code


def _load_module():
    fake_rospy = types.SimpleNamespace(
        loginfo=lambda *args, **kwargs: None,
        logerr=lambda *args, **kwargs: None,
        init_node=lambda *args, **kwargs: None,
        Service=lambda *args, **kwargs: None,
        spin=lambda: None,
    )
    fake_moveit_commander = types.SimpleNamespace(
        roscpp_initialize=lambda *args, **kwargs: None,
        RobotCommander=lambda: None,
        MoveGroupCommander=lambda *args, **kwargs: None,
    )
    fake_moveit_srv = types.SimpleNamespace(
        JointMoveitCtrl=object,
        JointMoveitCtrlResponse=_FakeResponse,
    )
    fake_geometry_msg = types.SimpleNamespace(Pose=_FakePose)

    old_modules = {
        name: sys.modules.get(name)
        for name in (
            "rospy",
            "moveit_commander",
            "moveit_ctrl",
            "moveit_ctrl.srv",
            "geometry_msgs",
            "geometry_msgs.msg",
        )
    }
    sys.modules["rospy"] = fake_rospy
    sys.modules["moveit_commander"] = fake_moveit_commander
    sys.modules["moveit_ctrl"] = types.SimpleNamespace(srv=fake_moveit_srv)
    sys.modules["moveit_ctrl.srv"] = fake_moveit_srv
    sys.modules["geometry_msgs"] = types.SimpleNamespace(msg=fake_geometry_msg)
    sys.modules["geometry_msgs.msg"] = fake_geometry_msg
    try:
        spec = importlib.util.spec_from_file_location("joint_moveit_ctrl_server_under_test", SCRIPT_PATH)
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        spec.loader.exec_module(module)
        return module
    finally:
        for name, previous in old_modules.items():
            if previous is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = previous


class _FakeTrajectory:
    def __init__(self, points):
        self.joint_trajectory = types.SimpleNamespace(points=list(points))


class _FakeGroup:
    def __init__(self):
        self.calls = []
        self.plan_result = (True, _FakeTrajectory([1, 2, 3]), 0.25, types.SimpleNamespace(val=1))

    def get_planning_frame(self):
        return "dummy_link"

    def set_start_state_to_current_state(self):
        self.calls.append(("set_start_state_to_current_state",))

    def set_pose_reference_frame(self, frame):
        self.calls.append(("set_pose_reference_frame", frame))

    def set_end_effector_link(self, link):
        self.calls.append(("set_end_effector_link", link))

    def set_pose_target(self, pose, link):
        self.calls.append(
            (
                "set_pose_target",
                [pose.position.x, pose.position.y, pose.position.z],
                [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
                link,
            )
        )

    def set_max_velocity_scaling_factor(self, value):
        self.calls.append(("set_max_velocity_scaling_factor", value))

    def set_max_acceleration_scaling_factor(self, value):
        self.calls.append(("set_max_acceleration_scaling_factor", value))

    def plan(self):
        self.calls.append(("plan",))
        return self.plan_result

    def execute(self, trajectory, wait=True):
        self.calls.append(("execute", trajectory, wait))
        return True

    def stop(self):
        self.calls.append(("stop",))

    def clear_pose_targets(self):
        self.calls.append(("clear_pose_targets",))


def test_endpose_planning_uses_arm_group_and_gripper_tcp():
    module = _load_module()
    server = module.JointMoveitCtrlServer.__new__(module.JointMoveitCtrlServer)
    server.arm_move_group = _FakeGroup()
    server.piper_move_group = _FakeGroup()

    request = types.SimpleNamespace(
        joint_endpose=[
            0.1938084985649814,
            0.003446942509262271,
            0.22727129423740444,
            -0.011341821756328864,
            0.67454000270002,
            -0.0036232226344859726,
            0.7381422763224215,
        ],
        max_velocity=0.05,
        max_acceleration=0.05,
    )

    plan_info, error_response = server._plan_endpose_for_tcp(request)

    assert error_response is None
    assert plan_info["planning_frame"] == "dummy_link"
    assert plan_info["end_effector_link"] == "gripper_tcp"
    assert plan_info["trajectory_points"] == 3
    assert ("set_end_effector_link", "gripper_tcp") in server.arm_move_group.calls
    assert (
        "set_pose_target",
        [0.1938084985649814, 0.003446942509262271, 0.22727129423740444],
        [-0.011341821756328864, 0.67454000270002, -0.0036232226344859726, 0.7381422763224215],
        "gripper_tcp",
    ) in server.arm_move_group.calls
    assert server.piper_move_group.calls == []


def test_endpose_handler_executes_verified_trajectory():
    module = _load_module()
    server = module.JointMoveitCtrlServer.__new__(module.JointMoveitCtrlServer)
    fake_group = _FakeGroup()
    server.arm_move_group = fake_group
    server.piper_move_group = _FakeGroup()

    request = types.SimpleNamespace(
        joint_endpose=[0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0],
        max_velocity=0.05,
        max_acceleration=0.05,
    )
    response = server.handle_joint_moveit_ctrl_endpose(request)
    assert response.status is True
    assert response.error_code == 0
    assert any(call[0] == "execute" for call in fake_group.calls)
    assert ("stop",) in fake_group.calls
    assert ("clear_pose_targets",) in fake_group.calls
