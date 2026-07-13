import os
import sys
import types
import unittest


AGENT_SERVER_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if AGENT_SERVER_DIR not in sys.path:
    sys.path.insert(0, AGENT_SERVER_DIR)

os.environ["PIPER_ENV_SOURCED"] = "1"


def _install_ros_stubs():
    rospy = types.ModuleType("rospy")
    rospy.logwarn = lambda *args, **kwargs: None
    rospy.get_node_uri = lambda: "stub-node-uri"
    rospy.wait_for_service = lambda *args, **kwargs: None
    rospy.ServiceProxy = lambda *args, **kwargs: None
    rospy.Subscriber = lambda *args, **kwargs: None
    rospy.sleep = lambda *args, **kwargs: None
    sys.modules["rospy"] = rospy

    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.PoseStamped = type("PoseStamped", (), {})
    geometry_msgs.msg = geometry_msgs_msg
    sys.modules["geometry_msgs"] = geometry_msgs
    sys.modules["geometry_msgs.msg"] = geometry_msgs_msg

    moveit_ctrl = types.ModuleType("moveit_ctrl")
    moveit_ctrl_srv = types.ModuleType("moveit_ctrl.srv")

    class JointMoveitCtrlRequest:
        def __init__(self):
            self.joint_states = []
            self.gripper = 0.0
            self.max_velocity = 0.0
            self.max_acceleration = 0.0
            self.joint_endpose = []

    moveit_ctrl_srv.JointMoveitCtrl = object
    moveit_ctrl_srv.JointMoveitCtrlRequest = JointMoveitCtrlRequest
    moveit_ctrl.srv = moveit_ctrl_srv
    sys.modules["moveit_ctrl"] = moveit_ctrl
    sys.modules["moveit_ctrl.srv"] = moveit_ctrl_srv

    tf = types.ModuleType("tf")
    tf_transformations = types.ModuleType("tf.transformations")
    tf_transformations.quaternion_from_euler = lambda *args: (0.0, 0.0, 0.0, 1.0)
    tf.transformations = tf_transformations
    sys.modules["tf"] = tf
    sys.modules["tf.transformations"] = tf_transformations

    image_sdk = types.ModuleType("robot_sdk.piper_image_sdk")
    image_sdk.ImageRecorder = type("ImageRecorder", (), {})
    image_sdk.Recorder = type("Recorder", (), {})
    sys.modules["robot_sdk.piper_image_sdk"] = image_sdk


_install_ros_stubs()

from robot_sdk.piper_sdk import (  # noqa: E402
    DEFAULT_JOINT_LIMIT_TOLERANCE_RAD,
    normalize_arm_joint_targets,
)


class PiperJointNormalizationTest(unittest.TestCase):
    def test_joint2_small_lower_bound_drift_clamps_to_zero(self):
        messages = []
        target = [-0.037644152, -0.001709512, 0.0, -0.065118452, 0.0, 0.079684192]

        normalized = normalize_arm_joint_targets(
            target,
            DEFAULT_JOINT_LIMIT_TOLERANCE_RAD,
            messages.append,
        )

        self.assertEqual(normalized[1], 0.0)
        self.assertEqual(normalized[:1] + normalized[2:], target[:1] + target[2:])
        self.assertEqual(len(messages), 1)
        self.assertIn("joint2", messages[0])
        self.assertIn("lower bound", messages[0])

    def test_inside_bounds_remains_unchanged(self):
        target = [0.1, 0.2, -0.3, 0.4, -0.5, 0.6]

        self.assertEqual(normalize_arm_joint_targets(target), target)

    def test_clearly_invalid_target_is_rejected(self):
        target = [0.0, -0.01, 0.0, 0.0, 0.0, 0.0]

        with self.assertRaisesRegex(ValueError, "joint2.*outside MoveIt bounds"):
            normalize_arm_joint_targets(target)

    def test_upper_bound_drift_clamps_to_upper_bound(self):
        target = [2.6185, 0.0, 0.0, 0.0, 0.0, 0.0]

        normalized = normalize_arm_joint_targets(target)

        self.assertEqual(normalized[0], 2.618)

    def test_wrong_target_length_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "must have 6 elements"):
            normalize_arm_joint_targets([0.0] * 5)


if __name__ == "__main__":
    unittest.main()
