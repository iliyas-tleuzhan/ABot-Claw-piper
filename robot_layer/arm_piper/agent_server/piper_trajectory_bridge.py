#!/usr/bin/env python3
"""Bridge MoveIt FollowJointTrajectory goals to the live Piper hardware interfaces."""

from __future__ import annotations

import actionlib
import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryFeedback, FollowJointTrajectoryResult
from piper_msgs.srv import Gripper
from sensor_msgs.msg import JointState


ARM_JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
GRIPPER_JOINT = "joint7"
GRIPPER_FEEDBACK_NAMES = ("joint7", "gripper")
MOVEIT_GRIPPER_JOINT7_MIN_M = 0.0
MOVEIT_GRIPPER_JOINT7_MAX_M = 0.035


class PiperTrajectoryBridge:
    def __init__(self) -> None:
        self.command_topic = rospy.get_param("~command_topic", "/piper_joint_commands")
        self.feedback_topic = rospy.get_param("~feedback_topic", "/joint_states_single")
        self.arm_require_feedback = rospy.get_param("~arm_require_feedback", True)
        self.arm_min_speed_percent = rospy.get_param("~arm_min_speed_percent", 10.0)
        self.arm_max_speed_percent = rospy.get_param("~arm_max_speed_percent", 20.0)
        self.arm_speed_reference_rad_s = rospy.get_param("~arm_speed_reference_rad_s", 3.0)
        self.feedback_tolerance_rad = rospy.get_param("~feedback_tolerance_rad", 0.02)
        self.feedback_timeout_s = rospy.get_param("~feedback_timeout_s", 45.0)
        self.gripper_service_name = rospy.get_param("~gripper_service", "/gripper_srv")
        self.gripper_effort = rospy.get_param("~gripper_effort", 1.0)
        self.gripper_feedback_tolerance_m = rospy.get_param("~gripper_feedback_tolerance_m", 0.003)
        self.gripper_feedback_timeout_s = rospy.get_param("~gripper_feedback_timeout_s", 5.0)
        self.gripper_service_wait_timeout_s = rospy.get_param("~gripper_service_wait_timeout_s", 5.0)
        self.gripper_require_feedback = rospy.get_param("~gripper_require_feedback", False)
        self.latest_positions = {}
        self.command_publisher = rospy.Publisher(self.command_topic, JointState, queue_size=1)
        self.gripper_service = rospy.ServiceProxy(self.gripper_service_name, Gripper)
        self.feedback_subscriber = rospy.Subscriber(
            self.feedback_topic, JointState, self.on_feedback, queue_size=1
        )
        self.server = actionlib.SimpleActionServer(
            "arm_controllers/follow_joint_trajectory",
            FollowJointTrajectoryAction,
            execute_cb=self.execute_arm,
            auto_start=False,
        )
        self.gripper_server = actionlib.SimpleActionServer(
            "gripper_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction,
            execute_cb=self.execute_gripper,
            auto_start=False,
        )
        self.server.start()
        self.gripper_server.start()
        rospy.loginfo(
            "Piper trajectory bridge ready: MoveIt arm trajectories publish to %s and wait for %s",
            self.command_topic,
            self.feedback_topic,
        )
        rospy.loginfo(
            "Piper gripper trajectory bridge ready: MoveIt gripper trajectories call %s",
            self.gripper_service_name,
        )

    def on_feedback(self, state: JointState) -> None:
        self.latest_positions = dict(zip(state.name, state.position))

    @staticmethod
    def moveit_finger_joint_to_total_opening_width(moveit_finger_joint_m: float) -> float:
        """Convert MoveIt joint7 single-finger displacement to physical total jaw opening."""
        return max(
            0.0,
            min(MOVEIT_GRIPPER_JOINT7_MAX_M, abs(float(moveit_finger_joint_m))),
        ) * 2.0

    @staticmethod
    def feedback_to_moveit_finger_joint(feedback_name: str, feedback_position_m: float) -> float:
        """Normalize live gripper feedback to MoveIt joint7 units.

        The running official Piper driver publishes ``gripper`` with
        gripper_val_mutiple=1, which is the physical total jaw opening.
        A future driver that publishes ``joint7`` is already in MoveIt units.
        """
        value = abs(float(feedback_position_m))
        if feedback_name == "gripper":
            return value / 2.0
        return value

    def reject(self, result: FollowJointTrajectoryResult, message: str, server=None) -> None:
        result.error_code = FollowJointTrajectoryResult.INVALID_JOINTS
        result.error_string = message
        (server or self.server).set_aborted(result, message)

    def wait_for_final_position(self, target_positions) -> bool:
        deadline = rospy.Time.now() + rospy.Duration(self.feedback_timeout_s)
        rate = rospy.Rate(50)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if self.server.is_preempt_requested():
                self.server.set_preempted(text="Trajectory preempted")
                return False
            if all(
                name in self.latest_positions
                and abs(self.latest_positions[name] - target) <= self.feedback_tolerance_rad
                for name, target in zip(ARM_JOINTS, target_positions)
            ):
                return True
            rate.sleep()
        return False

    def get_gripper_position(self):
        for name in GRIPPER_FEEDBACK_NAMES:
            if name in self.latest_positions:
                return self.feedback_to_moveit_finger_joint(name, self.latest_positions[name])
        return None

    def wait_for_final_gripper_position(self, target_position: float) -> bool:
        deadline = rospy.Time.now() + rospy.Duration(self.gripper_feedback_timeout_s)
        rate = rospy.Rate(50)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if self.gripper_server.is_preempt_requested():
                self.gripper_server.set_preempted(text="Gripper trajectory preempted")
                return False
            actual_position = self.get_gripper_position()
            if (
                actual_position is not None
                and abs(actual_position - abs(target_position)) <= self.gripper_feedback_tolerance_m
            ):
                return True
            rate.sleep()
        return False

    def arm_speed_percent_for_point(self, point) -> float:
        if len(point.velocities) == len(ARM_JOINTS):
            max_velocity_rad_s = max(abs(float(v)) for v in point.velocities)
            if max_velocity_rad_s > 0.0:
                speed_percent = 100.0 * max_velocity_rad_s / float(self.arm_speed_reference_rad_s)
            else:
                speed_percent = self.arm_min_speed_percent
        else:
            speed_percent = self.arm_min_speed_percent

        return max(
            float(self.arm_min_speed_percent),
            min(float(self.arm_max_speed_percent), speed_percent),
        )

    def execute_arm(self, goal) -> None:
        trajectory = goal.trajectory
        result = FollowJointTrajectoryResult()
        if tuple(trajectory.joint_names) != ARM_JOINTS:
            self.reject(result, f"Expected joints {ARM_JOINTS}, got {tuple(trajectory.joint_names)}")
            return
        if not trajectory.points:
            self.reject(result, "Trajectory contains no points")
            return
        if any(len(point.positions) != len(ARM_JOINTS) for point in trajectory.points):
            self.reject(result, "Each trajectory point must contain six arm joint positions")
            return

        started_at = rospy.Time.now()
        for point in trajectory.points:
            while not rospy.is_shutdown() and rospy.Time.now() - started_at < point.time_from_start:
                if self.server.is_preempt_requested():
                    self.server.set_preempted(text="Trajectory preempted")
                    return
                rospy.sleep(0.002)

            command = JointState()
            command.header.stamp = rospy.Time.now()
            command.name = list(ARM_JOINTS)
            command.position = list(point.positions)
            speed_percent = self.arm_speed_percent_for_point(point)
            command.velocity = [0.0] * len(ARM_JOINTS) + [speed_percent]
            self.command_publisher.publish(command)

            feedback = FollowJointTrajectoryFeedback()
            feedback.joint_names = list(ARM_JOINTS)
            feedback.desired.positions = list(point.positions)
            feedback.desired.time_from_start = point.time_from_start
            feedback.actual.positions = [self.latest_positions.get(name, 0.0) for name in ARM_JOINTS]
            feedback.actual.time_from_start = point.time_from_start
            self.server.publish_feedback(feedback)

        final_actual = [self.latest_positions.get(name) for name in ARM_JOINTS]
        rospy.loginfo(
            "Arm trajectory command stream complete: final_target=%s final_feedback=%s",
            [round(float(v), 6) for v in trajectory.points[-1].positions],
            [
                None if value is None else round(float(value), 6)
                for value in final_actual
            ],
        )

        if self.arm_require_feedback and not self.wait_for_final_position(trajectory.points[-1].positions):
            result.error_code = FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED
            result.error_string = "Timed out waiting for Piper joint feedback at final trajectory point"
            self.server.set_aborted(result, result.error_string)
            return

        result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
        self.server.set_succeeded(
            result,
            "Piper arm trajectory command stream complete"
            if not self.arm_require_feedback
            else "Piper joint feedback reached final trajectory point",
        )

    def execute_gripper(self, goal) -> None:
        trajectory = goal.trajectory
        result = FollowJointTrajectoryResult()
        if tuple(trajectory.joint_names) != (GRIPPER_JOINT,):
            self.reject(
                result,
                f"Expected gripper joint {(GRIPPER_JOINT,)}, got {tuple(trajectory.joint_names)}",
                self.gripper_server,
            )
            return
        if not trajectory.points:
            result.error_code = FollowJointTrajectoryResult.INVALID_GOAL
            result.error_string = "Gripper trajectory contains no points"
            self.gripper_server.set_aborted(result, result.error_string)
            return
        if any(len(point.positions) != 1 for point in trajectory.points):
            result.error_code = FollowJointTrajectoryResult.INVALID_GOAL
            result.error_string = "Each gripper trajectory point must contain one joint7 position"
            self.gripper_server.set_aborted(result, result.error_string)
            return

        try:
            rospy.wait_for_service(self.gripper_service_name, self.gripper_service_wait_timeout_s)
        except rospy.ROSException as exc:
            result.error_code = FollowJointTrajectoryResult.INVALID_GOAL
            result.error_string = f"Timed out waiting for {self.gripper_service_name}: {exc}"
            self.gripper_server.set_aborted(result, result.error_string)
            return

        started_at = rospy.Time.now()
        for point in trajectory.points:
            while not rospy.is_shutdown() and rospy.Time.now() - started_at < point.time_from_start:
                if self.gripper_server.is_preempt_requested():
                    self.gripper_server.set_preempted(text="Gripper trajectory preempted")
                    return
                rospy.sleep(0.002)

            moveit_finger_joint_m = max(
                MOVEIT_GRIPPER_JOINT7_MIN_M,
                min(MOVEIT_GRIPPER_JOINT7_MAX_M, float(point.positions[0])),
            )
            total_opening_width_m = self.moveit_finger_joint_to_total_opening_width(
                moveit_finger_joint_m
            )
            try:
                response = self.gripper_service(
                    gripper_angle=total_opening_width_m,
                    gripper_effort=self.gripper_effort,
                    gripper_code=1,
                    set_zero=0,
                )
            except rospy.ServiceException as exc:
                result.error_code = FollowJointTrajectoryResult.INVALID_GOAL
                result.error_string = f"Gripper service call failed: {exc}"
                self.gripper_server.set_aborted(result, result.error_string)
                return

            if not response.status:
                result.error_code = FollowJointTrajectoryResult.INVALID_GOAL
                result.error_string = (
                    "Gripper service rejected command "
                    f"moveit_finger_joint_m={moveit_finger_joint_m:.6f} "
                    f"total_opening_width_m={total_opening_width_m:.6f} "
                    f"with code {getattr(response, 'code', 'unknown')}"
                )
                self.gripper_server.set_aborted(result, result.error_string)
                return

            actual_position = self.get_gripper_position()
            rospy.loginfo(
                "Gripper command accepted: moveit_finger_joint_m=%.6f "
                "total_opening_width_m=%.6f feedback_moveit_finger_joint_m=%s",
                moveit_finger_joint_m,
                total_opening_width_m,
                "none" if actual_position is None else f"{actual_position:.6f}",
            )
            feedback = FollowJointTrajectoryFeedback()
            feedback.joint_names = [GRIPPER_JOINT]
            feedback.desired.positions = [moveit_finger_joint_m]
            feedback.desired.time_from_start = point.time_from_start
            feedback.actual.positions = [actual_position if actual_position is not None else 0.0]
            feedback.actual.time_from_start = point.time_from_start
            self.gripper_server.publish_feedback(feedback)

        if self.gripper_require_feedback and self.get_gripper_position() is not None and not self.wait_for_final_gripper_position(
            trajectory.points[-1].positions[0]
        ):
            result.error_code = FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED
            result.error_string = "Timed out waiting for Piper gripper feedback at final trajectory point"
            self.gripper_server.set_aborted(result, result.error_string)
            return

        result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
        self.gripper_server.set_succeeded(
            result,
            "Piper gripper service accepted final trajectory point"
            if not self.gripper_require_feedback
            else "Piper gripper feedback reached final trajectory point",
        )


def main() -> int:
    rospy.init_node("piper_trajectory_bridge")
    PiperTrajectoryBridge()
    rospy.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
