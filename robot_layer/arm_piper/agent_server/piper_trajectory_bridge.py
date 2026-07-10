#!/usr/bin/env python3
"""Bridge MoveIt FollowJointTrajectory goals to the live Piper joint command topic."""

from __future__ import annotations

import actionlib
import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryFeedback, FollowJointTrajectoryResult
from sensor_msgs.msg import JointState


ARM_JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")


class PiperTrajectoryBridge:
    def __init__(self) -> None:
        self.command_topic = rospy.get_param("~command_topic", "/piper_joint_commands")
        self.command_publisher = rospy.Publisher(self.command_topic, JointState, queue_size=1)
        self.server = actionlib.SimpleActionServer(
            "arm_controllers/follow_joint_trajectory",
            FollowJointTrajectoryAction,
            execute_cb=self.execute,
            auto_start=False,
        )
        self.server.start()
        rospy.loginfo(
            "Piper trajectory bridge ready: MoveIt arm trajectories publish to %s", self.command_topic
        )

    def reject(self, result: FollowJointTrajectoryResult, message: str) -> None:
        result.error_code = FollowJointTrajectoryResult.INVALID_JOINTS
        result.error_string = message
        self.server.set_aborted(result, message)

    def execute(self, goal) -> None:
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
            command.velocity = list(point.velocities) if len(point.velocities) == len(ARM_JOINTS) else []
            self.command_publisher.publish(command)

            feedback = FollowJointTrajectoryFeedback()
            feedback.joint_names = list(ARM_JOINTS)
            feedback.desired.positions = list(point.positions)
            feedback.desired.time_from_start = point.time_from_start
            feedback.actual = feedback.desired
            self.server.publish_feedback(feedback)

        result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
        self.server.set_succeeded(result, "Trajectory commands published to Piper driver")


def main() -> int:
    rospy.init_node("piper_trajectory_bridge")
    PiperTrajectoryBridge()
    rospy.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
