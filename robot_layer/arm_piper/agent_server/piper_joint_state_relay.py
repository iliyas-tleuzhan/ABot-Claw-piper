#!/usr/bin/env python3
"""Relay live Piper feedback into MoveIt's standard /joint_states topic."""

from __future__ import annotations

import rospy
from sensor_msgs.msg import JointState


class PiperJointStateRelay:
    def __init__(self) -> None:
        feedback_topic = rospy.get_param("~feedback_topic", "/joint_states_single")
        state_topic = rospy.get_param("~state_topic", "/joint_states")
        self.publisher = rospy.Publisher(state_topic, JointState, queue_size=1)
        rospy.Subscriber(feedback_topic, JointState, self.on_feedback, queue_size=1)
        rospy.loginfo("Relaying Piper feedback from %s to %s", feedback_topic, state_topic)

    def on_feedback(self, message: JointState) -> None:
        state = JointState()
        state.header = message.header
        state.name = message.name
        state.position = message.position
        state.velocity = message.velocity
        state.effort = message.effort
        self.publisher.publish(state)


def main() -> int:
    rospy.init_node("piper_joint_state_relay")
    PiperJointStateRelay()
    rospy.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
