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
        for index, name in enumerate(message.name):
            position = message.position[index]
            velocity = message.velocity[index] if index < len(message.velocity) else 0.0
            effort = message.effort[index] if index < len(message.effort) else 0.0
            if name == "gripper":
                opening = abs(position)
                state.name.extend(("joint7", "joint8"))
                state.position.extend((opening, -opening))
                state.velocity.extend((velocity, -velocity))
                state.effort.extend((effort, effort))
            else:
                state.name.append(name)
                state.position.append(position)
                state.velocity.append(velocity)
                state.effort.append(effort)
        self.publisher.publish(state)


def main() -> int:
    rospy.init_node("piper_joint_state_relay")
    PiperJointStateRelay()
    rospy.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
