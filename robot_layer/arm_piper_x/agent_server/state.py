"""ROS 2 state monitor for the PiPER-X Agent Server."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Any, Optional

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
except Exception:  # pragma: no cover - exercised when ROS 2 is unavailable.
    rclpy = None
    PoseStamped = None
    MultiThreadedExecutor = None
    Node = object
    JointState = None


def _stamp_to_seconds(msg: Any) -> Optional[float]:
    stamp = getattr(getattr(msg, "header", None), "stamp", None)
    if stamp is None:
        return None
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


@dataclass
class TopicSnapshot:
    names_and_types: list[tuple[str, list[str]]]
    services_and_types: list[tuple[str, list[str]]]
    actions: list[str]


class PiperXStateMonitor:
    def __init__(
        self,
        joint_state_topic: str,
        tcp_pose_topic: str,
        trajectory_action: str,
        marker_task_service: str,
        expected_joints: list[str],
        timeout_s: float,
    ):
        self.joint_state_topic = joint_state_topic
        self.tcp_pose_topic = tcp_pose_topic
        self.trajectory_action = trajectory_action
        self.marker_task_service = marker_task_service
        self.expected_joints = list(expected_joints)
        self.timeout_s = float(timeout_s)
        self._lock = threading.Lock()
        self._latest_joint_state: dict[str, Any] | None = None
        self._latest_tcp_pose: dict[str, Any] | None = None
        self._joint_received_monotonic_s: float | None = None
        self._tcp_received_monotonic_s: float | None = None
        self._node: Node | None = None
        self._executor: MultiThreadedExecutor | None = None
        self._thread: threading.Thread | None = None
        self._startup_error: str | None = None

    def start(self) -> None:
        if rclpy is None:
            self._startup_error = "rclpy is not importable; source /opt/ros/jazzy/setup.bash"
            return
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
            self._node = rclpy.create_node("piper_x_agent_state_monitor")
            self._node.create_subscription(JointState, self.joint_state_topic, self._joint_callback, 10)
            self._node.create_subscription(PoseStamped, self.tcp_pose_topic, self._tcp_callback, 10)
            self._executor = MultiThreadedExecutor()
            self._executor.add_node(self._node)
            self._thread = threading.Thread(target=self._executor.spin, daemon=True)
            self._thread.start()
        except Exception as exc:
            self._startup_error = str(exc)

    def stop(self) -> None:
        if self._executor is not None:
            self._executor.shutdown()
        if self._node is not None:
            self._node.destroy_node()
        if rclpy is not None and rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

    def _joint_callback(self, msg: JointState) -> None:
        positions = {name: float(value) for name, value in zip(msg.name, msg.position)}
        velocities = {name: float(value) for name, value in zip(msg.name, msg.velocity)}
        with self._lock:
            self._latest_joint_state = {
                "topic": self.joint_state_topic,
                "header_stamp_s": _stamp_to_seconds(msg),
                "names": list(msg.name),
                "positions_rad": positions,
                "velocities_rad_s": velocities,
            }
            self._joint_received_monotonic_s = time.monotonic()

    def _tcp_callback(self, msg: PoseStamped) -> None:
        pose = msg.pose
        with self._lock:
            self._latest_tcp_pose = {
                "topic": self.tcp_pose_topic,
                "frame_id": msg.header.frame_id,
                "header_stamp_s": _stamp_to_seconds(msg),
                "position_m": {
                    "x": float(pose.position.x),
                    "y": float(pose.position.y),
                    "z": float(pose.position.z),
                },
                "orientation_xyzw": {
                    "x": float(pose.orientation.x),
                    "y": float(pose.orientation.y),
                    "z": float(pose.orientation.z),
                    "w": float(pose.orientation.w),
                },
            }
            self._tcp_received_monotonic_s = time.monotonic()

    def _graph_snapshot(self) -> TopicSnapshot:
        if self._node is None:
            return TopicSnapshot([], [], [])
        names_and_types = self._node.get_topic_names_and_types()
        services_and_types = self._node.get_service_names_and_types()
        actions = [
            name.rsplit("/_action/", 1)[0]
            for name, _types in services_and_types
            if name.endswith("/_action/send_goal")
        ]
        return TopicSnapshot(names_and_types, services_and_types, sorted(set(actions)))

    def state(self) -> dict:
        now = time.monotonic()
        with self._lock:
            joint = dict(self._latest_joint_state) if self._latest_joint_state else None
            tcp = dict(self._latest_tcp_pose) if self._latest_tcp_pose else None
            joint_age = None if self._joint_received_monotonic_s is None else now - self._joint_received_monotonic_s
            tcp_age = None if self._tcp_received_monotonic_s is None else now - self._tcp_received_monotonic_s
        missing = list(self.expected_joints)
        if joint is not None:
            missing = [name for name in self.expected_joints if name not in joint.get("positions_rad", {})]
        return {
            "ros2_monitor_running": self._node is not None,
            "startup_error": self._startup_error,
            "joint_state": joint,
            "joint_state_age_s": joint_age,
            "joint_state_fresh": joint_age is not None and joint_age <= self.timeout_s and not missing,
            "missing_joints": missing,
            "tcp_pose": tcp,
            "tcp_pose_age_s": tcp_age,
            "tcp_pose_fresh": tcp_age is not None and tcp_age <= self.timeout_s,
        }

    def graph(self) -> dict:
        snapshot = self._graph_snapshot()
        interesting = ("gripper", "hand", "control", "feedback")
        topics = [
            {"name": name, "types": types}
            for name, types in snapshot.names_and_types
            if any(token in name.lower() for token in interesting)
        ]
        services = [
            {"name": name, "types": types}
            for name, types in snapshot.services_and_types
            if any(token in name.lower() for token in interesting)
        ]
        return {
            "topics": sorted(topics, key=lambda item: item["name"]),
            "services": sorted(services, key=lambda item: item["name"]),
            "actions": snapshot.actions,
            "trajectory_action_available": self.trajectory_action in snapshot.actions,
            "marker_task_service": self.marker_task_service,
            "marker_task_service_seen": any(name == self.marker_task_service for name, _ in snapshot.services_and_types),
        }
