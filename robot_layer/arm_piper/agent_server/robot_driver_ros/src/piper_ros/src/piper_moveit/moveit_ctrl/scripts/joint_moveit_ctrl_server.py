#!/usr/bin/env python

import rospy
from moveit_commander import *
from moveit_ctrl.srv import JointMoveitCtrl, JointMoveitCtrlResponse
from geometry_msgs.msg import Pose

class JointMoveitCtrlServer:
    ERROR_INVALID_REQUEST = 1
    ERROR_PLANNING_FAILURE = 2
    ERROR_EXCEPTION = 3
    ERROR_EMPTY_TRAJECTORY = 4
    ERROR_EXECUTION_FAILURE = 5

    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('joint_moveit_ctrl_server')

        # 初始化 MoveIt
        roscpp_initialize([])
        self.robot = RobotCommander()

        # 获取 MoveIt 规划组列表
        available_groups = self.robot.get_group_names()
        rospy.loginfo(f"Available MoveIt groups: {available_groups}")

        # 仅实例化存在的规划组
        self.arm_move_group = None
        self.gripper_move_group = None
        self.piper_move_group = None

        if "arm" in available_groups:
            self.arm_move_group = MoveGroupCommander("arm")
            rospy.loginfo("Initialized arm move group.")
        
        if "gripper" in available_groups:
            self.gripper_move_group = MoveGroupCommander("gripper")
            rospy.loginfo("Initialized gripper move group.")
        
        if "piper" in available_groups:
            self.piper_move_group = MoveGroupCommander("piper")
            rospy.loginfo("Initialized piper move group.")

        # 创建关节运动控制服务
        self.arm_srv = rospy.Service('joint_moveit_ctrl_arm', JointMoveitCtrl, self.handle_joint_moveit_ctrl_arm)
        self.gripper_srv = rospy.Service('joint_moveit_ctrl_gripper', JointMoveitCtrl, self.handle_joint_moveit_ctrl_gripper)
        self.piper_srv = rospy.Service('joint_moveit_ctrl_piper', JointMoveitCtrl, self.handle_joint_moveit_ctrl_piper)
        self.endpose_srv = rospy.Service('joint_moveit_ctrl_endpose', JointMoveitCtrl, self.handle_joint_moveit_ctrl_endpose)

        rospy.loginfo("Joint MoveIt Control Services Ready.")

    @staticmethod
    def _clamp_scaling(value):
        return max(1e-6, min(1 - 1e-6, value))

    @staticmethod
    def _extract_plan(plan_result):
        if isinstance(plan_result, tuple):
            success, trajectory, planning_time, error_code = plan_result
            return bool(success), trajectory, planning_time, getattr(error_code, "val", error_code)
        trajectory = plan_result
        points = len(trajectory.joint_trajectory.points) if trajectory is not None else 0
        return points > 0, trajectory, None, None

    @staticmethod
    def _trajectory_point_count(trajectory):
        if trajectory is None:
            return 0
        joint_trajectory = getattr(trajectory, "joint_trajectory", None)
        points = getattr(joint_trajectory, "points", None)
        return len(points) if points is not None else 0

    def _build_target_pose(self, joint_endpose):
        if len(joint_endpose) != 7:
            raise ValueError("Invalid joint_endpose size. It must be 7 (Quaternion).")

        position = joint_endpose[:3]
        quaternion = joint_endpose[3:]
        target_pose = Pose()
        target_pose.position.x = position[0]
        target_pose.position.y = position[1]
        target_pose.position.z = position[2]
        target_pose.orientation.x = quaternion[0]
        target_pose.orientation.y = quaternion[1]
        target_pose.orientation.z = quaternion[2]
        target_pose.orientation.w = quaternion[3]
        return target_pose

    def _plan_endpose_for_tcp(self, request):
        if not self.arm_move_group:
            rospy.logerr("Arm move group is not initialized.")
            return None, JointMoveitCtrlResponse(status=False, error_code=self.ERROR_INVALID_REQUEST)

        target_pose = self._build_target_pose(request.joint_endpose)
        group = self.arm_move_group
        planning_frame = group.get_planning_frame()
        end_effector_link = "gripper_tcp"

        group.set_start_state_to_current_state()
        group.set_pose_reference_frame(planning_frame)
        group.set_end_effector_link(end_effector_link)
        group.set_pose_target(target_pose, end_effector_link)

        max_velocity = self._clamp_scaling(request.max_velocity)
        max_acceleration = self._clamp_scaling(request.max_acceleration)
        group.set_max_velocity_scaling_factor(max_velocity)
        group.set_max_acceleration_scaling_factor(max_acceleration)

        rospy.loginfo(
            "Endpose request move_group=%s planning_frame=%s end_effector_link=%s target_position=(%.6f, %.6f, %.6f) target_quaternion=(%.6f, %.6f, %.6f, %.6f) max_velocity=%.6f max_acceleration=%.6f",
            "arm",
            planning_frame,
            end_effector_link,
            target_pose.position.x,
            target_pose.position.y,
            target_pose.position.z,
            target_pose.orientation.x,
            target_pose.orientation.y,
            target_pose.orientation.z,
            target_pose.orientation.w,
            max_velocity,
            max_acceleration,
        )

        plan_result = group.plan()
        planning_success, trajectory, planning_time, error_code = self._extract_plan(plan_result)
        trajectory_points = self._trajectory_point_count(trajectory)

        rospy.loginfo(
            "Endpose planning result move_group=%s planning_success=%s moveit_error_code=%s trajectory_points=%d planning_time=%s",
            "arm",
            planning_success,
            error_code,
            trajectory_points,
            planning_time,
        )

        if not planning_success:
            group.clear_pose_targets()
            return None, JointMoveitCtrlResponse(status=False, error_code=self.ERROR_PLANNING_FAILURE)

        if trajectory_points <= 0:
            group.clear_pose_targets()
            return None, JointMoveitCtrlResponse(status=False, error_code=self.ERROR_EMPTY_TRAJECTORY)

        return {
            "group": group,
            "trajectory": trajectory,
            "target_pose": target_pose,
            "planning_frame": planning_frame,
            "end_effector_link": end_effector_link,
            "trajectory_points": trajectory_points,
        }, None

    def handle_joint_moveit_ctrl_arm(self, request):
        rospy.loginfo("Received arm joint movement request.")

        try:
            if self.arm_move_group:
                arm_joint_goal = request.joint_states[:6]
                self.arm_move_group.set_joint_value_target(arm_joint_goal)
                max_velocity = self._clamp_scaling(request.max_velocity)
                max_acceleration = self._clamp_scaling(request.max_acceleration)
                self.arm_move_group.set_max_velocity_scaling_factor(max_velocity)
                self.arm_move_group.set_max_acceleration_scaling_factor(max_acceleration)
                rospy.loginfo(f"max_velocity: {max_velocity} max_acceleration: {max_acceleration}")
                success = bool(self.arm_move_group.go(wait=True))
                if success:
                    rospy.loginfo("Arm movement executed successfully.")
                    return JointMoveitCtrlResponse(status=True, error_code=0)
                rospy.logerr("Arm movement was not planned or executed.")
                return JointMoveitCtrlResponse(status=False, error_code=2)
            else:
                rospy.logerr("Arm move group is not initialized.")
                return JointMoveitCtrlResponse(status=False, error_code=1)
        except Exception as e:
            rospy.logerr(f"Exception during arm movement: {str(e)}")
            return JointMoveitCtrlResponse(status=False, error_code=3)

    def handle_joint_moveit_ctrl_gripper(self, request):
        rospy.loginfo("Received gripper joint movement request.")

        try:
            if self.gripper_move_group:
                gripper_goal = [request.gripper]
                self.gripper_move_group.set_joint_value_target(gripper_goal)
                success = bool(self.gripper_move_group.go(wait=True))
                if success:
                    rospy.loginfo("Gripper movement executed successfully.")
                    return JointMoveitCtrlResponse(status=True, error_code=0)
                rospy.logerr("Gripper movement was not planned or executed.")
                return JointMoveitCtrlResponse(status=False, error_code=2)
            else:
                rospy.logerr("Gripper move group is not initialized.")
                return JointMoveitCtrlResponse(status=False, error_code=1)
        except Exception as e:
            rospy.logerr(f"Exception during gripper movement: {str(e)}")
            return JointMoveitCtrlResponse(status=False, error_code=3)

    def handle_joint_moveit_ctrl_piper(self, request):
        rospy.loginfo("Received piper joint movement request.")

        try:
            if self.piper_move_group:
                piper_joint_goal = list(request.joint_states[:6]) + [request.gripper]
                self.piper_move_group.set_joint_value_target(piper_joint_goal)
                max_velocity = self._clamp_scaling(request.max_velocity)
                max_acceleration = self._clamp_scaling(request.max_acceleration)
                self.piper_move_group.set_max_velocity_scaling_factor(max_velocity)
                self.piper_move_group.set_max_acceleration_scaling_factor(max_acceleration)
                rospy.loginfo(f"max_velocity: {max_velocity} max_acceleration: {max_acceleration}")
                success = bool(self.piper_move_group.go(wait=True))
                if success:
                    rospy.loginfo("Piper movement executed successfully.")
                    return JointMoveitCtrlResponse(status=True, error_code=0)
                rospy.logerr("Piper movement was not planned or executed.")
                return JointMoveitCtrlResponse(status=False, error_code=2)
            else:
                rospy.logerr("Piper move group is not initialized.")
                return JointMoveitCtrlResponse(status=False, error_code=1)
        except Exception as e:
            rospy.logerr(f"Exception during piper movement: {str(e)}")
            return JointMoveitCtrlResponse(status=False, error_code=3)

    def handle_joint_moveit_ctrl_endpose(self, request):
        rospy.loginfo("Received endpose movement request.")

        try:
            plan_info, error_response = self._plan_endpose_for_tcp(request)
            if error_response is not None:
                return error_response

            group = plan_info["group"]
            trajectory = plan_info["trajectory"]
            execution_success = bool(group.execute(trajectory, wait=True))
            group.stop()
            group.clear_pose_targets()
            rospy.loginfo(
                "Endpose execution result move_group=%s end_effector_link=%s execution_success=%s trajectory_points=%d",
                "arm",
                plan_info["end_effector_link"],
                execution_success,
                plan_info["trajectory_points"],
            )
            if execution_success:
                return JointMoveitCtrlResponse(status=True, error_code=0)
            return JointMoveitCtrlResponse(status=False, error_code=self.ERROR_EXECUTION_FAILURE)
        except ValueError as e:
            rospy.logerr(f"Invalid endpose request: {str(e)}")
            return JointMoveitCtrlResponse(status=False, error_code=self.ERROR_INVALID_REQUEST)
        except Exception as e:
            rospy.logerr(f"Exception during endpose movement: {str(e)}")
            return JointMoveitCtrlResponse(status=False, error_code=self.ERROR_EXCEPTION)

if __name__ == '__main__':
    JointMoveitCtrlServer()
    rospy.spin()
