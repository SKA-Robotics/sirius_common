import threading
import time
from typing import Optional

import numpy as np
import roboticstoolbox as rtb
import spatialmath as sm

from geometry_msgs.msg import TwistStamped, PoseStamped
from control_msgs.msg import JointJog
from sensor_msgs.msg import JointState

from manip.manip_config import ManipConfig
from manip.arm_servo.command import Command, CommandType


class RosCommandReceiver:

    def __init__(self, config: ManipConfig, robot_model: rtb.ERobot, node):
        self.ee_frame = config.ee_frame_id
        self.robot_model = robot_model
        self.command_lock = threading.Lock()
        self.command = None

        node.create_subscription(TwistStamped, config.twist_topic,
                                 self.twist_callback, 10)
        node.create_subscription(PoseStamped, config.pose_topic,
                                 self.pose_callback, 10)
        node.create_subscription(JointJog, config.joint_topic,
                                 self.joint_callback, 10)
        node.create_subscription(JointState, config.preset_request_topic,
                                 self.preset_request_callback, 10)

    def twist_callback(self, twist: TwistStamped):
        cmd_frame = twist.header.frame_id
        linear = np.array([
            twist.twist.linear.x, twist.twist.linear.y,
            twist.twist.linear.z, 0.0])
        angular = np.array([
            twist.twist.angular.x, twist.twist.angular.y,
            twist.twist.angular.z, 0.0])

        if cmd_frame != self.ee_frame:
            T = self.robot_model.fkine(
                self.robot_model.q, start=cmd_frame, end=self.ee_frame)
            T_inv = np.linalg.inv(T)
            linear = T_inv @ linear
            angular = T_inv @ angular

        twist_data = np.array([
            linear[0], linear[1], linear[2],
            angular[0], angular[1], angular[2]])
        with self.command_lock:
            self.command = Command(
                CommandType.END_EFFECTOR_TWIST_CMD, twist_data, time.time())

    def pose_callback(self, pose: PoseStamped):
        rotation = sm.Quaternion([
            pose.pose.orientation.w,
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z])
        translation = sm.SE3.Trans(
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z)
        pose_matrix = translation.A @ rotation.matrix
        with self.command_lock:
            self.command = Command(
                CommandType.END_EFFECTOR_POSE_CMD, pose_matrix, time.time())

    def joint_callback(self, joint_jog: JointJog):
        with self.command_lock:
            self.command = Command(
                CommandType.JOINT_VELOCITY_CMD,
                np.array(joint_jog.velocities), time.time())

    def preset_request_callback(self, jointstate: JointState):
        with self.command_lock:
            self.command = Command(
                CommandType.TRAJECTORY_CMD,
                np.array(jointstate.position), time.time())

    def pop_command(self) -> Optional[Command]:
        with self.command_lock:
            cmd = self.command
            self.command = None
            return cmd

    def get_command(self) -> Optional[Command]:
        with self.command_lock:
            return self.command
