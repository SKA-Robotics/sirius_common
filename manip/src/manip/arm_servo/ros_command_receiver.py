from geometry_msgs.msg import TwistStamped, PoseStamped
from control_msgs.msg import JointJog
from sensor_msgs.msg import JointState
from manip.arm_servo.command import Command, CommandType

import rospy
from typing import Optional
import numpy as np
import spatialmath as sm
import time
import threading


class RosCommandReceiver:

    def __init__(self, twist_topic: str, pose_topic: str, joint_topic: str, preset_request_topic: str):
        self.command_lock = threading.Lock()
        self.command = None
        rospy.Subscriber(twist_topic, TwistStamped, self.twist_callback, queue_size=10)
        rospy.Subscriber(pose_topic, PoseStamped, self.pose_callback, queue_size=10)
        rospy.Subscriber(joint_topic, JointJog, self.joint_callback, queue_size=10)
        rospy.Subscriber(preset_request_topic, JointState, self.preset_request_callback, queue_size=10)

    def twist_callback(self, twist: TwistStamped):
        # TODO: Recalculate twist to correct frame
        twist_data = np.array([
            twist.twist.linear.x,
            twist.twist.linear.y,
            twist.twist.linear.z,
            twist.twist.angular.x,
            twist.twist.angular.y,
            twist.twist.angular.z])
        with self.command_lock:
            self.command = Command(CommandType.END_EFFECTOR_TWIST_CMD, twist_data, time.time())

    def pose_callback(self, pose: PoseStamped):
        # TODO: Recalculate pose to correct frame
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
            self.command = Command(CommandType.END_EFFECTOR_POSE_CMD, pose_matrix, time.time())

    def joint_callback(self, joint_jog: JointJog):
        with self.command_lock:
            self.command = Command(CommandType.JOINT_VELOCITY_CMD, np.array(joint_jog.velocities), time.time())
    
    def preset_request_callback(self, jointstate: JointState):
        with self.command_lock:
            self.command = Command(CommandType.TRAJECTORY_CMD, np.array(jointstate.position), time.time())

    def pop_command(self) -> Optional[Command]:
        """ 
        Returns the latest command and removes it from the queue.
        """
        with self.command_lock:
            command = self.command
            self.command = None
            return command

    def get_command(self) -> Optional[Command]:
        """ 
        Returns the latest command without removing it from the queue.
        """
        with self.command_lock:
            return self.command
    
