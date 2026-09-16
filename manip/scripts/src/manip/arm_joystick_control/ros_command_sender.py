from typing import List

from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32


class RosCommandSender:

    def __init__(self, twist_topic: str, joint_topic: str,
                 gripper_cmd_topic: str, preset_publisher_topic: str,
                 node):
        self._node = node
        self.twist_publisher = node.create_publisher(
            TwistStamped, twist_topic, 10)
        self.joint_publisher = node.create_publisher(JointJog, joint_topic, 10)
        self.gripper_publisher = node.create_publisher(
            Float32, gripper_cmd_topic, 10)
        self.preset_publisher = node.create_publisher(
            JointState, preset_publisher_topic, 10)

    def send_twist_command(self, twist_data: List[float], frame_id: str):
        cmd = TwistStamped()
        cmd.header.stamp = self._node.get_clock().now().to_msg()
        cmd.header.frame_id = frame_id
        cmd.twist.linear.x = twist_data[0]
        cmd.twist.linear.y = twist_data[1]
        cmd.twist.linear.z = twist_data[2]
        cmd.twist.angular.x = twist_data[3]
        cmd.twist.angular.y = twist_data[4]
        cmd.twist.angular.z = twist_data[5]
        self.twist_publisher.publish(cmd)

    def send_joint_command(self, joint_data: List[float]):
        cmd = JointJog()
        cmd.header.stamp = self._node.get_clock().now().to_msg()
        cmd.velocities = joint_data
        self.joint_publisher.publish(cmd)

    def send_gripper_command(self, gripper_cmd: float):
        self.gripper_publisher.publish(Float32(data=gripper_cmd))

    def send_preset_command(self, target_q: List[float]):
        cmd = JointState()
        cmd.position = target_q
        self.preset_publisher.publish(cmd)
