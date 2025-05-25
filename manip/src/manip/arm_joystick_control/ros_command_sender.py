from typing import List
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import rospy

class RosCommandSender:
    def __init__(self, twist_topic: str, joint_topic: str, gripper_cmd_topic: str, preset_publisher_topic: str):
        self.twist_publisher = rospy.Publisher(twist_topic, TwistStamped, queue_size=10)
        self.joint_publisher = rospy.Publisher(joint_topic, JointJog, queue_size=10)
        self.gripper_publisher = rospy.Publisher(gripper_cmd_topic, Float64, queue_size=10)
        self.preset_publisher = rospy.Publisher(preset_publisher_topic, JointState, queue_size=10)

    def send_twist_command(self, twist_data: List[float], frame_id: str):
        command = TwistStamped()
        command.header.stamp = rospy.Time.now()
        command.header.frame_id = frame_id
        command.twist.linear.x = twist_data[0]
        command.twist.linear.y = twist_data[1]
        command.twist.linear.z = twist_data[2]
        command.twist.angular.x = twist_data[3]
        command.twist.angular.y = twist_data[4]
        command.twist.angular.z = twist_data[5]
        self.twist_publisher.publish(command)

    def send_joint_command(self, joint_data: List[float]):
        command = JointJog()
        command.header.stamp = rospy.Time.now()
        command.velocities = joint_data
        self.joint_publisher.publish(command)

    def send_gripper_command(self, gripper_position: float):
        command = Float64()
        command.data = gripper_position
        self.gripper_publisher.publish(command)
    
    def send_preset_command(self, target_q: List[float]):
        command = JointState()
        command.position = target_q
        self.preset_publisher.publish(command)
