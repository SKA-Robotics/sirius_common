#!/usr/bin/env python3

import yaml
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from hardware_interface.manip_hardware_interface import ManipHardwareInterface


class ManipGazeboInterface(ManipHardwareInterface):

    def __init__(self):
        super().__init__('manip_gazebo_interface')

    def _initialize_hardware_connection(self):
        self.declare_parameter('command_topics', rclpy.Parameter.Type.STRING)
        self.declare_parameter('feedback_topic', '/manipulator/joint_states')

        raw = self.get_parameter('command_topics').value
        self.command_topics = (yaml.safe_load(raw) if isinstance(raw, str)
                               else (raw or {}))
        feedback_topic = self.get_parameter('feedback_topic').value

        self.create_subscription(JointState, feedback_topic,
                                 self._publish_state, 10)
        self.get_logger().info(f'Subscriber: {feedback_topic}')

        self._command_publishers = {}
        for joint, topic in self.command_topics.items():
            self._command_publishers[joint] = self.create_publisher(
                Float64, topic, 10)
            self.get_logger().info(f'Publisher: {joint} -> {topic}')

    def _send_hardware_command(self, joint_state: JointState):
        for i, pos in enumerate(joint_state.position):
            name = joint_state.name[i]
            if name not in self.command_topics:
                self.get_logger().warn(f'Unsupported joint: {name}')
                continue
            msg = Float64()
            msg.data = pos
            self._command_publishers[name].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ManipGazeboInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
