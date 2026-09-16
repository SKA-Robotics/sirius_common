#!/usr/bin/env python3
"""Mock manipulator node — republishes set_joint_states as joint_states."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

DEFAULT_JOINT_STATE = JointState()
DEFAULT_JOINT_STATE.name = [f'joint{i}' for i in range(1, 7)]
DEFAULT_JOINT_STATE.position = [0.0] * 6


class MockManipulatorNode(Node):

    def __init__(self):
        super().__init__('mock_manip_node')

        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('set_joint_states_topic',
                               '/manip/set_joint_states')
        self.declare_parameter('joint_states_topic', '/manip/joint_states')

        rate = self.get_parameter('publish_rate').value
        set_topic = self.get_parameter('set_joint_states_topic').value
        state_topic = self.get_parameter('joint_states_topic').value

        self._latest = DEFAULT_JOINT_STATE

        self._pub = self.create_publisher(JointState, state_topic, 10)
        self.create_subscription(JointState, set_topic,
                                 self._set_callback, 10)
        self.create_timer(1.0 / rate, self._publish_callback)

        self.get_logger().info(f'Publishing at {rate} Hz on {state_topic}')

    def _set_callback(self, msg: JointState):
        self._latest = msg

    def _publish_callback(self):
        msg = self._latest
        msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockManipulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('MockManipulatorNode shutting down.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
