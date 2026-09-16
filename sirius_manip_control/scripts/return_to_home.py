#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sirius_msgs.msg import JointState

TOPICS = {
    'base_cyl': 'roboclaw_driver/128/motor1/set_joint_state',
    'cyl_arm1': 'roboclaw_driver/129/motor1/set_joint_state',
    'arm1_arm2': 'roboclaw_driver/129/motor2/set_joint_state',
    'arm2_arm3': 'roboclaw_driver/130/motor1/set_joint_state',
    'arm3_tool': 'roboclaw_driver/130/motor2/set_joint_state',
}


class ReturnToHomeNode(Node):

    def __init__(self):
        super().__init__('return_to_home')
        self._publishers = [
            self.create_publisher(JointState, topic, 10)
            for topic in TOPICS.values()
        ]
        self._zero_message = JointState()
        self._zero_message.position = [0.0]
        self._phase = 0

        self.create_timer(0.1, self._step)
        # Switch to publishing all joints after 8 seconds
        self.create_timer(8.0, self._switch_phase)
        self.get_logger().info('Return to home procedure started...')

    def _switch_phase(self):
        self._phase = 1

    def _step(self):
        self._zero_message.header.stamp = self.get_clock().now().to_msg()
        if self._phase == 0:
            self._publishers[0].publish(self._zero_message)
        else:
            for pub in self._publishers:
                pub.publish(self._zero_message)


def main(args=None):
    rclpy.init(args=args)
    node = ReturnToHomeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Done. Back home.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
