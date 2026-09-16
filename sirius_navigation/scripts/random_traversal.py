#!/usr/bin/env python3

import select
import sys
from random import uniform

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class RandomTraversalNode(Node):

    def __init__(self):
        super().__init__('random_traversal')

        self.declare_parameter('max_linear_velocity', 1.0)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('max_period', 10.0)

        self.max_linear_velocity = self.get_parameter(
            'max_linear_velocity').value
        self.max_angular_velocity = self.get_parameter(
            'max_angular_velocity').value
        self.max_period = self.get_parameter('max_period').value

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.msg = Twist()

        self._step_timer = None
        self._publish_timer = self.create_timer(0.05, self._publish_callback)
        self._step()

        self.get_logger().info('Press Enter to reverse direction')

    def _step(self):
        self.msg.linear.x = uniform(-self.max_linear_velocity,
                                    self.max_linear_velocity)
        self.msg.angular.z = uniform(-self.max_angular_velocity,
                                     self.max_angular_velocity)
        period = uniform(0.1, self.max_period)

        if self._step_timer is not None:
            self._step_timer.cancel()
        self._step_timer = self.create_timer(period, self._on_period_elapsed)

    def _on_period_elapsed(self):
        if self._step_timer is not None:
            self._step_timer.cancel()
        self._step()

    def _publish_callback(self):
        # Non-blocking stdin check for direction reversal
        if select.select([sys.stdin], [], [], 0.0)[0]:
            sys.stdin.readline()
            self.msg.linear.x = -self.msg.linear.x
            self.msg.angular.z = -self.msg.angular.z
        self.publisher.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)
    node = RandomTraversalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
