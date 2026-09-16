#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from manip.arm_servo.arm_controller import main
from manip.manip_config import load_ros_params


class ArmControllerNode(Node):
    def __init__(self):
        super().__init__('arm_controller')


def entry():
    rclpy.init()
    node = ArmControllerNode()
    config = load_ros_params(node)
    try:
        main(config, node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    entry()
