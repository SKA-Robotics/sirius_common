#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from sirius_msgs.msg import JointState as SiriusJointState

VOODOO_TOPIC = 'voodoo_node/joint_states'
SIRIUS_TOPICS = {
    'base_cyl': '/roboclaw_driver/128/motor1/set_joint_state',
    'cyl_arm1': '/roboclaw_driver/129/motor1/set_joint_state',
    'arm1_arm2': '/roboclaw_driver/129/motor2/set_joint_state',
    'arm2_arm3': '/roboclaw_driver/130/motor1/set_joint_state',
    'arm3_tool': '/roboclaw_driver/130/motor2/set_joint_state',
    'gripper': 'gripper',
}


class VoodooDemultiplexerNode(Node):

    def __init__(self):
        super().__init__('voodoo_demultiplexer')
        self.publishers = {
            joint: self.create_publisher(SiriusJointState, topic, 10)
            for joint, topic in SIRIUS_TOPICS.items()
        }
        self.create_subscription(JointState, VOODOO_TOPIC, self._callback, 10)

    def _callback(self, message: JointState):
        stamp = self.get_clock().now().to_msg()
        for i, name in enumerate(message.name):
            if name not in self.publishers:
                continue
            msg = SiriusJointState()
            msg.header.stamp = stamp
            msg.position = [message.position[i]]
            self.publishers[name].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VoodooDemultiplexerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
