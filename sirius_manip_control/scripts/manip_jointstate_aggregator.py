#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from sirius_msgs.msg import JointState as SiriusJointState


class ManipJointstateAggregatorNode(Node):

    def __init__(self):
        super().__init__('manip_jointstate_aggregator')

        self.declare_parameter('motor_topics', rclpy.Parameter.Type.STRING)
        self.declare_parameter('jointstate_topic', '/manip_interface/state')
        self.declare_parameter('publish_rate', 10.0)

        import yaml
        raw = self.get_parameter('motor_topics').value
        if isinstance(raw, str):
            motor_topics = yaml.safe_load(raw) or {}
        else:
            motor_topics = raw or {}

        output_topic = self.get_parameter('jointstate_topic').value
        rate = self.get_parameter('publish_rate').value

        self._joint_values: dict = {}
        self._publisher = self.create_publisher(JointState, output_topic, 10)

        for joint, motor_topic in motor_topics.items():
            self.create_subscription(
                SiriusJointState, motor_topic,
                self._make_callback(joint), 10)

        self.create_timer(1.0 / rate, self._publish_joints)

    def _make_callback(self, joint_name: str):
        def _cb(msg: SiriusJointState):
            if msg.position:
                self._joint_values[joint_name] = msg.position[0]
        return _cb

    def _publish_joints(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for name, value in self._joint_values.items():
            msg.name.append(name)
            msg.position.append(value)
        self._publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ManipJointstateAggregatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
