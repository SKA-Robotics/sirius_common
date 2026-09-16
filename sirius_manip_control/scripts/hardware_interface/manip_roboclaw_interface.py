#!/usr/bin/env python3

import yaml
from functools import partial

import rclpy
from sensor_msgs.msg import JointState
from sirius_msgs.msg import JointState as SiriusJointState

from hardware_interface.manip_hardware_interface import ManipHardwareInterface


class ManipRoboclawInterface(ManipHardwareInterface):

    def __init__(self):
        super().__init__('manip_roboclaw_interface')

    def _initialize_hardware_connection(self):
        self.declare_parameter('command_topics', rclpy.Parameter.Type.STRING)
        self.declare_parameter('feedback_topics', rclpy.Parameter.Type.STRING)
        self.declare_parameter('rate', 10.0)

        def _load(name):
            raw = self.get_parameter(name).value
            return yaml.safe_load(raw) if isinstance(raw, str) else (raw or {})

        self.command_topics = _load('command_topics')
        self.feedback_topics = _load('feedback_topics')
        self._rate = self.get_parameter('rate').value
        self.joint_positions = {k: 0.0 for k in self.feedback_topics}

        self._feedback_subs = {}
        for joint, topic in self.feedback_topics.items():
            self._feedback_subs[joint] = self.create_subscription(
                SiriusJointState, topic,
                partial(self._process_hardware_state, joint_name=joint), 10)
            self.get_logger().info(f'Subscriber: {joint} -> {topic}')

        self._command_publishers = {}
        for joint, topic in self.command_topics.items():
            self._command_publishers[joint] = self.create_publisher(
                SiriusJointState, topic, 10)
            self.get_logger().info(f'Publisher: {joint} -> {topic}')

        self.create_timer(1.0 / self._rate, self._publish_hardware_state)

    def _process_hardware_state(self, state: SiriusJointState,
                                joint_name: str = ''):
        if state.position:
            self.joint_positions[joint_name] = state.position[0]

    def _publish_hardware_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for joint, pos in self.joint_positions.items():
            msg.name.append(joint)
            msg.position.append(pos)
        self._publish_state(msg)

    def _send_hardware_command(self, joint_state: JointState):
        stamp = self.get_clock().now().to_msg()
        for i, name in enumerate(joint_state.name):
            if name not in self.command_topics:
                self.get_logger().warn(f'Unsupported joint: {name}')
                continue
            if name not in self._command_publishers:
                continue

            msg = SiriusJointState()
            msg.header.stamp = stamp
            if i < len(joint_state.position):
                msg.position = [joint_state.position[i]]
                if name == 'arm3_tool':
                    continue
            if i < len(joint_state.velocity):
                msg.velocity = [joint_state.velocity[i]]
            if i < len(joint_state.effort):
                msg.duty = [joint_state.effort[i]]
            self._command_publishers[name].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ManipRoboclawInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
