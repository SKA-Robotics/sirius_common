#!/usr/bin/env python3

import struct

import rclpy
from rclpy.node import Node
from can_msgs.msg import Frame
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32


def _float_to_bits(value: float) -> int:
    return struct.unpack('I', struct.pack('f', value))[0]


def _bits_to_float(value: int) -> float:
    return struct.unpack('f', struct.pack('I', value))[0]


class GripperNode(Node):

    def __init__(self) -> None:
        super().__init__('gripper_node')

        self.device_id = 0x2c
        self.command_arb_id = 0x00
        self.report_arb_id = 0x01

        self._status_pub = self.create_publisher(JointState, '/manip/joint_states', 10)
        self._send_pub = self.create_publisher(Frame, '/sent_canbus_messages', 10)

        self.create_subscription(Float32, '/gripper_cmd', self._command_callback, 10)
        self.create_subscription(Frame, '/received_canbus_messages',
                                 self._receive_raw_frame, 10)

    def _send_frame(self, command_id: int, data: bytes):
        frame = Frame()
        frame.id = (self.device_id << 5) | command_id
        frame.dlc = len(data)
        frame.data = [data[i] if i < len(data) else 0 for i in range(8)]
        self._send_pub.publish(frame)

    def _receive_raw_frame(self, frame: Frame):
        device_id = frame.id >> 5
        if device_id == self.device_id:
            command_id = frame.id & 0b11111
            self._receive_frame(command_id, frame.data, frame)

    def _receive_frame(self, command_id: int, data, frame: Frame):
        if command_id == self.report_arb_id:
            raw = int.from_bytes(bytes(data), 'big')
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ['gripper']
            msg.position = [_bits_to_float(raw)]
            self._status_pub.publish(msg)

    def _command_callback(self, msg: Float32):
        value = _float_to_bits(-msg.data)
        data = value.to_bytes(8, byteorder='big')
        self._send_frame(self.command_arb_id, data)


def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
