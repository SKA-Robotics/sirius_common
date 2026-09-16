#!/usr/bin/env python3

import serial
import traceback
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


class GripperArduinoNode(Node):

    def __init__(self):
        super().__init__('gripper')

        self.declare_parameter('port_name', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 9600)

        portname = self.get_parameter('port_name').value
        baudrate = self.get_parameter('baudrate').value

        self.angle = 0.5
        self.speed = 0.03

        try:
            self._serial = serial.Serial(portname, baudrate)
        except serial.SerialException:
            self.get_logger().error(
                f'Cannot open serial port: {traceback.format_exc()}')
            raise

        self.create_subscription(Joy, '/joy', self._joy_callback, 10)

        # Thread to drain serial RX
        self._thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._thread.start()

    def _rx_loop(self):
        while rclpy.ok():
            if self._serial.in_waiting:
                print(self._serial.readline())

    def _joy_callback(self, msg: Joy):
        self.angle = clamp(self.angle + msg.axes[-1] * self.speed, -1, 1)
        command = int((self.angle + 1.0) * 123) + 1
        self._serial.write([128, clamp(command, 1, 254), 192])

    def destroy_node(self):
        if hasattr(self, '_serial') and self._serial.is_open:
            self._serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GripperArduinoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
