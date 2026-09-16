#!/usr/bin/env python3
"""ERC/URC competition autonomy node — RS232 satellite protocol + nav2."""

import crc
import serial
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32, String
from nav2_msgs.action import NavigateToPose
import pymap3d

Acknowledge = 0x00
ArmDisarm = 0x01
NavigateToGPS = 0x02
TaskCompleted = 0x03
SetStage = 0x04
Text = 0x05
ArucoTag = 0x06
LocateArucoTags = 0x07
Location3D = 0x08
Detection = 0x09
SetParameters = 0x0A

map_origin_coords = (39.9011333333333333, 32.77005, 907.476)

hold_repeater = True
lamp_color = 'red'


def _bits_to_float(value):
    return struct.unpack('f', struct.pack('I', value))[0]


class Rs232Adapter:

    def __init__(self, port='/dev/digitusrs232'):
        self.serial = serial.Serial(port=port, baudrate=115200, timeout=1)

    def await_message(self, msg_id):
        while True:
            start_bit = self.serial.read(1)
            if not start_bit or start_bit[0] != 0x7E:
                continue
            id_ = self.serial.read(1)[0]
            if id_ != msg_id:
                continue
            n = self.serial.read(1)[0]
            data = [self.serial.read(1)[0] for _ in range(n)]
            crc1 = self.serial.read(1)[0]
            crc2 = self.serial.read(1)[0]
            crc_value = (crc1 << 8) | crc2
            checksum = crc.Calculator(crc.Crc16.XMODEM.value).checksum(
                bytes([0x7E, id_, n, *data]))
            if crc_value == checksum:
                return {'msg_id': msg_id, 'data': data}

    def send_ack(self):
        data = [0x7E, 0x00, 0x00]
        checksum = crc.Calculator(crc.Crc16.XMODEM.value).checksum(bytes(data))
        self.serial.write(bytes([*data, checksum >> 8, checksum & 0xFF]))

    def send_task_completed(self):
        data = [0x7E, 0x03, 0x00]
        checksum = crc.Calculator(crc.Crc16.XMODEM.value).checksum(bytes(data))
        self.serial.write(bytes([*data, checksum >> 8, checksum & 0xFF]))


class Navigator:

    def __init__(self, node: Node):
        self._node = node
        self._action_client = ActionClient(node, NavigateToPose,
                                           'navigate_to_pose')
        self._action_client.wait_for_server()
        self.state = 'ok'
        self._goal_handle = None

    def start_navigating_to(self, pose: PoseStamped):
        self.state = 'ok'
        goal = NavigateToPose.Goal()
        goal.pose = pose
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        self._goal_handle = future.result()
        if self._goal_handle and self._goal_handle.accepted:
            self._goal_handle.get_result_async().add_done_callback(
                self._result_callback)

    def _result_callback(self, future):
        self.state = 'done'

    def get_status(self):
        return self.state

    def stop_navigating(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()


class Autonomy(Node):

    def __init__(self):
        super().__init__('autonomy')
        self.rs232 = Rs232Adapter()
        self.navigator = Navigator(self)
        self.stages_done = []

        self.gripper_pub = self.create_publisher(Float32, '/gripper/set_force', 10)
        self.lamp_pub = self.create_publisher(String, 'lamps/color_override', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def run(self):
        self.get_logger().info('Autonomy started')
        self.set_lamp('red')
        self.rs232.await_message(SetParameters)
        self.rs232.send_ack()

        while len(self.stages_done) < 3:
            message = self.rs232.await_message(SetStage)
            self.rs232.send_ack()
            stage = message['data'][0]
            self.get_logger().info(f'Stage {stage} requested')
            if stage not in self.stages_done:
                if stage == 1:
                    self._stage_one()
                    self.stages_done.append(1)
                elif stage == 2:
                    self._stage_two()
                    self.stages_done.append(2)
                elif stage == 3:
                    self._stage_three()
                    self.stages_done.append(3)

        self.get_logger().info('Autonomy done')

    def _stage_one(self):
        self.rs232.await_message(ArmDisarm)
        self.set_lamp('green')
        self.rs232.send_ack()
        message = self.rs232.await_message(NavigateToGPS)
        self.rs232.send_ack()
        self.set_lamp('yellow')
        target = self._parse_navigation_command(message['data'])
        self._leave_airlock()
        self._navigate_to(*target)
        time.sleep(4)
        self._drop_repeater()
        self.set_lamp('green')
        self.rs232.send_task_completed()

    def _stage_two(self):
        self.set_lamp('green')
        message = self.rs232.await_message(NavigateToGPS)
        self.rs232.send_ack()
        self.set_lamp('yellow')
        target = self._parse_navigation_command(message['data'])
        self._navigate_to(*target)
        self.set_lamp('green')
        self.rs232.send_task_completed()
        self.rs232.await_message(LocateArucoTags)
        self.rs232.send_ack()
        self.rs232.send_task_completed()

    def _stage_three(self):
        self.set_lamp('green')
        message = self.rs232.await_message(NavigateToGPS)
        self.rs232.send_ack()
        self.set_lamp('yellow')
        target = self._parse_navigation_command(message['data'])
        self._navigate_to(*target)
        self.set_lamp('green')
        self.rs232.send_task_completed()
        self.rs232.await_message(LocateArucoTags)
        self.rs232.send_ack()
        self.rs232.send_task_completed()
        self.rs232.await_message(ArmDisarm)
        self.rs232.send_ack()
        self.set_lamp('red')

    def _parse_navigation_command(self, command_data):
        lat_repr = ((command_data[0] << 24) | (command_data[1] << 16) |
                    (command_data[2] << 8) | command_data[3])
        lon_repr = ((command_data[4] << 24) | (command_data[5] << 16) |
                    (command_data[6] << 8) | command_data[7])
        return _bits_to_float(lat_repr), _bits_to_float(lon_repr)

    def _leave_airlock(self):
        cmd = Twist()
        cmd.linear.x = 0.1
        cmd.angular.z = -0.005
        deadline = time.time() + 40.0
        while time.time() < deadline and rclpy.ok():
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.1)

    def _navigate_to(self, lat, lon):
        self.get_logger().info(f'Navigating to {lat}, {lon}')
        e, n, _ = pymap3d.geodetic2enu(
            lat, lon, map_origin_coords[2],
            map_origin_coords[0], map_origin_coords[1], map_origin_coords[2])
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = e
        pose.pose.position.y = n
        pose.pose.orientation.w = 1.0
        self.navigator.start_navigating_to(pose)

        deadline = time.time() + 8 * 60
        while self.navigator.get_status() != 'done' and rclpy.ok():
            if time.time() > deadline:
                self.navigator.stop_navigating()
                self.get_logger().warn('Navigation timeout')
                return
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info('Target reached')

    def _drop_repeater(self):
        global hold_repeater
        hold_repeater = False

    def set_lamp(self, color):
        global lamp_color
        lamp_color = color
        self.lamp_pub.publish(String(data=color))


def main(args=None):
    rclpy.init(args=args)
    node = Autonomy()

    def _publisher_worker():
        rate_delay = 1.0
        has_opened = False
        opening_start = time.time()
        while rclpy.ok():
            if hold_repeater:
                node.gripper_pub.publish(Float32(data=1.0))
            else:
                msg = Float32(data=0.0)
                node.gripper_pub.publish(msg)
                if not has_opened:
                    has_opened = True
                    opening_start = time.time()
                if has_opened and time.time() > opening_start + 10:
                    break
            time.sleep(rate_delay)

    def _lamp_worker():
        while rclpy.ok():
            node.lamp_pub.publish(String(data=lamp_color))
            time.sleep(1.0)

    publisher_thread = threading.Thread(target=_publisher_worker, daemon=True)
    publisher_thread.start()
    lamp_thread = threading.Thread(target=_lamp_worker, daemon=True)
    lamp_thread.start()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
