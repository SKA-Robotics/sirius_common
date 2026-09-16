#!/usr/bin/env python3

from enum import Enum
from queue import Queue

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PointStamped, Twist
from std_srvs.srv import Empty

from manip import SiriusManip
from manip_interface_ros import ROSManipInterface
from ik import ManipPose


class ControlMode(Enum):
    CARTESIAN = 0
    JOINTSPACE = 1


class JoystickReceiver:

    def __init__(self, joystick_topic: str, timeout: float, node: Node):
        self._velocity = Twist()
        self._timeout = timeout
        self._node = node
        self._prev_time = node.get_clock().now()

        node.create_subscription(Twist, joystick_topic,
                                 self._set_velocity, 10)

    def _set_velocity(self, velocity: Twist):
        self._velocity = velocity
        self._prev_time = self._node.get_clock().now()

    def is_timed_out(self) -> bool:
        elapsed = (self._node.get_clock().now() -
                   self._prev_time).nanoseconds / 1e9
        return elapsed > self._timeout

    def get_pose_delta(self, deltatime: float) -> ManipPose:
        if self.is_timed_out():
            self._velocity = Twist()
        v = self._velocity
        values = [
            v.linear.x, v.linear.y, v.linear.z,
            v.angular.x, v.angular.y, v.angular.z,
        ]
        return ManipPose.from_list([x * deltatime for x in values])


class ManipControllerNode(Node):

    def __init__(self):
        super().__init__('manip_controller')

        self.declare_parameter('joystick_timeout', 0.5)
        self.declare_parameter('queue_size', 16)
        self.declare_parameter('control_modes.incremental.send_rate', 50.0)

        joystick_timeout = self.get_parameter('joystick_timeout').value
        queue_size = self.get_parameter('queue_size').value
        send_rate = self.get_parameter(
            'control_modes.incremental.send_rate').value

        interface = ROSManipInterface(self)
        self.manip = SiriusManip(interface)
        self.mode = ControlMode.JOINTSPACE
        self.joystick_receiver = JoystickReceiver(
            '/cmd_manip', joystick_timeout, self)
        self.pending_moves: Queue = Queue(queue_size)
        self._period = 1.0 / send_rate

        self.create_subscription(PointStamped, '/cmd_manip_pos',
                                 self._pose_callback, 10)
        self.create_service(Empty, '~/toggle_mode', self._toggle_mode_cb)

        self.create_timer(self._period, self._step)

    def _step(self):
        try:
            if self.joystick_receiver.is_timed_out():
                self.manip.reset_incremental_start_pose()
            self._execute_pending_moves()
            self._execute_joystick_command()
        except Exception as e:
            self.get_logger().warn(str(e))

    def _execute_pending_moves(self):
        while not self.pending_moves.empty():
            move = self.pending_moves.get()
            move[0](move[1])
            self.get_logger().info('Target pose reached')

    def _execute_joystick_command(self):
        delta = self.joystick_receiver.get_pose_delta(self._period)
        self.manip.move_incremental(delta)

    def _pose_callback(self, data: PointStamped):
        self.get_logger().info(f'Received request: {data.point}')
        target = ManipPose()
        target.x = data.point.x
        target.y = data.point.y
        target.z = data.point.z
        target.roll = target.pitch = target.yaw = 0
        if self.mode == ControlMode.CARTESIAN:
            self._enqueue((self.manip.move_cartesian, target))
        else:
            self._enqueue((self.manip.move_jointspace, target))

    def _enqueue(self, move):
        if self.pending_moves.full():
            self.get_logger().warn('Move queue full — dropping oldest')
            self.pending_moves.get()
        self.pending_moves.put(move)

    def _toggle_mode_cb(self, req, resp):
        if self.mode == ControlMode.CARTESIAN:
            self.mode = ControlMode.JOINTSPACE
        else:
            self.mode = ControlMode.CARTESIAN
        self.get_logger().info(f'Switched to {self.mode.name} mode')
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = ManipControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
