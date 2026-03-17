#!/usr/bin/env python3
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'utils'))

from threading import Lock

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import Twist
from joystick_control.msg import Gamepad

from utils.translate_joystick import JoystickTranslator
from utils.axis_transformations import deadzone


MODES = {
    'normal': 0,
    'car': 1,
    'tank': 2,
}


class JoystickDifferentialDrive(Node):

    def __init__(self) -> None:
        super().__init__('joy_diff_drive')

        self.declare_parameter('change_mode_button', 'back_button')
        self.declare_parameter('max_angular_rate', 1.0)
        self.declare_parameter('max_linear_rate', 1.0)
        self.declare_parameter('joystick_type', 'STANDARD')

        # dynamic_reconfigure parameters — now plain ROS2 parameters
        self.declare_parameter('mode', 0)
        self.declare_parameter('gear', 1)
        self.declare_parameter('gear_1_max_speed', 10)
        self.declare_parameter('gear_2_max_speed', 50)
        self.declare_parameter('gear_3_max_speed', 100)

        self.CHANGE_MODE_BUTTON = self.get_parameter(
            'change_mode_button').value
        self.MAX_ANGULAR_RATE = self.get_parameter('max_angular_rate').value
        self.MAX_LINEAR_RATE = self.get_parameter('max_linear_rate').value

        self.mode = self.get_parameter('mode').value
        self.gear = self.get_parameter('gear').value
        self.gear_max_speeds = {
            1: self.get_parameter('gear_1_max_speed').value,
            2: self.get_parameter('gear_2_max_speed').value,
            3: self.get_parameter('gear_3_max_speed').value,
        }

        self.subscriber = self.create_subscription(
            Gamepad,
            'joy_diff_drive',
            self._joy_subscriber_callback,
            10,
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.translator = JoystickTranslator(self)

        self.prev_inputs = {
            key: 0
            for key in self.translator.BUTTONS_ID.keys()
        }

        self.config_lock = Lock()

        # replaces dynamic_reconfigure callback
        self.add_on_set_parameters_callback(self._on_parameter_change)

    def _on_parameter_change(self, params):
        with self.config_lock:
            for param in params:
                if param.name == 'mode':
                    self.mode = param.value
                elif param.name == 'gear':
                    self.gear = max(1, min(3, param.value))
                elif param.name == 'gear_1_max_speed':
                    self.gear_max_speeds[1] = param.value
                elif param.name == 'gear_2_max_speed':
                    self.gear_max_speeds[2] = param.value
                elif param.name == 'gear_3_max_speed':
                    self.gear_max_speeds[3] = param.value
        return SetParametersResult(successful=True)

    def _joy_subscriber_callback(self, data: Gamepad):
        inputs = self.translator.translate(data)

        with self.config_lock:
            # gear up
            if inputs['right_bumper'] == 1 and self.prev_inputs[
                    'right_bumper'] == 0:
                new_gear = min(3, self.gear + 1)
                self.set_parameters(
                    [rclpy.parameter.Parameter('gear',
                                               rclpy.Parameter.Type.INTEGER,
                                               new_gear)])

            # gear down
            if inputs['left_bumper'] == 1 and self.prev_inputs[
                    'left_bumper'] == 0:
                new_gear = max(1, self.gear - 1)
                self.set_parameters(
                    [rclpy.parameter.Parameter('gear',
                                               rclpy.Parameter.Type.INTEGER,
                                               new_gear)])

            # mode change
            if (inputs[self.CHANGE_MODE_BUTTON] == 1
                    and self.prev_inputs[self.CHANGE_MODE_BUTTON] == 0):
                next_mode = (self.mode + 1) % len(MODES)
                self.set_parameters(
                    [rclpy.parameter.Parameter('mode',
                                               rclpy.Parameter.Type.INTEGER,
                                               next_mode)])

            linear_multiplier = (self.MAX_LINEAR_RATE *
                                 self.gear_max_speeds[self.gear] / 100)
            angular_multiplier = (self.MAX_ANGULAR_RATE *
                                  self.gear_max_speeds[self.gear] / 100)

            linear_speed = 0.0
            angular_speed = 0.0

            if self.mode == MODES['normal']:
                linear_speed = -1 * deadzone(inputs['right_stick_vertical'],
                                             0.15)
                angular_speed = -1 * deadzone(
                    inputs['right_stick_horizontal'], 0.15)

            elif self.mode == MODES['car']:
                linear_speed = (inputs['right_trigger'] -
                                inputs['left_trigger'])
                angular_speed = -1 * deadzone(
                    inputs['left_stick_horizontal'], 0.15)

            elif self.mode == MODES['tank']:
                linear_speed = (-1 * deadzone(
                    inputs['right_stick_vertical'] +
                    inputs['left_stick_vertical'], 0.15) / 2)
                angular_speed = (deadzone(
                    inputs['left_stick_vertical'] -
                    inputs['right_stick_vertical'], 0.15) / 2)

            message = Twist()
            message.linear.x = linear_multiplier * linear_speed
            message.angular.z = angular_multiplier * angular_speed
            self.publisher.publish(message)

        self.prev_inputs = inputs


def main(args=None):
    rclpy.init(args=args)
    node = JoystickDifferentialDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()