#!/usr/bin/env python3
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'utils'))

from threading import Lock

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from joystick_control.msg import Gamepad

from utils.translate_joystick import JoystickTranslator
from utils.axis_transformations import deadzone
from utils.debouncing import Debouncing


MODES = {
    'forward': 0,
    'inverse': 1,
}

# topic_tools MuxSelect replacement — we publish directly to both
# topics and let the downstream mux (if any) decide, or we
# publish to an output topic selected by parameter.
MUX_TOPICS = {
    MODES['forward']: 'joy_5dof_manipulator/manip_command',
    MODES['inverse']: 'manip_controller/manip_command',
}


class Joystick5dofManipulator(Node):

    def __init__(self) -> None:
        super().__init__('joy_5dof_manipulator')

        self.declare_parameter('change_mode_button', 'back_button')
        self.declare_parameter('max_joint_effort', 1.0)
        self.declare_parameter('max_joint_forward_velocity', 1.0)
        self.declare_parameter('max_linear_rate', 0.1)
        self.declare_parameter('max_angular_rate', 0.4)
        self.declare_parameter('gripper_step', 0.1)
        self.declare_parameter('joystick_type', 'STANDARD')

        # dynamic_reconfigure parameters — now plain ROS2 parameters
        self.declare_parameter('mode', 0)
        self.declare_parameter('gear', 1)
        self.declare_parameter('gear_1_max_speed', 10)
        self.declare_parameter('gear_2_max_speed', 50)
        self.declare_parameter('gear_3_max_speed', 100)

        self.CHANGE_MODE_BUTTON = self.get_parameter(
            'change_mode_button').value
        self.MAX_JOINT_EFFORT = self.get_parameter('max_joint_effort').value
        self.MAX_JOINT_FORWARD_VELOCITY = self.get_parameter(
            'max_joint_forward_velocity').value
        self.MAX_LINEAR_RATE = self.get_parameter('max_linear_rate').value
        self.MAX_ANGULAR_RATE = self.get_parameter('max_angular_rate').value
        self.GRIPPER_STEP = self.get_parameter('gripper_step').value

        self.mode = self.get_parameter('mode').value
        self.gear = self.get_parameter('gear').value
        self.gear_max_speeds = {
            1: self.get_parameter('gear_1_max_speed').value,
            2: self.get_parameter('gear_2_max_speed').value,
            3: self.get_parameter('gear_3_max_speed').value,
        }

        self.subscriber = self.create_subscription(
            Gamepad,
            'joy_5dof_manipulator',
            self._joy_subscriber_callback,
            10,
        )
        self.state_subscriber = self.create_subscription(
            JointState,
            '/manip_interface/state',
            self._joint_state_callback,
            10,
        )

        self.ik_publisher = self.create_publisher(Twist, '/cmd_manip', 10)
        self.fk_publisher = self.create_publisher(
            JointState, 'joy_5dof_manipulator/manip_command', 10)
        self.gripper_publisher = self.create_publisher(
            Float32, '/gripper/set_force', 10)
        self.klakson_publisher = self.create_publisher(
            String, 'klakson/cmd', 10)

        self.translator = JoystickTranslator(self)

        self.prev_inputs = {
            key: 0
            for key in self.translator.BUTTONS_ID.keys()
        }
        self.prev_time = self.get_clock().now()

        self.is_down_cross_pressed = False
        self.gripper_command = 0.0
        self.time_of_down_cross = self.get_clock().now()

        self.current_positions = {
            'waist': 0.0,
            'shoulder': 0.0,
            'elbow': 0.0,
            'wrist_lift': 0.0,
            'wrist_turn': 0.0,
            'wrist_spin': 0.0,
        }

        self.config_lock = Lock()
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
        debounce = Debouncing(inputs, self.prev_inputs)
        self.prev_inputs = inputs
        now = self.get_clock().now()

        with self.config_lock:
            # gear up
            if debounce.is_leading_edge('right_bumper'):
                new_gear = min(3, self.gear + 1)
                self.set_parameters([
                    rclpy.parameter.Parameter('gear',
                                              rclpy.Parameter.Type.INTEGER,
                                              new_gear)
                ])

            # gear down
            if debounce.is_leading_edge('left_bumper'):
                new_gear = max(1, self.gear - 1)
                self.set_parameters([
                    rclpy.parameter.Parameter('gear',
                                              rclpy.Parameter.Type.INTEGER,
                                              new_gear)
                ])

            # mode change
            if debounce.is_leading_edge(self.CHANGE_MODE_BUTTON):
                next_mode = (self.mode + 1) % len(MODES)
                self.set_parameters([
                    rclpy.parameter.Parameter('mode',
                                              rclpy.Parameter.Type.INTEGER,
                                              next_mode)
                ])

            multiplier = self.gear_max_speeds[self.gear] / 100
            effort_multiplier = self.MAX_JOINT_EFFORT * multiplier
            linear_multiplier = self.MAX_LINEAR_RATE * multiplier
            angular_multiplier = self.MAX_ANGULAR_RATE * multiplier
            dt = (now - self.prev_time).nanoseconds / 1e9
            forward_velocity_multiplier = (
                self.MAX_JOINT_FORWARD_VELOCITY * multiplier * min(1, dt))

            # gripper control
            if debounce.is_trailing_edge('up_cross'):
                self.gripper_command = min(1.0,
                                           self.gripper_command +
                                           self.GRIPPER_STEP)

            if debounce.is_leading_edge('down_cross'):
                self.time_of_down_cross = now
                self.is_down_cross_pressed = True

            down_cross_press_time = (
                now - self.time_of_down_cross).nanoseconds / 1e9

            if debounce.is_trailing_edge('down_cross'):
                self.is_down_cross_pressed = False
                if down_cross_press_time < 0.7:
                    self.gripper_command = max(0.0,
                                               self.gripper_command -
                                               self.GRIPPER_STEP)

            if self.is_down_cross_pressed and down_cross_press_time > 0.7:
                # open gripper trigger (publish 0.0 force = open)
                msg = Float32()
                msg.data = 0.0
                self.gripper_publisher.publish(msg)
                self.gripper_command = 0.0
            else:
                msg = Float32()
                msg.data = float(self.gripper_command)
                self.gripper_publisher.publish(msg)

            if debounce.is_leading_edge('start_button'):
                self.klakson_publisher.publish(String(data='on'))
            if debounce.is_trailing_edge('start_button'):
                self.klakson_publisher.publish(String(data='off'))

            # forward kinematics mode
            if self.mode == MODES['forward']:
                message = JointState()
                message.header.stamp = now.to_msg()
                message.name = [
                    'waist', 'shoulder', 'elbow',
                    'wrist_lift', 'wrist_turn', 'wrist_spin',
                ]
                rotation_effort = deadzone(inputs['left_stick_horizontal'],
                                           0.15)
                turn_effort = inputs['left_cross'] - inputs['right_cross']
                message.effort = [
                    -deadzone(inputs['right_stick_horizontal'], 0.15),
                    deadzone(inputs['left_stick_vertical'], 0.15),
                    deadzone(inputs['right_stick_vertical'], 0.15),
                    (inputs['left_trigger'] - inputs['right_trigger']),
                    (turn_effort + rotation_effort),
                    (-turn_effort + rotation_effort),
                ]
                message.effort = [
                    e * effort_multiplier for e in message.effort
                ]
                self.fk_publisher.publish(message)

            # inverse kinematics mode
            elif self.mode == MODES['inverse']:
                message = Twist()
                message.linear.x = -1 * deadzone(
                    inputs['left_stick_vertical'], 0.15)
                message.linear.y = -1 * deadzone(
                    inputs['right_stick_horizontal'], 0.15)
                message.linear.z = -1 * deadzone(
                    inputs['right_stick_vertical'], 0.15)
                message.angular.x = deadzone(
                    inputs['left_stick_horizontal'], 0.15) * 5
                message.angular.y = (inputs['left_trigger'] -
                                     inputs['right_trigger'])
                message.angular.z = (inputs['left_cross'] -
                                     inputs['right_cross'])

                message.linear.x *= linear_multiplier
                message.linear.y *= linear_multiplier
                message.linear.z *= linear_multiplier
                message.angular.x *= angular_multiplier
                message.angular.y *= angular_multiplier
                message.angular.z *= angular_multiplier
                self.ik_publisher.publish(message)

        self.prev_time = now

    def _joint_state_callback(self, msg: JointState):
        for i, joint in enumerate(msg.name):
            if joint in self.current_positions:
                self.current_positions[joint] = msg.position[i]


def main(args=None):
    rclpy.init(args=args)
    node = Joystick5dofManipulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()