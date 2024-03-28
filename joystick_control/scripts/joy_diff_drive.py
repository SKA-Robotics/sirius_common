#!/usr/bin/python3
import roslib

roslib.load_manifest("joystick_control")

import rospy
from threading import Lock
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.client import Client

from geometry_msgs.msg import Twist
from joystick_control.msg import Gamepad
from joystick_control.cfg import JoyDiffDriveConfig

from utils.translate_joystick import JoystickTranslator
from utils.axis_transformations import deadzone


class JoystickDifferentialDrive:

    def __init__(self) -> None:
        rospy.init_node("joy_diff_drive")
        self.CHANGE_MODE_BUTTON = rospy.get_param("~change_mode_button", None)
        self.MAX_ANGULAR_RATE = rospy.get_param("~max_angular_rate", None)
        self.MAX_LINEAR_RATE = rospy.get_param("~max_linear_rate", None)
        # topics for main processing
        self.subscriber = rospy.Subscriber(
            "joy_diff_drive",
            Gamepad,
            self._joy_subscriber_callback,
        )
        self.publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.translator = JoystickTranslator()

        # key debouncing
        self.prev_inputs = {
            key: 0
            for key in self.translator.BUTTONS_ID.keys()
        }

        # dynamic parameters
        self.mode = 0
        self.gear = 1
        self.gear_max_speeds = {
            1: 10,
            2: 50,
            3: 100,
        }

        # dynamic_reconfigure server
        self.config_lock = Lock()
        self.config_server = Server(JoyDiffDriveConfig,
                                    self._dynamic_reconfigure)
        self.config_client = Client("joy_diff_drive", timeout=30)

        self.MODES = next(
            ({
                entry["name"]: entry["value"]
                for entry in eval(param.edit_method)["enum"]
            } for param in self.config_server.description.groups[0].parameters
             if param.name == "mode"),
            {},
        )

    def run(self) -> None:
        rospy.spin()

    def _joy_subscriber_callback(self, data: Gamepad):
        inputs = self.translator.translate(data)

        with self.config_lock:
            # gear up button
            if inputs["right_bumper"] == 1 and self.prev_inputs[
                    "right_bumper"] == 0:
                self.config_lock.release()
                self.config_client.update_configuration(
                    {"gear": self.gear + 1})
                self.config_lock.acquire()

            # gear down button
            if inputs["left_bumper"] == 1 and self.prev_inputs[
                    "left_bumper"] == 0:
                self.config_lock.release()
                self.config_client.update_configuration(
                    {"gear": self.gear - 1})
                self.config_lock.acquire()

            # mode change
            if (inputs[self.CHANGE_MODE_BUTTON] == 1
                    and self.prev_inputs[self.CHANGE_MODE_BUTTON] == 0):
                next_mode = self.mode + 1
                if next_mode > self.config_server.type.max["mode"]:
                    next_mode = self.config_server.type.min["mode"]

                self.config_lock.release()
                self.config_client.update_configuration({"mode": next_mode})
                self.config_lock.acquire()

            linear_multiplier = (self.MAX_LINEAR_RATE *
                                 self.gear_max_speeds[self.gear] / 100)
            angular_multiplier = (self.MAX_ANGULAR_RATE *
                                  self.gear_max_speeds[self.gear] / 100)
            # calculate movement based on the current mode
            linear_speed = 0.0
            angular_speed = 0.0
            if self.mode == self.MODES["normal"]:
                linear_speed = -1 * deadzone(inputs["right_stick_vertical"],
                                             0.15)
                angular_speed = -1 * deadzone(inputs["right_stick_horizontal"],
                                              0.15)

            if self.mode == self.MODES["car"]:
                linear_speed = inputs["right_trigger"] - inputs["left_trigger"]
                angular_speed = -1 * deadzone(inputs["left_stick_horizontal"],
                                              0.15)

            if self.mode == self.MODES["tank"]:
                linear_speed = (-1 * deadzone(
                    inputs["right_stick_vertical"] +
                    inputs["left_stick_vertical"],
                    0.15,
                ) / 2)
                angular_speed = (deadzone(
                    inputs["left_stick_vertical"] -
                    inputs["right_stick_vertical"],
                    0.15,
                ) / 2)

            message = Twist()
            message.linear.x = linear_multiplier * linear_speed
            message.angular.z = angular_multiplier * angular_speed
            self.publisher.publish(message)

        self.prev_inputs = inputs

    def _dynamic_reconfigure(self, config, level):
        with self.config_lock:
            self.mode = config["mode"]
            self.gear = config["gear"]
            self.gear_max_speeds = {
                1: config["gear_1_max_speed"],
                2: config["gear_2_max_speed"],
                3: config["gear_3_max_speed"],
            }

        return config


if __name__ == "__main__":
    try:
        JoystickDifferentialDrive().run()
    except rospy.ROSInterruptException:
        pass
