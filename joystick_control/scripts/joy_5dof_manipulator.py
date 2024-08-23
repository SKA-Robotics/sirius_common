#!/usr/bin/python3
import roslib

roslib.load_manifest("joystick_control")

import rospy
from threading import Lock
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.client import Client

from std_msgs.msg import Float32, Empty, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from joystick_control.msg import Gamepad
from joystick_control.cfg import Joy5dofManipulatorConfig
from topic_tools.srv import MuxSelect

from utils.translate_joystick import JoystickTranslator
from utils.axis_transformations import deadzone
from utils.debouncing import Debouncing


class Joystick5dofManipulator:

    def __init__(self) -> None:
        rospy.init_node("joy_5dof_manipulator")
        self.CHANGE_MODE_BUTTON = rospy.get_param("~change_mode_button", None)
        self.MAX_JOINT_EFFORT = rospy.get_param("~max_joint_effort", None)
        self.MAX_JOINT_FORWARD_VELOCITY = rospy.get_param(
            "~max_joint_forward_velocity", None)
        self.MAX_LINEAR_RATE = rospy.get_param("~max_linear_rate", None)
        self.MAX_ANGULAR_RATE = rospy.get_param("~max_angular_rate", None)
        self.GRIPPER_STEP = rospy.get_param("~gripper_step", 0.1)

        print(self.GRIPPER_STEP)
        # topics for main processing
        self.subscriber = rospy.Subscriber(
            "joy_5dof_manipulator",
            Gamepad,
            self._joy_subscriber_callback,
        )
        self.state_subscriber = rospy.Subscriber("/manip_interface/state",
                                                 JointState,
                                                 self._joint_state_callback)
        self.ik_publisher = rospy.Publisher("/cmd_manip", Twist, queue_size=10)
        self.fk_publisher = rospy.Publisher(
            "/joy_5dof_manipulator/manip_command", JointState, queue_size=10)
        self.gripper_publisher = rospy.Publisher("/gripper/set_force",
                                                 Float32,
                                                 queue_size=10)
        self.gripper_open_publisher = rospy.Publisher("/gripper/open_trigger",
                                                      Empty,
                                                      queue_size=10)
        self.klakson_publisher = rospy.Publisher("klakson/cmd",
                                                 String,
                                                 queue_size=10)

        self.multiplexer_select_service = rospy.ServiceProxy(
            "/manip_command_mux/select", MuxSelect)

        self.translator = JoystickTranslator()

        # key debouncing
        self.prev_inputs = {
            key: 0
            for key in self.translator.BUTTONS_ID.keys()
        }
        self.prev_time = rospy.Time.now()

        self.is_down_cross_pressed = False
        self.gripper_command = 0
        self.is_gripper_opening = False
        self.time_of_down_cross = rospy.Time.now()

        # dynamic parameters
        self.mode = 0
        self.gear = 1
        self.gear_max_speeds = {
            1: 10,
            2: 50,
            3: 100,
        }

        # forward kinematics using position control
        self.current_positions = {
            "waist": 0,
            "shoulder": 0,
            "elbow": 0,
            "wrist_lift": 0,
            "wrist_turn": 0,
            "wrist_spin": 0,
        }

        # dynamic_reconfigure server
        self.config_lock = Lock()
        self.config_server = Server(Joy5dofManipulatorConfig,
                                    self._dynamic_reconfigure)
        self.config_client = Client("joy_5dof_manipulator", timeout=30)

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
        debounce = Debouncing(inputs, self.prev_inputs)
        self.prev_inputs = inputs

        with self.config_lock:
            # gear up button
            if debounce.is_leading_edge("right_bumper"):
                self.config_lock.release()
                self.config_client.update_configuration(
                    {"gear": self.gear + 1})
                self.config_lock.acquire()

            # gear down button
            if debounce.is_leading_edge("left_bumper"):
                self.config_lock.release()
                self.config_client.update_configuration(
                    {"gear": self.gear - 1})
                self.config_lock.acquire()

            # mode change
            if debounce.is_leading_edge(self.CHANGE_MODE_BUTTON):
                next_mode = self.mode + 1
                if next_mode > self.config_server.type.max["mode"]:
                    next_mode = self.config_server.type.min["mode"]

                self.config_lock.release()
                self.config_client.update_configuration({"mode": next_mode})
                self.config_lock.acquire()

            multiplier = self.gear_max_speeds[self.gear] / 100
            effort_multiplier = self.MAX_JOINT_EFFORT * multiplier
            linear_multiplier = self.MAX_LINEAR_RATE * multiplier
            angular_multiplier = self.MAX_ANGULAR_RATE * multiplier
            forward_velocity_multiplier = self.MAX_JOINT_FORWARD_VELOCITY * multiplier * min(
                1, (rospy.Time.now() - self.prev_time).to_sec())

            # gripper control
            if debounce.is_trailing_edge("up_cross"):
                self.gripper_command += self.GRIPPER_STEP
                self.gripper_command = min(1, self.gripper_command)
            if debounce.is_leading_edge("down_cross"):
                self.time_of_down_cross = rospy.Time.now()
                self.is_down_cross_pressed = True
            down_cross_press_time = rospy.Time.now() - self.time_of_down_cross
            if debounce.is_trailing_edge("down_cross"):
                self.is_down_cross_pressed = False
                if down_cross_press_time.to_sec() < 0.7:
                    self.gripper_command -= self.GRIPPER_STEP
                    self.gripper_command = max(0, self.gripper_command)
            if self.is_down_cross_pressed and down_cross_press_time.to_sec(
            ) > 0.7:
                self.gripper_open_publisher.publish(Empty())
                self.is_gripper_opening = True
                self.gripper_command = 0
            else:
                self.gripper_publisher.publish(self.gripper_command)
                pass

            if debounce.is_leading_edge("start_button"):
                self.klakson_publisher.publish(String("on"))
            if debounce.is_trailing_edge("start_button"):
                self.klakson_publisher.publish(String("off"))

            # calculate movement based on the current mode
            if self.mode == self.MODES["forward"]:
                message = JointState()

                message.header.stamp = rospy.Time.now()
                message.name = [
                    "waist",
                    "shoulder",
                    "elbow",
                    "wrist_lift",
                    "wrist_turn",
                    "wrist_spin",
                ]
                rotation_effort = -deadzone(inputs["right_stick_horizontal"],
                                            0.15)
                turn_effort = (inputs["left_cross"] - inputs["right_cross"])

                message.effort = [
                    -deadzone(inputs["left_stick_horizontal"], 0.15),
                    (inputs["left_trigger"] - inputs["right_trigger"]),
                    deadzone(inputs["left_stick_vertical"], 0.15),
                    deadzone(inputs["right_stick_vertical"], 0.15),
                    (rotation_effort + turn_effort) / 2,
                    (-rotation_effort + turn_effort) / 2,
                ]

                message.effort = [
                    effort * effort_multiplier for effort in message.effort
                ]

                self.fk_publisher.publish(message)

            if self.mode == self.MODES["forward_positional"]:
                message = JointState()

                message.header.stamp = rospy.Time.now()
                message.name = [
                    "waist",
                    "shoulder",
                    "elbow",
                    "wrist_lift",
                    "wrist_turn",
                    "wrist_spin",
                ]
                message.position = [
                    self.current_positions['waist'] +
                    deadzone(inputs["left_stick_horizontal"], 0.15),
                    self.current_positions['shoulder'] +
                    (inputs["left_trigger"] - inputs["right_trigger"]),
                    self.current_positions['elbow'] +
                    deadzone(inputs["left_stick_vertical"], 0.15),
                    self.current_positions['wrist_lift'] +
                    deadzone(inputs["right_stick_vertical"], 0.15),
                    self.current_positions['wrist_turn'] +
                    deadzone(inputs["right_stick_horizontal"], 0.15),
                    self.current_positions['wrist_spin'] +
                    (inputs["left_cross"] - inputs["right_cross"]),
                ]

                message.position = [
                    effort * forward_velocity_multiplier
                    for effort in message.effort
                ]

                self.fk_publisher.publish(message)

            if self.mode == self.MODES["inverse"]:
                message = Twist()

                message.linear.x = -1 * deadzone(inputs["left_stick_vertical"],
                                                 0.15)
                message.linear.y = -1 * deadzone(
                    inputs["right_stick_horizontal"], 0.15)
                message.linear.z = -1 * deadzone(
                    inputs["right_stick_vertical"], 0.15)

                message.angular.x = -1 * deadzone(
                    inputs["left_stick_horizontal"], 0.15)
                message.angular.y = inputs["right_trigger"] - inputs[
                    "left_trigger"]
                message.angular.z = inputs["right_cross"] - inputs["left_cross"]

                message.linear.x *= linear_multiplier
                message.linear.y *= linear_multiplier
                message.linear.z *= linear_multiplier

                message.angular.x *= angular_multiplier
                message.angular.y *= angular_multiplier
                message.angular.z *= angular_multiplier
                self.ik_publisher.publish(message)

        self.prev_time = rospy.Time.now()

    def _joint_state_callback(self, msg: JointState):
        for i, joint in enumerate(msg.name):
            self.current_positions[joint] = msg.position[i]

    def _dynamic_reconfigure(self, config, level):
        if hasattr(self, "MODES"):
            if config["mode"] == self.MODES["forward"]:
                self.multiplexer_select_service(
                    "/joy_5dof_manipulator/manip_command")
            if config["mode"] == self.MODES["inverse"]:
                self.multiplexer_select_service(
                    "/manip_controller/manip_command")

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
        Joystick5dofManipulator().run()
    except rospy.ROSInterruptException:
        pass
