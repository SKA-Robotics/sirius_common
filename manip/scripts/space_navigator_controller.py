#! /usr/bin/python3
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

import rospy

from manip.arm_joystick_control.gripper_controller import GripperController
from manip.arm_joystick_control.ros_command_sender import RosCommandSender
from manip.manip_config import ManipConfig, DEFAULT_CONFIG, load_ros_params
from manip.arm_joystick_control.utils import max_abs


class SpaceNavigatorController:

    def __init__(self, config: ManipConfig = DEFAULT_CONFIG):
        self.config = config
        self.frame = config.twist_cmd_frame_id
        self.previous_input_button2 = 0.0
        self.command = 0.0

        rospy.init_node("space_navigator_controller")
        self.command_sender = RosCommandSender(
            config.twist_topic,
            config.joint_topic,
            config.gripper_cmd_topic,
            config.preset_request_topic,
        )
        self.gripper_controller = GripperController()
        rospy.Subscriber("/spacenav/twist", Twist, self._twist_callback,
                         queue_size=10)
        rospy.Subscriber("/spacenav/joy", Joy, self._joy_callback,
                         queue_size=10)

    def _twist_callback(self, msg: Twist):
        command = [
            -msg.linear.z * self.config.max_ev[0],
            msg.linear.y * self.config.max_ev[1],
            msg.linear.x * self.config.max_ev[2],
            -msg.angular.z * self.config.max_ev[3],
            msg.angular.y * self.config.max_ev[4],
            msg.angular.x * self.config.max_ev[5],
        ]
        self.command_sender.send_twist_command(command, self.frame)

    def _joy_callback(self, msg: Joy):
        if not msg.buttons:
            return
        # gripper = max_abs(float(msg.buttons[0]), float(msg.buttons[1]))
        button1 = float(msg.buttons[0])
        button2 = float(msg.buttons[1])
        self._control_gripper(button1, button2)

    def _control_gripper(self, button1: float, button2: float):
        # gripper_force_cmd = self.gripper_controller.step(button1)

        release_cmd_value = -2.0
        close_cmd_value = 1.5
        self.command 
        
        if button1:
            # self.command_sender.send_gripper_command(close_cmd_value)
            self.command = close_cmd_value
        elif button2 == 0.0 and self.previous_input_button2 == 1.0:
            # self.command_sender.send_gripper_command(0.0)
            self.command = 0.0
        elif button2:
            # self.command_sender.send_gripper_command(release_cmd_value)
            self.command = release_cmd_value

        self.previous_input_button2 = button2

    def run(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.command_sender.send_gripper_command(self.command)
            rate.sleep()
        


if __name__ == "__main__":
    controller = SpaceNavigatorController(load_ros_params())
    controller.run()
