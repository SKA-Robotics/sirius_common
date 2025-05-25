#!/usr/bin/python3
from manip.arm_joystick_control.joystick import JoystickControl
from manip.manip_config import load_ros_params

config = load_ros_params()
joystick_control = JoystickControl(config)
joystick_control.run()