#!/usr/bin/python3

from manip.arm_servo.arm_controller import main
from manip.manip_config import load_ros_params

config = load_ros_params()
main(config)