cmake_minimum_required(VERSION 3.16.3...3.28.1)
project(manip)

find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs std_msgs)

catkin_package(
  CATKIN_DEPENDS
  rospy
  sensor_msgs
  std_msgs)

catkin_python_setup()

install(
  PROGRAMS scripts/arm_controller_node.py
           scripts/joystick_node.py
           scripts/gripper_node.py
           scripts/mock_manip.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
