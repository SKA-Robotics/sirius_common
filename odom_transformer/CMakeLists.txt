cmake_minimum_required(VERSION 3.16.3...3.28.1)
project(odom_transformer)
find_package(catkin REQUIRED COMPONENTS rospy nav_msgs)
catkin_package()
catkin_install_python(
  PROGRAMS
  scripts/child_frame_transform.py
  scripts/offset.py
  DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION})
