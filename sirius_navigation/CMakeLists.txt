cmake_minimum_required(VERSION 3.16.3...3.28.1)
project(sirius_navigation)
find_package(catkin REQUIRED COMPONENTS
  rospy
  mbf_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    mbf_msgs
)

catkin_install_python(
  PROGRAMS scripts/imu_align_north.py scripts/localization_evaluation.py
  scripts/random_traversal.py scripts/behavior_tree.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
