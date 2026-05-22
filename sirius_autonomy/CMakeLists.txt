cmake_minimum_required(VERSION 3.0.2)
project(sirius_autonomy)

find_package(
  catkin REQUIRED
  COMPONENTS rospy
             std_msgs
             geometry_msgs
             sensor_msgs
             tf2_ros
             tf2_geometry_msgs
             visualization_msgs)

catkin_package(
  CATKIN_DEPENDS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs)

include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(
  PROGRAMS
  scripts/aruco_detection_node.py
  scripts/robot_localization_node.py
  DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION})
