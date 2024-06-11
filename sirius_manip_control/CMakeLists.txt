cmake_minimum_required(VERSION 3.0.2)
project(manip_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  dynamic_reconfigure
)

catkin_python_setup()

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/ManipInterface.cfg
)

catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)