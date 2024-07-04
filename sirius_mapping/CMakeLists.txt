cmake_minimum_required(VERSION 3.16.3...3.28.1)
project(sirius_mapping)

find_package(
  catkin REQUIRED
  COMPONENTS rospy
             roscpp
             std_msgs
             grid_map_ros
             grid_map_msgs)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  grid_map_msgs
  grid_map_ros)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(gridmap_builder src/gridmap_builder.cpp)
target_link_libraries(gridmap_builder PRIVATE ${catkin_LIBRARIES})
install(TARGETS gridmap_builder DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
