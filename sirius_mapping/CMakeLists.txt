cmake_minimum_required(VERSION 3.16.3...3.28.1)
project(sirius_mapping)

find_package(
  catkin REQUIRED
  COMPONENTS rospy
             roscpp
             pcl_ros
             pcl_conversions
             std_msgs
             grid_map_ros
             grid_map_msgs)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  pcl_ros
  pcl_conversions
  grid_map_msgs
  grid_map_ros)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(gridmap_builder src/gridmap_builder.cpp)
add_executable(pointcloud_tranformer src/pointcloud_transformer.cpp)
target_link_libraries(gridmap_builder PRIVATE ${catkin_LIBRARIES})
target_link_libraries(pointcloud_tranformer PRIVATE ${catkin_LIBRARIES})
install(TARGETS gridmap_builder pointcloud_tranformer DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
