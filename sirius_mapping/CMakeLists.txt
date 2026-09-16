cmake_minimum_required(VERSION 3.8)
project(sirius_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# grid_map_core sets global EIGEN_FUNCTORS_PLUGIN / EIGEN_DENSEBASE_PLUGIN
# compiler definitions that affect every target in this CMake project.
# It must be found first and its include directory propagated to all targets.
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

# Collect the grid_map_core include directory so all targets can satisfy the
# EIGEN_FUNCTORS_PLUGIN include that grid_map_core injects globally.
get_target_property(_gmc_inc grid_map_core::grid_map_core
  INTERFACE_INCLUDE_DIRECTORIES)

# ---------- gridmap_builder ----------
add_executable(gridmap_builder src/gridmap_builder.cpp)
ament_target_dependencies(gridmap_builder
  rclcpp grid_map_core grid_map_ros grid_map_msgs sensor_msgs)

# ---------- pointcloud_transformer ----------
add_executable(pointcloud_transformer src/pointcloud_transformer.cpp)
ament_target_dependencies(pointcloud_transformer
  rclcpp pcl_ros sensor_msgs tf2_ros tf2_eigen)
# Needed because grid_map_core sets a global EIGEN_FUNCTORS_PLUGIN definition
target_include_directories(pointcloud_transformer PRIVATE ${_gmc_inc})

install(TARGETS gridmap_builder pointcloud_transformer
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
