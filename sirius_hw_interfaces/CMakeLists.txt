cmake_minimum_required(VERSION 3.8)
project(sirius_hw_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(sirius_hw_interfaces SHARED
  src/sirius_wheels_hw.cpp
  src/sirius_manip_hw.cpp
)
target_include_directories(sirius_hw_interfaces PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(sirius_hw_interfaces
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
)

pluginlib_export_plugin_description_file(hardware_interface
  sirius_hw_interfaces.xml)

install(TARGETS sirius_hw_interfaces
  EXPORT export_sirius_hw_interfaces
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)
install(FILES sirius_hw_interfaces.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_sirius_hw_interfaces HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface pluginlib rclcpp rclcpp_lifecycle sensor_msgs)

ament_package()
