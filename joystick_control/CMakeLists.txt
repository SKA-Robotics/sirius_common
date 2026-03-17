cmake_minimum_required(VERSION 3.8)
project(joystick_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Gamepad.msg"
  "msg/Topic.msg"
  "msg/TopicArray.msg"
  "srv/GetTopic.srv"
  "srv/GetTopicArray.srv"
  "srv/SendTopic.srv"
  DEPENDENCIES std_msgs
)

install(PROGRAMS
  scripts/joy_diff_drive.py
  scripts/joy_5dof_manipulator.py
  scripts/joy_multiplexer.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  joystick_control/
  DESTINATION lib/python3.12/site-packages/joystick_control
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
