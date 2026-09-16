cmake_minimum_required(VERSION 3.8)
project(manip)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install the Python package (contains arm_servo, arm_joystick_control modules)
ament_python_install_package(manip
  PACKAGE_DIR scripts/src/manip)

install(PROGRAMS
  scripts/arm_controller_node.py
  scripts/gripper_node.py
  scripts/joystick_node.py
  scripts/mock_manip.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
