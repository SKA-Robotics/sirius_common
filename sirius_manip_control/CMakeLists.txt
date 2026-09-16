cmake_minimum_required(VERSION 3.8)
project(sirius_manip_control)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(PROGRAMS
  scripts/manip_controller.py
  scripts/manip_interface.py
  scripts/manip_interface_ros.py
  scripts/manip_jointstate_aggregator.py
  scripts/voodoo_demultiplexer.py
  scripts/ik.py
  scripts/ik_6dof.py
  scripts/motion_interpolation.py
  scripts/motion_strategies.py
  scripts/home.py
  scripts/return_to_home.py
  scripts/hardware_interface/manip_gazebo_interface.py
  scripts/hardware_interface/manip_hardware_interface.py
  scripts/hardware_interface/manip_roboclaw_interface.py
  scripts/hardware_interface/manip_roboszpon_interface.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
