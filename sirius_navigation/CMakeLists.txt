cmake_minimum_required(VERSION 3.8)
project(sirius_navigation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(PROGRAMS
  scripts/autonomy.py
  scripts/imu_align_north.py
  scripts/localization_evaluation.py
  scripts/random_traversal.py
  scripts/simple_goal_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
