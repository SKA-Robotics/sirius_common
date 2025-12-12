cmake_minimum_required(VERSION 3.8)
project(sirius_description)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

install(DIRECTORY
  launch
  urdf
  meshes
  robots
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()