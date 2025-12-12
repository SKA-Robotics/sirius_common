cmake_minimum_required(VERSION 3.8)
project(sirius_calibration)

find_package(ament_cmake REQUIRED)

install(DIRECTORY kalibr allan_variance_ros DESTINATION share/${PROJECT_NAME})

ament_package()