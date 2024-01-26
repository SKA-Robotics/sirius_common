cmake_minimum_required(VERSION 3.16.3...3.28.1)
project(sirius_control)
find_package(catkin REQUIRED COMPONENTS combined_robot_hw controller_manager roscpp)

catkin_package(
  CATKIN_DEPENDS
  combined_robot_hw
  controller_manager
  roscpp)

include_directories(${catkin_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} src/control.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
