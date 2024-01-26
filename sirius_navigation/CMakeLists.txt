cmake_minimum_required(VERSION 3.16.3...3.28.1)
project(sirius_navigation)
find_package(catkin REQUIRED COMPONENTS rospy)
catkin_package()

catkin_install_python(
  PROGRAMS
  scripts/imu_align_north.py
  scripts/noise_variance_estimator.py
  scripts/random_traversal.py
  DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION})
