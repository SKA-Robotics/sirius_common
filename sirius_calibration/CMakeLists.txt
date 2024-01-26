cmake_minimum_required(VERSION 3.16.3...3.28.1)
project(sirius_calibration)
find_package(catkin REQUIRED)
catkin_package()

# ==============================================================================
# Start of yaml processing code
# ==============================================================================

# Following code processes all yaml files in the config directory to make them
# compatible with both opencv and ros yaml parsers. This is required because
# opencv saves yaml files with %YAML:1.0 directive while most modern parsers
# support only YAML version 1.1 or higher. To solve this problem, we basically
# replace %YAML:1.0 in all yaml files with '%YAML 1.1\n---'

# Create CMake script that processes single yaml file
file(
  WRITE ${CMAKE_CURRENT_BINARY_DIR}/process_opencv_yaml.cmake
  [=[
    file(READ ${INPUT_FILE} YAML)
    string(REPLACE "%YAML:1.0" "%YAML 1.1\n---" MODIFIED_YAML "${YAML}")
    file(WRITE ${OUTPUT_FILE} "${MODIFIED_YAML}")
  ]=])

# Process only yaml files in the config directory. We GLOB yaml files, because
# at the end we are modifying source files which are later committed to git. I
# belive, in this case, there is a higher chance for an (user) error without
# GLOB. Modifying source files is a trait of ROS 1 (devel workspace). When
# migrating to ROS 2, following code should be modified accordingly.
file(
  GLOB_RECURSE
  YAML_FILES
  CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/config/*.yaml")

foreach(YAML_FILE ${YAML_FILES})
  file(
    RELATIVE_PATH
    YAML_OUTPUT_PATH
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${YAML_FILE})
  set(YAML_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR}/${YAML_OUTPUT_PATH})
  # We have to gather all output files to add them later as a dependency,
  # because add_dependencies only works with targets, not files
  list(APPEND OUTPUT_YAML_FILES ${YAML_OUTPUT_PATH})
  add_custom_command(
    OUTPUT ${YAML_OUTPUT_PATH}
    COMMAND "${CMAKE_COMMAND}" "-DINPUT_FILE=${YAML_FILE}" "-DOUTPUT_FILE=${YAML_OUTPUT_PATH}" -P
            "${CMAKE_CURRENT_BINARY_DIR}/process_opencv_yaml.cmake"
    # Copy the file to the original location to make it available in the devel
    COMMAND ${CMAKE_COMMAND} -E copy ${YAML_OUTPUT_PATH} ${YAML_FILE}
    # We update the timestamp on the declared output file, so cmake won't rerun
    # this command every time
    COMMAND ${CMAKE_COMMAND} -E touch ${YAML_OUTPUT_PATH}
    DEPENDS ${YAML_FILE}
    COMMENT "Processing ${YAML_FILE}")
endforeach()
# Without target, cmake would not run custom commands
add_custom_target(process_opencv_yaml ALL DEPENDS ${OUTPUT_YAML_FILES})

# ==============================================================================
# End of yaml processing code
# ==============================================================================
