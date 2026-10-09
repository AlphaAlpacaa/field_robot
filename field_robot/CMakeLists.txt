cmake_minimum_required(VERSION 3.8)
project(field_robot)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY worlds launch urdf config
  DESTINATION share/${PROJECT_NAME}
)

# Every .py under scripts/ is installed as an executable, so adding a new one
# needs no edit here. CONFIGURE_DEPENDS makes CMake re-check the directory on
# each build instead of only when this file changes.
file(GLOB SCRIPTS CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.py)

# With --symlink-install the installed "executable" is a symlink to the source
# file, so the exec bit has to be set on the source. Editors and downloads
# routinely drop it, and then "ros2 run" reports "No executable found" for a
# script that is plainly there. Setting it on every build removes that whole
# class of confusion.
add_custom_target(make_scripts_executable ALL
  COMMAND ${CMAKE_COMMAND} -E echo "Marking scripts executable"
  COMMAND chmod +x ${SCRIPTS}
  VERBATIM
)

install(
  PROGRAMS ${SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
