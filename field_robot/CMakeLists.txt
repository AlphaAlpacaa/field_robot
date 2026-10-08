cmake_minimum_required(VERSION 3.8)
project(field_robot)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY worlds launch urdf config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/wave_arm.py
    scripts/test_gripper.py
    scripts/tomato_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
