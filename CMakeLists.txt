cmake_minimum_required(VERSION 3.8)
project(galbot_one_golf_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY meshes xacro config urdf launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  PROGRAMS
    scripts/create_description.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
