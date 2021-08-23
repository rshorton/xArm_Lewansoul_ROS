cmake_minimum_required(VERSION 3.5)
project(xarm_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config meshes launch urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

