cmake_minimum_required(VERSION 3.22)
project(c3pzero_bringup NONE)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch param script
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
