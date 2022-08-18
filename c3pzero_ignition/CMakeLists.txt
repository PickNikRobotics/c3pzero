cmake_minimum_required(VERSION 3.22)
project(c3pzero_ignition)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(odom2tf_broadcaster
    src/odom2tf_broadcaster_node.cpp
)
target_include_directories(odom2tf_broadcaster PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  odom2tf_broadcaster
    geometry_msgs
    nav_msgs
    rclcpp
    tf2
    tf2_ros
)

# Install executable
install(TARGETS odom2tf_broadcaster DESTINATION lib/${PROJECT_NAME})

# Install packages
install(DIRECTORY worlds rviz launch DESTINATION share/${PROJECT_NAME})

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
