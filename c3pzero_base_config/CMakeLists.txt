cmake_minimum_required(VERSION 3.22)
project(c3pzero_base_config NONE)

find_package(ament_cmake REQUIRED)
find_package(c3pzero_description REQUIRED)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
