find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(
  test_load_behavior_plugins
  test_load_behavior_plugins.cpp
)
ament_target_dependencies(test_load_behavior_plugins ${THIS_PACKAGE_INCLUDE_DEPENDS})
