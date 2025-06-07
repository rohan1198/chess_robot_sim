cmake_minimum_required(VERSION 3.8)
project(so_arm_101_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(
  DIRECTORY
    config
    launch
    meshes
    urdf
    worlds
    rviz
    scripts
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/ps5_arm_controller.py
  scripts/test_joints.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
