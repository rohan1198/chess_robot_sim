cmake_minimum_required(VERSION 3.22)
project(so_101_arm)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY 
    models
    urdf
    launch
    config
    scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
  PATTERN "setup_assistant.launch" EXCLUDE
)

install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()
