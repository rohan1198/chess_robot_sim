cmake_minimum_required(VERSION 3.22)
project(so_101_arm)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install directories
install(
  DIRECTORY 
    models
    urdf
    launch
    config
    scripts
    worlds
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
  PATTERN "setup_assistant.launch" EXCLUDE
)

install(
  DIRECTORY 
    models/chess_pawn_white
    models/chess_pawn_black
    models/chess_rook_white
    models/chess_rook_black
    models/chess_knight_white
    models/chess_knight_black
    models/chess_bishop_white
    models/chess_bishop_black
    models/chess_queen_white
    models/chess_queen_black
    models/chess_king_white
    models/chess_king_black
  DESTINATION share/${PROJECT_NAME}/models
  USE_SOURCE_PERMISSIONS
)

# Install Python scripts
install(PROGRAMS
  scripts/spawn_robot_gazebo.py
  scripts/chess_piece_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()
