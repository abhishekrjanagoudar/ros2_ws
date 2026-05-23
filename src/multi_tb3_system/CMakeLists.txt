cmake_minimum_required(VERSION 3.8)
project(multi_tb3_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ─── Dependencies ─────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# ─── Install Python Modules (scripts/) ─────────────────────────────────────────
ament_python_install_package(${PROJECT_NAME})

# Install individual Python scripts as executables
install(PROGRAMS
  scripts/follower_node.py
  scripts/teleop_controller.py
  scripts/laser_processor.py
  scripts/safety_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# ─── Install launch files ──────────────────────────────────────────────────────
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# ─── Install config files ─────────────────────────────────────────────────────
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ─── Install world files ──────────────────────────────────────────────────────
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# ─── Install models ──────────────────────────────────────────────────────────
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

# ─── Install rviz files ───────────────────────────────────────────────────────
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# ─── Install include headers ──────────────────────────────────────────────────
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
