cmake_minimum_required(VERSION 3.8)
project(multi_tb3_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ─── Build dependencies ────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# ─── Python package (multi_tb3_system/) ───────────────────────────────────────
# Installs multi_tb3_system/__init__.py and generate_sdf.py as an importable
# Python package.  Launch files import generate_robot_sdf from here.
ament_python_install_package(${PROJECT_NAME})

# ─── Python executables (scripts/) ────────────────────────────────────────────
# Note: costmap_utils.py, convoy_tracking.py, and follower_state.py are pure
# Python modules (no shebang) imported at runtime by the executable scripts via
# sys.path.insert of the script directory. Installing them via install(PROGRAMS)
# places them in lib/${PROJECT_NAME} alongside the executables so the imports
# resolve.
install(PROGRAMS
  scripts/convoy_publisher.py
  scripts/convoy_tracking.py
  scripts/costmap_generator.py
  scripts/costmap_utils.py
  scripts/follower_node.py
  scripts/follower_state.py
  scripts/laser_processor.py
  scripts/safety_controller.py
  scripts/teleop_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# ─── Launch files ──────────────────────────────────────────────────────────────
# All .launch.py files in launch/ are installed to the package share.
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# ─── Config files ─────────────────────────────────────────────────────────────
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ─── World files ──────────────────────────────────────────────────────────────
# Includes empty.world, pillars.world, columns.world (legacy).
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# ─── Model files ──────────────────────────────────────────────────────────────
# Local turtlebot3_burger model with topic patching support.
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

# ─── RViz config ──────────────────────────────────────────────────────────────
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# ─── Include headers (future C++ extensions) ──────────────────────────────────
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/include)
  install(DIRECTORY include/
    DESTINATION include
  )
endif()

# ─── Linting ──────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
