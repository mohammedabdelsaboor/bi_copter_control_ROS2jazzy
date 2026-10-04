# ==============================================================================
# BI-COPTER ROS 2 PACKAGE BUILD CONFIGURATION (CMakeLists.txt)
# ==============================================================================
#
# Hello developer! This file configures the build, installation, and packaging
# for the `bi_copter_control` ROS 2 Jazzy package.
#
# Key Configuration:
# - Uses `ament_cmake` and `ament_cmake_python` for a clean hybrid C++/Python workflow.
# - Installs the core Python nodes:
#     - `bicopter_controller_node.py` (Physics controller, Gazebo bridge, QGC MAVLink)
#     - `bicopter_teleop_node.py` (Interactive RC keyboard transmitter)
# - Installs all asset directories: launch/, config/, urdf/, meshes/, models/, worlds/
# ==============================================================================

cmake_minimum_required(VERSION 3.8)
project(bi_copter_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install Python executable nodes
install(PROGRAMS
  bi_copter_control/bicopter_controller_node.py
  bi_copter_control/bicopter_teleop_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install resource directories
install(DIRECTORY
  launch
  config
  urdf
  meshes
  models
  worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
