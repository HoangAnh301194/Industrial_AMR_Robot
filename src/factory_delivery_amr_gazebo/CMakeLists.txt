cmake_minimum_required(VERSION 3.8)

project(factory_delivery_amr_gazebo)

find_package(ament_cmake REQUIRED)

# ============================================================================
# INSTALL SIMULATION RESOURCES
# ============================================================================

install(
  DIRECTORY
    launch
    config
    worlds
    models
    rviz
  DESTINATION share/${PROJECT_NAME}
)

# ============================================================================
# INSTALL PYTHON SCRIPTS & CONTROLLERS
# ============================================================================

install(
  PROGRAMS
    scripts/lift_gazebo_controller.py
    scripts/dual_amr_teleop.py
    scripts/amr_keyboard_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================================================
# FINISH AMENT PACKAGE
# ============================================================================

ament_package()
