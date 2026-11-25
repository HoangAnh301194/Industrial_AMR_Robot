cmake_minimum_required(VERSION 3.8)

project(factory_delivery_amr_navigation)

find_package(ament_cmake REQUIRED)

# ============================================================================
# INSTALL NAVIGATION RESOURCES (CONFIG, LAUNCH, MAPS, RVIZ)
# ============================================================================

install(
  DIRECTORY
    config
    launch
    maps
    rviz
  DESTINATION share/${PROJECT_NAME}
)

# ============================================================================
# INSTALL AUTONOMOUS MISSION & UTILITY SCRIPTS
# ============================================================================

install(
  PROGRAMS
    scripts/autonomous_mission.py
    scripts/save_warehouse_map.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
