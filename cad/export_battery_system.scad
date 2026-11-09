// ============================================================================
// FILE: export_battery_system.scad
// OUTPUT: battery_system_visual.stl
// URDF LINK: battery_system_link
// ============================================================================

export_only = true;

include <main.scad>

union() {
    battery_pack();
    bms();
}