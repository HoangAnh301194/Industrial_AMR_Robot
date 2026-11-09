// ============================================================================
// FILE: export_chassis.scad
// OUTPUT: chassis_visual.stl
// URDF LINK: chassis_link
// ============================================================================

export_only = true;

include <main.scad>

union() {
    chassis_bottom();
    chassis_top();
    chassis_longitudinal_beams();
    chassis_cross_beams();
}