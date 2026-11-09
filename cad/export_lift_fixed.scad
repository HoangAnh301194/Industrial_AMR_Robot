// ============================================================================
// FILE: export_lift_fixed.scad
// OUTPUT: lift_fixed_visual.stl
// PURPOSE:
// Export fixed lift frame, guide rails, motor, gearbox and outer actuator barrel.
// The moving silver ram is excluded.
// ============================================================================

use <main.scad>


// ============================================================================
// RENDER SETTINGS
// ============================================================================

eps = 0.05;

quality = "render";

preview_fn = 32;
render_fn  = 72;

$fn = quality == "render"
    ? render_fn
    : preview_fn;


// ============================================================================
// REQUIRED PARAMETERS
// Units: millimeters
// ============================================================================

robot_body_height       = 300;
bottom_ground_clearance = 35;
chassis_plate_thickness = 6;
chassis_upper_z         = 250;

chassis_bottom_z0 =
    bottom_ground_clearance;

chassis_bottom_z1 =
    chassis_bottom_z0 +
    chassis_plate_thickness;

chassis_top_z0 =
    chassis_upper_z;

chassis_top_z1 =
    chassis_top_z0 +
    chassis_plate_thickness;

battery_height = 124;

battery_top_z =
    chassis_bottom_z1 +
    battery_height;

lift_center_x       = -15;
lift_lowered_deck_z = 330;
lift_deck_thickness = 12;
lift_travel         = 290;

lift_fixed_base_z  = 180;
lift_rail_bottom_z = 55;

lift_rail_x          = 170;
lift_rail_y          = 150;
lift_rail_diameter   = 24;
lift_sleeve_diameter = 38;

show_component_labels = false;


// ============================================================================
// FIXED LIFT EXPORT
// ============================================================================

translate([
    -lift_center_x,
    0,
    0
])
union() {
    // Fixed structural frame
    lift_fixed_part();

    // Four fixed guide rails
    lift_guide_rails();

    // Fixed motor, gearbox and outer actuator barrel
    lift_actuator_fixed_part();
}