// ============================================================================
// FILE: export_lift_platform.scad
// OUTPUT: lift_platform_visual.stl
// PURPOSE:
// Export only the moving lift deck and four moving sleeves.
// Scissor bars are excluded and displayed dynamically in RViz2.
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

lift_center_x       = -15;
lift_lowered_deck_z = 330;
lift_deck_thickness = 12;

lift_rail_x          = 170;
lift_rail_y          = 150;
lift_sleeve_diameter = 38;

payload_deck_length = 650;
payload_deck_width  = 560;

payload_box_length = 500;
payload_box_width  = 400;

chassis_upper_z         = 250;
chassis_top_z0          = chassis_upper_z;
chassis_plate_thickness = 6;

chassis_top_z1 =
    chassis_top_z0 +
    chassis_plate_thickness;


// ============================================================================
// MOVING LIFT PLATFORM EXPORT
//
// Move the lowered platform joint frame to STL origin.
//
// false = do not include scissor bars in this STL.
// ============================================================================

translate([
    -lift_center_x,
    0,
    -lift_lowered_deck_z
])
    lift_moving_part(
        0,
        false
    );