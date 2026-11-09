// ============================================================================
// FILE: export_right_wheel.scad
// OUTPUT: right_wheel_visual.stl
// PURPOSE: Export only the right main drive wheel.
// ============================================================================

// Chi nap module va function.
// Khong chay factory_scene() trong main.scad.
use <main.scad>


// ============================================================================
// REQUIRED PARAMETERS
// Units: millimeters
// ============================================================================

eps = 0.05;

quality = "render";
preview_fn = 32;
render_fn  = 72;

$fn = quality == "render"
    ? render_fn
    : preview_fn;


// Main drive-wheel dimensions
drive_wheel_diameter = 220;
drive_wheel_radius   = drive_wheel_diameter / 2;
drive_wheel_width    = 60;

// Distance between left and right wheel centerlines
drive_wheel_track = 620;

// Wheel axle height above the ground
drive_wheel_axle_z = drive_wheel_radius;

// Disable labels
show_component_labels = false;


// ============================================================================
// RIGHT WHEEL EXPORT
// ============================================================================
//
// Original right-wheel center:
//
// X = 0 mm
// Y = -310 mm
// Z = 110 mm
//
// Move the wheel axle center to STL origin [0, 0, 0].
//
// URDF joint:
// origin xyz="0 -0.310 0.110"
// axis xyz="0 1 0"
// ============================================================================

translate([
    0,
    drive_wheel_track / 2,
    -drive_wheel_axle_z
])
    drive_wheel(
        -1,
        0
    );