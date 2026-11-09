// ============================================================================
// FILE: export_left_wheel.scad
// OUTPUT: left_wheel_visual.stl
// PURPOSE: Export only the left main drive wheel.
// ============================================================================

// Chi nap module va function tu main.scad.
// Khong chay factory_scene() trong main.scad.
use <main.scad>


// ============================================================================
// RENDER SETTINGS
// Units: millimeters
// ============================================================================

eps = 0.05;

quality = "render";

preview_fn = 32;
render_fn  = 72;

$fn = quality == "render"
    ? render_fn
    : preview_fn;


// ============================================================================
// PARAMETERS REQUIRED BY drive_wheel()
// ============================================================================

// Main drive-wheel dimensions
drive_wheel_diameter = 220;
drive_wheel_width    = 60;

// Derived radius
drive_wheel_radius =
    drive_wheel_diameter / 2;

// Distance between the left and right wheel centerlines
drive_wheel_track = 620;

// Wheel axle height above the ground
drive_wheel_axle_z =
    drive_wheel_radius;

// Disable component labels
show_component_labels = false;


// ============================================================================
// LEFT WHEEL EXPORT
// ============================================================================
//
// Original left-wheel axle center:
//
// X = 0 mm
// Y = +310 mm
// Z = +110 mm
//
// Move the axle center to STL origin [0, 0, 0].
//
// URDF joint configuration:
//
// origin xyz="0 0.310 0.110"
// axis xyz="0 1 0"
// ============================================================================

translate([
    0,
    -drive_wheel_track / 2,
    -drive_wheel_axle_z
])
    drive_wheel(
        1,
        0
    );