// ============================================================================
// FILE: export_camera.scad
// OUTPUT: camera_visual.stl
// PURPOSE: Export only the Intel RealSense D455 camera and mounting bracket.
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
// PARAMETERS REQUIRED BY realsense_d455()
// ============================================================================

// Camera position in the original robot coordinate system
camera_pos_x        = 415;
camera_mount_height = 285;


// Camera housing dimensions
d455_length = 26;
d455_width  = 124;
d455_height = 29;


// Disable component labels during STL export
show_component_labels = false;


// ============================================================================
// CAMERA EXPORT
// ============================================================================
//
// Original camera housing center:
//
// X = camera_pos_x        = 415 mm
// Y = 0 mm
// Z = camera_mount_height = 285 mm
//
// The translation moves the camera housing center to STL origin [0, 0, 0].
//
// In URDF, restore the camera position using:
//
// origin xyz="0.415 0 0.285"
// ============================================================================

translate([
    -camera_pos_x,
    0,
    -camera_mount_height
])
    realsense_d455();