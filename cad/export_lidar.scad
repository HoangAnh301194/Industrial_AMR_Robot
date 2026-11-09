// ============================================================================
// FILE: export_lidar.scad
// OUTPUT: lidar_visual.stl
// PURPOSE: Export only the 2D LiDAR and its mounting pedestal.
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
// PARAMETERS REQUIRED BY lidar_2d()
// ============================================================================

// Main body height is used to calculate the pedestal height:
//
// pedestal_h = lidar_mount_height - robot_body_height
robot_body_height = 300;


// LiDAR position in the original robot coordinate system
lidar_pos_x        = 368;
lidar_mount_height = 390;


// LiDAR body dimensions
lidar_diameter = 76;
lidar_height   = 60;


// Disable component text labels during STL export
show_component_labels = false;


// ============================================================================
// LIDAR EXPORT
// ============================================================================
//
// Original LiDAR link frame:
//
// X = 368 mm
// Y = 0 mm
// Z = 390 mm
//
// The translation moves this frame to STL origin [0, 0, 0].
//
// The pedestal extends from:
//
// Z = 300 mm to Z = 390 mm
//
// Therefore, after moving the LiDAR frame to the STL origin,
// the pedestal extends from Z = -90 mm to Z = 0 mm.
//
// This is correct.
//
// In URDF, restore the LiDAR position with:
//
// origin xyz="0.368 0 0.390"
// ============================================================================

translate([
    -lidar_pos_x,
    0,
    -lidar_mount_height
])
    lidar_2d();