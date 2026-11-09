// ============================================================================
// FILE: export_charging_station.scad
// OUTPUT: charging_station_visual.stl
// PURPOSE: Export only the charging station.
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
// PARAMETERS REQUIRED BY charging_station()
// ============================================================================

// Robot size is used only to calculate the original station position.
robot_length = 850;

// Height of the charging contacts above the ground.
charging_contact_z = 95;

// Original station position in the complete scene:
//
// X = robot_length / 2 + 100
// X = 850 / 2 + 100
// X = 525 mm
charging_station_origin_x =
    robot_length / 2 + 100;


// ============================================================================
// CHARGING STATION EXPORT
// ============================================================================
//
// The charging_station() module already translates its geometry to:
//
// X = charging_station_origin_x
// Y = 0
// Z = 0
//
// The outer translation moves the station back to the STL origin.
//
// When using this mesh as a separate model in Gazebo, restore the station
// position with:
//
// pose = "0.525 0 0 0 0 0"
// ============================================================================

translate([
    -charging_station_origin_x,
    0,
    0
])
    charging_station();