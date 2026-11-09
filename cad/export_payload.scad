// ============================================================================
// FILE: export_payload.scad
// OUTPUT: payload_visual.stl
// PURPOSE: Export only the 20 kg payload tray.
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
// PARAMETERS REQUIRED BY payload_20kg()
// ============================================================================

// Original payload position
lift_center_x       = -15;
lift_lowered_deck_z = 330;


// Payload tray dimensions & side wing parameters (synchronized with main.scad)
payload_box_length       = 500;
payload_box_width        = 400;
payload_box_height       = 50;
rack_tray_wing_span_y    = 156;
rack_tray_wing_length    = 420;
rack_tray_wing_thickness = 16;
rack_tray_wing_z_offset  = 20;


// Disable debug labels during STL export
show_component_labels = false;


// ============================================================================
// PAYLOAD EXPORT
// ============================================================================
//
// In main.scad, payload_20kg(0) is positioned at:
//
// X = lift_center_x             = -15 mm
// Y = 0
// Z = lift_lowered_deck_z + 2 = 332 mm
//
// The translation moves the payload seating point to STL origin:
//
// X = 0
// Y = 0
// Z = 0
//
// In URDF, payload_link will be attached to lift_platform_link.
// ============================================================================

translate([
    -lift_center_x,
    0,
    -(lift_lowered_deck_z + 2)
])
    payload_20kg(0);