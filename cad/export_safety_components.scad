// ============================================================================
// FILE: export_safety_components.scad
// OUTPUT: safety_components_visual.stl
// URDF LINK: safety_components_link
// ============================================================================

export_only = true;

include <main.scad>

union() {
    emergency_stop();
    charging_contacts();
}