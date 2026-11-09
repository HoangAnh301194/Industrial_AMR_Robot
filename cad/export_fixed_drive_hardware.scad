// ============================================================================
// FILE: export_fixed_drive_hardware.scad
// OUTPUT: fixed_drive_hardware_visual.stl
// URDF LINK: fixed_drive_hardware_link
// ============================================================================

export_only = true;

include <main.scad>

union() {
    // Left fixed drive hardware
    gearbox(1);
    drive_motor(1);
    encoder_module(1);

    // Right fixed drive hardware
    gearbox(-1);
    drive_motor(-1);
    encoder_module(-1);
}