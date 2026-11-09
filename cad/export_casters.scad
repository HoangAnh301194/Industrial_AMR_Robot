// ============================================================================
// FILE: export_casters.scad
// OUTPUT: casters_visual.stl
// URDF LINK: casters_visual_link
// ============================================================================

export_only = true;

include <main.scad>

union() {
    for (pt = caster_positions) {
        caster_assembly(
            pt,
            0,
            0
        );
    }
}