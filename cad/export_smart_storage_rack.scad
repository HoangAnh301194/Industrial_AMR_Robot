export_only = true;
include <main.scad>
translate([-rack_center_x, -rack_center_y, 0])
    two_level_smart_rack(false, false, 0);
