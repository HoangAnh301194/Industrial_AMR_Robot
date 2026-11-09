use <main.scad>
drive_wheel_diameter = 220;
drive_wheel_radius   = 110;
drive_wheel_width    = 60;
drive_wheel_track    = 620;
drive_wheel_axle_z   = 110;
show_component_labels = false;
translate([0, -drive_wheel_track / 2, -drive_wheel_axle_z])
    drive_wheel_hub(1, 0);
