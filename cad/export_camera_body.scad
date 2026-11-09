use <main.scad>
camera_pos_x        = 415;
camera_mount_height = 285;
d455_length         = 26;
d455_width          = 124;
d455_height         = 29;
show_component_labels = false;
translate([-camera_pos_x, 0, -camera_mount_height])
    realsense_body();
