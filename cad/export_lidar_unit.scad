use <main.scad>
lidar_pos_x        = 368;
lidar_mount_height = 390;
lidar_diameter     = 76;
lidar_height       = 60;
show_component_labels = false;
translate([-lidar_pos_x, 0, -lidar_mount_height])
    lidar_unit();
