use <main.scad>
robot_body_height  = 300;
lidar_pos_x        = 368;
lidar_mount_height = 390;
show_component_labels = false;
translate([-lidar_pos_x, 0, -lidar_mount_height])
    lidar_pedestal();
