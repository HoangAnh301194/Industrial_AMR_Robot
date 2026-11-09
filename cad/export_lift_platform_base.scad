use <main.scad>
lift_center_x        = -15;
lift_lowered_deck_z  = 330;
lift_deck_thickness  = 12;
lift_rail_x          = 170;
lift_rail_y          = 150;
lift_sleeve_diameter = 38;
payload_deck_length  = 650;
payload_deck_width   = 560;
payload_box_length   = 500;
payload_box_width    = 400;
show_component_labels = false;
translate([-lift_center_x, 0, -lift_lowered_deck_z])
    lift_platform_base(0);
