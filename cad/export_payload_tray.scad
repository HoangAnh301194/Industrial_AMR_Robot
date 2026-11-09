use <main.scad>
lift_center_x            = -15;
lift_lowered_deck_z      = 330;
payload_box_length       = 500;
payload_box_width        = 400;
payload_box_height       = 50;
rack_tray_wing_span_y    = 156;
rack_tray_wing_length    = 420;
rack_tray_wing_thickness = 16;
rack_tray_wing_z_offset  = 20;
show_component_labels    = false;
translate([-lift_center_x, 0, -(lift_lowered_deck_z + 2)])
    payload_tray_body(0);
