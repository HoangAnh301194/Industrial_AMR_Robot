use <main.scad>
lift_center_x        = -15;
lift_lowered_deck_z  = 330;
lift_deck_thickness  = 12;
payload_deck_length  = 650;
payload_deck_width   = 560;
show_component_labels = false;
translate([-lift_center_x, 0, -lift_lowered_deck_z])
    lift_deck_pad(0);
