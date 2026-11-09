use <main.scad>
lift_center_x         = -15;
lift_lowered_deck_z   = 330;
payload_box_height    = 50;
show_component_labels = false;
translate([-lift_center_x, 0, -(lift_lowered_deck_z + 2)])
    payload_weights(0);
