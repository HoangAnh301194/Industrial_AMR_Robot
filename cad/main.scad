// ============================================================================
// FILE: main.scad
// PROJECT: Factory Delivery AMR (Autonomous Mobile Robot)
// COURSE: ELE14119 - Robot Design (PTIT - AVIS Lab)
// ARCHITECTURE REFERENCE: warehouse-amr-ros2 (Modular Industrial Differential AMR)
// UNITS: millimeters (mm), degrees (deg), kilograms (kg), Watts (W), Volts (V)
// COORDINATE SYSTEM CONVENTION:
//   +X : Forward driving direction of the robot
//   +Y : Left side of the robot
//   +Z : Upward vertical direction
//   Origin (0, 0, 0) : Center of robot footprint projected onto the ground (Z=0)
// ============================================================================

// ============================================================================
// 1. GLOBAL SETTINGS
// ============================================================================
quality = "preview"; // Options: "preview" (fast F5) or "render" (smooth F6)
preview_fn = 32;
render_fn  = 72;
$fn = (quality == "render") ? render_fn : preview_fn;

eps = 0.05; // Small boolean overlap tolerance to guarantee clean 2-manifold CSG

// ============================================================================
// 2. VERIFIED SLIDE PARAMETERS (VERIFIED_SLIDE)
// Directly verified from ELE14119 Lecture Slides (Factory Delivery AMR Case Study)
// ============================================================================
payload_kg                = 20;     // [kg] R01: Minimum payload >= 20 kg
max_speed                 = 1.2;    // [m/s] R02: Max speed >= 1.2 m/s
max_speed_mm_s            = max_speed * 1000; // [mm/s] 1200 mm/s
corridor_width            = 1200;   // [mm] R05: Minimum factory corridor width
side_clearance            = 100;    // [mm] S02: Target minimum clearance per side
docking_error             = 30;     // [mm] R03: Stopping/docking accuracy <= +-30 mm
runtime                   = 4;      // [hours] R04: Minimum operating runtime >= 4 h
human_detection_range     = 3000;   // [mm] R06: Human detection range >= 3000 mm (3 m)
battery_voltage           = 48;     // [V] Slide 69/87: 48 V DC system bus
battery_capacity          = 30;     // [Ah] Slide 69: 48 V - 30 Ah Li-ion battery pack
battery_capacity_ah       = battery_capacity; // Alias for mandatory assertion check
average_power             = 245;    // [W] Slide 68: Total average power consumption
drive_motor_nominal_power = 180;    // [W] Slide 68: 180 W nominal per BLDC drive motor

// ============================================================================
// 3. REFERENCE CAD PARAMETERS (REFERENCE_CAD)
// Architectural traits from https://github.com/Pouya-Mansournia/warehouse-amr-ros2
// Note: No external binary mesh/URDF numbers are guessed from images.
// All exact geometric values are parameterized in Section 4 (DESIGN_ASSUMPTION).
// ============================================================================
ref_architecture_type     = "warehouse-amr-ros2_differential_modular";
ref_caster_count          = 4;      // 4 corner swivel casters (front-left/right, rear-left/right)
ref_drive_wheel_count     = 2;      // 2 central differential drive wheels (left/right)
ref_chassis_layer_count   = 2;      // Lower base plate + upper structural deck plate

// ============================================================================
// 4. DESIGN ASSUMPTIONS (DESIGN_ASSUMPTION)
// All non-slide dimensions are grouped here as ASSUMPTION for easy tuning.
// ============================================================================
robot_length              = 850;    // [mm] ASSUMPTION: Overall body length along X
robot_width               = 720;    // [mm] ASSUMPTION: Overall body width along Y (700..800 mm)
robot_body_height         = 300;    // [mm] ASSUMPTION: Main enclosure top height from ground
body_height               = robot_body_height; // Alias for assertion check
total_height_lift_lowered = 450;    // [mm] ASSUMPTION: Overall robot height (to LiDAR guard top)
chassis_plate_thickness   = 6;      // [mm] ASSUMPTION: Thickness of upper/lower chassis plates
bottom_ground_clearance   = 35;     // [mm] ASSUMPTION: Ground clearance below bottom plate
ground_clearance          = bottom_ground_clearance; // Alias for assertion check

drive_wheel_diameter      = 220;    // [mm] ASSUMPTION: Main drive wheel outer diameter
drive_wheel_width         = 60;     // [mm] ASSUMPTION: Main drive wheel width
drive_wheel_track         = 620;    // [mm] ASSUMPTION: Track width between L/R wheel centers

caster_wheel_diameter     = 100;    // [mm] ASSUMPTION: Swivel caster wheel diameter
caster_wheel_width        = 35;     // [mm] ASSUMPTION: Swivel caster wheel width
caster_offset_x           = 310;    // [mm] ASSUMPTION: Caster swivel axis X distance from center
caster_offset_y           = 245;    // [mm] ASSUMPTION: Caster swivel axis Y distance from center
caster_trail              = 24;     // [mm] ASSUMPTION: Caster horizontal trail offset

payload_deck_length       = 650;    // [mm] ASSUMPTION: Lift platform length along X
payload_deck_width        = 560;    // [mm] ASSUMPTION: Lift platform width along Y
lift_travel               = 290;    // [mm] ASSUMPTION: Maximum vertical Z lift stroke (supports 2-level rack)

lidar_mount_height        = 390;    // [mm] ASSUMPTION: 2D LiDAR optical scan plane Z height
camera_mount_height       = 285;    // [mm] ASSUMPTION: Intel RealSense D455 center Z height

// Additional parametric sub-component assumptions (no magic numbers in modules)
body_corner_radius        = 80;     // [mm] ASSUMPTION: Outer body vertical corner fillet radius
chassis_margin            = 22;     // [mm] ASSUMPTION: Inset of internal chassis from outer shell
chassis_upper_z           = 250;    // [mm] ASSUMPTION: Bottom Z of upper chassis deck plate
shell_wall_thickness      = 4;      // [mm] ASSUMPTION: Outer fiberglass/sheet-metal skin thickness
bumper_height             = 46;     // [mm] ASSUMPTION: Perimeter safety bumper height
bumper_protrusion         = 10;     // [mm] ASSUMPTION: Bumper radial protrusion around lower rim

beam_width                = 16;     // [mm] ASSUMPTION: Structural longitudinal beam width
beam_height               = 28;     // [mm] ASSUMPTION: Structural beam section height
long_beam_y               = 118;    // [mm] ASSUMPTION: Y centerline of main longitudinal beams

motor_diameter            = 80;     // [mm] ASSUMPTION: 180W BLDC motor frame diameter
motor_length              = 74;     // [mm] ASSUMPTION: 180W BLDC motor body length
gearbox_diameter          = 82;     // [mm] ASSUMPTION: Planetary gearbox outer diameter
gearbox_length            = 52;     // [mm] ASSUMPTION: Planetary gearbox axial length
encoder_diameter          = 52;     // [mm] ASSUMPTION: Incremental optical encoder diameter
encoder_length            = 18;     // [mm] ASSUMPTION: Incremental optical encoder length

battery_length            = 270;    // [mm] ASSUMPTION: 48V 30Ah Li-ion pack length (X)
battery_width             = 210;    // [mm] ASSUMPTION: 48V 30Ah Li-ion pack width (Y)
battery_height            = 124;    // [mm] ASSUMPTION: 48V 30Ah Li-ion pack height (Z)
bms_length                = 160;    // [mm] ASSUMPTION: Smart BMS module length
bms_width                 = 95;     // [mm] ASSUMPTION: Smart BMS module width
bms_height                = 26;     // [mm] ASSUMPTION: Smart BMS module height

lift_center_x             = -15;    // [mm] ASSUMPTION: Slight rear shift of lift deck to clear LiDAR mast
lift_lowered_deck_z       = 330;    // [mm] ASSUMPTION: Lift deck top surface Z when lowered (30mm above body)
lift_deck_thickness       = 12;     // [mm] ASSUMPTION: Lift top plate thickness
lift_rail_x               = 170;    // [mm] ASSUMPTION: Guide rail X offset from lift center
lift_rail_y               = 150;    // [mm] ASSUMPTION: Guide rail Y offset from centerline
lift_rail_diameter        = 24;     // [mm] ASSUMPTION: Linear guide shaft diameter
lift_sleeve_diameter      = 38;     // [mm] ASSUMPTION: Linear bearing sleeve outer diameter
lift_fixed_base_z         = 180;    // [mm] ASSUMPTION: Lower bridge of lift fixed assembly (above battery)
lift_rail_bottom_z        = 55;     // [mm] ASSUMPTION: Bottom Z of 4 outer vertical guide rails

payload_box_length        = 500;    // [mm] ASSUMPTION: 20kg standard industrial tote length
payload_box_width         = 400;    // [mm] ASSUMPTION: 20kg standard industrial tote width
payload_box_height        = 50;     // [mm] ASSUMPTION: 20kg heavy metal component tray height

lidar_pos_x               = 368;    // [mm] ASSUMPTION: Front elevated LiDAR X position
lidar_diameter            = 76;     // [mm] ASSUMPTION: 2D LiDAR housing diameter
lidar_height              = 60;     // [mm] ASSUMPTION: 2D LiDAR unit height

camera_pos_x              = 415;    // [mm] ASSUMPTION: RealSense D455 X position at front nose
d455_length               = 26;     // [mm] ASSUMPTION: RealSense D455 depth (X)
d455_width                = 124;    // [mm] ASSUMPTION: RealSense D455 width (Y)
d455_height               = 29;     // [mm] ASSUMPTION: RealSense D455 height (Z)
d455_hfov_deg             = 86;     // [deg] ASSUMPTION: RealSense D455 horizontal FOV
d455_vfov_deg             = 57;     // [deg] ASSUMPTION: RealSense D455 vertical FOV
camera_debug_range        = 1200;   // [mm] ASSUMPTION: Displayed FOV pyramid length in debug mode

estop_pos_x               = -382;   // [mm] ASSUMPTION: E-stop X position on rear deck
estop_pos_y               = 210;    // [mm] ASSUMPTION: E-stop Y position on rear-left deck

docking_target_distance   = 900;
charging_contact_z        = 95;

// ============================================================================
// DOCKING GEOMETRY
// ============================================================================

// World X coordinate of charging station origin.
charging_station_origin_x = robot_length / 2 + 100;

// Station copper contact center is 8 mm toward the robot from station origin.
station_contact_center_x  = charging_station_origin_x - 8;

// Station contact cylinder has length 32 mm along X.
// Therefore, the robot-facing tip is 16 mm before its center.
station_contact_face_x    = station_contact_center_x - 16;

// Robot copper contact center:
// robot front edge + 2 mm housing offset + 5 mm contact offset.
robot_contact_center_local_x =
    robot_length / 2 + 2 + 5;

// Robot copper pad is 6 mm thick along X.
// Its front face is 3 mm forward from its center.
robot_contact_face_local_x =
    robot_contact_center_local_x + 3;

// Final robot-center position where both contact faces meet.
dock_robot_stop_x =
    station_contact_face_x -
    robot_contact_face_local_x;

// Small safety gap to prevent visual overlap in F5 preview.
dock_safety_gap = 2;

// Final safe docking position.
dock_robot_safe_x =
    dock_robot_stop_x -
    dock_safety_gap;

echo(str(
    "Docking station origin X : ",
    charging_station_origin_x,
    " mm"
));

echo(str(
    "Station contact face X   : ",
    station_contact_face_x,
    " mm"
));

echo(str(
    "Robot docking stop X     : ",
    dock_robot_safe_x,
    " mm"
));
// ============================================================================
// TWO-LEVEL SMART STORAGE RACK
// FUNCTION 9 (PICKUP BOTTOM-TO-TOP) & FUNCTION 10 (PLACE TOP-TO-BOTTOM)
// Units: millimeters
// ============================================================================

// Rack center in world coordinates.
rack_center_x = 0;
rack_center_y = 0;

// Robot X positions for rack operations:
// - rack_dock_robot_x: aligns robot lift center (lift_center_x = -15) exactly with rack_center_x (0)
// - rack_approach_robot_x: pre-entry stop outside rack where tray & wings are 100% clear of rack posts
// - rack_start_robot_x: staging/home position outside the rack
rack_dock_robot_x     = rack_center_x - lift_center_x; // +15 mm
rack_approach_robot_x = -750;                          // -750 mm
rack_start_robot_x    = -1050;                         // -1050 mm

// Overall rack tunnel dimensions (allows AMR width 720 mm and length 850 mm to drive inside safely).
rack_width  = 960;    // [mm] Length along X (tunnel length, fits robot length 850 mm)
rack_depth  = 920;    // [mm] Width along Y (inner clear width 840 mm between 40 mm posts, fits robot width 720 mm)
rack_height = 820;    // [mm] Overall height (ample headroom above lifted Level 2 tray)

// Structural dimensions.
rack_post_size       = 40;
rack_beam_height     = 40;
rack_shelf_thickness = 16;

// Rack support rail ("2 thanh để khay") dimensions:
// Placed at |Y| = 302..358 mm (center Y = +-330 mm), giving 22 mm lateral clearance
// outside the robot's moving lift deck (|Y| <= 280 mm) and inside the rack posts (|Y| >= 420 mm).
rack_rail_y          = 330;    // [mm] Y center of left/right tray support rails
rack_rail_width      = 56;     // [mm] Support rail width along Y (spans |Y| = 302..358 mm)
rack_rail_height     = 16;     // [mm] Support rail vertical thickness
rack_rail_length     = 520;    // [mm] Support rail length along X (-260..+260 mm)

// Rack tray & side wing ("2 cánh chìa ra vừa thanh để khay") dimensions:
// Tray body keeps its original 500 x 400 x 50 mm shape, plus 2 side wings extending
// from |Y| = 200 mm out to |Y| = 356 mm so they rest squarely on the 2 support rails (|Y| = 302..358 mm).
rack_tray_length         = payload_box_length; // 500 mm
rack_tray_width          = payload_box_width;  // 400 mm
rack_tray_height         = payload_box_height; // 50 mm
rack_tray_wing_span_y    = 156;                // [mm] Wing protrusion per side (from Y=200 to Y=356 mm)
rack_tray_wing_length    = 420;                // [mm] Wing length along X (-210..+210 mm)
rack_tray_wing_thickness = 16;                 // [mm] Wing vertical thickness
rack_tray_wing_z_offset  = 20;                 // [mm] Height of wing bottom above tray bottom surface

// Robot lift targets & vertical clearances (collision-free with robot body & front LiDAR):
// - Robot lowered tray bottom sits at Z = lift_lowered_deck_z + 2 = 332 mm.
// - Front 2D LiDAR top cap is at Z = 450 mm.
// - Level 1 (kệ dưới):
//     rack_level_1_lift = 133 mm -> Tray 1 bottom at Z = 332 + 133 = 465 mm (15 mm clear above LiDAR top 450 mm!)
//     Level 1 rail top (rack_level_1_z) = 465 + 20 = 485 mm (rail bottom at 469 mm, 19 mm above LiDAR top!)
//     Tray 1 top = 465 + 50 = 515 mm.
// - Level 2 (kệ trên):
//     rack_level_2_lift = 268 mm -> Tray 2 bottom at Z = 332 + 268 = 600 mm (85 mm clear above Tray 1 top 515 mm!)
//     Level 2 rail top (rack_level_2_z) = 600 + 20 = 620 mm (rail bottom at 604 mm).
//     Clear vertical gap between Level 1 rail top (485 mm) and Level 2 rail bottom (604 mm) = 119 mm!
// - Lift clearance when picking/placing a tray off/onto the rails:
//     rack_lift_clearance = 22 mm (lifts tray wings 22 mm clear above the rails while leaving 63 mm
//     vertical clearance between lifted Tray 1 top at 537 mm and Tray 2 bottom at 600 mm!).
rack_level_1_lift   = 133;
rack_level_2_lift   = 268;
rack_lift_clearance = 22;

rack_level_1_tray_bottom_z =
    lift_lowered_deck_z + 2 + rack_level_1_lift; // 465 mm

rack_level_2_tray_bottom_z =
    lift_lowered_deck_z + 2 + rack_level_2_lift; // 565 mm

rack_level_1_z =
    rack_level_1_tray_bottom_z +
    rack_tray_wing_z_offset; // 485 mm (top surface of Level 1 support rail)

rack_level_2_z =
    rack_level_2_tray_bottom_z +
    rack_tray_wing_z_offset; // 585 mm (top surface of Level 2 support rail)

// --------------------------------------------------------------------------
// Rack occupancy states (used when inspecting static/single-step state).
// In animation_mode == 9 (Pickup bottom-to-top), the rack starts with BOTH
// trays occupied (true, true) and the robot picks Level 1 first, then Level 2.
// In animation_mode == 10 (Place top-to-bottom), the rack starts with BOTH
// levels empty (false, false) and the robot places Level 2 first, then Level 1.
// --------------------------------------------------------------------------

rack_level_1_occupied = true;
rack_level_2_occupied = true;

// --------------------------------------------------------------------------
// Operation mode.
//
// "pickup" = robot takes trays from bottom to top (Level 1 -> Level 2).
// "place"  = robot places trays from top to bottom (Level 2 -> Level 1).
// --------------------------------------------------------------------------

rack_operation = "pickup";

// --------------------------------------------------------------------------
// Preferred level.
//
// 0 = automatic sequential selection (Mode 9: 1 -> 2, Mode 10: 2 -> 1).
// 1 = force level 1 only.
// 2 = force level 2 only.
// --------------------------------------------------------------------------

rack_preferred_level = 0;

// Show rack sensors and status indicators.
show_rack_sensors = true;
show_rack_labels  = true;

// ============================================================================
// 5. DISPLAY AND DEBUG SWITCHES
// ============================================================================
show_body                 = true;
show_chassis              = true;
show_drive                = true;
show_casters              = true;
show_battery              = true;
show_electronics          = true;
show_sensors              = true;
show_lift                 = true;
show_payload              = true;
show_fasteners            = true;
show_charging_station     = false;
show_corridor             = false;
show_turning_envelope     = false;
show_lidar_scan           = false;
show_camera_fov           = false;
show_support_polygon      = false;
show_coordinate_axes      = false;
show_component_labels     = false;
collision_debug           = false;
exploded_view             = false;

// ============================================================================
// ============================================================================
// ============================================================================
// 6. ANIMATION PARAMETERS
// ============================================================================
//
// 0 = Static model
// 1 = Drive straight forward
// 2 = Spin in place
// 3 = Circular arc trajectory
// 4 = Lift table raise and lower
// 5 = Caster dynamic swivel demo
// 6 = Approach and dock with charging station
// 7 = Complete factory delivery mission
// 8 = Exploded assembly animation
// 9 = Smart two-level rack pickup mission (Robot lấy/nhận khay từ kệ mang đi)
// 10 = Smart two-level rack place mission (Robot chở khay đến cất/trả vào kệ)
//
// For animation mode 7:
//   t = 0.00 : Fully assembled
//   t = 0.00 to 0.38 : Sequential disassembly
//   t = 0.38 to 0.62 : Hold exploded state
//   t = 0.62 to 1.00 : Sequential reassembly
//
// Open View -> Animate
// FPS   = 30
// Steps = 200
// ============================================================================

animation_mode = 9;

// Automatic operation selection based on animation mode:
//   Mode 9  : "pickup" (Lấy/nhận khay từ kệ)
//   Mode 10 : "place"  (Cất/trả khay vào kệ)
effective_rack_operation =
    animation_mode == 10
    ? "place"
    :
    animation_mode == 9
    ? "pickup"
    :
    rack_operation;

straight_travel_dist = 1000;
arc_turn_radius      = 850;
arc_turn_angle_deg   = 75;

// Maximum separation distance used by exploded animation.
// This value must not depend on exploded_view.
explode_gap = 220;
// 7. DERIVED DIMENSIONS
// ============================================================================
drive_wheel_radius        = drive_wheel_diameter / 2;
caster_wheel_radius       = caster_wheel_diameter / 2;

// Ground contact elevations (must both equal 0 mm)
drive_wheel_axle_z        = drive_wheel_radius;
caster_wheel_axle_z       = caster_wheel_radius;
drive_wheel_ground_z      = drive_wheel_axle_z - drive_wheel_radius;
caster_wheel_ground_z     = caster_wheel_axle_z - caster_wheel_radius;

// Chassis plate elevations
chassis_bottom_z0         = bottom_ground_clearance;
chassis_bottom_z1         = chassis_bottom_z0 + chassis_plate_thickness;
chassis_top_z0            = chassis_upper_z;
chassis_top_z1            = chassis_top_z0 + chassis_plate_thickness;
chassis_length            = robot_length - 2 * chassis_margin;
chassis_width             = robot_width - 2 * chassis_margin;

// Corridor & Turning Envelope calculations
max_allowed_robot_width   = corridor_width - 2 * side_clearance;
remaining_corridor_width  = corridor_width - robot_width;
actual_side_clearance     = remaining_corridor_width / 2;
sweep_diameter            = sqrt(pow(robot_length, 2) + pow(robot_width, 2));
sweep_radius              = sweep_diameter / 2;

// Drive assembly lateral span (from wheel inner face towards center)
wheel_inner_y             = (drive_wheel_track / 2) - (drive_wheel_width / 2);
wheel_outer_y             = (drive_wheel_track / 2) + (drive_wheel_width / 2);
gearbox_inner_y           = wheel_inner_y - 4 - gearbox_length;
motor_inner_y             = gearbox_inner_y - motor_length;
encoder_inner_y           = motor_inner_y - encoder_length;

// Battery & Lift envelope boundaries for collision verification
battery_max_y             = battery_width / 2;
battery_top_z             = chassis_bottom_z1 + battery_height;
lift_guide_min_x          = abs(lift_center_x) + lift_rail_x - lift_sleeve_diameter / 2;
caster_mount_plate_z      = 125; // Raised internal caster bridge height

// Positions of the 4 corner casters: [X, Y]
caster_positions = [
    [ caster_offset_x,  caster_offset_y], // 0: Front-Left (FL)
    [ caster_offset_x, -caster_offset_y], // 1: Front-Right (FR)
    [-caster_offset_x,  caster_offset_y], // 2: Rear-Left (RL)
    [-caster_offset_x, -caster_offset_y]  // 3: Rear-Right (RR)
];

// ============================================================================
// 8. VALIDATION ASSERTIONS
// Mandatory engineering & geometric checks executed prior to model generation
// ============================================================================
echo("================================================================");
echo("FACTORY DELIVERY AMR - GEOMETRIC & SYSTEM VERIFICATION REPORT");
echo(str("  Robot Footprint (L x W)      : ", robot_length, " x ", robot_width, " mm"));
echo(str("  Corridor Width               : ", corridor_width, " mm"));
echo(str("  Remaining Corridor Width     : ", remaining_corridor_width, " mm"));
echo(str("  Actual Side Clearance (each) : ", actual_side_clearance, " mm (Min required: ", side_clearance, " mm)"));
echo(str("  Turning Sweep Diameter       : ", sweep_diameter, " mm (Limit: ", corridor_width, " mm)"));
echo(str("  In-Place Turn in 1200mm      : ", (sweep_diameter <= corridor_width) ? "PASS (SAFE)" : "WARNING: EXCEEDS CORRIDOR!"));
echo(str("  Drive & Caster Ground Z      : ", drive_wheel_ground_z, " mm / ", caster_wheel_ground_z, " mm"));
echo(str("  Motor-to-Battery Y Clearance : ", encoder_inner_y - battery_max_y, " mm"));
echo(str("  Lift-to-Battery Z Clearance  : ", lift_fixed_base_z - battery_top_z, " mm"));
echo("================================================================");

if (sweep_diameter > corridor_width) {
    echo("WARNING: sweep_diameter exceeds corridor_width (1200 mm)! Robot cannot spin in place safely!");
}
if (collision_debug) {
    echo("COLLISION DEBUG MODE ACTIVE: Highlighting critical clearance zones in transparent red.");
}

assert(robot_width <= corridor_width - 2 * side_clearance,
    "ASSERT FAILED: robot_width exceeds maximum allowed width for corridor side clearance!");
assert(ground_clearance > 0,
    "ASSERT FAILED: ground_clearance must be strictly greater than 0 mm!");
assert(drive_wheel_diameter > caster_wheel_diameter,
    "ASSERT FAILED: drive_wheel_diameter must be larger than caster_wheel_diameter!");
assert(lift_travel >= 0,
    "ASSERT FAILED: lift_travel must be non-negative!");
assert(battery_voltage == 48,
    "ASSERT FAILED: battery_voltage must equal 48 V per slide specification!");
assert(battery_capacity_ah >= 30,
    "ASSERT FAILED: battery_capacity_ah must be at least 30 Ah for 4h runtime!");
assert(payload_kg >= 20,
    "ASSERT FAILED: payload_kg must be at least 20 kg per R01 requirement!");
assert(lidar_mount_height > body_height,
    "ASSERT FAILED: lidar_mount_height must be higher than body_height to prevent occlusion!");
assert(drive_wheel_track < robot_width,
    "ASSERT FAILED: drive_wheel_track must be narrower than robot_width!");
assert(payload_deck_width <= robot_width,
    "ASSERT FAILED: payload_deck_width must not exceed robot_width!");
assert(payload_deck_length <= robot_length,
    "ASSERT FAILED: payload_deck_length must not exceed robot_length!");
assert(encoder_inner_y > battery_max_y && wheel_outer_y < robot_width / 2,
    "ASSERT FAILED: Drive motor and gearbox exceed allowed chassis envelope or collide with battery!");
assert(lift_fixed_base_z > battery_top_z && lift_rail_y > battery_max_y,
    "ASSERT FAILED: Battery pack intersects lift mechanism envelope!");
assert(abs(drive_wheel_ground_z - caster_wheel_ground_z) < 0.001,
    "ASSERT FAILED: Caster wheels and drive wheels do not share the same ground contact elevation Z=0!");
assert(rack_level_1_tray_bottom_z > total_height_lift_lowered,
    "ASSERT FAILED: Level 1 tray bottom collides with front 2D LiDAR cap!");
assert(rack_level_1_z - rack_rail_height > total_height_lift_lowered,
    "ASSERT FAILED: Level 1 support rail collides with robot body or LiDAR!");
assert(rack_rail_y - rack_rail_width / 2 > payload_deck_width / 2,
    "ASSERT FAILED: Rack support rails intersect moving lift platform width!");
assert(rack_level_2_tray_bottom_z > rack_level_1_tray_bottom_z + rack_tray_height + rack_lift_clearance,
    "ASSERT FAILED: Insufficient vertical clearance between lifted Level 1 tray and Level 2 tray!");

// ============================================================================
// 9. UTILITY FUNCTIONS & HELPER MODULES
// ============================================================================

// Normalize global time t into [0..1] for a specific animation window [start..end]
function local_time(t, start, end) =
    (t <= start) ? 0 :
    (t >= end)   ? 1 :
    (t - start) / (end - start);

// Hermite smoothstep for jerk-free acceleration and deceleration
function smoothstep(x) =
    let(c = max(0, min(1, x)))
    c * c * (3 - 2 * c);
// ============================================================================
// EXPLODED ASSEMBLY ANIMATION FUNCTIONS
// ============================================================================

// Clamp a value into the [0, 1] interval.
function clamp01(x) =
    max(0, min(1, x));


// Smooth transition between two time values.
function timed_smoothstep(t, start_time, end_time) =
    smoothstep(
        clamp01(
            (t - start_time) /
            max(0.0001, end_time - start_time)
        )
    );


// Sequential exploded animation.
//
// During the first half of the animation, each component moves outward.
//
// During the second half, components return in reverse order.
//
// start_time:
// Time when the component begins moving outward.
//
// duration:
// Length of the component movement.
//
// Return:
// 0 = assembled position
// 1 = fully exploded position
function exploded_component_factor(
    t,
    start_time,
    duration = 0.10
) =
    t < 0.50
    ? timed_smoothstep(
        t,
        start_time,
        start_time + duration
      )
    :
    1 -
    timed_smoothstep(
        t,
        1 - start_time - duration,
        1 - start_time
    );


// Manual exploded-view factor.
// Animation mode 8 has its own individual factors.
function manual_explode_factor() =
    exploded_view && animation_mode != 8
    ? 1
    : 0;

// Linear interpolation
function lerp(a, b, u) = a + (b - a) * u;

// Convert arc distance (mm) to wheel rotation angle (deg)
function dist_to_wheel_deg(dist_mm, diameter_mm) =
    (dist_mm / (PI * diameter_mm)) * 360;

// Tangent swivel angle (deg) for caster at [cx, cy] spinning in place (d_yaw > 0)
function spin_caster_swivel_deg(cx, cy, dir_sign) =
    atan2(dir_sign * cx, -dir_sign * cy);

// Tangent swivel angle (deg) for caster at [cx, cy] turning along arc of radius R
function arc_caster_swivel_deg(cx, cy, R) =
    atan2(cx, R - cy);

// Module: rounded_box
// Description: Generates a box of [sx, sy, sz] with vertical edges filleted by 'radius'.
// Coordinate system: Centered at origin if center==true, else min corner at [0,0,0].
module rounded_box(size, radius, center = true) {
    sx = size[0];
    sy = size[1];
    sz = size[2];
    r  = min(radius, min(sx, sy) / 2 - eps);
    tx = center ? 0 : sx / 2;
    ty = center ? 0 : sy / 2;
    tz = center ? 0 : sz / 2;

    translate([tx, ty, tz])
    hull() {
        for (dx = [-1, 1], dy = [-1, 1]) {
            translate([dx * (sx / 2 - r), dy * (sy / 2 - r), 0])
                cylinder(r = r, h = sz, center = true);
        }
    }
}

// Helper module for 3D text labels in debug mode
module component_label(txt, pos = [0, 0, 0]) {
    if (show_component_labels) {
        translate(pos)
        color([1.0, 1.0, 0.2])
        rotate([90, 0, 0])
            linear_extrude(height = 1.5)
                text(txt, size = 16, halign = "center", valign = "center");
    }
}

// Helper module for coordinate axes visualization
module coordinate_axes_debug() {
    if (show_coordinate_axes) {
        // +X Axis (Red - Forward)
        color([0.9, 0.1, 0.1]) {
            rotate([0, 90, 0]) cylinder(r = 4, h = 550);
            translate([550, 0, 0]) rotate([0, 90, 0]) cylinder(r1 = 10, r2 = 0, h = 30);
        }
        // +Y Axis (Green - Left)
        color([0.1, 0.8, 0.2]) {
            rotate([-90, 0, 0]) cylinder(r = 4, h = 480);
            translate([0, 480, 0]) rotate([-90, 0, 0]) cylinder(r1 = 10, r2 = 0, h = 30);
        }
        // +Z Axis (Blue - Up)
        color([0.1, 0.4, 0.95]) {
            cylinder(r = 4, h = 600);
            translate([0, 0, 600]) cylinder(r1 = 10, r2 = 0, h = 30);
        }
    }
}
// ============================================================================
// FUNCTION 9 & 10: SMART RACK LEVEL SELECTION & SEQUENTIAL 2-TRAY LOGIC
// ============================================================================

// Return true when a level can be used for the requested operation.
function rack_level_is_available(
    occupied,
    operation
) =
    operation == "pickup"
    ? occupied
    :
    operation == "place"
    ? !occupied
    :
    false;

// Automatic selection rule:
// - Chức năng 9 (pickup): lấy khay từ dưới lên trên (khay dưới Level 1 -> khay trên Level 2)
// - Chức năng 10 (place): đặt khay từ trên xuống dưới (kệ trên Level 2 -> kệ dưới Level 1)
function rack_auto_select_level(
    operation,
    level_1_occupied,
    level_2_occupied
) =
    operation == "pickup"
    ? (
        level_1_occupied ? 1 :
        level_2_occupied ? 2 : 0
      )
    :
    operation == "place"
    ? (
        !level_2_occupied ? 2 :
        !level_1_occupied ? 1 : 0
      )
    :
    0;

// Validate a manually requested level.
function rack_requested_level_valid(
    requested_level,
    operation,
    level_1_occupied,
    level_2_occupied
) =
    requested_level == 1
    ? rack_level_is_available(level_1_occupied, operation)
    :
    requested_level == 2
    ? rack_level_is_available(level_2_occupied, operation)
    :
    false;

// Active target level at normalized animation time t in [0..1]:
// - Mode 9 (pickup bottom-to-top): Cycle 1 (t < 0.50) -> Level 1, Cycle 2 (t >= 0.50) -> Level 2
// - Mode 10 (place top-to-bottom): Cycle 1 (t < 0.50) -> Level 2, Cycle 2 (t >= 0.50) -> Level 1
function rack_active_level_at_t(mode, t) =
    rack_preferred_level != 0
    ? (
        rack_requested_level_valid(
            rack_preferred_level,
            effective_rack_operation,
            rack_level_1_occupied,
            rack_level_2_occupied
        )
        ? rack_preferred_level
        : rack_auto_select_level(
            effective_rack_operation,
            rack_level_1_occupied,
            rack_level_2_occupied
          )
      )
    :
    mode == 9
    ? (t < 0.50 ? 1 : 2)
    :
    mode == 10
    ? (t < 0.50 ? 2 : 1)
    :
    rack_auto_select_level(
        effective_rack_operation,
        rack_level_1_occupied,
        rack_level_2_occupied
    );

function rack_level_base_lift(level) =
    level == 1 ? rack_level_1_lift :
    level == 2 ? rack_level_2_lift : 0;

function rack_level_cleared_lift(level) =
    level == 1 ? (rack_level_1_lift + rack_lift_clearance) :
    level == 2 ? (rack_level_2_lift + rack_lift_clearance) : 0;

// Dynamic occupancy of Level 1 (kệ dưới) at animation time t:
// - Mode 9: starts occupied (true), picked up in Cycle 1 at t = 0.22 -> empty (false)
// - Mode 10: starts empty (false), placed in Cycle 2 at t = 0.80 -> occupied (true)
function rack_level_1_occupied_at_t(mode, t) =
    let(active_lvl = rack_active_level_at_t(mode, t))
    rack_preferred_level != 0
    ? (
        effective_rack_operation == "pickup" && active_lvl == 1 && t >= 0.44 ? false :
        effective_rack_operation == "place"  && active_lvl == 1 && t >= 0.60 ? true  :
        rack_level_1_occupied
      )
    :
    mode == 9  ? (t < 0.22) :
    mode == 10 ? (t >= 0.80) :
    rack_level_1_occupied;

// Dynamic occupancy of Level 2 (kệ trên) at animation time t:
// - Mode 9: starts occupied (true), remains occupied in Cycle 1, picked up in Cycle 2 at t = 0.72 -> empty (false)
// - Mode 10: starts empty (false), placed in Cycle 1 at t = 0.30 -> remains occupied (true)
function rack_level_2_occupied_at_t(mode, t) =
    let(active_lvl = rack_active_level_at_t(mode, t))
    rack_preferred_level != 0
    ? (
        effective_rack_operation == "pickup" && active_lvl == 2 && t >= 0.44 ? false :
        effective_rack_operation == "place"  && active_lvl == 2 && t >= 0.60 ? true  :
        rack_level_2_occupied
      )
    :
    mode == 9  ? (t < 0.72) :
    mode == 10 ? (t >= 0.30) :
    rack_level_2_occupied;

// Whether the robot carries a tray on its lift deck at animation time t:
function robot_carrying_tray_at_t(mode, t) =
    rack_preferred_level != 0
    ? (effective_rack_operation == "pickup" ? (t >= 0.44) : (t < 0.60))
    :
    mode == 9
    ? ((t >= 0.22 && t <= 0.48) || (t >= 0.72))
    :
    mode == 10
    ? ((t < 0.30) || (t >= 0.50 && t < 0.80))
    :
    true;

selected_rack_level =
    rack_active_level_at_t(animation_mode, $t);

selected_rack_lift_height =
    rack_level_base_lift(selected_rack_level);

selected_rack_level_text =
    selected_rack_level == 1 ? "LEVEL 1 (LOWER / KỆ DƯỚI)" :
    selected_rack_level == 2 ? "LEVEL 2 (UPPER / KỆ TRÊN)" :
    "NO VALID LEVEL";

echo("================================================================");
echo(str(
    "FUNCTION ",
    animation_mode,
    " - SMART TWO-LEVEL RACK (",
    effective_rack_operation == "place"
    ? "PLACE TOP-TO-BOTTOM / ĐẶT KHAY TỪ TRÊN XUỐNG DƯỚI"
    : "PICKUP BOTTOM-TO-TOP / LẤY KHAY TỪ DƯỚI LÊN TRÊN",
    ")"
));
echo(str("Operation          : ", effective_rack_operation));
echo(str("Level 1 rail Z     : ", rack_level_1_z, " mm (Tray bottom: ", rack_level_1_tray_bottom_z, " mm > LiDAR top 450 mm)"));
echo(str("Level 2 rail Z     : ", rack_level_2_z, " mm (Tray bottom: ", rack_level_2_tray_bottom_z, " mm, Clear gap: ", rack_level_2_z - rack_rail_height - rack_level_1_z, " mm)"));
echo(str("Active target level: ", selected_rack_level_text));
echo(str("Base / Cleared lift: ", selected_rack_lift_height, " / ", rack_level_cleared_lift(selected_rack_level), " mm"));
echo("================================================================");


// ============================================================================
// 10. FASTENER LIBRARY
// ============================================================================

// Module: fastener_m4
// Description: Socket head cap screw M4 with hex socket at given 3D position.
// Coordinate system: Head seat located at 'position', shank extending along -Z.
module fastener_m4(position = [0, 0, 0], length = 12) {
    translate(position)
    color([0.78, 0.80, 0.84]) {
        difference() {
            cylinder(d = 7.0, h = 4.0);
            translate([0, 0, 1.5]) cylinder(d = 3.5, h = 3.0, $fn = 6);
        }
        translate([0, 0, -length])
            cylinder(d = 4.0, h = length + eps);
    }
}

// Module: fastener_m5
// Description: Socket head cap screw M5 with washer and hex socket at 'position'.
// Coordinate system: Head seat located at 'position', shank extending along -Z.
module fastener_m5(position = [0, 0, 0], length = 16) {
    translate(position)
    color([0.74, 0.76, 0.80]) {
        cylinder(d = 10.0, h = 1.0); // Washer
        translate([0, 0, 1.0 - eps])
        difference() {
            cylinder(d = 8.5, h = 5.0);
            translate([0, 0, 2.0]) cylinder(d = 4.5, h = 3.5, $fn = 6);
        }
        translate([0, 0, -length])
            cylinder(d = 5.0, h = length + eps);
    }
}

// ============================================================================
// 11. CHASSIS MODULE
// Includes: chassis_bottom, chassis_top, chassis_longitudinal_beams, chassis_cross_beams
// ============================================================================

// Module: chassis_bottom
// Description: Lower 6mm laser-cut aluminum base plate with wheel wells, caster
//              swivel clearance pockets, battery mount slots, and M5 bolt pattern.
// Coordinate system: Robot frame, occupying Z = [chassis_bottom_z0 .. chassis_bottom_z1].
module chassis_bottom() {
    color([0.45, 0.48, 0.52])
    difference() {
        // Main lower plate body
        translate([0, 0, chassis_bottom_z0 + chassis_plate_thickness / 2])
            rounded_box([chassis_length, chassis_width, chassis_plate_thickness],
                        body_corner_radius - chassis_margin, true);

        // Left & Right drive wheel clearance cutouts (spans |Y| = 274..346 mm, leaving solid
        // inboard deck at |Y| <= 274 mm for the gearbox saddle bracket)
        for (side = [-1, 1]) {
            translate([0, side * (drive_wheel_track / 2), chassis_bottom_z0 + chassis_plate_thickness / 2])
                cube([drive_wheel_diameter + 28, drive_wheel_width + 12, chassis_plate_thickness + 4], center = true);
        }

        // 4 Corner caster swivel clearance pockets (R = 84 mm > 74 mm max caster sweep)
        for (pt = caster_positions) {
            translate([pt[0], pt[1], chassis_bottom_z0 - 1])
                cylinder(r = caster_trail + caster_wheel_radius + 10, h = chassis_plate_thickness + 2);
        }

        // Ventilation slots under front electronics bay (bridged by standoff rails)
        for (ix = [-2 : 2]) {
            translate([232 + ix * 20, 0, chassis_bottom_z0 + chassis_plate_thickness / 2])
                rounded_box([8, 140, chassis_plate_thickness + 4], 3, true);
        }

        // Mounting bolt holes along longitudinal beams
        for (sx = [-300, -150, 0, 150, 300], sy = [-1, 1]) {
            translate([sx, sy * long_beam_y, chassis_bottom_z0 - 1])
                cylinder(d = 5.2, h = chassis_plate_thickness + 2);
        }
    }
}

// Module: chassis_top
// Description: Upper 6mm structural deck plate supporting the lift mechanism,
//              IMU, LiDAR tower, camera bracket, and service access cutouts.
// Coordinate system: Robot frame, occupying Z = [chassis_top_z0 .. chassis_top_z1].
module chassis_top() {
    color([0.50, 0.53, 0.57])
    difference() {
        union() {
            translate([0, 0, chassis_top_z0 + chassis_plate_thickness / 2])
                rounded_box([chassis_length, chassis_width, chassis_plate_thickness],
                            body_corner_radius - chassis_margin, true);

            // Front nose structural tongue extending to X = 418 mm to support RealSense D455 bracket
            translate([395, 0, chassis_top_z0 + chassis_plate_thickness / 2])
                rounded_box([46, 136, chassis_plate_thickness], 8, true);
        }

        // 4 clearance holes for the vertical lift guide columns/sleeves
        for (dx = [-1, 1], dy = [-1, 1]) {
            translate([lift_center_x + dx * lift_rail_x, dy * lift_rail_y, chassis_top_z0 - 1])
                cylinder(d = lift_sleeve_diameter + 10, h = chassis_plate_thickness + 2);
        }

        // Central circular passage for ball-screw lift actuator barrel (leaves solid metal
        // at X = +45..+95 mm where the IMU module is mounted!)
        translate([lift_center_x, 0, chassis_top_z0 - 1])
            cylinder(d = 76, h = chassis_plate_thickness + 2);

        // Two side clearance slots for scissor stabilizer linkage bars
        for (sy = [-1, 1]) {
            translate([lift_center_x, sy * (lift_rail_y - 45), chassis_top_z0 + chassis_plate_thickness / 2])
                rounded_box([60, 28, chassis_plate_thickness + 4], 6, true);
        }

        // Cable routing & airflow pass-through slots placed at X = 250 mm, Y = +-175 mm
        // (completely separated from the [155, +-150] lift sleeve holes!)
        for (sy = [-1, 1]) {
            translate([250, sy * 175, chassis_top_z0 + chassis_plate_thickness / 2])
                rounded_box([64, 36, chassis_plate_thickness + 4], 8, true);
        }
    }
}

// Module: chassis_longitudinal_beams
// Description: Upper and lower heavy-duty aluminum extrusion beams running along X
//              with vertical structural stanchions and drive motor saddle brackets.
// Coordinate system: Robot frame, spanning between chassis_bottom and chassis_top.
module chassis_longitudinal_beams() {
    beam_len = chassis_length - 110;
    color([0.55, 0.58, 0.62]) {
        for (sy = [-1, 1]) {
            // Lower longitudinal rail on top of chassis_bottom
            translate([0, sy * long_beam_y, chassis_bottom_z1 + beam_height / 2])
                cube([beam_len, beam_width, beam_height], center = true);

            // Upper longitudinal rail under chassis_top
            translate([0, sy * long_beam_y, chassis_top_z0 - beam_height / 2])
                cube([beam_len, beam_width, beam_height], center = true);

            // Vertical structural pillars connecting lower and upper rails
            for (px = [-310, -195, 195, 310]) {
                translate([px, sy * long_beam_y, (chassis_bottom_z1 + chassis_top_z0) / 2])
                    cube([24, beam_width, chassis_top_z0 - chassis_bottom_z1], center = true);
            }

            // Drive gearbox & motor mounting bulkhead bracket at X = 0
            // Seated squarely on SOLID chassis_bottom at |Y| = wheel_inner_y - 19 = 261 mm
            // (mating flush against the inboard face of the gearbox square flange at |Y| = 266 mm)
            bracket_y = sy * (wheel_inner_y - 19);
            translate([0, bracket_y, (chassis_bottom_z1 + drive_wheel_axle_z + 46) / 2])
            difference() {
                rounded_box([104, 10, drive_wheel_axle_z + 46 - chassis_bottom_z1], 4, true);
                translate([0, 0, drive_wheel_axle_z - (chassis_bottom_z1 + drive_wheel_axle_z + 46) / 2])
                    rotate([90, 0, 0])
                        cylinder(d = gearbox_diameter + 1, h = 16, center = true);
            }

            // Two horizontal floor gusset ribs tying gearbox bulkhead to longitudinal beam
            for (gx = [-42, 42]) {
                translate([gx, sy * ((long_beam_y + wheel_inner_y - 19) / 2), chassis_bottom_z1 + 10])
                    cube([8, (wheel_inner_y - 19) - long_beam_y, 20], center = true);
            }
        }
    }
}

// Module: chassis_cross_beams
// Description: Transverse structural cross-beams, battery retention bay frame,
//              4 raised corner caster mounting bridges at Z = 125 mm, and
//              internal structural support pillars for LiDAR, E-stop, and charging port.
// Coordinate system: Robot frame.
module chassis_cross_beams() {
    color([0.48, 0.51, 0.55]) {
        // Transverse cross-beams front and rear of the battery pack
        for (cx = [-152, 152, -310, 310]) {
            translate([cx, 0, chassis_bottom_z1 + beam_height / 2])
                cube([beam_width, 2 * long_beam_y + beam_width, beam_height], center = true);
            // Upper cross-beams trimmed at cx = +-310 so they do not poke into caster bridges
            upper_span_y = (abs(cx) > 200) ? (2 * long_beam_y + beam_width) : (chassis_width - 160);
            translate([cx, 0, chassis_top_z0 - beam_height / 2])
                cube([beam_width, upper_span_y, beam_height], center = true);
        }

        // Battery side retention lips on chassis_bottom
        for (sy = [-1, 1]) {
            translate([0, sy * (battery_width / 2 + 5), chassis_bottom_z1 + 8])
                cube([battery_length + 12, 6, 16], center = true);
        }

        // 4 Raised Corner Caster Mounting Bridges (at Z = caster_mount_plate_z = 125 mm)
        for (pt = caster_positions) {
            sx = (pt[0] > 0) ? 1 : -1;
            sy = (pt[1] > 0) ? 1 : -1;

            // Top horizontal mounting bridge plate at Z = 125..133 mm (sized to stay inside shell)
            translate([pt[0], pt[1], caster_mount_plate_z + 4])
                rounded_box([104, 104, 8], 16, true);

            // Inboard vertical web tying caster bridge to chassis_bottom & longitudinal frame
            translate([pt[0] - sx * 48, pt[1] - sy * 48, (chassis_bottom_z1 + caster_mount_plate_z + 8) / 2])
                cylinder(d = 28, h = caster_mount_plate_z + 8 - chassis_bottom_z1, center = true);

            // Upper tie pillar from caster bridge up to chassis_top
            translate([pt[0] - sx * 40, pt[1] - sy * 40, (caster_mount_plate_z + 8 + chassis_top_z0) / 2])
                cylinder(d = 24, h = chassis_top_z0 - (caster_mount_plate_z + 8), center = true);
        }

        // Internal structural support column under Front 2D LiDAR tower (Z = 256..296 mm)
        translate([lidar_pos_x, 0, (chassis_top_z1 + robot_body_height - shell_wall_thickness) / 2])
            cylinder(d = 68, h = robot_body_height - shell_wall_thickness - chassis_top_z1, center = true);

        // Internal structural support column under Rear E-Stop button (Z = 256..296 mm)
        translate([estop_pos_x, estop_pos_y, (chassis_top_z1 + robot_body_height - shell_wall_thickness) / 2])
            cylinder(d = 44, h = robot_body_height - shell_wall_thickness - chassis_top_z1, center = true);

        // Front nose mounting bracket tying charging_contacts (X = 420, Z = 95) to chassis_bottom
        translate([392, 0, chassis_bottom_z1 + 36])
            rounded_box([36, 120, 72], 6, true);
    }
}

// ============================================================================
// 12. BODY SHELL MODULE
// Includes: body_shell, skin_panel, bumper_ring
// ============================================================================

// Module: skin_panel
// Description: Detachable industrial side/front/rear service skin panels with
//              recessed seam lines, cooling louvers, and LED accent strips.
//              Positioned proud of body_shell by 1.5 mm to prevent coplanar z-fighting.
// Coordinate system: Robot frame, side: 1=Left, -1=Right, 2=Front, -2=Rear.
module skin_panel(side = 1) {
    panel_z0 = bottom_ground_clearance + bumper_height + 6;
    panel_h  = robot_body_height - panel_z0 - 10;

    if (abs(side) == 1) {
        // Left (+1) or Right (-1) modular side service cover panel
        translate([0, side * (robot_width / 2 - 0.5), panel_z0 + panel_h / 2])
        color([0.12, 0.34, 0.62])
        difference() {
            rounded_box([robot_length - 2 * body_corner_radius + 16, 3, panel_h], 1.2, true);
            // Decorative industrial panel seam grooves
            for (gx = [-220, 220]) {
                translate([gx, side * 0.8, 0])
                    cube([3, 3, panel_h + 2], center = true);
            }
            // Cooling louvers near motor & motor driver bay (X = +95 mm)
            for (lz = [-35 : 14 : 35]) {
                translate([95, side * 0.6, lz])
                    cube([86, 4, 5], center = true);
            }
        }
        // Cyan industrial status light bar along side flank
        translate([0, side * (robot_width / 2 + 1.5), robot_body_height - 20])
        color([0.0, 0.85, 0.95])
            cube([robot_length - 2 * body_corner_radius - 40, 2, 6], center = true);
    } else if (side == 2) {
        // Front removable fascia panel (with cutout around charging contacts & below D455 bezel)
        front_panel_h = panel_h - 46;
        front_panel_z = panel_z0 + front_panel_h / 2;
        color([0.15, 0.38, 0.68])
        difference() {
            translate([robot_length / 2 - 0.5, 0, front_panel_z])
                rounded_box([3, robot_width - 2 * body_corner_radius - 24, front_panel_h], 1.2, true);
            // Cutout for front charging contacts housing (Z = 73..117 mm, Y = -72..+72 mm)
            translate([robot_length / 2, 0, charging_contact_z])
                rounded_box([16, 144, 44], 4, true);
        }
    } else if (side == -2) {
        // Rear removable maintenance fascia panel with cooling vents
        translate([-robot_length / 2 + 0.5, 0, panel_z0 + panel_h / 2])
        color([0.15, 0.38, 0.68])
        difference() {
            rounded_box([3, robot_width - 2 * body_corner_radius - 24, panel_h], 1.2, true);
            for (lz = [-40 : 16 : 40]) {
                translate([-0.6, 0, lz])
                    cube([4, 160, 6], center = true);
            }
        }
    }
}

// Module: bumper_ring
// Description: Perimeter impact-absorbing black elastomer bumper skirt around
//              the lower base of the robot with corner hazard markers.
// Coordinate system: Robot frame, near bottom edge of body shell.
module bumper_ring() {
    bz = bottom_ground_clearance + bumper_height / 2 + 2;
    color([0.12, 0.13, 0.15])
    difference() {
        translate([0, 0, bz])
            rounded_box([robot_length + 2 * bumper_protrusion,
                         robot_width + 2 * bumper_protrusion,
                         bumper_height],
                        body_corner_radius + bumper_protrusion, true);

        // Hollow interior mating cleanly with body_shell
        translate([0, 0, bz])
            rounded_box([robot_length - 6, robot_width - 6, bumper_height + 6],
                        body_corner_radius - 3, true);

        // Front charging contact recess window
        translate([robot_length / 2 + bumper_protrusion / 2, 0, charging_contact_z])
            rounded_box([bumper_protrusion * 3, 144, 44], 4, true);

        // Drive wheel side skirt relief slots
        for (sy = [-1, 1]) {
            translate([0, sy * (robot_width / 2 + bumper_protrusion / 2), bz - 10])
                cube([drive_wheel_diameter + 30, bumper_protrusion * 3, bumper_height], center = true);
        }
    }

    // Yellow industrial hazard stripes on front and rear bumper corners
    color([0.95, 0.78, 0.05])
    for (sx = [-1, 1], sy = [-1, 1]) {
        translate([sx * (robot_length / 2 - 22), sy * (robot_width / 2 - 22), bz])
            cylinder(r = 16, h = bumper_height - 10, center = true);
    }
}

// Module: body_shell_main
// Description: Main curved industrial AMR outer body shell without skin panels or bumper.
module body_shell_main() {
    shell_h  = robot_body_height - bottom_ground_clearance;
    shell_zc = bottom_ground_clearance + shell_h / 2;

    color([0.10, 0.26, 0.48]) {
        difference() {
            union() {
                // Outer rounded industrial shell
                translate([0, 0, shell_zc])
                    rounded_box([robot_length, robot_width, shell_h], body_corner_radius, true);

                // Protective upper camera hood brow at front nose (Z = 296..304 mm)
                translate([robot_length / 2 - 14, 0, robot_body_height + 1])
                    rounded_box([32, d455_width + 32, 6], 6, true);
            }

            // Hollow internal cavity leaving 6mm thick top deck & 4mm perimeter wall
            translate([0, 0, shell_zc - 6])
                rounded_box([robot_length - 2 * shell_wall_thickness,
                             robot_width - 2 * shell_wall_thickness,
                             shell_h],
                            body_corner_radius - shell_wall_thickness, true);

            // Central top opening for the lift columns & actuator (matches lift_fixed_part collar)
            translate([lift_center_x, 0, robot_body_height])
                rounded_box([2 * lift_rail_x + 62, 2 * lift_rail_y + 62, 30], 14, true);

            // Front recessed window for Intel RealSense D455 depth camera (kept below Z = 298 mm)
            translate([robot_length / 2 - 10, 0, camera_mount_height])
                rounded_box([36, d455_width + 12, d455_height + 6], 5, true);

            // Front recessed pocket for charging_contacts (Z = 75..115 mm, Y = -72..+72 mm)
            translate([robot_length / 2 - 6, 0, charging_contact_z])
                rounded_box([24, 144, 44], 4, true);

            // Wheel arch cutouts on lower left and right sides
            for (sy = [-1, 1]) {
                translate([0, sy * (drive_wheel_track / 2), drive_wheel_axle_z])
                    rotate([90, 0, 0])
                        cylinder(d = drive_wheel_diameter + 28, h = drive_wheel_width + 36, center = true);
            }
        }

        // Recessed camera pocket interior back/bottom shroud wall
        translate([robot_length / 2 - 16, 0, camera_mount_height])
        difference() {
            rounded_box([24, d455_width + 20, d455_height + 14], 6, true);
            translate([4, 0, 0])
                rounded_box([24, d455_width + 12, d455_height + 6], 4, true);
        }

        // Recessed charging-port interior back shroud wall
        translate([robot_length / 2 - 8, 0, charging_contact_z])
        difference() {
            rounded_box([16, 152, 50], 5, true);
            translate([4, 0, 0])
                rounded_box([16, 144, 44], 4, true);
        }

        // Reinforced circular mounting boss on top deck under 2D LiDAR tower & E-Stop
        translate([lidar_pos_x, 0, robot_body_height - 2])
            cylinder(d = 88, h = 6, center = true);
        translate([estop_pos_x, estop_pos_y, robot_body_height - 2])
            cylinder(d = 60, h = 6, center = true);
    }
}

// Module: body_skin_panels
module body_skin_panels() {
    panel_z0 = bottom_ground_clearance + bumper_height + 6;
    panel_h  = robot_body_height - panel_z0 - 10;

    for (side = [-1, 1]) {
        translate([0, side * (robot_width / 2 - 0.5), panel_z0 + panel_h / 2])
        color([0.12, 0.34, 0.62])
        difference() {
            rounded_box([robot_length - 2 * body_corner_radius + 16, 3, panel_h], 1.2, true);
            for (gx = [-220, 220]) {
                translate([gx, side * 0.8, 0])
                    cube([3, 3, panel_h + 2], center = true);
            }
            for (lz = [-35 : 14 : 35]) {
                translate([95, side * 0.6, lz])
                    cube([86, 4, 5], center = true);
            }
        }
    }

    front_panel_h = panel_h - 46;
    front_panel_z = panel_z0 + front_panel_h / 2;
    color([0.15, 0.38, 0.68])
    difference() {
        translate([robot_length / 2 - 0.5, 0, front_panel_z])
            rounded_box([3, robot_width - 2 * body_corner_radius - 24, front_panel_h], 1.2, true);
        translate([robot_length / 2, 0, charging_contact_z])
            rounded_box([16, 144, 44], 4, true);
    }

    translate([-robot_length / 2 + 0.5, 0, panel_z0 + panel_h / 2])
    color([0.15, 0.38, 0.68])
    difference() {
        rounded_box([3, robot_width - 2 * body_corner_radius - 24, panel_h], 1.2, true);
        for (lz = [-40 : 16 : 40]) {
            translate([-0.6, 0, lz])
                cube([4, 160, 6], center = true);
        }
    }
}

// Module: body_status_lights
module body_status_lights() {
    for (side = [-1, 1]) {
        translate([0, side * (robot_width / 2 + 1.5), robot_body_height - 20])
        color([0.0, 0.85, 0.95])
            cube([robot_length - 2 * body_corner_radius - 40, 2, 6], center = true);
    }
}

// Module: body_bumper_profile
module body_bumper_profile() {
    bz = bottom_ground_clearance + bumper_height / 2 + 2;
    color([0.12, 0.13, 0.15])
    difference() {
        translate([0, 0, bz])
            rounded_box([robot_length + 2 * bumper_protrusion,
                         robot_width + 2 * bumper_protrusion,
                         bumper_height],
                        body_corner_radius + bumper_protrusion, true);

        translate([0, 0, bz])
            rounded_box([robot_length - 6, robot_width - 6, bumper_height + 6],
                        body_corner_radius - 3, true);

        translate([robot_length / 2 + bumper_protrusion / 2, 0, charging_contact_z])
            rounded_box([bumper_protrusion * 3, 144, 44], 4, true);

        for (sy = [-1, 1]) {
            translate([0, sy * (robot_width / 2 + bumper_protrusion / 2), bz - 10])
                cube([drive_wheel_diameter + 30, bumper_protrusion * 3, bumper_height], center = true);
        }
    }
}

// Module: body_bumper_hazards
module body_bumper_hazards() {
    bz = bottom_ground_clearance + bumper_height / 2 + 2;
    color([0.95, 0.78, 0.05])
    for (sx = [-1, 1], sy = [-1, 1]) {
        translate([sx * (robot_length / 2 - 22), sy * (robot_width / 2 - 22), bz])
            cylinder(r = 16, h = bumper_height - 10, center = true);
    }
}

// Module: bumper_ring
module bumper_ring() {
    body_bumper_profile();
    body_bumper_hazards();
}

// Module: body_shell
module body_shell() {
    body_shell_main();
    body_skin_panels();
    body_status_lights();
    bumper_ring();
}

// ============================================================================
// 13, 14, 15. DRIVE MODULE, DRIVE WHEEL MODULE, MOTOR AND GEARBOX MODULE
// Includes: drive_wheel, gearbox, drive_motor, encoder_module, drive_assembly
// ============================================================================

// Module: drive_wheel_tire
module drive_wheel_tire(side = 1, wheel_angle = 0) {
    translate([0, side * (drive_wheel_track / 2), drive_wheel_axle_z])
    rotate([0, wheel_angle, 0]) {
        color([0.14, 0.14, 0.16])
        difference() {
            rotate([90, 0, 0])
                cylinder(d = drive_wheel_diameter, h = drive_wheel_width, center = true);
            rotate([90, 0, 0])
                cylinder(d = drive_wheel_diameter - 36, h = drive_wheel_width + 4, center = true);
            for (a = [0 : 30 : 330]) {
                rotate([0, a, 0])
                translate([drive_wheel_radius - 3, 0, 0])
                    cube([10, drive_wheel_width - 8, 8], center = true);
            }
        }
    }
}

// Module: drive_wheel_hub
module drive_wheel_hub(side = 1, wheel_angle = 0) {
    translate([0, side * (drive_wheel_track / 2), drive_wheel_axle_z])
    rotate([0, wheel_angle, 0]) {
        color([0.78, 0.80, 0.84])
        difference() {
            rotate([90, 0, 0])
                cylinder(d = drive_wheel_diameter - 35, h = drive_wheel_width - 8, center = true);
            for (a = [0 : 60 : 300]) {
                rotate([0, a, 0])
                translate([48, 0, 0])
                rotate([90, 0, 0])
                    cylinder(d = 26, h = drive_wheel_width, center = true);
            }
        }
        color([0.55, 0.58, 0.62])
        rotate([90, 0, 0])
            cylinder(d = 46, h = drive_wheel_width + 4, center = true);
    }
}

// Module: drive_wheel
module drive_wheel(side = 1, wheel_angle = 0) {
    drive_wheel_tire(side, wheel_angle);
    drive_wheel_hub(side, wheel_angle);
}

// Module: gearbox
// Description: Precision inline planetary gearbox connecting BLDC motor to drive wheel.
// Coordinate system: Coaxial with drive wheel axle along Y axis.
module gearbox(side = 1) {
    gb_center_y = side * (wheel_inner_y - 4 - gearbox_length / 2);
    translate([0, gb_center_y, drive_wheel_axle_z])
    rotate([side * 90, 0, 0]) {
        // Anodized dark-grey planetary gearbox housing
        color([0.32, 0.35, 0.38]) {
            cylinder(d = gearbox_diameter, h = gearbox_length, center = true);
            // Square mounting flange
            translate([0, 0, gearbox_length / 2 - 5])
                rounded_box([gearbox_diameter + 12, gearbox_diameter + 12, 10], 8, true);
        }
        // Output shaft coupling to wheel hub
        color([0.75, 0.77, 0.80])
        translate([0, 0, gearbox_length / 2 + 6])
            cylinder(d = 28, h = 14, center = true);
    }
}

// Module: drive_motor
// Description: 180W 48V Brushless DC (BLDC) servo motor with cooling fins and power plug.
// Coordinate system: Coaxial with gearbox along Y axis.
module drive_motor(side = 1) {
    mot_center_y = side * (gearbox_inner_y - motor_length / 2);
    translate([0, mot_center_y, drive_wheel_axle_z])
    rotate([side * 90, 0, 0]) {
        color([0.18, 0.20, 0.24]) {
            cylinder(d = motor_diameter, h = motor_length, center = true);
            // Radial cooling fins
            for (z_fin = [-motor_length / 2 + 10 : 10 : motor_length / 2 - 10]) {
                translate([0, 0, z_fin])
                    cylinder(d = motor_diameter + 5, h = 3, center = true);
            }
        }
        // Top electrical connector box on motor body
        color([0.25, 0.28, 0.32])
        translate([0, motor_diameter / 2 + 6, -motor_length / 4])
            cube([24, 16, 22], center = true);
    }
}

// Module: encoder_module
// Description: High-resolution incremental optical/magnetic rotary encoder on motor tail.
// Coordinate system: Coaxial on inner tail end of drive_motor.
module encoder_module(side = 1) {
    enc_center_y = side * (motor_inner_y - encoder_length / 2);
    translate([0, enc_center_y, drive_wheel_axle_z])
    rotate([side * 90, 0, 0])
    color([0.22, 0.45, 0.30]) {
        cylinder(d = encoder_diameter, h = encoder_length, center = true);
        // Feedback cable gland
        translate([0, encoder_diameter / 2 + 4, 0])
            cube([12, 10, 10], center = true);
    }
}

// Module: drive_assembly
// Description: Complete modular single-side differential drive unit combining
//              drive_wheel, gearbox, drive_motor, and encoder_module.
// Coordinate system: Robot frame, side = +1 (Left) or -1 (Right).
module drive_assembly(side = 1, wheel_angle = 0) {
    drive_wheel(side, wheel_angle);
    gearbox(side);
    drive_motor(side);
    encoder_module(side);
    component_label(side > 0 ? "LEFT_DRIVE_180W" : "RIGHT_DRIVE_180W",
                    [0, side * (drive_wheel_track / 2), drive_wheel_axle_z + 135]);
}

// ============================================================================
// 16. CASTER ASSEMBLY MODULE
// Includes: caster_fork, caster_assembly
// ============================================================================

// Module: caster_fork
// Description: Heavy-duty pressed steel swivel caster yoke (fork), top mounting
//              plate, vertical swivel bearing housing, and horizontal axle bolt.
// Coordinate system: Swivel axis at [position[0], position[1], Z = 50..125 mm].
module caster_fork(position = [0, 0, 0], swivel_angle = 0, wheel_angle = 0) {
    translate([position[0], position[1], 0]) {
        // Fixed top mounting plate bolted to underside of caster_mount_bridge at Z = 125 mm
        color([0.65, 0.68, 0.72])
        translate([0, 0, caster_mount_plate_z - 4])
            rounded_box([92, 92, 8], 10, true);

        // Swivel bearing raceway ring
        color([0.52, 0.55, 0.60])
        translate([0, 0, caster_mount_plate_z - 14])
            cylinder(d = 68, h = 12, center = true);

        // Rotating fork yoke (rotates around vertical Z axis by swivel_angle)
        rotate([0, 0, swivel_angle]) {
            color([0.60, 0.63, 0.68]) {
                // Fork crown plate
                translate([-caster_trail / 2, 0, caster_mount_plate_z - 23])
                    rounded_box([54 + caster_trail, 62, 6], 8, true);

                // Left and right fork legs trailing to [-caster_trail, +-Y, caster_wheel_axle_z]
                for (sy = [-1, 1]) {
                    hull() {
                        translate([-4, sy * (caster_wheel_width / 2 + 6), caster_mount_plate_z - 26])
                            cube([28, 6, 4], center = true);
                        translate([-caster_trail, sy * (caster_wheel_width / 2 + 6), caster_wheel_axle_z])
                            rotate([90, 0, 0])
                                cylinder(d = 26, h = 6, center = true);
                    }
                }
            }
            // Horizontal caster axle bolt
            color([0.30, 0.32, 0.35])
            translate([-caster_trail, 0, caster_wheel_axle_z])
                rotate([90, 0, 0])
                    cylinder(d = 12, h = caster_wheel_width + 22, center = true);
        }
    }
}

// Module: caster_assembly
// Description: Complete 360-deg swivel caster with fork and rolling polyurethane wheel.
// Coordinate system: Ground contact at Z = 0, top mount at Z = caster_mount_plate_z (125 mm).
module caster_assembly(position = [0, 0, 0], swivel_angle = 0, wheel_angle = 0) {
    caster_fork(position, swivel_angle, wheel_angle);

    // Caster wheel rolling around its horizontal axle at Z = caster_wheel_axle_z (50 mm)
    translate([position[0], position[1], caster_wheel_axle_z])
    rotate([0, 0, swivel_angle])
    translate([-caster_trail, 0, 0])
    rotate([0, wheel_angle, 0]) {
        // Dark polyurethane caster wheel tire
        color([0.16, 0.16, 0.18])
        difference() {
            rotate([90, 0, 0])
                cylinder(d = caster_wheel_diameter, h = caster_wheel_width, center = true);
            rotate([90, 0, 0])
                cylinder(d = caster_wheel_diameter - 26, h = caster_wheel_width + 2, center = true);
        }
        // Silver caster wheel core hub with spoke holes
        color([0.76, 0.78, 0.82])
        difference() {
            rotate([90, 0, 0])
                cylinder(d = caster_wheel_diameter - 25, h = caster_wheel_width - 4, center = true);
            for (a = [0 : 90 : 270]) {
                rotate([0, a, 0])
                translate([22, 0, 0])
                rotate([90, 0, 0])
                    cylinder(d = 12, h = caster_wheel_width, center = true);
            }
        }
    }
}

// ============================================================================
// 17. BATTERY MODULE
// Includes: battery_pack, bms
// ============================================================================

// Module: battery_pack
// Description: 48V 30Ah Li-ion battery pack mounted low at robot center (0, 0)
//              on chassis_bottom to minimize Center of Gravity (CoG) height.
// Coordinate system: Centered at (0, 0), Z = [chassis_bottom_z1 .. battery_top_z].
module battery_pack() {
    translate([0, 0, chassis_bottom_z1 + battery_height / 2]) {
        // Main heavy-duty sheet-metal battery enclosure (deep industrial blue/black)
        color([0.10, 0.18, 0.30])
            rounded_box([battery_length, battery_width, battery_height], 10, true);

        // Lifting handles on front and rear faces
        color([0.25, 0.28, 0.32])
        for (sx = [-1, 1]) {
            translate([sx * (battery_length / 2 + 6), 0, 15])
                cube([12, 80, 14], center = true);
        }

        // High-current Anderson-style +/- 48V power terminals on front face
        color([0.85, 0.12, 0.12])
        translate([battery_length / 2, 45, battery_height / 2 - 22])
            cube([16, 22, 22], center = true);
        color([0.12, 0.12, 0.12])
        translate([battery_length / 2, -45, battery_height / 2 - 22])
            cube([16, 22, 22], center = true);
    }
    component_label("BATTERY_48V_30AH", [0, 0, battery_top_z + 20]);
}

// Module: bms
// Description: Smart Battery Management System (current/temperature/cell monitoring)
//              mounted flush on chassis_bottom in the rear-left bay (Y = +60 mm),
//              completely separated from the PDU in the rear-right bay (Y = -52 mm).
// Coordinate system: Robot frame, rear-left of battery bay at Z = chassis_bottom_z1.
module bms() {
    translate([-232, 60, chassis_bottom_z1 + bms_height / 2]) {
        color([0.15, 0.42, 0.25])
            rounded_box([116, 82, bms_height], 6, true);
        // Aluminum heatsink fins on top of BMS
        color([0.70, 0.72, 0.75])
        for (fx = [-42 : 14 : 42]) {
            translate([fx, 0, bms_height / 2 + 4])
                cube([4, 68, 8], center = true);
        }
    }
}

// ============================================================================
// 18. ELECTRONICS MODULE
// Includes: motor_driver, esp32_s3_controller, jetson_or_sbc, power_distribution, imu_module
// ============================================================================

// Module: motor_driver
// Description: Single-side 48V BLDC servo motor controller with finned heatsink,
//              mounted flush on solid chassis_bottom at X = +95 mm, Y = side * 195 mm
//              immediately ahead of each drive motor and behind the side cooling louvers
//              (completely clear of the [155, +-150] lift rail pedestals!).
// Coordinate system: Mounted on chassis_bottom_z1 at [95, side * 195].
module motor_driver(side = 1) {
    translate([95, side * 195, chassis_bottom_z1 + 21]) {
        color([0.16, 0.18, 0.22])
            rounded_box([88, 66, 42], 6, true);
        color([0.68, 0.70, 0.74])
        translate([0, 0, 25])
            rounded_box([80, 58, 8], 3, true);
    }
}

// Module: esp32_s3_controller
// Description: Real-time MCU controller (ESP32-S3 industrial carrier board with CAN/UART,
//              encoder inputs, motor driver PWM/CAN interface, and hardware watchdog).
//              Mounted on a solid standoff carrier plate bridged onto chassis_bottom_z1.
// Coordinate system: Front-right electronics bay at [232, -56, chassis_bottom_z1].
module esp32_s3_controller() {
    translate([232, -56, chassis_bottom_z1]) {
        // Solid insulated mounting base plate seated directly on chassis_bottom_z1 (no floating!)
        color([0.24, 0.27, 0.31])
        translate([0, 0, 5])
            rounded_box([104, 76, 10], 5, true);

        // Industrial green carrier PCB
        color([0.08, 0.50, 0.22])
        translate([0, 0, 14])
            rounded_box([95, 70, 8], 4, true);

        // ESP32-S3-WROOM RF module shield
        color([0.75, 0.78, 0.80])
        translate([15, 0, 21])
            cube([32, 26, 6], center = true);

        // Pluggable terminal blocks for CAN, UART, Encoders, and Safety Watchdog I/O
        color([0.15, 0.58, 0.28])
        for (ty = [-24, 24]) {
            translate([-20, ty, 23])
                cube([42, 12, 10], center = true);
        }
    }
    component_label("ESP32_S3_MCU", [232, -56, chassis_bottom_z1 + 45]);
}

// Module: jetson_or_sbc
// Description: High-level compute unit (NVIDIA Jetson Orin / Industrial SBC) for ROS 2,
//              2D LiDAR SLAM, Navigation2, sensor fusion, and MES/WMS communication.
//              Mounted on a solid standoff base plate seated directly on chassis_bottom_z1.
// Coordinate system: Front-left electronics bay at [232, 54, chassis_bottom_z1].
module jetson_or_sbc() {
    translate([232, 54, chassis_bottom_z1]) {
        // Solid CNC aluminum mounting pedestal seated directly on chassis_bottom_z1
        color([0.24, 0.27, 0.31])
        translate([0, 0, 4])
            rounded_box([126, 98, 8], 6, true);

        // Carrier board & enclosure
        color([0.15, 0.17, 0.20])
        translate([0, 0, 24])
            rounded_box([120, 92, 32], 6, true);

        // Black anodized finned heatsink with cooling fan hub
        color([0.22, 0.24, 0.28])
        translate([0, 0, 46])
            rounded_box([104, 80, 12], 4, true);

        color([0.10, 0.10, 0.12])
        translate([0, 0, 53])
            cylinder(d = 60, h = 4, center = true);
    }
    component_label("JETSON_ROS2_SBC", [232, 54, chassis_bottom_z1 + 75]);
}

// Module: power_distribution
// Description: Complete power distribution & safety chain board containing:
//              Main Fuse, 48V Safety Contactor, Safety Relay, and 3 isolated DC/DC
//              converters (48V->24V, 48V->12V, 48V->5V).
//              Seated flush on chassis_bottom_z1 in the rear-right bay (Y = -52 mm),
//              completely clear of the BMS in the rear-left bay (Y = +60 mm).
// Coordinate system: Rear-right bay at [-232, -52, chassis_bottom_z1].
module power_distribution() {
    translate([-232, -52, chassis_bottom_z1 + 4]) {
        // Insulated DIN-rail / sub-plate carrier seated flush on chassis_bottom_z1
        color([0.20, 0.24, 0.28])
            rounded_box([116, 112, 8], 6, true);

        // Main High-Current Fuse Holder (48V Bus protection)
        color([0.80, 0.20, 0.15])
        translate([30, 32, 18])
            cube([38, 24, 28], center = true);

        // Main 48V Safety Contactor (Hardware E-stop power removal)
        color([0.18, 0.22, 0.55])
        translate([-24, 30, 22])
            cube([44, 38, 36], center = true);

        // Three isolated DC/DC Converter Bricks (48V->24V, 48V->12V, 48V->5V)
        color([0.28, 0.52, 0.75])
        translate([-34, -22, 16])
            rounded_box([34, 48, 24], 3, true);

        color([0.22, 0.62, 0.48])
        translate([2, -22, 16])
            rounded_box([32, 48, 22], 3, true);

        color([0.75, 0.55, 0.18])
        translate([36, -22, 14])
            rounded_box([28, 48, 18], 3, true);
    }
    component_label("PDU_48V_24V_12V_5V", [-232, -52, chassis_bottom_z1 + 65]);
}

// Module: imu_module
// Description: Industrial 6-axis/9-axis MEMS IMU mounted flush on solid metal of
//              chassis_top at X = +68 mm, Y = 0 (completely clear of the central
//              actuator hole at X = -53..+23 mm!).
// Coordinate system: Seated directly on [68, 0, chassis_top_z1].
module imu_module() {
    translate([68, 0, chassis_top_z1 + 7]) {
        color([0.85, 0.32, 0.12])
            rounded_box([38, 32, 14], 4, true);
        color([0.20, 0.20, 0.22])
        translate([0, 0, 8])
            cube([14, 14, 4], center = true);
    }
}

// ============================================================================
// 19, 20, 21, 22. SENSORS, E-STOP, AND CHARGING PORT MODULES
// Includes: lidar_2d, realsense_d455, emergency_stop, charging_contacts
// ============================================================================

// Module: lidar_pedestal
module lidar_pedestal() {
    pedestal_h = lidar_mount_height - robot_body_height;
    color([0.28, 0.30, 0.34]) {
        translate([lidar_pos_x, 0, robot_body_height + 3])
            cylinder(d1 = 84, d2 = 80, h = 6, center = true);
        translate([lidar_pos_x, 0, robot_body_height + pedestal_h / 2])
            cylinder(d1 = 78, d2 = 76, h = pedestal_h, center = true);
    }
}

// Module: lidar_unit
module lidar_unit() {
    translate([lidar_pos_x, 0, lidar_mount_height]) {
        color([0.12, 0.12, 0.14]) {
            cylinder(d = lidar_diameter, h = 16);
            translate([0, 0, 16])
                cylinder(d1 = lidar_diameter, d2 = lidar_diameter - 4, h = 2);
        }
        color([0.05, 0.08, 0.15])
        translate([0, 0, 18])
            cylinder(d = lidar_diameter - 4, h = 22);
        color([0.14, 0.15, 0.18]) {
            translate([0, 0, 40])
                cylinder(d1 = lidar_diameter - 4, d2 = lidar_diameter, h = 2);
            translate([0, 0, 42])
                cylinder(d = lidar_diameter, h = lidar_height - 42);
        }
    }
}

// Module: lidar_2d
module lidar_2d() {
    lidar_pedestal();
    lidar_unit();
    component_label("2D_LIDAR", [lidar_pos_x, 0, lidar_mount_height + lidar_height + 20]);
}

// Module: realsense_body
module realsense_body() {
    translate([camera_pos_x, 0, camera_mount_height]) {
        color([0.72, 0.75, 0.78])
            rounded_box([d455_length, d455_width, d455_height], 6, true);
        color([0.35, 0.38, 0.42])
        translate([-6, 0, -d455_height / 2 - 7])
            rounded_box([24, 68, 15], 3, true);
    }
}

// Module: realsense_face
module realsense_face() {
    translate([camera_pos_x, 0, camera_mount_height]) {
        color([0.08, 0.08, 0.10])
        translate([d455_length / 2 - 1, 0, 0])
            rounded_box([4, d455_width - 8, d455_height - 6], 4, true);
        color([0.20, 0.35, 0.55])
        for (ly = [-42, 0, 42]) {
            translate([d455_length / 2 + 1, ly, 0])
                rotate([0, 90, 0])
                    cylinder(d = 12, h = 2, center = true);
        }
    }
}

// Module: realsense_d455
module realsense_d455() {
    realsense_body();
    realsense_face();
    component_label("REALSENSE_D455", [camera_pos_x, 0, camera_mount_height + 35]);
}

// Module: estop_collar
module estop_collar() {
    translate([estop_pos_x, estop_pos_y, robot_body_height]) {
        color([0.95, 0.80, 0.05])
            cylinder(d1 = 56, d2 = 52, h = 18);
        color([0.75, 0.75, 0.78])
        translate([0, 0, 18])
            cylinder(d = 24, h = 8);
    }
}

// Module: estop_mushroom
module estop_mushroom() {
    translate([estop_pos_x, estop_pos_y, robot_body_height]) {
        color([0.88, 0.08, 0.08])
        translate([0, 0, 26]) {
            cylinder(d1 = 24, d2 = 38, h = 5);
            translate([0, 0, 5])
                cylinder(d1 = 38, d2 = 46, h = 9);
            translate([0, 0, 14])
                scale([1, 1, 0.35])
                    sphere(d = 46);
        }
    }
}

// Module: emergency_stop
module emergency_stop() {
    estop_collar();
    estop_mushroom();
    component_label("E_STOP", [estop_pos_x, estop_pos_y, robot_body_height + 65]);
}

// Module: charging_contacts_housing
module charging_contacts_housing() {
    translate([robot_length / 2 + 2, 0, charging_contact_z]) {
        color([0.15, 0.15, 0.18])
        translate([-4, 0, 0])
            rounded_box([22, 136, 36], 4, true);
    }
}

// Module: charging_contacts_plates
module charging_contacts_plates() {
    translate([robot_length / 2 + 2, 0, charging_contact_z]) {
        color([0.88, 0.65, 0.18])
        for (cy = [-38, 38]) {
            translate([5, cy, 0])
                cube([6, 36, 22], center = true);
        }
        color([0.75, 0.78, 0.82])
        translate([5, 0, 0])
            rotate([0, 90, 0])
                cylinder(d = 12, h = 8, center = true);
    }
}

// Module: charging_contacts
module charging_contacts() {
    charging_contacts_housing();
    charging_contacts_plates();
}

// ============================================================================
// 23, 24, 25, 26. DELIVERY LIFT & PAYLOAD MODULES
// Includes: lift_fixed_part, lift_guide_rails, lift_actuator, lift_moving_part, payload_20kg
// ============================================================================

// Module: lift_fixed_part
// Description: Rigid base structure of the vertical lift mechanism bolted directly
//              to chassis_bottom (Z = 41 mm), middle bridge (Z = 180 mm), and
//              chassis_top / body_shell without intersecting battery or motors.
// Coordinate system: Centered at X = lift_center_x, Z = [chassis_bottom_z1 .. robot_body_height+4].
module lift_fixed_part() {
    color([0.42, 0.45, 0.48]) {
        // Lower structural bridge frame above battery (Z = 180..190 mm, battery top is 165 mm)
        translate([lift_center_x, 0, lift_fixed_base_z + 5])
        difference() {
            rounded_box([2 * lift_rail_x + 56, 2 * lift_rail_y + 56, 10], 12, true);
            rounded_box([2 * lift_rail_x - 50, 2 * lift_rail_y - 50, 14], 8, true);
        }

        // Central cross-bridge supporting the vertical ball-screw actuator base
        translate([lift_center_x, 0, lift_fixed_base_z + 5])
            cube([70, 2 * lift_rail_y + 56, 10], center = true);

        // Upper mounting collar bolted to chassis_top and projecting through body_shell
        translate([lift_center_x, 0, chassis_top_z1 + 24])
        difference() {
            rounded_box([2 * lift_rail_x + 60, 2 * lift_rail_y + 60, 48], 14, true);
            rounded_box([2 * lift_rail_x - 36, 2 * lift_rail_y - 36, 54], 10, true);
        }

        // 4 Lower base pedestals anchored directly onto chassis_bottom_z1 (Z = 41..60 mm)
        // Located at X = lift_center_x +- 170 mm, Y = +- 150 mm (no floating gap!)
        ped_h = (lift_rail_bottom_z + 10) - chassis_bottom_z1;
        for (dx = [-1, 1], dy = [-1, 1]) {
            rx = lift_center_x + dx * lift_rail_x;
            ry = dy * lift_rail_y;
            translate([rx, ry, chassis_bottom_z1 + ped_h / 2])
                cylinder(d = 52, h = ped_h, center = true);
            // Lower mechanical travel limit stop bumper ring
            color([0.85, 0.25, 0.15])
            translate([rx, ry, lift_rail_bottom_z + 14])
                cylinder(d = 38, h = 8, center = true);
        }
    }
}

// Module: lift_guide_rails
// Description: Four precision hardened steel vertical linear guide shafts spanning
//              from chassis_bottom_z1 (41 mm) to Z = 315 mm with upper limit collars.
// Coordinate system: 4 corners around [lift_center_x, 0].
module lift_guide_rails() {
    rail_bottom = chassis_bottom_z1;                            // 41 mm (anchored on chassis_bottom)
    rail_top_z  = lift_lowered_deck_z - lift_deck_thickness - 3; // 315 mm
    rail_len    = rail_top_z - rail_bottom;                      // 274 mm

    for (dx = [-1, 1], dy = [-1, 1]) {
        rx = lift_center_x + dx * lift_rail_x;
        ry = dy * lift_rail_y;

        // Hardened chrome steel linear guide shaft
        color([0.78, 0.80, 0.84])
        translate([rx, ry, rail_bottom + rail_len / 2])
            cylinder(d = lift_rail_diameter, h = rail_len, center = true);

        // Upper mechanical limit stop ring at top of guide rail
        color([0.85, 0.25, 0.15])
        translate([rx, ry, rail_top_z - 5])
            cylinder(d = lift_rail_diameter + 10, h = 8, center = true);
    }
}

// Module: lift_actuator
// Description: Vertical motorized ball-screw linear actuator with 24V DC lift motor,
//              worm gearbox, leadscrew, and telescopic protective bellows.
// Coordinate system: Centered at [lift_center_x, 0], Z >= lift_fixed_base_z.
// ============================================================================
// FIXED PART OF CENTRAL LIFT ACTUATOR
// ============================================================================

module lift_actuator_fixed_part() {
    act_base_z = lift_fixed_base_z + 10;

    translate([lift_center_x, 0, 0]) {

        // Fixed gearbox housing
        color([0.20, 0.24, 0.30])
        translate([0, 0, act_base_z + 24])
            rounded_box(
                [68, 68, 48],
                8,
                true
            );

        // Fixed horizontal 24V motor
        color([0.15, 0.35, 0.55])
        translate([58, 0, act_base_z + 24])
        rotate([0, 90, 0])
            cylinder(
                d = 52,
                h = 76,
                center = true
            );

        // Fixed outer actuator barrel
        barrel_bottom_z = act_base_z + 40;
        barrel_top_z    = lift_lowered_deck_z - 15;
        barrel_length   = barrel_top_z - barrel_bottom_z;

        color([0.50, 0.53, 0.58])
        translate([
            0,
            0,
            barrel_bottom_z + barrel_length / 2
        ])
            cylinder(
                d = 48,
                h = barrel_length,
                center = true
            );
    }
}


// ============================================================================
// MOVING RAM FOR ORIGINAL OPENSCAD ANIMATION
// ============================================================================

module lift_actuator_moving_part(
    lift_height = 0
) {
    act_base_z = lift_fixed_base_z + 10;

    act_top_z =
        lift_lowered_deck_z -
        lift_deck_thickness +
        lift_height;

    ram_bottom_z = act_base_z + 45;

    ram_length =
        max(
            20,
            act_top_z - ram_bottom_z
        );

    translate([lift_center_x, 0, 0]) {

        // Extending silver ram
        color([0.82, 0.84, 0.88])
        translate([
            0,
            0,
            ram_bottom_z + ram_length / 2
        ])
            cylinder(
                d = 30,
                h = ram_length,
                center = true
            );

        // Upper flange connected to moving platform
        color([0.35, 0.38, 0.42])
        translate([
            0,
            0,
            act_top_z - 5
        ])
            cylinder(
                d = 64,
                h = 10,
                center = true
            );
    }
}


// ============================================================================
// MOVING RAM FOR URDF / RVIZ EXPORT
//
// Mesh origin:
//   X = center of ram
//   Y = center of ram
//   Z = upper attachment point
//
// Ram extends downward along local -Z.
// ============================================================================

module lift_actuator_moving_ram_urdf() {
    ram_length   = 230;
    ram_diameter = 30;

    // Silver moving ram
    color([0.82, 0.84, 0.88])
    translate([
        0,
        0,
        -ram_length / 2
    ])
        cylinder(
            d = ram_diameter,
            h = ram_length,
            center = true
        );

    // Upper attachment flange
    color([0.35, 0.38, 0.42])
    translate([
        0,
        0,
        -5
    ])
        cylinder(
            d = 64,
            h = 10,
            center = true
        );
}


// ============================================================================
// COMPLETE ACTUATOR FOR OPENSCAD
//
// Existing calls such as:
//
// lift_actuator(current_lift_height);
//
// continue to work normally.
// ============================================================================

module lift_actuator(
    lift_height = 0
) {
    lift_actuator_fixed_part();

    lift_actuator_moving_part(
        lift_height
    );
}


// Module: lift_platform_base
module lift_platform_base(lift_height = 0) {
    deck_top_z = lift_lowered_deck_z + lift_height;
    deck_bottom_z = deck_top_z - lift_deck_thickness;
    sleeve_len = 262;

    translate([lift_center_x, 0, deck_bottom_z + lift_deck_thickness / 2]) {
        // Main load-bearing platform
        color([0.92, 0.58, 0.08])
            rounded_box([payload_deck_length, payload_deck_width, lift_deck_thickness], 24, true);

        // Four L-shaped payload-retention corner locating brackets
        color([0.85, 0.45, 0.05])
        for (sx = [-1, 1], sy = [-1, 1]) {
            translate([sx * (payload_box_length / 2 - 12), sy * (payload_box_width / 2 + 8), lift_deck_thickness / 2 + 8])
                rounded_box([44, 14, 16], 3, true);
            translate([sx * (payload_box_length / 2 + 8), sy * (payload_box_width / 2 - 12), lift_deck_thickness / 2 + 8])
                rounded_box([14, 44, 16], 3, true);
        }
    }

    // Four moving linear-bearing sleeves
    color([0.88, 0.52, 0.10])
    for (dx = [-1, 1], dy = [-1, 1]) {
        rx = lift_center_x + dx * lift_rail_x;
        ry = dy * lift_rail_y;
        eff_sleeve_len = max(sleeve_len, deck_bottom_z - 285);
        translate([rx, ry, deck_bottom_z - eff_sleeve_len / 2])
            cylinder(d = lift_sleeve_diameter, h = eff_sleeve_len, center = true);
        translate([rx, ry, deck_bottom_z - 12])
            cylinder(d = lift_sleeve_diameter + 18, h = 24, center = true);
    }
}

// Module: lift_deck_pad
module lift_deck_pad(lift_height = 0) {
    deck_top_z = lift_lowered_deck_z + lift_height;
    deck_bottom_z = deck_top_z - lift_deck_thickness;

    translate([lift_center_x, 0, deck_bottom_z + lift_deck_thickness / 2]) {
        color([0.22, 0.24, 0.26])
        translate([0, 0, lift_deck_thickness / 2 + 1])
            rounded_box([payload_deck_length - 36, payload_deck_width - 36, 2], 16, true);
    }
}

// Module: lift_moving_part
module lift_moving_part(
    lift_height = 0,
    show_scissor_bars = true
) {
    lift_platform_base(lift_height);
    lift_deck_pad(lift_height);

    deck_top_z = lift_lowered_deck_z + lift_height;
    deck_bottom_z = deck_top_z - lift_deck_thickness;


    // ========================================================================
    // 3. OPTIONAL SCISSOR BARS
    //
    // Two crossing bars on the left side.
    // Two crossing bars on the right side.
    // ========================================================================

    if (show_scissor_bars) {
        scissor_lower_z =
            chassis_top_z1 +
            12;

        scissor_upper_z =
            deck_bottom_z -
            8;

        for (side = [-1, 1]) {
            // ------------------------------------------------------------
            // BAR A: lower-left to upper-right
            // ------------------------------------------------------------

            color([0.82, 0.48, 0.08])
            hull() {
                translate([
                    lift_center_x - 135,

                    side *
                    (
                        lift_rail_y -
                        38
                    ),

                    scissor_lower_z
                ])
                    sphere(d = 16);

                translate([
                    lift_center_x + 135,

                    side *
                    (
                        lift_rail_y -
                        38
                    ),

                    scissor_upper_z
                ])
                    sphere(d = 16);
            }


            // ------------------------------------------------------------
            // BAR B: lower-right to upper-left
            // ------------------------------------------------------------

            color([0.82, 0.48, 0.08])
            hull() {
                translate([
                    lift_center_x + 135,

                    side *
                    (
                        lift_rail_y -
                        52
                    ),

                    scissor_lower_z
                ])
                    sphere(d = 16);

                translate([
                    lift_center_x - 135,

                    side *
                    (
                        lift_rail_y -
                        52
                    ),

                    scissor_upper_z
                ])
                    sphere(d = 16);
            }
        }
    }
}

    

// Module: payload_tray_body
module payload_tray_body(
    lift_height = 0,
    tray_color = [0.72, 0.52, 0.32]
) {
    pz0         = lift_lowered_deck_z + lift_height + 2;
    wing_span_y = is_undef(rack_tray_wing_span_y)    ? 156 : rack_tray_wing_span_y;
    wing_len_x  = is_undef(rack_tray_wing_length)    ? 420 : rack_tray_wing_length;
    wing_thick  = is_undef(rack_tray_wing_thickness) ? 16  : rack_tray_wing_thickness;
    wing_z_off  = is_undef(rack_tray_wing_z_offset)  ? 20  : rack_tray_wing_z_offset;
    wing_z_ctr  = -payload_box_height / 2 + wing_z_off + wing_thick / 2;

    translate([lift_center_x, 0, pz0 + payload_box_height / 2]) {
        color(tray_color) {
            difference() {
                rounded_box([payload_box_length, payload_box_width, payload_box_height], 12, true);
                translate([0, 0, 8])
                    rounded_box([payload_box_length - 24, payload_box_width - 24, payload_box_height], 8, true);
            }

            for (sy = [-1, 1]) {
                translate([
                    0,
                    sy * (payload_box_width / 2 + wing_span_y / 2 - 2),
                    wing_z_ctr
                ])
                    rounded_box(
                        [wing_len_x, wing_span_y + 4, wing_thick],
                        5,
                        true
                    );

                for (rx = [-165, -82, 0, 82, 165]) {
                    translate([
                        rx,
                        sy * (payload_box_width / 2 + 48),
                        wing_z_ctr + wing_thick / 2 + 5
                    ])
                        rounded_box([10, 96, 10], 2, true);
                }
            }
        }

        color([0.24, 0.26, 0.30])
        for (sy = [-1, 1]) {
            translate([
                0,
                sy * (payload_box_width / 2 + wing_span_y - 27),
                wing_z_ctr - wing_thick / 2 + 1
            ])
                rounded_box([wing_len_x - 16, 50, 2.5], 3, true);
        }
    }
}

// Module: payload_weights
module payload_weights(lift_height = 0) {
    pz0 = lift_lowered_deck_z + lift_height + 2;
    translate([lift_center_x, 0, pz0 + payload_box_height / 2]) {
        color([0.62, 0.65, 0.70])
        for (px = [-150, 0, 150], py = [-95, 95]) {
            translate([px, py, -4])
                cylinder(d = 76, h = payload_box_height - 14, center = true);
        }
    }
}

// Module: payload_20kg
module payload_20kg(
    lift_height = 0,
    tray_color = [0.72, 0.52, 0.32]
) {
    payload_tray_body(lift_height, tray_color);
    payload_weights(lift_height);
    component_label("PAYLOAD_20KG", [lift_center_x, 0, lift_lowered_deck_z + lift_height + 2 + payload_box_height + 25]);
}

// ============================================================================
// 27, 28. SENSOR FOV, SUPPORT POLYGON & CORRIDOR CLEARANCE DEBUG MODULES
// Includes: lidar_scan_debug, camera_fov_debug, support_polygon_debug, corridor_debug
// ============================================================================

// Module: lidar_scan_debug
// Description: Visualizes the 360-deg horizontal 2D LiDAR scan disk (radius 3000 mm
//              per human_detection_range requirement) at Z = lidar_mount_height.
module lidar_scan_debug() {
    if (show_lidar_scan) {
        translate([lidar_pos_x, 0, lidar_mount_height + 30]) {
            // Near-field safety stop zone (R = 800 mm)
            color([1.0, 0.2, 0.2, 0.22])
                cylinder(r = 800, h = 2, center = true);
            // Human detection range disc (R = human_detection_range = 3000 mm)
            color([0.1, 0.9, 0.3, 0.12])
                cylinder(r = human_detection_range, h = 1, center = true);
        }
    }
}

// Module: camera_fov_debug
// Description: Visualizes the 3D frustum pyramid of the Intel RealSense D455 depth
//              camera (86 deg HFOV x 57 deg VFOV) projecting forward (+X).
// ============================================================================
// 27, 28. SENSOR FOV, SUPPORT POLYGON & CORRIDOR CLEARANCE DEBUG MODULES
// ============================================================================

// Module: camera_fov_debug
// Description: Visualizes the 3D viewing frustum of the Intel RealSense D455.
// Coordinate system: Camera looks forward in the +X direction.
module camera_fov_debug() {
    if (show_camera_fov) {
        fov_len = camera_debug_range;
        half_w  = tan(d455_hfov_deg / 2) * fov_len;
        half_h  = tan(d455_vfov_deg / 2) * fov_len;

        // Transparent camera field-of-view volume
        color([0.10, 0.65, 1.00, 0.12])
        translate([
            camera_pos_x + d455_length / 2,
            0,
            camera_mount_height
        ])
        polyhedron(
            points = [
                [0,       0,       0],
                [fov_len, half_w,   half_h],
                [fov_len, -half_w,  half_h],
                [fov_len, -half_w, -half_h],
                [fov_len, half_w,  -half_h]
            ],
            faces = [
                [0, 2, 1],
                [0, 3, 2],
                [0, 4, 3],
                [0, 1, 4],
                [1, 2, 3, 4]
            ],
            convexity = 10
        );

        // Camera optical axis
        color([0.05, 0.90, 1.00, 0.85])
        translate([
            camera_pos_x + d455_length / 2,
            0,
            camera_mount_height
        ])
        rotate([0, 90, 0])
            cylinder(d = 5, h = fov_len);

        // Far-plane frame
        color([0.05, 0.75, 1.00, 0.55])
        translate([
            camera_pos_x + d455_length / 2 + fov_len,
            0,
            camera_mount_height
        ]) {
            for (sy = [-1, 1]) {
                translate([0, sy * half_w, 0])
                    cube([5, 8, 2 * half_h], center = true);
            }

            for (sz = [-1, 1]) {
                translate([0, 0, sz * half_h])
                    cube([5, 2 * half_w, 8], center = true);
            }
        }
    }
}


// Module: support_polygon_debug
// Description: Displays the approximate support polygon of the robot.
module support_polygon_debug() {
    if (show_support_polygon) {
        support_x = caster_offset_x + caster_trail + caster_wheel_radius;

        support_y = max([
            caster_offset_y + caster_wheel_width / 2,
            drive_wheel_track / 2 + drive_wheel_width / 2
        ]);

        color([0.10, 0.90, 0.30, 0.22])
        translate([0, 0, 1])
        linear_extrude(height = 2)
        polygon(points = [
            [ support_x,  support_y],
            [ support_x, -support_y],
            [-support_x, -support_y],
            [-support_x,  support_y]
        ]);

        // Robot center marker
        color([1.00, 0.15, 0.10, 0.90])
        translate([0, 0, 5])
            cylinder(d = 28, h = 10, center = true);
    }
}


// Module: corridor_debug
// Description: Displays a 1200 mm wide factory corridor.
module corridor_debug() {
    if (show_corridor) {
        corridor_length_debug = 3200;
        wall_thickness_debug   = 25;
        wall_height_debug      = 450;

        // Factory floor
        color([0.68, 0.70, 0.72, 0.30])
        translate([0, 0, -6])
            cube([
                corridor_length_debug,
                corridor_width,
                10
            ], center = true);

        // Corridor walls
        color([0.72, 0.74, 0.77, 0.45])
        for (sy = [-1, 1]) {
            translate([
                0,
                sy * (
                    corridor_width / 2 +
                    wall_thickness_debug / 2
                ),
                wall_height_debug / 2
            ])
            cube([
                corridor_length_debug,
                wall_thickness_debug,
                wall_height_debug
            ], center = true);
        }

        // Robot side-clearance areas
        color([0.98, 0.78, 0.05, 0.30])
        for (sy = [-1, 1]) {
            translate([
                0,
                sy * (
                    robot_width / 2 +
                    actual_side_clearance / 2
                ),
                2
            ])
            cube([
                corridor_length_debug,
                actual_side_clearance,
                3
            ], center = true);
        }
    }
}


// Module: turning_envelope_debug
// Description: Displays the required circular sweep for an in-place rotation.
module turning_envelope_debug() {
    if (show_turning_envelope) {
        envelope_color =
            sweep_diameter <= corridor_width
            ? [0.10, 0.85, 0.25, 0.20]
            : [1.00, 0.10, 0.10, 0.25];

        color(envelope_color)
        translate([0, 0, 2])
        difference() {
            cylinder(
                r = sweep_radius,
                h = 3,
                center = true
            );

            cylinder(
                r = max(0, sweep_radius - 8),
                h = 5,
                center = true
            );
        }
    }
}
// ============================================================================
// FUNCTION 9 & 10: RACK TRAY (GIỮ HÌNH DÁNG CŨ + 2 CÁNH CHÌA RA VỪA THANH ĐỂ KHAY)
// ============================================================================

module rack_tray(
    tray_color = [0.72, 0.52, 0.32],
    show_contents = true
) {
    wing_z_ctr =
        -rack_tray_height / 2 +
        rack_tray_wing_z_offset +
        rack_tray_wing_thickness / 2;

    // Outer tray body (giữ hình dáng cũ) + 2 cánh chìa ra hai bên.
    color(tray_color) {
        difference() {
            rounded_box(
                [
                    rack_tray_length,
                    rack_tray_width,
                    rack_tray_height
                ],
                12,
                true
            );

            // Recessed tray interior.
            translate([0, 0, 8])
                rounded_box(
                    [
                        rack_tray_length - 24,
                        rack_tray_width - 24,
                        rack_tray_height
                    ],
                    8,
                    true
                );
        }

        // Two protruding side wings ("2 cánh chìa ra sao cho vừa thanh để khay")
        for (sy = [-1, 1]) {
            translate([
                0,
                sy * (rack_tray_width / 2 + rack_tray_wing_span_y / 2 - 2),
                wing_z_ctr
            ])
                rounded_box(
                    [
                        rack_tray_wing_length,
                        rack_tray_wing_span_y + 4,
                        rack_tray_wing_thickness
                    ],
                    5,
                    true
                );

            // Upper stiffening gusset ribs connecting tray side wall to the wing
            for (rx = [-165, -82, 0, 82, 165]) {
                translate([
                    rx,
                    sy * (rack_tray_width / 2 + 48),
                    wing_z_ctr + rack_tray_wing_thickness / 2 + 5
                ])
                    rounded_box([10, 96, 10], 2, true);
            }
        }
    }

    // Precision wear pads on the bottom of the outer wing tips (mating with rack rails)
    color([0.24, 0.26, 0.30])
    for (sy = [-1, 1]) {
        translate([
            0,
            sy * (rack_tray_width / 2 + rack_tray_wing_span_y - 27),
            wing_z_ctr - rack_tray_wing_thickness / 2 + 1
        ])
            rounded_box([rack_tray_wing_length - 16, 50, 2.5], 3, true);
    }

    // Industrial steel components inside the tray.
    if (show_contents) {
        color([0.62, 0.65, 0.70])
        for (
            px = [-150, 0, 150],
            py = [-95, 95]
        ) {
            translate([px, py, -4])
                cylinder(
                    d = 76,
                    h = rack_tray_height - 14,
                    center = true
                );
        }
    }
}

// ============================================================================
// FUNCTION 9 & 10: SENSOR PLUGIN MOUNTED ON RACK SUPPORT RAILS
// ("trên 2 thanh kệ có plugin cảm biến")
// ============================================================================

module rack_rail_sensor_plugin(
    side = 1,
    occupied = false,
    selected = false
) {
    // Mounted on the outer edge of each support rail (|Y| = 358..392 mm),
    // completely outside the tray wing envelope (|Y| <= 356 mm) so there is zero collision.
    rail_outer_y = side * (rack_rail_y + rack_rail_width / 2);

    // 1. Flush inductive/optical sensing strip embedded in the top wear surface of the rail
    color(occupied ? [0.10, 0.90, 0.30] : [0.35, 0.38, 0.42])
    translate([0, side * rack_rail_y, 0.5])
        rounded_box([70, 14, 2], 2, true);

    // 2. Industrial plug-in sensor housing clipped to the outer side of the rack rail
    translate([0, rail_outer_y + side * 16, 4]) {
        // Dark industrial polyamide sensor body
        color([0.10, 0.12, 0.15])
            rounded_box([64, 28, 28], 4, true);

        // Snap-on steel mounting bracket clamping around the support rail flank
        color([0.32, 0.36, 0.42])
        translate([0, -side * 10, -10])
            rounded_box([74, 16, 14], 2, true);

        // Optical retro-reflective lens facing inward (-side * Y) across the support rail top
        color(occupied ? [0.05, 0.95, 0.25] : [0.95, 0.12, 0.08])
        translate([0, -side * 14, 4])
        rotate([90, 0, 0])
            cylinder(d = 12, h = 4, center = true);

        // Top LED status indicator ring on the plugin module
        color(
            occupied ? [0.05, 0.95, 0.25] :
            selected ? [1.00, 0.75, 0.05] :
                       [0.95, 0.15, 0.08]
        )
        translate([0, 0, 15])
            rounded_box([36, 12, 4], 2, true);

        // Brass M12 industrial quick-disconnect plugin connector + cable gland on outer side
        color([0.82, 0.64, 0.20])
        translate([0, side * 18, -2])
        rotate([90, 0, 0])
            cylinder(d = 12, h = 10, center = true);

        color([0.18, 0.20, 0.24])
        translate([0, side * 28, -2])
        rotate([90, 0, 0])
            cylinder(d = 8, h = 14, center = true);
    }

    // 3. Active optical detection beam across the rail surface when show_rack_sensors is enabled
    if (show_rack_sensors) {
        color(
            occupied
            ? [0.05, 0.95, 0.25, 0.40]
            : [1.00, 0.20, 0.10, 0.25]
        )
        translate([0, side * rack_rail_y, 8])
            cube([4, rack_rail_width - 6, 3], center = true);
    }
}

// ============================================================================
// FUNCTION 9 & 10: RACK SUPPORT RAILS ("2 THANH ĐỂ KHAY CÓ PLUGIN CẢM BIẾN")
// ============================================================================

module rack_shelf_latches(
    level_z,
    occupied = false,
    selected = false
) {
    rail_y   = rack_rail_y;
    rail_len = rack_rail_length;
    rail_w   = rack_rail_width;
    rail_h   = rack_rail_height;

    rail_color =
        selected
        ? [1.00, 0.72, 0.05]
        : [0.85, 0.48, 0.10];

    for (sy = [-1, 1]) {
        // Main horizontal tray support rail (thanh để khay)
        color(rail_color)
        translate([0, sy * rail_y, level_z - rail_h / 2])
            rounded_box([rail_len, rail_w, rail_h], 4, true);

        // Low-friction UHMW-PE wear strip on top surface where the tray wing rests
        color([0.16, 0.18, 0.22])
        translate([0, sy * rail_y, level_z - 0.5])
            cube([rail_len - 28, rail_w - 10, 2], center = true);

        // Outer lateral guide rim (keeps tray wing centered at |Y| <= 356 mm)
        color([0.24, 0.27, 0.32])
        translate([
            0,
            sy * (rail_y + rail_w / 2 + 3),
            level_z + 2
        ])
            rounded_box([rail_len - 80, 6, rail_h + 4], 2, true);

        // Front sloped lead-in guide ramp (vát dẫn hướng đầu vào)
        color(rail_color)
        translate([
            -rail_len / 2 - 12,
            sy * rail_y,
            level_z - rail_h / 2 - 4
        ])
            rounded_box([26, rail_w, rail_h], 3, true);

        // Rear mechanical stop block (at X = +248 mm, 38 mm clear of wing length |X| <= 210 mm)
        color([0.22, 0.25, 0.30])
        translate([
            rail_len / 2 - 12,
            sy * rail_y,
            level_z + 7
        ])
            rounded_box([20, rail_w, 14], 3, true);

        // Cantilevered structural brackets connecting the rail outer flank to the rack columns
        color([0.18, 0.21, 0.25]) {
            post_inner_y = sy * (rack_depth / 2 - rack_post_size);
            rail_outer_y = sy * (rail_y + rail_w / 2);

            for (sx = [-1, 1]) {
                translate([
                    sx * (rail_len / 2 - 22),
                    (rail_outer_y + post_inner_y) / 2,
                    level_z - rail_h / 2
                ])
                    rounded_box(
                        [36, abs(post_inner_y - rail_outer_y) + 12, rail_h],
                        3,
                        true
                    );
            }

            // Full-length longitudinal side stringer tying front and rear posts at this level
            translate([
                0,
                post_inner_y + sy * (rack_post_size / 2),
                level_z - rail_h / 2
            ])
                rounded_box([rack_width, rack_post_size, rail_h], 3, true);
        }

        // Sensor plugin mounted directly on each support rail ("trên 2 thanh kệ có plugin cảm biến")
        translate([0, 0, level_z])
            rack_rail_sensor_plugin(sy, occupied, selected);
    }
}
// ============================================================================
// FUNCTION 9: LEVEL STATUS INDICATOR
//
// Yellow = selected by robot.
// Red    = occupied.
// Green  = empty.
// ============================================================================

module rack_level_indicator(
    level_number = 1,
    occupied = false,
    selected = false
) {
    indicator_color =
        selected
        ? [1.00, 0.72, 0.05]
        :
        occupied
        ? [0.95, 0.08, 0.05]
        :
        [0.05, 0.90, 0.18];

    // Indicator housing.
    color([0.10, 0.12, 0.15])
        rounded_box(
            [76, 22, 46],
            5,
            true
        );

    // Status lamp.
    color(indicator_color)
    translate([0, -14, 0])
    rotate([90, 0, 0])
        cylinder(
            d = 24,
            h = 7,
            center = true
        );

    // Level number label.
    if (show_rack_labels) {
        color([0.95, 0.95, 0.95])
        translate([0, -18, 0])
        rotate([90, 0, 0])
        linear_extrude(height = 1.5)
            text(
                str(level_number),
                size = 14,
                halign = "center",
                valign = "center"
            );
    }
}
// ============================================================================
// FUNCTION 9: COMPLETE TWO-LEVEL SMART RACK
// ============================================================================

module two_level_smart_rack(
    level_1_occupied = true,
    level_2_occupied = false,
    selected_level = 0
) {
    post_x =
        rack_width / 2 -
        rack_post_size / 2;

    post_y =
        rack_depth / 2 -
        rack_post_size / 2;

    rack_top_z =
        rack_height;

    translate([
        rack_center_x,
        rack_center_y,
        0
    ]) {
        // --------------------------------------------------------------------
        // Four vertical steel posts.
        // --------------------------------------------------------------------

        color([0.22, 0.25, 0.30])
        for (
            sx = [-1, 1],
            sy = [-1, 1]
        ) {
            translate([
                sx * post_x,
                sy * post_y,
                rack_top_z / 2
            ])
                rounded_box(
                    [
                        rack_post_size,
                        rack_post_size,
                        rack_top_z
                    ],
                    5,
                    true
                );
        }

        // --------------------------------------------------------------------
        // Lower base beams (Drive-in tunnel design: open front entrance).
        // --------------------------------------------------------------------

        color([0.17, 0.20, 0.24]) {
            // Left and right longitudinal floor guide rails
            for (sy = [-1, 1]) {
                translate([
                    0,
                    sy * post_y,
                    rack_beam_height / 2
                ])
                    rounded_box(
                        [
                            rack_width,
                            rack_post_size,
                            rack_beam_height
                        ],
                        4,
                        true
                    );
            }

            // Rear end backstop beam (entrance at -post_x is open for AMR drive-in)
            translate([
                post_x,
                0,
                rack_beam_height / 2
            ])
                rounded_box(
                    [
                        rack_post_size,
                        rack_depth,
                        rack_beam_height
                    ],
                    4,
                    true
                );
        }

        // --------------------------------------------------------------------
        // Top connecting beams.
        // --------------------------------------------------------------------

        color([0.17, 0.20, 0.24]) {
            for (sy = [-1, 1]) {
                translate([
                    0,
                    sy * post_y,
                    rack_top_z -
                    rack_beam_height / 2
                ])
                    rounded_box(
                        [
                            rack_width,
                            rack_post_size,
                            rack_beam_height
                        ],
                        4,
                        true
                    );
            }

            for (sx = [-1, 1]) {
                translate([
                    sx * post_x,
                    0,
                    rack_top_z -
                    rack_beam_height / 2
                ])
                    rounded_box(
                        [
                            rack_post_size,
                            rack_depth,
                            rack_beam_height
                        ],
                        4,
                        true
                    );
            }
        }

        // --------------------------------------------------------------------
        // Level 1 support rails & sensor plugins ("2 thanh để khay dưới + plugin cảm biến").
        // --------------------------------------------------------------------
        rack_shelf_latches(
            rack_level_1_z,
            level_1_occupied,
            selected_level == 1
        );

        // --------------------------------------------------------------------
        // Level 2 support rails & sensor plugins ("2 thanh để khay trên + plugin cảm biến").
        // --------------------------------------------------------------------
        rack_shelf_latches(
            rack_level_2_z,
            level_2_occupied,
            selected_level == 2
        );

        // --------------------------------------------------------------------
        // Level 1 tray (2 wings resting squarely on the 2 rails of Level 1).
        // --------------------------------------------------------------------
        if (level_1_occupied) {
            translate([
                0,
                0,
                rack_level_1_tray_bottom_z + rack_tray_height / 2
            ])
                rack_tray(
                    [0.72, 0.52, 0.32],
                    true
                );
        }

        // --------------------------------------------------------------------
        // Level 2 tray (2 wings resting squarely on the 2 rails of Level 2).
        // --------------------------------------------------------------------
        if (level_2_occupied) {
            translate([
                0,
                0,
                rack_level_2_tray_bottom_z + rack_tray_height / 2
            ])
                rack_tray(
                    [0.52, 0.65, 0.82],
                    true
                );
        }

        // --------------------------------------------------------------------
        // Status indicators on front entrance column.
        // --------------------------------------------------------------------

        translate([
            -post_x,
            -rack_depth / 2 - 11,
            rack_level_1_z + 24
        ])
            rack_level_indicator(
                1,
                level_1_occupied,
                selected_level == 1
            );

        translate([
            -post_x,
            -rack_depth / 2 - 11,
            rack_level_2_z + 24
        ])
            rack_level_indicator(
                2,
                level_2_occupied,
                selected_level == 2
            );

        // --------------------------------------------------------------------
        // Level labels (mounted neatly on the top entrance header).
        // --------------------------------------------------------------------

        if (show_rack_labels) {
            color([0.95, 0.95, 0.95])
            translate([
                -post_x - 18,
                0,
                rack_top_z - rack_beam_height / 2
            ])
            rotate([90, 0, -90])
            linear_extrude(height = 2)
                text(
                    "SMART STORAGE RACK - 2 LEVELS",
                    size = 18,
                    halign = "center",
                    valign = "center"
                );
        }
    }
}
// ============================================================================
// FUNCTION 9 & 10: SMART RACK SCENE
// - Mode 9 : Robot picks trays sequentially from bottom to top (Level 1 -> Level 2)
// - Mode 10: Robot places trays sequentially from top to bottom (Level 2 -> Level 1)
// ============================================================================

module function_9_smart_rack_scene() {
    active_level =
        rack_active_level_at_t(animation_mode, $t);

    displayed_level_1_occupied =
        rack_level_1_occupied_at_t(animation_mode, $t);

    displayed_level_2_occupied =
        rack_level_2_occupied_at_t(animation_mode, $t);

    // Draw the 2-level smart rack with sensor plugins on both rails.
    two_level_smart_rack(
        displayed_level_1_occupied,
        displayed_level_2_occupied,
        active_level
    );

    // Draw the robot using the animated transform system.
    robot_animation_transform() {
        factory_delivery_amr();
        support_polygon_debug();

        // 3D Optical scanner cone when robot pauses in front of rack to inspect levels
        u_cycle = (rack_preferred_level == 0) ? (($t < 0.50) ? ($t * 2) : (($t - 0.50) * 2)) : $t;
        if (u_cycle >= 0.12 && u_cycle <= 0.26) {
            color([0.05, 0.95, 0.85, 0.28])
            translate([camera_pos_x + 10, 0, camera_mount_height])
            rotate([0, 14, 0])
                cylinder(d1 = 12, d2 = 380, h = 320);
        }
    }

    // Warning if no suitable level exists.
    if (active_level == 0) {
        color([1.00, 0.08, 0.05])
        translate([
            rack_center_x,
            rack_center_y,
            rack_height + 80
        ])
        linear_extrude(height = 5)
            text(
                effective_rack_operation == "pickup"
                ? "NO TRAY AVAILABLE"
                : "NO EMPTY LEVEL",
                size = 38,
                halign = "center",
                valign = "center"
            );
    }
}

// ============================================================================
// 29. CHARGING STATION MODULE (INDUSTRIAL 48V AMR FAST-CHARGING DOCK)
// ============================================================================

// Sub-module 29A: Heavy-duty floor base plate, hazard striping & yellow side bollards
module charging_station_base_and_guards() {
    // Main dark powder-coated steel base plate (Z = 0..16 mm)
    color([0.18, 0.20, 0.24])
    translate([135, 0, 8])
        rounded_box([290, 540, 16], 14, true);

    // Raised structural cabinet plinth (Z = 16..32 mm)
    color([0.24, 0.27, 0.32])
    translate([155, 0, 24])
        rounded_box([220, 420, 16], 10, true);

    // Front safety yellow/black hazard threshold bar along docking edge
    color([0.95, 0.76, 0.05])
    translate([4, 0, 17])
        rounded_box([22, 460, 4], 3, true);

    color([0.12, 0.13, 0.15])
    for (hy = [-180 : 60 : 180]) {
        translate([4, hy, 18.5])
            cube([23, 24, 2], center = true);
    }

    // 6 Perimeter floor-anchor M12 bolts with washers
    color([0.74, 0.77, 0.82])
    for (bx = [20, 140, 255], by = [-245, 245]) {
        translate([bx, by, 16]) {
            cylinder(d = 22, h = 3);
            translate([0, 0, 3])
                cylinder(d = 14, h = 8, $fn = 6);
        }
    }

    // Left and right yellow tubular steel impact protection bollards
    for (sy = [-1, 1]) {
        bollard_y = sy * 238;

        color([0.95, 0.75, 0.05]) {
            for (bx = [42, 238]) {
                translate([bx, bollard_y, 216])
                    cylinder(d = 34, h = 400, center = true);
                translate([bx, bollard_y, 416])
                    sphere(d = 34);
            }
            translate([140, bollard_y, 416])
            rotate([0, 90, 0])
                cylinder(d = 34, h = 196, center = true);
        }

        // Black industrial warning bands on the front bollard post
        color([0.10, 0.11, 0.13])
        for (bz = [120, 220, 320]) {
            translate([42, bollard_y, bz])
                cylinder(d = 35.5, h = 28, center = true);
        }

        // Steel base mounting flanges for the bollards
        color([0.28, 0.31, 0.36])
        for (bx = [42, 238]) {
            translate([bx, bollard_y, 20])
                rounded_box([56, 56, 8], 6, true);
        }
    }
}

// Sub-module 29B: Floating spring-loaded collector head & high-current 48V brushes
// Preserves exact mating face X = -24 mm, Y = +-38 mm, Z = 95 mm
module charging_station_collector_head(contact_z = 95) {
    // Rear mounting bulkhead bolted to cabinet front wall (X = 68..86 mm)
    color([0.22, 0.25, 0.30])
    translate([77, 0, contact_z])
        rounded_box([18, 220, 92], 8, true);

    // Four telescopic stainless guide shafts + compression springs (X = 24..72 mm)
    for (sy = [-1, 1], sz = [-1, 1]) {
        rod_y = sy * 72;
        rod_z = contact_z + sz * 22;

        color([0.80, 0.83, 0.88])
        translate([48, rod_y, rod_z])
        rotate([0, 90, 0])
            cylinder(d = 14, h = 52, center = true);

        color([0.35, 0.38, 0.42])
        for (sx_coil = [30 : 8 : 62]) {
            translate([sx_coil, rod_y, rod_z])
            rotate([0, 90, 0])
                cylinder(d = 22, h = 3.5, center = true);
        }
    }

    // Central insulated high-current busbar flexible bellows (X = 26..70 mm)
    color([0.14, 0.15, 0.18])
    translate([48, 0, contact_z])
        rounded_box([44, 108, 52], 6, true);

    // Front high-temp PEEK/Phenolic insulated contact block (X = -8..28 mm)
    color([0.09, 0.10, 0.12])
    translate([10, 0, contact_z])
        rounded_box([36, 176, 68], 6, true);

    // Protective sloped steel dust/drip hood over the charging contacts
    color([0.26, 0.30, 0.36])
    translate([36, 0, contact_z + 44])
        rounded_box([96, 196, 8], 4, true);

    // Polarity indicator collars (+48V Red on Left, GND Blue on Right)
    for (sy = [-1, 1]) {
        color(sy > 0 ? [0.88, 0.12, 0.12] : [0.12, 0.42, 0.88])
        translate([-5, sy * 38, contact_z])
        rotate([0, 90, 0])
            cylinder(d = 34, h = 6, center = true);
    }

    // Positive and negative spring-loaded copper-graphite contacts
    // Center at X = -8 mm, length = 32 mm -> Front contact face at X = -24 mm!
    color([0.88, 0.62, 0.16])
    for (sy = [-1, 1]) {
        translate([-8, sy * 38, contact_z])
        rotate([0, 90, 0])
            cylinder(d = 28, h = 32, center = true);
    }

    // Central conical mechanical alignment receiver (Y = 0, Z = contact_z)
    color([0.74, 0.78, 0.82])
    translate([-9, 0, contact_z])
    rotate([0, 90, 0])
    difference() {
        cylinder(d = 26, h = 34, center = true);
        cylinder(d = 13, h = 38, center = true);
    }
}

// Sub-module 29C: Power cabinet, Optical/LiDAR docking targets, HMI & Andon tower
module charging_station_cabinet_and_hmi() {
    // Main cabinet body housing 48V/60A PFC rectifier modules
    color([0.16, 0.20, 0.26])
    translate([165, 0, 330])
        rounded_box([160, 380, 596], 14, true);

    // Sloped industrial anti-dust cabinet roof overhang (Z = 626..644 mm)
    color([0.22, 0.26, 0.32])
    translate([162, 0, 634])
        rounded_box([176, 396, 16], 10, true);

    // Protruding front service door panel (X = 79..85 mm, Z = 165..605 mm)
    color([0.23, 0.28, 0.35])
    translate([82, 0, 385])
        rounded_box([6, 332, 432], 6, true);

    // Cabinet door perimeter weatherproof rubber gasket frame (surrounding door)
    color([0.10, 0.11, 0.13])
    translate([83.5, 0, 385])
    difference() {
        rounded_box([4, 346, 446], 9, true);
        rounded_box([8, 332.5, 432.5], 6, true);
    }

    // Industrial swing-handle cabinet latch & keyhole on left of door
    color([0.12, 0.13, 0.15])
    translate([77, -142, 360])
        rounded_box([6, 22, 86], 3, true);
    color([0.75, 0.78, 0.82])
    translate([73.5, -142, 382])
    rotate([0, 90, 0])
        cylinder(d = 12, h = 4, center = true);

    // Side & lower front ventilation louver grilles
    color([0.11, 0.13, 0.17]) {
        for (lz = [185 : 18 : 239]) {
            translate([78.5, 0, lz])
                cube([3, 180, 7], center = true);
        }
        for (sy = [-1, 1], lz = [220 : 24 : 520]) {
            translate([165, sy * 190, lz])
                cube([95, 4, 9], center = true);
        }
    }

    // Optical Fiducial Docking Target at Z = 285 mm (aligned with RealSense D455)
    color([0.20, 0.23, 0.28])
    translate([66, 0, 285])
        rounded_box([28, 164, 56], 6, true);

    color([0.05, 0.88, 0.95])
    translate([51, 0, 285])
        rounded_box([4, 148, 44], 4, true);

    color([0.10, 0.12, 0.15]) {
        translate([49, 0, 285])
            cube([2, 32, 32], center = true);
        for (ty = [-52, 52]) {
            translate([49, ty, 285])
                cube([2, 14, 28], center = true);
        }
    }
    color([0.96, 0.98, 1.00])
    translate([48.2, 0, 285])
        cube([1.5, 16, 16], center = true);

    // 2D LiDAR V-Groove Retro-Reflective Fiducial Wedge at Z = 390 mm
    color([0.18, 0.21, 0.26])
    translate([69, 0, 390])
        rounded_box([22, 180, 48], 4, true);

    for (sy = [-1, 1]) {
        color([0.92, 0.95, 0.98])
        translate([58, sy * 38, 390])
        rotate([0, 0, sy * 18])
            cube([6, 72, 36], center = true);
    }

    // HMI OLED Control Panel, SOC Charge Bars & Status LEDs (Z = 465..555 mm)
    color([0.10, 0.12, 0.15])
    translate([77, -20, 510])
        rounded_box([6, 175, 92], 6, true);

    color([0.06, 0.18, 0.32])
    translate([73.5, -20, 510])
        rounded_box([3, 155, 74], 4, true);

    color([0.10, 0.95, 0.35])
    for (by = [-72 : 24 : 24]) {
        translate([71.8, by, 502])
            cube([2, 16, 28], center = true);
    }
    color([0.05, 0.85, 0.95])
    translate([71.8, -24, 530])
        cube([2, 128, 8], center = true);

    for (i = [0 : 3]) {
        led_cols = [
            [0.10, 0.95, 0.25],
            [0.05, 0.85, 0.95],
            [1.00, 0.75, 0.05],
            [0.45, 0.12, 0.12]
        ];
        color(led_cols[i])
        translate([77, -80 + i * 28, 445])
        rotate([0, 90, 0])
            cylinder(d = 10, h = 5, center = true);
    }

    // Cabinet-mounted Emergency Stop Button (Yellow collar + Red mushroom head)
    translate([79, 112, 510])
    rotate([0, -90, 0]) {
        color([0.95, 0.80, 0.05])
            cylinder(d = 42, h = 12);
        color([0.75, 0.75, 0.78])
        translate([0, 0, 12])
            cylinder(d = 16, h = 8);
        color([0.88, 0.08, 0.08])
        translate([0, 0, 18])
            cylinder(d1 = 28, d2 = 34, h = 12);
    }

    // High-Voltage Yellow Warning Badge on upper door
    color([0.96, 0.80, 0.05])
    translate([79, 112, 435])
    rotate([0, -90, 0])
        cylinder(d = 38, h = 3, $fn = 3);

    // Roof-mounted Industrial 3-Tier Andon Status Tower Light
    color([0.25, 0.28, 0.32])
    translate([165, 0, 646])
        cylinder(d = 64, h = 10, center = true);

    color([0.78, 0.80, 0.84])
    translate([165, 0, 672])
        cylinder(d = 22, h = 44, center = true);

    color([0.12, 0.13, 0.15])
    translate([165, 0, 700])
        cylinder(d = 52, h = 16, center = true);

    color([0.08, 0.55, 0.98])
    translate([165, 0, 718])
        cylinder(d = 50, h = 20, center = true);

    color([0.10, 0.95, 0.25])
    translate([165, 0, 738])
        cylinder(d = 50, h = 20, center = true);

    color([0.98, 0.72, 0.05])
    translate([165, 0, 758])
        cylinder(d = 50, h = 20, center = true);

    color([0.12, 0.13, 0.15])
    translate([165, 0, 773])
        cylinder(d1 = 52, d2 = 44, h = 10, center = true);
}

// Complete Industrial 48V Fast-Charging Station
module charging_station() {
    station_x = is_undef(charging_station_origin_x) ? 525 : charging_station_origin_x;
    contact_z = is_undef(charging_contact_z)        ? 95  : charging_contact_z;

    translate([station_x, 0, 0]) {
        charging_station_base_and_guards();
        charging_station_collector_head(contact_z);
        charging_station_cabinet_and_hmi();
    }
}


// ============================================================================
// 30. ANIMATION HELPER FUNCTIONS
// ============================================================================

function degrees_to_radians(angle_deg) =
    angle_deg * PI / 180;


// Lift height for animation modes 4 and 7.
// ============================================================================
// LIFT HEIGHT ANIMATION
// ============================================================================

// Single-cycle lift height profile for rack pickup/place (u in [0..1]):
// - Pickup ("pickup"):
//   Enters with lift = 0 (underneath tray & rails), raises at rack center to base_lift
//   (touches tray at u = 0.44), continues to cleared_lift (+20 mm off rails at u = 0.54),
//   backs out of rack at cleared_lift (u = 0.54..0.76), then lowers to 0 outside rack!
// - Place ("place"):
//   Raises outside rack from 0 to cleared_lift (u = 0.14..0.30), drives into rack at
//   cleared_lift (u = 0.30..0.50), lowers onto rails at base_lift (handoff at u = 0.60),
//   detaches lift down to 0 (u = 0.60..0.74), and backs out of rack with lift = 0!
function rack_cycle_lift_height(operation, level, u) =
    let(
        base_h    = rack_level_base_lift(level),
        cleared_h = rack_level_cleared_lift(level)
    )
    level == 0
    ? 0
    :
    operation == "pickup"
    ? (
        u < 0.38
        ? 0
        :
        u < 0.44
        ? lerp(0, base_h, smoothstep(local_time(u, 0.38, 0.44)))
        :
        u < 0.54
        ? lerp(base_h, cleared_h, smoothstep(local_time(u, 0.44, 0.54)))
        :
        u < 0.76
        ? cleared_h
        :
        u < 0.88
        ? lerp(cleared_h, 0, smoothstep(local_time(u, 0.76, 0.88)))
        :
        0
      )
    :
    (
        // operation == "place"
        u < 0.14
        ? 0
        :
        u < 0.30
        ? lerp(0, cleared_h, smoothstep(local_time(u, 0.14, 0.30)))
        :
        u < 0.50
        ? cleared_h
        :
        u < 0.60
        ? lerp(cleared_h, base_h, smoothstep(local_time(u, 0.50, 0.60)))
        :
        u < 0.74
        ? lerp(base_h, 0, smoothstep(local_time(u, 0.60, 0.74)))
        :
        0
    );

// Single-cycle robot X position profile for rack pickup/place (u in [0..1]):
function rack_cycle_robot_x(operation, u) =
    operation == "pickup"
    ? (
        // 1. Drive from start (-1050) to pre-rack scan stop (-750)
        u < 0.14
        ? lerp(rack_start_robot_x, rack_approach_robot_x, smoothstep(local_time(u, 0.00, 0.14)))
        :
        // 2. Pause at -750 to scan rack
        u < 0.22
        ? rack_approach_robot_x
        :
        // 3. Drive into rack center (+15 mm) with lift lowered
        u < 0.38
        ? lerp(rack_approach_robot_x, rack_dock_robot_x, smoothstep(local_time(u, 0.22, 0.38)))
        :
        // 4. Stationary at rack center (+15 mm) while lifting tray off rails
        u < 0.54
        ? rack_dock_robot_x
        :
        // 5. Back out of rack to -750 holding tray above rails
        u < 0.76
        ? lerp(rack_dock_robot_x, rack_approach_robot_x, smoothstep(local_time(u, 0.54, 0.76)))
        :
        // 6. Stationary at -750 while lowering lift to 0
        u < 0.88
        ? rack_approach_robot_x
        :
        // 7. Return to staging position (-1050)
        lerp(rack_approach_robot_x, rack_start_robot_x, smoothstep(local_time(u, 0.88, 1.00)))
      )
    :
    (
        // operation == "place"
        // 1. Drive from start (-1050) to pre-rack stop (-750)
        u < 0.14
        ? lerp(rack_start_robot_x, rack_approach_robot_x, smoothstep(local_time(u, 0.00, 0.14)))
        :
        // 2. Stationary at -750 while scanning and raising lift above target rail
        u < 0.30
        ? rack_approach_robot_x
        :
        // 3. Drive into rack center (+15 mm) holding tray above target rail
        u < 0.50
        ? lerp(rack_approach_robot_x, rack_dock_robot_x, smoothstep(local_time(u, 0.30, 0.50)))
        :
        // 4. Stationary at rack center (+15 mm) while setting tray on rails & lowering lift to 0
        u < 0.74
        ? rack_dock_robot_x
        :
        // 5. Back out empty from rack center (+15 mm) to start (-1050)
        lerp(rack_dock_robot_x, rack_start_robot_x, smoothstep(local_time(u, 0.74, 1.00)))
    );

function robot_lift_height(mode, t) =
    mode == 4
    ? (
        t <= 0.50
        ? lift_travel *
          smoothstep(
              local_time(
                  t,
                  0.00,
                  0.50
              )
          )

        :

        lift_travel *
        (
            1 -
            smoothstep(
                local_time(
                    t,
                    0.50,
                    1.00
                )
            )
        )
      )

    :

    mode == 7
    ? (
        // Lift remains lowered while robot travels.
        t < 0.30
        ? 0

        :

        // Raise lift at delivery point.
        t < 0.38
        ? lift_travel *
          smoothstep(
              local_time(
                  t,
                  0.30,
                  0.38
              )
          )

        :

        // Hold lift at maximum height.
        t < 0.46
        ? lift_travel

        :

        // Lower lift before robot departs.
        t < 0.54
        ? lift_travel *
          (
              1 -
              smoothstep(
                  local_time(
                      t,
                      0.46,
                      0.54
                  )
              )
          )

        :

        // Lift remains lowered during docking.
        0
      )

    :

    (mode == 9 || mode == 10)
    ? (
        rack_preferred_level != 0
        ? rack_cycle_lift_height(effective_rack_operation, selected_rack_level, t)
        : (
            t < 0.50
            ? rack_cycle_lift_height(effective_rack_operation, rack_active_level_at_t(mode, t), t * 2)
            : rack_cycle_lift_height(effective_rack_operation, rack_active_level_at_t(mode, t), (t - 0.50) * 2)
          )
      )

    :

    0;


// ============================================================================
// ROBOT X POSITION ANIMATION
// ============================================================================
//
// Mode 6:
// Robot approaches the charging station and stops at dock_robot_safe_x.
//
// Mode 7 mission sequence:
//   0.00 - 0.18 : Move from start point to delivery point
//   0.18 - 0.60 : Stop for lifting and delivery operation
//   0.60 - 0.72 : Move to pre-docking position
//   0.72 - 0.94 : Slowly approach charging station
//   0.94 - 1.00 : Hold at charging position
// ============================================================================

function robot_anim_x(mode, t) =
    mode == 1
    ? lerp(
        -straight_travel_dist / 2,
         straight_travel_dist / 2,
         smoothstep(t)
      )

    :

    mode == 6
    ? (
        t < 0.08
        ? -docking_target_distance

        :

        t < 0.82
        ? lerp(
            -docking_target_distance,
            dock_robot_safe_x,
            smoothstep(
                local_time(
                    t,
                    0.08,
                    0.82
                )
            )
          )

        :

        dock_robot_safe_x
      )

    :

    mode == 7
    ? (
        // Stage 1:
        // Robot drives from X = -700 mm to delivery point X = -180 mm.
        t < 0.18
        ? lerp(
            -700,
            -180,
            smoothstep(
                local_time(
                    t,
                    0.00,
                    0.18
                )
            )
          )

        :

        // Stage 2:
        // Robot remains stationary while the lift operates.
        t < 0.60
        ? -180

        :

        // Stage 3:
        // Robot moves to the pre-docking position.
        t < 0.72
        ? lerp(
            -180,
            -120,
            smoothstep(
                local_time(
                    t,
                    0.60,
                    0.72
                )
            )
          )

        :

        // Stage 4:
        // Slow final approach to the charging station.
        t < 0.94
        ? lerp(
            -120,
            dock_robot_safe_x,
            smoothstep(
                local_time(
                    t,
                    0.72,
                    0.94
                )
            )
          )

        :

        // Stage 5:
        // Robot remains at the safe charging position.
        dock_robot_safe_x
      )

    :

    (mode == 9 || mode == 10)
    ? (
        rack_preferred_level != 0
        ? rack_cycle_robot_x(effective_rack_operation, t)
        : (
            t < 0.50
            ? rack_cycle_robot_x(effective_rack_operation, t * 2)
            : rack_cycle_robot_x(effective_rack_operation, (t - 0.50) * 2)
          )
      )

    :

    0;

// ============================================================================
// ROBOT YAW ANIMATION
// ============================================================================
//
// Mode 2:
// Spin in place.
//
// Mode 7:
// Robot rotates 90 degrees at the delivery position,
// remains rotated during delivery, then rotates back to face the dock.
// ============================================================================

function robot_anim_yaw(mode, t) =
    mode == 2
    ? 360 * smoothstep(t)

    :

    mode == 7
    ? (
        // Robot initially faces forward.
        t < 0.22
        ? 0

        :

        // Rotate 90 degrees at delivery point.
        t < 0.30
        ? 90 *
          smoothstep(
              local_time(
                  t,
                  0.22,
                  0.30
              )
          )

        :

        // Hold sideways during lifting and delivery.
        t < 0.50
        ? 90

        :

        // Rotate back toward charging station.
        t < 0.58
        ? lerp(
            90,
            0,
            smoothstep(
                local_time(
                    t,
                    0.50,
                    0.58
                )
            )
          )

        :

        // Remain aligned with charging station.
        0
      )

    :

    0;


// Arc trajectory transform for mode 3.
module arc_path_transform(t) {
    theta =
        arc_turn_angle_deg *
        smoothstep(t);

    translate([
        arc_turn_radius * sin(theta),
        arc_turn_radius * (1 - cos(theta)),
        0
    ])
    rotate([0, 0, theta])
        children();
}


// Main robot animation transform.
module robot_animation_transform() {
    if (animation_mode == 3) {
        arc_path_transform($t)
            children();
    }
    else if (animation_mode == 8) {
        // Exploded animation remains fixed at world origin.
        children();
    }
    else {
        translate([
            robot_anim_x(animation_mode, $t),
            0,
            0
        ])
        rotate([
            0,
            0,
            robot_anim_yaw(animation_mode, $t)
        ])
            children();
    }
}


// ============================================================================
// 31. DRIVE WHEEL ANIMATION
// ============================================================================
// ============================================================================
// MODE 7 CUMULATIVE DRIVE DISTANCE
// ============================================================================
//
// Segment 1:
// X = -700 to X = -180
// Distance = 520 mm
//
// Segment 2:
// X = -180 to X = -120
// Distance = 60 mm
//
// Segment 3:
// X = -120 to X = dock_robot_safe_x
// Distance = dock_robot_safe_x + 120
// ============================================================================

function mission_drive_distance(t) =
    t < 0.18
    ? 520 *
      smoothstep(
          local_time(
              t,
              0.00,
              0.18
          )
      )

    :

    t < 0.60
    ? 520

    :

    t < 0.72
    ? 520 +
      60 *
      smoothstep(
          local_time(
              t,
              0.60,
              0.72
          )
      )

    :

    t < 0.94
    ? 580 +
      (
          dock_robot_safe_x + 120
      ) *
      smoothstep(
          local_time(
              t,
              0.72,
              0.94
          )
      )

    :

    580 +
    dock_robot_safe_x +
    120;
function left_drive_distance(mode, t) =
    mode == 1
    ? straight_travel_dist *
      smoothstep(t)

    :

    mode == 2
    ? -PI *
      drive_wheel_track *
      smoothstep(t)

    :

    mode == 3
    ? degrees_to_radians(
          arc_turn_angle_deg
      ) *
      (
          arc_turn_radius -
          drive_wheel_track / 2
      ) *
      smoothstep(t)

    :

    mode == 6
    ? (
        docking_target_distance +
        dock_robot_safe_x
      ) *
      smoothstep(
          local_time(
              t,
              0.08,
              0.82
          )
      )

    :

    mode == 7
    ? mission_drive_distance(t)

    :

    (mode == 9 || mode == 10)
    ? (robot_anim_x(mode, t) - rack_start_robot_x)

    :

    0;


function right_drive_distance(mode, t) =
    mode == 1
    ? straight_travel_dist *
      smoothstep(t)

    :

    mode == 2
    ? PI *
      drive_wheel_track *
      smoothstep(t)

    :

    mode == 3
    ? degrees_to_radians(
          arc_turn_angle_deg
      ) *
      (
          arc_turn_radius +
          drive_wheel_track / 2
      ) *
      smoothstep(t)

    :

    mode == 6
    ? (
        docking_target_distance +
        dock_robot_safe_x
      ) *
      smoothstep(
          local_time(
              t,
              0.08,
              0.82
          )
      )

    :

    mode == 7
    ? mission_drive_distance(t)

    :

    (mode == 9 || mode == 10)
    ? (robot_anim_x(mode, t) - rack_start_robot_x)

    :

    0;


// ============================================================================
// 32. CASTER ANIMATION
// ============================================================================

function animated_caster_swivel(cx, cy, mode, t) =
    mode == 2
    ? spin_caster_swivel_deg(cx, cy, 1)
    :
    mode == 3
    ? arc_caster_swivel_deg(
        cx,
        cy,
        arc_turn_radius
      )
    :
    mode == 5
    ? 55 * sin(
        360 * t +
        cx / 3 +
        cy / 4
      )
    :
    0;


function animated_caster_roll(cx, cy, mode, t) =
    mode == 1
    ? dist_to_wheel_deg(
        straight_travel_dist *
        smoothstep(t),
        caster_wheel_diameter
      )

    :

    mode == 2
    ? dist_to_wheel_deg(
        sqrt(
            cx * cx +
            cy * cy
        ) *
        degrees_to_radians(
            360 *
            smoothstep(t)
        ),
        caster_wheel_diameter
      )

    :

    mode == 3
    ? dist_to_wheel_deg(
        degrees_to_radians(
            arc_turn_angle_deg
        ) *
        sqrt(
            cx * cx +
            pow(
                arc_turn_radius - cy,
                2
            )
        ) *
        smoothstep(t),
        caster_wheel_diameter
      )

    :

    mode == 5
    ? 720 * t

    :

    mode == 6
    ? dist_to_wheel_deg(
        (
            docking_target_distance +
            dock_robot_safe_x
        ) *
        smoothstep(
            local_time(
                t,
                0.08,
                0.82
            )
        ),
        caster_wheel_diameter
      )

    :

    mode == 7
    ? dist_to_wheel_deg(
        mission_drive_distance(t),
        caster_wheel_diameter
      )

    :

    (mode == 9 || mode == 10)
    ? dist_to_wheel_deg(
        robot_anim_x(mode, t) - rack_start_robot_x,
        caster_wheel_diameter
      )

    :

    0;


// ============================================================================
// 33. FASTENER ASSEMBLY
// ============================================================================

module chassis_fastener_assembly(
    current_lift_height = 0
) {
    if (show_fasteners) {
        // Upper chassis structural deck bolts seated flush on chassis_top_z1
        // (bolting chassis_top into the upper longitudinal aluminum beams)
        for (
            sx = [-300, -150, 0, 150, 300],
            sy = [-1, 1]
        ) {
            fastener_m5(
                [
                    sx,
                    sy * long_beam_y,
                    chassis_top_z1
                ],
                16
            );
        }

        // Lift deck bolts (only shown when show_lift is enabled)
        if (show_lift) {
            for (
                dx = [-1, 1],
                dy = [-1, 1]
            ) {
                fastener_m5(
                    [
                        lift_center_x +
                        dx * lift_rail_x,

                        dy * lift_rail_y,

                        lift_lowered_deck_z +
                        current_lift_height +
                        2
                    ],
                    20
                );
            }
        }
    }
}


// ============================================================================
// 34. COLLISION DEBUG
// ============================================================================

module collision_debug_geometry() {
    if (collision_debug) {
        // Battery collision envelope
        color([1.00, 0.00, 0.00, 0.18])
        translate([
            0,
            0,
            chassis_bottom_z1 +
            battery_height / 2
        ])
        cube([
            battery_length,
            battery_width,
            battery_height
        ], center = true);

        // Lift guide collision envelopes
        color([1.00, 0.00, 0.00, 0.18])
        for (
            dx = [-1, 1],
            dy = [-1, 1]
        ) {
            translate([
                lift_center_x +
                dx * lift_rail_x,

                dy * lift_rail_y,

                lift_rail_bottom_z +
                (
                    lift_lowered_deck_z -
                    lift_rail_bottom_z
                ) / 2
            ])
            cylinder(
                d = lift_sleeve_diameter + 8,
                h =
                    lift_lowered_deck_z -
                    lift_rail_bottom_z,
                center = true
            );
        }
    }
}


// ============================================================================
// ============================================================================
// 35. COMPLETE ROBOT ASSEMBLY
// Sequential exploded assembly animation for mode 8
// ============================================================================

module factory_delivery_amr() {
    current_lift_height =
        robot_lift_height(
            animation_mode,
            $t
        );

    left_wheel_angle =
        dist_to_wheel_deg(
            left_drive_distance(
                animation_mode,
                $t
            ),
            drive_wheel_diameter
        );

    right_wheel_angle =
        dist_to_wheel_deg(
            right_drive_distance(
                animation_mode,
                $t
            ),
            drive_wheel_diameter
        );

    manual_factor = manual_explode_factor();

    // ------------------------------------------------------------------------
    // Sequential animation factors
    // Components are removed from outside to inside.
    // During reassembly, the sequence automatically runs in reverse.
    // ------------------------------------------------------------------------

    payload_factor =
        animation_mode == 8
        ? exploded_component_factor($t, 0.00, 0.07)
        : manual_factor;

    body_factor =
        animation_mode == 8
        ? exploded_component_factor($t, 0.025, 0.07)
        : manual_factor;

    sensor_factor =
        animation_mode == 8
        ? exploded_component_factor($t, 0.050, 0.07)
        : manual_factor;

    moving_lift_factor =
        animation_mode == 8
        ? exploded_component_factor($t, 0.075, 0.07)
        : manual_factor;

    fixed_lift_factor =
        animation_mode == 8
        ? exploded_component_factor($t, 0.100, 0.07)
        : manual_factor;

    electronics_factor =
        animation_mode == 8
        ? exploded_component_factor($t, 0.125, 0.07)
        : manual_factor;

    battery_factor =
        animation_mode == 8
        ? exploded_component_factor($t, 0.150, 0.07)
        : manual_factor;

    drive_factor =
        animation_mode == 8
        ? exploded_component_factor($t, 0.175, 0.07)
        : manual_factor;

    caster_factor =
        animation_mode == 8
        ? exploded_component_factor($t, 0.200, 0.07)
        : manual_factor;

    upper_chassis_factor =
        animation_mode == 8
        ? exploded_component_factor($t, 0.225, 0.07)
        : manual_factor;

    fastener_factor =
        animation_mode == 8
        ? exploded_component_factor($t, 0.250, 0.07)
        : manual_factor;


    // ========================================================================
    // A. MAIN CHASSIS FRAME
    // The structural frame remains at the robot origin as the assembly core.
    // ========================================================================

    if (show_chassis) {
        chassis_bottom();
        chassis_longitudinal_beams();
        chassis_cross_beams();

        // Upper structural plate moves upward.
        translate([
            0,
            0,
            explode_gap *
            0.70 *
            upper_chassis_factor
        ])
            chassis_top();
    }


    // ========================================================================
    // B. BODY SHELL
    // Shell rises vertically to expose internal components.
    // ========================================================================

    if (show_body) {
        translate([
            0,
            0,
            explode_gap *
            2.20 *
            body_factor
        ])
            body_shell();
    }


    // ========================================================================
    // C. LEFT AND RIGHT DRIVE ASSEMBLIES
    // Drive units move outwards along the Y axis.
    // ========================================================================

    if (show_drive) {
        // Left drive moves toward +Y.
        translate([
            0,
            explode_gap *
            1.45 *
            drive_factor,
            explode_gap *
            0.15 *
            drive_factor
        ])
            drive_assembly(
                1,
                left_wheel_angle
            );

        // Right drive moves toward -Y.
        translate([
            0,
            -explode_gap *
            1.45 *
            drive_factor,
            explode_gap *
            0.15 *
            drive_factor
        ])
            drive_assembly(
                -1,
                right_wheel_angle
            );
    }


    // ========================================================================
    // D. FOUR CASTER ASSEMBLIES
    // Casters separate diagonally from their mounting positions.
    // ========================================================================

    if (show_casters) {
        for (pt = caster_positions) {
            caster_swivel =
                animated_caster_swivel(
                    pt[0],
                    pt[1],
                    animation_mode,
                    $t
                );

            caster_roll =
                animated_caster_roll(
                    pt[0],
                    pt[1],
                    animation_mode,
                    $t
                );

            caster_direction_x =
                pt[0] >= 0
                ? 1
                : -1;

            caster_direction_y =
                pt[1] >= 0
                ? 1
                : -1;

            translate([
                caster_direction_x *
                explode_gap *
                0.85 *
                caster_factor,

                caster_direction_y *
                explode_gap *
                0.85 *
                caster_factor,

                -explode_gap *
                0.25 *
                caster_factor
            ])
                caster_assembly(
                    pt,
                    caster_swivel,
                    caster_roll
                );
        }
    }


    // ========================================================================
    // E. BATTERY AND BMS
    // ========================================================================

    if (show_battery) {
        // Battery slides toward the rear.
        translate([
            -explode_gap *
            1.25 *
            battery_factor,
            0,
            explode_gap *
            0.20 *
            battery_factor
        ])
            battery_pack();

        // BMS separates toward rear-left.
        translate([
            -explode_gap *
            1.35 *
            battery_factor,

            explode_gap *
            0.50 *
            battery_factor,

            explode_gap *
            0.45 *
            battery_factor
        ])
            bms();
    }


    // ========================================================================
    // F. ELECTRONIC COMPONENTS
    // Each unit moves in an individual direction.
    // ========================================================================

    if (show_electronics) {
        // Left motor driver
        translate([
            explode_gap *
            0.70 *
            electronics_factor,

            explode_gap *
            1.05 *
            electronics_factor,

            explode_gap *
            0.65 *
            electronics_factor
        ])
            motor_driver(1);

        // Right motor driver
        translate([
            explode_gap *
            0.70 *
            electronics_factor,

            -explode_gap *
            1.05 *
            electronics_factor,

            explode_gap *
            0.65 *
            electronics_factor
        ])
            motor_driver(-1);

        // ESP32-S3 controller
        translate([
            explode_gap *
            1.10 *
            electronics_factor,

            -explode_gap *
            0.55 *
            electronics_factor,

            explode_gap *
            1.05 *
            electronics_factor
        ])
            esp32_s3_controller();

        // Jetson/SBC
        translate([
            explode_gap *
            1.10 *
            electronics_factor,

            explode_gap *
            0.55 *
            electronics_factor,

            explode_gap *
            1.05 *
            electronics_factor
        ])
            jetson_or_sbc();

        // Power distribution unit
        translate([
            -explode_gap *
            1.15 *
            electronics_factor,

            0,

            explode_gap *
            0.85 *
            electronics_factor
        ])
            power_distribution();

        // IMU moves directly upward.
        translate([
            0,
            0,
            explode_gap *
            1.30 *
            electronics_factor
        ])
            imu_module();
    }


    // ========================================================================
    // G. SENSORS AND SAFETY COMPONENTS
    // ========================================================================

    if (show_sensors) {
        // LiDAR moves forward and upward.
        translate([
            explode_gap *
            0.45 *
            sensor_factor,

            0,

            explode_gap *
            1.85 *
            sensor_factor
        ])
            lidar_2d();

        // Depth camera moves forward.
        translate([
            explode_gap *
            1.25 *
            sensor_factor,

            0,

            explode_gap *
            0.45 *
            sensor_factor
        ])
            realsense_d455();

        // E-stop moves rear-left and upward.
        translate([
            -explode_gap *
            0.85 *
            sensor_factor,

            explode_gap *
            0.85 *
            sensor_factor,

            explode_gap *
            1.25 *
            sensor_factor
        ])
            emergency_stop();

        // Charging contacts move forward and downward slightly.
        translate([
            explode_gap *
            1.05 *
            sensor_factor,

            0,

            -explode_gap *
            0.15 *
            sensor_factor
        ])
            charging_contacts();
    }


    // ========================================================================
    // H. FIXED LIFT STRUCTURE
    // ========================================================================

    if (show_lift) {
        // Fixed lift frame moves rearward.
        translate([
            -explode_gap *
            0.55 *
            fixed_lift_factor,

            0,

            explode_gap *
            0.55 *
            fixed_lift_factor
        ])
            lift_fixed_part();

        // Four guide rails move vertically upward.
        translate([
            0,
            0,
            explode_gap *
            1.10 *
            fixed_lift_factor
        ])
            lift_guide_rails();

        // Central lift actuator moves forward and upward.
        translate([
            explode_gap *
            0.70 *
            fixed_lift_factor,

            0,

            explode_gap *
            0.85 *
            fixed_lift_factor
        ])
            lift_actuator(
                current_lift_height
            );

        // Moving platform rises above the fixed lift mechanism.
        translate([
            0,
            0,
            explode_gap *
            1.65 *
            moving_lift_factor
        ])
            lift_moving_part(
                current_lift_height
            );
    }


    // ========================================================================
    // I. PAYLOAD
    // Payload rises separately above the moving lift platform.
    // ========================================================================

    show_payload_for_mode =
        show_payload &&
        (
            (animation_mode != 9 && animation_mode != 10) ||
            robot_carrying_tray_at_t(animation_mode, $t)
        );

    active_payload_color =
        (animation_mode == 9 || animation_mode == 10) &&
        rack_active_level_at_t(animation_mode, $t) == 2
        ? [0.52, 0.65, 0.82]
        : [0.72, 0.52, 0.32];

    if (show_payload_for_mode) {
        translate([
            0,
            0,
            explode_gap *
            2.50 *
            payload_factor
        ])
            payload_20kg(
                current_lift_height,
                active_payload_color
            );
    }


    // ========================================================================
    // J. FASTENERS
    // Fasteners move upward and slightly outward.
    // ========================================================================

    if (show_fasteners) {
        translate([
            explode_gap *
            0.20 *
            fastener_factor,

            0,

            explode_gap *
            1.85 *
            fastener_factor
        ])
            chassis_fastener_assembly(
                current_lift_height
            );
    }


    collision_debug_geometry();
    coordinate_axes_debug();
}


// ============================================================================
// ============================================================================
// 36. COMPLETE FACTORY SCENE
// ============================================================================
// ============================================================================
// DOCKING CONTACT DEBUG
// ============================================================================

show_docking_debug = false;

module docking_contact_debug() {
    if (
        show_docking_debug &&
        (
            animation_mode == 6 ||
            animation_mode == 7
        )
    ) {
        // Red plane:
        // Physical front contact face of charging station.
        color([1.00, 0.10, 0.10, 0.55])
        translate([
            station_contact_face_x,
            0,
            charging_contact_z
        ])
            cube(
                [3, 170, 75],
                center = true
            );

        // Green marker:
        // Correct final robot center position.
        color([0.10, 1.00, 0.20, 0.85])
        translate([
            dock_robot_safe_x,
            0,
            8
        ])
            cylinder(
                d = 22,
                h = 16,
                center = true
            );

        // Yellow line from robot stop position to station.
        color([1.00, 0.80, 0.05, 0.65])
        translate([
            (
                dock_robot_safe_x +
                station_contact_face_x
            ) / 2,
            0,
            3
        ])
            cube(
                [
                    station_contact_face_x -
                    dock_robot_safe_x,
                    5,
                    5
                ],
                center = true
            );
    }
}
module factory_scene() {
    // Function 9 (Pickup) & 10 (Place) use the smart-rack scene.
    if (animation_mode == 9 || animation_mode == 10) {
        function_9_smart_rack_scene();
    }
    else {
        // World-fixed debug geometry.
        corridor_debug();
        turning_envelope_debug();
        docking_contact_debug();

        // Charging station appears automatically in modes 6 and 7.
        if (
            show_charging_station ||
            animation_mode == 6 ||
            animation_mode == 7
        ) {
            charging_station();
        }

        // Robot and robot-attached debug geometry.
        robot_animation_transform() {
            factory_delivery_amr();
            lidar_scan_debug();
            camera_fov_debug();
            support_polygon_debug();
        }
    }
}


// ============================================================================
// ============================================================================
// FINAL MODEL GENERATION
// ============================================================================
//
// main.scad opened directly:
//   export_only is undefined -> display factory_scene()
//
// main.scad included by an export file:
//   export_only = true -> do not display factory_scene()
// ============================================================================

if (is_undef(export_only)) {
    factory_scene();
}