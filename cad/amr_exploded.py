"""Factory Delivery AMR — Native CAD Assembly & Visual Exploded Breakdown.

Faithfully translates the parametric Factory Delivery AMR (cad/main.scad) into
native build123d B-rep geometry for cadgen:
  1. `factory_delivery_amr_exploded`: Architectural 3D exploded view (bản phân tách
     trực quan) with 29 functional subsystems cleanly separated along vertical Z
     layers and radial X/Y axes with zero occlusion.
  2. `factory_delivery_amr`: Assembled AMR with sequential GLB explode-and-reassemble
     animation (`exploded_breakdown`).

Units: millimetres (mm), +X forward, +Y left, +Z up. Ground plane at Z = 0.
"""

import build123d as bd
from bd_materials import metals
from cadgen import memo, model
from cadgen.gltf import animation

# ============================================================================
# 1. VERIFIED & PARAMETRIC DIMENSIONS (from cad/main.scad)
# ============================================================================
ROBOT_LENGTH = 850.0
ROBOT_WIDTH = 720.0
ROBOT_BODY_HEIGHT = 300.0
BODY_CORNER_RADIUS = 80.0

CHASSIS_MARGIN = 22.0
CHASSIS_LENGTH = ROBOT_LENGTH - 2 * CHASSIS_MARGIN  # 806 mm
CHASSIS_WIDTH = ROBOT_WIDTH - 2 * CHASSIS_MARGIN    # 676 mm
CHASSIS_CORNER_R = BODY_CORNER_RADIUS - CHASSIS_MARGIN  # 58 mm
CHASSIS_PLATE_T = 6.0
GROUND_CLEARANCE = 35.0
CHASSIS_BOTTOM_Z0 = GROUND_CLEARANCE                # 35 mm
CHASSIS_BOTTOM_Z1 = CHASSIS_BOTTOM_Z0 + CHASSIS_PLATE_T  # 41 mm
CHASSIS_TOP_Z0 = 250.0
CHASSIS_TOP_Z1 = CHASSIS_TOP_Z0 + CHASSIS_PLATE_T   # 256 mm

BEAM_WIDTH = 16.0
BEAM_HEIGHT = 28.0
LONG_BEAM_Y = 118.0

DRIVE_WHEEL_D = 220.0
DRIVE_WHEEL_R = DRIVE_WHEEL_D / 2.0                 # 110 mm
DRIVE_WHEEL_W = 60.0
DRIVE_WHEEL_TRACK = 620.0
DRIVE_AXLE_Z = DRIVE_WHEEL_R                        # 110 mm
WHEEL_INNER_Y = (DRIVE_WHEEL_TRACK / 2.0) - (DRIVE_WHEEL_W / 2.0)  # 280 mm

GEARBOX_D = 82.0
GEARBOX_L = 52.0
GEARBOX_INNER_Y = WHEEL_INNER_Y - 4.0 - GEARBOX_L   # 224 mm
MOTOR_D = 80.0
MOTOR_L = 74.0
MOTOR_INNER_Y = GEARBOX_INNER_Y - MOTOR_L           # 150 mm
ENCODER_D = 52.0
ENCODER_L = 18.0

CASTER_D = 100.0
CASTER_R = CASTER_D / 2.0                           # 50 mm
CASTER_W = 35.0
CASTER_OFFSET_X = 310.0
CASTER_OFFSET_Y = 245.0
CASTER_TRAIL = 24.0
CASTER_MOUNT_Z = 125.0

BATTERY_L = 270.0
BATTERY_W = 210.0
BATTERY_H = 124.0

BMS_L = 116.0
BMS_W = 82.0
BMS_H = 26.0

LIFT_CENTER_X = -15.0
LIFT_LOWERED_DECK_Z = 330.0
LIFT_DECK_T = 12.0
LIFT_TRAVEL = 290.0
LIFT_RAIL_X = 170.0
LIFT_RAIL_Y = 150.0
LIFT_RAIL_D = 24.0
LIFT_SLEEVE_D = 38.0
LIFT_FIXED_BASE_Z = 180.0
LIFT_RAIL_BOTTOM_Z = 55.0

PAYLOAD_DECK_L = 650.0
PAYLOAD_DECK_W = 560.0
PAYLOAD_BOX_L = 500.0
PAYLOAD_BOX_W = 400.0
PAYLOAD_BOX_H = 50.0
WING_SPAN_Y = 156.0
WING_LEN_X = 420.0
WING_THICK = 16.0
WING_Z_OFFSET = 20.0

LIDAR_POS_X = 368.0
LIDAR_MOUNT_Z = 390.0
LIDAR_D = 76.0
LIDAR_H = 60.0

CAMERA_POS_X = 415.0
CAMERA_MOUNT_Z = 285.0
D455_L = 26.0
D455_W = 124.0
D455_H = 29.0

ESTOP_POS_X = -382.0
ESTOP_POS_Y = 210.0
CHARGING_CONTACT_Z = 95.0

SHELL_WALL_T = 4.0
BUMPER_H = 46.0
BUMPER_PROTRUSION = 10.0


# ============================================================================
# 2. GEOMETRY HELPERS
# ============================================================================
def rounded_plate(sx: float, sy: float, sz: float, radius: float) -> bd.Solid:
    """Create a box of (sx, sy, sz) centered at origin with vertical Z edges filleted."""
    r = min(radius, min(sx, sy) / 2.0 - 0.5)
    sketch = bd.RectangleRounded(sx, sy, r)
    return bd.extrude(sketch, amount=sz / 2.0, both=True)


def paint_part(shape, hex_color: str, label: str, metallic: bool = False, roughness: float = 0.45):
    """Assign material, sRGB hex colour, and label to a build123d leaf solid."""
    if metallic:
        shape.material = metals.aluminum()
        shape.material.pbr = shape.material.pbr.override(roughness=roughness)
    shape.color = hex_color
    shape.label = label
    return shape


def paint_solids(
    shapes: list[bd.Shape],
    hex_color: str,
    base_label: str,
    metallic: bool = False,
    roughness: float = 0.45,
) -> list[bd.Solid]:
    """Extract all leaf solids from `shapes` and paint/label each one."""
    out: list[bd.Solid] = []
    idx = 1
    for sh in shapes:
        for sol in sh.solids():
            lbl = base_label if len(shapes) == 1 and len(sh.solids()) == 1 else f"{base_label}_{idx}"
            paint_part(sol, hex_color, lbl, metallic=metallic, roughness=roughness)
            out.append(sol)
            idx += 1
    return out


# ============================================================================
# 3. SUBSYSTEM BUILDERS
# ============================================================================
@memo
def build_chassis_core() -> bd.Compound:
    """Lower chassis plate, longitudinal beams, cross beams, and caster bridges."""
    zc_bot = CHASSIS_BOTTOM_Z0 + CHASSIS_PLATE_T / 2.0
    bot_plate = bd.Pos(0, 0, zc_bot) * rounded_plate(
        CHASSIS_LENGTH, CHASSIS_WIDTH, CHASSIS_PLATE_T, CHASSIS_CORNER_R
    )
    cuts = []
    for side in (-1, 1):
        cuts.append(
            bd.Pos(0, side * (DRIVE_WHEEL_TRACK / 2.0), zc_bot)
            * bd.Box(DRIVE_WHEEL_D + 28, DRIVE_WHEEL_W + 12, CHASSIS_PLATE_T + 4)
        )
    for sx in (-1, 1):
        for sy in (-1, 1):
            cuts.append(
                bd.Pos(sx * CASTER_OFFSET_X, sy * CASTER_OFFSET_Y, zc_bot)
                * bd.Cylinder(CASTER_TRAIL + CASTER_R + 10, CHASSIS_PLATE_T + 4)
            )
    bot_plate = (bot_plate - cuts).solids()[0]
    paint_part(bot_plate, "#737A85", "chassis_bottom_plate", metallic=True, roughness=0.4)

    beam_len = CHASSIS_LENGTH - 110.0
    beam_solids = []
    for sy in (-1, 1):
        beam_solids.append(
            bd.Pos(0, sy * LONG_BEAM_Y, CHASSIS_BOTTOM_Z1 + BEAM_HEIGHT / 2.0)
            * bd.Box(beam_len, BEAM_WIDTH, BEAM_HEIGHT)
        )
        beam_solids.append(
            bd.Pos(0, sy * LONG_BEAM_Y, CHASSIS_TOP_Z0 - BEAM_HEIGHT / 2.0)
            * bd.Box(beam_len, BEAM_WIDTH, BEAM_HEIGHT)
        )
        for px in (-310.0, -195.0, 195.0, 310.0):
            beam_solids.append(
                bd.Pos(px, sy * LONG_BEAM_Y, (CHASSIS_BOTTOM_Z1 + CHASSIS_TOP_Z0) / 2.0)
                * bd.Box(24.0, BEAM_WIDTH, CHASSIS_TOP_Z0 - CHASSIS_BOTTOM_Z1)
            )
        bracket_y = sy * (WHEEL_INNER_Y - 19.0)
        bh_h = DRIVE_AXLE_Z + 46.0 - CHASSIS_BOTTOM_Z1
        bh_zc = (CHASSIS_BOTTOM_Z1 + DRIVE_AXLE_Z + 46.0) / 2.0
        bulkhead = bd.Pos(0, bracket_y, bh_zc) * bd.Box(104.0, 10.0, bh_h)
        bore = (
            bd.Pos(0, bracket_y, DRIVE_AXLE_Z)
            * bd.Rotation(90, 0, 0)
            * bd.Cylinder(GEARBOX_D / 2.0 + 0.5, 16.0)
        )
        beam_solids.append(bulkhead - bore)

    for cx in (-310.0, -152.0, 152.0, 310.0):
        beam_solids.append(
            bd.Pos(cx, 0, CHASSIS_BOTTOM_Z1 + BEAM_HEIGHT / 2.0)
            * bd.Box(BEAM_WIDTH, 2 * LONG_BEAM_Y + BEAM_WIDTH, BEAM_HEIGHT)
        )
        upper_span = (2 * LONG_BEAM_Y + BEAM_WIDTH) if abs(cx) > 200 else (CHASSIS_WIDTH - 160.0)
        beam_solids.append(
            bd.Pos(cx, 0, CHASSIS_TOP_Z0 - BEAM_HEIGHT / 2.0)
            * bd.Box(BEAM_WIDTH, upper_span, BEAM_HEIGHT)
        )

    for sx in (-1, 1):
        for sy in (-1, 1):
            px, py = sx * CASTER_OFFSET_X, sy * CASTER_OFFSET_Y
            beam_solids.append(
                bd.Pos(px, py, CASTER_MOUNT_Z + 4.0) * rounded_plate(104.0, 104.0, 8.0, 16.0)
            )
            beam_solids.append(
                bd.Pos(px - sx * 44.0, py - sy * 44.0, (CHASSIS_BOTTOM_Z1 + CHASSIS_TOP_Z0) / 2.0)
                * bd.Cylinder(13.0, CHASSIS_TOP_Z0 - CHASSIS_BOTTOM_Z1)
            )

    painted_beams = paint_solids(beam_solids, "#8C949E", "frame_beam", metallic=True, roughness=0.35)
    return bd.Compound(children=[bot_plate, *painted_beams], label="chassis_core")


@memo
def build_chassis_top_deck() -> bd.Solid:
    """Upper 6mm structural deck plate at Z = 250..256 mm."""
    zc_top = CHASSIS_TOP_Z0 + CHASSIS_PLATE_T / 2.0
    main_plate = bd.Pos(0, 0, zc_top) * rounded_plate(
        CHASSIS_LENGTH, CHASSIS_WIDTH, CHASSIS_PLATE_T, CHASSIS_CORNER_R
    )
    nose_tongue = bd.Pos(395.0, 0, zc_top) * rounded_plate(46.0, 136.0, CHASSIS_PLATE_T, 8.0)
    top_plate = main_plate + nose_tongue

    holes = [
        bd.Pos(LIFT_CENTER_X, 0, zc_top) * bd.Cylinder(38.0, CHASSIS_PLATE_T + 4.0)
    ]
    for dx in (-1, 1):
        for dy in (-1, 1):
            holes.append(
                bd.Pos(LIFT_CENTER_X + dx * LIFT_RAIL_X, dy * LIFT_RAIL_Y, zc_top)
                * bd.Cylinder((LIFT_SLEEVE_D + 10.0) / 2.0, CHASSIS_PLATE_T + 4.0)
            )
    for sy in (-1, 1):
        holes.append(
            bd.Pos(250.0, sy * 175.0, zc_top)
            * rounded_plate(64.0, 36.0, CHASSIS_PLATE_T + 4.0, 8.0)
        )
    top_plate = (top_plate - holes).solids()[0]
    return paint_part(top_plate, "#808791", "chassis_top_deck", metallic=True, roughness=0.38)


@memo
def build_drive_unit(side: int) -> bd.Compound:
    """Single-side 180W BLDC differential drive unit (wheel + gearbox + motor + encoder)."""
    tag = "left" if side > 0 else "right"
    wy = side * (DRIVE_WHEEL_TRACK / 2.0)

    tire_outer = bd.Rotation(90, 0, 0) * bd.Cylinder(DRIVE_WHEEL_R, DRIVE_WHEEL_W)
    tire_inner = bd.Rotation(90, 0, 0) * bd.Cylinder(DRIVE_WHEEL_R - 18.0, DRIVE_WHEEL_W + 2.0)
    tire = (bd.Pos(0, wy, DRIVE_AXLE_Z) * (tire_outer - tire_inner)).solids()[0]
    paint_part(tire, "#242429", f"{tag}_tire")

    hub_solid = bd.Rotation(90, 0, 0) * bd.Cylinder(DRIVE_WHEEL_R - 17.5, DRIVE_WHEEL_W - 8.0)
    pockets = [
        bd.Rotation(0, a, 0)
        * bd.Pos(48.0, 0, 0)
        * bd.Rotation(90, 0, 0)
        * bd.Cylinder(13.0, DRIVE_WHEEL_W)
        for a in range(0, 360, 60)
    ]
    boss = bd.Rotation(90, 0, 0) * bd.Cylinder(23.0, DRIVE_WHEEL_W + 4.0)
    hub = (bd.Pos(0, wy, DRIVE_AXLE_Z) * ((hub_solid - pockets) + boss)).solids()[0]
    paint_part(hub, "#C7CCD6", f"{tag}_wheel_hub", metallic=True, roughness=0.25)

    gb_y = side * (WHEEL_INNER_Y - 4.0 - GEARBOX_L / 2.0)
    gb_body = bd.Rotation(90, 0, 0) * bd.Cylinder(GEARBOX_D / 2.0, GEARBOX_L)
    gb_flange = (
        bd.Pos(0, side * (GEARBOX_L / 2.0 - 5.0), 0)
        * bd.Rotation(90, 0, 0)
        * rounded_plate(GEARBOX_D + 12.0, GEARBOX_D + 12.0, 10.0, 8.0)
    )
    gearbox = (bd.Pos(0, gb_y, DRIVE_AXLE_Z) * (gb_body + gb_flange)).solids()[0]
    paint_part(gearbox, "#525961", f"{tag}_gearbox", metallic=True, roughness=0.4)

    mot_y = side * (GEARBOX_INNER_Y - MOTOR_L / 2.0)
    mot_body = (
        bd.Pos(0, mot_y, DRIVE_AXLE_Z)
        * (
            bd.Rotation(90, 0, 0) * bd.Cylinder(MOTOR_D / 2.0, MOTOR_L)
            + bd.Pos(0, -side * MOTOR_L / 4.0, MOTOR_D / 2.0 + 6.0) * bd.Box(24.0, 22.0, 16.0)
        )
    ).solids()[0]
    paint_part(mot_body, "#2E333D", f"{tag}_bldc_motor", metallic=True, roughness=0.45)

    enc_y = side * (MOTOR_INNER_Y - ENCODER_L / 2.0)
    encoder = (
        bd.Pos(0, enc_y, DRIVE_AXLE_Z)
        * bd.Rotation(90, 0, 0)
        * bd.Cylinder(ENCODER_D / 2.0, ENCODER_L)
    )
    paint_part(encoder, "#38734D", f"{tag}_encoder")

    return bd.Compound(
        children=[tire, hub, gearbox, mot_body, encoder],
        label=f"drive_module_{tag}",
    )


@memo
def build_caster_unit(cx: float, cy: float, name: str) -> bd.Compound:
    """Corner 360-deg swivel caster assembly at (cx, cy)."""
    top_plate = bd.Pos(cx, cy, CASTER_MOUNT_Z - 4.0) * rounded_plate(92.0, 92.0, 8.0, 10.0)
    swivel_ring = bd.Pos(cx, cy, CASTER_MOUNT_Z - 14.0) * bd.Cylinder(34.0, 12.0)
    crown = bd.Pos(cx - CASTER_TRAIL / 2.0, cy, CASTER_MOUNT_Z - 23.0) * rounded_plate(
        54.0 + CASTER_TRAIL, 62.0, 6.0, 8.0
    )
    leg_h = (CASTER_MOUNT_Z - 23.0) - CASTER_R + 2.0
    legs = [
        bd.Pos(cx - CASTER_TRAIL / 2.0, cy + sy * (CASTER_W / 2.0 + 6.0), CASTER_R + leg_h / 2.0 - 1.0)
        * bd.Box(26.0, 6.0, leg_h)
        for sy in (-1, 1)
    ]
    axle = (
        bd.Pos(cx - CASTER_TRAIL, cy, CASTER_R)
        * bd.Rotation(90, 0, 0)
        * bd.Cylinder(6.0, CASTER_W + 22.0)
    )
    yoke_solids = paint_solids(
        [top_plate, swivel_ring, crown, *legs, axle],
        "#9EA6B0",
        f"{name}_yoke",
        metallic=True,
        roughness=0.35,
    )

    wheel_tire = (
        bd.Pos(cx - CASTER_TRAIL, cy, CASTER_R)
        * bd.Rotation(90, 0, 0)
        * (bd.Cylinder(CASTER_R, CASTER_W) - bd.Cylinder(CASTER_R - 13.0, CASTER_W + 2.0))
    ).solids()[0]
    paint_part(wheel_tire, "#29292E", f"{name}_wheel")

    wheel_core = (
        bd.Pos(cx - CASTER_TRAIL, cy, CASTER_R)
        * bd.Rotation(90, 0, 0)
        * bd.Cylinder(CASTER_R - 12.5, CASTER_W - 4.0)
    )
    paint_part(wheel_core, "#C2C7D1", f"{name}_core", metallic=True, roughness=0.3)

    return bd.Compound(children=[*yoke_solids, wheel_tire, wheel_core], label=name)


@memo
def build_power_system() -> tuple[bd.Compound, bd.Compound]:
    """48V 30Ah Li-ion battery pack and Smart BMS module."""
    bat_zc = CHASSIS_BOTTOM_Z1 + BATTERY_H / 2.0
    bat_box = bd.Pos(0, 0, bat_zc) * rounded_plate(BATTERY_L, BATTERY_W, BATTERY_H, 10.0)
    term_pos = bd.Pos(BATTERY_L / 2.0, 45.0, bat_zc + BATTERY_H / 2.0 - 22.0) * bd.Box(16.0, 22.0, 22.0)
    term_neg = bd.Pos(BATTERY_L / 2.0, -45.0, bat_zc + BATTERY_H / 2.0 - 22.0) * bd.Box(16.0, 22.0, 22.0)
    paint_part(bat_box, "#1A2E4D", "battery_enclosure_48v_30ah")
    paint_part(term_pos, "#D91F1F", "terminal_pos_48v")
    paint_part(term_neg, "#1F1F24", "terminal_neg_gnd")
    battery = bd.Compound(children=[bat_box, term_pos, term_neg], label="battery_pack_48v")

    bms_zc = CHASSIS_BOTTOM_Z1 + BMS_H / 2.0
    bms_body = bd.Pos(-232.0, 60.0, bms_zc) * rounded_plate(BMS_L, BMS_W, BMS_H, 6.0)
    paint_part(bms_body, "#266B40", "bms_housing")
    bms_fins = paint_solids(
        [
            bd.Pos(-232.0 + fx, 60.0, CHASSIS_BOTTOM_Z1 + BMS_H + 4.0) * bd.Box(4.0, 68.0, 8.0)
            for fx in range(-42, 43, 14)
        ],
        "#B3B8BF",
        "bms_fin",
        metallic=True,
        roughness=0.3,
    )
    bms = bd.Compound(children=[bms_body, *bms_fins], label="smart_bms")

    return battery, bms


@memo
def build_electronics() -> dict[str, bd.Shape]:
    """Motor drivers, ESP32-S3 MCU, Jetson Orin SBC, PDU, and 9-axis IMU."""
    parts = {}

    for side, tag in ((1, "left"), (-1, "right")):
        drv = bd.Pos(95.0, side * 195.0, CHASSIS_BOTTOM_Z1 + 21.0) * rounded_plate(88.0, 66.0, 42.0, 6.0)
        sink = bd.Pos(95.0, side * 195.0, CHASSIS_BOTTOM_Z1 + 46.0) * rounded_plate(80.0, 58.0, 8.0, 3.0)
        paint_part(drv, "#292E38", f"motor_driver_{tag}_body")
        paint_part(sink, "#ADB3BD", f"motor_driver_{tag}_sink", metallic=True, roughness=0.3)
        parts[f"motor_driver_{tag}"] = bd.Compound(children=[drv, sink], label=f"motor_driver_{tag}")

    esp_base = bd.Pos(232.0, -56.0, CHASSIS_BOTTOM_Z1 + 5.0) * rounded_plate(104.0, 76.0, 10.0, 5.0)
    esp_pcb = bd.Pos(232.0, -56.0, CHASSIS_BOTTOM_Z1 + 14.0) * rounded_plate(95.0, 70.0, 8.0, 4.0)
    esp_rf = bd.Pos(247.0, -56.0, CHASSIS_BOTTOM_Z1 + 21.0) * bd.Box(32.0, 26.0, 6.0)
    paint_part(esp_base, "#3D454F", "esp32_carrier_base")
    paint_part(esp_pcb, "#148038", "esp32_s3_pcb")
    paint_part(esp_rf, "#BFC7CC", "esp32_rf_shield", metallic=True, roughness=0.25)
    parts["esp32_s3_mcu"] = bd.Compound(children=[esp_base, esp_pcb, esp_rf], label="esp32_s3_mcu")

    jet_base = bd.Pos(232.0, 54.0, CHASSIS_BOTTOM_Z1 + 20.0) * rounded_plate(120.0, 92.0, 40.0, 6.0)
    jet_sink = bd.Pos(232.0, 54.0, CHASSIS_BOTTOM_Z1 + 46.0) * rounded_plate(104.0, 80.0, 12.0, 4.0)
    jet_fan = bd.Pos(232.0, 54.0, CHASSIS_BOTTOM_Z1 + 53.0) * bd.Cylinder(30.0, 4.0)
    paint_part(jet_base, "#262B33", "jetson_enclosure")
    paint_part(jet_sink, "#383D47", "jetson_heatsink", metallic=True, roughness=0.35)
    paint_part(jet_fan, "#1A1A1F", "jetson_fan")
    parts["jetson_orin_sbc"] = bd.Compound(children=[jet_base, jet_sink, jet_fan], label="jetson_orin_sbc")

    pdu_base = bd.Pos(-232.0, -52.0, CHASSIS_BOTTOM_Z1 + 4.0) * rounded_plate(116.0, 112.0, 8.0, 6.0)
    pdu_contactor = bd.Pos(-256.0, -22.0, CHASSIS_BOTTOM_Z1 + 26.0) * bd.Box(44.0, 38.0, 36.0)
    pdu_fuse = bd.Pos(-202.0, -20.0, CHASSIS_BOTTOM_Z1 + 22.0) * bd.Box(38.0, 24.0, 28.0)
    pdu_dcdc = bd.Pos(-232.0, -74.0, CHASSIS_BOTTOM_Z1 + 20.0) * rounded_plate(96.0, 48.0, 24.0, 4.0)
    paint_part(pdu_base, "#333D47", "pdu_plate")
    paint_part(pdu_contactor, "#2E388C", "safety_contactor_48v")
    paint_part(pdu_fuse, "#CC3326", "main_fuse_holder")
    paint_part(pdu_dcdc, "#4785BF", "dcdc_converters_24v_12v_5v")
    parts["pdu_module"] = bd.Compound(
        children=[pdu_base, pdu_contactor, pdu_fuse, pdu_dcdc], label="pdu_module"
    )

    imu = bd.Pos(68.0, 0, CHASSIS_TOP_Z1 + 7.0) * rounded_plate(38.0, 32.0, 14.0, 4.0)
    paint_part(imu, "#D9521F", "imu_sensor")
    parts["imu_sensor"] = imu

    return parts


@memo
def build_sensors_and_safety() -> dict[str, bd.Shape]:
    """Front 2D LiDAR + pedestal, RealSense D455 depth camera, E-stop, and charging contacts."""
    ped_h = LIDAR_MOUNT_Z - ROBOT_BODY_HEIGHT  # 90 mm
    riser = bd.Pos(LIDAR_POS_X, 0, ROBOT_BODY_HEIGHT + ped_h / 2.0) * bd.Cone(
        39.0, 38.0, ped_h
    )
    lidar_base = bd.Pos(LIDAR_POS_X, 0, LIDAR_MOUNT_Z + 9.0) * bd.Cylinder(LIDAR_D / 2.0, 18.0)
    lidar_win = bd.Pos(LIDAR_POS_X, 0, LIDAR_MOUNT_Z + 29.0) * bd.Cylinder(LIDAR_D / 2.0 - 2.0, 22.0)
    lidar_cap = bd.Pos(LIDAR_POS_X, 0, LIDAR_MOUNT_Z + 50.0) * bd.Cylinder(LIDAR_D / 2.0, 20.0)
    paint_part(riser, "#474D57", "lidar_riser_tower", metallic=True, roughness=0.35)
    paint_part(lidar_base, "#1F1F24", "lidar_stator")
    paint_part(lidar_win, "#0D1426", "lidar_optical_window")
    paint_part(lidar_cap, "#24262E", "lidar_top_cap")
    lidar_unit = bd.Compound(
        children=[riser, lidar_base, lidar_win, lidar_cap], label="lidar_2d_unit"
    )

    cam_body = bd.Pos(CAMERA_POS_X, 0, CAMERA_MOUNT_Z) * rounded_plate(
        D455_L, D455_W, D455_H, 6.0
    )
    cam_face = bd.Pos(CAMERA_POS_X + D455_L / 2.0 - 1.0, 0, CAMERA_MOUNT_Z) * rounded_plate(
        4.0, D455_W - 8.0, D455_H - 6.0, 4.0
    )
    cam_bracket = bd.Pos(CAMERA_POS_X - 6.0, 0, CAMERA_MOUNT_Z - D455_H / 2.0 - 7.0) * rounded_plate(
        24.0, 68.0, 15.0, 3.0
    )
    paint_part(cam_body, "#B8BFC7", "realsense_d455_housing", metallic=True, roughness=0.28)
    paint_part(cam_face, "#14141A", "realsense_d455_glass")
    paint_part(cam_bracket, "#59616B", "realsense_d455_bracket", metallic=True, roughness=0.4)
    camera_unit = bd.Compound(
        children=[cam_body, cam_face, cam_bracket], label="realsense_d455"
    )

    es_collar = bd.Pos(ESTOP_POS_X, ESTOP_POS_Y, ROBOT_BODY_HEIGHT + 9.0) * bd.Cone(28.0, 26.0, 18.0)
    es_stem = bd.Pos(ESTOP_POS_X, ESTOP_POS_Y, ROBOT_BODY_HEIGHT + 22.0) * bd.Cylinder(12.0, 8.0)
    es_head = bd.Pos(ESTOP_POS_X, ESTOP_POS_Y, ROBOT_BODY_HEIGHT + 35.0) * bd.Cone(19.0, 23.0, 18.0)
    paint_part(es_collar, "#F2CC0D", "estop_yellow_collar")
    paint_part(es_stem, "#BFC2C7", "estop_stem", metallic=True, roughness=0.3)
    paint_part(es_head, "#E01414", "estop_red_mushroom")
    estop_unit = bd.Compound(children=[es_collar, es_stem, es_head], label="emergency_stop")

    cx = ROBOT_LENGTH / 2.0 + 2.0
    ch_block = bd.Pos(cx - 4.0, 0, CHARGING_CONTACT_Z) * rounded_plate(22.0, 136.0, 36.0, 4.0)
    paint_part(ch_block, "#26262E", "charging_insulator_block")
    ch_pads = paint_solids(
        [
            bd.Pos(cx + 5.0, cy, CHARGING_CONTACT_Z) * bd.Box(6.0, 36.0, 22.0)
            for cy in (-38.0, 38.0)
        ],
        "#E0A62E",
        "charging_copper_pad",
        metallic=True,
        roughness=0.2,
    )
    charging_unit = bd.Compound(children=[ch_block, *ch_pads], label="charging_contacts")

    return {
        "lidar_2d_unit": lidar_unit,
        "realsense_d455": camera_unit,
        "emergency_stop": estop_unit,
        "charging_contacts": charging_unit,
    }


@memo
def build_lift_fixed_subsystems() -> tuple[bd.Compound, bd.Compound, bd.Compound]:
    """Fixed lift bridge frame, 4 vertical linear guide rails, and central ball-screw actuator."""
    low_outer = rounded_plate(2 * LIFT_RAIL_X + 56.0, 2 * LIFT_RAIL_Y + 56.0, 10.0, 12.0)
    low_inner = rounded_plate(2 * LIFT_RAIL_X - 50.0, 2 * LIFT_RAIL_Y - 50.0, 14.0, 8.0)
    low_bridge = (
        bd.Pos(LIFT_CENTER_X, 0, LIFT_FIXED_BASE_Z + 5.0)
        * ((low_outer - low_inner) + bd.Box(70.0, 2 * LIFT_RAIL_Y + 56.0, 10.0))
    ).solids()[0]
    up_outer = rounded_plate(2 * LIFT_RAIL_X + 60.0, 2 * LIFT_RAIL_Y + 60.0, 48.0, 14.0)
    up_inner = rounded_plate(2 * LIFT_RAIL_X - 36.0, 2 * LIFT_RAIL_Y - 36.0, 54.0, 10.0)
    up_collar = (bd.Pos(LIFT_CENTER_X, 0, CHASSIS_TOP_Z1 + 24.0) * (up_outer - up_inner)).solids()[0]

    ped_h = (LIFT_RAIL_BOTTOM_Z + 10.0) - CHASSIS_BOTTOM_Z1  # 24 mm
    peds = [
        bd.Pos(LIFT_CENTER_X + dx * LIFT_RAIL_X, dy * LIFT_RAIL_Y, CHASSIS_BOTTOM_Z1 + ped_h / 2.0)
        * bd.Cylinder(26.0, ped_h)
        for dx in (-1, 1)
        for dy in (-1, 1)
    ]
    frame_solids = paint_solids(
        [low_bridge, up_collar, *peds],
        "#6B737A",
        "lift_fixed_part",
        metallic=True,
        roughness=0.4,
    )
    fixed_frame = bd.Compound(children=frame_solids, label="lift_fixed_frame")

    rail_bot = CHASSIS_BOTTOM_Z1  # 41 mm
    rail_top = 326.0  # 326 mm (lift_lowered_deck_z - lift_deck_thickness + 8)
    rail_len = rail_top - rail_bot  # 285 mm
    shafts = []
    stops = []
    for dx in (-1, 1):
        for dy in (-1, 1):
            rx, ry = LIFT_CENTER_X + dx * LIFT_RAIL_X, dy * LIFT_RAIL_Y
            shafts.append(
                bd.Pos(rx, ry, rail_bot + rail_len / 2.0) * bd.Cylinder(LIFT_RAIL_D / 2.0, rail_len)
            )
            stops.append(bd.Pos(rx, ry, rail_top - 5.0) * bd.Cylinder(LIFT_RAIL_D / 2.0 + 5.0, 8.0))
    painted_shafts = paint_solids(shafts, "#C7CCD6", "guide_shaft", metallic=True, roughness=0.18)
    painted_stops = paint_solids(stops, "#D94026", "guide_limit_stop")
    guide_rails = bd.Compound(children=[*painted_shafts, *painted_stops], label="lift_guide_rails")

    act_base_z = LIFT_FIXED_BASE_Z + 10.0  # 190 mm
    act_gearbox = bd.Pos(LIFT_CENTER_X, 0, act_base_z + 24.0) * rounded_plate(68.0, 68.0, 48.0, 8.0)
    act_motor = (
        bd.Pos(LIFT_CENTER_X + 58.0, 0, act_base_z + 24.0)
        * bd.Rotation(0, 90, 0)
        * bd.Cylinder(26.0, 76.0)
    )
    barrel_bot = act_base_z + 40.0  # 230 mm
    barrel_top = 326.0  # 326 mm
    barrel_len = barrel_top - barrel_bot
    act_barrel = bd.Pos(LIFT_CENTER_X, 0, barrel_bot + barrel_len / 2.0) * bd.Cylinder(24.0, barrel_len)
    paint_part(act_gearbox, "#333D4D", "actuator_gearbox")
    paint_part(act_motor, "#26598C", "actuator_24v_motor")
    paint_part(act_barrel, "#808794", "actuator_outer_barrel", metallic=True, roughness=0.3)
    actuator = bd.Compound(children=[act_gearbox, act_motor, act_barrel], label="lift_actuator")

    return fixed_frame, guide_rails, actuator


@memo
def build_lift_moving_platform() -> bd.Compound:
    """Moving lift platform deck, anti-slip pad, 4 linear bearing sleeves, ram, and scissor bars."""
    deck_bottom_z = LIFT_LOWERED_DECK_Z - LIFT_DECK_T  # 318 mm
    deck_zc = deck_bottom_z + LIFT_DECK_T / 2.0        # 324 mm

    deck_plate = bd.Pos(LIFT_CENTER_X, 0, deck_zc) * rounded_plate(
        PAYLOAD_DECK_L, PAYLOAD_DECK_W, LIFT_DECK_T, 24.0
    )
    brackets = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            brackets.append(
                bd.Pos(
                    LIFT_CENTER_X + sx * (PAYLOAD_BOX_L / 2.0 - 12.0),
                    sy * (PAYLOAD_BOX_W / 2.0 + 8.0),
                    LIFT_LOWERED_DECK_Z + 8.0,
                )
                * bd.Box(44.0, 14.0, 16.0)
            )
            brackets.append(
                bd.Pos(
                    LIFT_CENTER_X + sx * (PAYLOAD_BOX_L / 2.0 + 8.0),
                    sy * (PAYLOAD_BOX_W / 2.0 - 12.0),
                    LIFT_LOWERED_DECK_Z + 8.0,
                )
                * bd.Box(14.0, 44.0, 16.0)
            )
    deck_solids = paint_solids([deck_plate, *brackets], "#EB9414", "lift_platform_deck")

    pad = bd.Pos(LIFT_CENTER_X, 0, LIFT_LOWERED_DECK_Z + 1.0) * rounded_plate(
        PAYLOAD_DECK_L - 36.0, PAYLOAD_DECK_W - 36.0, 2.0, 16.0
    )
    paint_part(pad, "#383D42", "anti_slip_pad")

    sleeve_len = 282.0
    sleeves = []
    for dx in (-1, 1):
        for dy in (-1, 1):
            rx, ry = LIFT_CENTER_X + dx * LIFT_RAIL_X, dy * LIFT_RAIL_Y
            sleeves.append(
                bd.Pos(rx, ry, deck_bottom_z - sleeve_len / 2.0)
                * bd.Cylinder(LIFT_SLEEVE_D / 2.0, sleeve_len)
                + bd.Pos(rx, ry, deck_bottom_z - 12.0)
                * bd.Cylinder((LIFT_SLEEVE_D + 18.0) / 2.0, 24.0)
            )
    painted_sleeves = paint_solids(sleeves, "#E0851A", "linear_bearing_sleeve", metallic=True, roughness=0.35)

    ram_len = 282.0
    ram = bd.Pos(LIFT_CENTER_X, 0, deck_bottom_z - ram_len / 2.0) * bd.Cylinder(15.0, ram_len)
    ram_flange = bd.Pos(LIFT_CENTER_X, 0, deck_bottom_z - 5.0) * bd.Cylinder(32.0, 10.0)
    paint_part(ram, "#D1D6E0", "actuator_moving_ram", metallic=True, roughness=0.15)
    paint_part(ram_flange, "#5A616B", "ram_upper_flange", metallic=True, roughness=0.3)

    scissor_zc = (CHASSIS_TOP_Z1 + 12.0 + deck_bottom_z - 8.0) / 2.0
    bars = []
    clevis_brackets = []
    for side in (-1, 1):
        bars.append(
            bd.Pos(LIFT_CENTER_X, side * (LIFT_RAIL_Y - 38.0), scissor_zc)
            * bd.Rotation(0, -8.85, 0)
            * bd.Box(375.0, 14.0, 16.0)
        )
        bars.append(
            bd.Pos(LIFT_CENTER_X, side * (LIFT_RAIL_Y - 54.0), scissor_zc)
            * bd.Rotation(0, 8.85, 0)
            * bd.Box(375.0, 14.0, 16.0)
        )
        for sx in (-135.0, 135.0):
            clevis_brackets.append(
                bd.Pos(LIFT_CENTER_X + sx, side * (LIFT_RAIL_Y - 46.0), deck_bottom_z - 6.0)
                * bd.Box(28.0, 34.0, 12.0)
            )
    painted_bars = paint_solids(bars, "#D17A14", "scissor_bar", metallic=True, roughness=0.38)
    painted_clevis = paint_solids(clevis_brackets, "#B8660D", "scissor_upper_clevis", metallic=True, roughness=0.4)

    return bd.Compound(
        children=[*deck_solids, pad, *painted_sleeves, ram, ram_flange, *painted_bars, *painted_clevis],
        label="lift_moving_assembly",
    )


@memo
def build_payload_20kg() -> bd.Compound:
    """20 kg industrial tray with cantilevered rack wings and 6 steel cylinder weights."""
    pz0 = LIFT_LOWERED_DECK_Z + 2.0  # 332 mm
    pzc = pz0 + PAYLOAD_BOX_H / 2.0  # 357 mm

    outer = rounded_plate(PAYLOAD_BOX_L, PAYLOAD_BOX_W, PAYLOAD_BOX_H, 12.0)
    inner = bd.Pos(0, 0, 8.0) * rounded_plate(
        PAYLOAD_BOX_L - 24.0, PAYLOAD_BOX_W - 24.0, PAYLOAD_BOX_H, 8.0
    )
    tray_shell = bd.Pos(LIFT_CENTER_X, 0, pzc) * (outer - inner)

    wing_zc = pz0 + WING_Z_OFFSET + WING_THICK / 2.0
    wings = []
    for sy in (-1, 1):
        wy = sy * (PAYLOAD_BOX_W / 2.0 + WING_SPAN_Y / 2.0 - 2.0)
        wings.append(
            bd.Pos(LIFT_CENTER_X, wy, wing_zc)
            * rounded_plate(WING_LEN_X, WING_SPAN_Y + 4.0, WING_THICK, 5.0)
        )
        for rx in (-165.0, -82.0, 0.0, 82.0, 165.0):
            wings.append(
                bd.Pos(
                    LIFT_CENTER_X + rx,
                    sy * (PAYLOAD_BOX_W / 2.0 + 48.0),
                    wing_zc + WING_THICK / 2.0 + 5.0,
                )
                * bd.Box(10.0, 96.0, 10.0)
            )
    tray_solids = paint_solids([tray_shell + wings], "#B88552", "payload_tray_with_wings")

    weights = [
        bd.Pos(LIFT_CENTER_X + px, py, pzc - 4.0) * bd.Cylinder(38.0, PAYLOAD_BOX_H - 14.0)
        for px in (-150.0, 0.0, 150.0)
        for py in (-95.0, 95.0)
    ]
    painted_weights = paint_solids(weights, "#9EA6B3", "steel_weight", metallic=True, roughness=0.25)

    return bd.Compound(children=[*tray_solids, *painted_weights], label="payload_20kg")


@memo
def build_body_enclosure_parts() -> dict[str, bd.Shape]:
    """Main curved body shell, left/right detachable service panels, and perimeter bumper."""
    shell_h = ROBOT_BODY_HEIGHT - GROUND_CLEARANCE  # 265 mm
    shell_zc = GROUND_CLEARANCE + shell_h / 2.0    # 167.5 mm

    outer = bd.Pos(0, 0, shell_zc) * rounded_plate(
        ROBOT_LENGTH, ROBOT_WIDTH, shell_h, BODY_CORNER_RADIUS
    )
    cavity = bd.Pos(0, 0, shell_zc - 6.0) * rounded_plate(
        ROBOT_LENGTH - 2 * SHELL_WALL_T,
        ROBOT_WIDTH - 2 * SHELL_WALL_T,
        shell_h,
        BODY_CORNER_RADIUS - SHELL_WALL_T,
    )
    lift_open = bd.Pos(LIFT_CENTER_X, 0, ROBOT_BODY_HEIGHT) * rounded_plate(
        2 * LIFT_RAIL_X + 62.0, 2 * LIFT_RAIL_Y + 62.0, 30.0, 14.0
    )
    cam_win = bd.Pos(ROBOT_LENGTH / 2.0 - 10.0, 0, CAMERA_MOUNT_Z) * rounded_plate(
        36.0, D455_W + 12.0, D455_H + 6.0, 5.0
    )
    chg_win = bd.Pos(ROBOT_LENGTH / 2.0 - 6.0, 0, CHARGING_CONTACT_Z) * rounded_plate(
        24.0, 144.0, 44.0, 4.0
    )
    wheel_arches = [
        bd.Pos(0, sy * (DRIVE_WHEEL_TRACK / 2.0), DRIVE_AXLE_Z)
        * bd.Rotation(90, 0, 0)
        * bd.Cylinder(DRIVE_WHEEL_R + 14.0, DRIVE_WHEEL_W + 36.0)
        for sy in (-1, 1)
    ]
    main_shell = (outer - [cavity, lift_open, cam_win, chg_win, *wheel_arches]).solids()[0]
    paint_part(main_shell, "#1A427A", "body_shell_main")

    panel_z0 = GROUND_CLEARANCE + BUMPER_H + 6.0  # 87 mm
    panel_h = ROBOT_BODY_HEIGHT - panel_z0 - 10.0  # 203 mm
    panel_zc = panel_z0 + panel_h / 2.0

    side_panels = {}
    for sy, tag in ((1, "left"), (-1, "right")):
        pnl = bd.Pos(0, sy * (ROBOT_WIDTH / 2.0 + 0.5), panel_zc) * bd.Box(
            ROBOT_LENGTH - 2 * BODY_CORNER_RADIUS + 16.0, 3.0, panel_h
        )
        led = bd.Pos(0, sy * (ROBOT_WIDTH / 2.0 + 2.2), ROBOT_BODY_HEIGHT - 20.0) * bd.Box(
            ROBOT_LENGTH - 2 * BODY_CORNER_RADIUS - 40.0, 2.0, 6.0
        )
        paint_part(pnl, "#1F579E", f"skin_panel_{tag}_body")
        paint_part(led, "#00D9F2", f"status_led_{tag}")
        side_panels[f"skin_panel_{tag}"] = bd.Compound(
            children=[pnl, led], label=f"skin_panel_{tag}"
        )

    bz = GROUND_CLEARANCE + BUMPER_H / 2.0 + 2.0
    b_outer = bd.Pos(0, 0, bz) * rounded_plate(
        ROBOT_LENGTH + 2 * BUMPER_PROTRUSION,
        ROBOT_WIDTH + 2 * BUMPER_PROTRUSION,
        BUMPER_H,
        BODY_CORNER_RADIUS + BUMPER_PROTRUSION,
    )
    b_inner = bd.Pos(0, 0, bz) * rounded_plate(
        ROBOT_LENGTH - 6.0, ROBOT_WIDTH - 6.0, BUMPER_H + 6.0, BODY_CORNER_RADIUS - 3.0
    )
    b_cuts = [
        bd.Pos(ROBOT_LENGTH / 2.0 + BUMPER_PROTRUSION / 2.0, 0, CHARGING_CONTACT_Z)
        * bd.Box(BUMPER_PROTRUSION * 4.0, 144.0, 44.0)
    ]
    for sy in (-1, 1):
        b_cuts.append(
            bd.Pos(0, sy * (ROBOT_WIDTH / 2.0 + BUMPER_PROTRUSION / 2.0), bz - 10.0)
            * bd.Box(DRIVE_WHEEL_D + 30.0, BUMPER_PROTRUSION * 4.0, BUMPER_H)
        )
    bumper_solids = paint_solids([b_outer - [b_inner, *b_cuts]], "#1F2126", "bumper_segment")
    bumper = bd.Compound(children=bumper_solids, label="perimeter_bumper_ring")

    return {
        "body_shell_main": main_shell,
        "skin_panel_left": side_panels["skin_panel_left"],
        "skin_panel_right": side_panels["skin_panel_right"],
        "perimeter_bumper_ring": bumper,
    }


# ============================================================================
# 4. EXPLODED ARCHITECTURAL OFFSETS (non-overlapping vertical + radial tiers)
# ============================================================================
def get_explode_offsets() -> dict[str, tuple[float, float, float]]:
    """Return (dx, dy, dz) explosion vectors so every subsystem is clearly visible."""
    return {
        # Tier 0: Ground Running Gear (below & outward from chassis)
        "caster_fl": (260.0, 240.0, -180.0),
        "caster_fr": (260.0, -240.0, -180.0),
        "caster_rl": (-260.0, 240.0, -180.0),
        "caster_rr": (-260.0, -240.0, -180.0),
        "drive_module_left": (0.0, 460.0, 0.0),
        "drive_module_right": (0.0, -460.0, 0.0),
        # Tier 1: Structural Chassis Core (fixed reference datum at Z = 35..250 mm)
        "chassis_core": (0.0, 0.0, 0.0),
        # Tier 2: Internal Power, Compute & Drive Electronics (radially fanned out)
        "charging_contacts": (380.0, 0.0, 0.0),
        "battery_pack_48v": (-460.0, 0.0, 80.0),
        "smart_bms": (-440.0, 280.0, 120.0),
        "pdu_module": (-440.0, -280.0, 120.0),
        "motor_driver_left": (220.0, 380.0, 140.0),
        "motor_driver_right": (220.0, -380.0, 140.0),
        "jetson_orin_sbc": (420.0, 220.0, 120.0),
        "esp32_s3_mcu": (420.0, -220.0, 120.0),
        # Tier 3: Upper Structural Deck & Fixed Lift Mechanism
        "chassis_top_deck": (0.0, 0.0, 220.0),
        "imu_sensor": (120.0, 0.0, 300.0),
        "lift_fixed_frame": (-260.0, 0.0, 580.0),
        "lift_guide_rails": (0.0, 0.0, 580.0),
        "lift_actuator": (240.0, 0.0, 580.0),
        # Tier 4: Outer Body Shell, Bumper, Service Panels & Navigation Sensors
        "perimeter_bumper_ring": (0.0, 0.0, 960.0),
        "body_shell_main": (0.0, 0.0, 1120.0),
        "skin_panel_left": (0.0, 320.0, 1120.0),
        "skin_panel_right": (0.0, -320.0, 1120.0),
        "realsense_d455": (320.0, 0.0, 1120.0),
        "lidar_2d_unit": (180.0, 0.0, 1260.0),
        "emergency_stop": (-180.0, 180.0, 1260.0),
        # Tier 5: Moving Lift Platform & 20 kg Industrial Rack Payload Tray
        "lift_moving_assembly": (0.0, 0.0, 1580.0),
        "payload_20kg": (0.0, 0.0, 1920.0),
    }


def collect_subsystems() -> dict[str, bd.Shape]:
    """Build and return all 29 labeled subsystems at their assembled coordinates."""
    battery, bms = build_power_system()
    elec = build_electronics()
    sensors = build_sensors_and_safety()
    lift_fixed_frame, lift_guide_rails, lift_actuator = build_lift_fixed_subsystems()
    enclosure = build_body_enclosure_parts()

    return {
        "chassis_core": build_chassis_core(),
        "chassis_top_deck": build_chassis_top_deck(),
        "body_shell_main": enclosure["body_shell_main"],
        "skin_panel_left": enclosure["skin_panel_left"],
        "skin_panel_right": enclosure["skin_panel_right"],
        "perimeter_bumper_ring": enclosure["perimeter_bumper_ring"],
        "drive_module_left": build_drive_unit(1),
        "drive_module_right": build_drive_unit(-1),
        "caster_fl": build_caster_unit(CASTER_OFFSET_X, CASTER_OFFSET_Y, "caster_fl"),
        "caster_fr": build_caster_unit(CASTER_OFFSET_X, -CASTER_OFFSET_Y, "caster_fr"),
        "caster_rl": build_caster_unit(-CASTER_OFFSET_X, CASTER_OFFSET_Y, "caster_rl"),
        "caster_rr": build_caster_unit(-CASTER_OFFSET_X, -CASTER_OFFSET_Y, "caster_rr"),
        "battery_pack_48v": battery,
        "smart_bms": bms,
        "motor_driver_left": elec["motor_driver_left"],
        "motor_driver_right": elec["motor_driver_right"],
        "esp32_s3_mcu": elec["esp32_s3_mcu"],
        "jetson_orin_sbc": elec["jetson_orin_sbc"],
        "pdu_module": elec["pdu_module"],
        "imu_sensor": elec["imu_sensor"],
        "lidar_2d_unit": sensors["lidar_2d_unit"],
        "realsense_d455": sensors["realsense_d455"],
        "emergency_stop": sensors["emergency_stop"],
        "charging_contacts": sensors["charging_contacts"],
        "lift_fixed_frame": lift_fixed_frame,
        "lift_guide_rails": lift_guide_rails,
        "lift_actuator": lift_actuator,
        "lift_moving_assembly": build_lift_moving_platform(),
        "payload_20kg": build_payload_20kg(),
    }


def move_part_rigidly(part: bd.Shape, dx: float, dy: float, dz: float, name: str) -> bd.Shape:
    """Move a solid or compound cleanly by (dx, dy, dz) while preserving child labels/colors."""
    if not (dx or dy or dz):
        part.label = name
        return part
    shift = bd.Pos(dx, dy, dz)
    if isinstance(part, bd.Compound) and part.children:
        moved_children = [
            move_part_rigidly(ch, dx, dy, dz, ch.label) for ch in part.children
        ]
        comp = bd.Compound(children=moved_children, label=name)
        comp.color = part.color
        comp.material = part.material
        return comp
    moved = shift * part
    moved.label = name
    moved.color = part.color
    moved.material = part.material
    return moved


# ============================================================================
# 5. CAD MODELS (@model)
# ============================================================================
@model
def factory_delivery_amr_exploded():
    """Static 3D visual exploded breakdown (Bản phân tách trực quan) of the AMR."""
    subsystems = collect_subsystems()
    offsets = get_explode_offsets()

    placed_children = []
    for name, part in subsystems.items():
        dx, dy, dz = offsets[name]
        placed_children.append(move_part_rigidly(part, dx, dy, dz, name))

    assembly = bd.Compound(children=placed_children, label="factory_delivery_amr_exploded")
    bbox = assembly.bounding_box()
    assert bbox.size.X > 1400.0 and bbox.size.Y > 1400.0 and bbox.size.Z > 2000.0
    return assembly


@model
def factory_delivery_amr():
    """Assembled Factory Delivery AMR with sequential GLB explode-and-reassemble animation."""
    subsystems = collect_subsystems()
    children = []
    for name, part in subsystems.items():
        part.label = name
        children.append(part)

    assembly = bd.Compound(children=children, label="factory_delivery_amr")

    bbox = assembly.bounding_box()
    assert abs(bbox.size.X - (ROBOT_LENGTH + 2 * BUMPER_PROTRUSION)) < 1.0
    assert abs(bbox.size.Y - (ROBOT_WIDTH + 2 * BUMPER_PROTRUSION)) < 1.0
    assert abs(bbox.min.Z - 0.0) < 1.0

    # Sequential outside-to-inside disassembly & reverse reassembly GLB animation
    offsets = get_explode_offsets()
    stage_schedule = [
        # Stage 1: Payload Tray rises to clear the deck
        ("payload_20kg", 0.2, 1.4),
        # Stage 2: Moving Lift Platform rises
        ("lift_moving_assembly", 0.6, 1.8),
        # Stage 3: Top sensors and emergency components
        ("lidar_2d_unit", 1.0, 2.2),
        ("realsense_d455", 1.0, 2.2),
        ("emergency_stop", 1.0, 2.2),
        ("charging_contacts", 1.0, 2.2),
        # Stage 4: Exterior body panels & bumper ring
        ("skin_panel_left", 1.4, 2.6),
        ("skin_panel_right", 1.4, 2.6),
        ("body_shell_main", 1.7, 2.9),
        ("perimeter_bumper_ring", 2.0, 3.2),
        # Stage 5: Fixed lift frame, rails, actuator & top deck
        ("lift_fixed_frame", 2.3, 3.5),
        ("lift_guide_rails", 2.3, 3.5),
        ("lift_actuator", 2.3, 3.5),
        ("imu_sensor", 2.5, 3.7),
        ("chassis_top_deck", 2.7, 3.9),
        # Stage 6: Computing & power electronics radially fanning out
        ("motor_driver_left", 3.0, 4.2),
        ("motor_driver_right", 3.0, 4.2),
        ("esp32_s3_mcu", 3.0, 4.2),
        ("jetson_orin_sbc", 3.0, 4.2),
        ("pdu_module", 3.0, 4.2),
        ("battery_pack_48v", 3.3, 4.5),
        ("smart_bms", 3.3, 4.5),
        # Stage 7: Drive modules sliding out laterally
        ("drive_module_left", 3.6, 4.8),
        ("drive_module_right", 3.6, 4.8),
        # Stage 8: 4 Swivel casters dropping down & outward
        ("caster_fl", 3.9, 5.1),
        ("caster_fr", 3.9, 5.1),
        ("caster_rl", 3.9, 5.1),
        ("caster_rr", 3.9, 5.1),
    ]

    duration = 12.0
    anim = animation(assembly, "exploded_breakdown", duration=duration)
    for part_name, t_start, t_end in stage_schedule:
        dx, dy, dz = offsets[part_name]
        anim.channel(
            part_name,
            path="translation",
            keys=[
                (0.0, (0.0, 0.0, 0.0)),
                (t_start, (0.0, 0.0, 0.0)),
                (t_end, (dx, dy, dz)),
                (duration - t_end, (dx, dy, dz)),
                (duration - t_start, (0.0, 0.0, 0.0)),
                (duration, (0.0, 0.0, 0.0)),
            ],
            interpolation="CUBICSPLINE",
        )

    return assembly


if __name__ == "__main__":
    factory_delivery_amr_exploded()
    factory_delivery_amr()
