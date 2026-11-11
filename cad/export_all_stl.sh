#!/usr/bin/env bash

set -e

# Đảm bảo luôn thực thi tại đúng thư mục chứa script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Tự động tìm OpenSCAD (ưu tiên openscad-nightly nếu có, sau đó là openscad thông thường)
if command -v openscad-nightly >/dev/null 2>&1; then
    OPENSCAD="$(command -v openscad-nightly)"
elif [ -x "/snap/bin/openscad-nightly" ]; then
    OPENSCAD="/snap/bin/openscad-nightly"
elif command -v openscad >/dev/null 2>&1; then
    OPENSCAD="$(command -v openscad)"
elif [ -x "/usr/bin/openscad" ]; then
    OPENSCAD="/usr/bin/openscad"
else
    echo "ERROR: Không tìm thấy OpenSCAD (openscad hoặc openscad-nightly) trên hệ thống!"
    echo "Vui lòng cài đặt: sudo apt install openscad (hoặc sudo snap install openscad-nightly)"
    exit 1
fi

mkdir -p meshes


export_stl() {
    input_file="$1"
    output_file="$2"

    if [ ! -f "$input_file" ]; then
        echo "ERROR: Khong tim thay file:"
        echo "$input_file"
        exit 1
    fi

    echo
    echo "============================================"
    echo "Dang xuat Binary STL"
    echo "Input : $input_file"
    echo "Output: meshes/$output_file"
    echo "============================================"

    "$OPENSCAD" \
        --export-format binstl \
        -o "meshes/$output_file" \
        "$input_file"

    if [ ! -s "meshes/$output_file" ]; then
        echo "ERROR: STL rong hoac khong duoc tao:"
        echo "meshes/$output_file"
        exit 1
    fi

    if [ "$(head -c 5 "meshes/$output_file")" = "solid" ]; then
        echo "ERROR: File vua xuat van la ASCII STL:"
        echo "meshes/$output_file"
        exit 1
    fi

    echo "[OK] Da xuat Binary STL:"
    echo "meshes/$output_file"
}


# ============================================================================
# FIXED ROBOT COMPONENTS
# ============================================================================

export_stl \
    export_chassis.scad \
    chassis_visual.stl

export_stl \
    export_body_shell.scad \
    body_shell_visual.stl

export_stl \
    export_casters.scad \
    casters_visual.stl

export_stl \
    export_battery_system.scad \
    battery_system_visual.stl

export_stl \
    export_electronics.scad \
    electronics_visual.stl

export_stl \
    export_fixed_drive_hardware.scad \
    fixed_drive_hardware_visual.stl

export_stl \
    export_safety_components.scad \
    safety_components_visual.stl


# ============================================================================
# MAIN DRIVE WHEELS
# ============================================================================

export_stl \
    export_left_wheel.scad \
    left_wheel_visual.stl

export_stl \
    export_right_wheel.scad \
    right_wheel_visual.stl


# ============================================================================
# LIFT SYSTEM
# ============================================================================

export_stl \
    export_lift_fixed.scad \
    lift_fixed_visual.stl

export_stl \
    export_lift_platform.scad \
    lift_platform_visual.stl

export_stl \
    export_lift_ram.scad \
    lift_ram_visual.stl

export_stl \
    export_payload.scad \
    payload_visual.stl


# ============================================================================
# SENSORS
# ============================================================================

export_stl \
    export_lidar.scad \
    lidar_visual.stl

export_stl \
    export_camera.scad \
    camera_visual.stl


# ============================================================================
# HIGH-FIDELITY SUB-COMPONENT MESHES FOR GAZEBO & RVIZ2 MULTI-COLOR ACCURACY
# ============================================================================

# Body Shell detailed visual elements
export_stl export_body_shell_main.scad body_shell_main_visual.stl
export_stl export_body_skin_panels.scad body_skin_panels_visual.stl
export_stl export_body_status_lights.scad body_status_lights_visual.stl
export_stl export_body_bumper.scad body_bumper_visual.stl
export_stl export_body_hazard_stripes.scad body_hazard_stripes_visual.stl

# Wheel tires vs silver spoked aluminum hubs
export_stl export_left_wheel_tire.scad left_wheel_tire_visual.stl
export_stl export_left_wheel_hub.scad left_wheel_hub_visual.stl
export_stl export_right_wheel_tire.scad right_wheel_tire_visual.stl
export_stl export_right_wheel_hub.scad right_wheel_hub_visual.stl

# Lift platform & anti-slip deck pad
export_stl export_lift_platform_base.scad lift_platform_base_visual.stl
export_stl export_lift_deck_pad.scad lift_deck_pad_visual.stl

# 20kg Payload tote tray with wings vs machined steel weights
export_stl export_payload_tray.scad payload_tray_visual.stl
export_stl export_payload_weights.scad payload_weights_visual.stl

# Safety components: red e-stop, yellow collar, black charging block, copper pads
export_stl export_estop_button.scad estop_button_visual.stl
export_stl export_estop_collar.scad estop_collar_visual.stl
export_stl export_charging_contacts_housing.scad charging_contacts_housing_visual.stl
export_stl export_charging_contacts_plates.scad charging_contacts_plates_visual.stl

# Sensors: aluminum pedestal vs 2D LiDAR scanner unit, camera housing vs lens face
export_stl export_lidar_pedestal.scad lidar_pedestal_visual.stl
export_stl export_lidar_unit.scad lidar_unit_visual.stl
export_stl export_camera_body.scad camera_body_visual.stl
export_stl export_camera_face.scad camera_face_visual.stl

# Internal electronics & battery sub-components
export_stl export_battery_pack.scad battery_pack_visual.stl
export_stl export_bms.scad bms_visual.stl
export_stl export_pdu.scad pdu_visual.stl
export_stl export_motor_drivers.scad motor_drivers_visual.stl
export_stl export_esp32_s3.scad esp32_s3_visual.stl
export_stl export_jetson_sbc.scad jetson_sbc_visual.stl
export_stl export_imu.scad imu_visual.stl

# ============================================================================
# SEPARATE GAZEBO WORLD MODELS
# ============================================================================

export_stl \
    export_charging_station.scad \
    charging_station_visual.stl

export_stl \
    export_smart_storage_rack.scad \
    smart_storage_rack_visual.stl


# ============================================================================
# FINAL REPORT
# ============================================================================

echo
echo "============================================"
echo "DA XUAT XONG TAT CA BINARY STL"
echo "Thu muc ket qua:"
echo "$(pwd)/meshes"
echo "============================================"
echo

ls -lh meshes/*.stl

echo
echo "Tong so file STL:"

find meshes \
    -maxdepth 1 \
    -type f \
    -name "*.stl" |
    wc -l