// ============================================================================
// FILE: export_electronics.scad
// OUTPUT: electronics_visual.stl
// URDF LINK: electronics_link
// ============================================================================

export_only = true;

include <main.scad>

union() {
    // Power distribution and converters
    power_distribution();

    // Motor controllers
    motor_driver(1);
    motor_driver(-1);

    // Low-level controller
    esp32_s3_controller();

    // ROS 2 computer
    jetson_or_sbc();

    // IMU
    imu_module();
}