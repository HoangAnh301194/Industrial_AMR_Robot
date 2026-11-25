#!/usr/bin/env python3
"""
Autonomous Warehouse Delivery Mission Demo for Factory Delivery AMR.

Demonstrates full end-to-end autonomy using Nav2 (Simple Commander) + Lift Control:
  1. Navigate autonomously from spawn to Smart Rack 1 staging area
  2. Raise scissor lift to Level 1 (0.133 m) to simulate payload pickup
  3. Navigate through warehouse aisle to Delivery / Drop Zone
  4. Lower scissor lift (0.000 m) to unload
  5. Navigate autonomously to Charging Station 1 standby position
"""

import math
import sys
import time

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_simple_commander.robot_navigator import TaskResult
import rclpy
from std_msgs.msg import Float64


def make_pose(navigator: BasicNavigator, x: float, y: float, yaw_deg: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    yaw = math.radians(yaw_deg)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def navigate_to(navigator: BasicNavigator, label: str, pose: PoseStamped) -> bool:
    navigator.get_logger().info(f"[MISSION] Navigating to: {label} ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})")
    navigator.goToPose(pose)
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback is not None and feedback.distance_remaining is not None:
            navigator.get_logger().info(
                f"  -> Remaining distance to {label}: {feedback.distance_remaining:.2f} m",
                throttle_duration_sec=2.0,
            )
        rclpy.spin_once(navigator, timeout_sec=0.1)

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info(f"[MISSION] Reached {label}!")
        return True
    navigator.get_logger().error(f"[MISSION] Navigation to {label} finished with status: {result}")
    return False


def main():
    robot_ns = sys.argv[1] if len(sys.argv) > 1 else "amr_1"
    if not robot_ns.startswith("/"):
        robot_ns = "/" + robot_ns

    rclpy.init()
    navigator = BasicNavigator(namespace=robot_ns.strip("/"))
    lift_pub = navigator.create_publisher(Float64, f"{robot_ns}/lift_command", 10)

    init_y = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    initial_pose = make_pose(navigator, 0.0, init_y, 0.0)
    navigator.setInitialPose(initial_pose)

    navigator.get_logger().info(f"[MISSION] Waiting for Nav2 on namespace {robot_ns} ...")
    navigator.waitUntilNav2Active(localizer="amcl")

    def set_lift(height_m: float, settle_sec: float = 3.0):
        msg = Float64()
        msg.data = height_m
        for _ in range(3):
            lift_pub.publish(msg)
            time.sleep(0.1)
        navigator.get_logger().info(f"[MISSION] Lift height -> {height_m:.3f} m")
        time.sleep(settle_sec)

    waypoints = [
        ("Smart Rack 1 Staging", make_pose(navigator, 1.85, 1.15, 0.0), 0.133),
        ("Delivery Zone East", make_pose(navigator, 4.5, 0.0, 0.0), 0.000),
        ("Charging Dock 1 Approach", make_pose(navigator, -2.2, 1.2, 180.0), None),
        ("Home Standby", make_pose(navigator, 0.0, init_y, 0.0), None),
    ]

    for label, target_pose, lift_target in waypoints:
        if not navigate_to(navigator, label, target_pose):
            break
        if lift_target is not None:
            set_lift(lift_target)

    navigator.get_logger().info("[MISSION] Autonomous cycle complete!")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
