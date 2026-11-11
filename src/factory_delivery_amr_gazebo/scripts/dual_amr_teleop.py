#!/usr/bin/env python3
"""Interactive Dual-AMR Keyboard Teleop for Gazebo (amr_1 & amr_2)."""

import select
import sys
import termios
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

BANNER = """
====================================================================
   ĐIỀU KHIỂN 2 XE AGV/AMR TRONG GAZEBO (amr_1 & amr_2)
====================================================================
  CHỌN XE:
    [1] : Điều khiển XE 1 (/amr_1)
    [2] : Điều khiển XE 2 (/amr_2)
    [3] : Điều khiển CẢ 2 XE cùng lúc (/amr_1 + /amr_2)

  DI CHUYỂN:
    [W] / [S]     : Tiến / Lùi (+/- 0.15 m/s, tối đa 1.20 m/s)
    [A] / [D]     : Quay Trái / Phải (+/- 0.30 rad/s, tối đa 2.00 rad/s)
    [Space] / [X] : Dừng xe (0.0 m/s, 0.0 rad/s)

  BÀN NÂNG KHAY:
    [0] hoặc [J]  : Hạ bàn nâng về 0.000 m
    [K]           : Nâng lên Kệ Dưới - Tầng 1 (0.133 m)
    [L]           : Nâng lên Kệ Trên - Tầng 2 (0.268 m)
    [U] / [M]     : Tinh chỉnh bàn nâng +0.020 m / -0.020 m

  Nhấn [Q] hoặc [Ctrl+C] để thoát.
====================================================================
"""


class DualAmrTeleop(Node):

    def __init__(self):
        super().__init__("dual_amr_teleop")
        self.cmd_pubs = {
            1: self.create_publisher(Twist, "/amr_1/cmd_vel", 10),
            2: self.create_publisher(Twist, "/amr_2/cmd_vel", 10),
        }
        self.lift_pubs = {
            1: self.create_publisher(Float64, "/amr_1/lift_command", 10),
            2: self.create_publisher(Float64, "/amr_2/lift_command", 10),
        }
        self.active_mode = 1
        self.linear_vel = {1: 0.0, 2: 0.0}
        self.angular_vel = {1: 0.0, 2: 0.0}
        self.lift_height = {1: 0.0, 2: 0.0}
        self.timer = self.create_timer(0.05, self.publish_active_commands)

    def target_ids(self):
        return [1] if self.active_mode == 1 else ([2] if self.active_mode == 2 else [1, 2])

    def publish_active_commands(self):
        for rid in (1, 2):
            twist = Twist()
            twist.linear.x = float(self.linear_vel[rid])
            twist.angular.z = float(self.angular_vel[rid])
            self.cmd_pubs[rid].publish(twist)

            lift_msg = Float64()
            lift_msg.data = float(self.lift_height[rid])
            self.lift_pubs[rid].publish(lift_msg)

    def print_status(self):
        mode_map = {1: "XE 1 (/amr_1)", 2: "XE 2 (/amr_2)", 3: "CẢ 2 XE"}
        sys.stdout.write(
            f"\r[Chọn: {mode_map[self.active_mode]:14s}] "
            f"Xe1(v={self.linear_vel[1]:+.2f}, w={self.angular_vel[1]:+.2f}, lift={self.lift_height[1]:.3f}m) | "
            f"Xe2(v={self.linear_vel[2]:+.2f}, w={self.angular_vel[2]:+.2f}, lift={self.lift_height[2]:.3f}m)   "
        )
        sys.stdout.flush()


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    try:
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
        if not rlist:
            return ""
        c1 = sys.stdin.read(1)
        if c1 == "\x1b":
            r2, _, _ = select.select([sys.stdin], [], [], 0.01)
            if r2:
                c2 = sys.stdin.read(1)
                if c2 in ("[", "O"):
                    r3, _, _ = select.select([sys.stdin], [], [], 0.01)
                    if r3:
                        c3 = sys.stdin.read(1)
                        return {"A": "UP", "B": "DOWN", "C": "RIGHT", "D": "LEFT"}.get(c3, "ESC")
            return "ESC"
        return c1
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main(args=None):
    rclpy.init(args=args)
    node = DualAmrTeleop()
    if not sys.stdin.isatty():
        node.get_logger().error("Chạy trực tiếp dual_amr_teleop.py trong terminal tương tác.")
        node.destroy_node()
        rclpy.shutdown()
        return

    settings = termios.tcgetattr(sys.stdin)
    print(BANNER)
    node.print_status()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            key = get_key(settings)
            if not key:
                continue
            if key in ("\x03", "q", "Q"):
                break
            elif key in ("1", "2", "3"):
                node.active_mode = int(key)
            elif key in ("w", "W"):
                for rid in node.target_ids():
                    node.linear_vel[rid] = min(1.20, round(node.linear_vel[rid] + 0.15, 2))
            elif key in ("s", "S"):
                for rid in node.target_ids():
                    node.linear_vel[rid] = max(-1.20, round(node.linear_vel[rid] - 0.15, 2))
            elif key in ("a", "A"):
                for rid in node.target_ids():
                    node.angular_vel[rid] = min(2.00, round(node.angular_vel[rid] + 0.30, 2))
            elif key in ("d", "D"):
                for rid in node.target_ids():
                    node.angular_vel[rid] = max(-2.00, round(node.angular_vel[rid] - 0.30, 2))
            elif key in (" ", "x", "X"):
                for rid in node.target_ids():
                    node.linear_vel[rid] = 0.0
                    node.angular_vel[rid] = 0.0
            elif key in ("0", "j", "J"):
                for rid in node.target_ids():
                    node.lift_height[rid] = 0.000
            elif key in ("k", "K"):
                for rid in node.target_ids():
                    node.lift_height[rid] = 0.133
            elif key in ("l", "L"):
                for rid in node.target_ids():
                    node.lift_height[rid] = 0.268
            elif key in ("u", "U", "UP"):
                for rid in node.target_ids():
                    node.lift_height[rid] = min(0.290, round(node.lift_height[rid] + 0.015, 3))
            elif key in ("m", "M", "DOWN"):
                for rid in node.target_ids():
                    node.lift_height[rid] = max(0.000, round(node.lift_height[rid] - 0.015, 3))
            elif key == "LEFT":
                for rid in node.target_ids():
                    node.lift_height[rid] = 0.000 if abs(node.lift_height[rid] - 0.133) < 0.005 else 0.133
            elif key == "RIGHT":
                for rid in node.target_ids():
                    node.lift_height[rid] = 0.268

            node.publish_active_commands()
            node.print_status()
    finally:
        for rid in (1, 2):
            node.linear_vel[rid] = 0.0
            node.angular_vel[rid] = 0.0
        node.publish_active_commands()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("\nĐã dừng cả 2 xe và thoát điều khiển.")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
