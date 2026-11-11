#!/usr/bin/env python3
"""
Single-Robot Keyboard Teleop for Factory Delivery AMR in Gazebo.
Run 1 terminal per robot:
  Terminal Xe 1: ros2 run factory_delivery_amr_gazebo amr_keyboard_teleop.py 1
  Terminal Xe 2: ros2 run factory_delivery_amr_gazebo amr_keyboard_teleop.py 2
"""

import select
import sys
import termios
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


def build_banner(ns: str) -> str:
    return f"""
====================================================================
   KEYBOARD TELEOP - ĐIỀU KHIỂN RIÊNG XE: {ns}
====================================================================
  1. DI CHUYỂN XE (Cụm phím Teleop I/J/K/L/, hoặc W/A/S/D):
        [W] / [I] : Tiến thẳng (+v)
        [S] / [,] : Lùi lại (-v)
        [A] / [J] : Quay trái (+w)
        [D] / [L] : Quay phải (-w)
        [U] / [O] : Tiến + Cua Trái / Phải
        [M] / [.] : Lùi + Cua Trái / Phải
    [Space] / [K] : DỪNG XE NGAY (v = 0, w = 0)
        [Q] / [Z] : Tăng / Giảm tốc độ bước nhảy

  2. BỘ NÂNG HẠ KHAY (CỤM PHÍM MŨI TÊN):
        [ ↑ ] Mũi tên LÊN   : Nâng bàn nâng lên (+0.015 m, tối đa 0.290 m)
        [ ↓ ] Mũi tên XUỐNG : Hạ bàn nâng xuống (-0.015 m, về 0.000 m)
        [ ← ] Mũi tên TRÁI  : Mốc Tầng 1 - Kệ dưới (0.133 m) / Hạ 0.000 m
        [ → ] Mũi tên PHẢI  : Mốc Tầng 2 - Kệ trên (0.268 m)

  Nhấn [Ctrl+C] để thoát.
====================================================================
"""


class AmrKeyboardTeleop(Node):

    def __init__(self, robot_ns: str):
        suffix = robot_ns.strip("/").replace("/", "_") or "amr_1"
        super().__init__(f"keyboard_teleop_{suffix}")
        self.robot_ns = "/" + robot_ns.strip("/")
        self.cmd_pub = self.create_publisher(Twist, f"{self.robot_ns}/cmd_vel", 10)
        self.lift_pub = self.create_publisher(Float64, f"{self.robot_ns}/lift_command", 10)

        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.lift_height = 0.0
        self.lin_step = 0.15
        self.ang_step = 0.30
        self.lift_step = 0.015
        self.timer = self.create_timer(0.05, self.publish_commands)

    def publish_commands(self):
        twist = Twist()
        twist.linear.x = float(self.linear_vel)
        twist.angular.z = float(self.angular_vel)
        self.cmd_pub.publish(twist)

        lift_msg = Float64()
        lift_msg.data = float(self.lift_height)
        self.lift_pub.publish(lift_msg)

    def print_status(self):
        sys.stdout.write(
            f"\r[{self.robot_ns}] "
            f"v = {self.linear_vel:+.2f} m/s | "
            f"w = {self.angular_vel:+.2f} rad/s | "
            f"Lift = {self.lift_height:.3f} m ({self.lift_height * 1000:.0f} mm)    "
        )
        sys.stdout.flush()


def read_key(settings) -> str:
    tty.setraw(sys.stdin.fileno())
    try:
        r1, _, _ = select.select([sys.stdin], [], [], 0.05)
        if not r1:
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
                        return {
                            "A": "UP",
                            "B": "DOWN",
                            "C": "RIGHT",
                            "D": "LEFT",
                        }.get(c3, "ESC")
            return "ESC"
        return c1
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def resolve_ns(argv) -> str:
    args = [a for a in argv[1:] if not a.startswith("--ros-args") and not a.startswith("-r")]
    if not args:
        return "/amr_1"
    a = args[0].strip()
    if a in ("1", "amr_1", "/amr_1", "xe1"):
        return "/amr_1"
    if a in ("2", "amr_2", "/amr_2", "xe2"):
        return "/amr_2"
    return "/" + a.lstrip("/")


def main(args=None):
    rclpy.init(args=args)
    ns = resolve_ns(sys.argv)
    node = AmrKeyboardTeleop(ns)

    if not sys.stdin.isatty():
        node.get_logger().error("Chạy amr_keyboard_teleop.py trực tiếp trong terminal.")
        node.destroy_node()
        rclpy.shutdown()
        return

    settings = termios.tcgetattr(sys.stdin)
    print(build_banner(ns))
    node.print_status()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            k = read_key(settings)
            if not k:
                continue
            if k == "\x03":
                break

            # Cụm phím mũi tên điều khiển bộ nâng
            if k == "UP":
                node.lift_height = min(0.290, round(node.lift_height + node.lift_step, 3))
            elif k == "DOWN":
                node.lift_height = max(0.000, round(node.lift_height - node.lift_step, 3))
            elif k == "LEFT":
                node.lift_height = 0.000 if abs(node.lift_height - 0.133) < 0.005 else 0.133
            elif k == "RIGHT":
                node.lift_height = 0.268

            # Cụm phím di chuyển xe
            elif k in ("w", "W", "i", "I"):
                node.linear_vel = min(1.20, round(node.linear_vel + node.lin_step, 2))
            elif k in ("s", "S", ","):
                node.linear_vel = max(-1.20, round(node.linear_vel - node.lin_step, 2))
            elif k in ("a", "A", "j", "J"):
                node.angular_vel = min(2.00, round(node.angular_vel + node.ang_step, 2))
            elif k in ("d", "D", "l", "L"):
                node.angular_vel = max(-2.00, round(node.angular_vel - node.ang_step, 2))
            elif k in ("u", "U"):
                node.linear_vel = min(1.20, round(node.linear_vel + node.lin_step, 2))
                node.angular_vel = min(2.00, round(node.angular_vel + node.ang_step, 2))
            elif k in ("o", "O"):
                node.linear_vel = min(1.20, round(node.linear_vel + node.lin_step, 2))
                node.angular_vel = max(-2.00, round(node.angular_vel - node.ang_step, 2))
            elif k in ("m", "M"):
                node.linear_vel = max(-1.20, round(node.linear_vel - node.lin_step, 2))
                node.angular_vel = max(-2.00, round(node.angular_vel - node.ang_step, 2))
            elif k == ".":
                node.linear_vel = max(-1.20, round(node.linear_vel - node.lin_step, 2))
                node.angular_vel = min(2.00, round(node.angular_vel + node.ang_step, 2))
            elif k in (" ", "k", "K", "x", "X"):
                node.linear_vel = 0.0
                node.angular_vel = 0.0
            elif k in ("q", "Q"):
                node.lin_step = min(0.40, round(node.lin_step + 0.05, 2))
                node.ang_step = min(0.80, round(node.ang_step + 0.05, 2))
            elif k in ("z", "Z"):
                node.lin_step = max(0.05, round(node.lin_step - 0.05, 2))
                node.ang_step = max(0.10, round(node.ang_step - 0.05, 2))

            node.publish_commands()
            node.print_status()
    finally:
        node.linear_vel = 0.0
        node.angular_vel = 0.0
        node.publish_commands()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print(f"\nĐã dừng xe {ns} và thoát.")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

