#!/usr/bin/env python3
"""
Single-Robot Keyboard Teleop for Factory Delivery AMR in Gazebo (hold-to-move).
Run 1 terminal per robot (even across different PCs):
  PC/Terminal Xe 1: ros2 run factory_delivery_amr_gazebo amr_keyboard_teleop.py 1
  PC/Terminal Xe 2: ros2 run factory_delivery_amr_gazebo amr_keyboard_teleop.py 2
"""

import os
import select
import sys
import termios
import time
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64

KEY_TIMEOUT_SEC = 0.35


def build_banner(ns: str) -> str:
    return f"""
====================================================================
   KEYBOARD TELEOP - ĐIỀU KHIỂN RIÊNG XE: {ns}
====================================================================
  1. DI CHUYỂN XE (giữ phím mới chạy, nhả phím tự dừng sau 0.35s):
        [W] / [I] : Tiến thẳng (+v)
        [S] / [,] : Lùi lại (-v)
        [A] / [J] : Quay trái (+w)
        [D] / [L] : Quay phải (-w)
        [U] / [O] : Tiến + Cua Trái / Phải
        [M] / [.] : Lùi + Cua Trái / Phải
    [Space] / [K] : DỪNG XE NGAY (v = 0, w = 0)
        [+] / [-] : Tăng / Giảm tốc độ

  2. BỘ NÂNG HẠ KHAY (CỤM PHÍM MŨI TÊN):
        [ ↑ ] Mũi tên LÊN   : Nâng bàn nâng lên (+0.015 m, tối đa 0.290 m)
        [ ↓ ] Mũi tên XUỐNG : Hạ bàn nâng xuống (-0.015 m, về 0.000 m)
        [ ← ] Mũi tên TRÁI  : Mốc Tầng 1 - Kệ dưới (0.133 m) / Hạ 0.000 m
        [ → ] Mũi tên PHẢI  : Mốc Tầng 2 - Kệ trên (0.268 m)

  Nhấn [Q] hoặc [Ctrl+C] để thoát.
====================================================================
"""


class AmrKeyboardTeleop(Node):

    def __init__(self, robot_ns: str):
        suffix = robot_ns.strip("/").replace("/", "_") or "amr_1"
        super().__init__(f"keyboard_teleop_{suffix}_{os.getpid()}")
        self.robot_ns = "/" + robot_ns.strip("/")
        cmd_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.cmd_pub = self.create_publisher(Twist, f"{self.robot_ns}/cmd_vel", cmd_qos)
        self.lift_pub = self.create_publisher(Float64, f"{self.robot_ns}/lift_command", cmd_qos)

        self.lin_speed = 0.45
        self.ang_speed = 0.90
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.lift_height = 0.0
        self.lift_step = 0.015
        self.last_move_time = 0.0
        self.last_sent_zero = True
        self.timer = self.create_timer(0.10, self.on_timer)

    def publish_cmd_vel(self):
        twist = Twist()
        twist.linear.x = float(self.linear_vel)
        twist.angular.z = float(self.angular_vel)
        self.cmd_pub.publish(twist)

    def publish_lift(self):
        lift_msg = Float64()
        lift_msg.data = float(self.lift_height)
        self.lift_pub.publish(lift_msg)

    def on_timer(self):
        now = time.monotonic()
        if (self.linear_vel != 0.0 or self.angular_vel != 0.0) and (
            now - self.last_move_time > KEY_TIMEOUT_SEC
        ):
            self.linear_vel = 0.0
            self.angular_vel = 0.0
            self.print_status()

        is_zero = self.linear_vel == 0.0 and self.angular_vel == 0.0
        if not is_zero or not self.last_sent_zero:
            self.publish_cmd_vel()
            self.last_sent_zero = is_zero

    def print_status(self):
        sys.stdout.write(
            f"\r[{self.robot_ns} | max={self.lin_speed:.2f}] "
            f"v = {self.linear_vel:+.2f} m/s | "
            f"w = {self.angular_vel:+.2f} rad/s | "
            f"Lift = {self.lift_height:.3f} m ({self.lift_height * 1000:.0f} mm)    "
        )
        sys.stdout.flush()


def read_key() -> str:
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

    while select.select([sys.stdin], [], [], 0.0)[0]:
        nxt = sys.stdin.read(1)
        if nxt != c1:
            c1 = nxt
            break
    return c1


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
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            k = read_key()
            if not k:
                continue
            if k in ("\x03", "q", "Q"):
                break

            now = time.monotonic()
            vel_updated = False
            lift_updated = False

            if k == "UP":
                node.lift_height = min(0.290, round(node.lift_height + node.lift_step, 3))
                lift_updated = True
            elif k == "DOWN":
                node.lift_height = max(0.000, round(node.lift_height - node.lift_step, 3))
                lift_updated = True
            elif k == "LEFT":
                node.lift_height = 0.000 if abs(node.lift_height - 0.133) < 0.005 else 0.133
                lift_updated = True
            elif k == "RIGHT":
                node.lift_height = 0.268
                lift_updated = True
            elif k in ("+", "="):
                node.lin_speed = min(1.00, round(node.lin_speed + 0.10, 2))
                node.ang_speed = min(1.80, round(node.ang_speed + 0.15, 2))
            elif k in ("-", "_"):
                node.lin_speed = max(0.15, round(node.lin_speed - 0.10, 2))
                node.ang_speed = max(0.30, round(node.ang_speed - 0.15, 2))
            elif k in ("w", "W", "i", "I"):
                node.linear_vel = node.lin_speed
                node.angular_vel = 0.0
                node.last_move_time = now
                vel_updated = True
            elif k in ("s", "S", ","):
                node.linear_vel = -node.lin_speed
                node.angular_vel = 0.0
                node.last_move_time = now
                vel_updated = True
            elif k in ("a", "A", "j", "J"):
                node.linear_vel = 0.0
                node.angular_vel = node.ang_speed
                node.last_move_time = now
                vel_updated = True
            elif k in ("d", "D", "l", "L"):
                node.linear_vel = 0.0
                node.angular_vel = -node.ang_speed
                node.last_move_time = now
                vel_updated = True
            elif k in ("u", "U"):
                node.linear_vel = node.lin_speed
                node.angular_vel = node.ang_speed
                node.last_move_time = now
                vel_updated = True
            elif k in ("o", "O"):
                node.linear_vel = node.lin_speed
                node.angular_vel = -node.ang_speed
                node.last_move_time = now
                vel_updated = True
            elif k in ("m", "M"):
                node.linear_vel = -node.lin_speed
                node.angular_vel = -node.ang_speed
                node.last_move_time = now
                vel_updated = True
            elif k == ".":
                node.linear_vel = -node.lin_speed
                node.angular_vel = node.ang_speed
                node.last_move_time = now
                vel_updated = True
            elif k in (" ", "k", "K", "x", "X"):
                node.linear_vel = 0.0
                node.angular_vel = 0.0
                vel_updated = True

            if vel_updated:
                node.publish_cmd_vel()
                node.last_sent_zero = node.linear_vel == 0.0 and node.angular_vel == 0.0
            if lift_updated:
                node.publish_lift()

            node.print_status()
    finally:
        node.linear_vel = 0.0
        node.angular_vel = 0.0
        node.publish_cmd_vel()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print(f"\nĐã dừng xe {node.robot_ns} và thoát.")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
