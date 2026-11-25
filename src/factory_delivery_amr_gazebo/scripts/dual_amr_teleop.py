#!/usr/bin/env python3
"""Interactive Dual-AMR Keyboard Teleop for Gazebo (hold-to-move, auto-stop)."""

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

DEFAULT_LIN_SPEED = 0.45
DEFAULT_ANG_SPEED = 0.90
MAX_LIN_SPEED = 1.00
MAX_ANG_SPEED = 1.80
KEY_TIMEOUT_SEC = 0.35

BANNER = """
====================================================================
   ĐIỀU KHIỂN 2 XE AGV/AMR TRONG GAZEBO (amr_1 & amr_2)
====================================================================
  CHỌN XE (chỉ gửi lệnh cho xe đang chọn, không tranh chấp xe còn lại):
    [1] : Điều khiển XE 1 (/amr_1)
    [2] : Điều khiển XE 2 (/amr_2)
    [3] : Điều khiển CẢ 2 XE cùng lúc (/amr_1 + /amr_2)

  DI CHUYỂN (không cộng dồn, nhả phím tự dừng sau 0.35s):
    [W] / [S]     : Tiến / Lùi
    [A] / [D]     : Quay Trái / Phải
    [Space] / [X] : Dừng khẩn cấp ngay (0.0 m/s, 0.0 rad/s)
    [+] / [-]     : Tăng / Giảm tốc độ di chuyển

  BÀN NÂNG KHAY:
    [0] hoặc [J]  : Hạ bàn nâng về 0.000 m
    [K]           : Nâng lên Kệ Dưới - Tầng 1 (0.133 m)
    [L]           : Nâng lên Kệ Trên - Tầng 2 (0.268 m)
    [U] / [M]     : Tinh chỉnh bàn nâng +0.015 m / -0.015 m

  Nhấn [Q] hoặc [Ctrl+C] để thoát.
====================================================================
"""


class DualAmrTeleop(Node):

    def __init__(self):
        super().__init__(f"dual_amr_teleop_{os.getpid()}")
        cmd_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.cmd_pubs = {
            1: self.create_publisher(Twist, "/amr_1/cmd_vel", cmd_qos),
            2: self.create_publisher(Twist, "/amr_2/cmd_vel", cmd_qos),
        }
        self.lift_pubs = {
            1: self.create_publisher(Float64, "/amr_1/lift_command", cmd_qos),
            2: self.create_publisher(Float64, "/amr_2/lift_command", cmd_qos),
        }
        self.active_mode = 1
        self.lin_speed = DEFAULT_LIN_SPEED
        self.ang_speed = DEFAULT_ANG_SPEED
        self.linear_vel = {1: 0.0, 2: 0.0}
        self.angular_vel = {1: 0.0, 2: 0.0}
        self.lift_height = {1: 0.0, 2: 0.0}
        self.last_move_time = {1: 0.0, 2: 0.0}
        self.last_sent_zero = {1: True, 2: True}
        self.timer = self.create_timer(0.10, self.on_timer)

    def target_ids(self):
        return [1] if self.active_mode == 1 else ([2] if self.active_mode == 2 else [1, 2])

    def stop_robots(self, rids):
        for rid in rids:
            self.linear_vel[rid] = 0.0
            self.angular_vel[rid] = 0.0
            if not self.last_sent_zero[rid]:
                self.publish_cmd_vel(rid)
                self.last_sent_zero[rid] = True

    def publish_cmd_vel(self, rid: int):
        twist = Twist()
        twist.linear.x = float(self.linear_vel[rid])
        twist.angular.z = float(self.angular_vel[rid])
        self.cmd_pubs[rid].publish(twist)

    def publish_lift(self, rid: int):
        lift_msg = Float64()
        lift_msg.data = float(self.lift_height[rid])
        self.lift_pubs[rid].publish(lift_msg)

    def on_timer(self):
        now = time.monotonic()
        changed = False
        for rid in self.target_ids():
            if (self.linear_vel[rid] != 0.0 or self.angular_vel[rid] != 0.0) and (
                now - self.last_move_time[rid] > KEY_TIMEOUT_SEC
            ):
                self.linear_vel[rid] = 0.0
                self.angular_vel[rid] = 0.0
                changed = True

            is_zero = self.linear_vel[rid] == 0.0 and self.angular_vel[rid] == 0.0
            if not is_zero or not self.last_sent_zero[rid]:
                self.publish_cmd_vel(rid)
                self.last_sent_zero[rid] = is_zero

        if changed:
            self.print_status()

    def print_status(self):
        mode_map = {1: "XE 1 (/amr_1)", 2: "XE 2 (/amr_2)", 3: "CẢ 2 XE"}
        sys.stdout.write(
            f"\r[Chọn: {mode_map[self.active_mode]:14s} | v_max={self.lin_speed:.2f}] "
            f"Xe1(v={self.linear_vel[1]:+.2f}, w={self.angular_vel[1]:+.2f}, lift={self.lift_height[1]:.3f}m) | "
            f"Xe2(v={self.linear_vel[2]:+.2f}, w={self.angular_vel[2]:+.2f}, lift={self.lift_height[2]:.3f}m)   "
        )
        sys.stdout.flush()


def read_latest_key() -> str:
    """Read key from already-raw stdin and drain queued auto-repeat bytes."""
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

    while select.select([sys.stdin], [], [], 0.0)[0]:
        nxt = sys.stdin.read(1)
        if nxt != c1:
            c1 = nxt
            break
    return c1


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
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            key = read_latest_key()
            if not key:
                continue
            if key in ("\x03", "q", "Q"):
                break

            now = time.monotonic()
            lift_updated = False
            vel_updated = False

            if key in ("1", "2", "3"):
                old_targets = node.target_ids()
                node.stop_robots(old_targets)
                node.active_mode = int(key)
            elif key in ("+", "="):
                node.lin_speed = min(MAX_LIN_SPEED, round(node.lin_speed + 0.10, 2))
                node.ang_speed = min(MAX_ANG_SPEED, round(node.ang_speed + 0.15, 2))
            elif key in ("-", "_"):
                node.lin_speed = max(0.15, round(node.lin_speed - 0.10, 2))
                node.ang_speed = max(0.30, round(node.ang_speed - 0.15, 2))
            elif key in ("w", "W"):
                for rid in node.target_ids():
                    node.linear_vel[rid] = node.lin_speed
                    node.angular_vel[rid] = 0.0
                    node.last_move_time[rid] = now
                vel_updated = True
            elif key in ("s", "S"):
                for rid in node.target_ids():
                    node.linear_vel[rid] = -node.lin_speed
                    node.angular_vel[rid] = 0.0
                    node.last_move_time[rid] = now
                vel_updated = True
            elif key in ("a", "A"):
                for rid in node.target_ids():
                    node.linear_vel[rid] = 0.0
                    node.angular_vel[rid] = node.ang_speed
                    node.last_move_time[rid] = now
                vel_updated = True
            elif key in ("d", "D"):
                for rid in node.target_ids():
                    node.linear_vel[rid] = 0.0
                    node.angular_vel[rid] = -node.ang_speed
                    node.last_move_time[rid] = now
                vel_updated = True
            elif key in (" ", "x", "X"):
                for rid in node.target_ids():
                    node.linear_vel[rid] = 0.0
                    node.angular_vel[rid] = 0.0
                vel_updated = True
            elif key in ("0", "j", "J"):
                for rid in node.target_ids():
                    node.lift_height[rid] = 0.000
                lift_updated = True
            elif key in ("k", "K"):
                for rid in node.target_ids():
                    node.lift_height[rid] = 0.133
                lift_updated = True
            elif key in ("l", "L"):
                for rid in node.target_ids():
                    node.lift_height[rid] = 0.268
                lift_updated = True
            elif key in ("u", "U", "UP"):
                for rid in node.target_ids():
                    node.lift_height[rid] = min(0.290, round(node.lift_height[rid] + 0.015, 3))
                lift_updated = True
            elif key in ("m", "M", "DOWN"):
                for rid in node.target_ids():
                    node.lift_height[rid] = max(0.000, round(node.lift_height[rid] - 0.015, 3))
                lift_updated = True
            elif key == "LEFT":
                for rid in node.target_ids():
                    node.lift_height[rid] = 0.000 if abs(node.lift_height[rid] - 0.133) < 0.005 else 0.133
                lift_updated = True
            elif key == "RIGHT":
                for rid in node.target_ids():
                    node.lift_height[rid] = 0.268
                lift_updated = True

            if vel_updated:
                for rid in node.target_ids():
                    node.publish_cmd_vel(rid)
                    node.last_sent_zero[rid] = node.linear_vel[rid] == 0.0 and node.angular_vel[rid] == 0.0
            if lift_updated:
                for rid in node.target_ids():
                    node.publish_lift(rid)

            node.print_status()
    finally:
        node.stop_robots(node.target_ids())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("\nĐã dừng xe đang chọn và thoát điều khiển.")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
