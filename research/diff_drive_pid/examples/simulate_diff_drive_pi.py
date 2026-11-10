#!/usr/bin/env python3
"""Mô phỏng minh họa bộ điều khiển PI tốc độ 2 bánh vi sai.

Mô hình giản lược phẳng, không phải mô hình đã hiệu chuẩn của robot thật,
không chứng minh chức năng an toàn, docking hay yêu cầu ±30 mm.

Chạy: python3 examples/simulate_diff_drive_pi.py
Yêu cầu: numpy, matplotlib
"""
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

DT = 0.005             # s, giả định chu kỳ mô phỏng
R = 0.08               # m, bán kính bánh đề xuất
TRACK = 0.45           # m, khoảng cách tâm hai bánh (giả định)
MASS = 75.0            # kg, robot 55 + hàng 20
LENGTH, WIDTH = 0.70, 0.55  # m, giả định hình học
IZ = MASS * (LENGTH**2 + WIDTH**2) / 12.0  # kg m^2; xấp xỉ hình hộp đồng nhất
JW = 0.012             # kg m^2 / bánh, giả định
CRR = 0.025            # hệ số cản lăn giả định
VISCOUS = 0.015        # N m s/rad, cản nhớt tại bánh giả định
TAU_LIMIT = 8.0        # N m/bánh đầu ra, GIẢ ĐỊNH mô phỏng, không phải motor chốt
KP = 1.1               # N m/(rad/s), GIẢ ĐỊNH khởi đầu để mô phỏng
KI = 3.0               # N m/rad, GIẢ ĐỊNH khởi đầu để mô phỏng

# Mô hình động lực 2 bánh từ năng lượng động học của thân và bánh:
# T = 0.5*m*v^2 + 0.5*Iz*w^2 + 0.5*Jw*(wl^2+wr^2)
# v= r/2 (wr+wl), w=r/track (wr-wl)
common = MASS * R**2 / 4.0
rot = IZ * R**2 / TRACK**2
MATRIX = np.array([[common + rot + JW, common - rot],
                   [common - rot, common + rot + JW]])  # thứ tự [R,L]
MINV = np.linalg.inv(MATRIX)


def inv_kin(v, yaw_rate):
    """[rad/s] for [right, left], ROS x-forward, z-up."""
    return np.array([(v + 0.5 * TRACK * yaw_rate) / R,
                     (v - 0.5 * TRACK * yaw_rate) / R])


def forward_kin(wheels):
    right, left = wheels
    return 0.5 * R * (right + left), R * (right - left) / TRACK


def velocity_profile(t):
    """Ramped from rest to 1.2 m/s then down; yaw rate 0, illustration only."""
    if t < 0.5: return 0.0
    if t < 2.0: return min(1.2, 0.8 * (t - 0.5))
    if t < 4.0: return 1.2
    if t < 5.5: return max(0.0, 1.2 - 0.8 * (t - 4.0))
    return 0.0


def simulate():
    times = np.arange(0, 6.2 + DT/2, DT)
    wheels = np.zeros(2)             # rad/s [R,L]
    integ = np.zeros(2)              # integral of wheel speed error
    pose = np.zeros(3)               # x,y,theta
    hist = []
    for t in times:
        v_ref = velocity_profile(t)
        wanted = inv_kin(v_ref, 0.0)
        error = wanted - wheels
        # Anti-windup đơn giản: chỉ tích phân nếu chưa bão hòa hoặc lỗi có xu hướng giảm bão hòa.
        before = KP * error + KI * integ
        saturated = np.clip(before, -TAU_LIMIT, TAU_LIMIT)
        eligible = (np.abs(before) < TAU_LIMIT) | (np.sign(error) != np.sign(before))
        integ += DT * error * eligible
        torque = np.clip(KP * error + KI * integ, -TAU_LIMIT, TAU_LIMIT)
        # Hình thức cản lăn được làm mượt quanh tốc độ 0.
        wheel_roll = CRR * MASS * 9.81 * R / 2.0 * np.tanh(wheels / 0.2)
        ddot = MINV @ (torque - VISCOUS * wheels - wheel_roll)
        wheels += DT * ddot
        v, yaw_rate = forward_kin(wheels)
        yaw_mid = pose[2] + 0.5 * yaw_rate * DT
        pose[0] += v * np.cos(yaw_mid) * DT
        pose[1] += v * np.sin(yaw_mid) * DT
        pose[2] += yaw_rate * DT
        hist.append((t, v_ref, v, *wanted, *wheels, *torque, *pose))
    return np.array(hist)


def save_plots(data, out):
    out.mkdir(parents=True, exist_ok=True)
    t, vref, vmeas = data[:, 0], data[:, 1], data[:, 2]
    fig, ax = plt.subplots(figsize=(9, 4.1))
    ax.plot(t, vref, label='Van toc dat', linewidth=2.2)
    ax.plot(t, vmeas, label='Van toc mo phong (PI)', linewidth=1.8)
    ax.set(xlabel='Thoi gian (s)', ylabel='Van toc (m/s)', title='Minh hoa bam van toc: PI hai banh vi sai')
    ax.grid(alpha=0.25); ax.legend(); fig.tight_layout()
    fig.savefig(out / 'speed_response.png', dpi=175); plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 4.1))
    ax.plot(t, data[:,7], label='Mo-men banh phai', linewidth=1.8)
    ax.plot(t, data[:,8], label='Mo-men banh trai', linestyle='--', linewidth=1.6)
    ax.axhline(TAU_LIMIT, color='gray', linestyle=':', label='Gioi han gia dinh')
    ax.axhline(-TAU_LIMIT, color='gray', linestyle=':')
    ax.set(xlabel='Thoi gian (s)', ylabel='Mo-men banh (N.m)', title='Mo-men dau ra tu PI (mo phong minh hoa)')
    ax.grid(alpha=0.25); ax.legend(); fig.tight_layout()
    fig.savefig(out / 'wheel_torque_response.png', dpi=175); plt.close(fig)
    fig, ax = plt.subplots(figsize=(7.3, 4.2))
    ax.plot(data[:,9], data[:,10], linewidth=2)
    ax.scatter([data[0,9],data[-1,9]], [data[0,10],data[-1,10]], s=45)
    ax.set(xlabel='X (m)', ylabel='Y (m)', title='Duong di thang: so do toan hoc + dieu khien PI')
    ax.axis('equal'); ax.grid(alpha=0.25); fig.tight_layout()
    fig.savefig(out / 'illustrative_straight_trajectory.png', dpi=175); plt.close(fig)


def basic_tests():
    assert np.allclose(forward_kin(inv_kin(0.5, 0.2)), (0.5, 0.2))
    assert np.allclose(inv_kin(0.0, 0.0), [0, 0])
    data = simulate()
    # Simulation smoke tests only; NOT real acceptance tests
    mask = (data[:,0] > 3.0) & (data[:,0] < 3.8)
    assert abs(data[mask,2].mean() - 1.2) < 0.07
    assert np.max(np.abs(data[:,7:9])) <= TAU_LIMIT + 1e-9
    print('PASS: inverse/forward kinematics, numerical stability, preliminary speed tracking')
    print('speed during cruise (mean) = %.4f m/s' % data[mask,2].mean())
    print('final simulated x = %.3f m (NOT docking accuracy)' % data[-1,9])
    return data


if __name__ == '__main__':
    data = basic_tests()
    output_dir = Path(__file__).resolve().parents[1] / 'docs' / 'assets'
    save_plots(data, output_dir)
    print('saved graphs to', output_dir)
