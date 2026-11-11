# Factory Delivery AMR — Động học, động lực học và PID robot vi sai

Phần được giao cho thành viên **Nguyễn Hoàng Anh** trong repository nhóm `Industrial_AMR_Robot`.

## Bắt đầu tại đây

1. [Báo cáo đầy đủ có đồ thị — PDF](Robot_Vi_Sai_Dong_Hoc_Dong_Luc_PID.pdf)  
   [Bản Markdown nguồn](docs/00_BAO_CAO_DAY_DU_MO_PHONG_MATLAB.md) · [Bản LaTeX chỉnh sửa được](docs/00_BAO_CAO_DAY_DU_MO_PHONG_MATLAB.tex)
2. [Tài liệu lý thuyết chi tiết](docs/01_diff_drive_kinematics_dynamics_pid.md)
3. [Diễn giải 9 biểu đồ Python](docs/03_bieu_do_mo_phong_mo_rong.md)
4. [Mã MATLAB và hướng dẫn chạy](matlab/README.md)
5. [Danh sách tích hợp ROS 2 / URDF](docs/02_repo_integration_checklist.md)

## Kết quả mô phỏng có thể tái lập

```powershell
cd research/diff_drive_pid
python -m pip install numpy matplotlib
python examples/simulate_diff_drive_pi.py
python examples/simulate_extended_plots.py
python examples/simulate_pi_pid_comparison.py
```

Kết quả được lưu ở `docs/assets/` và `examples/data/`. Ảnh `speed_response.png`, `extended_*.png`, `python_*.png` là **mô phỏng số Python**; không được gọi là kết quả MATLAB hay Gazebo.

### MATLAB

Mở MATLAB, chuyển thư mục làm việc (*Current Folder*) về thư mục chứa mã nguồn MATLAB và chạy:

```matlab
% Nếu đang ở thư mục gốc repo:
cd research/diff_drive_pid/matlab
% Hoặc mở trực tiếp thư mục research/diff_drive_pid/matlab trên thanh điều hướng MATLAB

run_all
```

MATLAB dự kiến sinh 14 ảnh tiền tố `matlab_` và 7 CSV tiền tố `matlab_`. Những ảnh này **chỉ tồn tại sau khi chạy MATLAB trên máy cá nhân**; mã `.m` chưa được chạy trong môi trường chuẩn bị tài liệu. Bộ mã MATLAB bao gồm 16 file `.m`; không kèm `.slx`.

## Phân loại thông số

- **Khách hàng:** tải hữu ích 20 kg, hành lang 1,2 m, tốc độ mục tiêu ít nhất 1,2 m/s, sai số dừng dưới 30 mm, tối thiểu 4 giờ, tránh vật cản và tự sạc.
- **Giả định kỹ thuật trong ví dụ:** tổng khối lượng 75 kg, bán kính bánh 0,08 m, khoảng cách tâm hai bánh 0,45 m, PI Kp=1.1 / Ki=3.0, PID Kd=0.02, giới hạn mô-men mô phỏng ±8 N.m.
- **Chưa được nghiệm thu:** tốc độ robot thật, mô-men motor thật, khả năng chạy Gazebo/Nav2, docking 30 mm, an toàn, độ bền pin.

## Cấu trúc

```text
research/diff_drive_pid/
├── README.md
├── Robot_Vi_Sai_Dong_Hoc_Dong_Luc_PID.pdf
├── Phu_Luc_Bieu_Do_Mo_Phong.pdf
├── docs/
│   ├── 00_BAO_CAO_DAY_DU_MO_PHONG_MATLAB.md / .tex
│   ├── 01_diff_drive_kinematics_dynamics_pid.md
│   ├── 02_repo_integration_checklist.md
│   ├── 03_bieu_do_mo_phong_mo_rong.md
│   └── assets/               # PNG Python + sơ đồ; matlab_ sau khi MATLAB chạy
├── matlab/                   # 16 file .m và README
└── examples/
    ├── simulate_diff_drive_pi.py
    ├── simulate_extended_plots.py
    ├── simulate_pi_pid_comparison.py
    ├── create_diagrams.py
    └── data/                 # CSV và JSON minh họa
```

## Trạng thái

Đã chạy lại các script Python và kiểm tra dữ liệu, đồ thị. Mã MATLAB đã rà soát tĩnh, sửa cách cập nhật đạo hàm đo trong mô phỏng PID; cần thành viên chạy và kiểm tra trên MATLAB cá nhân. **Chưa triển khai PID lên robot thật.**
