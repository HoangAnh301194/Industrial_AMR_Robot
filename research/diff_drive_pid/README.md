# Factory Delivery AMR — tài liệu phần động học, động lực học, PID

Người tổng hợp: Nguyễn Hoàng Anh (phần việc được giao trong nhóm).

## Mục lục

- [Tài liệu kỹ thuật chính](docs/01_diff_drive_kinematics_dynamics_pid.md)
- [Ví dụ chương trình mô phỏng PI hai bánh](examples/simulate_diff_drive_pi.py)
- [Script tạo các sơ đồ minh họa](examples/create_diagrams.py)
- [Hình sơ đồ robot](docs/assets/diff_drive_geometry.png)
- [Hình sơ đồ điều khiển](docs/assets/control_architecture.png)
- [Kết quả mô phỏng số minh họa](docs/assets/speed_response.png)
- [Danh sách cần đồng bộ với repository thật](docs/02_repo_integration_checklist.md)

## Trạng thái tài liệu

Nội dung toán học hoàn thiện ở mức phục vụ báo cáo. Thông số phụ thuộc phần cứng/URDF đang dùng giá trị **giả định** từ báo cáo trước. Script Python chạy được và tạo lại biểu đồ; chưa xác nhận mô phỏng Gazebo hoặc robot thực tế. Các tham số PI trong script không phải tham số cuối.

## Chạy mô phỏng ví dụ

```bash
python3 -m pip install numpy matplotlib
python3 examples/simulate_diff_drive_pi.py
```

File được dự kiến nhập vào repository [Industrial_AMR_Robot](https://github.com/HoangAnh301194/Industrial_AMR_Robot) sau khi người có quyền ghi kiểm tra và phê duyệt.
