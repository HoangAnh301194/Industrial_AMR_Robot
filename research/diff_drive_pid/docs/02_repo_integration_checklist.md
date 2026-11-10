# Checklist đồng bộ với repository Industrial_AMR_Robot

- [ ] Xác nhận có quyền truy cập repository và nhận đúng nhánh `main` phiên bản mới nhất.
- [ ] Đọc `urdf/` hoặc `xacro/`: bán kính bánh chủ động **thực tế** `r` và khoảng cách tâm bánh `b`.
- [ ] Đọc mass/inertia từng link, vị trí trọng tâm và cơ cấu tải; cập nhật mô hình động lực.
- [ ] Đọc Gazebo plugin `DiffDrive`, `ros2_control` hoặc driver hiện có; xác định ai đang phát lệnh và đo tốc độ bánh.
- [ ] Đo kiểm tra dấu joint trục bánh và quy ước bên trái/phải so với công thức.
- [ ] Lấy datasheet motor, hộp số, encoder, driver và hiệu suất truyền động để tính lại mô-men.
- [ ] Cập nhật tên topic ROS 2 thật của nhóm vào bảng và vẽ TF tree thực tế.
- [ ] Lấy ảnh minh chứng `/odom`, `/joint_states`, RViz, Gazebo, rqt_plot / PlotJuggler và log chạy thử.
- [ ] Tách kết quả **Python demo** khỏi kết quả **Gazebo** và kết quả **robot thật**.
- [ ] Ghi giá trị PID cuối cùng chỉ sau khi log cho thấy tốc độ, độ ổn định và hoạt động có tải đã được kiểm tra.
- [ ] Nếu commit lên `main`: kéo thay đổi mới nhất, kiểm tra lịch sử, không force push, dùng commit có nội dung rõ ràng.
