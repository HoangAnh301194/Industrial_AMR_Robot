# Kiểm tra, nguồn dữ liệu và giới hạn kết quả

Ngày cập nhật: 10/11/2026. Phạm vi: phần động học, động lực học và PID robot vi sai.

## Trạng thái kiểm tra

| Thành phần | Trạng thái | Chi tiết |
|---|---|---|
| Phương trình động học thuận/nghịch | Đã kiểm tra số Python | Thử lệnh thẳng và lệnh quay, sai số tròn số nhỏ |
| `simulate_diff_drive_pi.py` | Đã chạy | Tạo `speed_response.png`, `wheel_torque_response.png` |
| `simulate_extended_plots.py` | Đã chạy | Tạo chín hình `extended_*.png`, ba CSV và JSON |
| `simulate_pi_pid_comparison.py` | Đã chạy | Tạo hai hình `python_pi_vs_pid_*.png`, một CSV và JSON |
| 16 file `.m` MATLAB | Đã rà soát tĩnh | **Chưa chạy trên MATLAB** trong lần đóng gói này |
| `run_all.m` | Chưa chạy trên MATLAB | Dự kiến 14 hình `matlab_*.png` và 7 CSV nếu thành công |
| Gazebo / Nav2 / ROS 2 | Chưa kiểm chứng ở lần cập nhật | Cần nhóm cung cấp log, ảnh hoặc rosbag |
| Thử nghiệm AMR thật | Chưa có | Không được tuyên bố đạt tốc độ/docking/an toàn |

## Chỉ tiêu quan sát ở kịch bản Python

- Đi thẳng: tốc độ trung bình khi giữ lệnh 1,2 m/s khoảng 1,1963 m/s trong mô hình số.
- A→B→C→D: chuyển B tại 4,940 s; C tại 9,560 s; D tại 15,745 s, với tọa độ tự đặt.
- So sánh PI/PID minh họa: RMSE PI khoảng 0,211 m/s; PID khoảng 0,217 m/s. Không kết luận PID tốt hơn PI trong ví dụ này.
- Mô-men bị giới hạn ở 8 N.m/bánh **theo giả định**, không phải datasheet motor.
- Ngưỡng đổi waypoint là 0,12 m, không phải dung sai docking ±30 mm.

## Lưu ý về mã PID MATLAB

`simulate_pi_pid_comparison.m` đã được sửa để cập nhật `wheel_prev` từ tốc độ bánh **trước khi bước tích phân hoàn tất**. Nếu gán `wheel_prev` sau bước mới, đạo hàm tốc độ đo có thể luôn bằng 0, làm nhánh D không thể hiện đúng tác dụng.

## Việc cần làm khi thành viên chạy MATLAB

1. Chạy `run_all` trong MATLAB, lưu ảnh `matlab_*.png` và dữ liệu `matlab_*.csv`.
2. Chụp phiên bản MATLAB, log Command Window có dòng PASS hoặc lỗi.
3. So sánh đường bám lệnh và mô-men giữa MATLAB với hình Python, tìm nguyên nhân nếu sai khác đáng kể.
4. Nếu được xác minh, thêm nguồn ảnh MATLAB vào báo cáo bằng nhãn rõ ràng, không thay nhãn ảnh Python bằng tên MATLAB.
5. Chỉ chuyển từ mô phỏng số sang kiểm chứng sản phẩm khi đồng bộ các giá trị với URDF, motor và log ROS 2 thực tế.
